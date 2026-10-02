//! postcard-RPC over RTT transport layer
//!
//! This module provides RTT-based transport for postcard-RPC communication.
//! Based on prpcrtt by James Munns: https://github.com/jamesmunns/prpcrtt

use core::fmt::Arguments;
use core::sync::atomic::{AtomicU32, Ordering};

use defmt::info;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use postcard_rpc::header::VarHeader;
use postcard_rpc::header::VarKeyKind;
use postcard_rpc::server::{WireRx, WireRxErrorKind, WireTx, WireTxErrorKind};
use rtt_target::{ChannelMode, DownChannel, UpChannel, rtt_init, set_defmt_channel};
use serde::Serialize;
use static_cell::StaticCell;

use crate::frame_buf::FrameBuf;

// Buffer sizes
const TX_BUF_SIZE: usize = 1024;
const RX_BUF_SIZE: usize = 1024;

/// Worst-case COBS encoding of a full `TX_BUF_SIZE` frame, plus its 0x00 delimiter.
const TX_FRAME_SIZE: usize = cobs::max_encoding_length(TX_BUF_SIZE) + 1;

// Static buffers for TX double-buffering
static BUF_TX_1: StaticCell<[u8; TX_BUF_SIZE]> = StaticCell::new();
static BUF_TX_2: StaticCell<[u8; TX_FRAME_SIZE]> = StaticCell::new();

// Static storage for RttTx inner state
static TX_STO: StaticCell<Mutex<CriticalSectionRawMutex, RttTxInner>> = StaticCell::new();

/// RTT TX transport inner state
struct RttTxInner {
    channel: UpChannel,
    /// Header + postcard body, before COBS.
    buf1: &'static mut [u8; TX_BUF_SIZE],
    /// The COBS-encoded frame and its delimiter, as written to RTT.
    buf2: &'static mut [u8; TX_FRAME_SIZE],
}

/// COBS-encode `raw` (`buf1[..len]` or caller data) into `buf2` and write the
/// frame and its delimiter to RTT in one write.
///
/// The up channel is `NoBlockSkip`: a write that does not fit is dropped whole.
/// One write per frame means a full buffer loses whole frames, never a frame's
/// delimiter (which would glue it to the next frame and lose both).
fn write_frame(
    channel: &mut UpChannel,
    buf2: &mut [u8],
    raw: &[u8],
) -> Result<(), WireTxErrorKind> {
    let len = encode_frame(raw, buf2).ok_or(WireTxErrorKind::Other)?;
    if channel.write(&buf2[..len]) == len {
        Ok(())
    } else {
        // Host not reading fast enough (or not at all): frame dropped.
        Err(WireTxErrorKind::Other)
    }
}

/// COBS-encode `raw` into `out` followed by the 0x00 delimiter, as one frame.
/// `None` if `out` is too small; never panics (unlike `cobs::encode`).
fn encode_frame(raw: &[u8], out: &mut [u8]) -> Option<usize> {
    let len = cobs::try_encode(raw, out).ok()?;
    *out.get_mut(len)? = 0;
    Some(len + 1)
}

/// RTT TX transport for postcard-RPC
///
/// Shared by the RPC server and the log publisher (both on Core 0's thread
/// executor) through an async mutex: encoding and the RTT copy run with
/// interrupts enabled, so a send never delays the DShot interrupt executor.
///
/// Errors are `WireTxErrorKind::Other`, which the postcard-rpc server treats as
/// non-fatal: a message too large for the buffers, or a frame dropped because
/// the host is not reading.
#[derive(Clone)]
pub struct RttTx {
    inner: &'static Mutex<CriticalSectionRawMutex, RttTxInner>,
}

impl WireTx for RttTx {
    type Error = WireTxErrorKind;

    async fn send<T: Serialize + ?Sized>(
        &self,
        hdr: VarHeader,
        msg: &T,
    ) -> Result<(), Self::Error> {
        let mut inner = self.inner.lock().await;
        let RttTxInner {
            channel,
            buf1,
            buf2,
        } = &mut *inner;

        // Header, then the postcard body right after it, in buf1
        let (hdr_slice, later) = hdr
            .write_to_slice(&mut buf1[..])
            .ok_or(WireTxErrorKind::Other)?;
        let hdr_len = hdr_slice.len();
        let body_len = postcard::to_slice(msg, later)
            .map_err(|_| WireTxErrorKind::Other)?
            .len();

        write_frame(channel, &mut buf2[..], &buf1[..hdr_len + body_len])
    }

    async fn send_raw(&self, buf: &[u8]) -> Result<(), Self::Error> {
        let mut inner = self.inner.lock().await;
        let RttTxInner { channel, buf2, .. } = &mut *inner;
        write_frame(channel, &mut buf2[..], buf)
    }

    async fn send_log_str(&self, _key: VarKeyKind, _s: &str) -> Result<(), Self::Error> {
        // Logging over RPC not implemented
        Ok(())
    }

    async fn send_log_fmt(&self, _key: VarKeyKind, _a: Arguments<'_>) -> Result<(), Self::Error> {
        // Logging over RPC not implemented
        Ok(())
    }
}

/// RTT RX transport for postcard-RPC
pub struct RttRx {
    channel: DownChannel,
    frames: FrameBuf<RX_BUF_SIZE>,
}

impl RttRx {
    const fn new(channel: DownChannel) -> Self {
        Self {
            channel,
            frames: FrameBuf::new(),
        }
    }
}

/// When the last well-formed frame arrived from the host, in ms since boot (0:
/// never). The RPC control loop uses it as its command timestamp, so the RC
/// failsafe also covers a dead host link (TUI closed, probe unplugged).
pub static HOST_LAST_RX_MS: AtomicU32 = AtomicU32::new(0);

impl WireRx for RttRx {
    type Error = WireRxErrorKind;

    async fn receive<'a>(&mut self, buf: &'a mut [u8]) -> Result<&'a mut [u8], Self::Error> {
        loop {
            // Serve every frame already buffered before reading more: one RTT
            // read can hold several requests, and the ones after the first must
            // not wait for (and get merged with) the next arrival.
            while let Some(frame) = self.frames.pop_frame(buf) {
                // Oversized, empty or corrupt frames are dropped; keep going.
                if let Ok(len) = frame
                    && len > 0
                    && let Ok(decoded_len) = cobs::decode_in_place(&mut buf[..len])
                {
                    HOST_LAST_RX_MS.store(
                        embassy_time::Instant::now().as_millis() as u32,
                        Ordering::Relaxed,
                    );
                    return Ok(&mut buf[..decoded_len]);
                }
            }

            // Full with no delimiter: out of sync or an over-long frame. Resync.
            if self.frames.is_stuck() {
                self.frames.clear();
            }

            let count = self.channel.read(self.frames.spare());
            if count > 0 {
                self.frames.commit(count);
                continue;
            }

            // Nothing new — yield to other tasks
            embassy_time::Timer::after(embassy_time::Duration::from_micros(100)).await;
        }
    }
}

/// Minimal WireSpawn implementation for define_dispatch! compatibility.
///
/// Since all RPC handlers are blocking (no spawn-flavored handlers),
/// this is never actually used at runtime — it just satisfies the type requirement.
#[derive(Clone)]
pub struct ElleWireSpawn;

impl postcard_rpc::server::WireSpawn for ElleWireSpawn {
    type Error = core::convert::Infallible;
    type Info = ();
    fn info(&self) -> &Self::Info {
        &()
    }
}

/// RTT channels for RPC communication
pub struct RttChannels {
    pub tx: RttTx,
    pub rx: RttRx,
}

/// Initialize RTT for postcard-RPC
///
/// Sets up:
/// - Up channel 0: defmt logs
/// - Up channel 1: RPC responses (COBS encoded)
/// - Down channel 0: RPC requests (COBS encoded)
///
/// All `NoBlockSkip`: a write that does not fit is dropped. `BlockIfFull` would
/// spin Core 0 forever once the host stops reading (TUI closed, probe left
/// attached), until the watchdog resets the board.
pub fn init_rtt_rpc() -> RttChannels {
    // Initialize RTT with channels for defmt and RPC
    let channels = rtt_init! {
        up: {
            0: {
                size: 1024,
                mode: ChannelMode::NoBlockSkip,
                name: "defmt"
            }
            1: {
                size: 4096,
                mode: ChannelMode::NoBlockSkip,
                name: "rpc_tx"
            }
        }
        down: {
            0: {
                size: 1024,
                mode: ChannelMode::NoBlockSkip,
                name: "rpc_rx"
            }
        }
    };

    // Set channel 0 for defmt logging
    set_defmt_channel(channels.up.0);

    info!("RTT RPC channels initialized");

    // Initialize TX transport with double buffering
    let buf1 = BUF_TX_1.init([0u8; TX_BUF_SIZE]);
    let buf2 = BUF_TX_2.init([0u8; TX_FRAME_SIZE]);

    let tx_inner = TX_STO.init(Mutex::new(RttTxInner {
        channel: channels.up.1,
        buf1,
        buf2,
    }));

    RttChannels {
        tx: RttTx { inner: tx_inner },
        rx: RttRx::new(channels.down.0),
    }
}
