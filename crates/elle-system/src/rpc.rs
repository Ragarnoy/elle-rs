//! postcard-RPC over RTT transport layer
//!
//! This module provides RTT-based transport for postcard-RPC communication.
//! Based on prpcrtt by James Munns: https://github.com/jamesmunns/prpcrtt

use core::cell::RefCell;
use core::fmt::Arguments;

use defmt::info;
use embassy_sync::blocking_mutex::{Mutex, raw::CriticalSectionRawMutex};
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

// Static buffers for TX double-buffering
static BUF_TX_1: StaticCell<[u8; TX_BUF_SIZE]> = StaticCell::new();
static BUF_TX_2: StaticCell<[u8; TX_BUF_SIZE]> = StaticCell::new();

// Static storage for RttTx inner state
static TX_STO: StaticCell<Mutex<CriticalSectionRawMutex, RefCell<RttTxInner>>> = StaticCell::new();

/// RTT TX transport inner state
struct RttTxInner {
    channel: UpChannel,
    buf1: &'static mut [u8; TX_BUF_SIZE],
    buf2: &'static mut [u8; TX_BUF_SIZE],
}

/// RTT TX transport for postcard-RPC
///
/// Uses double-buffering to allow encoding while sending.
#[derive(Clone)]
pub struct RttTx {
    inner: &'static Mutex<CriticalSectionRawMutex, RefCell<RttTxInner>>,
}

impl WireTx for RttTx {
    type Error = WireTxErrorKind;

    async fn send<T: Serialize + ?Sized>(
        &self,
        hdr: VarHeader,
        msg: &T,
    ) -> Result<(), Self::Error> {
        self.inner.lock(|inner| {
            let mut inner = inner.borrow_mut();
            let RttTxInner {
                channel,
                buf1,
                buf2,
            } = &mut *inner;

            // Write header to buf1
            let Some((hdr_slice, later)) = hdr.write_to_slice(&mut buf1[..]) else {
                return Ok(());
            };
            let hdr_len = hdr_slice.len();

            // Serialize message body after header
            let Ok(body) = postcard::to_slice(msg, later) else {
                return Ok(());
            };
            let body_len = body.len();
            let used = hdr_len + body_len;

            // COBS encode from buf1 into buf2
            let encoded_len = cobs::encode(&buf1[..used], &mut buf2[..]);

            // Write encoded data + sentinel
            channel.write(&buf2[..encoded_len]);
            channel.write(&[0x00]);

            Ok(())
        })
    }

    async fn send_raw(&self, buf: &[u8]) -> Result<(), Self::Error> {
        self.inner.lock(|inner| {
            let mut inner = inner.borrow_mut();
            let RttTxInner { channel, buf2, .. } = &mut *inner;

            // COBS encode raw data into buf2
            let encoded_len = cobs::encode(buf, &mut buf2[..]);
            channel.write(&buf2[..encoded_len]);
            channel.write(&[0x00]);

            Ok(())
        })
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
    let buf2 = BUF_TX_2.init([0u8; TX_BUF_SIZE]);

    let tx_inner = TX_STO.init(Mutex::new(RefCell::new(RttTxInner {
        channel: channels.up.1,
        buf1,
        buf2,
    })));

    RttChannels {
        tx: RttTx { inner: tx_inner },
        rx: RttRx::new(channels.down.0),
    }
}
