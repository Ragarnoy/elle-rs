//! ESC configuration over the AM32 bootloader (proposal 0002).
//!
//! In an ESC-config boot the engine pins carry a one-wire UART
//! (`esc_onewire_rp::OneWireUart`) instead of DShot. [`esc_config_task`] owns the
//! UARTs and runs `esc_session` against each ESC, one whole operation per
//! request, exactly like the esc-config-rs relay: no bootloader sequence ever
//! crosses the RPC boundary.
//!
//! The task runs on the DShot interrupt executor, which is otherwise idle in
//! this mode. A gap of more than ~230 µs inside a frame splits it, and a split
//! payload leaves the bootloader in payload mode; the interrupt executor
//! preempts every thread-mode task, so a long poll elsewhere cannot cause one.
//!
//! RPC handlers send an [`EscRequest`] with [`request`] and get the reply back.
//! Only one request is in flight at a time (the RPC server runs one handler at a
//! time).

use core::sync::atomic::{AtomicU32, Ordering};

use embassy_rp::peripherals::{PIO1, PIO2};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, with_timeout};
pub use esc_config_icd::{
    AckResp, Block, BlockResp, ConnectResp, EEPROM_SIZE, Eeprom, EepromResp, EscError, EscInfo,
    MAX_BLOCK, ReadReq,
};
pub use esc_onewire_rp::OneWireUart;
use esc_protocol::layout::Layout;
use esc_protocol::packet::ConnectResponse;
use esc_session::{Esc, Link, Timing};

// The settings block size the verify compare assumes is esc-config-icd's.
const _: () = assert!(EEPROM_SIZE == elle_control::boot_mode::ESC_EEPROM_SIZE);

/// Left engine UART: PIO1, TX on SM0, RX on SM1 (`PIN_14`).
pub type LeftUart = OneWireUart<'static, PIO1, 0, 1>;
/// Right engine UART (eagle): PIO2, TX on SM0, RX on SM1 (`PIN_11`).
pub type RightUart = OneWireUart<'static, PIO2, 0, 1>;

/// Which engine's ESC.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Engine {
    /// `PIN_14`; the dart's only engine.
    Left,
    /// `PIN_11`; eagle only.
    Right,
}

/// One whole ESC operation.
#[derive(Debug, Clone)]
pub enum EscOp {
    /// Bootloader handshake (replaces an earlier connection).
    Connect,
    /// Read the 192-byte settings structure.
    ReadEeprom,
    /// Write the settings structure, then read it back and compare.
    WriteEeprom(Eeprom),
    /// Read the 32-byte firmware file name.
    ReadName,
    /// Raw read at a protocol address.
    Read(ReadReq),
    /// Leave the bootloader and start the app.
    Run,
}

/// A request for [`esc_config_task`].
#[derive(Debug, Clone)]
pub struct EscRequest {
    /// Which ESC.
    pub engine: Engine,
    /// What to do.
    pub op: EscOp,
}

/// The answer to an [`EscRequest`], one variant per response type.
#[derive(Debug, Clone)]
pub enum EscReply {
    /// `Connect`.
    Connect(ConnectResp),
    /// `ReadEeprom`.
    Eeprom(EepromResp),
    /// `ReadName`, `Read`.
    Block(BlockResp),
    /// `WriteEeprom`, `Run`.
    Ack(AckResp),
}

impl EscReply {
    /// The error this reply carries, if any.
    pub fn error(&self) -> Option<EscError> {
        match self {
            Self::Connect(Err(e))
            | Self::Eeprom(Err(e))
            | Self::Block(Err(e))
            | Self::Ack(Err(e)) => Some(*e),
            _ => None,
        }
    }
}

static REQUESTS: Channel<CriticalSectionRawMutex, (u32, EscRequest), 1> = Channel::new();
static REPLY: Signal<CriticalSectionRawMutex, (u32, EscReply)> = Signal::new();
static SEQ: AtomicU32 = AtomicU32::new(0);

/// Run `req` on the ESC task and wait for its reply, at most `timeout`.
///
/// `None` on timeout (the task is still busy with an operation, or not running:
/// a normal boot). The caller maps that to `EscError::NoReply`. Each request
/// carries a sequence number, so the late reply to a request that timed out is
/// never taken for the answer to the next one.
pub async fn request(req: EscRequest, timeout: Duration) -> Option<EscReply> {
    let seq = SEQ.fetch_add(1, Ordering::Relaxed).wrapping_add(1);
    if REQUESTS.try_send((seq, req)).is_err() {
        return None;
    }
    with_timeout(timeout, async {
        loop {
            let (got, reply) = REPLY.wait().await;
            if got == seq {
                return reply;
            }
        }
    })
    .await
    .ok()
}

/// One engine: its UART and, while connected, what the bootloader reported.
struct Port<U: Link> {
    uart: U,
    connected: Option<(ConnectResponse, Layout)>,
}

impl<U: Link> Port<U> {
    fn new(uart: U) -> Self {
        Self {
            uart,
            connected: None,
        }
    }

    /// The connected ESC, over the borrowed UART.
    fn esc(&mut self) -> Result<Esc<&mut U>, EscError> {
        let (reply, layout) = self.connected.ok_or(EscError::NotConnected)?;
        Ok(Esc::resume(
            &mut self.uart,
            Timing::default(),
            reply,
            layout,
        ))
    }

    async fn run_op(&mut self, op: EscOp, engine: Engine) -> EscReply {
        match op {
            EscOp::Connect => EscReply::Connect(self.connect(engine).await),
            EscOp::ReadEeprom => EscReply::Eeprom(self.read_eeprom().await),
            EscOp::WriteEeprom(data) => {
                let result = self.write_eeprom(&data).await;
                let code = match (&result, engine) {
                    (Ok(()), _) => crate::event::EVT_ESC_WRITE_OK,
                    (Err(_), Engine::Left) => crate::event::EVT_ESC_LEFT_WRITE_FAILED,
                    (Err(_), Engine::Right) => crate::event::EVT_ESC_RIGHT_WRITE_FAILED,
                };
                if result.is_ok() {
                    crate::elle_event!(info, code, "ESC write committed and verified");
                } else {
                    crate::elle_event!(warn, code, "ESC write failed");
                }
                EscReply::Ack(result)
            }
            EscOp::ReadName => EscReply::Block(self.read_name().await),
            EscOp::Read(req) => EscReply::Block(self.read(req).await),
            EscOp::Run => EscReply::Ack(self.run().await),
        }
    }

    async fn connect(&mut self, engine: Engine) -> ConnectResp {
        self.connected = None;
        let esc = Esc::connect_with(&mut self.uart, Timing::default()).await?;
        let info = EscInfo::of(&esc);
        self.connected = Some((*esc.connect_reply(), *esc.layout()));
        let code = match engine {
            Engine::Left => crate::event::EVT_ESC_LEFT_CONNECTED,
            Engine::Right => crate::event::EVT_ESC_RIGHT_CONNECTED,
        };
        crate::elle_event!(
            info,
            code,
            "ESC bootloader connected: 0x{:04X}",
            info.device_id
        );
        Ok(info)
    }

    async fn read_eeprom(&mut self) -> EepromResp {
        let data = self.esc()?.read_eeprom().await?;
        Ok(Eeprom::from_slice(&data).unwrap_or_default())
    }

    /// Write, then read back: everything but byte 2 must match (the bootloader
    /// writes its own version there).
    async fn write_eeprom(&mut self, data: &Eeprom) -> AckResp {
        let block: [u8; EEPROM_SIZE] = data
            .as_slice()
            .try_into()
            .map_err(|_| EscError::InvalidLength(data.len() as u16))?;
        let mut esc = self.esc()?;
        esc.write_eeprom(&block).await?;
        let back = esc.read_eeprom().await?;
        match elle_control::boot_mode::esc_write_mismatch(&block, &back) {
            None => Ok(()),
            Some(offset) => Err(EscError::Verify {
                offset: offset as u32,
            }),
        }
    }

    async fn read_name(&mut self) -> BlockResp {
        let name = self.esc()?.read_file_name().await?;
        Ok(Block::from_slice(&name).unwrap_or_default())
    }

    async fn read(&mut self, req: ReadReq) -> BlockResp {
        let len = match usize::from(req.len) {
            n @ 1..=MAX_BLOCK => n,
            _ => return Err(EscError::InvalidLength(req.len)),
        };
        let mut buf = [0u8; MAX_BLOCK];
        self.esc()?.read_at(req.address, &mut buf[..len]).await?;
        Ok(Block::from_slice(&buf[..len]).unwrap_or_default())
    }

    async fn run(&mut self) -> AckResp {
        let esc = self.esc()?;
        let result = esc.run().await.map(|_| ()).map_err(EscError::from);
        self.connected = None;
        result
    }
}

/// The ESC session task (ESC-config boots only). `right` is `None` on the dart.
///
/// Spawn it on the DShot interrupt executor (see the module docs).
#[embassy_executor::task]
pub async fn esc_config_task(left: LeftUart, right: Option<RightUart>) {
    let mut left = Port::new(left);
    let mut right = right.map(Port::new);
    loop {
        let (seq, EscRequest { engine, op }) = REQUESTS.receive().await;
        let reply = match engine {
            Engine::Left => left.run_op(op, engine).await,
            Engine::Right => match right.as_mut() {
                Some(port) => port.run_op(op, engine).await,
                // No second engine on this airframe.
                None => match op {
                    EscOp::Connect => EscReply::Connect(Err(EscError::Link)),
                    EscOp::ReadEeprom => EscReply::Eeprom(Err(EscError::Link)),
                    EscOp::ReadName | EscOp::Read(_) => EscReply::Block(Err(EscError::Link)),
                    EscOp::WriteEeprom(_) | EscOp::Run => EscReply::Ack(Err(EscError::Link)),
                },
            },
        };
        REPLY.signal((seq, reply));
    }
}
