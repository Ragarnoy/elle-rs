use crsf::{Packet, Parser, ParserConfig};
use defmt::{debug, warn};
use elle_control::commands::{PilotCommands, RawCommands};
use embassy_rp::mode::Async;
use embassy_rp::uart::{Config, DataBits, Parity, StopBits, UartRx};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer};

/// CRSF baud rate (420 kbaud).
pub(crate) const CRSF_BAUD: u32 = 420_000;

/// Return a UART config suitable for CRSF (420 kbaud, 8N1).
#[must_use]
pub fn crsf_uart_config() -> Config {
    let mut config = Config::default();
    config.baudrate = CRSF_BAUD;
    config.data_bits = DataBits::DataBits8;
    config.stop_bits = StopBits::STOP1;
    config.parity = Parity::ParityNone;
    config
}

/// Shared signal for latest RC commands from receiver task to control loop.
/// The CRSF task updates this signal when new packets arrive, and the control
/// loop reads from it non-blocking via try_take().
pub static RC_COMMANDS: Signal<CriticalSectionRawMutex, PilotCommands> = Signal::new();

pub struct CrsfReceiver<'d> {
    uart: UartRx<'d, Async>,
    parser: Parser,
}

impl<'d> CrsfReceiver<'d> {
    #[must_use]
    pub const fn new(rx: UartRx<'d, Async>) -> Self {
        let parser = Parser::new(ParserConfig::default());
        Self { uart: rx, parser }
    }
}

/// CRSF 11-bit channel minimum (center is 992).
const CRSF_CHANNEL_MIN: u16 = 172;
/// CRSF 11-bit channel maximum.
const CRSF_CHANNEL_MAX: u16 = 1811;
/// Top of the 0–2047 range the RC LUTs expect.
const RC_VALUE_MAX: u32 = 2047;
const CRSF_CHANNEL_RANGE: u32 = (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN) as u32;

/// Scale CRSF channel value (172–1811) to 0–2047 range for LUT compatibility.
/// CRSF 11-bit channels: min=172, center=992, max=1811 (range=1639).
#[inline]
fn crsf_to_rc(value: u16) -> u16 {
    let v = u32::from(value.clamp(CRSF_CHANNEL_MIN, CRSF_CHANNEL_MAX));
    ((v - u32::from(CRSF_CHANNEL_MIN)) * RC_VALUE_MAX / CRSF_CHANNEL_RANGE) as u16
}

/// CRSF frame header bytes that can start a frame (sync / device addresses).
const CRSF_FRAME_STARTS: [u8; 4] = [crsf::SYNC_BYTE, crsf::SYNC_RC_BYTE, 0xEA, 0xEC];
/// Valid CRSF length byte: type + payload + CRC, at most 62 (64-byte frames).
const CRSF_LEN_RANGE: core::ops::RangeInclusive<u8> = 2..=62;

/// Receiver-task counters, for the periodic debug line and rate-limited logs.
#[derive(Default)]
struct RxStats {
    frames: u32,
    parse_errors: u32,
    uart_errors: u32,
}

/// Feed one frame's bytes to the parser and publish any RC channels it yields.
fn handle_frame(parser: &mut Parser, bytes: &[u8], stats: &mut RxStats) {
    let mut remaining = bytes;
    while !remaining.is_empty() {
        let Some((result, rest)) = parser.push_bytes(remaining) else {
            break;
        };
        remaining = rest;
        match result {
            Ok(Packet::RcChannelsPacked(channels)) => {
                stats.frames += 1;
                if stats.frames == 1 {
                    crate::elle_event!(
                        info,
                        crate::event::EVT_CRSF_RX_FIRST_FRAME,
                        "CRSF: first RC frame received (ch1={} ch2={} ch3={} ch4={})",
                        channels.0[0],
                        channels.0[1],
                        channels.0[2],
                        channels.0[3]
                    );
                } else if stats.frames.is_multiple_of(500) {
                    debug!(
                        "CRSF: {} frames ok, {} parse errors, {} uart errors",
                        stats.frames, stats.parse_errors, stats.uart_errors
                    );
                }
                let scaled = core::array::from_fn(|i| crsf_to_rc(channels.0[i]));
                RC_COMMANDS.signal(PilotCommands::Raw(RawCommands {
                    channels: scaled,
                    timestamp: Instant::now(),
                }));
            }
            Ok(_) => {}
            Err(_) => {
                stats.parse_errors += 1;
                if stats.parse_errors <= 3 || stats.parse_errors.is_multiple_of(1000) {
                    warn!("CRSF: parse error (total={})", stats.parse_errors);
                }
            }
        }
    }
}

/// Dedicated CRSF receiver task that runs independently from the control loop.
///
/// Reads exactly one frame at a time: the start byte, the length byte, then
/// exactly `len` bytes, so each RC frame is published the moment its last byte
/// arrives. (A fixed 64-byte read held a finished 26-byte frame until the
/// buffer filled: up to ~2 frame periods, 13 ms at 150 Hz, 40 ms at 50 Hz.)
/// Every frame begins by checking its start byte, so after a bad length, a
/// parse error or a UART error it hunts byte by byte for a valid frame start
/// before trusting a length again.
#[embassy_executor::task]
pub async fn crsf_receiver_task(mut receiver: CrsfReceiver<'static>) {
    let mut stats = RxStats::default();
    let mut frame = [0u8; 64];
    loop {
        let result: Result<(), embassy_rp::uart::Error> = async {
            // Hunt for a frame start (one byte at a time: cheap, and only long
            // when out of step).
            loop {
                receiver.uart.read(&mut frame[..1]).await?;
                if CRSF_FRAME_STARTS.contains(&frame[0]) {
                    break;
                }
            }
            receiver.uart.read(&mut frame[1..2]).await?;
            let len = frame[1];
            if !CRSF_LEN_RANGE.contains(&len) {
                return Ok(()); // not a frame start after all: hunt again
            }
            let end = 2 + usize::from(len);
            receiver.uart.read(&mut frame[2..end]).await?;
            handle_frame(&mut receiver.parser, &frame[..end], &mut stats);
            Ok(())
        }
        .await;

        if let Err(e) = result {
            stats.uart_errors += 1;
            if stats.uart_errors <= 3 || stats.uart_errors.is_multiple_of(1000) {
                crate::elle_event!(
                    warn,
                    crate::event::EVT_CRSF_RX_UART_ERROR,
                    "CRSF: UART error: {} (total={})",
                    e,
                    stats.uart_errors
                );
            }
            Timer::after(Duration::from_millis(1)).await;
        }
    }
}
