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

/// Dedicated CRSF receiver task that runs independently from the control loop.
/// Reads UART data in chunks, parses CRSF packets, scales channel values to
/// 0–2047 range, and updates RC_COMMANDS signal with latest packets.
#[embassy_executor::task]
pub async fn crsf_receiver_task(mut receiver: CrsfReceiver<'static>) {
    let mut frame_count: u32 = 0;
    let mut error_count: u32 = 0;
    let mut parse_error_count: u32 = 0;

    loop {
        let mut buf = [0u8; 64];

        match receiver.uart.read(&mut buf).await {
            Ok(()) => {
                // Process all parsed packets from this chunk
                let mut remaining = &buf[..];
                while !remaining.is_empty() {
                    match receiver.parser.push_bytes(remaining) {
                        Some((result, rest)) => {
                            remaining = rest;
                            match result {
                                Ok(packet) => {
                                    if let Packet::RcChannelsPacked(channels) = packet {
                                        frame_count += 1;
                                        if frame_count == 1 {
                                            crate::elle_event!(
                                                info,
                                                crate::event::EVT_CRSF_RX_FIRST_FRAME,
                                                "CRSF: first RC frame received (ch1={} ch2={} ch3={} ch4={})",
                                                channels.0[0],
                                                channels.0[1],
                                                channels.0[2],
                                                channels.0[3]
                                            );
                                        } else if frame_count.is_multiple_of(500) {
                                            debug!(
                                                "CRSF: {} frames ok, {} parse errors, {} uart errors",
                                                frame_count, parse_error_count, error_count
                                            );
                                        }
                                        let scaled =
                                            core::array::from_fn(|i| crsf_to_rc(channels.0[i]));
                                        let commands = PilotCommands::Raw(RawCommands {
                                            channels: scaled,
                                            timestamp: Instant::now(),
                                        });
                                        RC_COMMANDS.signal(commands);
                                    }
                                }
                                Err(_) => {
                                    parse_error_count += 1;
                                    if parse_error_count <= 3
                                        || parse_error_count.is_multiple_of(1000)
                                    {
                                        warn!("CRSF: parse error (total={})", parse_error_count);
                                    }
                                }
                            }
                        }
                        None => break,
                    }
                }
            }
            Err(e) => {
                error_count += 1;
                if error_count <= 3 || error_count.is_multiple_of(1000) {
                    crate::elle_event!(
                        warn,
                        crate::event::EVT_CRSF_RX_UART_ERROR,
                        "CRSF: UART error: {} (total={})",
                        e,
                        error_count
                    );
                }
                Timer::after(Duration::from_millis(1)).await;
            }
        }

        // Yield briefly to ensure other tasks get CPU time
        Timer::after(Duration::from_micros(100)).await;
    }
}
