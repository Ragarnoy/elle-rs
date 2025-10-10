use crate::config::SBUS_BAUD;
use crate::control::commands::{PilotCommands, RawCommands};
use embassy_futures::select::{Either, select};
use embassy_rp::Peri;
use embassy_rp::dma::Channel;
use embassy_rp::interrupt::typelevel::Binding;
use embassy_rp::uart::{Async, Config, DataBits, InterruptHandler, Parity, StopBits, UartRx};
use embassy_time::{Duration, Instant, Timer};
use sbus_rs::{SbusPacket, StreamingParser};

pub struct SbusReceiver<'d> {
    uart: UartRx<'d, Async>,
    parser: StreamingParser,
}

impl<'d> SbusReceiver<'d> {
    pub fn new<T: embassy_rp::uart::Instance, P: embassy_rp::uart::RxPin<T>, D: Channel>(
        uart: Peri<'d, T>,
        rx_pin: Peri<'d, P>,
        irqs: impl Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        rx_dma: Peri<'d, D>,
    ) -> Self {
        let mut config = Config::default();
        config.baudrate = SBUS_BAUD;
        config.data_bits = DataBits::DataBits8;
        config.stop_bits = StopBits::STOP2;
        config.parity = Parity::ParityEven;
        config.invert_rx = true;

        // UartRx with DMA - hardware transfers bytes in background at zero CPU cost
        let uart = UartRx::new(uart, rx_pin, irqs, rx_dma, config);
        let parser = StreamingParser::new();

        Self { uart, parser }
    }

    /// Non-blocking packet read using DMA with immediate timeout
    /// DMA transfers happen in hardware with zero CPU overhead
    pub async fn try_read_packet(&mut self) -> Option<SbusPacket> {
        let mut buffer = [0u8; 64];

        // Try DMA read with 1μs timeout - returns immediately if no data ready
        match select(
            self.uart.read(&mut buffer),
            Timer::after(Duration::from_micros(1)),
        )
        .await
        {
            Either::First(Ok(())) => {
                // DMA completed - process all bytes
                for &byte in &buffer {
                    if let Ok(Some(packet)) = self.parser.push_byte(byte) {
                        return Some(packet);
                    }
                }
                None
            }
            _ => None, // Timeout - no complete DMA transfer available
        }
    }

    /// Non-blocking read for raw commands (fast path)
    #[inline(always)]
    pub async fn try_read_commands(&mut self) -> Option<PilotCommands> {
        self.try_read_packet().await.map(|packet| {
            PilotCommands::Raw(RawCommands {
                channels: packet.channels,
                timestamp: Instant::now(),
            })
        })
    }
}
