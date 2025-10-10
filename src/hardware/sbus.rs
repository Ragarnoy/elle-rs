use crate::config::SBUS_BAUD;
use crate::control::commands::{PilotCommands, RawCommands};
use embassy_rp::Peri;
use embassy_rp::dma::Channel;
use embassy_rp::interrupt::typelevel::Binding;
use embassy_rp::uart::{Async, Config, DataBits, InterruptHandler, Parity, StopBits, UartRx};
use embassy_time::{Instant, Timer};
use sbus_rs::{SbusPacket, StreamingParser};

pub struct SbusReceiver<'a> {
    uart: UartRx<'a, Async>,
    parser: StreamingParser,
}

impl<'a> SbusReceiver<'a> {
    pub fn new<T: embassy_rp::uart::Instance>(
        uart: Peri<'a, T>,
        rx_pin: Peri<'a, impl embassy_rp::uart::RxPin<T>>,
        irqs: impl Binding<T::Interrupt, InterruptHandler<T>>,
        rx_dma: Peri<'a, impl Channel>,
    ) -> Self {
        let mut config = Config::default();
        config.baudrate = SBUS_BAUD;
        config.data_bits = DataBits::DataBits8;
        config.stop_bits = StopBits::STOP2;
        config.parity = Parity::ParityEven;
        config.invert_rx = true;

        let uart = UartRx::new(uart, rx_pin, irqs, rx_dma, config);
        let parser = StreamingParser::new();

        Self { uart, parser }
    }

    pub async fn read_packet(&mut self) -> Option<SbusPacket> {
        let mut byte = [0u8; 1];

        match embassy_futures::select::select(
            self.uart.read(&mut byte),
            Timer::after(embassy_time::Duration::from_millis(15)), // Reasonable timeout for SBUS (~14ms interval)
        )
        .await
        {
            embassy_futures::select::Either::First(Ok(())) => {
                match self.parser.push_byte(byte[0]) {
                    Ok(Some(packet)) => Some(packet),
                    _ => None,
                }
            }
            _ => None,
        }
    }

    /// Read SBUS packet and return as raw commands (fast path)
    #[inline(always)]
    pub async fn read_commands(&mut self) -> Option<PilotCommands> {
        self.read_packet().await.map(|packet| {
            PilotCommands::Raw(RawCommands {
                channels: packet.channels,
                timestamp: Instant::now(),
            })
        })
    }
}
