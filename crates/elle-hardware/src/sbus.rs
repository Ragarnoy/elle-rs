use elle_config::SBUS_BAUD;
use elle_control::commands::{PilotCommands, RawCommands};
use embassy_futures::select::{Either, select};
use embassy_rp::Peri;
use embassy_rp::dma::Channel;
use embassy_rp::interrupt::typelevel::Binding;
use embassy_rp::uart::{Async, Config, DataBits, InterruptHandler, Parity, StopBits, UartRx};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer};
use sbus_rs::{SbusPacket, StreamingParser};

/// Shared signal for latest SBUS commands from receiver task to control loop
/// The SBUS task updates this signal when new packets arrive, and the control
/// loop reads from it non-blocking via try_take()
pub static SBUS_COMMANDS: Signal<CriticalSectionRawMutex, PilotCommands> = Signal::new();

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

        // UartRx with DMA - hardware transfers bytes in background with minimal CPU overhead
        let uart = UartRx::new(uart, rx_pin, irqs, rx_dma, config);
        let parser = StreamingParser::new();

        Self { uart, parser }
    }

    /// Non-blocking packet read using DMA with reasonable timeout
    /// Reads bytes in chunks with timeout, feeding them to the parser
    /// SBUS packets arrive every 7-14ms and take 2.5ms to transmit
    /// 15ms timeout ensures reliable packet capture with margin for timing variability
    pub async fn try_read_packet(&mut self) -> Option<SbusPacket> {
        // Read up to 8 bytes at a time with 15ms total budget
        // This allows for partial reads while keeping the buffer small
        const CHUNK_SIZE: usize = 8;
        let deadline = Instant::now() + Duration::from_millis(15);

        loop {
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.as_millis() == 0 {
                return None; // Timeout
            }

            let mut chunk = [0u8; CHUNK_SIZE];

            // Try to read a chunk with remaining time budget
            match select(self.uart.read(&mut chunk), Timer::at(deadline)).await {
                Either::First(Ok(())) => {
                    // Successfully read CHUNK_SIZE bytes, process them
                    for &byte in &chunk {
                        if let Ok(Some(packet)) = self.parser.push_byte(byte) {
                            return Some(packet);
                        }
                    }
                    // Continue reading more chunks
                }
                _ => return None, // Timeout or error
            }
        }
    }

    /// Blocking read for a complete SBUS packet
    /// This method can block for up to 13-15ms waiting for a full packet
    /// Should only be called from a dedicated SBUS task
    pub async fn read_packet(&mut self) -> Option<SbusPacket> {
        self.try_read_packet().await
    }
}

/// Dedicated SBUS receiver task that runs independently from the control loop
/// This task blocks on UART reads (up to 15ms per attempt) to reliably catch packets.
/// The control loop runs at 13ms and yields, so tasks interleave without starvation.
/// Updates SBUS_COMMANDS signal with latest packets.
///
/// # Usage
/// Spawn this task with a static SbusReceiver:
/// ```ignore
/// let sbus = SbusReceiver::new(uart, pin, irqs, dma);
/// spawner.spawn(sbus_receiver_task(sbus)).unwrap();
/// ```
#[embassy_executor::task]
pub async fn sbus_receiver_task(mut receiver: SbusReceiver<'static>) {
    loop {
        if let Some(packet) = receiver.read_packet().await {
            // Convert packet to PilotCommands and update shared signal
            let commands = PilotCommands::Raw(RawCommands {
                channels: packet.channels,
                timestamp: Instant::now(),
            });
            SBUS_COMMANDS.signal(commands);
        }

        // Yield briefly to ensure other tasks get CPU time
        // This prevents SBUS task from starving the control loop
        Timer::after(Duration::from_micros(100)).await;
    }
}
