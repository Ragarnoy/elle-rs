//! SD card ULog writer — drains ULOG_WRITE_CHANNEL to FAT32 files on SPI1 SD card.
//!
//! Uses the async `sdspi` + `embedded-fatfs` + `block-device-adapters` stack for
//! DMA-driven SPI transfers. If this stack proves unreliable, the fallback plan is
//! `embedded-sdmmc` 0.8 (blocking SPI, same channel interface, ~1ms/s overhead).
//!
//! Fallback switch in elle-hardware/Cargo.toml:
//!   Remove: sdspi, embedded-fatfs, block-device-adapters, embassy-embedded-hal, aligned
//!   Add: embedded-sdmmc = "0.8"
//!   See git tag `sd-blocking-fallback` for the blocking sd_writer.rs implementation.

use defmt::*;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDeviceWithConfig;
use embassy_rp::gpio::{Input, Output};
use embassy_rp::peripherals::SPI1;
use embassy_rp::spi::{Async, Config, Spi};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_time::Instant;
use embedded_fatfs::{Date, DateTime, FsOptions, Time, TimeProvider};
use embedded_io_async::Write;
use static_cell::StaticCell;

use block_device_adapters::BufStream;
use sdspi::SdSpi;

use core::sync::atomic::{AtomicBool, Ordering};

use crate::flash::manager::ULOG_WRITE_CHANNEL;

/// Commands sent to the SD writer task
#[derive(Clone, Copy, defmt::Format)]
pub enum SdCommand {
    /// Start a new log file
    Start,
    /// Flush and close the current file
    Stop,
}

/// Signal for controlling the SD writer from the main loop
pub static SD_CMD_SIGNAL: Signal<CriticalSectionRawMutex, SdCommand> = Signal::new();

/// SD card ready status — true when the writer task has mounted FAT32 and is draining the channel.
/// Check this before starting ULog recording to avoid writing into a dead channel.
pub static SD_READY: AtomicBool = AtomicBool::new(false);

static SPI_BUS: StaticCell<Mutex<CriticalSectionRawMutex, Spi<'static, SPI1, Async>>> =
    StaticCell::new();

/// AON-backed time provider for FAT32 file timestamps.
/// Stores boot epoch; derives wall-clock from Embassy monotonic clock.
#[derive(Debug)]
struct AonTimeProvider {
    epoch_ms: u64,
}

impl TimeProvider for AonTimeProvider {
    fn get_current_date(&self) -> Date {
        self.get_current_date_time().date
    }

    fn get_current_date_time(&self) -> DateTime {
        let total_ms = self.epoch_ms + Instant::now().as_millis();
        let total_secs = total_ms / 1000;
        let days = total_secs / 86400;
        let time_of_day = total_secs % 86400;

        let (year, month, day) = days_to_ymd(days);

        let date = Date::new(year as u16, month as u16, day as u16);
        let time = Time::new(
            (time_of_day / 3600) as u16,
            ((time_of_day % 3600) / 60) as u16,
            (time_of_day % 60) as u16,
            0,
        );
        DateTime::new(date, time)
    }
}

fn days_to_ymd(mut days: u64) -> (u32, u32, u32) {
    let mut year = 1970u32;
    loop {
        let dy: u64 = if is_leap(year) { 366 } else { 365 };
        if days < dy {
            break;
        }
        days -= dy;
        year += 1;
    }
    let leap = is_leap(year);
    let md: [u64; 12] = [
        31,
        if leap { 29 } else { 28 },
        31, 30, 31, 30, 31, 31, 30, 31, 30, 31,
    ];
    let mut month = 1u32;
    for &m in &md {
        if days < m {
            break;
        }
        days -= m;
        month += 1;
    }
    (year, month, days as u32 + 1)
}

const fn is_leap(y: u32) -> bool {
    (y % 4 == 0 && y % 100 != 0) || y % 400 == 0
}

/// Session file counter — seeded from existing files on SD card after mount.
static FILE_COUNTER: core::sync::atomic::AtomicU16 = core::sync::atomic::AtomicU16::new(0);

/// Format a log filename as LOG_NNNN.ULG using the session counter.
fn next_log_name() -> heapless::String<16> {
    let n = FILE_COUNTER.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
    let mut s = heapless::String::new();
    let _ = core::fmt::Write::write_fmt(&mut s, format_args!("LOG_{:04}.ULG", n));
    s
}

/// Drain and discard all pending messages from the ULog write channel.
fn drain_channel() {
    while ULOG_WRITE_CHANNEL.try_receive().is_ok() {}
}

/// SD card writer task — drains ULOG_WRITE_CHANNEL and writes to FAT32.
///
/// Runs on Core0 alongside the flash manager. The flash manager handles only
/// PID profiles and mag calibration; this task handles all ULog data via SD card.
#[embassy_executor::task]
pub async fn sd_writer_task(
    mut spi: Spi<'static, SPI1, Async>,
    mut cs: Output<'static>,
    mut card_detect: Input<'static>,
    epoch_ms: u64,
) {
    info!("SD: writer task starting");

    // Wait for card insertion (CARD_DETECT is active-low: low = card present)
    if card_detect.is_high() {
        info!("SD: no card detected, waiting for insertion...");
        card_detect.wait_for_low().await;
        // Small debounce after insertion
        embassy_time::Timer::after_millis(100).await;
    }
    info!("SD: card detected");

    // SD cards need 74+ clock cycles without CS asserted before init
    loop {
        match sdspi::sd_init(&mut spi, &mut cs).await {
            Ok(()) => break,
            Err(e) => {
                warn!("SD: pre-init retry: {}", defmt::Debug2Format(&e));
                embassy_time::Timer::after_millis(100).await;
            }
        }
    }
    info!("SD: pre-init clocking done");

    // Move SPI bus into shared mutex (SpiDeviceWithConfig requires this)
    let init_config = Config::default(); // 400kHz for card init
    let spi_bus = SPI_BUS.init(Mutex::new(spi));
    let spid = SpiDeviceWithConfig::new(spi_bus, cs, init_config);
    let mut sd = SdSpi::<_, _, aligned::A1>::new(spid, embassy_time::Delay);

    // Initialize the SD card protocol
    loop {
        match sd.init().await {
            Ok(()) => break,
            Err(e) => {
                warn!("SD: card init retry: {}", defmt::Debug2Format(&e));
                embassy_time::Timer::after_millis(100).await;
            }
        }
    }

    // Increase SPI clock to 25MHz after successful init
    let mut fast_config = Config::default();
    fast_config.frequency = 25_000_000;
    sd.spi().set_config(fast_config);
    info!("SD: card initialized at 25MHz");

    // Wrap in buffered block stream for embedded-fatfs
    let inner = BufStream::<_, 512>::new(sd);

    // Mount FAT filesystem with wall-clock time provider
    let tp = AonTimeProvider { epoch_ms };
    let opts = FsOptions::new().time_provider(tp);
    let fs = match embedded_fatfs::FileSystem::new(inner, opts).await {
        Ok(fs) => fs,
        Err(e) => {
            error!("SD: FAT mount failed: {}", defmt::Debug2Format(&e));
            return;
        }
    };

    // Scan existing LOG_NNNN.ULG files to seed the counter past them
    {
        let root = fs.root_dir();
        let mut max_num: i32 = -1;
        let mut iter = root.iter();
        while let Some(Ok(entry)) = iter.next().await {
            let raw_name = entry.short_file_name_as_bytes();
            // Match "LOG_NNNN.ULG" in 8.3 format: "LOG_NNNNULG"
            if raw_name.len() >= 11
                && &raw_name[..4] == b"LOG_"
                && &raw_name[8..11] == b"ULG"
            {
                if let Ok(s) = core::str::from_utf8(&raw_name[4..8]) {
                    if let Ok(n) = s.parse::<i32>() {
                        if n > max_num {
                            max_num = n;
                        }
                    }
                }
            }
        }
        let start = (max_num + 1).max(0) as u16;
        FILE_COUNTER.store(start, core::sync::atomic::Ordering::Relaxed);
        if start > 0 {
            info!("SD: found existing logs, next file: LOG_{:04}.ULG", start);
        }
    }

    SD_READY.store(true, Ordering::Release);
    info!("SD: FAT32 mounted, waiting for commands");

    // Main loop: wait for start/stop, write ULog data to files
    loop {
        // Wait for Start command, discarding any stale ULog data
        loop {
            match embassy_futures::select::select(
                SD_CMD_SIGNAL.wait(),
                ULOG_WRITE_CHANNEL.receive(),
            )
            .await
            {
                embassy_futures::select::Either::First(SdCommand::Start) => break,
                embassy_futures::select::Either::First(SdCommand::Stop) => {}
                embassy_futures::select::Either::Second(_discard) => {
                    drain_channel();
                }
            }
        }

        // Open a new log file
        let filename = next_log_name();
        let root = fs.root_dir();

        let mut file = match root.create_file(filename.as_str()).await {
            Ok(f) => f,
            Err(e) => {
                error!(
                    "SD: create {} failed: {}",
                    filename.as_str(),
                    defmt::Debug2Format(&e)
                );
                continue;
            }
        };

        info!("SD: recording to {}", filename.as_str());
        let mut bytes_written: u32 = 0;
        let mut last_flush = Instant::now();

        // Recording loop
        'recording: loop {
            let req = ULOG_WRITE_CHANNEL.receive().await;

            // Write received chunk
            let len = req.len;
            let mut write_failed = false;
            if let Err(e) = file.write_all(&req.data[..len]).await {
                warn!("SD: write error: {}", defmt::Debug2Format(&e));
                write_failed = true;
            }
            bytes_written += len as u32;

            // Batch-drain additional pending messages
            if !write_failed {
                while let Ok(extra) = ULOG_WRITE_CHANNEL.try_receive() {
                    let elen = extra.len;
                    if let Err(e) = file.write_all(&extra.data[..elen]).await {
                        warn!("SD: write error: {}", defmt::Debug2Format(&e));
                        write_failed = true;
                        break;
                    }
                    bytes_written += elen as u32;
                }
            }

            // On write failure, check if card was ejected
            if write_failed && card_detect.is_high() {
                error!("SD: card ejected during recording!");
                SD_READY.store(false, Ordering::Release);
                drain_channel();
                break 'recording;
            }

            // Periodic flush (~1s) to protect against power loss
            if last_flush.elapsed().as_secs() >= 1 {
                if let Err(e) = file.flush().await {
                    warn!("SD: flush error: {}", defmt::Debug2Format(&e));
                }
                last_flush = Instant::now();
            }

            // Check for stop command (non-blocking)
            if let Some(SdCommand::Stop) = SD_CMD_SIGNAL.try_take() {
                let _ = file.flush().await;
                info!(
                    "SD: stopped, {} bytes to {}",
                    bytes_written,
                    filename.as_str()
                );
                break 'recording;
            }
        }
    }
}
