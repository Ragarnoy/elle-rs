use core::sync::atomic::{AtomicU32, Ordering};
use defmt::*;
use elle_config::CalibrationLevels;
use elle_config::profile::BNO055_CALIB_SIZE;
use elle_config::profile::{FlashRequest, FlashResponse, ULOG_CHUNK_SIZE};
use elle_error::{ElleResult, FlashError};
use embassy_rp::flash::{Async, Flash};
use embassy_rp::peripherals::FLASH;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer};
use embedded_storage_async::nor_flash::NorFlash as AsyncNorFlash;
use sequential_storage::cache::NoCache;
use sequential_storage::map::{Key, MapConfig, MapStorage, SerializationError, Value};
use sequential_storage::queue::{QueueConfig, QueueStorage};

use crate::flash_constants::{CALIBRATION_FLASH_END, CALIBRATION_FLASH_START, ULOG_FLASH_START};

/// Inter-core communication signals for flash operations
pub static FLASH_REQUEST_SIGNAL: Signal<CriticalSectionRawMutex, FlashRequest> = Signal::new();
pub static FLASH_RESPONSE_SIGNAL: Signal<CriticalSectionRawMutex, FlashResponse> = Signal::new();

/// ULog storage usage counters (readable from any context for `ulog info`)
pub static ULOG_BYTES_USED: AtomicU32 = AtomicU32::new(0);
pub static ULOG_ITEMS_STORED: AtomicU32 = AtomicU32::new(0);

const DATA_BUFFER_SIZE: usize = 512; // Buffer for serialization

type FlashDevice<'a> = Flash<'a, FLASH, Async, { elle_config::profile::FLASH_SIZE }>;

/// Key for calibration storage - we only store one calibration
#[derive(Clone, Copy, PartialEq, Eq)]
struct CalibrationKey;

impl Key for CalibrationKey {
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
        if buffer.is_empty() {
            return Err(SerializationError::BufferTooSmall);
        }
        buffer[0] = 0x42;
        Ok(1)
    }

    fn deserialize_from(buffer: &[u8]) -> Result<(Self, usize), SerializationError> {
        if buffer.is_empty() || buffer[0] != 0x42 {
            return Err(SerializationError::InvalidFormat);
        }
        Ok((CalibrationKey, 1))
    }

    fn get_len(buffer: &[u8]) -> Result<usize, SerializationError> {
        if buffer.is_empty() {
            return Err(SerializationError::BufferTooSmall);
        }
        Ok(1)
    }
}

/// Value for calibration storage
#[derive(Clone, Copy)]
struct CalibrationData {
    profile_data: [u8; BNO055_CALIB_SIZE],
    quality: CalibrationLevels,
    timestamp: u64,
}

impl Value<'_> for CalibrationData {
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
        const REQUIRED_SIZE: usize = BNO055_CALIB_SIZE + 4 + 8;

        if buffer.len() < REQUIRED_SIZE {
            return Err(SerializationError::BufferTooSmall);
        }

        let mut offset = 0;

        buffer[offset..offset + BNO055_CALIB_SIZE].copy_from_slice(&self.profile_data);
        offset += BNO055_CALIB_SIZE;

        buffer[offset] = self.quality.sys;
        buffer[offset + 1] = self.quality.gyro;
        buffer[offset + 2] = self.quality.accel;
        buffer[offset + 3] = self.quality.mag;
        offset += 4;

        let timestamp_bytes = self.timestamp.to_le_bytes();
        buffer[offset..offset + 8].copy_from_slice(&timestamp_bytes);
        offset += 8;

        Ok(offset)
    }

    fn deserialize_from(buffer: &[u8]) -> Result<(CalibrationData, usize), SerializationError> {
        const REQUIRED_SIZE: usize = BNO055_CALIB_SIZE + 4 + 8;

        if buffer.len() < REQUIRED_SIZE {
            return Err(SerializationError::BufferTooSmall);
        }

        let mut offset = 0;

        let mut profile_data = [0u8; BNO055_CALIB_SIZE];
        profile_data.copy_from_slice(&buffer[offset..offset + BNO055_CALIB_SIZE]);
        offset += BNO055_CALIB_SIZE;

        let quality = CalibrationLevels {
            sys: buffer[offset],
            gyro: buffer[offset + 1],
            accel: buffer[offset + 2],
            mag: buffer[offset + 3],
        };
        offset += 4;

        let mut timestamp_bytes = [0u8; 8];
        timestamp_bytes.copy_from_slice(&buffer[offset..offset + 8]);
        let timestamp = u64::from_le_bytes(timestamp_bytes);
        offset += 8;

        Ok((
            CalibrationData {
                profile_data,
                quality,
                timestamp,
            },
            offset,
        ))
    }
}

pub struct SequentialFlashManager<'a> {
    /// Flash device wrapped in Option for take/put ownership transfer.
    /// `pop()` requires `MultiwriteNorFlash` which isn't impl'd for `&mut Flash`,
    /// so we move flash into QueueStorage and back via destroy().
    flash: Option<FlashDevice<'a>>,
    last_save_time: Option<Instant>,
    data_buffer: [u8; DATA_BUFFER_SIZE],
    ulog_buffer: [u8; ULOG_CHUNK_SIZE],
    ulog_initialized: bool,
}

impl<'a> SequentialFlashManager<'a> {
    pub fn new(flash: FlashDevice<'a>) -> Self {
        Self {
            flash: Some(flash),
            last_save_time: None,
            data_buffer: [0; DATA_BUFFER_SIZE],
            ulog_buffer: [0; ULOG_CHUNK_SIZE],
            ulog_initialized: false,
        }
    }

    /// Take the flash device out. Panics if already taken.
    fn take_flash(&mut self) -> FlashDevice<'a> {
        self.flash.take().expect("flash already taken")
    }

    /// Put the flash device back.
    fn put_flash(&mut self, flash: FlashDevice<'a>) {
        self.flash = Some(flash);
    }

    /// Main flash manager task running on Core 0
    pub async fn run(&mut self) {
        info!("Core0: Sequential flash manager started and waiting for requests");

        loop {
            let request = FLASH_REQUEST_SIGNAL.wait().await;

            match request {
                FlashRequest::LoadCalibration => {
                    let response = match self.load_calibration_internal().await {
                        Ok(Some(profile_data)) => {
                            info!("Flash: calibration loaded");
                            FlashResponse::LoadSuccess(profile_data)
                        }
                        Ok(None) => FlashResponse::LoadFailed,
                        Err(e) => {
                            warn!("Flash: load failed: {}", e);
                            FlashResponse::LoadFailed
                        }
                    };
                    FLASH_RESPONSE_SIGNAL.signal(response);
                }

                FlashRequest::SaveCalibration {
                    profile_data,
                    quality,
                    timestamp,
                } => {
                    let response = match self
                        .save_calibration_internal(&profile_data, &quality, timestamp)
                        .await
                    {
                        Ok(_) => {
                            info!("Flash: calibration saved");
                            FlashResponse::SaveSuccess
                        }
                        Err(e) => {
                            warn!("Flash: save failed: {}", e);
                            FlashResponse::SaveFailed
                        }
                    };
                    FLASH_RESPONSE_SIGNAL.signal(response);
                }

                FlashRequest::WriteULog { data, len } => {
                    let response = match self.write_ulog_internal(&data[..len]).await {
                        Ok(_) => FlashResponse::ULogWriteSuccess,
                        Err(e) => {
                            warn!("Flash: ULog write failed: {}", e);
                            FlashResponse::ULogWriteFailed
                        }
                    };
                    FLASH_RESPONSE_SIGNAL.signal(response);
                }

                FlashRequest::PeekULog => {
                    let response = self.peek_ulog_internal().await;
                    FLASH_RESPONSE_SIGNAL.signal(response);
                }

                FlashRequest::PopULog => {
                    let response = self.pop_ulog_internal().await;
                    FLASH_RESPONSE_SIGNAL.signal(response);
                }

                FlashRequest::EraseULog => {
                    info!("Flash: erasing ULog region");
                    let response = self.erase_ulog_internal().await;
                    FLASH_RESPONSE_SIGNAL.signal(response);
                }
            }

            // Small yield to ensure other tasks can run
            Timer::after(Duration::from_millis(1)).await;
        }
    }

    /// Load calibration from flash using sequential-storage map
    async fn load_calibration_internal(&mut self) -> ElleResult<Option<[u8; BNO055_CALIB_SIZE]>> {
        let key = CalibrationKey;
        let flash = self.take_flash();
        let config = MapConfig::new(CALIBRATION_FLASH_START..(CALIBRATION_FLASH_END + 1));
        let mut map = MapStorage::new(flash, config, NoCache::new());

        let result = map
            .fetch_item::<CalibrationData>(&mut self.data_buffer, &key)
            .await;

        let (flash, _cache) = map.destroy();
        self.put_flash(flash);

        match result {
            Ok(Some(calibration_data)) => {
                let current_time = Instant::now();
                const MILLISECONDS_PER_HOUR: u64 = 3600_u64 * 1000_u64;
                let age_hours = current_time
                    .as_ticks()
                    .saturating_sub(calibration_data.timestamp)
                    as f64
                    / MILLISECONDS_PER_HOUR as f64;

                if age_hours < 24.0 * 30.0 && calibration_data.quality.is_flight_ready() {
                    Ok(Some(calibration_data.profile_data))
                } else {
                    Ok(None)
                }
            }
            Ok(None) => Ok(None),
            Err(e) => {
                warn!("Flash: read failed: {:?}", Debug2Format(&e));
                Err(FlashError::ReadFailed.into())
            }
        }
    }

    /// Save calibration to flash using sequential-storage map
    async fn save_calibration_internal(
        &mut self,
        profile_data: &[u8; BNO055_CALIB_SIZE],
        quality: &CalibrationLevels,
        timestamp: u64,
    ) -> ElleResult<()> {
        // Rate limiting - only save once per 10 minutes
        if let Some(last_save) = self.last_save_time
            && last_save.elapsed() < Duration::from_secs(600)
        {
            return Ok(());
        }

        if !quality.is_flight_ready() {
            return Ok(());
        }

        let calibration_data = CalibrationData {
            profile_data: *profile_data,
            quality: *quality,
            timestamp,
        };

        let key = CalibrationKey;
        let flash = self.take_flash();
        let config = MapConfig::new(CALIBRATION_FLASH_START..(CALIBRATION_FLASH_END + 1));
        let mut map = MapStorage::new(flash, config, NoCache::new());

        let result = map
            .store_item(&mut self.data_buffer, &key, &calibration_data)
            .await;

        let (flash, _cache) = map.destroy();
        self.put_flash(flash);

        match result {
            Ok(_) => {
                self.last_save_time = Some(Instant::now());
                Ok(())
            }
            Err(e) => {
                error!("Flash: save failed: {:?}", Debug2Format(&e));
                Err(FlashError::WriteFailed.into())
            }
        }
    }

    /// Write ULog data to flash using sequential-storage queue
    async fn write_ulog_internal(&mut self, data: &[u8]) -> ElleResult<()> {
        if data.is_empty() {
            return Ok(());
        }

        if !self.ulog_initialized {
            self.ulog_initialized = true;
        }

        let len = data.len().min(ULOG_CHUNK_SIZE);
        self.ulog_buffer[..len].copy_from_slice(&data[..len]);

        let flash = self.take_flash();
        let config = QueueConfig::new(ULOG_FLASH_START..0x1000000);
        let mut queue = QueueStorage::new(flash, config, NoCache::new());

        let result = queue.push(&self.ulog_buffer[..len], false).await;

        let (flash, _cache) = queue.destroy();
        self.put_flash(flash);

        match result {
            Ok(_) => {
                ULOG_BYTES_USED.fetch_add(len as u32, Ordering::Relaxed);
                ULOG_ITEMS_STORED.fetch_add(1, Ordering::Relaxed);
                Ok(())
            }
            Err(e) => {
                crate::elle_event!(
                    error,
                    crate::event::EVT_FLASH_ULOG_PUSH_FAILED,
                    "Flash: ULog push failed: {:?}",
                    Debug2Format(&e)
                );
                Err(FlashError::WriteFailed.into())
            }
        }
    }

    /// Peek at the oldest ULog entry without removing it
    async fn peek_ulog_internal(&mut self) -> FlashResponse {
        let flash = self.take_flash();
        let config = QueueConfig::new(ULOG_FLASH_START..0x1000000);
        let mut queue = QueueStorage::new(flash, config, NoCache::new());

        let result = queue.peek(&mut self.ulog_buffer).await;

        // Copy data before destroying queue (result borrows ulog_buffer)
        let response = match result {
            Ok(Some(data)) => {
                let len = data.len();
                let mut resp_data = [0u8; ULOG_CHUNK_SIZE];
                resp_data[..len].copy_from_slice(data);
                FlashResponse::ULogData {
                    data: resp_data,
                    len,
                }
            }
            Ok(None) => FlashResponse::ULogEmpty,
            Err(e) => {
                warn!("Flash: ULog peek failed: {:?}", Debug2Format(&e));
                FlashResponse::ULogEmpty
            }
        };

        let (flash, _cache) = queue.destroy();
        self.put_flash(flash);

        response
    }

    /// Pop the oldest ULog entry from the queue
    async fn pop_ulog_internal(&mut self) -> FlashResponse {
        let flash = self.take_flash();
        let config = QueueConfig::new(ULOG_FLASH_START..0x1000000);
        let mut queue = QueueStorage::new(flash, config, NoCache::new());

        let result = queue.pop(&mut self.ulog_buffer).await;

        let response = match result {
            Ok(Some(data)) => {
                ULOG_BYTES_USED.fetch_sub(data.len() as u32, Ordering::Relaxed);
                ULOG_ITEMS_STORED.fetch_sub(1, Ordering::Relaxed);
                FlashResponse::ULogPopSuccess
            }
            Ok(None) => FlashResponse::ULogEmpty,
            Err(e) => {
                warn!("Flash: ULog pop failed: {:?}", Debug2Format(&e));
                FlashResponse::ULogEmpty
            }
        };

        let (flash, _cache) = queue.destroy();
        self.put_flash(flash);

        response
    }

    /// Erase the entire ULog flash region
    async fn erase_ulog_internal(&mut self) -> FlashResponse {
        let flash = self.flash.as_mut().expect("flash not available");
        let mut addr = ULOG_FLASH_START;
        let end = 0x1000000u32;
        const ERASE_CHUNK: u32 = 64 * 1024; // 64KB per iteration

        while addr < end {
            let chunk_end = (addr + ERASE_CHUNK).min(end);
            match flash.erase(addr, chunk_end).await {
                Ok(_) => {
                    addr = chunk_end;
                }
                Err(e) => {
                    crate::elle_event!(
                        error,
                        crate::event::EVT_FLASH_ULOG_ERASE_FAILED,
                        "Flash: ULog erase failed at 0x{:X}: {:?}",
                        addr,
                        Debug2Format(&e)
                    );
                    return FlashResponse::ULogEraseFailed;
                }
            }
            Timer::after(Duration::from_millis(1)).await;
        }

        self.ulog_initialized = false;
        ULOG_BYTES_USED.store(0, Ordering::Relaxed);
        ULOG_ITEMS_STORED.store(0, Ordering::Relaxed);
        FlashResponse::ULogEraseSuccess
    }
}

/// Helper functions for core 1 to request flash operations
pub async fn request_load_calibration() -> Option<[u8; BNO055_CALIB_SIZE]> {
    FLASH_REQUEST_SIGNAL.signal(FlashRequest::LoadCalibration);

    let timeout = Timer::after(Duration::from_secs(30));

    match embassy_futures::select::select(FLASH_RESPONSE_SIGNAL.wait(), timeout).await {
        embassy_futures::select::Either::First(response) => match response {
            FlashResponse::LoadSuccess(data) => Some(data),
            FlashResponse::LoadFailed => None,
            _ => None,
        },
        embassy_futures::select::Either::Second(_) => {
            error!("Flash: timeout waiting for calibration load");
            None
        }
    }
}

pub async fn request_save_calibration(
    profile_data: [u8; BNO055_CALIB_SIZE],
    quality: CalibrationLevels,
    timestamp: u64,
) -> bool {
    FLASH_REQUEST_SIGNAL.signal(FlashRequest::SaveCalibration {
        profile_data,
        quality,
        timestamp,
    });

    let timeout = Timer::after(Duration::from_secs(30));

    match embassy_futures::select::select(FLASH_RESPONSE_SIGNAL.wait(), timeout).await {
        embassy_futures::select::Either::First(response) => match response {
            FlashResponse::SaveSuccess => true,
            FlashResponse::SaveFailed => false,
            _ => false,
        },
        embassy_futures::select::Either::Second(_) => {
            error!("Flash: timeout waiting for calibration save");
            false
        }
    }
}

/// Request ULog write from any core
pub async fn request_write_ulog(data: &[u8]) -> bool {
    if data.is_empty() || data.len() > ULOG_CHUNK_SIZE {
        warn!("Flash: invalid ULog data size: {}", data.len());
        return false;
    }

    let mut buffer = [0u8; ULOG_CHUNK_SIZE];
    buffer[..data.len()].copy_from_slice(data);

    FLASH_REQUEST_SIGNAL.signal(FlashRequest::WriteULog {
        data: buffer,
        len: data.len(),
    });

    let timeout = Timer::after(Duration::from_secs(10));

    match embassy_futures::select::select(FLASH_RESPONSE_SIGNAL.wait(), timeout).await {
        embassy_futures::select::Either::First(response) => match response {
            FlashResponse::ULogWriteSuccess => true,
            FlashResponse::ULogWriteFailed => false,
            _ => false,
        },
        embassy_futures::select::Either::Second(_) => {
            crate::elle_event!(
                error,
                crate::event::EVT_FLASH_ULOG_WRITE_TIMEOUT,
                "Flash: timeout waiting for ULog write"
            );
            false
        }
    }
}
