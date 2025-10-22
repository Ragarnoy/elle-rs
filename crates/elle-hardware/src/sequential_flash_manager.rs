use elle_config::profile::{FlashRequest, FlashResponse};
use elle_config::CalibrationLevels;
use elle_error::{ElleResult, FlashError};
use bno055::BNO055_CALIB_SIZE;
use defmt::*;
use embassy_rp::flash::{Async, Flash};
use embassy_rp::peripherals::FLASH;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer};
use sequential_storage::cache::NoCache;
use sequential_storage::map::{Key, SerializationError, Value, fetch_item, store_item};

/// Inter-core communication signals for flash operations
pub static FLASH_REQUEST_SIGNAL: Signal<CriticalSectionRawMutex, FlashRequest> = Signal::new();
pub static FLASH_RESPONSE_SIGNAL: Signal<CriticalSectionRawMutex, FlashResponse> = Signal::new();

const FLASH_RANGE_START: u32 = 0xF00000; // 15MB offset
const FLASH_RANGE_END: u32 = 0xF10000; // 64KB range for calibration storage
const DATA_BUFFER_SIZE: usize = 512; // Buffer for serialization

/// Key for calibration storage - we only store one calibration
#[derive(Clone, Copy, PartialEq, Eq)]
struct CalibrationKey;

impl Key for CalibrationKey {
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
        // Simple key: just use a single byte
        if buffer.is_empty() {
            return Err(SerializationError::BufferTooSmall);
        }
        buffer[0] = 0x42; // Calibration key identifier
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
        const REQUIRED_SIZE: usize = BNO055_CALIB_SIZE + 4 + 8; // profile + quality + timestamp

        if buffer.len() < REQUIRED_SIZE {
            return Err(SerializationError::BufferTooSmall);
        }

        let mut offset = 0;

        // Serialize profile data
        buffer[offset..offset + BNO055_CALIB_SIZE].copy_from_slice(&self.profile_data);
        offset += BNO055_CALIB_SIZE;

        // Serialize quality levels (4 bytes)
        buffer[offset] = self.quality.sys;
        buffer[offset + 1] = self.quality.gyro;
        buffer[offset + 2] = self.quality.accel;
        buffer[offset + 3] = self.quality.mag;
        offset += 4;

        // Serialize timestamp (8 bytes)
        let timestamp_bytes = self.timestamp.to_le_bytes();
        buffer[offset..offset + 8].copy_from_slice(&timestamp_bytes);
        offset += 8;

        Ok(offset)
    }

    fn deserialize_from(buffer: &[u8]) -> Result<Self, SerializationError> {
        const REQUIRED_SIZE: usize = BNO055_CALIB_SIZE + 4 + 8;

        if buffer.len() < REQUIRED_SIZE {
            return Err(SerializationError::BufferTooSmall);
        }

        let mut offset = 0;

        // Deserialize profile data
        let mut profile_data = [0u8; BNO055_CALIB_SIZE];
        profile_data.copy_from_slice(&buffer[offset..offset + BNO055_CALIB_SIZE]);
        offset += BNO055_CALIB_SIZE;

        // Deserialize quality levels
        let quality = CalibrationLevels {
            sys: buffer[offset],
            gyro: buffer[offset + 1],
            accel: buffer[offset + 2],
            mag: buffer[offset + 3],
        };
        offset += 4;

        // Deserialize timestamp
        let mut timestamp_bytes = [0u8; 8];
        timestamp_bytes.copy_from_slice(&buffer[offset..offset + 8]);
        let timestamp = u64::from_le_bytes(timestamp_bytes);

        Ok(CalibrationData {
            profile_data,
            quality,
            timestamp,
        })
    }
}

pub struct SequentialFlashManager<'a> {
    flash: Flash<'a, FLASH, Async, { elle_config::profile::FLASH_SIZE }>,
    last_save_time: Option<Instant>,
    data_buffer: [u8; DATA_BUFFER_SIZE],
}

impl<'a> SequentialFlashManager<'a> {
    pub fn new(flash: Flash<'a, FLASH, Async, { elle_config::profile::FLASH_SIZE }>) -> Self {
        Self {
            flash,
            last_save_time: None,
            data_buffer: [0; DATA_BUFFER_SIZE],
        }
    }

    /// Main flash manager task running on Core 0
    pub async fn run(&mut self) {
        info!("Core0: Sequential flash manager started and waiting for requests");

        loop {
            info!("Core0: Flash manager waiting for next request");

            // Wait for flash requests from other cores
            let request = FLASH_REQUEST_SIGNAL.wait().await;
            info!(
                "Core0: Received flash request: {:?}",
                Debug2Format(&request)
            );

            match request {
                FlashRequest::LoadCalibration => {
                    info!("Core0: Processing load calibration request");
                    let response = match self.load_calibration_internal().await {
                        Ok(Some(profile_data)) => {
                            info!("Core0: Load successful");
                            FlashResponse::LoadSuccess(profile_data)
                        }
                        Ok(None) => {
                            info!("Core0: No calibration found");
                            FlashResponse::LoadFailed
                        }
                        Err(e) => {
                            warn!("Core0: Load failed: {}", e);
                            FlashResponse::LoadFailed
                        }
                    };

                    info!("Core0: Sending load response");
                    FLASH_RESPONSE_SIGNAL.signal(response);
                }

                FlashRequest::SaveCalibration {
                    profile_data,
                    quality,
                    timestamp,
                } => {
                    info!("Core0: Processing save calibration request");
                    let response = match self
                        .save_calibration_internal(&profile_data, &quality, timestamp)
                        .await
                    {
                        Ok(_) => {
                            info!("Core0: Save successful");
                            FlashResponse::SaveSuccess
                        }
                        Err(e) => {
                            warn!("Core0: Save failed: {}", e);
                            FlashResponse::SaveFailed
                        }
                    };

                    info!("Core0: Sending save response");
                    FLASH_RESPONSE_SIGNAL.signal(response);
                }
            }

            // Small yield to ensure other tasks can run
            Timer::after(Duration::from_millis(1)).await;
        }
    }

    /// Load calibration from flash using sequential-storage
    async fn load_calibration_internal(&mut self) -> ElleResult<Option<[u8; BNO055_CALIB_SIZE]>> {
        info!("Core0: Starting sequential flash read operation");

        let flash_range = FLASH_RANGE_START..FLASH_RANGE_END;
        let key = CalibrationKey;
        let mut cache = NoCache::new();

        match fetch_item::<CalibrationKey, CalibrationData, _>(
            &mut self.flash,
            flash_range,
            &mut cache,
            &mut self.data_buffer,
            &key,
        )
        .await
        {
            Ok(Some(calibration_data)) => {
                info!("Core0: Found calibration data, checking age and quality");

                // Check age (30 days max)
                let current_time = Instant::now();
                const MILLISECONDS_PER_HOUR: u64 = 3600_u64 * 1000_u64;
                let age_hours = current_time
                    .as_ticks()
                    .saturating_sub(calibration_data.timestamp)
                    as f64
                    / MILLISECONDS_PER_HOUR as f64;

                info!("Core0: Calibration age: {}h", age_hours);
                if age_hours < 24.0 * 30.0 && calibration_data.quality.is_flight_ready() {
                    info!("Core0: Valid calibration found");
                    Ok(Some(calibration_data.profile_data))
                } else {
                    info!("Core0: Calibration too old or poor quality");
                    Ok(None)
                }
            }
            Ok(None) => {
                info!("Core0: No calibration data found");
                Ok(None)
            }
            Err(e) => {
                warn!(
                    "Core0: Sequential storage read failed: {:?}",
                    Debug2Format(&e)
                );
                Err(FlashError::ReadFailed.into())
            }
        }
    }

    /// Save calibration to flash using sequential-storage
    async fn save_calibration_internal(
        &mut self,
        profile_data: &[u8; BNO055_CALIB_SIZE],
        quality: &CalibrationLevels,
        timestamp: u64,
    ) -> ElleResult<()> {
        info!("Core0: Starting sequential save operation");

        // Rate limiting - only save once per 10 minutes
        if let Some(last_save) = self.last_save_time
            && last_save.elapsed() < Duration::from_secs(600)
        {
            info!("Core0: Rate limiting - skipping save");
            return Ok(());
        }

        // Only save flight-ready calibrations
        if !quality.is_flight_ready() {
            info!("Core0: Quality not flight-ready - skipping save");
            return Ok(());
        }

        info!("Core0: Checking if current calibration is better than stored");

        // Check if we should save this calibration
        let should_save = match self.load_calibration_internal().await {
            Ok(Some(_)) => {
                // We have existing calibration, check if new one is better
                // For now, always save if it's flight-ready (could be improved)
                true
            }
            Ok(None) => {
                // No existing calibration, save this one
                true
            }
            Err(_) => {
                // Error reading, try to save anyway
                true
            }
        };

        if !should_save {
            info!("Core0: Current calibration not better than stored, skipping save");
            return Ok(());
        }

        let calibration_data = CalibrationData {
            profile_data: *profile_data,
            quality: *quality,
            timestamp,
        };

        let flash_range = FLASH_RANGE_START..FLASH_RANGE_END;
        let key = CalibrationKey;
        let mut cache = NoCache::new();

        match store_item::<CalibrationKey, CalibrationData, _>(
            &mut self.flash,
            flash_range,
            &mut cache,
            &mut self.data_buffer,
            &key,
            &calibration_data,
        )
        .await
        {
            Ok(_) => {
                info!("Core0: Sequential storage save successful");
                self.last_save_time = Some(Instant::now());
                Ok(())
            }
            Err(e) => {
                error!(
                    "Core0: Sequential storage save failed: {:?}",
                    Debug2Format(&e)
                );
                Err(FlashError::WriteFailed.into())
            }
        }
    }
}

/// Helper functions for core 1 to request flash operations (unchanged)
pub async fn request_load_calibration() -> Option<[u8; BNO055_CALIB_SIZE]> {
    info!("Core1: Sending load calibration request");
    FLASH_REQUEST_SIGNAL.signal(FlashRequest::LoadCalibration);

    info!("Core1: Waiting for load calibration response");

    // Add timeout to prevent infinite blocking
    let timeout = Timer::after(Duration::from_secs(30)); // 30 second timeout

    match embassy_futures::select::select(FLASH_RESPONSE_SIGNAL.wait(), timeout).await {
        embassy_futures::select::Either::First(response) => match response {
            FlashResponse::LoadSuccess(data) => {
                info!("Core1: Received successful load response");
                Some(data)
            }
            FlashResponse::LoadFailed => {
                info!("Core1: Received failed load response");
                None
            }
            other => {
                warn!(
                    "Core1: Received unexpected response: {:?}",
                    Debug2Format(&other)
                );
                None
            }
        },
        embassy_futures::select::Either::Second(_) => {
            error!("Core1: Timeout waiting for load calibration response");
            None
        }
    }
}

pub async fn request_save_calibration(
    profile_data: [u8; BNO055_CALIB_SIZE],
    quality: CalibrationLevels,
    timestamp: u64,
) -> bool {
    info!("Core1: Sending save calibration request");
    FLASH_REQUEST_SIGNAL.signal(FlashRequest::SaveCalibration {
        profile_data,
        quality,
        timestamp,
    });

    info!("Core1: Waiting for save calibration response");

    // Add timeout to prevent infinite blocking
    let timeout = Timer::after(Duration::from_secs(30)); // 30 second timeout

    match embassy_futures::select::select(FLASH_RESPONSE_SIGNAL.wait(), timeout).await {
        embassy_futures::select::Either::First(response) => match response {
            FlashResponse::SaveSuccess => {
                info!("Core1: Received successful save response");
                true
            }
            FlashResponse::SaveFailed => {
                info!("Core1: Received failed save response");
                false
            }
            other => {
                warn!(
                    "Core1: Received unexpected response: {:?}",
                    Debug2Format(&other)
                );
                false
            }
        },
        embassy_futures::select::Either::Second(_) => {
            error!("Core1: Timeout waiting for save calibration response");
            false
        }
    }
}
