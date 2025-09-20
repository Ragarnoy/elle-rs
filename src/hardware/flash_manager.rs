use crate::config::profile::{
    CALIBRATION_FLASH_OFFSET, FLASH_SIZE, FlashRequest, FlashResponse, StoredCalibration,
};
use crate::hardware::flash_constants::{
    ASYNC_READ_SIZE, ERASE_SIZE, PAGE_SIZE, READ_SIZE, WRITE_SIZE,
};
use crate::hardware::imu::CalibrationLevels;
#[cfg(feature = "performance-monitoring")]
use crate::system::{TimingMeasurement, update_flash_timing};

#[cfg(not(feature = "performance-monitoring"))]
struct TimingMeasurement;

#[cfg(not(feature = "performance-monitoring"))]
impl TimingMeasurement {
    fn start() -> Self {
        Self
    }
    fn elapsed_us(&self) -> u32 {
        0
    }
}

#[cfg(not(feature = "performance-monitoring"))]
fn update_flash_timing(_elapsed_us: u32) {}
use bno055::BNO055_CALIB_SIZE;
use core::mem::size_of;
use defmt::*;
use embassy_futures;
use embassy_rp::flash::{Async, Flash};
use embassy_rp::peripherals::FLASH;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer};

/// Inter-core communication signals for flash operations
pub static FLASH_REQUEST_SIGNAL: Signal<CriticalSectionRawMutex, FlashRequest> = Signal::new();
pub static FLASH_RESPONSE_SIGNAL: Signal<CriticalSectionRawMutex, FlashResponse> = Signal::new();

pub struct FlashManager<'a> {
    flash: Flash<'a, FLASH, Async, { FLASH_SIZE }>,
    last_save_time: Option<Instant>,
}

impl<'a> FlashManager<'a> {
    pub fn new(flash: Flash<'a, FLASH, Async, FLASH_SIZE>) -> Self {
        Self {
            flash,
            last_save_time: None,
        }
    }

    /// Main flash manager task running on Core 0
    pub async fn run(&mut self) {
        info!("Core0: Flash manager started and waiting for requests");

        loop {
            info!("Core0: Flash manager waiting for next request");

            // Wait for flash requests from other cores
            let request = FLASH_REQUEST_SIGNAL.wait().await;
            info!(
                "Core0: Received flash request: {:?}",
                Debug2Format(&request)
            );

            let operation_timer = TimingMeasurement::start();

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

            // Update performance metrics
            update_flash_timing(operation_timer.elapsed_us());

            // Small yield to ensure other tasks can run
            Timer::after(Duration::from_millis(1)).await;
        }
    }

    /// Load calibration from flash (internal method)
    async fn load_calibration_internal(
        &mut self,
    ) -> Result<Option<[u8; BNO055_CALIB_SIZE]>, &'static str> {
        info!("Core0: Starting flash read operation");

        const STORAGE_SIZE_U32: usize = size_of::<StoredCalibration>() / 4;
        let mut buf = [0u32; STORAGE_SIZE_U32];

        // Read from flash
        info!("Core0: Creating background read future");

        // Ensure read address is aligned to READ_SIZE
        let aligned_offset =
            CALIBRATION_FLASH_OFFSET - (CALIBRATION_FLASH_OFFSET % READ_SIZE as u32);
        if aligned_offset != CALIBRATION_FLASH_OFFSET {
            warn!("Core0: Calibration offset not aligned to READ_SIZE, using aligned address");
        }

        // Ensure buffer is aligned for optimal DMA performance
        if !(buf.as_ptr() as usize).is_multiple_of(ASYNC_READ_SIZE) {
            warn!("Core0: Buffer not aligned to ASYNC_READ_SIZE for optimal DMA performance");
        }

        let read_future = match self.flash.background_read(aligned_offset, &mut buf) {
            Ok(future) => {
                info!("Core0: Background read future created successfully");
                future
            }
            Err(e) => {
                warn!("Core0: Failed to start flash read: {:?}", Debug2Format(&e));
                return Err("Flash read failed");
            }
        };

        info!("Core0: Awaiting background read completion");
        read_future.await;
        info!("Core0: Background read completed");

        let stored_cal = unsafe { &*(buf.as_ptr() as *const StoredCalibration) };

        info!("Core0: Validating stored calibration");
        if stored_cal.is_valid() {
            // Check age (30 days max)
            let current_time = Instant::now();
            let stored_time = stored_cal.get_timestamp();
            const MILLISECONDS_PER_HOUR: u64 = 3600_u64 * 1000_u64;
            let age_hours = (current_time.as_ticks().saturating_sub(stored_time)) as f64
                / MILLISECONDS_PER_HOUR as f64;

            info!("Core0: Calibration age: {}h", age_hours);
            if age_hours < 24.0 * 30.0 {
                info!("Core0: Valid calibration found");
                return Ok(Some(stored_cal.get_profile_bytes()));
            } else {
                info!("Core0: Calibration too old");
            }
        } else {
            info!("Core0: Invalid calibration data");
        }

        Ok(None)
    }

    /// Save calibration to flash (internal method)
    async fn save_calibration_internal(
        &mut self,
        profile_data: &[u8; BNO055_CALIB_SIZE],
        quality: &CalibrationLevels,
        timestamp: u64,
    ) -> Result<(), &'static str> {
        info!("Core0: Starting save operation");

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

        // Check if current calibration is better than stored (direct flash read, no recursion)
        const STORAGE_SIZE_U32: usize = size_of::<StoredCalibration>() / 4;
        let mut buf = [0u32; STORAGE_SIZE_U32];

        // Separate the read operation from using the data
        // Ensure read address is aligned to READ_SIZE
        let aligned_offset =
            CALIBRATION_FLASH_OFFSET - (CALIBRATION_FLASH_OFFSET % READ_SIZE as u32);
        if aligned_offset != CALIBRATION_FLASH_OFFSET {
            warn!("Core0: Calibration offset not aligned to READ_SIZE, using aligned address");
        }

        // Ensure buffer is aligned for optimal DMA performance
        if !(buf.as_ptr() as usize).is_multiple_of(ASYNC_READ_SIZE) {
            warn!("Core0: Buffer not aligned to ASYNC_READ_SIZE for optimal DMA performance");
        }

        let read_successful =
            if let Ok(read_future) = self.flash.background_read(aligned_offset, &mut buf) {
                info!("Core0: Reading existing calibration for comparison");
                read_future.await;
                true
            } else {
                false
            };

        if read_successful {
            let stored_cal = unsafe { &*(buf.as_ptr() as *const StoredCalibration) };
            if stored_cal.is_valid() && !stored_cal.is_better_than(quality) {
                info!("Core0: Current calibration not better than stored, skipping save");
                return Ok(());
            }
        }

        info!("Core0: Creating new calibration entry");
        // Create new calibration entry
        let stored_cal = StoredCalibration::new(profile_data, quality, timestamp);

        info!("Core0: Erasing flash sector");
        // Erase flash sector (Core 0 only operation)
        // Ensure erase address is aligned to ERASE_SIZE
        let erase_start = CALIBRATION_FLASH_OFFSET & !(ERASE_SIZE as u32 - 1);
        let erase_end = erase_start + ERASE_SIZE as u32;

        info!(
            "Core0: Erasing flash from 0x{:X} to 0x{:X}",
            erase_start, erase_end
        );
        match self.flash.blocking_erase(erase_start, erase_end) {
            Ok(_) => info!("Core0: Flash sector erased successfully"),
            Err(e) => {
                error!("Core0: Failed to erase flash: {:?}", Debug2Format(&e));
                return Err("Flash erase failed");
            }
        }

        info!("Core0: Writing calibration data to flash");
        // Write calibration data (Core 0 only operation)
        let data_slice = stored_cal.as_u32_slice();
        let byte_slice = unsafe {
            core::slice::from_raw_parts(data_slice.as_ptr() as *const u8, data_slice.len() * 4)
        };

        // Ensure write address is aligned to WRITE_SIZE
        let write_offset = CALIBRATION_FLASH_OFFSET;
        if !write_offset.is_multiple_of(WRITE_SIZE as u32) {
            error!("Core0: Calibration offset not aligned to WRITE_SIZE");
            return Err("Flash write alignment error");
        }

        // Ensure data size is a multiple of PAGE_SIZE for optimal writing
        let data_size = byte_slice.len();
        info!(
            "Core0: Writing {} bytes to flash at 0x{:X}",
            data_size, write_offset
        );

        if data_size % PAGE_SIZE != 0 {
            warn!(
                "Core0: Data size {} is not a multiple of PAGE_SIZE {}",
                data_size, PAGE_SIZE
            );
            // This is not a critical error, just a warning for optimization
        } else {
            info!(
                "Core0: Data size is properly padded to match PAGE_SIZE ({})",
                PAGE_SIZE
            );
        }

        match self.flash.blocking_write(write_offset, byte_slice) {
            Ok(_) => {
                info!("Core0: Calibration written successfully");
                self.last_save_time = Some(Instant::now());
                Ok(())
            }
            Err(e) => {
                error!("Core0: Failed to write to flash: {:?}", Debug2Format(&e));
                Err("Flash write failed")
            }
        }
    }
}

/// Helper functions for core 1 to request flash operations
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
