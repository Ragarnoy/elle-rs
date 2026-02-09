#![no_std]

pub mod flash_constants;
pub mod imu;
pub mod led;
pub mod pwm;
pub mod crsf;
pub mod sequential_flash_manager;

#[cfg(feature = "ulog-logging")]
pub mod ulog_logger;

#[cfg(feature = "crsf-telemetry")]
pub mod crsf_telemetry;

// Re-export commonly used types
pub use flash_constants::{
    CALIBRATION_FLASH_END, CALIBRATION_FLASH_SIZE, CALIBRATION_FLASH_START, ULOG_FLASH_END,
    ULOG_FLASH_SIZE, ULOG_FLASH_START,
};
pub use imu::{AttitudeData, BnoImu, CORE1_HEARTBEAT, ImuStatus};
pub use led::LedPattern;
pub use pwm::{PwmOutputs, PwmPins};
pub use crsf::CrsfReceiver;
pub use sequential_flash_manager::SequentialFlashManager;

#[cfg(feature = "ulog-logging")]
pub use ulog_logger::ULogLogger;
