#![no_std]

pub mod crsf;
pub mod dshot;
pub mod event;
pub mod flash_constants;
pub mod imu;
pub mod led;
pub mod pwm;
pub mod sequential_flash_manager;
pub mod signal_cache;

pub mod ulog_logger;

pub mod crsf_telemetry;

pub use crsf::CrsfReceiver;
// Re-export commonly used types
pub use flash_constants::{ULOG_FLASH_END, ULOG_FLASH_SIZE, ULOG_FLASH_START};
pub use signal_cache::SignalCache;
pub use imu::{AttitudeData, CORE1_HEARTBEAT, Imu, ImuStatus};
pub use led::LedPattern;
pub use pwm::{PwmOutputs, PwmPins};
pub use sequential_flash_manager::SequentialFlashManager;

pub use ulog_logger::ULogLogger;
