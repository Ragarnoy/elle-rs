#![no_std]

pub mod crsf;
pub mod dshot;
pub mod event;
pub mod flash;
#[cfg(feature = "gnss")]
pub mod gnss;
pub mod imu;
pub mod led;
pub mod pwm;
pub mod sd_writer;
pub mod signal_cache;

pub mod ulog_logger;

pub use crsf::CrsfReceiver;
// Re-export commonly used types
pub use flash::SequentialFlashManager;
pub use flash::{ULOG_FLASH_END, ULOG_FLASH_SIZE, ULOG_FLASH_START};
pub use imu::{AttitudeData, CORE1_HEARTBEAT, Imu, ImuStatus};
pub use led::LedPattern;
pub use pwm::{PwmOutputs, PwmPins};
pub use signal_cache::SignalCache;

pub use ulog_logger::ULogLogger;
