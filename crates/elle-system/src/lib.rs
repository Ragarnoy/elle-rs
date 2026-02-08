#![no_std]

pub mod system;

#[cfg(feature = "rpc-control")]
pub mod rpc;

// Re-export main types
pub use system::{ControlMode, CoreHealth, FlightController};

// Re-export supervisor signals and task
pub use system::{
    SUP_FC_READY, SUP_IMU_READY, SUP_LED_READY, SUP_START_FC, SUP_START_IMU, supervisor_task,
};

// Re-export performance monitoring types (both real and no-op versions exist in system.rs)
pub use system::{
    TimingMeasurement, log_performance_summary, update_control_loop_timing, update_led_timing,
    update_ulog_timing,
};

#[cfg(feature = "performance-monitoring")]
pub use system::PERFORMANCE_MONITOR;
