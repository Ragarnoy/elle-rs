#![no_std]

pub mod system;

#[cfg(feature = "rtt-control")]
pub mod rtt_control;

// Re-export main types
pub use system::{ControlMode, CoreHealth, FlightController};

// Re-export supervisor signals and task
pub use system::{
    SUP_FC_READY, SUP_IMU_READY, SUP_LED_READY, SUP_START_FC, SUP_START_IMU, supervisor_task,
};

#[cfg(feature = "rtt-control")]
pub use system::{SUP_RTT_READY, SUP_START_RTT};

// Re-export performance monitoring types
#[cfg(feature = "performance-monitoring")]
pub use system::{
    TimingMeasurement, log_performance_summary, update_control_loop_timing, update_led_timing,
};
