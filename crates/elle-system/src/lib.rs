#![no_std]

pub mod system;

#[cfg(feature = "rtt-control")]
pub mod rtt_control;

// Re-export main types
pub use system::{ControlMode, CoreHealth, FlightController};
