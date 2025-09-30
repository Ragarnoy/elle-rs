#![no_std]

#[cfg(all(feature = "rtt-control", feature = "defmt-logging"))]
compile_error!("features `rtt-control` and `defmt-logging` are mutually exclusive");

pub mod config;
pub mod control;
pub mod error;
pub mod hardware;
pub mod rtt_control;
pub mod system;
