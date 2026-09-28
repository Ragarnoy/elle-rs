#![no_std]

//! PX4 ULog encoder
//!
//! Encodes the PX4 ULog format (self-describing, little-endian) into a byte
//! buffer. Buffering and storage (the SD card) are handled by
//! `elle_hardware::ULogLogger`; see the crate README for the message set.
//!
//! # Format Structure
//! ```text
//! [Header (16 bytes)]
//! [Definitions Section] - Format definitions, Info, Parameters
//! [Data Section] - Subscriptions and logged data
//! ```

pub mod format;
mod messages;
mod writer;

pub use messages::{
    AttitudeMessage, AutotuneMessage, BarometerMessage, CommandsMessage, ControllerMessage,
    Core1LoadMessage, EngineMessage, EscHealthMessage, GnssMessage, GyroRawMessage,
    LogEventMessage, LoopStagesMessage, MagnetometerMessage, MessageType, PidGainsMessage,
    StatusMessage,
};
pub use writer::{ULogWriter, WriteError};
