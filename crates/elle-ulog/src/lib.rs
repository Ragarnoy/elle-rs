#![no_std]

//! ULog flash logging implementation
//!
//! This crate implements the PX4 ULog file format for embedded flash storage.
//! ULog is a self-describing binary format with little-endian byte ordering.
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
    EngineMessage, GnssMessage, GyroRawMessage, LogEventMessage, MagnetometerMessage, MessageType,
    PidGainsMessage, StatusMessage,
};
pub use writer::{ULogWriter, WriteError};
