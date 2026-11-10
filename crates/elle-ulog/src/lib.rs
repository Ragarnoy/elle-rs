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
    Core1LoadMessage, EngineMessage, EscHealthMessage, GnssMessage, ImuRawCtxMessage,
    ImuRawFixMessage, ImuRawMagMessage, ImuRawMessage, LogEventMessage, LoopStagesMessage,
    MagnetometerMessage, MessageType, NavMessage, PidGainsMessage, StatusMessage,
};
pub use writer::{ULogWriter, WriteError};

/// Info entry naming the sign of `attitude_data.yaw` and `yaw_rate`. Logs
/// written before proposal 0006 lack it and have nose **left** positive.
pub const YAW_INFO_KEY: &str = "char[] elle_yaw";
/// [`YAW_INFO_KEY`]'s value: yaw rises as the nose turns right (clockwise
/// seen from above).
pub const YAW_NOSE_RIGHT_POSITIVE: &str = "nose_right_positive";
