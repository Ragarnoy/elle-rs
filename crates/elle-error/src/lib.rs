#![no_std]

use defmt::Format;
use thiserror::Error;

/// IMU-related errors
#[derive(Error, Debug, Format, Clone, Copy, PartialEq, Eq)]
pub enum ImuError {
    #[error("IMU initialization failed after multiple attempts")]
    InitializationFailed,
}

/// ULog logging errors
#[derive(Error, Debug, Format, Clone, Copy, PartialEq, Eq)]
pub enum ULogError {
    #[error("ULog not initialized")]
    NotInitialized,

    #[error("ULog initialization failed")]
    InitFailed,

    #[error("ULog buffer overflow")]
    BufferFull,

    #[error("ULog flash write failed")]
    FlushFailed,
}

/// Main error type that encompasses all subsystem errors
#[derive(Error, Debug, Format, Clone, Copy, PartialEq, Eq)]
pub enum ElleError {
    #[error("IMU error: {0}")]
    Imu(#[from] ImuError),
}

pub type ElleResult<T> = Result<T, ElleError>;
