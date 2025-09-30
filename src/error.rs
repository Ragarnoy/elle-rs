//! Flash-efficient error handling using thiserror 2.0

use defmt::Format;
use thiserror::Error;

/// IMU-related errors
#[derive(Error, Debug, Format, Clone, Copy, PartialEq, Eq)]
pub enum ImuError {
    #[error("IMU initialization failed after multiple attempts")]
    InitializationFailed,

    #[error("IMU axis sign configuration failed")]
    AxisConfigFailed,

    #[error("IMU NDOF mode configuration failed")]
    ModeConfigFailed,

    #[error("IMU quaternion read failed")]
    QuaternionReadFailed,

    #[error("IMU gyroscope read failed")]
    GyroscopeReadFailed,

    #[error("IMU calibration status read failed")]
    CalibrationReadFailed,
}

/// Flash storage errors
#[derive(Error, Debug, Format, Clone, Copy, PartialEq, Eq)]
pub enum FlashError {
    #[error("Flash read operation failed")]
    ReadFailed,

    #[error("Flash erase operation failed")]
    EraseFailed,

    #[error("Flash write operation failed")]
    WriteFailed,

    #[error("Flash write alignment error")]
    AlignmentError,
}

/// Calibration errors
#[derive(Error, Debug, Format, Clone, Copy, PartialEq, Eq)]
pub enum CalibrationError {
    #[error("Calibration load from flash failed")]
    LoadFailed,

    #[error("Calibration save to flash failed")]
    SaveFailed,

    #[error("Calibration profile retrieval failed")]
    ProfileFailed,
}

/// Main error type that encompasses all subsystem errors
#[derive(Error, Debug, Format, Clone, Copy, PartialEq, Eq)]
pub enum ElleError {
    #[error("IMU error: {0}")]
    Imu(#[from] ImuError),

    #[error("Flash error: {0}")]
    Flash(#[from] FlashError),

    #[error("Calibration error: {0}")]
    Calibration(#[from] CalibrationError),
}

pub type ElleResult<T> = Result<T, ElleError>;
