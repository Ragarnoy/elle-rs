//! Flash-efficient error handling using numeric codes

use defmt::Format;

/// Error codes for different subsystems
#[derive(Debug, Format, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
pub enum ElleError {
    // IMU errors (0x1000-0x1FFF)
    ImuInitFailed = 0x1001,
    ImuAxisConfigFailed = 0x1002,
    ImuModeConfigFailed = 0x1003,
    ImuQuaternionReadFailed = 0x1004,
    ImuGyroReadFailed = 0x1005,
    ImuCalibrationReadFailed = 0x1006,

    // Flash errors (0x2000-0x2FFF)
    FlashReadFailed = 0x2001,
    FlashEraseFailed = 0x2002,
    FlashWriteFailed = 0x2003,
    FlashAlignmentError = 0x2004,

    // Calibration errors (0x3000-0x3FFF)
    CalibrationLoadFailed = 0x3001,
    CalibrationSaveFailed = 0x3002,
    CalibrationProfileFailed = 0x3003,
    CalibrationTimeout = 0x3004,

    // Hardware errors (0x4000-0x4FFF)
    HardwareI2cError = 0x4001,
    HardwareSpiError = 0x4002,
    HardwareUartError = 0x4003,
    HardwareGpioError = 0x4004,
}

impl ElleError {
    /// Get the subsystem from error code
    pub fn subsystem(&self) -> &'static str {
        match (*self as u16) & 0xF000 {
            0x1000 => "IMU",
            0x2000 => "Flash",
            0x3000 => "Calibration",
            0x4000 => "Hardware",
            _ => "Unknown",
        }
    }

    /// Get error code as hex string for logging
    pub fn code(&self) -> u16 {
        *self as u16
    }
}

// Convenience type alias
pub type ElleResult<T> = Result<T, ElleError>;

/// Extension trait for converting other errors to ElleError
pub trait IntoElleError<T> {
    fn to_imu_init_err(self) -> ElleResult<T>;
    fn to_imu_axis_err(self) -> ElleResult<T>;
    fn to_imu_mode_err(self) -> ElleResult<T>;
    fn to_imu_quat_err(self) -> ElleResult<T>;
    fn to_imu_gyro_err(self) -> ElleResult<T>;
    fn to_imu_cal_err(self) -> ElleResult<T>;
    fn to_flash_read_err(self) -> ElleResult<T>;
    fn to_flash_erase_err(self) -> ElleResult<T>;
    fn to_flash_write_err(self) -> ElleResult<T>;
    fn to_flash_align_err(self) -> ElleResult<T>;
    fn to_cal_load_err(self) -> ElleResult<T>;
    fn to_cal_save_err(self) -> ElleResult<T>;
    fn to_cal_profile_err(self) -> ElleResult<T>;
}

impl<T, E> IntoElleError<T> for Result<T, E> {
    fn to_imu_init_err(self) -> ElleResult<T> {
        self.map_err(|_| ElleError::ImuInitFailed)
    }

    fn to_imu_axis_err(self) -> ElleResult<T> {
        self.map_err(|_| ElleError::ImuAxisConfigFailed)
    }

    fn to_imu_mode_err(self) -> ElleResult<T> {
        self.map_err(|_| ElleError::ImuModeConfigFailed)
    }

    fn to_imu_quat_err(self) -> ElleResult<T> {
        self.map_err(|_| ElleError::ImuQuaternionReadFailed)
    }

    fn to_imu_gyro_err(self) -> ElleResult<T> {
        self.map_err(|_| ElleError::ImuGyroReadFailed)
    }

    fn to_imu_cal_err(self) -> ElleResult<T> {
        self.map_err(|_| ElleError::ImuCalibrationReadFailed)
    }

    fn to_flash_read_err(self) -> ElleResult<T> {
        self.map_err(|_| ElleError::FlashReadFailed)
    }

    fn to_flash_erase_err(self) -> ElleResult<T> {
        self.map_err(|_| ElleError::FlashEraseFailed)
    }

    fn to_flash_write_err(self) -> ElleResult<T> {
        self.map_err(|_| ElleError::FlashWriteFailed)
    }

    fn to_flash_align_err(self) -> ElleResult<T> {
        self.map_err(|_| ElleError::FlashAlignmentError)
    }

    fn to_cal_load_err(self) -> ElleResult<T> {
        self.map_err(|_| ElleError::CalibrationLoadFailed)
    }

    fn to_cal_save_err(self) -> ElleResult<T> {
        self.map_err(|_| ElleError::CalibrationSaveFailed)
    }

    fn to_cal_profile_err(self) -> ElleResult<T> {
        self.map_err(|_| ElleError::CalibrationProfileFailed)
    }
}

// Host-side decoding (for development tools)
#[cfg(feature = "error-strings")]
impl ElleError {
    pub fn description(&self) -> &'static str {
        match self {
            ElleError::ImuInitFailed => "IMU initialization failed after multiple attempts",
            ElleError::ImuAxisConfigFailed => "IMU axis sign configuration failed",
            ElleError::ImuModeConfigFailed => "IMU NDOF mode configuration failed",
            ElleError::ImuQuaternionReadFailed => "IMU quaternion read failed",
            ElleError::ImuGyroReadFailed => "IMU gyroscope read failed",
            ElleError::ImuCalibrationReadFailed => "IMU calibration status read failed",

            ElleError::FlashReadFailed => "Flash read operation failed",
            ElleError::FlashEraseFailed => "Flash erase operation failed",
            ElleError::FlashWriteFailed => "Flash write operation failed",
            ElleError::FlashAlignmentError => "Flash write alignment error",

            ElleError::CalibrationLoadFailed => "Calibration load from flash failed",
            ElleError::CalibrationSaveFailed => "Calibration save to flash failed",
            ElleError::CalibrationProfileFailed => "Calibration profile retrieval failed",
            ElleError::CalibrationTimeout => "Calibration timeout",

            ElleError::HardwareI2cError => "I2C communication error",
            ElleError::HardwareSpiError => "SPI communication error",
            ElleError::HardwareUartError => "UART communication error",
            ElleError::HardwareGpioError => "GPIO configuration error",
        }
    }
}
