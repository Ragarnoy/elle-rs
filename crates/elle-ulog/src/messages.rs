//! Flight data message definitions for ULog
//!
//! Defines the message structures and formats for logging flight controller data.

use embassy_time::Instant;

/// Log levels for string messages (type 'L' and 'C')
#[repr(u8)]
#[derive(Debug, Clone, Copy)]
pub enum LogLevel {
    Emergency = 0,
    Alert = 1,
    Critical = 2,
    Error = 3,
    Warning = 4,
    Notice = 5,
    Info = 6,
    Debug = 7,
}

/// Message type identifiers
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MessageType {
    /// 'B' - Flag bits (must be first)
    FlagBits = b'B',
    /// 'F' - Format definition
    Format = b'F',
    /// 'I' - Information key-value
    Info = b'I',
    /// 'M' - Multi information
    MultiInfo = b'M',
    /// 'P' - Parameter
    Parameter = b'P',
    /// 'Q' - Default parameter
    DefaultParameter = b'Q',
    /// 'A' - Add logged message (subscription)
    AddLogged = b'A',
    /// 'R' - Remove logged message
    RemoveLogged = b'R',
    /// 'D' - Data
    Data = b'D',
    /// 'L' - Logged string
    LoggedString = b'L',
    /// 'C' - Tagged logged string
    TaggedLoggedString = b'C',
    /// 'S' - Synchronization
    Sync = b'S',
    /// 'O' - Dropout mark
    Dropout = b'O',
}

/// Message definition helper
#[derive(Debug, Clone, Copy)]
pub struct MessageDefinition {
    pub msg_id: u16,
    pub multi_id: u8,
}

impl MessageDefinition {
    pub const fn new(msg_id: u16, multi_id: u8) -> Self {
        Self { msg_id, multi_id }
    }
}

/// Attitude data message - logs IMU orientation and rates
///
/// Format: "attitude_data:uint64_t timestamp;float pitch;float roll;float yaw;float pitch_rate;float roll_rate;float yaw_rate"
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct AttitudeMessage {
    /// Timestamp in microseconds (monotonically increasing)
    pub timestamp: u64,
    /// Pitch angle in radians
    pub pitch: f32,
    /// Roll angle in radians
    pub roll: f32,
    /// Yaw angle in radians
    pub yaw: f32,
    /// Pitch rate in rad/s
    pub pitch_rate: f32,
    /// Roll rate in rad/s
    pub roll_rate: f32,
    /// Yaw rate in rad/s
    pub yaw_rate: f32,
}

impl AttitudeMessage {
    /// Format definition string for ULog
    pub const FORMAT: &'static str = "attitude_data:uint64_t timestamp;float pitch;float roll;float yaw;float pitch_rate;float roll_rate;float yaw_rate";

    /// Message name
    pub const NAME: &'static str = "attitude_data";

    /// Create a new attitude message
    pub fn new(
        timestamp: Instant,
        pitch: f32,
        roll: f32,
        yaw: f32,
        pitch_rate: f32,
        roll_rate: f32,
        yaw_rate: f32,
    ) -> Self {
        Self {
            timestamp: timestamp.as_micros(),
            pitch,
            roll,
            yaw,
            pitch_rate,
            roll_rate,
            yaw_rate,
        }
    }

    /// Size of the message in bytes
    pub const SIZE: usize = 32; // 8 + 6*4

    /// Serialize to little-endian bytes
    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0..8].copy_from_slice(&self.timestamp.to_le_bytes());
        buf[8..12].copy_from_slice(&self.pitch.to_le_bytes());
        buf[12..16].copy_from_slice(&self.roll.to_le_bytes());
        buf[16..20].copy_from_slice(&self.yaw.to_le_bytes());
        buf[20..24].copy_from_slice(&self.pitch_rate.to_le_bytes());
        buf[24..28].copy_from_slice(&self.roll_rate.to_le_bytes());
        buf[28..32].copy_from_slice(&self.yaw_rate.to_le_bytes());
        buf
    }
}

/// Pilot commands message - logs RC input and setpoints
///
/// Format: "commands:uint64_t timestamp;float throttle;float pitch;float roll;float yaw;uint8_t attitude_mode;float pitch_setpoint_deg;float roll_setpoint_deg"
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct CommandsMessage {
    /// Timestamp in microseconds
    pub timestamp: u64,
    /// Throttle command [-1.0, 1.0]
    pub throttle: f32,
    /// Pitch command [-1.0, 1.0]
    pub pitch: f32,
    /// Roll command [-1.0, 1.0]
    pub roll: f32,
    /// Yaw command [-1.0, 1.0]
    pub yaw: f32,
    /// Attitude mode (0=Rate, 1=Angle)
    pub attitude_mode: u8,
    /// Pitch setpoint in degrees
    pub pitch_setpoint_deg: f32,
    /// Roll setpoint in degrees
    pub roll_setpoint_deg: f32,
}

impl CommandsMessage {
    /// Format definition string for ULog
    pub const FORMAT: &'static str = "commands:uint64_t timestamp;float throttle;float pitch;float roll;float yaw;uint8_t attitude_mode;float pitch_setpoint_deg;float roll_setpoint_deg";

    /// Message name
    pub const NAME: &'static str = "commands";

    /// Size of the message in bytes
    pub const SIZE: usize = 37; // 8 + 6*4 + 1 + 4

    /// Create a new commands message
    pub fn new(
        timestamp: Instant,
        throttle: f32,
        pitch: f32,
        roll: f32,
        yaw: f32,
        attitude_mode: u8,
        pitch_setpoint_deg: f32,
        roll_setpoint_deg: f32,
    ) -> Self {
        Self {
            timestamp: timestamp.as_micros(),
            throttle,
            pitch,
            roll,
            yaw,
            attitude_mode,
            pitch_setpoint_deg,
            roll_setpoint_deg,
        }
    }

    /// Serialize to little-endian bytes
    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0..8].copy_from_slice(&self.timestamp.to_le_bytes());
        buf[8..12].copy_from_slice(&self.throttle.to_le_bytes());
        buf[12..16].copy_from_slice(&self.pitch.to_le_bytes());
        buf[16..20].copy_from_slice(&self.roll.to_le_bytes());
        buf[20..24].copy_from_slice(&self.yaw.to_le_bytes());
        buf[24] = self.attitude_mode;
        buf[25..29].copy_from_slice(&self.pitch_setpoint_deg.to_le_bytes());
        buf[29..33].copy_from_slice(&self.roll_setpoint_deg.to_le_bytes());
        // Pad to alignment if needed
        buf
    }
}

/// System status message - logs performance and health metrics
///
/// Format: "system_status:uint64_t timestamp;uint32_t loop_time_us;uint32_t imu_errors;uint8_t calibrated;uint8_t armed;float cpu_load"
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct StatusMessage {
    /// Timestamp in microseconds
    pub timestamp: u64,
    /// Control loop time in microseconds
    pub loop_time_us: u32,
    /// IMU error count
    pub imu_errors: u32,
    /// Calibration status (0=uncalibrated, 1=calibrated)
    pub calibrated: u8,
    /// Armed status (0=disarmed, 1=armed)
    pub armed: u8,
    /// CPU load percentage [0.0, 100.0]
    pub cpu_load: f32,
}

impl StatusMessage {
    /// Format definition string for ULog
    pub const FORMAT: &'static str = "system_status:uint64_t timestamp;uint32_t loop_time_us;uint32_t imu_errors;uint8_t calibrated;uint8_t armed;float cpu_load";

    /// Message name
    pub const NAME: &'static str = "system_status";

    /// Size of the message in bytes
    pub const SIZE: usize = 22; // 8 + 4 + 4 + 1 + 1 + 4

    /// Create a new status message
    pub fn new(
        timestamp: Instant,
        loop_time_us: u32,
        imu_errors: u32,
        calibrated: bool,
        armed: bool,
        cpu_load: f32,
    ) -> Self {
        Self {
            timestamp: timestamp.as_micros(),
            loop_time_us,
            imu_errors,
            calibrated: calibrated as u8,
            armed: armed as u8,
            cpu_load,
        }
    }

    /// Serialize to little-endian bytes
    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0..8].copy_from_slice(&self.timestamp.to_le_bytes());
        buf[8..12].copy_from_slice(&self.loop_time_us.to_le_bytes());
        buf[12..16].copy_from_slice(&self.imu_errors.to_le_bytes());
        buf[16] = self.calibrated;
        buf[17] = self.armed;
        buf[18..22].copy_from_slice(&self.cpu_load.to_le_bytes());
        buf
    }
}

/// Logged string message (type 'L')
#[derive(Debug, Clone)]
pub struct LoggedString<'a> {
    pub log_level: LogLevel,
    pub timestamp: u64,
    pub message: &'a str,
}

impl<'a> LoggedString<'a> {
    /// Create a new logged string
    pub fn new(log_level: LogLevel, timestamp: Instant, message: &'a str) -> Self {
        Self {
            log_level,
            timestamp: timestamp.as_micros(),
            message,
        }
    }

    /// Calculate message size
    pub fn msg_size(&self) -> u16 {
        (9 + self.message.len()) as u16
    }

    /// Serialize to bytes
    pub fn to_bytes(&self, buf: &mut [u8]) -> usize {
        buf[0] = self.log_level as u8;
        buf[1..9].copy_from_slice(&self.timestamp.to_le_bytes());
        buf[9..9 + self.message.len()].copy_from_slice(self.message.as_bytes());
        9 + self.message.len()
    }
}
