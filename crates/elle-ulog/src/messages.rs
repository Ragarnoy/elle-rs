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
    #[must_use]
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

    /// Pre-serialized format definition message (header + payload, computed at compile time)
    pub const FORMAT_MSG: &'static [u8] = b"\x71\x00Fattitude_data:uint64_t timestamp;float pitch;float roll;float yaw;float pitch_rate;float roll_rate;float yaw_rate";

    /// Create a new attitude message
    #[must_use]
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
    #[must_use]
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

    /// Pre-serialized format definition message (header + payload, computed at compile time)
    /// msg_size = 146 bytes (format string length)
    pub const FORMAT_MSG: &'static [u8] = b"\x92\x00Fcommands:uint64_t timestamp;float throttle;float pitch;float roll;float yaw;uint8_t attitude_mode;float pitch_setpoint_deg;float roll_setpoint_deg";

    /// Size of the message in bytes
    pub const SIZE: usize = 33; // 8 + 4*4 + 1 + 2*4

    /// Create a new commands message
    #[must_use]
    #[allow(clippy::too_many_arguments)]
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
    #[must_use]
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
/// Format: "system_status:uint64_t timestamp;uint32_t loop_time_us;uint32_t imu_errors;uint8_t calibrated;uint8_t armed;float cpu_load;uint16_t rc_age_ms"
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
    /// RC signal age in milliseconds
    pub rc_age_ms: u16,
}

impl StatusMessage {
    /// Format definition string for ULog
    pub const FORMAT: &'static str = "system_status:uint64_t timestamp;uint32_t loop_time_us;uint32_t imu_errors;uint8_t calibrated;uint8_t armed;float cpu_load;uint16_t rc_age_ms";

    /// Message name
    pub const NAME: &'static str = "system_status";

    /// Pre-serialized format definition message (header + payload, computed at compile time)
    pub const FORMAT_MSG: &'static [u8] = b"\x8d\x00Fsystem_status:uint64_t timestamp;uint32_t loop_time_us;uint32_t imu_errors;uint8_t calibrated;uint8_t armed;float cpu_load;uint16_t rc_age_ms";

    /// Size of the message in bytes
    pub const SIZE: usize = 24; // 8 + 4 + 4 + 1 + 1 + 4 + 2

    /// Create a new status message
    #[must_use]
    pub fn new(
        timestamp: Instant,
        loop_time_us: u32,
        imu_errors: u32,
        calibrated: bool,
        armed: bool,
        cpu_load: f32,
        rc_age_ms: u16,
    ) -> Self {
        Self {
            timestamp: timestamp.as_micros(),
            loop_time_us,
            imu_errors,
            calibrated: calibrated as u8,
            armed: armed as u8,
            cpu_load,
            rc_age_ms,
        }
    }

    /// Serialize to little-endian bytes
    #[must_use]
    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0..8].copy_from_slice(&self.timestamp.to_le_bytes());
        buf[8..12].copy_from_slice(&self.loop_time_us.to_le_bytes());
        buf[12..16].copy_from_slice(&self.imu_errors.to_le_bytes());
        buf[16] = self.calibrated;
        buf[17] = self.armed;
        buf[18..22].copy_from_slice(&self.cpu_load.to_le_bytes());
        buf[22..24].copy_from_slice(&self.rc_age_ms.to_le_bytes());
        buf
    }
}

/// Barometer data message - logs BMP390 pressure, temperature, altitude
///
/// Format: "barometer_data:uint64_t timestamp;float pressure_hpa;float temperature_c;float altitude_m"
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct BarometerMessage {
    /// Timestamp in microseconds
    pub timestamp: u64,
    /// Pressure in hectopascals
    pub pressure_hpa: f32,
    /// Temperature in degrees Celsius
    pub temperature_c: f32,
    /// Barometric altitude in meters
    pub altitude_m: f32,
}

impl BarometerMessage {
    /// Format definition string for ULog
    pub const FORMAT: &'static str =
        "barometer_data:uint64_t timestamp;float pressure_hpa;float temperature_c;float altitude_m";

    /// Message name
    pub const NAME: &'static str = "barometer_data";

    /// Pre-serialized format definition message (header + payload, computed at compile time)
    pub const FORMAT_MSG: &'static [u8] = b"\x59\x00Fbarometer_data:uint64_t timestamp;float pressure_hpa;float temperature_c;float altitude_m";

    /// Size of the message in bytes
    pub const SIZE: usize = 20; // 8 + 3*4

    /// Create a new barometer message
    #[must_use]
    pub fn new(timestamp: Instant, pressure_hpa: f32, temperature_c: f32, altitude_m: f32) -> Self {
        Self {
            timestamp: timestamp.as_micros(),
            pressure_hpa,
            temperature_c,
            altitude_m,
        }
    }

    /// Serialize to little-endian bytes
    #[must_use]
    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0..8].copy_from_slice(&self.timestamp.to_le_bytes());
        buf[8..12].copy_from_slice(&self.pressure_hpa.to_le_bytes());
        buf[12..16].copy_from_slice(&self.temperature_c.to_le_bytes());
        buf[16..20].copy_from_slice(&self.altitude_m.to_le_bytes());
        buf
    }
}

/// Magnetometer data message - logs MMC5616WA magnetic field
///
/// Format: "magnetometer_data:uint64_t timestamp;float mag_x;float mag_y;float mag_z"
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct MagnetometerMessage {
    /// Timestamp in microseconds
    pub timestamp: u64,
    /// Magnetic field X axis (counts cast to float)
    pub mag_x: f32,
    /// Magnetic field Y axis (counts cast to float)
    pub mag_y: f32,
    /// Magnetic field Z axis (counts cast to float)
    pub mag_z: f32,
}

impl MagnetometerMessage {
    /// Format definition string for ULog
    pub const FORMAT: &'static str =
        "magnetometer_data:uint64_t timestamp;float mag_x;float mag_y;float mag_z";

    /// Message name
    pub const NAME: &'static str = "magnetometer_data";

    /// Pre-serialized format definition message (header + payload, computed at compile time)
    pub const FORMAT_MSG: &'static [u8] =
        b"\x48\x00Fmagnetometer_data:uint64_t timestamp;float mag_x;float mag_y;float mag_z";

    /// Size of the message in bytes
    pub const SIZE: usize = 20; // 8 + 3*4

    /// Create a new magnetometer message
    #[must_use]
    pub fn new(timestamp: Instant, mag_x: f32, mag_y: f32, mag_z: f32) -> Self {
        Self {
            timestamp: timestamp.as_micros(),
            mag_x,
            mag_y,
            mag_z,
        }
    }

    /// Serialize to little-endian bytes
    #[must_use]
    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0..8].copy_from_slice(&self.timestamp.to_le_bytes());
        buf[8..12].copy_from_slice(&self.mag_x.to_le_bytes());
        buf[12..16].copy_from_slice(&self.mag_y.to_le_bytes());
        buf[16..20].copy_from_slice(&self.mag_z.to_le_bytes());
        buf
    }
}

/// GNSS data message - logs GPS position fix
///
/// Format: "gnss_data:uint64_t timestamp;float latitude;float longitude;float altitude_m;uint8_t fix_quality;uint8_t num_satellites;float hdop"
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct GnssMessage {
    /// Timestamp in microseconds
    pub timestamp: u64,
    /// Latitude in degrees
    pub latitude: f32,
    /// Longitude in degrees
    pub longitude: f32,
    /// Altitude in meters
    pub altitude_m: f32,
    /// Fix quality (0=none, 1=GPS, 2=DGPS)
    pub fix_quality: u8,
    /// Number of satellites
    pub num_satellites: u8,
    /// Horizontal dilution of precision
    pub hdop: f32,
}

impl GnssMessage {
    /// Format definition string for ULog
    pub const FORMAT: &'static str = "gnss_data:uint64_t timestamp;float latitude;float longitude;float altitude_m;uint8_t fix_quality;uint8_t num_satellites;float hdop";

    /// Message name
    pub const NAME: &'static str = "gnss_data";

    /// Pre-serialized format definition message (header + payload, computed at compile time)
    pub const FORMAT_MSG: &'static [u8] = b"\x82\x00Fgnss_data:uint64_t timestamp;float latitude;float longitude;float altitude_m;uint8_t fix_quality;uint8_t num_satellites;float hdop";

    /// Size of the message in bytes
    pub const SIZE: usize = 26; // 8 + 3*4 + 1 + 1 + 4

    /// Create a new GNSS message
    #[must_use]
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        timestamp: Instant,
        latitude: f32,
        longitude: f32,
        altitude_m: f32,
        fix_quality: u8,
        num_satellites: u8,
        hdop: f32,
    ) -> Self {
        Self {
            timestamp: timestamp.as_micros(),
            latitude,
            longitude,
            altitude_m,
            fix_quality,
            num_satellites,
            hdop,
        }
    }

    /// Serialize to little-endian bytes
    #[must_use]
    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0..8].copy_from_slice(&self.timestamp.to_le_bytes());
        buf[8..12].copy_from_slice(&self.latitude.to_le_bytes());
        buf[12..16].copy_from_slice(&self.longitude.to_le_bytes());
        buf[16..20].copy_from_slice(&self.altitude_m.to_le_bytes());
        buf[20] = self.fix_quality;
        buf[21] = self.num_satellites;
        buf[22..26].copy_from_slice(&self.hdop.to_le_bytes());
        buf
    }
}

/// Engine data message - logs DShot engine RPM, throttle, governor targets, and EDT
///
/// Format: "engine_data:uint64_t timestamp;uint32_t left_erpm;uint32_t right_erpm;uint16_t left_throttle;uint16_t right_throttle;uint32_t left_target_erpm;uint32_t right_target_erpm;uint8_t left_temperature;uint8_t right_temperature;uint32_t left_voltage_mv;uint32_t right_voltage_mv;uint32_t left_current_ma;uint32_t right_current_ma"
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct EngineMessage {
    pub timestamp: u64,
    pub left_erpm: u32,
    pub right_erpm: u32,
    pub left_throttle: u16,
    pub right_throttle: u16,
    pub left_target_erpm: u32,
    pub right_target_erpm: u32,
    pub left_temperature: u8,
    pub right_temperature: u8,
    pub left_voltage_mv: u32,
    pub right_voltage_mv: u32,
    pub left_current_ma: u32,
    pub right_current_ma: u32,
}

impl EngineMessage {
    pub const FORMAT: &'static str = "engine_data:uint64_t timestamp;uint32_t left_erpm;uint32_t right_erpm;uint16_t left_throttle;uint16_t right_throttle;uint32_t left_target_erpm;uint32_t right_target_erpm;uint8_t left_temperature;uint8_t right_temperature;uint32_t left_voltage_mv;uint32_t right_voltage_mv;uint32_t left_current_ma;uint32_t right_current_ma";

    pub const NAME: &'static str = "engine_data";

    pub const FORMAT_MSG: &'static [u8] = b"\x42\x01Fengine_data:uint64_t timestamp;uint32_t left_erpm;uint32_t right_erpm;uint16_t left_throttle;uint16_t right_throttle;uint32_t left_target_erpm;uint32_t right_target_erpm;uint8_t left_temperature;uint8_t right_temperature;uint32_t left_voltage_mv;uint32_t right_voltage_mv;uint32_t left_current_ma;uint32_t right_current_ma";

    /// Size: 8 + 4+4 + 2+2 + 4+4 + 1+1 + 4+4 + 4+4 = 46
    pub const SIZE: usize = 46;

    #[must_use]
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        timestamp: Instant,
        left_erpm: u32,
        right_erpm: u32,
        left_throttle: u16,
        right_throttle: u16,
        left_target_erpm: u32,
        right_target_erpm: u32,
        left_temperature: u8,
        right_temperature: u8,
        left_voltage_mv: u32,
        right_voltage_mv: u32,
        left_current_ma: u32,
        right_current_ma: u32,
    ) -> Self {
        Self {
            timestamp: timestamp.as_micros(),
            left_erpm,
            right_erpm,
            left_throttle,
            right_throttle,
            left_target_erpm,
            right_target_erpm,
            left_temperature,
            right_temperature,
            left_voltage_mv,
            right_voltage_mv,
            left_current_ma,
            right_current_ma,
        }
    }

    #[must_use]
    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0..8].copy_from_slice(&self.timestamp.to_le_bytes());
        buf[8..12].copy_from_slice(&self.left_erpm.to_le_bytes());
        buf[12..16].copy_from_slice(&self.right_erpm.to_le_bytes());
        buf[16..18].copy_from_slice(&self.left_throttle.to_le_bytes());
        buf[18..20].copy_from_slice(&self.right_throttle.to_le_bytes());
        buf[20..24].copy_from_slice(&self.left_target_erpm.to_le_bytes());
        buf[24..28].copy_from_slice(&self.right_target_erpm.to_le_bytes());
        buf[28] = self.left_temperature;
        buf[29] = self.right_temperature;
        buf[30..34].copy_from_slice(&self.left_voltage_mv.to_le_bytes());
        buf[34..38].copy_from_slice(&self.right_voltage_mv.to_le_bytes());
        buf[38..42].copy_from_slice(&self.left_current_ma.to_le_bytes());
        buf[42..46].copy_from_slice(&self.right_current_ma.to_le_bytes());
        buf
    }
}

/// Log event message — compact discrete event for ULog flash
///
/// Format: "log_event:uint64_t timestamp;uint8_t level;uint16_t code"
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct LogEventMessage {
    /// Timestamp in microseconds
    pub timestamp: u64,
    /// Log level (0=trace, 1=debug, 2=info, 3=warn, 4=error)
    pub level: u8,
    /// Application-defined event code
    pub code: u16,
}

impl LogEventMessage {
    /// Format definition string for ULog
    pub const FORMAT: &'static str = "log_event:uint64_t timestamp;uint8_t level;uint16_t code";

    /// Message name
    pub const NAME: &'static str = "log_event";

    /// Pre-serialized format definition message (header + payload, computed at compile time)
    pub const FORMAT_MSG: &'static [u8] =
        b"\x38\x00Flog_event:uint64_t timestamp;uint8_t level;uint16_t code";

    /// Size of the message in bytes
    pub const SIZE: usize = 11; // 8 + 1 + 2

    /// Create a new log event message
    #[must_use]
    pub fn new(timestamp: Instant, level: u8, code: u16) -> Self {
        Self {
            timestamp: timestamp.as_micros(),
            level,
            code,
        }
    }

    /// Serialize to little-endian bytes
    #[must_use]
    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0..8].copy_from_slice(&self.timestamp.to_le_bytes());
        buf[8] = self.level;
        buf[9..11].copy_from_slice(&self.code.to_le_bytes());
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
    #[must_use]
    pub fn new(log_level: LogLevel, timestamp: Instant, message: &'a str) -> Self {
        Self {
            log_level,
            timestamp: timestamp.as_micros(),
            message,
        }
    }

    /// Calculate message size
    #[must_use]
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

// Compile-time checks: the msg_size field (LE u16 in first 2 bytes of FORMAT_MSG)
// must equal FORMAT string length, and total FORMAT_MSG length must be 3 + FORMAT length.
const _: () = {
    // AttitudeMessage
    let encoded =
        AttitudeMessage::FORMAT_MSG[0] as usize | (AttitudeMessage::FORMAT_MSG[1] as usize) << 8;
    assert!(
        encoded == AttitudeMessage::FORMAT.len(),
        "AttitudeMessage FORMAT_MSG msg_size mismatch"
    );
    assert!(AttitudeMessage::FORMAT_MSG.len() == 3 + AttitudeMessage::FORMAT.len());

    // CommandsMessage
    let encoded =
        CommandsMessage::FORMAT_MSG[0] as usize | (CommandsMessage::FORMAT_MSG[1] as usize) << 8;
    assert!(
        encoded == CommandsMessage::FORMAT.len(),
        "CommandsMessage FORMAT_MSG msg_size mismatch"
    );
    assert!(CommandsMessage::FORMAT_MSG.len() == 3 + CommandsMessage::FORMAT.len());

    // StatusMessage
    let encoded =
        StatusMessage::FORMAT_MSG[0] as usize | (StatusMessage::FORMAT_MSG[1] as usize) << 8;
    assert!(
        encoded == StatusMessage::FORMAT.len(),
        "StatusMessage FORMAT_MSG msg_size mismatch"
    );
    assert!(StatusMessage::FORMAT_MSG.len() == 3 + StatusMessage::FORMAT.len());

    // BarometerMessage
    let encoded =
        BarometerMessage::FORMAT_MSG[0] as usize | (BarometerMessage::FORMAT_MSG[1] as usize) << 8;
    assert!(
        encoded == BarometerMessage::FORMAT.len(),
        "BarometerMessage FORMAT_MSG msg_size mismatch"
    );
    assert!(BarometerMessage::FORMAT_MSG.len() == 3 + BarometerMessage::FORMAT.len());

    // MagnetometerMessage
    let encoded = MagnetometerMessage::FORMAT_MSG[0] as usize
        | (MagnetometerMessage::FORMAT_MSG[1] as usize) << 8;
    assert!(
        encoded == MagnetometerMessage::FORMAT.len(),
        "MagnetometerMessage FORMAT_MSG msg_size mismatch"
    );
    assert!(MagnetometerMessage::FORMAT_MSG.len() == 3 + MagnetometerMessage::FORMAT.len());

    // GnssMessage
    let encoded = GnssMessage::FORMAT_MSG[0] as usize | (GnssMessage::FORMAT_MSG[1] as usize) << 8;
    assert!(
        encoded == GnssMessage::FORMAT.len(),
        "GnssMessage FORMAT_MSG msg_size mismatch"
    );
    assert!(GnssMessage::FORMAT_MSG.len() == 3 + GnssMessage::FORMAT.len());

    // LogEventMessage
    let encoded =
        LogEventMessage::FORMAT_MSG[0] as usize | (LogEventMessage::FORMAT_MSG[1] as usize) << 8;
    assert!(
        encoded == LogEventMessage::FORMAT.len(),
        "LogEventMessage FORMAT_MSG msg_size mismatch"
    );
    assert!(LogEventMessage::FORMAT_MSG.len() == 3 + LogEventMessage::FORMAT.len());

    // EngineMessage
    let encoded =
        EngineMessage::FORMAT_MSG[0] as usize | (EngineMessage::FORMAT_MSG[1] as usize) << 8;
    assert!(
        encoded == EngineMessage::FORMAT.len(),
        "EngineMessage FORMAT_MSG msg_size mismatch"
    );
    assert!(EngineMessage::FORMAT_MSG.len() == 3 + EngineMessage::FORMAT.len());
};
