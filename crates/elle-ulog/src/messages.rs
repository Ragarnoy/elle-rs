//! Flight data message definitions for ULog
//!
//! Defines the message structures and formats for logging flight controller data.

use crate::format::{MESSAGE_HEADER_SIZE, format_msg};
use embassy_time::Instant;

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

/// Attitude data message - logs IMU orientation and rates
///
/// Format: "attitude_data:uint64_t timestamp;float pitch;float roll;float yaw;float pitch_rate;float roll_rate;float yaw_rate"
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct AttitudeMessage {
    /// Timestamp in microseconds (monotonically increasing)
    timestamp: u64,
    /// Pitch angle in radians
    pitch: f32,
    /// Roll angle in radians
    roll: f32,
    /// Yaw angle in radians
    yaw: f32,
    /// Pitch rate in rad/s
    pitch_rate: f32,
    /// Roll rate in rad/s
    roll_rate: f32,
    /// Yaw rate in rad/s
    yaw_rate: f32,
}

impl AttitudeMessage {
    /// Format definition string for ULog
    pub(crate) const FORMAT: &'static str = "attitude_data:uint64_t timestamp;float pitch;float roll;float yaw;float pitch_rate;float roll_rate;float yaw_rate";

    /// Message name
    pub const NAME: &'static str = "attitude_data";

    /// Format definition message (header + `FORMAT`), built at compile time
    pub(crate) const FORMAT_MSG: &'static [u8] =
        &format_msg::<{ Self::FORMAT.len() + MESSAGE_HEADER_SIZE }>(Self::FORMAT);

    /// Create a new attitude message
    #[must_use]
    pub const fn new(
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
    pub(crate) const SIZE: usize = 32; // 8 + 6*4

    /// Serialize to little-endian bytes
    #[must_use]
    pub(crate) fn to_bytes(self) -> [u8; Self::SIZE] {
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

/// Pilot commands message - logs RC input, setpoints, PID output, and servo positions
///
/// Format: "commands:uint64_t timestamp;float throttle;float pitch;float roll;float yaw;uint8_t attitude_mode;float pitch_setpoint_deg;float roll_setpoint_deg;float pitch_correction;float roll_correction;uint32_t elevon_left_us;uint32_t elevon_right_us"
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct CommandsMessage {
    /// Timestamp in microseconds
    timestamp: u64,
    /// Throttle command [-1.0, 1.0]
    throttle: f32,
    /// Pitch command [-1.0, 1.0]
    pitch: f32,
    /// Roll command [-1.0, 1.0]
    roll: f32,
    /// Yaw command [-1.0, 1.0]
    yaw: f32,
    /// Attitude mode (0=Manual, 1=Stabilized, 2=AltitudeHold)
    attitude_mode: u8,
    /// Pitch setpoint in degrees
    pitch_setpoint_deg: f32,
    /// Roll setpoint in degrees
    roll_setpoint_deg: f32,
    /// PID pitch correction output [-1.0, 1.0]
    pitch_correction: f32,
    /// PID roll correction output [-1.0, 1.0]
    roll_correction: f32,
    /// Left elevon servo position in microseconds
    elevon_left_us: u32,
    /// Right elevon servo position in microseconds
    elevon_right_us: u32,
}

impl CommandsMessage {
    /// Format definition string for ULog
    pub(crate) const FORMAT: &'static str = "commands:uint64_t timestamp;float throttle;float pitch;float roll;float yaw;uint8_t attitude_mode;float pitch_setpoint_deg;float roll_setpoint_deg;float pitch_correction;float roll_correction;uint32_t elevon_left_us;uint32_t elevon_right_us";

    /// Message name
    pub const NAME: &'static str = "commands";

    /// Format definition message (header + `FORMAT`), built at compile time
    pub(crate) const FORMAT_MSG: &'static [u8] =
        &format_msg::<{ Self::FORMAT.len() + MESSAGE_HEADER_SIZE }>(Self::FORMAT);

    /// Size of the message in bytes
    pub(crate) const SIZE: usize = 49; // 8 + 4*4 + 1 + 2*4 + 2*4 + 2*4

    /// Create a new commands message
    #[must_use]
    #[allow(clippy::too_many_arguments)]
    pub const fn new(
        timestamp: Instant,
        throttle: f32,
        pitch: f32,
        roll: f32,
        yaw: f32,
        attitude_mode: u8,
        pitch_setpoint_deg: f32,
        roll_setpoint_deg: f32,
        pitch_correction: f32,
        roll_correction: f32,
        elevon_left_us: u32,
        elevon_right_us: u32,
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
            pitch_correction,
            roll_correction,
            elevon_left_us,
            elevon_right_us,
        }
    }

    /// Serialize to little-endian bytes
    #[must_use]
    pub(crate) fn to_bytes(self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0..8].copy_from_slice(&self.timestamp.to_le_bytes());
        buf[8..12].copy_from_slice(&self.throttle.to_le_bytes());
        buf[12..16].copy_from_slice(&self.pitch.to_le_bytes());
        buf[16..20].copy_from_slice(&self.roll.to_le_bytes());
        buf[20..24].copy_from_slice(&self.yaw.to_le_bytes());
        buf[24] = self.attitude_mode;
        buf[25..29].copy_from_slice(&self.pitch_setpoint_deg.to_le_bytes());
        buf[29..33].copy_from_slice(&self.roll_setpoint_deg.to_le_bytes());
        buf[33..37].copy_from_slice(&self.pitch_correction.to_le_bytes());
        buf[37..41].copy_from_slice(&self.roll_correction.to_le_bytes());
        buf[41..45].copy_from_slice(&self.elevon_left_us.to_le_bytes());
        buf[45..49].copy_from_slice(&self.elevon_right_us.to_le_bytes());
        buf
    }
}

/// Controller cycle message - what the attitude PID actually saw and did each tick:
/// timing, the setpoint it used, its P/I/D terms, mixer saturation and the pulses
/// the PWM really output. Complements `commands`, which logs pilot input.
///
/// Format: "controller:uint64_t timestamp;uint32_t dt_us;uint32_t att_age_us;float pitch_sp_deg;float roll_sp_deg;float pitch_p;float pitch_i;float pitch_d;float roll_p;float roll_i;float roll_d;uint8_t saturation;uint16_t elevon_left_pulse_us;uint16_t elevon_right_pulse_us"
#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct ControllerMessage {
    /// Timestamp in microseconds
    pub timestamp: u64,
    /// Measured time since the previous controller update, in microseconds
    pub dt_us: u32,
    /// Age of the attitude sample the PID used, in microseconds (`u32::MAX`: none)
    pub att_age_us: u32,
    /// Pitch setpoint the PID used (after smoothing and rate limit), degrees
    pub pitch_sp_deg: f32,
    /// Roll setpoint the PID used (after smoothing and rate limit), degrees
    pub roll_sp_deg: f32,
    /// Pitch P contribution (scaled; P + I + D = pitch correction)
    pub pitch_p: f32,
    /// Pitch I contribution (scaled)
    pub pitch_i: f32,
    /// Pitch D (gyro damping) contribution (scaled)
    pub pitch_d: f32,
    /// Roll P contribution (scaled; P + I + D = roll correction)
    pub roll_p: f32,
    /// Roll I contribution (scaled)
    pub roll_i: f32,
    /// Roll D (gyro damping) contribution (scaled)
    pub roll_d: f32,
    /// Mixer saturation bits: pitch_up, pitch_down, roll_right, roll_left (LSB first)
    pub saturation: u8,
    /// Left elevon pulse actually output (after trim), microseconds
    pub elevon_left_pulse_us: u16,
    /// Right elevon pulse actually output (after trim and inversion), microseconds
    pub elevon_right_pulse_us: u16,
}

impl ControllerMessage {
    /// Format definition string for ULog
    pub(crate) const FORMAT: &'static str = "controller:uint64_t timestamp;uint32_t dt_us;uint32_t att_age_us;float pitch_sp_deg;float roll_sp_deg;float pitch_p;float pitch_i;float pitch_d;float roll_p;float roll_i;float roll_d;uint8_t saturation;uint16_t elevon_left_pulse_us;uint16_t elevon_right_pulse_us";

    /// Message name
    pub const NAME: &'static str = "controller";

    /// Format definition message (header + `FORMAT`), built at compile time
    pub(crate) const FORMAT_MSG: &'static [u8] =
        &format_msg::<{ Self::FORMAT.len() + MESSAGE_HEADER_SIZE }>(Self::FORMAT);

    /// Size of the message in bytes
    pub(crate) const SIZE: usize = 53;

    /// Serialize to little-endian bytes
    #[must_use]
    pub(crate) fn to_bytes(self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0..8].copy_from_slice(&self.timestamp.to_le_bytes());
        buf[8..12].copy_from_slice(&self.dt_us.to_le_bytes());
        buf[12..16].copy_from_slice(&self.att_age_us.to_le_bytes());
        buf[16..20].copy_from_slice(&self.pitch_sp_deg.to_le_bytes());
        buf[20..24].copy_from_slice(&self.roll_sp_deg.to_le_bytes());
        buf[24..28].copy_from_slice(&self.pitch_p.to_le_bytes());
        buf[28..32].copy_from_slice(&self.pitch_i.to_le_bytes());
        buf[32..36].copy_from_slice(&self.pitch_d.to_le_bytes());
        buf[36..40].copy_from_slice(&self.roll_p.to_le_bytes());
        buf[40..44].copy_from_slice(&self.roll_i.to_le_bytes());
        buf[44..48].copy_from_slice(&self.roll_d.to_le_bytes());
        buf[48] = self.saturation;
        buf[49..51].copy_from_slice(&self.elevon_left_pulse_us.to_le_bytes());
        buf[51..53].copy_from_slice(&self.elevon_right_pulse_us.to_le_bytes());
        buf
    }
}

const _: () = assert!(
    ControllerMessage::SIZE == 8 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 1 + 2 + 2,
    "ControllerMessage::SIZE does not match its field widths"
);

/// Active PID gains - logged at recording start and whenever the gains change.
///
/// Format: "pid_gains:uint64_t timestamp;float kp_pitch;float ki_pitch;float kd_pitch;float kp_roll;float ki_roll;float kd_roll;float i_limit;float scale"
#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct PidGainsMessage {
    /// Timestamp in microseconds
    pub timestamp: u64,
    /// Pitch proportional gain
    pub kp_pitch: f32,
    /// Pitch integral gain
    pub ki_pitch: f32,
    /// Pitch derivative (rate) gain
    pub kd_pitch: f32,
    /// Roll proportional gain
    pub kp_roll: f32,
    /// Roll integral gain
    pub ki_roll: f32,
    /// Roll derivative (rate) gain
    pub kd_roll: f32,
    /// Integral clamp (rad·s)
    pub i_limit: f32,
    /// Output scale applied to P + I + D
    pub scale: f32,
}

impl PidGainsMessage {
    /// Format definition string for ULog
    pub(crate) const FORMAT: &'static str = "pid_gains:uint64_t timestamp;float kp_pitch;float ki_pitch;float kd_pitch;float kp_roll;float ki_roll;float kd_roll;float i_limit;float scale";

    /// Message name
    pub const NAME: &'static str = "pid_gains";

    /// Format definition message (header + `FORMAT`), built at compile time
    pub(crate) const FORMAT_MSG: &'static [u8] =
        &format_msg::<{ Self::FORMAT.len() + MESSAGE_HEADER_SIZE }>(Self::FORMAT);

    /// Size of the message in bytes
    pub(crate) const SIZE: usize = 40;

    /// Serialize to little-endian bytes
    #[must_use]
    pub(crate) fn to_bytes(self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0..8].copy_from_slice(&self.timestamp.to_le_bytes());
        buf[8..12].copy_from_slice(&self.kp_pitch.to_le_bytes());
        buf[12..16].copy_from_slice(&self.ki_pitch.to_le_bytes());
        buf[16..20].copy_from_slice(&self.kd_pitch.to_le_bytes());
        buf[20..24].copy_from_slice(&self.kp_roll.to_le_bytes());
        buf[24..28].copy_from_slice(&self.ki_roll.to_le_bytes());
        buf[28..32].copy_from_slice(&self.kd_roll.to_le_bytes());
        buf[32..36].copy_from_slice(&self.i_limit.to_le_bytes());
        buf[36..40].copy_from_slice(&self.scale.to_le_bytes());
        buf
    }
}

const _: () = assert!(
    PidGainsMessage::SIZE == 8 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4,
    "PidGainsMessage::SIZE does not match its field widths"
);

/// Raw gyro sample - every 1 kHz sample, unfiltered and bias-corrected, in the
/// airframe frame (rad/s, sensor axes: x = roll axis before the roll sign flip).
/// Only written by builds with the `gyro-raw-log` feature, for sizing the rate
/// filter from a real vibration spectrum.
///
/// Format: "gyro_raw:uint64_t timestamp;float gyro_x;float gyro_y;float gyro_z"
#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct GyroRawMessage {
    /// Timestamp in microseconds (Core1 time of processing; samples drained in
    /// one wake-up share nearly the same stamp)
    pub timestamp: u64,
    /// Gyro X (rad/s)
    pub gyro_x: f32,
    /// Gyro Y (rad/s)
    pub gyro_y: f32,
    /// Gyro Z (rad/s)
    pub gyro_z: f32,
}

impl GyroRawMessage {
    /// Format definition string for ULog
    pub(crate) const FORMAT: &'static str =
        "gyro_raw:uint64_t timestamp;float gyro_x;float gyro_y;float gyro_z";

    /// Message name
    pub const NAME: &'static str = "gyro_raw";

    /// Format definition message (header + `FORMAT`), built at compile time
    pub(crate) const FORMAT_MSG: &'static [u8] =
        &format_msg::<{ Self::FORMAT.len() + MESSAGE_HEADER_SIZE }>(Self::FORMAT);

    /// Size of the message in bytes
    pub(crate) const SIZE: usize = 20;

    /// Serialize to little-endian bytes
    #[must_use]
    pub(crate) fn to_bytes(self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0..8].copy_from_slice(&self.timestamp.to_le_bytes());
        buf[8..12].copy_from_slice(&self.gyro_x.to_le_bytes());
        buf[12..16].copy_from_slice(&self.gyro_y.to_le_bytes());
        buf[16..20].copy_from_slice(&self.gyro_z.to_le_bytes());
        buf
    }
}

const _: () = assert!(
    GyroRawMessage::SIZE == 8 + 4 + 4 + 4,
    "GyroRawMessage::SIZE does not match its field widths"
);

/// System status message - logs performance and health metrics
///
/// Format: "system_status:uint64_t timestamp;uint32_t loop_time_us;uint32_t imu_errors;uint8_t calibrated;uint8_t armed;float cpu_load;uint16_t rc_age_ms"
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct StatusMessage {
    /// Timestamp in microseconds
    timestamp: u64,
    /// Control loop time in microseconds
    loop_time_us: u32,
    /// IMU error count
    imu_errors: u32,
    /// Calibration status (0=uncalibrated, 1=calibrated)
    calibrated: u8,
    /// Armed status (0=disarmed, 1=armed)
    armed: u8,
    /// CPU load percentage [0.0, 100.0]
    cpu_load: f32,
    /// RC signal age in milliseconds
    rc_age_ms: u16,
}

impl StatusMessage {
    /// Format definition string for ULog
    pub(crate) const FORMAT: &'static str = "system_status:uint64_t timestamp;uint32_t loop_time_us;uint32_t imu_errors;uint8_t calibrated;uint8_t armed;float cpu_load;uint16_t rc_age_ms";

    /// Message name
    pub const NAME: &'static str = "system_status";

    /// Format definition message (header + `FORMAT`), built at compile time
    pub(crate) const FORMAT_MSG: &'static [u8] =
        &format_msg::<{ Self::FORMAT.len() + MESSAGE_HEADER_SIZE }>(Self::FORMAT);

    /// Size of the message in bytes
    pub(crate) const SIZE: usize = 24; // 8 + 4 + 4 + 1 + 1 + 4 + 2

    /// Create a new status message
    #[must_use]
    pub const fn new(
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
    pub(crate) fn to_bytes(self) -> [u8; Self::SIZE] {
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
/// Format: "barometer_data:uint64_t timestamp;float pressure_hpa;float temperature_c;float altitude_m;float vario_ms"
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct BarometerMessage {
    /// Timestamp in microseconds
    timestamp: u64,
    /// Pressure in hectopascals
    pressure_hpa: f32,
    /// Temperature in degrees Celsius
    temperature_c: f32,
    /// Barometric altitude in meters
    altitude_m: f32,
    /// Vertical speed in m/s (positive = climbing)
    vario_ms: f32,
}

impl BarometerMessage {
    /// Format definition string for ULog
    pub(crate) const FORMAT: &'static str = "barometer_data:uint64_t timestamp;float pressure_hpa;float temperature_c;float altitude_m;float vario_ms";

    /// Message name
    pub const NAME: &'static str = "barometer_data";

    /// Format definition message (header + `FORMAT`), built at compile time
    pub(crate) const FORMAT_MSG: &'static [u8] =
        &format_msg::<{ Self::FORMAT.len() + MESSAGE_HEADER_SIZE }>(Self::FORMAT);

    /// Size of the message in bytes
    pub(crate) const SIZE: usize = 24; // 8 + 4*4

    /// Create a new barometer message
    #[must_use]
    pub const fn new(
        timestamp: Instant,
        pressure_hpa: f32,
        temperature_c: f32,
        altitude_m: f32,
        vario_ms: f32,
    ) -> Self {
        Self {
            timestamp: timestamp.as_micros(),
            pressure_hpa,
            temperature_c,
            altitude_m,
            vario_ms,
        }
    }

    /// Serialize to little-endian bytes
    #[must_use]
    pub(crate) fn to_bytes(self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0..8].copy_from_slice(&self.timestamp.to_le_bytes());
        buf[8..12].copy_from_slice(&self.pressure_hpa.to_le_bytes());
        buf[12..16].copy_from_slice(&self.temperature_c.to_le_bytes());
        buf[16..20].copy_from_slice(&self.altitude_m.to_le_bytes());
        buf[20..24].copy_from_slice(&self.vario_ms.to_le_bytes());
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
    timestamp: u64,
    /// Magnetic field X axis (counts cast to float)
    mag_x: f32,
    /// Magnetic field Y axis (counts cast to float)
    mag_y: f32,
    /// Magnetic field Z axis (counts cast to float)
    mag_z: f32,
}

impl MagnetometerMessage {
    /// Format definition string for ULog
    pub(crate) const FORMAT: &'static str =
        "magnetometer_data:uint64_t timestamp;float mag_x;float mag_y;float mag_z";

    /// Message name
    pub const NAME: &'static str = "magnetometer_data";

    /// Format definition message (header + `FORMAT`), built at compile time
    pub(crate) const FORMAT_MSG: &'static [u8] =
        &format_msg::<{ Self::FORMAT.len() + MESSAGE_HEADER_SIZE }>(Self::FORMAT);

    /// Size of the message in bytes
    pub(crate) const SIZE: usize = 20; // 8 + 3*4

    /// Create a new magnetometer message
    #[must_use]
    pub const fn new(timestamp: Instant, mag_x: f32, mag_y: f32, mag_z: f32) -> Self {
        Self {
            timestamp: timestamp.as_micros(),
            mag_x,
            mag_y,
            mag_z,
        }
    }

    /// Serialize to little-endian bytes
    #[must_use]
    pub(crate) fn to_bytes(self) -> [u8; Self::SIZE] {
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
/// Position, velocity and accuracy come from UBX-NAV-PVT. On the NMEA GGA
/// fallback path only the position fields and `hdop` are meaningful; the
/// velocity and accuracy fields hold the last NAV-PVT values, or zero if none
/// was ever received.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct GnssMessage {
    /// Timestamp in microseconds
    timestamp: u64,
    /// Latitude in degrees
    latitude: f32,
    /// Longitude in degrees
    longitude: f32,
    /// Altitude in meters
    altitude_m: f32,
    /// Fix quality (0=none, 1=GPS, 2=DGPS)
    fix_quality: u8,
    /// Number of satellites
    num_satellites: u8,
    /// Horizontal dilution of precision (GGA path only)
    hdop: f32,
    /// Velocity north in m/s (NAV-PVT only)
    vel_n_ms: f32,
    /// Velocity east in m/s (NAV-PVT only)
    vel_e_ms: f32,
    /// Velocity down in m/s (NAV-PVT only)
    vel_d_ms: f32,
    /// Ground speed in m/s (NAV-PVT only)
    ground_speed_ms: f32,
    /// Course over ground in degrees (NAV-PVT only)
    heading_motion_deg: f32,
    /// Horizontal accuracy estimate in m (NAV-PVT only)
    h_acc_m: f32,
    /// Vertical accuracy estimate in m (NAV-PVT only)
    v_acc_m: f32,
    /// Speed accuracy estimate in m/s (NAV-PVT only)
    s_acc_ms: f32,
}

impl GnssMessage {
    /// Format definition string for ULog
    pub(crate) const FORMAT: &'static str = "gnss_data:uint64_t timestamp;float latitude;float longitude;float altitude_m;uint8_t fix_quality;uint8_t num_satellites;float hdop;float vel_n_ms;float vel_e_ms;float vel_d_ms;float ground_speed_ms;float heading_motion_deg;float h_acc_m;float v_acc_m;float s_acc_ms";

    /// Message name
    pub const NAME: &'static str = "gnss_data";

    /// Format definition message (header + `FORMAT`), built at compile time
    pub(crate) const FORMAT_MSG: &'static [u8] =
        &format_msg::<{ Self::FORMAT.len() + MESSAGE_HEADER_SIZE }>(Self::FORMAT);

    /// Size of the message in bytes
    pub(crate) const SIZE: usize = 58; // 8 + 3*4 + 1 + 1 + 4 + 8*4

    /// Create a new GNSS message
    #[must_use]
    #[allow(clippy::too_many_arguments)]
    pub const fn new(
        timestamp: Instant,
        latitude: f32,
        longitude: f32,
        altitude_m: f32,
        fix_quality: u8,
        num_satellites: u8,
        hdop: f32,
        vel_n_ms: f32,
        vel_e_ms: f32,
        vel_d_ms: f32,
        ground_speed_ms: f32,
        heading_motion_deg: f32,
        h_acc_m: f32,
        v_acc_m: f32,
        s_acc_ms: f32,
    ) -> Self {
        Self {
            timestamp: timestamp.as_micros(),
            latitude,
            longitude,
            altitude_m,
            fix_quality,
            num_satellites,
            hdop,
            vel_n_ms,
            vel_e_ms,
            vel_d_ms,
            ground_speed_ms,
            heading_motion_deg,
            h_acc_m,
            v_acc_m,
            s_acc_ms,
        }
    }

    /// Serialize to little-endian bytes
    #[must_use]
    pub(crate) fn to_bytes(self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0..8].copy_from_slice(&self.timestamp.to_le_bytes());
        buf[8..12].copy_from_slice(&self.latitude.to_le_bytes());
        buf[12..16].copy_from_slice(&self.longitude.to_le_bytes());
        buf[16..20].copy_from_slice(&self.altitude_m.to_le_bytes());
        buf[20] = self.fix_quality;
        buf[21] = self.num_satellites;
        buf[22..26].copy_from_slice(&self.hdop.to_le_bytes());
        buf[26..30].copy_from_slice(&self.vel_n_ms.to_le_bytes());
        buf[30..34].copy_from_slice(&self.vel_e_ms.to_le_bytes());
        buf[34..38].copy_from_slice(&self.vel_d_ms.to_le_bytes());
        buf[38..42].copy_from_slice(&self.ground_speed_ms.to_le_bytes());
        buf[42..46].copy_from_slice(&self.heading_motion_deg.to_le_bytes());
        buf[46..50].copy_from_slice(&self.h_acc_m.to_le_bytes());
        buf[50..54].copy_from_slice(&self.v_acc_m.to_le_bytes());
        buf[54..58].copy_from_slice(&self.s_acc_ms.to_le_bytes());
        buf
    }
}

// 8 (u64) + 3 f32 + 2 u8 + 1 f32 + 8 f32
const _: () = assert!(
    GnssMessage::SIZE == 8 + 3 * 4 + 1 + 1 + 4 + 8 * 4,
    "GnssMessage::SIZE does not match its field widths"
);

/// Engine data message - logs DShot engine RPM, throttle, governor targets, and EDT
///
/// Format: "engine_data:uint64_t timestamp;uint32_t left_erpm;uint32_t right_erpm;uint16_t left_throttle;uint16_t right_throttle;uint32_t left_target_erpm;uint32_t right_target_erpm;uint8_t left_temperature;uint8_t right_temperature;uint32_t left_voltage_mv;uint32_t right_voltage_mv;uint32_t left_current_ma;uint32_t right_current_ma"
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct EngineMessage {
    timestamp: u64,
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
}

impl EngineMessage {
    pub(crate) const FORMAT: &'static str = "engine_data:uint64_t timestamp;uint32_t left_erpm;uint32_t right_erpm;uint16_t left_throttle;uint16_t right_throttle;uint32_t left_target_erpm;uint32_t right_target_erpm;uint8_t left_temperature;uint8_t right_temperature;uint32_t left_voltage_mv;uint32_t right_voltage_mv;uint32_t left_current_ma;uint32_t right_current_ma";

    pub const NAME: &'static str = "engine_data";

    pub(crate) const FORMAT_MSG: &'static [u8] =
        &format_msg::<{ Self::FORMAT.len() + MESSAGE_HEADER_SIZE }>(Self::FORMAT);

    /// Size: 8 + 4+4 + 2+2 + 4+4 + 1+1 + 4+4 + 4+4 = 46
    pub(crate) const SIZE: usize = 46;

    #[must_use]
    #[allow(clippy::too_many_arguments)]
    pub const fn new(
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
    pub(crate) fn to_bytes(self) -> [u8; Self::SIZE] {
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

/// ESC link health: cumulative bidirectional DShot reply counters per engine.
///
/// `bad_frames` (replies failing GCR decoding or CRC) is the line-noise
/// indicator; `timeouts` climbing while stopped means the ESC is silent;
/// `reconfigs` counts configurations re-sent after an ESC (re)appeared.
///
/// Format: "esc_health:uint64_t timestamp;uint32_t left_replies;uint32_t right_replies;uint32_t left_timeouts;uint32_t right_timeouts;uint32_t left_bad_frames;uint32_t right_bad_frames;uint16_t left_reconfigs;uint16_t right_reconfigs"
#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct EscHealthMessage {
    /// Timestamp in microseconds (filled in by the logger)
    pub timestamp: u64,
    pub left_replies: u32,
    pub right_replies: u32,
    pub left_timeouts: u32,
    pub right_timeouts: u32,
    pub left_bad_frames: u32,
    pub right_bad_frames: u32,
    pub left_reconfigs: u16,
    pub right_reconfigs: u16,
}

impl EscHealthMessage {
    /// Format definition string for ULog
    pub(crate) const FORMAT: &'static str = "esc_health:uint64_t timestamp;uint32_t left_replies;uint32_t right_replies;uint32_t left_timeouts;uint32_t right_timeouts;uint32_t left_bad_frames;uint32_t right_bad_frames;uint16_t left_reconfigs;uint16_t right_reconfigs";

    /// Message name
    pub const NAME: &'static str = "esc_health";

    /// Format definition message (header + `FORMAT`), built at compile time
    pub(crate) const FORMAT_MSG: &'static [u8] =
        &format_msg::<{ Self::FORMAT.len() + MESSAGE_HEADER_SIZE }>(Self::FORMAT);

    /// Size of the message in bytes
    pub(crate) const SIZE: usize = 36;

    /// Serialize to little-endian bytes
    #[must_use]
    pub(crate) fn to_bytes(self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0..8].copy_from_slice(&self.timestamp.to_le_bytes());
        buf[8..12].copy_from_slice(&self.left_replies.to_le_bytes());
        buf[12..16].copy_from_slice(&self.right_replies.to_le_bytes());
        buf[16..20].copy_from_slice(&self.left_timeouts.to_le_bytes());
        buf[20..24].copy_from_slice(&self.right_timeouts.to_le_bytes());
        buf[24..28].copy_from_slice(&self.left_bad_frames.to_le_bytes());
        buf[28..32].copy_from_slice(&self.right_bad_frames.to_le_bytes());
        buf[32..34].copy_from_slice(&self.left_reconfigs.to_le_bytes());
        buf[34..36].copy_from_slice(&self.right_reconfigs.to_le_bytes());
        buf
    }
}

const _: () = assert!(
    EscHealthMessage::SIZE == 8 + 6 * 4 + 2 * 2,
    "EscHealthMessage::SIZE does not match its field widths"
);

/// Core 1 (IMU task) load over the last logging window (~1 s).
///
/// Busy time is wall time per IMU DATA_RDY wake-up (deadline 1 ms). `mag_max_us`
/// and `baro_max_us` are the longest mag and baro reads (their own task since
/// they left the IMU wake-up, so they no longer add to `busy_*`);
/// `max_drain` > 1 means a wake-up ran long enough for samples to queue.
///
/// Format: "core1_load:uint64_t timestamp;uint32_t wakes;uint32_t busy_avg_us;uint32_t busy_max_us;uint32_t mag_max_us;uint32_t baro_max_us;uint32_t max_drain"
#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct Core1LoadMessage {
    /// Timestamp in microseconds (filled in by the logger)
    pub timestamp: u64,
    pub wakes: u32,
    pub busy_avg_us: u32,
    pub busy_max_us: u32,
    pub mag_max_us: u32,
    pub baro_max_us: u32,
    pub max_drain: u32,
}

impl Core1LoadMessage {
    /// Format definition string for ULog
    pub(crate) const FORMAT: &'static str = "core1_load:uint64_t timestamp;uint32_t wakes;uint32_t busy_avg_us;uint32_t busy_max_us;uint32_t mag_max_us;uint32_t baro_max_us;uint32_t max_drain";

    /// Message name
    pub const NAME: &'static str = "core1_load";

    /// Format definition message (header + `FORMAT`), built at compile time
    pub(crate) const FORMAT_MSG: &'static [u8] =
        &format_msg::<{ Self::FORMAT.len() + MESSAGE_HEADER_SIZE }>(Self::FORMAT);

    /// Size of the message in bytes
    pub(crate) const SIZE: usize = 32;

    /// Serialize to little-endian bytes
    #[must_use]
    pub(crate) fn to_bytes(self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0..8].copy_from_slice(&self.timestamp.to_le_bytes());
        buf[8..12].copy_from_slice(&self.wakes.to_le_bytes());
        buf[12..16].copy_from_slice(&self.busy_avg_us.to_le_bytes());
        buf[16..20].copy_from_slice(&self.busy_max_us.to_le_bytes());
        buf[20..24].copy_from_slice(&self.mag_max_us.to_le_bytes());
        buf[24..28].copy_from_slice(&self.baro_max_us.to_le_bytes());
        buf[28..32].copy_from_slice(&self.max_drain.to_le_bytes());
        buf
    }
}

const _: () = assert!(
    Core1LoadMessage::SIZE == 8 + 6 * 4,
    "Core1LoadMessage::SIZE does not match its field widths"
);

/// Log event message — compact discrete event for ULog flash
///
/// Format: "log_event:uint64_t timestamp;uint8_t level;uint16_t code"
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct LogEventMessage {
    /// Timestamp in microseconds
    timestamp: u64,
    /// Log level (0=trace, 1=debug, 2=info, 3=warn, 4=error)
    level: u8,
    /// Application-defined event code
    code: u16,
}

impl LogEventMessage {
    /// Format definition string for ULog
    pub(crate) const FORMAT: &'static str =
        "log_event:uint64_t timestamp;uint8_t level;uint16_t code";

    /// Message name
    pub const NAME: &'static str = "log_event";

    /// Format definition message (header + `FORMAT`), built at compile time
    pub(crate) const FORMAT_MSG: &'static [u8] =
        &format_msg::<{ Self::FORMAT.len() + MESSAGE_HEADER_SIZE }>(Self::FORMAT);

    /// Size of the message in bytes
    pub(crate) const SIZE: usize = 11; // 8 + 1 + 2

    /// Create a new log event message
    #[must_use]
    pub const fn new(timestamp: Instant, level: u8, code: u16) -> Self {
        Self {
            timestamp: timestamp.as_micros(),
            level,
            code,
        }
    }

    /// Serialize to little-endian bytes
    #[must_use]
    pub(crate) fn to_bytes(self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0..8].copy_from_slice(&self.timestamp.to_le_bytes());
        buf[8] = self.level;
        buf[9..11].copy_from_slice(&self.code.to_le_bytes());
        buf
    }
}

/// Autotune status message — logged at the control-loop rate during active autotune only
///
/// Format: "autotune_status:uint64_t timestamp;uint8_t phase;uint8_t axis;uint8_t relay_positive;float setpoint_deg;float measurement_deg;uint8_t cycles_done;float amplitude_deg"
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct AutotuneMessage {
    /// Timestamp in microseconds
    timestamp: u64,
    /// Phase: 0=Idle, 1=Settling, 2=Relay, 3=Complete, 4=Aborted
    phase: u8,
    /// Axis: 0=Pitch, 1=Roll
    axis: u8,
    /// Current relay direction: 0=negative, 1=positive
    relay_positive: u8,
    /// Current setpoint being applied (degrees)
    setpoint_deg: f32,
    /// Current attitude measurement on tuned axis (degrees)
    measurement_deg: f32,
    /// Full oscillation cycles completed so far
    cycles_done: u8,
    /// Current half-cycle peak amplitude (degrees)
    amplitude_deg: f32,
}

impl AutotuneMessage {
    /// Format definition string for ULog
    pub(crate) const FORMAT: &'static str = "autotune_status:uint64_t timestamp;uint8_t phase;uint8_t axis;uint8_t relay_positive;float setpoint_deg;float measurement_deg;uint8_t cycles_done;float amplitude_deg";

    /// Message name
    pub const NAME: &'static str = "autotune_status";

    /// Format definition message (header + `FORMAT`), built at compile time
    pub(crate) const FORMAT_MSG: &'static [u8] =
        &format_msg::<{ Self::FORMAT.len() + MESSAGE_HEADER_SIZE }>(Self::FORMAT);

    /// Size of the message in bytes
    pub(crate) const SIZE: usize = 24; // 8 + 1 + 1 + 1 + 4 + 4 + 1 + 4

    /// Create a new autotune status message
    #[must_use]
    #[allow(clippy::too_many_arguments)]
    pub const fn new(
        timestamp: Instant,
        phase: u8,
        axis: u8,
        relay_positive: bool,
        setpoint_deg: f32,
        measurement_deg: f32,
        cycles_done: u8,
        amplitude_deg: f32,
    ) -> Self {
        Self {
            timestamp: timestamp.as_micros(),
            phase,
            axis,
            relay_positive: relay_positive as u8,
            setpoint_deg,
            measurement_deg,
            cycles_done,
            amplitude_deg,
        }
    }

    /// Serialize to little-endian bytes
    #[must_use]
    pub(crate) fn to_bytes(self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0..8].copy_from_slice(&self.timestamp.to_le_bytes());
        buf[8] = self.phase;
        buf[9] = self.axis;
        buf[10] = self.relay_positive;
        buf[11..15].copy_from_slice(&self.setpoint_deg.to_le_bytes());
        buf[15..19].copy_from_slice(&self.measurement_deg.to_le_bytes());
        buf[19] = self.cycles_done;
        buf[20..24].copy_from_slice(&self.amplitude_deg.to_le_bytes());
        buf
    }
}
