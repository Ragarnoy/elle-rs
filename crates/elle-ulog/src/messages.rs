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
/// Format: "controller:uint64_t timestamp;uint32_t dt_us;uint32_t att_age_us;float pitch_sp_deg;float roll_sp_deg;float pitch_p;float pitch_i;float pitch_d;float roll_p;float roll_i;float roll_d;uint8_t saturation;uint16_t elevon_left_pulse_us;uint16_t elevon_right_pulse_us;float yaw_damp"
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
    /// Yaw damper command added to the differential thrust (normalized yaw,
    /// positive slows the left engine); 0 when the damper is not running
    pub yaw_damp: f32,
}

impl ControllerMessage {
    /// Format definition string for ULog
    pub(crate) const FORMAT: &'static str = "controller:uint64_t timestamp;uint32_t dt_us;uint32_t att_age_us;float pitch_sp_deg;float roll_sp_deg;float pitch_p;float pitch_i;float pitch_d;float roll_p;float roll_i;float roll_d;uint8_t saturation;uint16_t elevon_left_pulse_us;uint16_t elevon_right_pulse_us;float yaw_damp";

    /// Message name
    pub const NAME: &'static str = "controller";

    /// Format definition message (header + `FORMAT`), built at compile time
    pub(crate) const FORMAT_MSG: &'static [u8] =
        &format_msg::<{ Self::FORMAT.len() + MESSAGE_HEADER_SIZE }>(Self::FORMAT);

    /// Size of the message in bytes
    pub(crate) const SIZE: usize = 57;

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
        buf[53..57].copy_from_slice(&self.yaw_damp.to_le_bytes());
        buf
    }
}

const _: () = assert!(
    ControllerMessage::SIZE == 8 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 4 + 1 + 2 + 2 + 4,
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

/// Raw IMU capture (`imu-raw-log` builds): 10 consecutive ICM-42686 samples
/// as the 20-bit FIFO integers, 24-bit little-endian, gyro x/y/z then accel
/// x/y/z per sample (`elle_control::imu_raw`). Timestamp = when the first
/// sample was read. With `imu_raw_mag` and `imu_raw_ctx`, a replay reproduces
/// the firmware's attitude exactly.
///
/// Format: "imu_raw:uint64_t timestamp;uint32_t first_index;uint8_t count;int16_t temp_centi_c;uint8_t[180] data"
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct ImuRawMessage {
    pub timestamp: u64,
    pub first_index: u32,
    pub count: u8,
    pub temp_centi_c: i16,
    pub data: [u8; 180],
}

impl ImuRawMessage {
    /// Format definition string for ULog
    pub(crate) const FORMAT: &'static str = "imu_raw:uint64_t timestamp;uint32_t first_index;uint8_t count;int16_t temp_centi_c;uint8_t[180] data";

    /// Message name
    pub const NAME: &'static str = "imu_raw";

    /// Format definition message (header + `FORMAT`), built at compile time
    pub(crate) const FORMAT_MSG: &'static [u8] =
        &format_msg::<{ Self::FORMAT.len() + MESSAGE_HEADER_SIZE }>(Self::FORMAT);

    /// Size of the message in bytes
    pub(crate) const SIZE: usize = 8 + 4 + 1 + 2 + 180;

    /// Serialize to little-endian bytes
    #[must_use]
    pub(crate) fn to_bytes(self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0..8].copy_from_slice(&self.timestamp.to_le_bytes());
        buf[8..12].copy_from_slice(&self.first_index.to_le_bytes());
        buf[12] = self.count;
        buf[13..15].copy_from_slice(&self.temp_centi_c.to_le_bytes());
        buf[15..].copy_from_slice(&self.data);
        buf
    }
}

/// The mag vector fed to the AHRS changed (`imu-raw-log` builds): airframe
/// frame, offset-corrected, as fed; `valid` 0 = 6-DOF from `index` on.
///
/// Format: "imu_raw_mag:uint64_t timestamp;uint32_t index;uint8_t valid;float[3] mag"
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct ImuRawMagMessage {
    pub timestamp: u64,
    pub index: u32,
    pub valid: u8,
    pub mag: [f32; 3],
}

impl ImuRawMagMessage {
    /// Format definition string for ULog
    pub(crate) const FORMAT: &'static str =
        "imu_raw_mag:uint64_t timestamp;uint32_t index;uint8_t valid;float[3] mag";

    /// Message name
    pub const NAME: &'static str = "imu_raw_mag";

    /// Format definition message (header + `FORMAT`), built at compile time
    pub(crate) const FORMAT_MSG: &'static [u8] =
        &format_msg::<{ Self::FORMAT.len() + MESSAGE_HEADER_SIZE }>(Self::FORMAT);

    /// Size of the message in bytes
    pub(crate) const SIZE: usize = 8 + 4 + 1 + 12;

    /// Serialize to little-endian bytes
    #[must_use]
    pub(crate) fn to_bytes(self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0..8].copy_from_slice(&self.timestamp.to_le_bytes());
        buf[8..12].copy_from_slice(&self.index.to_le_bytes());
        buf[12] = self.valid;
        for (i, v) in self.mag.iter().enumerate() {
            buf[13 + 4 * i..17 + 4 * i].copy_from_slice(&v.to_le_bytes());
        }
        buf
    }
}

/// Attitude pipeline state entering sample `index` (`imu-raw-log` builds):
/// AHRS quaternion, and the gyro bias and level-cal mount that sample was fused
/// with. Every 1000 samples and on any change; a replay starts or resumes here.
/// `roundtrip_errors` > 0 means the logged integers do not reproduce the
/// driver's floats, and the replay cannot be exact.
///
/// `aid_state` is the turn compensation state beyond the quaternion
/// (`elle_control::attitude::AidState`); `turn_comp` and `gate_g` are the
/// recording build's modes (`AhrsTurnComp as u8`; gate in g, NaN for none).
///
/// Format: "imu_raw_ctx:uint64_t timestamp;uint32_t index;float[4] quat;float[3] gyro_bias;float[4] mount;uint32_t roundtrip_errors;float[5] aid_state;uint8_t turn_comp;float gate_g"
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct ImuRawCtxMessage {
    pub timestamp: u64,
    pub index: u32,
    /// w, x, y, z
    pub quat: [f32; 4],
    pub gyro_bias: [f32; 3],
    /// w, x, y, z
    pub mount: [f32; 4],
    pub roundtrip_errors: u32,
    pub aid_state: [f32; 5],
    pub turn_comp: u8,
    pub gate_g: f32,
}

impl ImuRawCtxMessage {
    /// Format definition string for ULog
    pub(crate) const FORMAT: &'static str = "imu_raw_ctx:uint64_t timestamp;uint32_t index;float[4] quat;float[3] gyro_bias;float[4] mount;uint32_t roundtrip_errors;float[5] aid_state;uint8_t turn_comp;float gate_g";

    /// Message name
    pub const NAME: &'static str = "imu_raw_ctx";

    /// Format definition message (header + `FORMAT`), built at compile time
    pub(crate) const FORMAT_MSG: &'static [u8] =
        &format_msg::<{ Self::FORMAT.len() + MESSAGE_HEADER_SIZE }>(Self::FORMAT);

    /// Size of the message in bytes
    pub(crate) const SIZE: usize = 8 + 4 + 16 + 12 + 16 + 4 + 20 + 1 + 4;

    /// Serialize to little-endian bytes
    #[must_use]
    pub(crate) fn to_bytes(self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0..8].copy_from_slice(&self.timestamp.to_le_bytes());
        buf[8..12].copy_from_slice(&self.index.to_le_bytes());
        let floats = self
            .quat
            .iter()
            .chain(self.gyro_bias.iter())
            .chain(self.mount.iter());
        for (i, v) in floats.enumerate() {
            buf[12 + 4 * i..16 + 4 * i].copy_from_slice(&v.to_le_bytes());
        }
        buf[56..60].copy_from_slice(&self.roundtrip_errors.to_le_bytes());
        for (i, v) in self.aid_state.iter().enumerate() {
            buf[60 + 4 * i..64 + 4 * i].copy_from_slice(&v.to_le_bytes());
        }
        buf[80] = self.turn_comp;
        buf[81..85].copy_from_slice(&self.gate_g.to_le_bytes());
        buf
    }
}

/// A GNSS fix handed to the attitude pipeline (`imu-raw-log` builds with turn
/// compensation): receive time, the first sample `index` it applied to, NED
/// velocity, whether from NAV-PVT. A replay applies it at that index.
///
/// Format: "imu_raw_fix:uint64_t timestamp;uint32_t index;float[3] vel_ned;uint8_t pvt"
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct ImuRawFixMessage {
    pub timestamp: u64,
    pub index: u32,
    pub vel_ned: [f32; 3],
    pub pvt: u8,
}

impl ImuRawFixMessage {
    /// Format definition string for ULog
    pub(crate) const FORMAT: &'static str =
        "imu_raw_fix:uint64_t timestamp;uint32_t index;float[3] vel_ned;uint8_t pvt";

    /// Message name
    pub const NAME: &'static str = "imu_raw_fix";

    /// Format definition message (header + `FORMAT`), built at compile time
    pub(crate) const FORMAT_MSG: &'static [u8] =
        &format_msg::<{ Self::FORMAT.len() + MESSAGE_HEADER_SIZE }>(Self::FORMAT);

    /// Size of the message in bytes
    pub(crate) const SIZE: usize = 8 + 4 + 12 + 1;

    /// Serialize to little-endian bytes
    #[must_use]
    pub(crate) fn to_bytes(self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0..8].copy_from_slice(&self.timestamp.to_le_bytes());
        buf[8..12].copy_from_slice(&self.index.to_le_bytes());
        for (i, v) in self.vel_ned.iter().enumerate() {
            buf[12 + 4 * i..16 + 4 * i].copy_from_slice(&v.to_le_bytes());
        }
        buf[24] = self.pvt;
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

/// GNSS data message - one per new solution
///
/// The timestamp is when the solution was received, not when it was logged.
/// Position, velocity and accuracy come from UBX-NAV-PVT (`pvt_active` = 1).
/// On the NMEA GGA fallback only the position fields and `hdop` are
/// meaningful; the velocity and accuracy fields are NaN.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct GnssMessage {
    /// Timestamp in microseconds
    timestamp: u64,
    /// Latitude, degrees × 10⁷
    lat_e7: i32,
    /// Longitude, degrees × 10⁷
    lon_e7: i32,
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
    /// 1 when the solution came from NAV-PVT, 0 on the GGA fallback
    pvt_active: u8,
}

impl GnssMessage {
    /// Format definition string for ULog
    pub(crate) const FORMAT: &'static str = "gnss_data:uint64_t timestamp;int32_t lat_e7;int32_t lon_e7;float altitude_m;uint8_t fix_quality;uint8_t num_satellites;float hdop;float vel_n_ms;float vel_e_ms;float vel_d_ms;float ground_speed_ms;float heading_motion_deg;float h_acc_m;float v_acc_m;float s_acc_ms;uint8_t pvt_active";

    /// Message name
    pub const NAME: &'static str = "gnss_data";

    /// Format definition message (header + `FORMAT`), built at compile time
    pub(crate) const FORMAT_MSG: &'static [u8] =
        &format_msg::<{ Self::FORMAT.len() + MESSAGE_HEADER_SIZE }>(Self::FORMAT);

    /// Size of the message in bytes
    pub(crate) const SIZE: usize = 59; // 8 + 2*4 + 4 + 1 + 1 + 4 + 8*4 + 1

    /// Create a new GNSS message
    #[must_use]
    #[allow(clippy::too_many_arguments)]
    pub const fn new(
        timestamp: Instant,
        lat_e7: i32,
        lon_e7: i32,
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
        pvt_active: bool,
    ) -> Self {
        Self {
            timestamp: timestamp.as_micros(),
            lat_e7,
            lon_e7,
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
            pvt_active: pvt_active as u8,
        }
    }

    /// Serialize to little-endian bytes
    #[must_use]
    pub(crate) fn to_bytes(self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0..8].copy_from_slice(&self.timestamp.to_le_bytes());
        buf[8..12].copy_from_slice(&self.lat_e7.to_le_bytes());
        buf[12..16].copy_from_slice(&self.lon_e7.to_le_bytes());
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
        buf[58] = self.pvt_active;
        buf
    }
}

// 8 (u64) + 2 i32 + 1 f32 + 2 u8 + 1 f32 + 8 f32 + 1 u8
const _: () = assert!(
    GnssMessage::SIZE == 8 + 2 * 4 + 4 + 1 + 1 + 4 + 8 * 4 + 1,
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
/// `reconfigs` counts configurations re-sent after an ESC (re)appeared or sent
/// no EDT; `edt_frames` staying at 0 while `replies` climb means EDT is off.
///
/// Format: "esc_health:uint64_t timestamp;uint32_t left_replies;uint32_t right_replies;uint32_t left_timeouts;uint32_t right_timeouts;uint32_t left_bad_frames;uint32_t right_bad_frames;uint16_t left_reconfigs;uint16_t right_reconfigs;uint32_t left_edt_frames;uint32_t right_edt_frames"
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
    pub left_edt_frames: u32,
    pub right_edt_frames: u32,
}

impl EscHealthMessage {
    /// Format definition string for ULog
    pub(crate) const FORMAT: &'static str = "esc_health:uint64_t timestamp;uint32_t left_replies;uint32_t right_replies;uint32_t left_timeouts;uint32_t right_timeouts;uint32_t left_bad_frames;uint32_t right_bad_frames;uint16_t left_reconfigs;uint16_t right_reconfigs;uint32_t left_edt_frames;uint32_t right_edt_frames";

    /// Message name
    pub const NAME: &'static str = "esc_health";

    /// Format definition message (header + `FORMAT`), built at compile time
    pub(crate) const FORMAT_MSG: &'static [u8] =
        &format_msg::<{ Self::FORMAT.len() + MESSAGE_HEADER_SIZE }>(Self::FORMAT);

    /// Size of the message in bytes
    pub(crate) const SIZE: usize = 44;

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
        buf[36..40].copy_from_slice(&self.left_edt_frames.to_le_bytes());
        buf[40..44].copy_from_slice(&self.right_edt_frames.to_le_bytes());
        buf
    }
}

const _: () = assert!(
    EscHealthMessage::SIZE == 8 + 6 * 4 + 2 * 2 + 2 * 4,
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

/// Flight-loop tick split into stages, over a window of `ULOG_STATUS_DIVISOR` ticks (8 Hz).
///
/// Per stage, mean and max µs across the window's ticks (wall time, so
/// preemption included). Stages in order: intake, update, outputs, switches,
/// autotune, log, tail. `dshot_busy_us` / `dshot_runs` are the DShot executor
/// interrupt's total time and run count on Core 0 over the same window.
///
/// Format: "loop_stages:uint64_t timestamp;uint16_t ticks;uint16_t[7] avg_us;uint16_t[7] max_us;uint32_t dshot_busy_us;uint32_t dshot_runs"
#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct LoopStagesMessage {
    /// Timestamp in microseconds (filled in by the logger)
    pub timestamp: u64,
    pub ticks: u16,
    pub avg_us: [u16; 7],
    pub max_us: [u16; 7],
    pub dshot_busy_us: u32,
    pub dshot_runs: u32,
}

impl LoopStagesMessage {
    /// Format definition string for ULog
    pub(crate) const FORMAT: &'static str = "loop_stages:uint64_t timestamp;uint16_t ticks;uint16_t[7] avg_us;uint16_t[7] max_us;uint32_t dshot_busy_us;uint32_t dshot_runs";

    /// Message name
    pub const NAME: &'static str = "loop_stages";

    /// Format definition message (header + `FORMAT`), built at compile time
    pub(crate) const FORMAT_MSG: &'static [u8] =
        &format_msg::<{ Self::FORMAT.len() + MESSAGE_HEADER_SIZE }>(Self::FORMAT);

    /// Size of the message in bytes
    pub(crate) const SIZE: usize = 46;

    /// Serialize to little-endian bytes
    #[must_use]
    pub(crate) fn to_bytes(self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0..8].copy_from_slice(&self.timestamp.to_le_bytes());
        buf[8..10].copy_from_slice(&self.ticks.to_le_bytes());
        for (i, v) in self.avg_us.iter().enumerate() {
            buf[10 + 2 * i..12 + 2 * i].copy_from_slice(&v.to_le_bytes());
        }
        for (i, v) in self.max_us.iter().enumerate() {
            buf[24 + 2 * i..26 + 2 * i].copy_from_slice(&v.to_le_bytes());
        }
        buf[38..42].copy_from_slice(&self.dshot_busy_us.to_le_bytes());
        buf[42..46].copy_from_slice(&self.dshot_runs.to_le_bytes());
        buf
    }
}

const _: () = assert!(
    LoopStagesMessage::SIZE == 8 + 2 + 7 * 2 + 7 * 2 + 4 + 4,
    "LoopStagesMessage::SIZE does not match its field widths"
);

/// Navigator output (observation mode), 25 Hz.
///
/// Position and velocity are in the home north/east frame; `status` holds the
/// `elle_nav::status` bits that say which fields are valid. Invalid values are
/// NaN. `bank_demand_deg` is what lateral guidance asks for on the reference
/// path (a loiter around home); it is not applied. `roll_deg` is the measured
/// roll at the same instant, positive right like the demand, for comparison.
///
/// Format: "nav:uint64_t timestamp;uint16_t status;uint16_t fix_age_ms;float pos_n_m;float pos_e_m;float vel_n_ms;float vel_e_ms;float alt_rel_m;float climb_ms;float gnss_alt_rel_m;float home_dist_m;float home_bearing_deg;float track_error_m;float lat_accel_ms2;float bank_demand_deg;float roll_deg"
#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct NavMessage {
    /// Timestamp in microseconds (filled in by the logger)
    pub timestamp: u64,
    pub status: u16,
    /// Age of the fix behind the position, ms (saturates at 65535)
    pub fix_age_ms: u16,
    pub pos_n_m: f32,
    pub pos_e_m: f32,
    pub vel_n_ms: f32,
    pub vel_e_ms: f32,
    /// Baro height above home
    pub alt_rel_m: f32,
    pub climb_ms: f32,
    /// GNSS height above home
    pub gnss_alt_rel_m: f32,
    pub home_dist_m: f32,
    /// Bearing to home, degrees clockwise from north
    pub home_bearing_deg: f32,
    /// Off the path: positive right of a line, positive outside a circle
    pub track_error_m: f32,
    pub lat_accel_ms2: f32,
    pub bank_demand_deg: f32,
    pub roll_deg: f32,
}

impl NavMessage {
    /// Format definition string for ULog
    pub(crate) const FORMAT: &'static str = "nav:uint64_t timestamp;uint16_t status;uint16_t fix_age_ms;float pos_n_m;float pos_e_m;float vel_n_ms;float vel_e_ms;float alt_rel_m;float climb_ms;float gnss_alt_rel_m;float home_dist_m;float home_bearing_deg;float track_error_m;float lat_accel_ms2;float bank_demand_deg;float roll_deg";

    /// Message name
    pub const NAME: &'static str = "nav";

    /// Format definition message (header + `FORMAT`), built at compile time
    pub(crate) const FORMAT_MSG: &'static [u8] =
        &format_msg::<{ Self::FORMAT.len() + MESSAGE_HEADER_SIZE }>(Self::FORMAT);

    /// Size of the message in bytes
    pub(crate) const SIZE: usize = 64;

    /// Serialize to little-endian bytes
    #[must_use]
    pub(crate) fn to_bytes(self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0..8].copy_from_slice(&self.timestamp.to_le_bytes());
        buf[8..10].copy_from_slice(&self.status.to_le_bytes());
        buf[10..12].copy_from_slice(&self.fix_age_ms.to_le_bytes());
        let floats = [
            self.pos_n_m,
            self.pos_e_m,
            self.vel_n_ms,
            self.vel_e_ms,
            self.alt_rel_m,
            self.climb_ms,
            self.gnss_alt_rel_m,
            self.home_dist_m,
            self.home_bearing_deg,
            self.track_error_m,
            self.lat_accel_ms2,
            self.bank_demand_deg,
            self.roll_deg,
        ];
        for (i, v) in floats.iter().enumerate() {
            buf[12 + 4 * i..16 + 4 * i].copy_from_slice(&v.to_le_bytes());
        }
        buf
    }
}

const _: () = assert!(
    NavMessage::SIZE == 8 + 2 + 2 + 13 * 4,
    "NavMessage::SIZE does not match its field widths"
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
