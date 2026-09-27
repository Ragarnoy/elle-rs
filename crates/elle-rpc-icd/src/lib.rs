//! Elle RPC Interface Control Document
//!
//! Shared types and endpoint definitions for postcard-RPC communication
//! between the flight controller and host tools.

#![cfg_attr(not(feature = "use-std"), no_std)]

use postcard_rpc::{TopicDirection, endpoints, topics};
use postcard_schema::Schema;
use serde::{Deserialize, Serialize};

// ============================================================================
// Wire Types - Requests
// ============================================================================

/// Set throttle percentage (0-100)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
pub struct SetThrottleReq {
    pub percent: u8,
}

/// Set elevon positions (-100 to 100 for each)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
pub struct SetElevonsReq {
    pub left: i8,
    pub right: i8,
}

/// Control mode selection
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
#[repr(u8)]
pub enum ControlMode {
    Manual = 0,
    Stabilized = 1,
    AltitudeHold = 2,
}

/// Set control mode
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
pub struct SetControlModeReq {
    pub mode: ControlMode,
}

/// Engage/disengage heading-hold (modifier active only in Stabilized mode).
/// `heading_cdeg` is only used when `enabled` is true.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
pub struct SetHeadingHoldReq {
    pub enabled: bool,
    /// Target heading in centidegrees (degrees * 100)
    pub heading_cdeg: i16,
}

// ============================================================================
// Wire Types - Responses
// ============================================================================

/// Generic acknowledgement response
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
pub struct AckResp {
    pub success: bool,
    pub error_code: u8,
}

impl AckResp {
    #[must_use]
    pub const fn ok() -> Self {
        Self {
            success: true,
            error_code: 0,
        }
    }

    #[must_use]
    pub const fn error(code: u8) -> Self {
        Self {
            success: false,
            error_code: code,
        }
    }
}

/// System status response
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
pub struct StatusResp {
    pub armed: bool,
    pub failsafe: bool,
    pub mode: ControlMode,
    pub imu_calibrated: bool,
    pub imu_error_count: u32,
    pub rc_age_ms: u16,
    /// Autotune state: 0=off, 1=pitch, 2=roll, 3=done, 4=error
    pub autotune_state: u8,
}

/// Attitude data response (scaled integers for efficiency)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
pub struct AttitudeResp {
    /// Pitch angle in centidegrees (degrees * 100)
    pub pitch_cdeg: i16,
    /// Roll angle in centidegrees
    pub roll_cdeg: i16,
    /// Yaw angle in centidegrees
    pub yaw_cdeg: i16,
    /// Pitch rate in centidegrees/second
    pub pitch_rate_cdeg: i16,
    /// Roll rate in centidegrees/second
    pub roll_rate_cdeg: i16,
    /// Yaw rate in centidegrees/second
    pub yaw_rate_cdeg: i16,
}

/// Performance statistics response
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
pub struct PerformanceResp {
    pub control_loop_avg_us: u32,
    pub control_loop_max_us: u32,
    pub imu_avg_us: u32,
    pub imu_max_us: u32,
}

/// Version information response
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
pub struct VersionResp {
    pub major: u8,
    pub minor: u8,
    pub patch: u8,
}

/// Magnetometer data response (signed counts from MMC5616WA)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
pub struct MagnetometerResp {
    pub x: i32,
    pub y: i32,
    pub z: i32,
}

/// RC channel data response (16 channels, 0–2047 range)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
pub struct RcChannelsResp {
    pub channels: [u16; 16],
}

/// Barometer data response (BMP390)
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Schema)]
pub struct BarometerResp {
    /// Pressure in hectopascals
    pub pressure_hpa: f32,
    /// Temperature in degrees Celsius
    pub temperature_c: f32,
    /// Barometric altitude in meters
    pub altitude_m: f32,
    /// Vertical speed in m/s (positive = climbing)
    pub vario_ms: f32,
}

/// Per-engine telemetry (DShot bidirectional RPM + throttle + governor + EDT)
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
pub struct EngineUnit {
    pub erpm: u32,
    pub throttle: u16,
    pub valid: bool,
    pub target_erpm: u32,
    /// ESC temperature in °C (from EDT)
    pub temperature: u8,
    /// Supply voltage in millivolts (from EDT)
    pub voltage_mv: u32,
    /// Current draw in milliamps (from EDT)
    pub current_ma: u32,
}

/// Engine telemetry response (left + right engines)
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
pub struct EngineResp {
    pub left: EngineUnit,
    pub right: EngineUnit,
}

/// Magnetometer calibration status response
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Schema)]
pub struct MagCalResp {
    pub offset_x: f32,
    pub offset_y: f32,
    pub offset_z: f32,
    pub calibrated: bool,
    pub collecting: bool,
    pub samples: u16,
}

/// Level calibration (IMU mounting offset) status response.
/// Angles use the attitude telemetry's sign convention: what the uncorrected
/// attitude reads with the airframe level ("pitch -2.1" = board 2.1° nose-down).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Schema)]
pub struct LevelCalResp {
    pub roll_deg: f32,
    pub pitch_deg: f32,
    pub calibrated: bool,
    pub collecting: bool,
}

/// `ULog` read chunk response
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Schema)]
pub struct ULogReadResp {
    /// Data payload (variable length, up to 512 bytes)
    pub data: heapless::Vec<u8, 512>,
    /// More data available after this chunk
    pub has_more: bool,
    /// Not ready yet, host should retry
    pub pending: bool,
}

/// `ULog` storage info response
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
pub struct ULogInfoResp {
    /// Whether `ULog` recording is currently active
    pub recording: bool,
    /// Total `ULog` flash region size in bytes
    pub region_total: u32,
    /// Approximate bytes currently stored in queue
    pub bytes_used: u32,
    /// Number of items currently in queue
    pub items_stored: u32,
}

/// Controller output snapshot (integer-scaled to avoid f32 on wire)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
pub struct ControllerOutputResp {
    /// PID pitch correction * 10000
    pub pitch_correction_cp: i16,
    /// PID roll correction * 10000
    pub roll_correction_cp: i16,
    /// Pitch setpoint in centidegrees
    pub pitch_setpoint_cdeg: i16,
    /// Roll setpoint in centidegrees
    pub roll_setpoint_cdeg: i16,
    /// Left elevon servo PWM microseconds
    pub elevon_left_us: u16,
    /// Right elevon servo PWM microseconds
    pub elevon_right_us: u16,
    /// Left engine DShot throttle (0-1999)
    pub engine_left_dshot: u16,
    /// Right engine DShot throttle (0-1999)
    pub engine_right_dshot: u16,
    /// True while the heading-hold modifier is engaged
    pub heading_hold_active: bool,
    /// Locked heading-hold target in centidegrees (valid only when active)
    pub heading_target_cdeg: i16,
    /// Heading error (shortest path) in centidegrees (valid only when active)
    pub heading_error_cdeg: i16,
}

/// Set PID gains (integer-scaled: x1000 for gains, x10000 for scale, x10 for i_limit)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
pub struct SetPidGainsReq {
    pub pitch_kp_x1000: i16,
    pub pitch_ki_x1000: i16,
    pub pitch_kd_x1000: i16,
    pub roll_kp_x1000: i16,
    pub roll_ki_x1000: i16,
    pub roll_kd_x1000: i16,
    /// Scale * 10000 (e.g. 150 = 0.0150)
    pub scale_x10000: i16,
    /// I-limit * 10 (e.g. 250 = 25.0)
    pub i_limit_x10: i16,
}

/// `StartAutotuneReq::axis`: tune pitch.
pub const AUTOTUNE_AXIS_PITCH: u8 = 0;
/// `StartAutotuneReq::axis`: tune roll.
pub const AUTOTUNE_AXIS_ROLL: u8 = 1;
/// `StartAutotuneReq::axis`: save the current PID gains to flash (no tuning).
pub const AUTOTUNE_AXIS_SAVE_PID: u8 = 0xFF;
/// `StartAutotuneReq::axis`: erase the saved PID profile from flash (no tuning).
pub const AUTOTUNE_AXIS_ERASE_PID: u8 = 0xFE;
/// `AckResp::error_code` for a `StartAutotuneReq` with an unknown axis.
pub const AUTOTUNE_ERR_BAD_AXIS: u8 = 2;

/// Start autotune on a specific axis
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
pub struct StartAutotuneReq {
    /// One of the `AUTOTUNE_AXIS_*` values. Anything else is rejected
    /// (`AckResp` error code [`AUTOTUNE_ERR_BAD_AXIS`]).
    pub axis: u8,
    /// Relay amplitude * 10 (e.g. 50 = 5.0°)
    pub relay_deg_x10: u8,
    /// Number of measurable cycles to collect
    pub num_cycles: u8,
    /// Tuning rule: 0=TyreusLuyben, 1=ZieglerNichols, 2=SomeOvershoot
    pub rule: u8,
}

/// GNSS position fix response
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Schema)]
pub struct GnssResp {
    /// Latitude in degrees (positive = North)
    pub latitude: f32,
    /// Longitude in degrees (positive = East)
    pub longitude: f32,
    /// Altitude above mean sea level in meters
    pub altitude_m: f32,
    /// Fix quality: 0=none, 1=GPS, 2=DGPS
    pub fix_quality: u8,
    /// Number of satellites used for fix
    pub num_satellites: u8,
    /// Horizontal dilution of precision.
    ///
    /// Only meaningful on the NMEA GGA fallback path — NAV-PVT supplies
    /// `h_acc_m` instead, an error estimate rather than a geometry figure.
    pub hdop: f32,
    /// Velocity north in m/s (NAV-PVT only)
    pub vel_n_ms: f32,
    /// Velocity east in m/s (NAV-PVT only)
    pub vel_e_ms: f32,
    /// Velocity down in m/s (NAV-PVT only)
    pub vel_d_ms: f32,
    /// Ground speed in m/s (NAV-PVT only)
    pub ground_speed_ms: f32,
    /// Course over ground in degrees (NAV-PVT only)
    pub heading_motion_deg: f32,
    /// Horizontal accuracy estimate in meters (NAV-PVT only)
    pub h_acc_m: f32,
    /// Vertical accuracy estimate in meters (NAV-PVT only)
    pub v_acc_m: f32,
    /// Speed accuracy estimate in m/s (NAV-PVT only)
    pub s_acc_ms: f32,
    /// True while NAV-PVT is arriving; false when the GGA fallback is driving
    pub pvt_active: bool,
    /// Link speed the receiver settled on, in baud
    pub link_baud: u32,
    /// Configured solution interval, in milliseconds.
    ///
    /// Reports what the module actually accepted, not what was requested — if
    /// the rate key was rejected this is the module's own default.
    pub nav_rate_ms: u16,
    /// Bitmask of configuration keys the module acknowledged, one bit per entry
    /// of [`GNSS_CFG_KEY_NAMES`].
    ///
    /// A clear bit means the key is not in force, which is not the same as
    /// rejected: if [`GNSS_CFG_ABANDONED`] is set, configuration was given up
    /// on partway through and the remaining groups were never attempted.
    pub cfg_mask: u16,
    /// Satellites in view, summed across constellations (NMEA GSV).
    ///
    /// Unlike `num_satellites`, which counts satellites *used in the fix* and
    /// so reads zero throughout acquisition, this shows what the receiver can
    /// actually see. Zero unless the firmware was built with `gnss-gsv` and
    /// the fast link was achieved.
    pub sats_in_view: u8,
}

/// Number of GNSS configuration key groups, and so of meaningful bits in
/// [`GnssResp::cfg_mask`].
///
/// Firmware has its own `CFG_GROUP_COUNT`; the two are checked against each
/// other at compile time where the handler bridges them.
pub const GNSS_CFG_KEY_COUNT: usize = GNSS_CFG_KEY_NAMES.len();

/// Every configuration group applied — the all-clear value of
/// [`GnssResp::cfg_mask`], ignoring [`GNSS_CFG_ABANDONED`].
pub const GNSS_CFG_MASK_ALL: u16 = (1 << GNSS_CFG_KEY_COUNT) - 1;

/// Set in [`GnssResp::cfg_mask`] when the firmware stopped configuring partway
/// through because the module went silent, rather than running every group.
///
/// Groups after that point were never sent, so their clear bits say nothing
/// about whether the receiver would have accepted them.
pub const GNSS_CFG_ABANDONED: u16 = 1 << 15;

// The abandoned marker must stay clear of the per-group bits.
const _: () = assert!(GNSS_CFG_KEY_COUNT < 15);

/// Names of the configuration keys applied at GNSS boot, in the order their
/// bits appear in [`GnssResp::cfg_mask`].
pub const GNSS_CFG_KEY_NAMES: [&str; 10] = [
    "DYNMODEL+FIXMODE",
    "RATE-MEAS",
    "RATE-NAV",
    "MSGOUT-NAV-PVT",
    "MSGOUT-GGA",
    "MSGOUT-GLL",
    "MSGOUT-GSA",
    "MSGOUT-GSV",
    "MSGOUT-VTG",
    "MSGOUT-RMC",
];

// ============================================================================
// Wire Types - Topics (streaming data)
// ============================================================================

/// Log message for device-side logging
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
pub struct LogMsg {
    /// Log level: 0=trace, 1=debug, 2=info, 3=warn, 4=error
    pub level: u8,
    /// Application-defined log code
    pub code: u16,
}

// ============================================================================
// Endpoint Definitions
// ============================================================================

endpoints! {
    list = ENDPOINT_LIST;
    | EndpointTy                | RequestTy         | ResponseTy        | Path                  |
    | ----------                | ---------         | ----------        | ----                  |
    | SetThrottleEndpoint       | SetThrottleReq    | AckResp           | "elle/ctrl/throttle"  |
    | SetElevonsEndpoint        | SetElevonsReq     | AckResp           | "elle/ctrl/elevons"   |
    | SetControlModeEndpoint    | SetControlModeReq | AckResp           | "elle/ctrl/mode"      |
    | ArmEndpoint               | ()                | AckResp           | "elle/safety/arm"     |
    | DisarmEndpoint            | ()                | AckResp           | "elle/safety/disarm"  |
    | EmergencyStopEndpoint     | ()                | AckResp           | "elle/safety/estop"   |
    | GetStatusEndpoint         | ()                | StatusResp        | "elle/query/status"   |
    | GetAttitudeEndpoint       | ()                | AttitudeResp      | "elle/query/attitude" |
    | GetPerformanceEndpoint    | ()                | PerformanceResp   | "elle/query/perf"     |
    | ResetPerformanceEndpoint  | ()                | AckResp           | "elle/perf/reset"     |
    | PingEndpoint              | ()                | ()                | "elle/sys/ping"       |
    | GetVersionEndpoint        | ()                | VersionResp       | "elle/sys/version"    |
    | GetMagnetometerEndpoint   | ()                | MagnetometerResp  | "elle/query/mag"      |
    | GetBarometerEndpoint      | ()                | BarometerResp     | "elle/query/baro"     |
    | GetEngineEndpoint         | ()                | EngineResp        | "elle/query/engine"   |
    | GetGnssEndpoint           | ()                | GnssResp          | "elle/query/gnss"     |
    | GetRcChannelsEndpoint     | ()                | RcChannelsResp    | "elle/query/rc"       |
    | StartULogEndpoint         | ()                | AckResp           | "elle/ulog/start"     |
    | StopULogEndpoint          | ()                | AckResp           | "elle/ulog/stop"      |
    | ReadULogChunkEndpoint     | ()                | ULogReadResp      | "elle/ulog/read"      |
    | EraseULogEndpoint         | ()                | AckResp           | "elle/ulog/erase"     |
    | GetULogInfoEndpoint       | ()                | ULogInfoResp      | "elle/ulog/info"      |
    | GetTimeEndpoint           | ()                | u64               | "elle/sys/time"       |
    | GetControllerOutputEndpoint | ()              | ControllerOutputResp | "elle/query/ctrl_out" |
    | SetPidGainsEndpoint       | SetPidGainsReq    | AckResp           | "elle/ctrl/pid"       |
    | StartAutotuneEndpoint   | StartAutotuneReq  | AckResp           | "elle/ctrl/atune/start" |
    | AbortAutotuneEndpoint   | ()                | AckResp           | "elle/ctrl/atune/abort" |
    | StartMagCalEndpoint     | ()                | AckResp           | "elle/cal/mag/start"    |
    | ClearMagCalEndpoint     | ()                | AckResp           | "elle/cal/mag/clear"    |
    | GetMagCalEndpoint       | ()                | MagCalResp        | "elle/cal/mag/get"      |
    | StartLevelCalEndpoint   | ()                | AckResp           | "elle/cal/level/start"  |
    | ClearLevelCalEndpoint   | ()                | AckResp           | "elle/cal/level/clear"  |
    | GetLevelCalEndpoint     | ()                | LevelCalResp      | "elle/cal/level/get"    |
    | SetHeadingHoldEndpoint  | SetHeadingHoldReq | AckResp           | "elle/ctrl/heading_hold" |
}

// ============================================================================
// Topic Definitions
// ============================================================================

// Incoming topics to server (host -> device) — none
topics! {
    list = TOPICS_IN_LIST;
    direction = TopicDirection::ToServer;
    | TopicTy                   | MessageTy     | Path              |
    | -------                   | ---------     | ----              |
}

// Outgoing topics from server (device -> host)
topics! {
    list = TOPICS_OUT_LIST;
    direction = TopicDirection::ToClient;
    | TopicTy                   | MessageTy     | Path              | Cfg   |
    | -------                   | ---------     | ----              | ---   |
    | LogTopic                  | LogMsg        | "elle/log"        |       |
}
