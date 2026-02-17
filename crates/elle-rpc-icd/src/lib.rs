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
    Mixed = 1,
    Autopilot = 2,
}

/// Set control mode
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
pub struct SetControlModeReq {
    pub mode: ControlMode,
}

/// Adjust trim values (-100 to 100 for each)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
pub struct AdjustTrimReq {
    pub left: i8,
    pub right: i8,
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
    pub const fn ok() -> Self {
        Self {
            success: true,
            error_code: 0,
        }
    }

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
}

/// ULog read chunk response
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Schema)]
pub struct ULogReadResp {
    /// Data payload (variable length, up to 512 bytes)
    pub data: heapless::Vec<u8, 512>,
    /// More data available after this chunk
    pub has_more: bool,
    /// Not ready yet, host should retry
    pub pending: bool,
}

/// ULog storage info response
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
pub struct ULogInfoResp {
    /// Whether ULog recording is currently active
    pub recording: bool,
    /// Total ULog flash region size in bytes
    pub region_total: u32,
    /// Approximate bytes currently stored in queue
    pub bytes_used: u32,
    /// Number of items currently in queue
    pub items_stored: u32,
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
    /// Horizontal dilution of precision
    pub hdop: f32,
}

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
    | AdjustTrimEndpoint        | AdjustTrimReq     | AckResp           | "elle/trim/adjust"    |
    | SaveCalibrationEndpoint   | ()                | AckResp           | "elle/cal/save"       |
    | ClearCalibrationEndpoint  | ()                | AckResp           | "elle/cal/clear"      |
    | GetStatusEndpoint         | ()                | StatusResp        | "elle/query/status"   |
    | GetAttitudeEndpoint       | ()                | AttitudeResp      | "elle/query/attitude" |
    | GetPerformanceEndpoint    | ()                | PerformanceResp   | "elle/query/perf"     |
    | ResetPerformanceEndpoint  | ()                | AckResp           | "elle/perf/reset"     |
    | PingEndpoint              | ()                | ()                | "elle/sys/ping"       |
    | GetVersionEndpoint        | ()                | VersionResp       | "elle/sys/version"    |
    | GetMagnetometerEndpoint   | ()                | MagnetometerResp  | "elle/query/mag"      |
    | GetBarometerEndpoint      | ()                | BarometerResp     | "elle/query/baro"     |
    | GetGnssEndpoint           | ()                | GnssResp          | "elle/query/gnss"     |
    | GetRcChannelsEndpoint     | ()                | RcChannelsResp    | "elle/query/rc"       |
    | StartULogEndpoint         | ()                | AckResp           | "elle/ulog/start"     |
    | StopULogEndpoint          | ()                | AckResp           | "elle/ulog/stop"      |
    | ReadULogChunkEndpoint     | ()                | ULogReadResp      | "elle/ulog/read"      |
    | EraseULogEndpoint         | ()                | AckResp           | "elle/ulog/erase"     |
    | GetULogInfoEndpoint       | ()                | ULogInfoResp      | "elle/ulog/info"      |
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
