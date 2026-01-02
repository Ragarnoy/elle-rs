//! Elle RPC Interface Control Document
//!
//! Shared types and endpoint definitions for postcard-RPC communication
//! between the flight controller and host tools.

#![cfg_attr(not(feature = "use-std"), no_std)]

use postcard_rpc::{endpoint, topic};
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

// ============================================================================
// Wire Types - Topics (streaming data)
// ============================================================================

/// Telemetry message for periodic updates
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Schema)]
pub struct TelemetryMsg {
    pub timestamp_ms: u32,
    pub pitch_cdeg: i16,
    pub roll_cdeg: i16,
    pub throttle_percent: u8,
    pub armed: bool,
}

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

// Control endpoints
endpoint!(SetThrottleEndpoint, SetThrottleReq, AckResp, "elle/ctrl/throttle");
endpoint!(SetElevonsEndpoint, SetElevonsReq, AckResp, "elle/ctrl/elevons");
endpoint!(SetControlModeEndpoint, SetControlModeReq, AckResp, "elle/ctrl/mode");

// Safety endpoints
endpoint!(ArmEndpoint, (), AckResp, "elle/safety/arm");
endpoint!(DisarmEndpoint, (), AckResp, "elle/safety/disarm");
endpoint!(EmergencyStopEndpoint, (), AckResp, "elle/safety/estop");

// Trim/Calibration endpoints
endpoint!(AdjustTrimEndpoint, AdjustTrimReq, AckResp, "elle/trim/adjust");
endpoint!(SaveCalibrationEndpoint, (), AckResp, "elle/cal/save");
endpoint!(ClearCalibrationEndpoint, (), AckResp, "elle/cal/clear");

// Query endpoints
endpoint!(GetStatusEndpoint, (), StatusResp, "elle/query/status");
endpoint!(GetAttitudeEndpoint, (), AttitudeResp, "elle/query/attitude");
endpoint!(GetPerformanceEndpoint, (), PerformanceResp, "elle/query/perf");
endpoint!(ResetPerformanceEndpoint, (), AckResp, "elle/perf/reset");

// System endpoints
endpoint!(PingEndpoint, (), (), "elle/sys/ping");
endpoint!(GetVersionEndpoint, (), VersionResp, "elle/sys/version");

// ============================================================================
// Topic Definitions
// ============================================================================

// Device -> Host topics
topic!(TelemetryTopic, TelemetryMsg, "elle/telem");
topic!(LogTopic, LogMsg, "elle/log");
