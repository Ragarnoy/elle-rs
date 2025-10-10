use crate::config::*;
use defmt::Format;
use embassy_time::{Duration, Instant};

// Import LUT functions and channel indices for decoding
use crate::config::{
    ATTITUDE_ENABLE_CH, ATTITUDE_PITCH_SETPOINT_CH, ATTITUDE_ROLL_SETPOINT_CH, PITCH_CH, ROLL_CH,
    THROTTLE_CH, YAW_CH, sbus_to_normalized_pitch_lut, sbus_to_normalized_roll_lut,
    sbus_to_normalized_yaw_lut,
};

/// Commands can be raw (fast path) or normalized (semantic)
#[derive(Clone, Copy, Debug, Format)]
pub enum PilotCommands {
    /// Raw SBUS-like values (0-2047)
    Raw(RawCommands),
    /// Normalized semantic commands (-1.0 to 1.0)
    Normalized(NormalizedCommands),
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Format)]
pub struct RawCommands {
    pub channels: [u16; 16],
    pub timestamp: Instant,
}

impl RawCommands {
    /// Convert raw SBUS channels to normalized commands
    pub fn to_normalized(&self) -> NormalizedCommands {
        NormalizedCommands {
            throttle: (self.channels[THROTTLE_CH] as f32 / 2047.0).clamp(0.0, 1.0),
            pitch: sbus_to_normalized_pitch_lut(self.channels[PITCH_CH]),
            roll: sbus_to_normalized_roll_lut(self.channels[ROLL_CH]),
            yaw: sbus_to_normalized_yaw_lut(self.channels[YAW_CH]),
            attitude_mode: decode_attitude_mode(self.channels[ATTITUDE_ENABLE_CH]),
            pitch_setpoint_deg: decode_pitch_setpoint(self.channels[ATTITUDE_PITCH_SETPOINT_CH]),
            roll_setpoint_deg: decode_roll_setpoint(self.channels[ATTITUDE_ROLL_SETPOINT_CH]),
            timestamp: self.timestamp,
        }
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Format)]
pub struct NormalizedCommands {
    pub throttle: f32, // 0.0 to 1.0
    pub pitch: f32,    // -1.0 to 1.0
    pub roll: f32,     // -1.0 to 1.0
    pub yaw: f32,      // -1.0 to 1.0
    pub attitude_mode: AttitudeMode,
    pub pitch_setpoint_deg: f32,
    pub roll_setpoint_deg: f32,
    pub timestamp: Instant,
}

#[derive(Clone, Copy, Debug, Format, PartialEq)]
pub enum AttitudeMode {
    Manual,
    Mixed,
    Autopilot,
}

impl PilotCommands {
    #[inline]
    pub fn timestamp(&self) -> Instant {
        match self {
            Self::Raw(r) => r.timestamp,
            Self::Normalized(n) => n.timestamp,
        }
    }

    #[inline]
    pub fn is_fresh(&self, max_age: Duration) -> bool {
        self.timestamp().elapsed() < max_age
    }

    #[inline]
    pub fn attitude_mode(&self) -> AttitudeMode {
        match self {
            Self::Raw(r) => decode_attitude_mode(r.channels[ATTITUDE_ENABLE_CH]),
            Self::Normalized(n) => n.attitude_mode,
        }
    }
}

impl NormalizedCommands {
    pub const fn neutral() -> Self {
        Self {
            throttle: 0.0,
            pitch: 0.0,
            roll: 0.0,
            yaw: 0.0,
            attitude_mode: AttitudeMode::Manual,
            pitch_setpoint_deg: 0.0,
            roll_setpoint_deg: 0.0,
            timestamp: Instant::from_ticks(0),
        }
    }
}

// Helper functions
pub fn decode_attitude_mode(ch5_value: u16) -> AttitudeMode {
    if ch5_value < MANUAL_MODE_THRESHOLD {
        AttitudeMode::Manual
    } else if ch5_value < MIXED_MODE_THRESHOLD {
        AttitudeMode::Mixed
    } else {
        AttitudeMode::Autopilot
    }
}

pub fn decode_pitch_setpoint(ch6_value: u16) -> f32 {
    let normalized = sbus_to_normalized_pitch_lut(ch6_value);
    ATTITUDE_PITCH_MIN_DEG
        + (normalized + 1.0) * 0.5 * (ATTITUDE_PITCH_MAX_DEG - ATTITUDE_PITCH_MIN_DEG)
}

pub fn decode_roll_setpoint(ch8_value: u16) -> f32 {
    let normalized = sbus_to_normalized_roll_lut(ch8_value);
    ATTITUDE_ROLL_MIN_DEG
        + (normalized + 1.0) * 0.5 * (ATTITUDE_ROLL_MAX_DEG - ATTITUDE_ROLL_MIN_DEG)
}
