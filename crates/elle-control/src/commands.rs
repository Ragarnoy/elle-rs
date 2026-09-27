use defmt::Format;
use elle_config::*;
use embassy_time::Instant;

// Import LUT functions and channel indices for decoding
use elle_config::{
    ATTITUDE_ENABLE_CH, DSHOT_THROTTLE_MAX, PITCH_CH, ROLL_CH, THROTTLE_CH, YAW_CH,
    rc_to_normalized, throttle_curve_lut,
};

/// Commands can be raw (fast path) or normalized (semantic)
#[derive(Clone, Copy, Debug, Format)]
pub enum PilotCommands {
    /// Raw RC values (0-2047)
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
    /// Convert raw RC channels to normalized commands
    #[must_use]
    pub fn to_normalized(&self) -> NormalizedCommands {
        NormalizedCommands {
            throttle: (throttle_curve_lut(self.channels[THROTTLE_CH]) as f32
                / DSHOT_THROTTLE_MAX as f32)
                .clamp(0.0, 1.0),
            pitch: rc_to_normalized(self.channels[PITCH_CH]) * elle_config::PITCH_INVERT,
            roll: rc_to_normalized(self.channels[ROLL_CH]) * elle_config::ROLL_INVERT,
            yaw: rc_to_normalized(self.channels[YAW_CH]) * elle_config::YAW_INVERT,
            attitude_mode: decode_attitude_mode(self.channels[ATTITUDE_ENABLE_CH]),
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
    pub timestamp: Instant,
}

#[derive(Clone, Copy, Debug, Format, PartialEq, Eq)]
pub enum AttitudeMode {
    Manual,
    Stabilized,
    AltitudeHold,
}

impl PilotCommands {
    #[must_use]
    #[inline]
    pub const fn timestamp(&self) -> Instant {
        match self {
            Self::Raw(r) => r.timestamp,
            Self::Normalized(n) => n.timestamp,
        }
    }

    #[must_use]
    #[inline]
    pub fn attitude_mode(&self) -> AttitudeMode {
        match self {
            Self::Raw(r) => decode_attitude_mode(r.channels[ATTITUDE_ENABLE_CH]),
            Self::Normalized(n) => n.attitude_mode,
        }
    }
}

impl NormalizedCommands {
    #[must_use]
    pub const fn neutral() -> Self {
        Self {
            throttle: 0.0,
            pitch: 0.0,
            roll: 0.0,
            yaw: 0.0,
            attitude_mode: AttitudeMode::Manual,
            timestamp: Instant::from_ticks(0),
        }
    }
}

// Helper functions
#[must_use]
const fn decode_attitude_mode(ch5_value: u16) -> AttitudeMode {
    if ch5_value < MANUAL_MODE_THRESHOLD {
        AttitudeMode::Manual
    } else if ch5_value < STABILIZED_MODE_THRESHOLD {
        AttitudeMode::Stabilized
    } else {
        AttitudeMode::AltitudeHold
    }
}
