#![allow(clippy::inline_always)]
use defmt::Format;
use elle_config::*;

/// Convert normalized control input to servo pulse width using LUT
#[must_use]
#[inline(always)]
fn normalized_to_servo_us(normalized: f32) -> u32 {
    // Convert normalized to RC equivalent for LUT lookup
    let rc_equiv = ((normalized * 1023.5) + 1023.5).clamp(0.0, 2047.0) as u16;
    rc_to_pulse_lut(rc_equiv)
}

/// Flight control inputs in normalized form
#[derive(Debug, Clone, Copy, Format)]
pub struct ControlInputs {
    pub pitch: f32,    // -1.0 = nose down, +1.0 = nose up
    pub roll: f32,     // -1.0 = left roll, +1.0 = right roll
    pub yaw: f32,      // -1.0 = left yaw, +1.0 = right yaw
    pub throttle: f32, // 0.0 = min, 1.0 = max
}

/// Elevon output positions
#[derive(Debug, Clone, Copy)]
pub struct ElevonOutputs {
    pub left_us: u32,
    pub right_us: u32,
    /// Which command directions hit a limit in this mix.
    pub saturation: MixSaturation,
}

/// Directions in which a larger pitch/roll command would have no further effect,
/// because an axis input or an elevon was clipped. The attitude PID stops
/// integrating in a blocked direction (anti-windup after mixing: pitch and roll
/// share both surfaces, so one axis can saturate the other's authority).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Format)]
pub struct MixSaturation {
    /// More nose-up (positive pitch) is blocked.
    pub pitch_up: bool,
    /// More nose-down (negative pitch) is blocked.
    pub pitch_down: bool,
    /// More right roll (positive roll) is blocked.
    pub roll_right: bool,
    /// More left roll (negative roll) is blocked.
    pub roll_left: bool,
}

impl MixSaturation {
    /// Pack as bits: pitch_up, pitch_down, roll_right, roll_left (LSB first).
    #[must_use]
    pub const fn bits(self) -> u8 {
        self.pitch_up as u8
            | (self.pitch_down as u8) << 1
            | (self.roll_right as u8) << 2
            | (self.roll_left as u8) << 3
    }
}

// The saturation directions below assume positive pitch raises both elevons and
// positive roll raises the right one and lowers the left.
const _: () = assert!(ELEVON_PITCH_GAIN > 0.0 && ELEVON_ROLL_GAIN > 0.0);

/// Mixes pitch, roll and yaw inputs into elevon control surface positions
#[must_use]
#[inline(always)]
pub fn mix_elevons(inputs: &ControlInputs) -> ElevonOutputs {
    let pitch = inputs.pitch.clamp(-1.0, 1.0);
    let roll = inputs.roll.clamp(-1.0, 1.0);
    let yaw = inputs.yaw.clamp(-1.0, 1.0);

    // Elevon mixing: each elevon responds to both pitch and roll
    let left_raw =
        (pitch * ELEVON_PITCH_GAIN) - (roll * ELEVON_ROLL_GAIN) + (yaw * YAW_TO_ELEVON_GAIN);
    let right_raw =
        (pitch * ELEVON_PITCH_GAIN) + (roll * ELEVON_ROLL_GAIN) - (yaw * YAW_TO_ELEVON_GAIN);

    let saturation = MixSaturation {
        pitch_up: inputs.pitch >= 1.0 || left_raw >= 1.0 || right_raw >= 1.0,
        pitch_down: inputs.pitch <= -1.0 || left_raw <= -1.0 || right_raw <= -1.0,
        roll_right: inputs.roll >= 1.0 || left_raw <= -1.0 || right_raw >= 1.0,
        roll_left: inputs.roll <= -1.0 || left_raw >= 1.0 || right_raw <= -1.0,
    };

    // Convert to servo pulse widths using LUT
    let left_us = normalized_to_servo_us(left_raw.clamp(-1.0, 1.0));
    let right_us = normalized_to_servo_us(right_raw.clamp(-1.0, 1.0));

    ElevonOutputs {
        left_us,
        right_us,
        saturation,
    }
}

/// Ultra-fast elevon mixing using direct RC values
#[must_use]
#[inline(always)]
pub fn mix_elevons_direct_lut(channels: &[u16]) -> ElevonOutputs {
    let (roll, pitch, yaw, _throttle) = channels_to_normalized_lut(channels);

    let left_elevon_normalized = ((pitch * ELEVON_PITCH_GAIN) - (roll * ELEVON_ROLL_GAIN)
        + (yaw * YAW_TO_ELEVON_GAIN))
        .clamp(-1.0, 1.0);

    let right_elevon_normalized = ((pitch * ELEVON_PITCH_GAIN) + (roll * ELEVON_ROLL_GAIN)
        - (yaw * YAW_TO_ELEVON_GAIN))
        .clamp(-1.0, 1.0);

    // Convert back to RC equivalent and use LUT
    let left_rc = ((left_elevon_normalized * 1023.5) + 1023.5).clamp(0.0, 2047.0) as u16;
    let right_rc = ((right_elevon_normalized * 1023.5) + 1023.5).clamp(0.0, 2047.0) as u16;

    ElevonOutputs {
        left_us: rc_to_pulse_lut(left_rc),
        right_us: rc_to_pulse_lut(right_rc),
        saturation: MixSaturation::default(),
    }
}
