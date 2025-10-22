use elle_config::*;
use defmt::Format;

/// Convert SBUS values to normalized control inputs using ultra-fast LUT
#[inline(always)]
pub fn sbus_to_normalized(sbus_value: u16) -> f32 {
    // Default to roll normalization - this maintains backward compatibility
    sbus_to_normalized_roll_lut(sbus_value)
}

/// Convert SBUS value to normalized with specific center point using LUT
#[inline(always)]
pub fn sbus_to_normalized_with_center(sbus_value: u16, _center: u16) -> f32 {
    sbus_to_normalized_roll_lut(sbus_value) // Default to roll
}

/// Convert normalized control input to servo pulse width using LUT
/// For values outside -1.0 to 1.0, falls back to calculation
#[inline(always)]
pub fn normalized_to_servo_us(normalized: f32) -> u32 {
    // Convert normalized to SBUS equivalent for LUT lookup
    let sbus_equiv = ((normalized * 1023.5) + 1023.5).clamp(0.0, 2047.0) as u16;
    sbus_to_pulse_lut(sbus_equiv)
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
}

impl ControlInputs {
    /// Create control inputs from SBUS packet channels using ultra-fast LUTs
    #[inline(always)]
    pub fn from_sbus_channels(channels: &[u16]) -> Self {
        Self {
            pitch: sbus_to_normalized_pitch_lut(channels[PITCH_CH]),
            roll: sbus_to_normalized_roll_lut(channels[ROLL_CH]),
            yaw: sbus_to_normalized_yaw_lut(channels[YAW_CH]),
            throttle: (channels[THROTTLE_CH] as f32 / 2047.0).clamp(0.0, 1.0),
        }
    }

    /// Ultra-fast batch conversion using single LUT call
    #[inline(always)]
    pub fn from_sbus_channels_fast(channels: &[u16]) -> Self {
        let (roll, pitch, yaw, throttle) = channels_to_normalized_lut(channels);
        Self {
            pitch,
            roll,
            yaw,
            throttle,
        }
    }

    /// Legacy: create from direct elevon channels (for backwards compatibility)
    #[cfg(feature = "legacy-ctrl")]
    pub fn from_direct_elevons(channels: &[u16]) -> Self {
        Self {
            pitch: 0.0, // No pitch input in direct mode
            roll: 0.0,  // No roll input in direct mode
            yaw: sbus_to_normalized_yaw_lut(channels[DIFFERENTIAL_CH]),
            throttle: (channels[ENGINE_CH] as f32 / 2047.0).clamp(0.0, 1.0),
        }
    }
}

/// Mixes pitch, roll and yaw inputs into elevon control surface positions
/// Optimized version using fast normalization
#[inline(always)]
pub fn mix_elevons(inputs: &ControlInputs) -> ElevonOutputs {
    // Inputs are already clamped from LUT lookups, but clamp again for safety
    let pitch = inputs.pitch.clamp(-1.0, 1.0);
    let roll = inputs.roll.clamp(-1.0, 1.0);
    let yaw = inputs.yaw.clamp(-1.0, 1.0);

    // Elevon mixing: each elevon responds to both pitch and roll
    let left_elevon_normalized = ((pitch * ELEVON_PITCH_GAIN) - (roll * ELEVON_ROLL_GAIN)
        + (yaw * YAW_TO_ELEVON_GAIN))
        .clamp(-1.0, 1.0);

    let right_elevon_normalized = ((pitch * ELEVON_PITCH_GAIN) + (roll * ELEVON_ROLL_GAIN)
        - (yaw * YAW_TO_ELEVON_GAIN))
        .clamp(-1.0, 1.0);

    // Convert to servo pulse widths using LUT
    let left_us = normalized_to_servo_us(left_elevon_normalized);
    let right_us = normalized_to_servo_us(right_elevon_normalized);

    ElevonOutputs { left_us, right_us }
}

/// Ultra-fast elevon mixing using direct SBUS values
/// This version skips the ControlInputs struct for maximum performance
#[inline(always)]
pub fn mix_elevons_direct_lut(channels: &[u16]) -> ElevonOutputs {
    let (roll, pitch, yaw, _throttle) = channels_to_normalized_lut(channels);

    // Direct mixing calculations
    let left_elevon_normalized = ((pitch * ELEVON_PITCH_GAIN) - (roll * ELEVON_ROLL_GAIN)
        + (yaw * YAW_TO_ELEVON_GAIN))
        .clamp(-1.0, 1.0);

    let right_elevon_normalized = ((pitch * ELEVON_PITCH_GAIN) + (roll * ELEVON_ROLL_GAIN)
        - (yaw * YAW_TO_ELEVON_GAIN))
        .clamp(-1.0, 1.0);

    // Convert back to SBUS equivalent and use LUT
    let left_sbus = ((left_elevon_normalized * 1023.5) + 1023.5).clamp(0.0, 2047.0) as u16;
    let right_sbus = ((right_elevon_normalized * 1023.5) + 1023.5).clamp(0.0, 2047.0) as u16;

    ElevonOutputs {
        left_us: sbus_to_pulse_lut(left_sbus),
        right_us: sbus_to_pulse_lut(right_sbus),
    }
}

/// Get direct elevon control using LUT (legacy mode, trim applied in PWM layer)
#[inline(always)]
pub fn direct_elevon_control(channels: &[u16]) -> ElevonOutputs {
    ElevonOutputs {
        left_us: sbus_to_pulse_lut(channels[ELEVON_LEFT_CH]),
        right_us: sbus_to_pulse_lut(channels[ELEVON_RIGHT_CH]),
    }
}
