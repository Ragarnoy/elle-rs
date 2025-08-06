use crate::config::*;
use defmt::Format;

/// Convert SBUS values to normalized control inputs (-1.0 to 1.0) with calibrated centers
pub fn sbus_to_normalized(sbus_value: u16) -> f32 {
    // SBUS range is 0-2047, center at ~1023 (but varies by transmitter)
    let normalized = (sbus_value as f32 - 1023.5) / 1023.5;
    normalized.clamp(-1.0, 1.0)
}

/// Convert SBUS value to normalized with specific center point
fn sbus_to_normalized_with_center(sbus_value: u16, center: u16) -> f32 {
    // Validate center point is reasonable
    let safe_center = center.clamp(1023 - SBUS_CENTER_TOLERANCE, 1023 + SBUS_CENTER_TOLERANCE);
    let normalized = (sbus_value as f32 - safe_center as f32) / 1023.5;
    normalized.clamp(-1.0, 1.0)
}

/// Convert normalized control input to servo pulse width
pub fn normalized_to_servo_us(normalized: f32) -> u32 {
    let center = SERVO_CENTER_US as f32;
    let range = (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) as f32 / 2.0;
    let pulse_us = center + (normalized * range);
    pulse_us.clamp(SERVO_MIN_PULSE_US as f32, SERVO_MAX_PULSE_US as f32) as u32
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
    /// Create control inputs from SBUS packet channels with calibrated centers
    pub fn from_sbus_channels(channels: &[u16]) -> Self {
        Self {
            pitch: sbus_to_normalized_with_center(channels[PITCH_CH], SBUS_PITCH_CENTER),
            roll: sbus_to_normalized_with_center(channels[ROLL_CH], SBUS_ROLL_CENTER),
            yaw: sbus_to_normalized_with_center(channels[YAW_CH], SBUS_YAW_CENTER),
            throttle: (channels[THROTTLE_CH] as f32 / 2047.0).clamp(0.0, 1.0),
        }
    }

    /// Legacy: create from direct elevon channels (for backwards compatibility)
    pub fn from_direct_elevons(channels: &[u16]) -> Self {
        Self {
            pitch: 0.0, // No pitch input in direct mode
            roll: 0.0,  // No roll input in direct mode
            yaw: sbus_to_normalized_with_center(channels[DIFFERENTIAL_CH], SBUS_YAW_CENTER),
            throttle: (channels[ENGINE_CH] as f32 / 2047.0).clamp(0.0, 1.0),
        }
    }
}

/// Mixes pitch, roll and yaw inputs into elevon control surface positions
///
/// # Arguments
/// * `inputs` - Normalized control inputs (-1.0 to 1.0)
///
/// # Returns
/// * `ElevonOutputs` - Servo pulse widths for left and right elevons
pub fn mix_elevons(inputs: &ControlInputs) -> ElevonOutputs {
    // Clamp input values to valid range
    let pitch = inputs.pitch.clamp(-1.0, 1.0);
    let roll = inputs.roll.clamp(-1.0, 1.0);
    let yaw = inputs.yaw.clamp(-1.0, 1.0);

    // Elevon mixing: each elevon responds to both pitch and roll
    // Left elevon: positive pitch (nose up) + positive roll (right roll) = up deflection
    // Right elevon: positive pitch (nose up) - positive roll (right roll) = up deflection
    let left_elevon_normalized =
        ((pitch * ELEVON_PITCH_GAIN) + (roll * ELEVON_ROLL_GAIN) + (yaw * YAW_TO_ELEVON_GAIN))
            .clamp(-1.0, 1.0);

    let right_elevon_normalized =
        ((pitch * ELEVON_PITCH_GAIN) - (roll * ELEVON_ROLL_GAIN) - (yaw * YAW_TO_ELEVON_GAIN))
            .clamp(-1.0, 1.0);

    // Convert to servo pulse widths (trim applied in PWM layer)
    let left_us = normalized_to_servo_us(left_elevon_normalized);
    let right_us = normalized_to_servo_us(right_elevon_normalized);

    ElevonOutputs { left_us, right_us }
}

/// Get direct elevon control (legacy mode, trim applied in PWM layer)
pub fn direct_elevon_control(channels: &[u16]) -> ElevonOutputs {
    use crate::control::throttle::sbus_to_pulse_us;

    let left_us = sbus_to_pulse_us(
        channels[ELEVON_LEFT_CH],
        SERVO_MIN_PULSE_US,
        SERVO_MAX_PULSE_US,
    );
    let right_us = sbus_to_pulse_us(
        channels[ELEVON_RIGHT_CH],
        SERVO_MIN_PULSE_US,
        SERVO_MAX_PULSE_US,
    );

    ElevonOutputs { left_us, right_us }
}
