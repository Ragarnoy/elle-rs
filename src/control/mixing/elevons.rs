use crate::config::*;

/// Convert SBUS values to normalized control inputs (-1.0 to 1.0)
pub fn sbus_to_normalized(sbus_value: u16) -> f32 {
    // SBUS range is 0-2047, center at ~1023
    let normalized = (sbus_value as f32 - 1023.5) / 1023.5;
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
#[derive(Debug, Clone, Copy)]
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
    /// Create control inputs from SBUS packet channels
    pub fn from_sbus_channels(channels: &[u16]) -> Self {
        Self {
            pitch: sbus_to_normalized(channels[PITCH_CH]),
            roll: sbus_to_normalized(channels[ROLL_CH]),
            yaw: sbus_to_normalized(channels[YAW_CH]),
            throttle: (channels[THROTTLE_CH] as f32 / 2047.0).clamp(0.0, 1.0),
        }
    }

    /// Legacy: create from direct elevon channels (for backwards compatibility)
    pub fn from_direct_elevons(channels: &[u16]) -> Self {
        Self {
            pitch: 0.0, // No pitch input in direct mode
            roll: 0.0,  // No roll input in direct mode
            yaw: sbus_to_normalized(channels[DIFFERENTIAL_CH]),
            throttle: (channels[ENGINE_CH] as f32 / 2047.0).clamp(0.0, 1.0),
        }
    }
}

/// Mix pitch and roll inputs into elevon positions
pub fn mix_elevons(inputs: &ControlInputs) -> ElevonOutputs {
    // Elevon mixing: each elevon responds to both pitch and roll
    // Left elevon: positive pitch (nose up) + positive roll (right roll) = up deflection
    // Right elevon: positive pitch (nose up) - positive roll (right roll) = up deflection

    let left_elevon_normalized = (inputs.pitch * ELEVON_PITCH_GAIN)
        + (inputs.roll * ELEVON_ROLL_GAIN)
        + (inputs.yaw * YAW_TO_ELEVON_GAIN);

    let right_elevon_normalized = (inputs.pitch * ELEVON_PITCH_GAIN)
        - (inputs.roll * ELEVON_ROLL_GAIN)
        - (inputs.yaw * YAW_TO_ELEVON_GAIN);

    // Convert to servo pulse widths
    let left_us = normalized_to_servo_us(left_elevon_normalized);
    let right_us = normalized_to_servo_us(right_elevon_normalized);

    ElevonOutputs { left_us, right_us }
}

/// Get direct elevon control (legacy mode)
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
