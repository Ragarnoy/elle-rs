use crate::config::*;

/// Differential thrust factors for yaw control
#[derive(Debug, Clone, Copy)]
pub struct DifferentialFactors {
    pub left_mult: f32,  // Multiplier for left engine (0.0 to 1.0)
    pub right_mult: f32, // Multiplier for right engine (0.0 to 1.0)
}

/// Calculate differential thrust from normalized yaw input
pub fn calculate_yaw_differential(yaw_input: f32) -> DifferentialFactors {
    // Yaw control through differential thrust
    // Positive yaw (right turn) = reduce left engine, maintain right engine
    // Negative yaw (left turn) = reduce right engine, maintain left engine

    let yaw_factor = yaw_input * YAW_TO_DIFF_GAIN;

    let left_mult = if yaw_factor > 0.0 {
        // Right turn: reduce left engine
        1.0 - (yaw_factor * 0.2).min(0.2)
    } else {
        1.0 // Full power for left turn
    };

    let right_mult = if yaw_factor < 0.0 {
        // Left turn: reduce right engine
        1.0 - (yaw_factor.abs() * 0.2).min(0.2)
    } else {
        1.0 // Full power for right turn
    };

    DifferentialFactors {
        left_mult: left_mult.clamp(0.8, 1.0), // Don't reduce below 80%
        right_mult: right_mult.clamp(0.8, 1.0),
    }
}

/// Calculate differential thrust from SBUS channel (legacy)
pub fn calculate_differential_legacy(ch4_value: u16) -> DifferentialFactors {
    if (DIFF_NEUTRAL_MIN..=DIFF_NEUTRAL_MAX).contains(&ch4_value) {
        DifferentialFactors {
            left_mult: 1.0,
            right_mult: 1.0,
        }
    } else if ch4_value < DIFF_NEUTRAL_MIN {
        let amount = (DIFF_NEUTRAL_MIN - ch4_value) as i32;
        let max_range = (DIFF_NEUTRAL_MIN - 300) as i32;
        let reduction = (amount * DIFF_MAX_PERCENT / max_range).min(DIFF_MAX_PERCENT);
        DifferentialFactors {
            left_mult: (100 - reduction) as f32 / 100.0,
            right_mult: 1.0,
        }
    } else {
        let amount = (ch4_value - DIFF_NEUTRAL_MAX) as i32;
        let max_range = (1700 - DIFF_NEUTRAL_MAX) as i32;
        let reduction = (amount * DIFF_MAX_PERCENT / max_range).min(DIFF_MAX_PERCENT);
        DifferentialFactors {
            left_mult: 1.0,
            right_mult: (100 - reduction) as f32 / 100.0,
        }
    }
}

/// Apply differential factors to base engine thrust
pub fn apply_differential_thrust(base_thrust: u32, factors: &DifferentialFactors) -> (u32, u32) {
    if base_thrust > ENGINE_MIN_PULSE_US {
        let thrust_range = base_thrust - ENGINE_MIN_PULSE_US;
        let left = ENGINE_MIN_PULSE_US + ((thrust_range as f32 * factors.left_mult) as u32);
        let right = ENGINE_MIN_PULSE_US + ((thrust_range as f32 * factors.right_mult) as u32);

        (
            left.clamp(ENGINE_MIN_PULSE_US, ENGINE_MAX_PULSE_US),
            right.clamp(ENGINE_MIN_PULSE_US, ENGINE_MAX_PULSE_US),
        )
    } else {
        (ENGINE_MIN_PULSE_US, ENGINE_MIN_PULSE_US)
    }
}
