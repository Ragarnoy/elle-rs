#![allow(clippy::inline_always)]
use crate::*;

/// RC value range (0-2047, so we need 2048 entries)
pub const RC_MAX_VALUE: usize = 2047;
pub const RC_LUT_SIZE: usize = RC_MAX_VALUE + 1;

// Differential thrust LUT size - covers full RC range
pub const DIFF_LUT_SIZE: usize = RC_LUT_SIZE;

/// Const function to generate throttle curve lookup table at compile time
const fn generate_throttle_lut() -> [u32; RC_LUT_SIZE] {
    let mut lut = [0u32; RC_LUT_SIZE];
    let mut i = 0;

    while i < RC_LUT_SIZE {
        let rc = i as u32;

        let value = if rc <= THROTTLE_DEADZONE {
            ENGINE_MIN_PULSE_US
        } else if rc <= THROTTLE_START_POINT {
            let progress =
                (rc - THROTTLE_DEADZONE) * 100 / (THROTTLE_START_POINT - THROTTLE_DEADZONE);
            ENGINE_MIN_PULSE_US + (ENGINE_START_PULSE_US - ENGINE_MIN_PULSE_US) * progress / 100
        } else {
            let range = 2047 - THROTTLE_START_POINT;
            let position = rc - THROTTLE_START_POINT;
            ENGINE_START_PULSE_US + position * (ENGINE_MAX_PULSE_US - ENGINE_START_PULSE_US) / range
        };

        lut[i] = value;
        i += 1;
    }

    lut
}

/// Const function to generate servo pulse lookup table at compile time
const fn generate_servo_lut(min_us: u32, max_us: u32) -> [u32; RC_LUT_SIZE] {
    let mut lut = [0u32; RC_LUT_SIZE];
    let mut i = 0;

    while i < RC_LUT_SIZE {
        let rc = i as u32;
        let value = min_us + (rc * (max_us - min_us) / 2047);
        lut[i] = value;
        i += 1;
    }

    lut
}

/// Const function to generate normalized value lookup table
/// Using fixed-point arithmetic (multiplied by 1024) for integer math
const fn generate_normalized_lut(center: u16) -> [i32; RC_LUT_SIZE] {
    let mut lut = [0i32; RC_LUT_SIZE];
    let mut i = 0;

    while i < RC_LUT_SIZE {
        let rc = i as i32;
        // Fixed-point math: multiply by 1024 for precision
        let normalized_fp = ((rc - center as i32) * 1024) / 1024;

        // Clamp to -1024 to 1024 (representing -1.0 to 1.0)
        lut[i] = if normalized_fp < -1024 {
            -1024
        } else if normalized_fp > 1024 {
            1024
        } else {
            normalized_fp
        };

        i += 1;
    }

    lut
}

/// Generate differential thrust multiplier LUT (legacy support)
const fn generate_differential_lut() -> [(u32, u32); DIFF_LUT_SIZE] {
    let mut lut = [(100u32, 100u32); DIFF_LUT_SIZE];
    let mut i = 0;

    while i < DIFF_LUT_SIZE {
        let ch4_value = i as u16;

        let (left, right) = if ch4_value >= DIFF_NEUTRAL_MIN && ch4_value <= DIFF_NEUTRAL_MAX {
            (100, 100)
        } else if ch4_value < DIFF_NEUTRAL_MIN {
            let amount = (DIFF_NEUTRAL_MIN - ch4_value) as i32;
            let max_range = (DIFF_NEUTRAL_MIN - 300) as i32;
            let reduction = if max_range > 0 {
                amount * DIFF_MAX_PERCENT / max_range
            } else {
                DIFF_MAX_PERCENT
            };
            let reduction = if reduction < DIFF_MAX_PERCENT {
                reduction
            } else {
                DIFF_MAX_PERCENT
            };
            ((100 - reduction) as u32, 100)
        } else {
            let amount = (ch4_value - DIFF_NEUTRAL_MAX) as i32;
            let max_range = (1700 - DIFF_NEUTRAL_MAX) as i32;
            let reduction = if max_range > 0 {
                amount * DIFF_MAX_PERCENT / max_range
            } else {
                DIFF_MAX_PERCENT
            };
            let reduction = if reduction < DIFF_MAX_PERCENT {
                reduction
            } else {
                DIFF_MAX_PERCENT
            };
            (100, (100 - reduction) as u32)
        };

        lut[i] = (left, right);
        i += 1;
    }

    lut
}

/// Generate yaw differential factors LUT (for mixing mode)
const fn generate_yaw_differential_lut() -> [(i32, i32); RC_LUT_SIZE] {
    let mut lut = [(1024i32, 1024i32); RC_LUT_SIZE]; // 1024 = 1.0 in fixed point
    let mut i = 0;

    while i < RC_LUT_SIZE {
        let rc_value = i as u16;

        // Convert to normalized yaw input using fixed-point math
        let center = RC_CENTER as i32;
        let normalized_fp = ((rc_value as i32 - center) * 1024) / 1024;
        let yaw_input_fp = if normalized_fp < -1024 {
            -1024
        } else if normalized_fp > 1024 {
            1024
        } else {
            normalized_fp
        };

        // Apply YAW_TO_DIFF_GAIN (assuming 1.0 for now, can be adjusted)
        let yaw_factor_fp = yaw_input_fp; // * YAW_TO_DIFF_GAIN in fixed point

        let (left_mult_fp, right_mult_fp) = if yaw_factor_fp > 0 {
            // Right turn: reduce left engine
            let reduction = (yaw_factor_fp * 205) / 1024; // 0.2 * 1024 = ~205 in fixed point
            let reduction = if reduction > 205 { 205 } else { reduction };
            (1024 - reduction, 1024)
        } else if yaw_factor_fp < 0 {
            // Left turn: reduce right engine
            let reduction = ((-yaw_factor_fp) * 205) / 1024; // 0.2 * 1024 = ~205 in fixed point
            let reduction = if reduction > 205 { 205 } else { reduction };
            (1024, 1024 - reduction)
        } else {
            (1024, 1024) // No yaw input
        };

        // Clamp to 0.8-1.0 range (819 to 1024 in fixed point)
        let left_final = if left_mult_fp < 819 {
            819
        } else if left_mult_fp > 1024 {
            1024
        } else {
            left_mult_fp
        };
        let right_final = if right_mult_fp < 819 {
            819
        } else if right_mult_fp > 1024 {
            1024
        } else {
            right_mult_fp
        };

        lut[i] = (left_final, right_final);
        i += 1;
    }

    lut
}

// Pre-computed lookup tables - all generated at compile time
pub static THROTTLE_LUT: [u32; RC_LUT_SIZE] = generate_throttle_lut();
pub static SERVO_LUT: [u32; RC_LUT_SIZE] =
    generate_servo_lut(SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
pub static ENGINE_LUT: [u32; RC_LUT_SIZE] =
    generate_servo_lut(ENGINE_MIN_PULSE_US, ENGINE_MAX_PULSE_US);

// Single normalized lookup table — all channels share RC_CENTER after CRSF scaling
pub static NORMALIZED_LUT: [i32; RC_LUT_SIZE] = generate_normalized_lut(RC_CENTER);

// Differential thrust LUTs
pub static DIFFERENTIAL_LEGACY_LUT: [(u32, u32); DIFF_LUT_SIZE] = generate_differential_lut();
pub static YAW_DIFFERENTIAL_LUT: [(i32, i32); RC_LUT_SIZE] = generate_yaw_differential_lut();

/// Ultra-fast throttle curve lookup - single array access
#[must_use]
#[inline(always)]
pub fn throttle_curve_lut(rc_value: u16) -> u32 {
    unsafe {
        // SAFETY: We clamp the index to valid range
        *THROTTLE_LUT.get_unchecked((rc_value as usize).min(RC_MAX_VALUE))
    }
}

/// Ultra-fast servo pulse lookup
#[must_use]
#[inline(always)]
pub fn rc_to_pulse_lut(rc_value: u16) -> u32 {
    unsafe {
        // SAFETY: We clamp the index to valid range
        *SERVO_LUT.get_unchecked((rc_value as usize).min(RC_MAX_VALUE))
    }
}

/// Ultra-fast engine pulse lookup (for arming logic - linear mapping)
#[must_use]
#[inline(always)]
pub fn rc_to_engine_pulse_lut(rc_value: u16) -> u32 {
    unsafe {
        // SAFETY: We clamp the index to valid range
        *ENGINE_LUT.get_unchecked((rc_value as usize).min(RC_MAX_VALUE))
    }
}

/// Ultra-fast normalized value lookup (all axes share the same center)
#[must_use]
#[inline(always)]
pub fn rc_to_normalized(rc_value: u16) -> f32 {
    unsafe {
        // SAFETY: We clamp the index to valid range
        let fixed_point =
            *NORMALIZED_LUT.get_unchecked((rc_value as usize).min(RC_MAX_VALUE));
        fixed_point as f32 / 1024.0
    }
}

/// Ultra-fast differential thrust calculation (legacy)
#[must_use]
#[inline(always)]
pub fn calculate_differential_lut(ch4_value: u16) -> (u32, u32) {
    unsafe {
        // SAFETY: We clamp the index to valid range
        *DIFFERENTIAL_LEGACY_LUT.get_unchecked((ch4_value as usize).min(RC_MAX_VALUE))
    }
}

/// Ultra-fast yaw differential factors (mixing mode)
#[must_use]
#[inline(always)]
pub fn calculate_yaw_differential_lut(yaw_rc: u16) -> (f32, f32) {
    unsafe {
        // SAFETY: We clamp the index to valid range
        let (left_fp, right_fp) =
            *YAW_DIFFERENTIAL_LUT.get_unchecked((yaw_rc as usize).min(RC_MAX_VALUE));
        (left_fp as f32 / 1024.0, right_fp as f32 / 1024.0)
    }
}

/// Convert raw RC channels to normalized control inputs using LUTs
#[must_use]
#[inline(always)]
pub fn channels_to_normalized_lut(channels: &[u16]) -> (f32, f32, f32, f32) {
    (
        rc_to_normalized(channels[ROLL_CH]),
        rc_to_normalized(channels[PITCH_CH]),
        rc_to_normalized(channels[YAW_CH]),
        (channels[THROTTLE_CH] as f32) / 2047.0, // Simple division for throttle normalization
    )
}

/// Apply differential thrust using pre-computed values (mixing mode)
#[must_use]
#[inline(always)]
pub fn apply_differential_thrust_lut(base_thrust: u32, yaw_rc: u16) -> (u32, u32) {
    let (left_mult, right_mult) = calculate_yaw_differential_lut(yaw_rc);

    if base_thrust > ENGINE_MIN_PULSE_US {
        let thrust_range = base_thrust - ENGINE_MIN_PULSE_US;
        let left = ENGINE_MIN_PULSE_US + ((thrust_range as f32 * left_mult) as u32);
        let right = ENGINE_MIN_PULSE_US + ((thrust_range as f32 * right_mult) as u32);

        (
            left.clamp(ENGINE_MIN_PULSE_US, ENGINE_MAX_PULSE_US),
            right.clamp(ENGINE_MIN_PULSE_US, ENGINE_MAX_PULSE_US),
        )
    } else {
        (ENGINE_MIN_PULSE_US, ENGINE_MIN_PULSE_US)
    }
}

/// Apply differential thrust using pre-computed values (legacy)
#[must_use]
#[inline(always)]
pub fn apply_differential_lut(base_thrust: u32, ch4_value: u16) -> (u32, u32) {
    let (left_mult, right_mult) = calculate_differential_lut(ch4_value);

    if base_thrust > ENGINE_MIN_PULSE_US {
        let thrust_range = base_thrust - ENGINE_MIN_PULSE_US;
        let left = ENGINE_MIN_PULSE_US + (thrust_range * left_mult / 100);
        let right = ENGINE_MIN_PULSE_US + (thrust_range * right_mult / 100);

        (
            left.clamp(ENGINE_MIN_PULSE_US, ENGINE_MAX_PULSE_US),
            right.clamp(ENGINE_MIN_PULSE_US, ENGINE_MAX_PULSE_US),
        )
    } else {
        (ENGINE_MIN_PULSE_US, ENGINE_MIN_PULSE_US)
    }
}
