#![allow(clippy::inline_always)]
use elle_config::*;

/// Differential thrust factors for yaw control
#[derive(Debug, Clone, Copy)]
pub struct DifferentialFactors {
    pub left_mult: f32,  // Multiplier for left engine (0.0 to 1.0)
    pub right_mult: f32, // Multiplier for right engine (0.0 to 1.0)
}

/// Calculate differential thrust from normalized yaw input using LUT
#[must_use]
#[inline(always)]
pub fn calculate_yaw_differential(yaw_input: f32) -> DifferentialFactors {
    // Convert normalized yaw input back to RC value for LUT lookup
    let rc_equiv = ((yaw_input * 1023.5) + 1023.5).clamp(0.0, 2047.0) as u16;
    let (left_mult, right_mult) = calculate_yaw_differential_lut(rc_equiv);

    DifferentialFactors {
        left_mult,
        right_mult,
    }
}

/// Calculate differential thrust directly from raw RC channel (ultra-fast)
#[must_use]
#[inline(always)]
pub fn calculate_yaw_differential_from_raw(yaw_rc: u16) -> DifferentialFactors {
    let (left_mult, right_mult) = calculate_yaw_differential_lut(yaw_rc);

    DifferentialFactors {
        left_mult,
        right_mult,
    }
}

/// Calculate differential thrust from RC channel (legacy) using LUT
#[must_use]
#[inline(always)]
pub fn calculate_differential_legacy(ch4_value: u16) -> DifferentialFactors {
    let (left_mult_percent, right_mult_percent) = calculate_differential_lut(ch4_value);

    DifferentialFactors {
        left_mult: left_mult_percent as f32 / 100.0,
        right_mult: right_mult_percent as f32 / 100.0,
    }
}

/// Apply differential factors to base engine thrust (DShot space)
#[must_use]
#[inline(always)]
pub fn apply_differential_thrust(base_thrust: u16, factors: &DifferentialFactors) -> (u16, u16) {
    if base_thrust == 0 {
        return (0, 0);
    }
    let left = ((base_thrust as f32 * factors.left_mult) as u16).min(DSHOT_THROTTLE_MAX);
    let right = ((base_thrust as f32 * factors.right_mult) as u16).min(DSHOT_THROTTLE_MAX);
    (left, right)
}

/// Ultra-fast differential thrust calculation directly from RC value to thrust values (DShot space)
#[must_use]
#[inline(always)]
pub fn apply_differential_thrust_direct(base_thrust: u16, yaw_rc: u16) -> (u16, u16) {
    apply_differential_thrust_lut(base_thrust, yaw_rc)
}

/// Combined throttle curve + differential thrust calculation (maximum performance, DShot space)
#[must_use]
#[inline(always)]
pub fn throttle_with_differential_lut(throttle_rc: u16, yaw_rc: u16) -> (u16, u16) {
    let base_thrust = throttle_curve_lut(throttle_rc);
    apply_differential_thrust_lut(base_thrust, yaw_rc)
}
