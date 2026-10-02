#![allow(clippy::inline_always)]
use elle_config::*;

/// Combined throttle curve + differential thrust calculation (maximum performance, DShot space)
#[must_use]
#[inline(always)]
pub fn throttle_with_differential_lut(throttle_rc: u16, yaw_rc: u16) -> (u16, u16) {
    let base_thrust = throttle_curve_lut(throttle_rc);
    apply_differential_thrust_lut(base_thrust, yaw_rc)
}

/// Normalized yaw command (-1..1) to the RC-space value the differential thrust
/// LUT takes. Positive is above centre, which slows the left engine: a nose-left
/// moment. (A right stick arrives negative, through `YAW_INVERT`.)
#[must_use]
pub fn normalized_yaw_to_rc(yaw: f32) -> u16 {
    ((yaw * 1023.5) + 1023.5).clamp(0.0, 2047.0) as u16
}
