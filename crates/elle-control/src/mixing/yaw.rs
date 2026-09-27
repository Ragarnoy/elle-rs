#![allow(clippy::inline_always)]
use elle_config::*;

/// Combined throttle curve + differential thrust calculation (maximum performance, DShot space)
#[must_use]
#[inline(always)]
pub fn throttle_with_differential_lut(throttle_rc: u16, yaw_rc: u16) -> (u16, u16) {
    let base_thrust = throttle_curve_lut(throttle_rc);
    apply_differential_thrust_lut(base_thrust, yaw_rc)
}
