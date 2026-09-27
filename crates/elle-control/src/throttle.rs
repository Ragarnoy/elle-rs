#![allow(clippy::inline_always)]
use elle_config::lut::*;
use elle_config::*;

/// Convert RC value to pulse width using LUT
/// This function handles engine vs servo ranges
#[must_use]
#[inline(always)]
pub(crate) fn rc_to_pulse_us(rc_value: u16, min_us: u32, max_us: u32) -> u32 {
    // Check if this is for engine range (for arming logic)
    if min_us == ENGINE_MIN_PULSE_US && max_us == ENGINE_MAX_PULSE_US {
        rc_to_engine_pulse_lut(rc_value)
    } else if min_us == SERVO_MIN_PULSE_US && max_us == SERVO_MAX_PULSE_US {
        rc_to_pulse_lut(rc_value)
    } else {
        // Fallback for custom ranges not in LUT
        rc_to_pulse_us_custom(rc_value, min_us, max_us)
    }
}

/// Convert RC value to pulse width with custom range (fallback to calculation)
#[must_use]
const fn rc_to_pulse_us_custom(rc_value: u16, min_us: u32, max_us: u32) -> u32 {
    min_us + (rc_value as u32 * (max_us - min_us) / 2047)
}
