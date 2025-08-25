use crate::config::lut::*;
use crate::config::*;

/// Apply throttle curve for better low-end control using LUT
/// This is now a simple wrapper around the LUT function
#[inline(always)]
pub fn throttle_curve(sbus_value: u16) -> u32 {
    throttle_curve_lut(sbus_value)
}

/// Convert SBUS value to pulse width using LUT
/// This function now properly handles engine vs servo ranges
#[inline(always)]
pub fn sbus_to_pulse_us(sbus_value: u16, min_us: u32, max_us: u32) -> u32 {
    // Check if this is for engine range (for arming logic)
    if min_us == ENGINE_MIN_PULSE_US && max_us == ENGINE_MAX_PULSE_US {
        sbus_to_engine_pulse_lut(sbus_value)
    } else if min_us == SERVO_MIN_PULSE_US && max_us == SERVO_MAX_PULSE_US {
        sbus_to_pulse_lut(sbus_value)
    } else {
        // Fallback for custom ranges not in LUT
        sbus_to_pulse_us_custom(sbus_value, min_us, max_us)
    }
}

/// Convert SBUS value to pulse width with custom range (fallback to calculation)
/// Use this when you need non-standard ranges not covered by LUTs
pub fn sbus_to_pulse_us_custom(sbus_value: u16, min_us: u32, max_us: u32) -> u32 {
    min_us + (sbus_value as u32 * (max_us - min_us) / 2047)
}

// Legacy functions for backward compatibility - now using LUTs internally
#[inline(always)]
pub fn throttle_curve_fast(sbus_value: u16) -> u32 {
    throttle_curve_lut(sbus_value)
}

#[inline(always)]
pub fn sbus_to_pulse_fast(sbus_value: u16) -> u32 {
    sbus_to_pulse_lut(sbus_value)
}

/// Engine-specific function for arming logic
#[inline(always)]
pub fn sbus_to_engine_pulse_us(sbus_value: u16) -> u32 {
    sbus_to_engine_pulse_lut(sbus_value)
}
