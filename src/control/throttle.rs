use crate::config::*;

/// Apply throttle curve for better low-end control
pub fn throttle_curve(sbus_value: u16) -> u32 {
    let sbus = sbus_value as u32;

    if sbus <= THROTTLE_DEADZONE {
        ENGINE_MIN_PULSE_US
    } else if sbus <= THROTTLE_START_POINT {
        let progress = (sbus - THROTTLE_DEADZONE) * 100 / (THROTTLE_START_POINT - THROTTLE_DEADZONE);
        ENGINE_MIN_PULSE_US + (ENGINE_START_PULSE_US - ENGINE_MIN_PULSE_US) * progress / 100
    } else {
        let range = 2047 - THROTTLE_START_POINT;
        let position = sbus - THROTTLE_START_POINT;
        ENGINE_START_PULSE_US + position * (ENGINE_MAX_PULSE_US - ENGINE_START_PULSE_US) / range
    }
}

/// Convert SBUS value to pulse width (linear mapping)
pub fn sbus_to_pulse_us(sbus_value: u16, min_us: u32, max_us: u32) -> u32 {
    min_us + (sbus_value as u32 * (max_us - min_us) / 2047)
}
