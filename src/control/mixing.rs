#[cfg(feature = "mixing")]
pub mod elevons;

#[cfg(feature = "mixing")]
pub mod yaw;

#[cfg(not(feature = "mixing"))]
pub use crate::control::mixing::legacy::*;

#[cfg(not(feature = "mixing"))]
mod legacy {
    use crate::config::*;

    /// Calculate differential thrust from rudder input (legacy)
    pub fn calculate_differential(ch4_value: u16) -> (u32, u32) {
        if ch4_value >= DIFF_NEUTRAL_MIN && ch4_value <= DIFF_NEUTRAL_MAX {
            (100, 100)
        } else if ch4_value < DIFF_NEUTRAL_MIN {
            let amount = (DIFF_NEUTRAL_MIN - ch4_value) as i32;
            let max_range = (DIFF_NEUTRAL_MIN - 300) as i32;
            let reduction = (amount * DIFF_MAX_PERCENT / max_range).min(DIFF_MAX_PERCENT);
            ((100 - reduction) as u32, 100)
        } else {
            let amount = (ch4_value - DIFF_NEUTRAL_MAX) as i32;
            let max_range = (1700 - DIFF_NEUTRAL_MAX) as i32;
            let reduction = (amount * DIFF_MAX_PERCENT / max_range).min(DIFF_MAX_PERCENT);
            (100, (100 - reduction) as u32)
        }
    }

    /// Apply differential to engine thrust (legacy)
    pub fn apply_differential(base_thrust: u32, left_mult: u32, right_mult: u32) -> (u32, u32) {
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
}
