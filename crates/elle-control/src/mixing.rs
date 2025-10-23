#[cfg(not(feature = "legacy-ctrl"))]
pub mod elevons;

#[cfg(not(feature = "legacy-ctrl"))]
pub mod yaw;

#[cfg(feature = "legacy-ctrl")]
pub use crate::mixing::legacy::*;

#[cfg(feature = "legacy-ctrl")]
mod legacy {
    use elle_config::lut::*;
    use elle_config::*;

    /// Calculate differential thrust from rudder input using ultra-fast LUT (legacy)
    #[inline(always)]
    pub fn calculate_differential(ch4_value: u16) -> (u32, u32) {
        calculate_differential_lut(ch4_value)
    }

    /// Apply differential to engine thrust using ultra-fast LUT (legacy)
    #[inline(always)]
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

    /// Combined legacy differential calculation and application (maximum performance)
    #[inline(always)]
    pub fn apply_differential_complete(base_thrust: u32, ch4_value: u16) -> (u32, u32) {
        apply_differential_lut(base_thrust, ch4_value)
    }
}
