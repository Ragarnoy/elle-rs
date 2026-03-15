use elle_config::{
    DSHOT_THROTTLE_MAX, GOVERNOR_DEADBAND_ERPM, GOVERNOR_DT, GOVERNOR_ERPM_MAX_JUMP,
    GOVERNOR_KI, GOVERNOR_KP,
};
use elle_config::lut::governor_feedforward;

/// Per-engine PI controller for RPM governing.
/// Converts target eRPM to DShot output using feedforward + PI correction.
#[derive(Default)]
pub struct RpmGovernor {
    integrator: f32,
    last_measured: u32,
}

impl RpmGovernor {
    pub const fn new() -> Self {
        Self {
            integrator: 0.0,
            last_measured: 0,
        }
    }

    /// Compute DShot output for a target eRPM given measured eRPM.
    /// Returns DShot 0-1999.
    pub fn update(&mut self, target_erpm: u32, measured_erpm: u32, valid: bool) -> u16 {
        if target_erpm == 0 {
            self.integrator = 0.0;
            self.last_measured = 0;
            return 0;
        }

        // Feedforward: measured LUT (piecewise linear interpolation)
        let ff = governor_feedforward(target_erpm) as f32;

        if !valid {
            // No telemetry — use feedforward only, freeze integrator
            return (ff as u16).min(DSHOT_THROTTLE_MAX);
        }

        // Rate-limit telemetry: reject readings that jump too far from previous
        let is_spike = self.last_measured > 0
            && measured_erpm.abs_diff(self.last_measured) > GOVERNOR_ERPM_MAX_JUMP;
        if !is_spike {
            self.last_measured = measured_erpm;
        }
        let filtered = self.last_measured;

        // PI correction with deadband
        let error = target_erpm as f32 - filtered as f32;

        // Inside deadband: no correction, freeze integrator
        if error.abs() < GOVERNOR_DEADBAND_ERPM {
            return (ff + self.integrator).clamp(0.0, DSHOT_THROTTLE_MAX as f32) as u16;
        }

        let p_term = GOVERNOR_KP * error;

        // Anti-windup: only integrate if output is not saturated
        // (or if error would reduce saturation)
        let candidate = ff + p_term + self.integrator;
        if (candidate > 0.0 && candidate < DSHOT_THROTTLE_MAX as f32)
            || (candidate <= 0.0 && error > 0.0)
            || (candidate >= DSHOT_THROTTLE_MAX as f32 && error < 0.0)
        {
            self.integrator += GOVERNOR_KI * error * GOVERNOR_DT;
        }

        (ff + p_term + self.integrator).clamp(0.0, DSHOT_THROTTLE_MAX as f32) as u16
    }

    /// Reset integrator (e.g. on disarm or mode switch)
    pub fn reset(&mut self) {
        self.integrator = 0.0;
        self.last_measured = 0;
    }
}
