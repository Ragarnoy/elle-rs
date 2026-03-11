use elle_config::{DSHOT_THROTTLE_MAX, GOVERNOR_DT, GOVERNOR_KI, GOVERNOR_KP, MAX_ERPM};

/// Per-engine PI controller for RPM governing.
/// Converts target eRPM to DShot output using feedforward + PI correction.
#[derive(Default)]
pub struct RpmGovernor {
    integrator: f32,
}

impl RpmGovernor {
    pub const fn new() -> Self {
        Self { integrator: 0.0 }
    }

    /// Compute DShot output for a target eRPM given measured eRPM.
    /// Returns DShot 0-1999.
    pub fn update(&mut self, target_erpm: u32, measured_erpm: u32, valid: bool) -> u16 {
        if target_erpm == 0 {
            self.integrator = 0.0;
            return 0;
        }

        // Feedforward: linear estimate
        let ff = (target_erpm as f32 / MAX_ERPM as f32) * DSHOT_THROTTLE_MAX as f32;

        if !valid {
            // No telemetry — use feedforward only, freeze integrator
            return (ff as u16).min(DSHOT_THROTTLE_MAX);
        }

        // PI correction
        let error = target_erpm as f32 - measured_erpm as f32;
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
    }
}
