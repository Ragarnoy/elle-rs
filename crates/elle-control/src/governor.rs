use elle_config::lut::governor_feedforward;
use elle_config::{
    GOVERNOR_DEADBAND_ERPM, GOVERNOR_DSHOT_MAX, GOVERNOR_DT, GOVERNOR_ERPM_MAX_JUMP, GOVERNOR_KI,
    GOVERNOR_KP,
};

/// Consecutive telemetry readings the spike filter may reject before it gives up and
/// accepts one. Without this the filter can latch permanently on a stale value: every
/// new reading looks like a spike relative to the stale one, so the stale one is never
/// replaced. 20 ticks = 20ms at the 1kHz task rate — long enough to ride out a genuine
/// glitch, short enough that a real step change is adopted almost immediately.
const MAX_CONSECUTIVE_SPIKE_REJECTS: u8 = 20;

/// `GOVERNOR_DSHOT_MAX` as the float the PI arithmetic clamps against.
const DSHOT_MAX_F32: f32 = GOVERNOR_DSHOT_MAX as f32;

/// Per-engine PI controller for RPM governing.
/// Converts target eRPM to DShot output using feedforward + PI correction.
#[derive(Default)]
pub struct RpmGovernor {
    integrator: f32,
    last_measured: u32,
    spike_rejects: u8,
}

impl RpmGovernor {
    pub const fn new() -> Self {
        Self {
            integrator: 0.0,
            last_measured: 0,
            spike_rejects: 0,
        }
    }

    /// Compute DShot output for a target eRPM given measured eRPM.
    ///
    /// Returns DShot 0..=`GOVERNOR_DSHOT_MAX`. A return of 0 with a non-zero target
    /// means "minimum spin" (DShot frame 48), **not** motor stop — the caller must
    /// only send `MotorStop` when the target itself is zero, or the engine loses
    /// telemetry and the loop cannot recover.
    pub fn update(&mut self, target_erpm: u32, measured_erpm: u32, valid: bool) -> u16 {
        if target_erpm == 0 {
            self.reset();
            return 0;
        }

        // Feedforward: measured LUT (piecewise linear interpolation)
        let ff = governor_feedforward(target_erpm) as f32;

        if !valid {
            // No telemetry — use feedforward only, freeze integrator
            return (ff as u16).min(GOVERNOR_DSHOT_MAX);
        }

        // Rate-limit telemetry: reject readings that jump too far from previous.
        // Bounded by MAX_CONSECUTIVE_SPIKE_REJECTS so a stale `last_measured` can
        // never latch the filter shut against every subsequent reading.
        let is_spike = self.last_measured > 0
            && self.spike_rejects < MAX_CONSECUTIVE_SPIKE_REJECTS
            && measured_erpm.abs_diff(self.last_measured) > GOVERNOR_ERPM_MAX_JUMP;
        if is_spike {
            self.spike_rejects += 1;
        } else {
            self.spike_rejects = 0;
            self.last_measured = measured_erpm;
        }
        let filtered = self.last_measured;

        // PI correction with deadband
        let error = target_erpm as f32 - filtered as f32;

        // Inside deadband: no correction, freeze integrator
        if error.abs() < GOVERNOR_DEADBAND_ERPM {
            return (ff + self.integrator).clamp(0.0, DSHOT_MAX_F32) as u16;
        }

        let p_term = GOVERNOR_KP * error;

        // Anti-windup: only integrate if output is not saturated
        // (or if error would reduce saturation). Saturation ceiling is
        // GOVERNOR_DSHOT_MAX, not DSHOT_THROTTLE_MAX — pushing past it moves into
        // a region the feedforward table refuses to enter (declining RPM for a
        // given throttle increase), so windup there is runaway, not correction.
        let candidate = ff + p_term + self.integrator;
        if (candidate > 0.0 && candidate < DSHOT_MAX_F32)
            || (candidate <= 0.0 && error > 0.0)
            || (candidate >= DSHOT_MAX_F32 && error < 0.0)
        {
            self.integrator += GOVERNOR_KI * error * GOVERNOR_DT;
        }

        (ff + p_term + self.integrator).clamp(0.0, DSHOT_MAX_F32) as u16
    }

    /// Reset integrator (e.g. on disarm or mode switch)
    pub fn reset(&mut self) {
        self.integrator = 0.0;
        self.last_measured = 0;
        self.spike_rejects = 0;
    }
}
