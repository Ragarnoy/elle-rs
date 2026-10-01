//! Yaw damper: washed-out yaw rate → differential thrust command.
//!
//! A flying wing has little directional stability, so its Dutch roll is lightly
//! damped; the roll PID only sees the roll half of it. This feeds the yaw rate
//! back into the engines instead. The washout (first-order high-pass) makes the
//! damper fade out of a steady turn, whose yaw rate is constant, so it damps
//! oscillation without fighting turns the pilot or heading hold asked for.
//!
//! Sign: a positive yaw rate is nose right (heading hold's convention: a right
//! bank raises AHRS yaw). The output is a normalized yaw command in the
//! differential thrust convention, where positive slows the left engine (nose
//! left), so the command is `+gain · rate`: it opposes the motion.
//!
//! Proposal and verification: `docs/changes/0001-eagle-yaw-damper.md`.

/// Washout high-pass, gain and clamp. Call `update` once per control tick with
/// the yaw rate and `reset` whenever the damper is not running.
#[derive(Clone, Copy, Debug)]
pub struct YawDamper {
    gain: f32,
    max: f32,
    /// High-pass weight `tau / (tau + dt)`.
    alpha: f32,
    prev_rate: Option<f32>,
    washed_out: f32,
}

impl YawDamper {
    /// `gain` in normalized yaw per rad/s, `washout_s` the high-pass time
    /// constant, `max` the output clamp, `dt` the tick period in seconds.
    #[must_use]
    pub fn new(gain: f32, washout_s: f32, max: f32, dt: f32) -> Self {
        Self {
            gain,
            max,
            alpha: washout_s / (washout_s + dt),
            prev_rate: None,
            washed_out: 0.0,
        }
    }

    /// The firmware configuration (`YAW_DAMPER_*` at `CONTROL_LOOP_DT`).
    #[must_use]
    pub fn from_config() -> Self {
        Self::new(
            elle_config::YAW_DAMPER_GAIN,
            elle_config::YAW_DAMPER_WASHOUT_S,
            elle_config::YAW_DAMPER_MAX,
            elle_config::CONTROL_LOOP_DT,
        )
    }

    /// Forget the filter state. The next `update` starts from its rate with a
    /// zero output, so engaging the damper never steps the engines.
    pub fn reset(&mut self) {
        self.prev_rate = None;
        self.washed_out = 0.0;
    }

    /// One tick: yaw rate (rad/s, positive nose right) → normalized yaw command
    /// (positive slows the left engine), clamped to ±`max`.
    pub fn update(&mut self, yaw_rate: f32) -> f32 {
        let prev = self.prev_rate.unwrap_or(yaw_rate);
        self.washed_out = self.alpha * (self.washed_out + yaw_rate - prev);
        self.prev_rate = Some(yaw_rate);
        (self.gain * self.washed_out).clamp(-self.max, self.max)
    }

    /// The washed-out rate from the last `update` (rad/s), for logging and tests.
    #[must_use]
    pub const fn washed_out_rate(&self) -> f32 {
        self.washed_out
    }
}
