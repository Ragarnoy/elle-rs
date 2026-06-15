//! Attitude controller for flying-wing stabilization.
//!
//! Each axis is a simple angle PID: P and I act on the angle error, D acts on
//! a directly-supplied gyro rate (already sign-corrected by the caller to
//! represent d(error)/dt) rather than a finite difference of the noisy AHRS
//! angle.
use elle_config::CONTROL_LOOP_DT;
use embassy_time::Instant;

/// Single-axis angle PID: P on angle error, I on error (clamped), D on supplied rate.
#[derive(Clone, Copy, Default)]
struct AxisPid {
    kp: f32,
    ki: f32,
    kd: f32,
    integral: f32,
}

impl AxisPid {
    fn compute(
        &mut self,
        error: f32,
        rate: f32,
        i_limit: f32,
        dt: f32,
        reset_integral: bool,
    ) -> f32 {
        // Negative i_limit would invert the clamp bounds below; treat as zero.
        let i_limit = if i_limit > 0.0 { i_limit } else { 0.0 };
        self.integral = if reset_integral {
            0.0
        } else {
            let sum = self.integral + error * dt;
            if sum > i_limit {
                i_limit
            } else if sum < -i_limit {
                -i_limit
            } else {
                sum
            }
        };
        self.kp * error + self.ki * self.integral + self.kd * rate
    }

    fn reset(&mut self) {
        self.integral = 0.0;
    }
}

/// PID gains plus shared output scale and integral clamp.
#[derive(Clone, Copy)]
pub struct PidConfig {
    pub kp_pitch: f32,
    pub ki_pitch: f32,
    pub kd_pitch: f32,
    pub kp_roll: f32,
    pub ki_roll: f32,
    pub kd_roll: f32,
    pub i_limit: f32,
    pub scale: f32,
}

/// Attitude controller for flying-wing pitch/roll stabilization.
pub struct AttitudeController {
    pitch_pid: AxisPid,
    roll_pid: AxisPid,
    i_limit: f32,
    scale: f32,
    last_time: Option<Instant>,

    // Control flags
    pub enabled: bool,
    pub pitch_hold_enabled: bool,
    pub roll_hold_enabled: bool,
}

impl AttitudeController {
    /// Apply gains/scale/limit from `config`, resetting both axes' integrators.
    fn apply_gains(&mut self, config: PidConfig) {
        self.pitch_pid = AxisPid {
            kp: config.kp_pitch,
            ki: config.ki_pitch,
            kd: config.kd_pitch,
            integral: 0.0,
        };
        self.roll_pid = AxisPid {
            kp: config.kp_roll,
            ki: config.ki_roll,
            kd: config.kd_roll,
            integral: 0.0,
        };
        self.i_limit = config.i_limit;
        self.scale = config.scale;
    }

    /// Create with the given gains/scale/limit
    #[must_use]
    pub fn with_config(config: PidConfig) -> Self {
        let mut controller = Self {
            pitch_pid: AxisPid::default(),
            roll_pid: AxisPid::default(),
            i_limit: 0.0,
            scale: 0.0,
            last_time: None,
            enabled: false,
            pitch_hold_enabled: true,
            roll_hold_enabled: false, // Usually just level wings for flying wings
        };
        controller.apply_gains(config);
        controller
    }

    /// Update gains/scale/limit and reset integral state, leaving enabled/hold flags untouched
    pub fn update_config(&mut self, config: PidConfig) {
        self.apply_gains(config);
        self.last_time = None;
    }

    /// Reset the controller state (clears integrators and timing)
    pub fn reset(&mut self) {
        self.pitch_pid.reset();
        self.roll_pid.reset();
        self.last_time = None;
    }

    /// Calculate attitude corrections.
    ///
    /// `gyro_rates` is `(roll_rate, pitch_rate, yaw_rate)`, already sign-corrected
    /// by the caller so that each component equals d(error)/dt for that axis.
    #[allow(clippy::too_many_arguments)]
    pub fn update(
        &mut self,
        desired_pitch: f32,
        desired_roll: f32,
        current_pitch: f32,
        current_roll: f32,
        gyro_rates: Option<(f32, f32, f32)>,
        now: Instant,
        low_throttle: bool,
    ) -> (f32, f32) {
        if !self.enabled {
            return (0.0, 0.0);
        }

        let dt = CONTROL_LOOP_DT;
        self.last_time = Some(now);

        let (roll_rate, pitch_rate, _yaw_rate) = gyro_rates.unwrap_or((0.0, 0.0, 0.0));

        let pitch_error = if self.pitch_hold_enabled {
            desired_pitch - current_pitch
        } else {
            0.0
        };
        let roll_error = if self.roll_hold_enabled {
            desired_roll - current_roll
        } else {
            0.0
        };

        let pitch_output = self.scale
            * self
                .pitch_pid
                .compute(pitch_error, pitch_rate, self.i_limit, dt, low_throttle);
        let roll_output = self.scale
            * self
                .roll_pid
                .compute(roll_error, roll_rate, self.i_limit, dt, low_throttle);

        (pitch_output, roll_output)
    }

    /// Check if controller has been initialized
    #[must_use]
    pub const fn is_active(&self) -> bool {
        self.last_time.is_some()
    }
}
