//! PID attitude controller for flying wing stabilization
use embassy_time::Instant;

/// Generic PID controller with anti-windup
#[derive(Debug, Clone)]
pub struct PidController {
    // Gains
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,

    // State
    integral: f32,
    last_error: f32,
    last_time: Option<Instant>,

    // Limits
    pub output_limit: f32,      // Max output magnitude
    pub integral_limit: f32,    // Anti-windup limit
    pub derivative_filter: f32, // Low-pass filter coefficient (0-1)

    // Filtered derivative
    filtered_derivative: f32,
}

impl PidController {
    pub fn new(kp: f32, ki: f32, kd: f32) -> Self {
        Self {
            kp,
            ki,
            kd,
            integral: 0.0,
            last_error: 0.0,
            last_time: None,
            output_limit: 1.0,      // Default to normalized output
            integral_limit: 0.5,    // Limit integral to 50% of output
            derivative_filter: 0.1, // Moderate filtering
            filtered_derivative: 0.0,
        }
    }

    /// Reset the controller state
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.last_error = 0.0;
        self.last_time = None;
        self.filtered_derivative = 0.0;
    }

    /// Update the PID controller
    pub fn update(&mut self, setpoint: f32, measurement: f32, now: Instant) -> f32 {
        let error = setpoint - measurement;

        // Calculate dt
        let dt = if let Some(last) = self.last_time {
            let elapsed = now.duration_since(last);
            elapsed.as_micros() as f32 / 1_000_000.0
        } else {
            // First iteration, use a nominal dt
            0.01 // 10ms
        };

        // Proportional term
        let p_term = self.kp * error;

        // Integral term with anti-windup
        self.integral += error * dt;
        self.integral = self
            .integral
            .clamp(-self.integral_limit, self.integral_limit);
        let i_term = self.ki * self.integral;

        // Derivative term with filtering
        let derivative = if dt > 0.0 {
            (error - self.last_error) / dt
        } else {
            0.0
        };

        // Low-pass filter the derivative to reduce noise
        self.filtered_derivative = self.derivative_filter * derivative
            + (1.0 - self.derivative_filter) * self.filtered_derivative;
        let d_term = self.kd * self.filtered_derivative;

        // Calculate total output
        let output = p_term + i_term + d_term;

        // Update state
        self.last_error = error;
        self.last_time = Some(now);

        // Apply output limits
        output.clamp(-self.output_limit, self.output_limit)
    }

    /// Update gains
    pub fn set_gains(&mut self, kp: f32, ki: f32, kd: f32) {
        self.kp = kp;
        self.ki = ki;
        self.kd = kd;
    }

    /// Check if controller has been initialized (has received at least one update)
    pub fn is_active(&self) -> bool {
        self.last_time.is_some()
    }
}

/// Attitude controller combining multiple PID controllers
pub struct AttitudeController {
    pub pitch_pid: PidController,
    pub roll_pid: PidController,

    // Control flags
    pub enabled: bool,
    pub pitch_hold_enabled: bool,
    pub roll_hold_enabled: bool,
}

impl Default for AttitudeController {
    fn default() -> Self {
        Self::new()
    }
}

impl AttitudeController {
    pub fn new() -> Self {
        // Conservative initial gains - tune these based on your aircraft
        let mut pitch_pid = PidController::new(
            0.5,  // Kp - start conservative
            0.05, // Ki - small integral
            0.1,  // Kd - some damping
        );
        pitch_pid.output_limit = 0.5; // Limit to 50% control authority
        pitch_pid.integral_limit = 0.2; // Limit integral windup

        let mut roll_pid = PidController::new(
            0.4,  // Kp
            0.05, // Ki
            0.08, // Kd
        );
        roll_pid.output_limit = 0.5;
        roll_pid.integral_limit = 0.2;

        Self {
            pitch_pid,
            roll_pid,
            enabled: false,
            pitch_hold_enabled: true,
            roll_hold_enabled: false, // Usually just level wings for flying wings
        }
    }

    /// Reset all controllers
    pub fn reset(&mut self) {
        self.pitch_pid.reset();
        self.roll_pid.reset();
    }

    /// Calculate attitude corrections
    pub fn update(
        &mut self,
        desired_pitch: f32,
        desired_roll: f32,
        current_pitch: f32,
        current_roll: f32,
        now: Instant,
    ) -> (f32, f32) {
        if !self.enabled {
            return (0.0, 0.0);
        }

        let pitch_correction = if self.pitch_hold_enabled {
            self.pitch_pid.update(desired_pitch, current_pitch, now)
        } else {
            0.0
        };

        let roll_correction = if self.roll_hold_enabled {
            self.roll_pid.update(desired_roll, current_roll, now)
        } else {
            0.0
        };

        (pitch_correction, roll_correction)
    }

    /// Check if controller has been initialized
    pub fn is_active(&self) -> bool {
        self.pitch_pid.is_active() || self.roll_pid.is_active()
    }
}
