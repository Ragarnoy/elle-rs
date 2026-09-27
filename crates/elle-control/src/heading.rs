//! Heading-hold outer loop: heading error (AHRS yaw, radians) -> roll setpoint (degrees).
//!
//! Sits ahead of `AttitudeController`: its output is fed in as `desired_roll`
//! (after degrees->radians conversion) instead of a stick-derived setpoint.
use core::f32::consts::PI;
use elle_config::CONTROL_LOOP_DT;

/// Shortest-path heading error in radians, wrapped to (-PI, PI].
///
/// `target_rad`/`current_rad` are both AHRS yaw in radians (any range); the
/// result is the signed error you'd get by always turning the short way
/// around the compass (e.g. target=350deg, current=10deg -> ~-20deg, not ~-340deg).
#[must_use]
fn wrap_heading_error_rad(target_rad: f32, current_rad: f32) -> f32 {
    let mut err = (target_rad - current_rad) % (2.0 * PI);
    if err > PI {
        err -= 2.0 * PI;
    } else if err < -PI {
        err += 2.0 * PI;
    }
    err
}

/// P/PI controller producing a clamped, slew-limited roll (bank angle) setpoint
/// in degrees from a heading error in radians.
#[derive(Clone, Copy)]
pub struct HeadingController {
    kp: f32,
    ki: f32,
    integral: f32,
    i_limit_deg: f32,
    max_roll_deg: f32,
    max_roll_rate_deg_s: f32,
    last_output_deg: f32,
}

impl HeadingController {
    #[must_use]
    pub const fn new(
        kp: f32,
        ki: f32,
        i_limit_deg: f32,
        max_roll_deg: f32,
        max_roll_rate_deg_s: f32,
    ) -> Self {
        Self {
            kp,
            ki,
            integral: 0.0,
            i_limit_deg,
            max_roll_deg,
            max_roll_rate_deg_s,
            last_output_deg: 0.0,
        }
    }

    /// Clears integrator and slew-limiter state. Call on engage and on disengage.
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.last_output_deg = 0.0;
    }

    /// Returns the heading error in degrees (shortest path), for telemetry/observability.
    #[must_use]
    pub fn heading_error_deg(&self, target_rad: f32, current_rad: f32) -> f32 {
        wrap_heading_error_rad(target_rad, current_rad).to_degrees()
    }

    /// Computes the next roll setpoint (degrees), clamped to `max_roll_deg` and
    /// slew-limited to `max_roll_rate_deg_s` per `CONTROL_LOOP_DT` tick.
    pub fn update(&mut self, target_rad: f32, current_rad: f32) -> f32 {
        let error_deg = self.heading_error_deg(target_rad, current_rad);

        let i_limit = if self.i_limit_deg > 0.0 {
            self.i_limit_deg
        } else {
            0.0
        };
        let sum = self.integral + error_deg * CONTROL_LOOP_DT;
        self.integral = sum.clamp(-i_limit, i_limit);

        let raw_deg = self.kp * error_deg + self.ki * self.integral;
        let clamped_deg = raw_deg.clamp(-self.max_roll_deg, self.max_roll_deg);

        let max_step = self.max_roll_rate_deg_s * CONTROL_LOOP_DT;
        let delta = (clamped_deg - self.last_output_deg).clamp(-max_step, max_step);
        self.last_output_deg += delta;
        self.last_output_deg
    }
}
