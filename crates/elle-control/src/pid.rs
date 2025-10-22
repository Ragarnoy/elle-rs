//! Attitude controller for flying wing stabilization using free-flight-stabilization crate
use defmt::warn;
use elle_config::CONTROL_LOOP_DT;
use embassy_time::Instant;
use free_flight_stabilization::{AngleStabilizer, FlightStabilizer, FlightStabilizerConfig};

/// Attitude controller using free-flight-stabilization crate
pub struct AttitudeController {
    stabilizer: AngleStabilizer<f32>,
    last_time: Option<Instant>,

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
        Self::with_config(FlightStabilizerConfig::new())
    }

    /// Create with custom configuration (matches free-flight-stabilization pattern)
    pub fn with_config(config: FlightStabilizerConfig<f32>) -> Self {
        let stabilizer = AngleStabilizer::with_config(config);

        Self {
            stabilizer,
            last_time: None,
            enabled: false,
            pitch_hold_enabled: true,
            roll_hold_enabled: false, // Usually just level wings for flying wings
        }
    }

    /// Reset the controller state
    pub fn reset(&mut self) {
        // Reset by creating a new stabilizer with default config
        // Since we can't access the current config, use default
        self.stabilizer = AngleStabilizer::new();
        self.last_time = None;
    }

    /// Control method compatible with free-flight-stabilization crate interface
    pub fn control(
        &mut self,
        set_point: (f32, f32, f32),    // (roll, pitch, yaw) setpoints
        imu_attitude: (f32, f32, f32), // (roll, pitch, yaw) current attitude
        gyro_rate: (f32, f32, f32),    // (roll_rate, pitch_rate, yaw_rate)
        dt: f32,                       // time step
        low_throttle: bool,            // low throttle flag
    ) -> (f32, f32, f32) {
        if !self.enabled {
            return (0.0, 0.0, 0.0);
        }

        // Use free-flight-stabilization crate directly
        self.stabilizer
            .control(set_point, imu_attitude, gyro_rate, dt, low_throttle)
    }

    /// Calculate attitude corrections (existing interface for compatibility)
    pub fn update(
        &mut self,
        desired_pitch: f32,
        desired_roll: f32,
        current_pitch: f32,
        current_roll: f32,
        gyro_rates: Option<(f32, f32, f32)>, // (roll_rate, pitch_rate, yaw_rate)
        now: Instant,
    ) -> (f32, f32) {
        if !self.enabled {
            return (0.0, 0.0);
        }

        // Use fixed dt for consistent control performance
        let dt = CONTROL_LOOP_DT;

        // Update timing for diagnostics (optional)
        if let Some(last) = self.last_time {
            let actual_dt = now.duration_since(last).as_micros() as f32 / 1_000_000.0;

            // Warn if timing is significantly off (>30% deviation)
            if (actual_dt - dt).abs() / dt > 0.3 {
                warn!(
                    "Timing deviation: expected {}ms, got {}ms",
                    (dt * 1000.0) as u32,
                    (actual_dt * 1000.0) as u32
                );
            }
        }

        self.last_time = Some(now);

        // Use the control interface internally
        let set_point = (
            if self.roll_hold_enabled {
                desired_roll
            } else {
                current_roll
            },
            if self.pitch_hold_enabled {
                desired_pitch
            } else {
                current_pitch
            },
            0.0, // No yaw control for flying wing
        );

        let current_attitude = (current_roll, current_pitch, 0.0);

        // Use provided gyro rates or fall back to zeros
        let gyro_rates = gyro_rates.unwrap_or((0.0, 0.0, 0.0));

        let low_throttle = false;

        let (roll_output, pitch_output, _yaw_output) =
            self.control(set_point, current_attitude, gyro_rates, dt, low_throttle);

        (pitch_output, roll_output)
    }

    /// Check if controller has been initialized
    pub fn is_active(&self) -> bool {
        self.last_time.is_some()
    }
}
