use crate::config::*;
use crate::control::{arming::ArmingState, pid::AttitudeController, throttle::*};
use crate::hardware::imu::AttitudeData;
use crate::hardware::pwm::PwmOutputs;
use defmt::info;
use embassy_time::{Duration, Instant, Timer};
use free_flight_stabilization::FlightStabilizerConfig;
use sbus_rs::SbusPacket;

#[cfg(feature = "mixing")]
use crate::control::mixing::{
    elevons::{ControlInputs, mix_elevons},
    yaw::{apply_differential_thrust, calculate_yaw_differential},
};

#[cfg(not(feature = "mixing"))]
use crate::control::mixing::{apply_differential, calculate_differential};
#[cfg(not(feature = "mixing"))]
use crate::control::throttle::sbus_to_pulse_us;

#[derive(Debug, Clone, Copy, PartialEq, defmt::Format)]
pub enum ControlMode {
    Manual,    // Full manual control (~306)
    Mixed,     // Pilot + Autopilot blend (~1000)
    Autopilot, // Full autopilot control (~1694)
}

pub struct FlightController<'a> {
    pwm: PwmOutputs<'a>,
    arming: ArmingState,
    attitude_controller: AttitudeController,
    last_packet_time: Instant,
    last_attitude: Option<AttitudeData>,
    // Smoothed setpoints for attitude hold
    filtered_pitch_setpoint: f32,
    filtered_roll_setpoint: f32,
    last_setpoint_update: Option<Instant>,
    // Control mode tracking
    current_control_mode: ControlMode,
}

impl<'a> FlightController<'a> {
    pub fn new(pwm: PwmOutputs<'a>) -> Self {
        // Use conservative gains similar to free-flight-stabilization example
        let mut config = FlightStabilizerConfig::<f32>::new();

        // Set the PID gains for roll, pitch, and yaw (from the example you provided)
        config.kp_roll = ROLL_KP;
        config.ki_roll = ROLL_KI;
        config.kd_roll = ROLL_KD;
        config.kp_pitch = PITCH_KP;
        config.ki_pitch = PITCH_KI;
        config.kd_pitch = PITCH_KD;
        config.kp_yaw = 0.3;
        config.ki_yaw = 0.05;
        config.kd_yaw = 0.00015;

        // Set the upper limit for the integral term to prevent windup
        config.i_limit = 25.0;

        // Set the scale to adjust the PID outputs to the actuator range
        config.scale = 0.015; // Increased for more responsive but still smooth control

        let mut attitude_controller = AttitudeController::with_config(config);
        attitude_controller.roll_hold_enabled = true; // Enable roll hold for CH8 control

        Self {
            pwm,
            arming: ArmingState::default(),
            attitude_controller,
            last_packet_time: Instant::now(),
            last_attitude: None,
            filtered_pitch_setpoint: 0.0,
            filtered_roll_setpoint: 0.0,
            last_setpoint_update: None,
            current_control_mode: ControlMode::Manual,
        }
    }

    pub async fn initialize_escs(&mut self) {
        info!("ESC init: Starting");

        // Hold at minimum
        for _ in 0..200 {
            self.pwm
                .set_engines(ENGINE_MIN_PULSE_US, ENGINE_MIN_PULSE_US);
            Timer::after(Duration::from_millis(10)).await;
        }

        // Brief idle pulse
        info!("ESC init: Idle pulse");
        for _ in 0..50 {
            self.pwm
                .set_engines(ENGINE_IDLE_PULSE_US, ENGINE_IDLE_PULSE_US);
            Timer::after(Duration::from_millis(10)).await;
        }

        // Return to minimum
        for _ in 0..100 {
            self.pwm
                .set_engines(ENGINE_MIN_PULSE_US, ENGINE_MIN_PULSE_US);
            Timer::after(Duration::from_millis(10)).await;
        }

        info!("ESC init: Complete");
    }

    /// Determine control mode from CH5 3-state switch
    fn determine_control_mode(&self, ch5_value: u16) -> ControlMode {
        if ch5_value < MANUAL_MODE_THRESHOLD {
            ControlMode::Manual
        } else if ch5_value < MIXED_MODE_THRESHOLD {
            ControlMode::Mixed
        } else {
            ControlMode::Autopilot
        }
    }

    /// Update method with feature flag support
    pub fn update(&mut self, packet: &SbusPacket) {
        self.last_packet_time = Instant::now();

        #[cfg(feature = "mixing")]
        {
            // Update arming state using throttle channel
            self.arming
                .update(packet.channels[THROTTLE_CH], packet.flags.failsafe);
            self.update_with_mixing(packet, None);
        }

        #[cfg(not(feature = "mixing"))]
        {
            // Update arming state using legacy engine channel
            self.arming
                .update(packet.channels[ENGINE_CH], packet.flags.failsafe);
            self.update_direct_elevons(packet);
        }
    }

    /// New update method with flight control mixing
    #[cfg(feature = "mixing")]
    pub fn update_with_mixing(&mut self, packet: &SbusPacket, attitude: Option<&AttitudeData>) {
        self.last_packet_time = Instant::now();

        // Update arming state
        self.arming
            .update(packet.channels[THROTTLE_CH], packet.flags.failsafe);

        // Determine control mode from CH5 3-state switch
        let new_control_mode = self.determine_control_mode(packet.channels[ATTITUDE_ENABLE_CH]);
        if new_control_mode != self.current_control_mode {
            info!(
                "Control mode changed: {:?} -> {:?}",
                self.current_control_mode, new_control_mode
            );
            self.current_control_mode = new_control_mode;
        }

        // Enable attitude controller for Mixed and Autopilot modes
        self.attitude_controller.enabled = (self.current_control_mode == ControlMode::Mixed
            || self.current_control_mode == ControlMode::Autopilot)
            && self.arming.armed;

        // Get raw desired angles from SBUS channels
        let ch6_normalized = (packet.channels[ATTITUDE_PITCH_SETPOINT_CH] as f32 - 1023.5) / 1023.5;
        let raw_pitch_deg = ATTITUDE_PITCH_MIN_DEG
            + (ch6_normalized + 1.0) * 0.5 * (ATTITUDE_PITCH_MAX_DEG - ATTITUDE_PITCH_MIN_DEG);

        let ch8_normalized = (packet.channels[ATTITUDE_ROLL_SETPOINT_CH] as f32 - 1023.5) / 1023.5;
        let raw_roll_deg = ATTITUDE_ROLL_MIN_DEG
            + (ch8_normalized + 1.0) * 0.5 * (ATTITUDE_ROLL_MAX_DEG - ATTITUDE_ROLL_MIN_DEG);

        // Apply setpoint smoothing and rate limiting
        let now = Instant::now();
        let dt = if let Some(last_update) = self.last_setpoint_update {
            now.duration_since(last_update).as_micros() as f32 / 1_000_000.0
        } else {
            CONTROL_LOOP_DT // First update, use nominal dt
        };
        self.last_setpoint_update = Some(now);

        // Low-pass filter for smooth setpoint transitions
        self.filtered_pitch_setpoint +=
            SETPOINT_FILTER_ALPHA * (raw_pitch_deg - self.filtered_pitch_setpoint);
        self.filtered_roll_setpoint +=
            SETPOINT_FILTER_ALPHA * (raw_roll_deg - self.filtered_roll_setpoint);

        // Rate limiting for setpoint changes (secondary smoothing)
        let max_change_deg = MAX_SETPOINT_RATE_DEG_S * dt;
        let pitch_change =
            (raw_pitch_deg - self.filtered_pitch_setpoint).clamp(-max_change_deg, max_change_deg);
        let roll_change =
            (raw_roll_deg - self.filtered_roll_setpoint).clamp(-max_change_deg, max_change_deg);

        self.filtered_pitch_setpoint += pitch_change;
        self.filtered_roll_setpoint += roll_change;

        // Convert smoothed setpoints to radians
        let desired_pitch_rad = self.filtered_pitch_setpoint * core::f32::consts::PI / 180.0;
        let desired_roll_rad = self.filtered_roll_setpoint * core::f32::consts::PI / 180.0;
        // Get pilot control inputs
        let pilot_inputs = ControlInputs::from_sbus_channels(&packet.channels);

        // Store new attitude if provided
        if let Some(att) = attitude {
            self.last_attitude = Some(*att);
        }

        // Apply control mode logic
        let final_inputs = match self.current_control_mode {
            ControlMode::Manual => {
                // Full manual control - use pilot inputs directly
                info!("Manual Mode - Direct pilot control");
                // Reset PID when in manual mode
                if self.attitude_controller.is_active() {
                    self.attitude_controller.reset();
                }
                pilot_inputs
            }

            ControlMode::Mixed => {
                // Mixed mode - blend pilot and autopilot
                if let Some(effective_attitude) = attitude.or(self.last_attitude.as_ref()) {
                    let gyro_rates = Some((
                        effective_attitude.roll_rate,
                        effective_attitude.pitch_rate,
                        effective_attitude.yaw_rate,
                    ));
                    let (pitch_correction, roll_correction) = self.attitude_controller.update(
                        desired_pitch_rad,
                        desired_roll_rad,
                        effective_attitude.pitch,
                        effective_attitude.roll,
                        gyro_rates,
                        Instant::now(),
                    );

                    // Blend pilot inputs with autopilot corrections
                    let mut mixed_inputs = pilot_inputs;
                    mixed_inputs.pitch = (1.0 - MIXED_MODE_AUTOPILOT_WEIGHT) * pilot_inputs.pitch
                        + MIXED_MODE_AUTOPILOT_WEIGHT * pitch_correction;
                    mixed_inputs.roll = (1.0 - MIXED_MODE_AUTOPILOT_WEIGHT) * pilot_inputs.roll
                        + MIXED_MODE_AUTOPILOT_WEIGHT * roll_correction;

                    info!(
                        "Mixed Mode ({}% Auto) - P: {}°/{}° R: {}°/{}°",
                        (MIXED_MODE_AUTOPILOT_WEIGHT * 100.0) as i16,
                        self.filtered_pitch_setpoint as i16,
                        (effective_attitude.pitch * 180.0 / core::f32::consts::PI) as i16,
                        self.filtered_roll_setpoint as i16,
                        (effective_attitude.roll * 180.0 / core::f32::consts::PI) as i16
                    );

                    mixed_inputs
                } else {
                    // No attitude data - use pilot inputs only
                    pilot_inputs
                }
            }

            ControlMode::Autopilot => {
                // Full autopilot control - use attitude corrections only
                if let Some(effective_attitude) = attitude.or(self.last_attitude.as_ref()) {
                    let gyro_rates = Some((
                        effective_attitude.roll_rate,
                        effective_attitude.pitch_rate,
                        effective_attitude.yaw_rate,
                    ));
                    let (pitch_correction, roll_correction) = self.attitude_controller.update(
                        desired_pitch_rad,
                        desired_roll_rad,
                        effective_attitude.pitch,
                        effective_attitude.roll,
                        gyro_rates,
                        Instant::now(),
                    );

                    // Use autopilot corrections only, keep pilot throttle and yaw
                    let mut auto_inputs = pilot_inputs;
                    auto_inputs.pitch = pitch_correction;
                    auto_inputs.roll = roll_correction;

                    info!(
                        "Autopilot Mode - P: {}°/{}° R: {}°/{}° Corrections: P:{} R:{}",
                        self.filtered_pitch_setpoint as i16,
                        (effective_attitude.pitch * 180.0 / core::f32::consts::PI) as i16,
                        self.filtered_roll_setpoint as i16,
                        (effective_attitude.roll * 180.0 / core::f32::consts::PI) as i16,
                        (pitch_correction * 100.0) as i16,
                        (roll_correction * 100.0) as i16
                    );

                    auto_inputs
                } else {
                    // No attitude data - fallback to manual
                    info!("Autopilot Mode - No attitude data, using manual control");
                    pilot_inputs
                }
            }
        };

        // Mix controls to get surface positions
        let elevon_outputs = mix_elevons(&final_inputs);
        let yaw_factors = calculate_yaw_differential(final_inputs.yaw);

        // Set elevon positions
        self.pwm
            .set_elevons_with_trim(elevon_outputs.left_us, elevon_outputs.right_us);

        // Calculate engine thrust with differential
        let base_thrust = throttle_curve((final_inputs.throttle * 2047.0) as u16);

        let (left_thrust, right_thrust) = if self.arming.armed {
            apply_differential_thrust(base_thrust, &yaw_factors)
        } else {
            (ENGINE_MIN_PULSE_US, ENGINE_MIN_PULSE_US)
        };

        self.pwm.set_engines(left_thrust, right_thrust);
    }

    /// Legacy direct elevon control method
    #[cfg(not(feature = "mixing"))]
    fn update_direct_elevons(&mut self, packet: &SbusPacket) {
        let elevon_left_us = sbus_to_pulse_us(
            packet.channels[ELEVON_LEFT_CH],
            SERVO_MIN_PULSE_US,
            SERVO_MAX_PULSE_US,
        );
        let elevon_right_us = sbus_to_pulse_us(
            packet.channels[ELEVON_RIGHT_CH],
            SERVO_MIN_PULSE_US,
            SERVO_MAX_PULSE_US,
        );

        // Set elevon positions
        self.pwm.set_elevons(elevon_left_us, elevon_right_us);

        // Calculate engine thrust
        let base_thrust = throttle_curve(packet.channels[ENGINE_CH]);
        let (left_mult, right_mult) = calculate_differential(packet.channels[DIFFERENTIAL_CH]);

        let (left_thrust, right_thrust) = if self.arming.armed {
            apply_differential(base_thrust, left_mult, right_mult)
        } else {
            (ENGINE_MIN_PULSE_US, ENGINE_MIN_PULSE_US)
        };

        self.pwm.set_engines(left_thrust, right_thrust);

        info!(
            "Direct - LEFT: {}μs RIGHT: {}μs",
            elevon_left_us, elevon_right_us
        );
    }

    /// Updated method that uses mixing by default when feature is enabled
    pub fn update_with_attitude(&mut self, packet: &SbusPacket, attitude: Option<&AttitudeData>) {
        #[cfg(feature = "mixing")]
        {
            self.update_with_mixing(packet, attitude);
        }

        #[cfg(not(feature = "mixing"))]
        {
            // Legacy behavior - attitude is ignored in direct mode
            self.update(packet);
        }
    }

    pub fn check_failsafe(&mut self) {
        if self.last_packet_time.elapsed() > Duration::from_millis(SBUS_TIMEOUT_MS) {
            self.arming.signal_loss();
            self.attitude_controller.reset(); // Reset PID on signal loss
            self.apply_failsafe();
        }
    }

    pub fn apply_failsafe(&mut self) {
        self.pwm.set_safe_positions();
    }

    pub fn is_armed(&self) -> bool {
        self.arming.armed
    }

    pub fn is_failsafe(&self) -> bool {
        self.arming.failsafe_active
    }

    pub fn is_attitude_enabled(&self) -> bool {
        self.attitude_controller.enabled
    }

    pub fn current_control_mode(&self) -> ControlMode {
        self.current_control_mode
    }
}
