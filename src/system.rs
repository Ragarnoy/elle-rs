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

pub struct FlightController<'a> {
    pwm: PwmOutputs<'a>,
    arming: ArmingState,
    attitude_controller: AttitudeController,
    last_packet_time: Instant,
    last_attitude: Option<AttitudeData>,
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
        config.scale = 0.01;

        Self {
            pwm,
            arming: ArmingState::default(),
            attitude_controller: AttitudeController::with_config(config),
            last_packet_time: Instant::now(),
            last_attitude: None,
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

        // Check attitude control enable (CH5)
        let attitude_enabled = packet.channels[ATTITUDE_ENABLE_CH] > ATTITUDE_ENABLE_THRESHOLD;
        self.attitude_controller.enabled = attitude_enabled && self.arming.armed;

        // Get desired pitch from CH6 (map SBUS to angle)
        let ch6_normalized = (packet.channels[ATTITUDE_SETPOINT_CH] as f32 - 1023.5) / 1023.5;
        let desired_pitch_deg = ATTITUDE_PITCH_MIN_DEG
            + (ch6_normalized + 1.0) * 0.5 * (ATTITUDE_PITCH_MAX_DEG - ATTITUDE_PITCH_MIN_DEG);
        let desired_pitch_rad = desired_pitch_deg * core::f32::consts::PI / 180.0;
        // Get pilot control inputs
        let mut pilot_inputs = ControlInputs::from_sbus_channels(&packet.channels);

        // Store new attitude if provided
        if let Some(att) = attitude {
            self.last_attitude = Some(*att);
        }

        // Use the last known attitude if available, otherwise use direct pilot inputs
        let final_inputs =
            if let Some(effective_attitude) = attitude.or(self.last_attitude.as_ref()) {
                if self.attitude_controller.enabled {
                    // Get attitude corrections with gyro rates
                    let gyro_rates = Some((
                        effective_attitude.roll_rate,
                        effective_attitude.pitch_rate,
                        effective_attitude.yaw_rate,
                    ));
                    let (pitch_correction, roll_correction) = self.attitude_controller.update(
                        desired_pitch_rad,
                        0.0, // Desired roll = level
                        effective_attitude.pitch,
                        effective_attitude.roll,
                        gyro_rates,
                        Instant::now(),
                    );

                    // Use only attitude corrections when enabled
                    pilot_inputs.pitch = pitch_correction;
                    pilot_inputs.roll = roll_correction;

                    info!(
                        "Attitude Hold - Target: {}° Current: {}° Correction: {}",
                        desired_pitch_deg as i16,
                        (effective_attitude.pitch * 180.0 / core::f32::consts::PI) as i16,
                        (pitch_correction * 100.0) as i16
                    );

                    pilot_inputs
                } else {
                    // Reset PID when disabled
                    if self.attitude_controller.is_active() {
                        self.attitude_controller.reset();
                    }
                    pilot_inputs
                }
            } else {
                pilot_inputs
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
}
