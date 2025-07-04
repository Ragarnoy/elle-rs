use crate::config::*;
use crate::control::{arming::ArmingState, throttle::*};
use crate::hardware::imu::AttitudeData;
use crate::hardware::pwm::PwmOutputs;
use defmt::info;
use embassy_time::{Duration, Instant, Timer};
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
    last_packet_time: Instant,
}

impl<'a> FlightController<'a> {
    pub fn new(pwm: PwmOutputs<'a>) -> Self {
        Self {
            pwm,
            arming: ArmingState::new(),
            last_packet_time: Instant::now(),
        }
    }

    pub async fn initialize_escs(&mut self) {
        defmt::info!("ESC init: Starting");

        // Hold at minimum
        for _ in 0..200 {
            self.pwm
                .set_engines(ENGINE_MIN_PULSE_US, ENGINE_MIN_PULSE_US);
            Timer::after(Duration::from_millis(10)).await;
        }

        // Brief idle pulse
        defmt::info!("ESC init: Idle pulse");
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

        // Get pilot control inputs
        let pilot_inputs = ControlInputs::from_sbus_channels(&packet.channels);

        // Apply attitude stabilization if available and armed
        let final_inputs = if let Some(att) = attitude {
            if self.arming.armed {
                // TODO: Apply PID attitude correction here
                // For now, just use pilot inputs directly
                defmt::debug!(
                    "Attitude - Pitch: {}°, Roll: {}°, Controls - P:{} R:{} Y:{}",
                    (att.pitch * 180.0 / core::f32::consts::PI) as i16,
                    (att.roll * 180.0 / core::f32::consts::PI) as i16,
                    (pilot_inputs.pitch * 100.0) as i16,
                    (pilot_inputs.roll * 100.0) as i16,
                    (pilot_inputs.yaw * 100.0) as i16
                );

                pilot_inputs // Will be replaced with stabilized inputs later
            } else {
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

        // Debug output
        defmt::debug!(
            "Controls: P:{} R:{} Y:{} T:{} | Elevons: L:{}μs R:{}μs | Engines: L:{}μs R:{}μs",
            (final_inputs.pitch * 100.0) as i16,
            (final_inputs.roll * 100.0) as i16,
            (final_inputs.yaw * 100.0) as i16,
            (final_inputs.throttle * 100.0) as i16,
            elevon_outputs.left_us,
            elevon_outputs.right_us,
            left_thrust,
            right_thrust
        );
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
}
