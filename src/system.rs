use crate::config::*;
use crate::control::{arming::ArmingState, mixing::*, throttle::*};
use crate::hardware::imu::AttitudeData;
use crate::hardware::pwm::PwmOutputs;
use defmt::info;
use embassy_time::{Duration, Instant, Timer};
use sbus_rs::SbusPacket;

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

    pub fn update(&mut self, packet: &SbusPacket) {
        self.last_packet_time = Instant::now();

        // Update arming state
        self.arming
            .update(packet.channels[ENGINE_CH], packet.flags.failsafe);

        // Update elevons
        let elevon_left = sbus_to_pulse_us(
            packet.channels[ELEVON_LEFT_CH],
            SERVO_MIN_PULSE_US,
            SERVO_MAX_PULSE_US,
        );
        let elevon_right = sbus_to_pulse_us(
            packet.channels[ELEVON_RIGHT_CH],
            SERVO_MIN_PULSE_US,
            SERVO_MAX_PULSE_US,
        );
        info!("LEFT: {} RIGHT: {}", elevon_left, elevon_right);
        self.pwm.set_elevons(elevon_left, elevon_right);

        // Calculate engine thrust
        let base_thrust = throttle_curve(packet.channels[ENGINE_CH]);
        let (left_mult, right_mult) = calculate_differential(packet.channels[DIFFERENTIAL_CH]);

        let (left_thrust, right_thrust) = if self.arming.armed {
            apply_differential(base_thrust, left_mult, right_mult)
        } else {
            (ENGINE_MIN_PULSE_US, ENGINE_MIN_PULSE_US)
        };

        self.pwm.set_engines(left_thrust, right_thrust);
    }

    pub fn update_with_attitude(&mut self, packet: &SbusPacket, attitude: Option<&AttitudeData>) {
        self.last_packet_time = Instant::now();

        // Update arming state
        self.arming
            .update(packet.channels[ENGINE_CH], packet.flags.failsafe);

        // Get base elevon positions from pilot input
        let elevon_left = sbus_to_pulse_us(
            packet.channels[ELEVON_LEFT_CH],
            SERVO_MIN_PULSE_US,
            SERVO_MAX_PULSE_US,
        );
        let elevon_right = sbus_to_pulse_us(
            packet.channels[ELEVON_RIGHT_CH],
            SERVO_MIN_PULSE_US,
            SERVO_MAX_PULSE_US,
        );

        // Apply attitude stabilization if available and armed
        let (final_left, final_right) = if let Some(att) = attitude {
            if self.arming.armed {
                // For now, just log the attitude - PID control comes next
                defmt::debug!(
                    "Attitude - Pitch: {}°, Roll: {}°",
                    (att.pitch * 180.0 / core::f32::consts::PI) as i16,
                    (att.roll * 180.0 / core::f32::consts::PI) as i16
                );

                // TODO: Apply PID control here
                // let correction = self.attitude_controller.update(target_pitch, att.pitch);
                // apply_attitude_correction(elevon_left, elevon_right, correction)

                (elevon_left, elevon_right)
            } else {
                (elevon_left, elevon_right)
            }
        } else {
            (elevon_left, elevon_right)
        };

        self.pwm.set_elevons(final_left, final_right);

        // Engine control remains the same
        let base_thrust = throttle_curve(packet.channels[ENGINE_CH]);
        let (left_mult, right_mult) = calculate_differential(packet.channels[DIFFERENTIAL_CH]);

        let (left_thrust, right_thrust) = if self.arming.armed {
            apply_differential(base_thrust, left_mult, right_mult)
        } else {
            (ENGINE_MIN_PULSE_US, ENGINE_MIN_PULSE_US)
        };

        self.pwm.set_engines(left_thrust, right_thrust);
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
