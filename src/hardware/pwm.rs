use crate::config::*;
use core::time::Duration;
use embassy_rp::Peri;
use embassy_rp::peripherals::{PIN_11, PIN_12, PIN_14, PIN_15, PIO0};
use embassy_rp::pio::{Common, StateMachine};
use embassy_rp::pio_programs::pwm::{PioPwm, PioPwmProgram};

pub struct PwmOutputs<'a> {
    pub elevon_left: PioPwm<'a, PIO0, 0>,
    pub elevon_right: PioPwm<'a, PIO0, 1>,
    pub engine_left: PioPwm<'a, PIO0, 2>,
    pub engine_right: PioPwm<'a, PIO0, 3>,
}

impl<'a> PwmOutputs<'a> {
    pub fn new(
        common: &mut Common<'a, PIO0>,
        sm0: StateMachine<'a, PIO0, 0>,
        sm1: StateMachine<'a, PIO0, 1>,
        sm2: StateMachine<'a, PIO0, 2>,
        sm3: StateMachine<'a, PIO0, 3>,
        pins: &'a mut PwmPins,
    ) -> Self {
        let prg = PioPwmProgram::new(common);

        let mut elevon_left = PioPwm::new(common, sm0, pins.elevon_left.reborrow(), &prg);
        let mut elevon_right = PioPwm::new(common, sm1, pins.elevon_right.reborrow(), &prg);
        let mut engine_left = PioPwm::new(common, sm2, pins.engine_left.reborrow(), &prg);
        let mut engine_right = PioPwm::new(common, sm3, pins.engine_right.reborrow(), &prg);

        // Configure periods
        let period = Duration::from_micros(REFRESH_INTERVAL_US.into());
        elevon_left.set_period(period);
        elevon_right.set_period(period);
        engine_left.set_period(period);
        engine_right.set_period(period);

        // Start all outputs
        elevon_left.start();
        elevon_right.start();
        engine_left.start();
        engine_right.start();

        Self {
            elevon_left,
            elevon_right,
            engine_left,
            engine_right,
        }
    }

    pub fn set_safe_positions(&mut self) {
        // Use individual trim-adjusted center positions
        self.elevon_left
            .write(Duration::from_micros(ELEVON_LEFT_CENTER_US.into()));
        self.elevon_right
            .write(Duration::from_micros(ELEVON_RIGHT_CENTER_US.into()));
        self.engine_left
            .write(Duration::from_micros(ENGINE_MIN_PULSE_US.into()));
        self.engine_right
            .write(Duration::from_micros(ENGINE_MIN_PULSE_US.into()));
    }

    /// Set elevons without trim applied (legacy)
    pub fn set_elevons(&mut self, left_us: u32, right_us: u32) {
        self.elevon_left
            .write(Duration::from_micros(left_us.into()));
        self.elevon_right
            .write(Duration::from_micros(right_us.into()));
    }

    /// Set elevons with trim applied (recommended method)
    pub fn set_elevons_with_trim(&mut self, left_us: u32, right_us: u32) {
        // Apply trim adjustments
        let left_trimmed = apply_elevon_trim(left_us, ELEVON_LEFT_TRIM_US);
        let right_trimmed = apply_elevon_trim(right_us, ELEVON_RIGHT_TRIM_US);

        self.elevon_left
            .write(Duration::from_micros(left_trimmed.into()));
        self.elevon_right
            .write(Duration::from_micros(right_trimmed.into()));

        // Debug output for trim monitoring
        if ELEVON_LEFT_TRIM_US != 0 || ELEVON_RIGHT_TRIM_US != 0 {
            defmt::trace!(
                "Elevon trim: L:{}μs→{}μs R:{}μs→{}μs",
                left_us,
                left_trimmed,
                right_us,
                right_trimmed
            );
        }
    }

    pub fn set_engines(&mut self, left_us: u32, right_us: u32) {
        self.engine_left
            .write(Duration::from_micros(left_us.into()));
        self.engine_right.write(Duration::from_micros(
            (right_us + ENGINE_RIGHT_OFFSET_US).into(),
        ));
    }
}

/// Apply trim adjustment to elevon position
fn apply_elevon_trim(base_us: u32, trim_us: i32) -> u32 {
    // Clamp trim to safe bounds
    let clamped_trim = trim_us.clamp(-MAX_TRIM_US, MAX_TRIM_US);

    // Apply trim and ensure result stays within servo bounds
    let trimmed = (base_us as i32 + clamped_trim) as u32;
    trimmed.clamp(SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US)
}

pub struct PwmPins<'a> {
    pub elevon_left: Peri<'a, PIN_15>,
    pub elevon_right: Peri<'a, PIN_14>,
    pub engine_left: Peri<'a, PIN_12>,
    pub engine_right: Peri<'a, PIN_11>,
}
