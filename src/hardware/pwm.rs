use crate::config::*;
use core::time::Duration;
use embassy_rp::Peri;
use embassy_rp::peripherals::{PIN_10, PIN_11, PIN_16, PIN_17, PIO0};
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
        self.elevon_left
            .write(Duration::from_micros(SERVO_CENTER_US.into()));
        self.elevon_right
            .write(Duration::from_micros(SERVO_CENTER_US.into()));
        self.engine_left
            .write(Duration::from_micros(ENGINE_MIN_PULSE_US.into()));
        self.engine_right
            .write(Duration::from_micros(ENGINE_MIN_PULSE_US.into()));
    }

    pub fn set_elevons(&mut self, left_us: u32, right_us: u32) {
        self.elevon_left
            .write(Duration::from_micros(left_us.into()));
        self.elevon_right
            .write(Duration::from_micros(right_us.into()));
    }

    pub fn set_engines(&mut self, left_us: u32, right_us: u32) {
        self.engine_left
            .write(Duration::from_micros(left_us.into()));
        self.engine_right.write(Duration::from_micros(
            (right_us + ENGINE_RIGHT_OFFSET_US).into(),
        ));
    }
}

pub struct PwmPins<'a> {
    pub elevon_left: Peri<'a, PIN_16>,
    pub elevon_right: Peri<'a, PIN_17>,
    pub engine_left: Peri<'a, PIN_10>,
    pub engine_right: Peri<'a, PIN_11>,
}
