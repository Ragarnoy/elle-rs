//! Elevon servo outputs on hardware PWM slice 6 (PIN_12 = A, PIN_13 = B).
//!
//! The slice's compare register is double-buffered and latches at counter wrap,
//! so each frame carries the most recent command, never more than one frame old.
//! The PIO PWM this replaced queued commands in a 4-deep TX FIFO: written every
//! control tick but drained once per 20 ms frame, the FIFO stayed full, new
//! writes were dropped, and the servos ran ~70–80 ms behind the controller.
use elle_config::*;
use embassy_rp::Peri;
use embassy_rp::peripherals::{PIN_12, PIN_13, PWM_SLICE6};
use embassy_rp::pwm::{Config, Pwm};
use fixed::FixedU16;
use fixed::types::extra::U4;

/// PWM counter rate: one count per microsecond, so compare values are pulse µs.
const PWM_TICK_HZ: u32 = 1_000_000;

// The slice counter is 16-bit, and a pulse must fit inside its frame.
const _: () = assert!(REFRESH_INTERVAL_US >= 1 && REFRESH_INTERVAL_US <= 1 << 16);
const _: () = assert!(SERVO_MAX_PULSE_US < REFRESH_INTERVAL_US);

pub struct PwmOutputs<'a> {
    slice: Pwm<'a>,
    config: Config,
}

impl<'a> PwmOutputs<'a> {
    pub fn new(pins: PwmPins<'a>) -> Self {
        let sys_hz = embassy_rp::clocks::clk_sys_freq();
        let mut config = Config::default();
        // 8.4 fixed-point divider: sys_hz / 1 MHz, in sixteenths.
        let div_bits = u64::from(sys_hz) * 16 / u64::from(PWM_TICK_HZ);
        assert!(
            (16..=0xFFF).contains(&div_bits),
            "clk_sys out of range for the servo PWM divider"
        );
        config.divider = FixedU16::<U4>::from_bits(div_bits as u16);
        config.top = (REFRESH_INTERVAL_US - 1) as u16;
        config.compare_a = ELEVON_RIGHT_CENTER_US as u16;
        config.compare_b = ELEVON_LEFT_CENTER_US as u16;

        let slice = Pwm::new_output_ab(
            pins.slice,
            pins.elevon_right,
            pins.elevon_left,
            config.clone(),
        );

        Self { slice, config }
    }

    /// Latch both pulse widths; they take effect together at the next wrap.
    fn write(&mut self, left_us: u32, right_us: u32) {
        self.config.compare_b = left_us as u16;
        self.config.compare_a = right_us as u16;
        self.slice.set_config(&self.config);
    }

    pub fn set_safe_positions(&mut self) {
        // Use individual trim-adjusted center positions
        self.write(ELEVON_LEFT_CENTER_US, ELEVON_RIGHT_CENTER_US);
    }

    /// Set elevons without trim applied (legacy)
    pub fn set_elevons(&mut self, left_us: u32, right_us: u32) {
        self.write(left_us, right_us);
    }

    /// Set elevons with trim applied (recommended method). Returns the pulses
    /// actually output, after trim and the right servo's inversion.
    pub fn set_elevons_with_trim(&mut self, mut left_us: u32, mut right_us: u32) -> (u32, u32) {
        // Store original values for debug output
        let orig_left = left_us;
        let orig_right = right_us;

        // Apply trim adjustments
        left_us = apply_elevon_trim(left_us, ELEVON_LEFT_TRIM_US);

        // Invert the right elevon signal to account for opposite servo orientation
        // This ensures that when the same value is provided to both elevons,
        // they will move in the same physical direction
        let inverted_right_us = SERVO_MAX_PULSE_US + SERVO_MIN_PULSE_US - right_us;
        right_us = apply_elevon_trim(inverted_right_us, ELEVON_RIGHT_TRIM_US);

        self.write(left_us, right_us);

        // Debug output for trim monitoring
        if ELEVON_LEFT_TRIM_US != 0 || ELEVON_RIGHT_TRIM_US != 0 {
            defmt::trace!(
                "Elevon trim: L:{}μs→{}μs R:{}μs→{}μs (inverted)",
                orig_left,
                left_us,
                orig_right,
                right_us
            );
        }
        (left_us, right_us)
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
    pub slice: Peri<'a, PWM_SLICE6>,
    pub elevon_left: Peri<'a, PIN_13>,
    pub elevon_right: Peri<'a, PIN_12>,
}
