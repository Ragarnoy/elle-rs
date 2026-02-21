use crate::throttle::rc_to_pulse_us;
use elle_config::*;

#[derive(Debug, Default)]
pub struct ArmingState {
    pub armed: bool,
    pub failsafe_active: bool,
}

impl ArmingState {
    pub fn update(&mut self, throttle_raw: u16, failsafe: bool) {
        // Check arming conditions
        if !self.armed {
            let throttle_us =
                rc_to_pulse_us(throttle_raw, ENGINE_MIN_PULSE_US, ENGINE_MAX_PULSE_US);
            if throttle_us < ENGINE_ARM_THRESHOLD {
                self.armed = true;
            }
        }

        // Handle failsafe
        if failsafe {
            self.armed = false;
            self.failsafe_active = true;
        }
    }

    pub const fn signal_loss(&mut self) {
        self.armed = false;
        self.failsafe_active = true;
    }

    pub const fn signal_restored(&mut self) {
        self.failsafe_active = false;
    }

    /// Manual arm (for RTT/debug control)
    pub const fn arm(&mut self) {
        self.armed = true;
        self.failsafe_active = false;
    }

    /// Manual disarm (for RTT/debug control)
    pub const fn disarm(&mut self) {
        self.armed = false;
    }
}
