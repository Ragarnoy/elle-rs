use crate::throttle::sbus_to_pulse_us;
use elle_config::*;

#[derive(Debug, Default)]
pub struct ArmingState {
    pub armed: bool,
    pub failsafe_active: bool,
}

impl ArmingState {
    pub fn update(&mut self, throttle_sbus: u16, sbus_failsafe: bool) {
        // Check arming conditions
        if !self.armed {
            let throttle_us =
                sbus_to_pulse_us(throttle_sbus, ENGINE_MIN_PULSE_US, ENGINE_MAX_PULSE_US);
            if throttle_us < ENGINE_ARM_THRESHOLD {
                self.armed = true;
            }
        }

        // Handle failsafe
        if sbus_failsafe {
            self.armed = false;
            self.failsafe_active = true;
        }
    }

    pub fn signal_loss(&mut self) {
        self.armed = false;
        self.failsafe_active = true;
    }

    pub fn signal_restored(&mut self) {
        self.failsafe_active = false;
    }

    /// Manual arm (for RTT/debug control)
    pub fn arm(&mut self) {
        self.armed = true;
        self.failsafe_active = false;
    }

    /// Manual disarm (for RTT/debug control)
    pub fn disarm(&mut self) {
        self.armed = false;
    }
}
