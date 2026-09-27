use elle_config::ARM_THROTTLE_HIGH_RAW;
use elle_config::lut::throttle_curve_lut;

#[derive(Debug, Default)]
pub struct ArmingState {
    pub armed: bool,
    pub failsafe_active: bool,
    /// The throttle has been above `ARM_THROTTLE_HIGH_RAW` since the last disarm.
    seen_throttle_high: bool,
}

impl ArmingState {
    /// Throttle-gesture auto-arm: the stick must go above `ARM_THROTTLE_HIGH_RAW`,
    /// then back to where the throttle curve gives **zero thrust**. Arming at the
    /// first low reading would arm on boot with the stick already down, and an
    /// arm threshold above the curve's deadzone armed with thrust commanded.
    ///
    /// Never while failsafe is active: the caller may be replaying the last
    /// frame received before the link dropped. The failsafe also clears the
    /// gesture, so after `signal_restored()` the pilot must repeat it.
    pub fn update(&mut self, throttle_raw: u16) {
        if self.armed || self.failsafe_active {
            return;
        }
        if throttle_raw >= ARM_THROTTLE_HIGH_RAW {
            self.seen_throttle_high = true;
        } else if self.seen_throttle_high && throttle_curve_lut(throttle_raw) == 0 {
            self.armed = true;
            self.seen_throttle_high = false;
        }
    }

    pub fn signal_loss(&mut self) {
        self.armed = false;
        self.failsafe_active = true;
        self.seen_throttle_high = false;
    }

    pub fn signal_restored(&mut self) {
        self.failsafe_active = false;
    }

    /// Manual arm (for RTT/debug control)
    pub fn arm(&mut self) {
        self.armed = true;
        self.failsafe_active = false;
    }

    /// Manual disarm (for RTT/debug control). A new throttle gesture is needed
    /// to auto-arm again.
    pub fn disarm(&mut self) {
        self.armed = false;
        self.seen_throttle_high = false;
    }
}
