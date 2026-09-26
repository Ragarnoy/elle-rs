//! Arming state machine: failsafe must not be undone by a replayed RC frame.
//!
//!   cargo test -p elle-control --target x86_64-unknown-linux-gnu --test arming

use elle_control::ArmingState;

const THROTTLE_LOW: u16 = 0;
const THROTTLE_HIGH: u16 = 1800;

#[test]
fn low_throttle_arms() {
    let mut a = ArmingState::default();
    a.update(THROTTLE_LOW);
    assert!(a.armed);
}

#[test]
fn stale_low_throttle_cannot_rearm_during_failsafe() {
    let mut a = ArmingState::default();
    a.update(THROTTLE_LOW);
    a.signal_loss();
    assert!(!a.armed && a.failsafe_active);

    // The control loop keeps replaying the last frame it got — throttle low.
    for _ in 0..100 {
        a.update(THROTTLE_LOW);
    }
    assert!(
        !a.armed,
        "re-armed from a replayed frame while the link was lost"
    );
}

#[test]
fn restore_requires_a_live_low_throttle() {
    let mut a = ArmingState::default();
    a.update(THROTTLE_LOW);
    a.signal_loss();
    a.signal_restored();

    // Link back with the stick up: stays disarmed until the pilot pulls it down.
    a.update(THROTTLE_HIGH);
    assert!(!a.armed);
    a.update(THROTTLE_LOW);
    assert!(a.armed);
}
