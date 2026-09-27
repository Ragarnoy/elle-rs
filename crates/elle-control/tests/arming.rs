//! Arming state machine: zero-thrust arming after a deliberate throttle
//! up-then-down, and failsafe must not be undone by a replayed RC frame.
//!
//!   cargo test -p elle-control --target x86_64-unknown-linux-gnu --test arming

use elle_config::ARM_THROTTLE_HIGH_RAW;
use elle_config::lut::throttle_curve_lut;
use elle_control::ArmingState;

const THROTTLE_LOW: u16 = 0;
const THROTTLE_HIGH: u16 = 1800;

/// Do the arm gesture: stick up, then back to zero.
fn gesture(a: &mut ArmingState) {
    a.update(THROTTLE_HIGH);
    a.update(THROTTLE_LOW);
}

#[test]
fn low_throttle_at_boot_does_not_arm() {
    let mut a = ArmingState::default();
    for _ in 0..100 {
        a.update(THROTTLE_LOW);
    }
    assert!(!a.armed);
}

#[test]
fn up_then_down_arms() {
    let mut a = ArmingState::default();
    gesture(&mut a);
    assert!(a.armed);
}

#[test]
fn going_up_is_not_enough_below_the_high_mark() {
    let mut a = ArmingState::default();
    a.update(ARM_THROTTLE_HIGH_RAW - 1);
    a.update(THROTTLE_LOW);
    assert!(!a.armed);
}

/// Every raw value that produces thrust must never arm, even after the
/// gesture's high: the old 1100 µs threshold armed at raw 341 (~5,400 RPM).
#[test]
fn never_arms_with_thrust_commanded() {
    for raw in 0..ARM_THROTTLE_HIGH_RAW {
        let mut a = ArmingState::default();
        a.update(THROTTLE_HIGH);
        a.update(raw);
        assert_eq!(
            a.armed,
            throttle_curve_lut(raw) == 0,
            "raw {raw}: thrust {}",
            throttle_curve_lut(raw)
        );
    }
    let mut a = ArmingState::default();
    a.update(THROTTLE_HIGH);
    a.update(341);
    assert!(!a.armed, "raw 341 commands thrust and must not arm");
}

#[test]
fn disarm_needs_a_new_gesture() {
    let mut a = ArmingState::default();
    gesture(&mut a);
    a.disarm();
    a.update(THROTTLE_LOW);
    assert!(!a.armed, "re-armed without a new gesture");
    gesture(&mut a);
    assert!(a.armed);
}

#[test]
fn stale_low_throttle_cannot_rearm_during_failsafe() {
    let mut a = ArmingState::default();
    gesture(&mut a);
    a.signal_loss();
    assert!(!a.armed && a.failsafe_active);

    // The control loop keeps replaying the last frame it got.
    for _ in 0..100 {
        a.update(THROTTLE_HIGH);
        a.update(THROTTLE_LOW);
    }
    assert!(
        !a.armed,
        "re-armed from replayed frames while the link was lost"
    );
}

#[test]
fn restore_requires_a_new_gesture() {
    let mut a = ArmingState::default();
    gesture(&mut a);
    a.signal_loss();
    a.signal_restored();

    // Link back with the stick low: the gesture was cleared by the failsafe.
    a.update(THROTTLE_LOW);
    assert!(!a.armed);
    gesture(&mut a);
    assert!(a.armed);
}
