//! Host tests for mixer saturation flags and the PID's anti-windup.
//!
//!   cargo test -p elle-control --target x86_64-unknown-linux-gnu --test anti_windup

use elle_control::mixing::elevons::{ControlInputs, MixSaturation, mix_elevons};
use elle_control::pid::{AttitudeController, PidConfig};

fn inputs(pitch: f32, roll: f32) -> ControlInputs {
    ControlInputs {
        pitch,
        roll,
        yaw: 0.0,
        throttle: 0.5,
    }
}

fn sat(pitch: f32, roll: f32) -> MixSaturation {
    mix_elevons(&inputs(pitch, roll)).saturation
}

#[test]
fn unsaturated_mix_blocks_nothing() {
    assert_eq!(sat(0.3, -0.4), MixSaturation::default());
}

#[test]
fn pitch_input_clip_blocks_only_that_direction() {
    let s = sat(1.5, 0.0);
    assert!(s.pitch_up && !s.pitch_down);
    // Both elevons at +1: more roll either way would drive one of them further up.
    assert!(s.roll_right && s.roll_left);
    let s = sat(-1.5, 0.0);
    assert!(s.pitch_down && !s.pitch_up);
}

#[test]
fn roll_saturates_through_the_right_elevon() {
    // right = 0.6 + 0.6 = 1.2 > 1; left = 0.
    let s = sat(0.6, 0.6);
    assert!(s.pitch_up, "more pitch would push the right elevon further");
    assert!(s.roll_right, "more right roll would push the right elevon further");
    assert!(!s.pitch_down && !s.roll_left, "unwinding directions stay open");
}

#[test]
fn left_roll_saturates_through_the_left_elevon() {
    // left = 0 - (-1.1) = 1.1 > 1 via the roll input clip too.
    let s = sat(0.0, -1.1);
    assert!(s.roll_left && s.pitch_up);
    assert!(!s.roll_right);
}

/// Integral-only controller: output == scale * ki * integral.
fn integrator() -> AttitudeController {
    let mut c = AttitudeController::with_config(PidConfig {
        kp_pitch: 0.0,
        ki_pitch: 1.0,
        kd_pitch: 0.0,
        kp_roll: 0.0,
        ki_roll: 1.0,
        kd_roll: 0.0,
        i_limit: 10.0,
        scale: 1.0,
    });
    c.enabled = true;
    c.pitch_hold_enabled = true;
    c.roll_hold_enabled = true;
    c
}

fn step(c: &mut AttitudeController, pitch_err: f32, s: MixSaturation) -> f32 {
    c.update(pitch_err, 0.0, 0.0, 0.0, None, false, s).0
}

#[test]
fn integral_holds_while_blocked_and_unwinds_after() {
    let mut c = integrator();
    let free = MixSaturation::default();
    let up_blocked = MixSaturation {
        pitch_up: true,
        ..free
    };

    let a = step(&mut c, 1.0, free);
    assert!(a > 0.0);
    // Positive error, nose-up blocked: no further growth.
    let b = step(&mut c, 1.0, up_blocked);
    assert_eq!(a, b);
    // Error reverses: unwinding is allowed even while up is blocked.
    let c2 = step(&mut c, -1.0, up_blocked);
    assert!(c2 < b);
}

#[test]
fn blocking_one_axis_leaves_the_other_integrating() {
    let mut c = integrator();
    let s = MixSaturation {
        pitch_up: true,
        ..MixSaturation::default()
    };
    let (p1, r1) = c.update(1.0, 1.0, 0.0, 0.0, None, false, s);
    let (p2, r2) = c.update(1.0, 1.0, 0.0, 0.0, None, false, s);
    assert_eq!(p1, 0.0);
    assert_eq!(p2, 0.0);
    assert!(r2 > r1 && r1 > 0.0);
}
