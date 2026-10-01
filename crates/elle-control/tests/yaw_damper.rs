//! Host tests for the yaw damper (docs/changes/0001-eagle-yaw-damper.md).
//!
//!   cargo test -p elle-control --target x86_64-unknown-linux-gnu --test yaw_damper
//!   cargo test -p elle-control --target x86_64-unknown-linux-gnu --features platform-dart --test yaw_damper
//!
//! The firmware gain is 0 until the proposal's gate 1, so these build dampers
//! with an explicit gain; the shipped configuration is checked separately.

use elle_config::{CONTROL_LOOP_DT, YAW_DAMPER_MAX, YAW_DAMPER_WASHOUT_S};
use elle_control::yaw_damper::YawDamper;

const GAIN: f32 = 0.3;

fn damper() -> YawDamper {
    YawDamper::new(GAIN, YAW_DAMPER_WASHOUT_S, YAW_DAMPER_MAX, CONTROL_LOOP_DT)
}

fn ticks(seconds: f32) -> usize {
    (seconds / CONTROL_LOOP_DT) as usize
}

/// Engaging mid-turn must not step the engines.
#[test]
fn first_update_after_reset_outputs_zero() {
    let mut d = damper();
    assert_eq!(d.update(0.8), 0.0);
    d.reset();
    assert_eq!(d.update(-0.5), 0.0);
}

/// A steady turn has a constant yaw rate: the washout lets it through.
#[test]
fn steady_turn_washes_out() {
    let mut d = damper();
    d.update(0.0);
    let first = d.update(0.4);
    assert!(first > 0.0, "{first}");
    let mut out = first;
    for _ in 0..ticks(5.0 * YAW_DAMPER_WASHOUT_S) {
        out = d.update(0.4);
    }
    assert!(
        out.abs() < 0.01 * first,
        "after 5 tau: {out} (first {first})"
    );
}

/// A 1 Hz oscillation (about the Dutch roll period) passes nearly unattenuated:
/// the washout corner is ~0.16 Hz.
#[test]
fn dutch_roll_band_passes() {
    let mut d = damper();
    let amp = 0.2; // rad/s, small enough to stay below the clamp
    let mut peak = 0.0f32;
    for k in 0..ticks(10.0) {
        let t = k as f32 * CONTROL_LOOP_DT;
        let out = d.update(amp * (2.0 * core::f32::consts::PI * t).sin());
        if t > 5.0 {
            peak = peak.max(out.abs());
        }
    }
    let ideal = GAIN * amp;
    assert!(
        peak > 0.95 * ideal && peak <= 1.01 * ideal,
        "{peak} vs {ideal}"
    );
}

#[test]
fn output_is_clamped() {
    let mut d = damper();
    d.update(0.0);
    assert_eq!(d.update(100.0), YAW_DAMPER_MAX);
    d.reset();
    d.update(0.0);
    assert_eq!(d.update(-100.0), -YAW_DAMPER_MAX);
}

#[test]
fn zero_gain_is_inert() {
    let mut d = YawDamper::new(0.0, YAW_DAMPER_WASHOUT_S, YAW_DAMPER_MAX, CONTROL_LOOP_DT);
    for k in 0..ticks(3.0) {
        assert_eq!(d.update((k as f32 * 0.37).sin() * 3.0), 0.0);
    }
}

/// The shipped configuration does nothing until gate 1 says otherwise.
#[test]
fn shipped_gain_is_zero() {
    let mut d = YawDamper::from_config();
    d.update(0.0);
    assert_eq!(d.update(1.0), 0.0);
}

/// End to end through the engine mixer: a nose-right yaw rate must slow the
/// left engine (a nose-left moment). An inverted sign here is positive feedback.
#[cfg(not(feature = "platform-dart"))]
#[test]
fn nose_right_rate_slows_left_engine() {
    use elle_config::lut::apply_differential_thrust_lut;
    use elle_control::mixing::yaw::normalized_yaw_to_rc;

    let base = 1000;
    let mut d = damper();
    d.update(0.0);
    let cmd = d.update(0.5);
    let (left, right) = apply_differential_thrust_lut(base, normalized_yaw_to_rc(cmd));
    assert!(left < right && right == base, "nose right: {left} {right}");

    d.reset();
    d.update(0.0);
    let cmd = d.update(-0.5);
    let (left, right) = apply_differential_thrust_lut(base, normalized_yaw_to_rc(cmd));
    assert!(right < left && left == base, "nose left: {left} {right}");
}

/// The pilot's right stick (negative normalized yaw, through `YAW_INVERT`) slows
/// the right engine (TEST_PLAN 1.2), the opposite of a nose-right damper command.
#[cfg(not(feature = "platform-dart"))]
#[test]
fn right_stick_slows_right_engine() {
    use elle_config::lut::apply_differential_thrust_lut;
    use elle_control::mixing::yaw::normalized_yaw_to_rc;

    let (left, right) = apply_differential_thrust_lut(1000, normalized_yaw_to_rc(-1.0));
    assert!(right < left, "{left} {right}");
}
