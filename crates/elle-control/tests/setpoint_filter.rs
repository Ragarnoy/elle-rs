//! Host tests for the stick-to-setpoint smoothing.
//!
//!   cargo test -p elle-control --target x86_64-unknown-linux-gnu --test setpoint_filter
//!
//! `FlightController` runs `smooth_setpoint` once per tick with a weight derived
//! from `SETPOINT_FILTER_TAU_S` and the loop period. These tests pin that the
//! response is set in seconds, not ticks, so a loop-rate change doesn't change how
//! the sticks feel, and that the rate cap bounds each step.

use elle_config::{MAX_SETPOINT_RATE_DEG_S, SETPOINT_FILTER_TAU_S, setpoint_filter_alpha};
use elle_control::filter::smooth_setpoint;

/// Seconds for a unit step to reach 1 - 1/e at loop period `dt`, rate cap off.
fn time_to_63(dt: f32) -> f32 {
    let a = setpoint_filter_alpha(dt);
    let (mut y, mut t) = (0.0f32, 0.0f32);
    while y < 1.0 - (-1.0f32).exp() {
        y = smooth_setpoint(y, 1.0, a, f32::INFINITY);
        t += dt;
    }
    t
}

#[test]
fn old_rate_keeps_the_old_weight() {
    // 0.068 s was chosen as what alpha 0.15 meant at the old 12 ms loop.
    let a = setpoint_filter_alpha(0.012);
    assert!((a - 0.15).abs() < 1e-3, "alpha at 12 ms = {a}");
}

#[test]
fn step_response_is_set_in_seconds() {
    // With alpha = dt / (tau + dt) a step crosses 63 % after about tau + dt / 2,
    // rounded up to a whole tick. Check that at each rate, and that the old and
    // new loop rates land within one old (12 ms) tick of each other.
    for dt in [0.012f32, 0.005, 0.004] {
        let t = time_to_63(dt);
        let expected = SETPOINT_FILTER_TAU_S + dt / 2.0;
        assert!(
            (t - expected).abs() <= dt + 1e-4,
            "dt {dt}: 63 % after {t} s, expected ~{expected} s"
        );
    }
    let (old, new) = (time_to_63(0.012), time_to_63(0.005));
    assert!((old - new).abs() <= 0.012, "12 ms: {old} s, 5 ms: {new} s");
}

#[test]
fn rate_cap_bounds_a_full_stick_step() {
    // A full-bank step (45 deg) moves at most MAX_SETPOINT_RATE_DEG_S, in either
    // direction.
    let dt = 0.005;
    let max_step = MAX_SETPOINT_RATE_DEG_S * dt;
    let a = setpoint_filter_alpha(dt);
    let up = smooth_setpoint(0.0, 45.0, a, max_step);
    let down = smooth_setpoint(0.0, -45.0, a, max_step);
    assert!((up - max_step).abs() < 1e-6, "up step {up}");
    assert!((down + max_step).abs() < 1e-6, "down step {down}");
    // A small error is below the cap and takes the plain EMA step.
    let small = smooth_setpoint(0.0, 0.1, a, max_step);
    assert!((small - a * 0.1).abs() < 1e-7, "small step {small}");
}
