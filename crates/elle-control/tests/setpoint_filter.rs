//! Host tests for the stick-to-setpoint smoothing time constant.
//!
//!   cargo test -p elle-control --target x86_64-unknown-linux-gnu --test setpoint_filter
//!
//! The filter is a per-tick EMA in `FlightController` (elle-system); its weight is
//! derived from `SETPOINT_FILTER_TAU_S` and the loop period in `elle-config`. These
//! tests pin that the response is set in seconds, not ticks, so a loop-rate change
//! doesn't change how the sticks feel.

use elle_config::{CONTROL_LOOP_DT, SETPOINT_FILTER_ALPHA, SETPOINT_FILTER_TAU_S};

/// The config's formula, for an arbitrary loop period.
fn alpha(dt: f32) -> f32 {
    dt / (SETPOINT_FILTER_TAU_S + dt)
}

/// Seconds for a unit step to reach 1 - 1/e through the EMA at period `dt`.
fn time_to_63(dt: f32) -> f32 {
    let a = alpha(dt);
    let (mut y, mut t) = (0.0f32, 0.0f32);
    while y < 1.0 - (-1.0f32).exp() {
        y += a * (1.0 - y);
        t += dt;
    }
    t
}

#[test]
fn config_uses_the_formula() {
    assert!((SETPOINT_FILTER_ALPHA - alpha(CONTROL_LOOP_DT)).abs() < 1e-6);
}

#[test]
fn old_rate_keeps_the_old_weight() {
    // 0.068 s was chosen as what alpha 0.15 meant at the old 12 ms loop.
    assert!(
        (alpha(0.012) - 0.15).abs() < 1e-3,
        "alpha at 12 ms = {}",
        alpha(0.012)
    );
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
