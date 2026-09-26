//! Host tests for the gyro low-pass filter.
//!
//!   cargo test -p elle-control --target x86_64-unknown-linux-gnu --test filter

use elle_control::filter::LowPass2;

const FS: f32 = 1000.0;
const FC: f32 = 30.0;

/// Steady-state amplitude ratio for a sine at `f` Hz.
fn gain_at(f: f32) -> f32 {
    let mut lp = LowPass2::new(FC, FS);
    let mut peak = 0.0f32;
    for n in 0..4000 {
        let y = lp.apply((2.0 * core::f32::consts::PI * f * n as f32 / FS).sin());
        if n > 2000 {
            peak = peak.max(y.abs());
        }
    }
    peak
}

#[test]
fn passes_the_control_band() {
    // 5-8 Hz is where the attitude loop lives; keep it within a few percent.
    assert!(gain_at(5.0) > 0.98, "{}", gain_at(5.0));
    assert!(gain_at(8.0) > 0.96, "{}", gain_at(8.0));
}

#[test]
fn is_minus_3db_at_the_corner() {
    let g = gain_at(FC);
    assert!((g - core::f32::consts::FRAC_1_SQRT_2).abs() < 0.02, "{g}");
}

#[test]
fn removes_engine_vibration() {
    // EDF rotation at ~7-14k rpm is 115-230 Hz: second order gives -24 dB or more.
    assert!(gain_at(115.0) < 0.08, "{}", gain_at(115.0));
    assert!(gain_at(230.0) < 0.02, "{}", gain_at(230.0));
}

#[test]
fn constant_input_passes_unchanged_from_the_first_sample() {
    let mut lp = LowPass2::new(FC, FS);
    for _ in 0..50 {
        assert!((lp.apply(0.3) - 0.3).abs() < 1e-5);
    }
}

#[test]
fn step_settles_to_the_input() {
    let mut lp = LowPass2::new(FC, FS);
    lp.apply(0.0);
    let mut y = 0.0;
    for _ in 0..200 {
        y = lp.apply(1.0);
    }
    assert!((y - 1.0).abs() < 1e-3, "{y}");
}
