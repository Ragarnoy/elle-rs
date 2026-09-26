//! Host tests for the boot-time gyro bias estimator.
//!
//!   cargo test -p elle-control --target x86_64-unknown-linux-gnu --test gyro_bias

use elle_control::gyro_bias::{GyroBiasEstimator, GyroBiasFail};
use nalgebra::Vector3;

const WINDOW: u32 = 100;
const SPREAD: f32 = 0.035;
const MAX_BIAS: f32 = 0.1;
const TIMEOUT: u32 = 1000;

fn estimator() -> GyroBiasEstimator {
    GyroBiasEstimator::with_limits(WINDOW, SPREAD, MAX_BIAS, TIMEOUT)
}

/// A still board: constant offset plus small alternating noise.
fn still(i: u32, bias: Vector3<f32>) -> Vector3<f32> {
    let noise = if i % 2 == 0 { 0.002 } else { -0.002 };
    bias + Vector3::repeat(noise)
}

fn run(
    est: &mut GyroBiasEstimator,
    samples: impl Iterator<Item = Vector3<f32>>,
) -> (u32, Option<Result<Vector3<f32>, GyroBiasFail>>) {
    let mut count = 0;
    for s in samples {
        count += 1;
        if let Some(r) = est.push(s) {
            return (count, Some(r));
        }
    }
    (count, None)
}

#[test]
fn still_board_gives_the_offset_after_one_window() {
    let bias = Vector3::new(0.01, -0.005, 0.003);
    let mut est = estimator();
    let (count, result) = run(&mut est, (0..).map(|i| still(i, bias)));
    assert_eq!(count, WINDOW);
    let got = result.unwrap().unwrap();
    assert!((got - bias).amax() < 1e-4, "{got:?}");
}

#[test]
fn movement_restarts_the_window() {
    let bias = Vector3::new(0.02, 0.0, 0.0);
    let mut est = estimator();
    // 60 still, one jolt, then still: must take the jolt + a full window after it.
    let samples = (0..60)
        .map(|i| still(i, bias))
        .chain(core::iter::once(Vector3::new(0.5, 0.0, 0.0)))
        .chain((0..).map(|i| still(i, bias)));
    let (count, result) = run(&mut est, samples);
    assert_eq!(count, 61 + WINDOW);
    assert!((result.unwrap().unwrap() - bias).amax() < 1e-4);
}

#[test]
fn never_still_times_out() {
    let mut est = estimator();
    let samples = (0..).map(|i| Vector3::new(if i % 2 == 0 { 0.2 } else { -0.2 }, 0.0, 0.0));
    let (count, result) = run(&mut est, samples);
    assert_eq!(count, TIMEOUT);
    assert_eq!(result, Some(Err(GyroBiasFail::Timeout)));
}

#[test]
fn implausible_offset_is_rejected() {
    // Perfectly steady, but 0.3 rad/s: a slow rotation, not a zero-rate offset.
    let mut est = estimator();
    let (_, result) = run(
        &mut est,
        (0..).map(|i| still(i, Vector3::new(0.0, 0.0, 0.3))),
    );
    assert_eq!(result, Some(Err(GyroBiasFail::TooLarge)));
}

#[test]
fn non_finite_sample_restarts() {
    let bias = Vector3::zeros();
    let mut est = estimator();
    let samples = (0..10)
        .map(|i| still(i, bias))
        .chain(core::iter::once(Vector3::new(f32::NAN, 0.0, 0.0)))
        .chain((0..).map(|i| still(i, bias)));
    let (count, result) = run(&mut est, samples);
    assert_eq!(count, 11 + WINDOW);
    assert!(result.unwrap().is_ok());
}
