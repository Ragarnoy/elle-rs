//! Host tests for the shared attitude pipeline (`elle_control::attitude`).
//!
//!   cargo test -p elle-control --target x86_64-unknown-linux-gnu --test attitude

use elle_control::attitude::AttitudePipeline;
use nalgebra::{UnitQuaternion, Vector3};

const G: f32 = 9.81;

/// Gravity reaction as the accelerometer sees it for a board at `roll`/`pitch`
/// (degrees) in the AHRS's Euler convention, sensor frame.
fn accel_for(roll_deg: f32, pitch_deg: f32) -> Vector3<f32> {
    let board =
        UnitQuaternion::from_euler_angles(roll_deg.to_radians(), pitch_deg.to_radians(), 0.0);
    board.inverse() * Vector3::new(0.0, 0.0, G)
}

/// Run `n` still samples at a fixed accel.
fn settle(
    p: &mut AttitudePipeline,
    accel: Vector3<f32>,
    n: usize,
) -> elle_control::attitude::Attitude {
    let mut last = None;
    for _ in 0..n {
        last = Some(p.fuse(p.debias(Vector3::zeros()), accel, None));
    }
    last.expect("AHRS accepted the samples")
}

/// Madgwick at β 0.033 converges slowly; start near the answer.
fn seeded(roll_deg: f32, pitch_deg: f32) -> AttitudePipeline {
    let mut p = AttitudePipeline::new();
    p.set_quat(UnitQuaternion::from_euler_angles(
        roll_deg.to_radians(),
        pitch_deg.to_radians(),
        0.0,
    ));
    p
}

#[test]
fn level_board_reads_level() {
    let a = settle(&mut AttitudePipeline::new(), accel_for(0.0, 0.0), 1000);
    assert!(a.pitch.abs() < 1e-3 && a.roll.abs() < 1e-3, "{a:?}");
}

#[test]
fn nose_up_is_positive_pitch() {
    let a = settle(&mut seeded(0.0, 10.0), accel_for(0.0, 10.0), 2000);
    assert!((a.pitch.to_degrees() - 10.0).abs() < 0.5, "{a:?}");
}

#[test]
fn roll_is_negated_for_this_pcb() {
    // A sensor-frame roll of +10° reads as -10°: the board is mounted so the
    // sensor's positive roll is the airframe's left wing down.
    let a = settle(&mut seeded(10.0, 0.0), accel_for(10.0, 0.0), 2000);
    assert!((a.roll.to_degrees() + 10.0).abs() < 0.5, "{a:?}");
}

#[test]
fn rates_are_filtered_and_signed_like_attitude() {
    let mut p = AttitudePipeline::new();
    let mut last = None;
    for _ in 0..500 {
        // Sensor +x rate: roll rate reads negative, like roll; +y is pitch rate.
        last = Some(p.fuse(
            p.debias(Vector3::new(0.2, 0.1, 0.05)),
            accel_for(0.0, 0.0),
            None,
        ));
    }
    let a = last.unwrap();
    assert!((a.roll_rate + 0.2).abs() < 1e-3, "{a:?}");
    assert!(
        (a.pitch_rate - 0.1).abs() < 1e-3 && (a.yaw_rate - 0.05).abs() < 1e-3,
        "{a:?}"
    );
    // The filter lags: from rest, the first sample of a step is far below it.
    let mut p = AttitudePipeline::new();
    let _ = p.fuse(Vector3::zeros(), accel_for(0.0, 0.0), None);
    let first = p.fuse(Vector3::new(0.0, 0.1, 0.0), accel_for(0.0, 0.0), None);
    assert!(first.pitch_rate < 0.05, "{first:?}");
}

#[test]
fn bias_is_subtracted_before_the_mount() {
    let mut p = AttitudePipeline::new();
    p.gyro_bias = Vector3::new(0.01, -0.02, 0.03);
    assert_eq!(p.debias(Vector3::new(0.01, -0.02, 0.03)), Vector3::zeros());
    // With a 90° yaw mount, a sensor-x rate becomes an airframe-y (pitch) rate.
    p.mount = UnitQuaternion::from_euler_angles(0.0, 0.0, core::f32::consts::FRAC_PI_2);
    let mut last = None;
    for _ in 0..500 {
        last = Some(p.fuse(
            p.debias(Vector3::new(0.11, -0.02, 0.03)),
            Vector3::new(0.0, 0.0, G),
            None,
        ));
    }
    let a = last.unwrap();
    assert!(
        (a.pitch_rate - 0.1).abs() < 1e-3 && a.roll_rate.abs() < 1e-3,
        "{a:?}"
    );
}

#[test]
fn seeded_pipelines_replay_identically() {
    // The replay contract: same seed, same inputs → bit-identical outputs.
    let seed = UnitQuaternion::from_euler_angles(0.1, -0.05, 1.2);
    let mut a = AttitudePipeline::new();
    let mut b = AttitudePipeline::new();
    a.set_quat(seed);
    b.set_quat(seed);
    let mag = Vector3::new(0.2, 0.05, -0.4);
    for i in 0..2000 {
        let t = i as f32 * 1e-3;
        let gyro = Vector3::new(0.3 * (7.0 * t).sin(), 0.1 * (3.0 * t).cos(), 0.05);
        let accel = Vector3::new(0.5 * (5.0 * t).sin(), 0.3, G);
        let m = (i % 100 < 50).then_some(&mag);
        assert_eq!(a.fuse(gyro, accel, m), b.fuse(gyro, accel, m));
    }
    assert_eq!(a.quat(), b.quat());
}
