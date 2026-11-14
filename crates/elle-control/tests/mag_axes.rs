//! Host tests for the magnetometer axis alignment (proposal 0010).
//!
//!   cargo test -p elle-control --target x86_64-unknown-linux-gnu --test mag_axes
//!
//! The sign test uses a real reading: the eagle level on the bench, 2026-11-14,
//! nose at 46° by phone compass, in France (dip ~60–62° down).

use elle_control::attitude::{AttitudePipeline, mag_axes};
use nalgebra::{UnitQuaternion, Vector3};

const G: f32 = 9.81;

/// Raw MMC5616WA counts, eagle level on the bench, and the stored offsets.
const RAW: [f32; 3] = [3034.0, -803.0, -7189.0];
const OFFSET: [f32; 3] = [1167.5, 984.0, -2386.5];

fn yaw180() -> UnitQuaternion<f32> {
    UnitQuaternion::from_axis_angle(&Vector3::z_axis(), core::f32::consts::PI)
}

fn corrected() -> Vector3<f32> {
    Vector3::new(RAW[0] - OFFSET[0], RAW[1] - OFFSET[1], RAW[2] - OFFSET[2])
}

/// Yaw (AHRS convention, degrees) and dip (degrees, + = down) of a level
/// airframe-frame field: north lies at atan2(y, x) anticlockwise from body x.
fn yaw_and_dip(m: Vector3<f32>) -> (f32, f32) {
    let yaw = -m.y.atan2(m.x).to_degrees();
    let dip = (-m.z).atan2(m.x.hypot(m.y)).to_degrees();
    (yaw, dip)
}

fn wrap(d: f32) -> f32 {
    (d + 540.0).rem_euclid(360.0) - 180.0
}

#[test]
fn quarter_turns() {
    let m = Vector3::new(1.0f32, 2.0, 3.0);
    assert_eq!(mag_axes(m, 0.0), m);
    assert_eq!(mag_axes(m, 90.0), Vector3::new(-2.0, 1.0, 3.0));
    assert_eq!(mag_axes(m, 180.0), Vector3::new(-1.0, -2.0, 3.0));
    assert_eq!(mag_axes(m, 270.0), Vector3::new(2.0, -1.0, 3.0));
    assert_eq!(mag_axes(m, -90.0), mag_axes(m, 270.0));
    // Each is the rotation it claims to be.
    for deg in [90.0f32, 180.0, 270.0] {
        let r = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), deg.to_radians());
        assert!((mag_axes(m, deg) - r * m).norm() < 1e-5, "{deg}");
    }
}

#[test]
fn platform_value() {
    #[cfg(not(feature = "platform-dart"))]
    assert_eq!(elle_config::MAG_AXES_YAW_DEG, 90.0);
    // Dart: 0 until its gate (magnet removed, ground tumble).
    #[cfg(feature = "platform-dart")]
    assert_eq!(elle_config::MAG_AXES_YAW_DEG, 0.0);
}

/// The bench reading through the eagle's board (180°) and mag (90°) rotations:
/// nose 46° → yaw ≈ 180° − 46° = 134° (the airframe x axis points aft), dip
/// 61.7° down. 270 instead of 90 would read the tail at the nose (yaw ≈ −46°).
#[test]
fn bench_reading_points_where_the_compass_did() {
    let p = AttitudePipeline::with_board(yaw180()).with_mag_axes(90.0);
    let (yaw, dip) = yaw_and_dip(p.mag_to_airframe(corrected()));
    assert!(wrap(yaw - 134.0).abs() < 1.0, "yaw {yaw}");
    assert!((dip - 61.7).abs() < 1.0, "dip {dip}");

    // Today's mapping read 90° off: the firmware showed −141° at this pose.
    let old = AttitudePipeline::with_board(yaw180());
    let (yaw_old, _) = yaw_and_dip(old.mag_to_airframe(corrected()));
    assert!(wrap(yaw_old + 136.3).abs() < 1.0, "old yaw {yaw_old}");
}

/// The whole pipeline, converging from level/north on that reading.
#[test]
fn pipeline_converges_to_the_compass_heading() {
    let mut p = AttitudePipeline::with_board(yaw180()).with_mag_axes(90.0);
    // Level: gravity reaction straight up in the sensor frame (the 180° board
    // rotation leaves it there).
    let accel = Vector3::new(0.0, 0.0, G);
    let mag = p.mag_to_airframe(corrected());
    let mut last = None;
    for _ in 0..120_000 {
        last = Some(p.fuse(p.debias(Vector3::zeros()), accel, Some(&mag)));
    }
    let a = last.expect("fused");
    assert!(
        wrap(a.yaw.to_degrees() - 134.0).abs() < 1.0,
        "yaw {}",
        a.yaw.to_degrees()
    );
    assert!(a.pitch.to_degrees().abs() < 0.5 && a.roll.to_degrees().abs() < 0.5);
}

/// 0 leaves the pipeline exactly as before (the dart until its gate).
#[test]
fn zero_is_bit_for_bit() {
    let m = corrected();
    let a = AttitudePipeline::with_board(yaw180());
    let b = AttitudePipeline::with_board(yaw180()).with_mag_axes(0.0);
    assert_eq!(a.mag_to_airframe(m), b.mag_to_airframe(m));
}
