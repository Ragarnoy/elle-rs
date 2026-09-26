//! Host tests for the level-calibration math.
//!
//!   cargo test -p elle-control --target x86_64-unknown-linux-gnu --test level_cal

use elle_control::level_cal::{
    LevelCalFail, compute_mount, identity_bytes, mount_from_bytes, mount_to_bytes,
    mount_to_display_deg,
};
use nalgebra::{UnitQuaternion, Vector3};

const G: f32 = 9.81;
const QUIET: f32 = 0.01;
const LIMIT: f32 = 0.1;

/// Gravity reaction as the accelerometer sees it for a board whose orientation
/// (earth ← sensor) is `roll`/`pitch` in degrees, in the AHRS's Euler convention.
fn accel_for_board(roll_deg: f32, pitch_deg: f32) -> Vector3<f32> {
    let board =
        UnitQuaternion::from_euler_angles(roll_deg.to_radians(), pitch_deg.to_radians(), 0.0);
    board.inverse() * Vector3::new(0.0, 0.0, G)
}

fn assert_close(a: f32, b: f32, tol: f32) {
    assert!((a - b).abs() < tol, "{a} != {b} (±{tol})");
}

#[test]
fn flat_board_needs_no_correction() {
    let mount = compute_mount(accel_for_board(0.0, 0.0), QUIET, LIMIT).unwrap();
    assert!(mount.angle() < 1e-6);
}

#[test]
fn nose_down_board_is_reported_and_corrected() {
    let accel = accel_for_board(0.0, -3.0);
    let mount = compute_mount(accel, QUIET, LIMIT).unwrap();
    let (roll, pitch) = mount_to_display_deg(&mount);
    assert_close(pitch, -3.0, 0.01);
    assert_close(roll, 0.0, 0.01);
    // After correction, gravity lies on +Z: the airframe reads level.
    let corrected = mount * accel;
    assert_close(corrected.x, 0.0, 1e-3);
    assert_close(corrected.y, 0.0, 1e-3);
    assert_close(corrected.z, G, 1e-3);
}

#[test]
fn roll_offset_uses_the_telemetry_sign() {
    // The driver publishes roll negated; the display must match what the
    // uncorrected telemetry shows, so a raw +4° roll displays as -4°.
    let mount = compute_mount(accel_for_board(4.0, 0.0), QUIET, LIMIT).unwrap();
    let (roll, pitch) = mount_to_display_deg(&mount);
    assert_close(roll, -4.0, 0.01);
    assert_close(pitch, 0.0, 0.01);
}

#[test]
fn combined_offset_corrects_gravity() {
    let accel = accel_for_board(2.5, -1.5);
    let mount = compute_mount(accel, QUIET, LIMIT).unwrap();
    let c = mount * accel;
    assert_close(c.x, 0.0, 1e-3);
    assert_close(c.y, 0.0, 1e-3);
}

#[test]
fn large_tilt_is_rejected() {
    assert_eq!(
        compute_mount(accel_for_board(0.0, 20.0), QUIET, LIMIT),
        Err(LevelCalFail::Tilted)
    );
}

#[test]
fn inverted_and_zero_gravity_are_rejected() {
    assert_eq!(
        compute_mount(Vector3::new(0.0, 0.0, -G), QUIET, LIMIT),
        Err(LevelCalFail::Tilted)
    );
    assert_eq!(
        compute_mount(Vector3::zeros(), QUIET, LIMIT),
        Err(LevelCalFail::Tilted)
    );
}

#[test]
fn movement_is_rejected() {
    assert_eq!(
        compute_mount(accel_for_board(0.0, 0.0), 0.5, LIMIT),
        Err(LevelCalFail::Moving)
    );
    assert_eq!(
        compute_mount(accel_for_board(0.0, 0.0), f32::NAN, LIMIT),
        Err(LevelCalFail::Moving)
    );
}

#[test]
fn bytes_round_trip() {
    let mount = compute_mount(accel_for_board(1.0, -2.0), QUIET, LIMIT).unwrap();
    let back = mount_from_bytes(&mount_to_bytes(&mount)).unwrap();
    assert!(mount.angle_to(&back) < 1e-6);
}

#[test]
fn cleared_and_corrupt_storage_load_as_none() {
    assert_eq!(mount_from_bytes(&identity_bytes()), None);
    assert_eq!(mount_from_bytes(&[0u8; 16]), None); // not unit
    assert_eq!(mount_from_bytes(&[0xFF; 16]), None); // NaN
    let tilted = UnitQuaternion::from_euler_angles(0.0, 30f32.to_radians(), 0.0);
    assert_eq!(mount_from_bytes(&mount_to_bytes(&tilted)), None);
}
