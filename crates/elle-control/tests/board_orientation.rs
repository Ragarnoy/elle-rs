//! Host tests for the per-airframe board orientation (proposal 0003).
//!
//!   cargo test -p elle-control --target x86_64-unknown-linux-gnu --test board_orientation
//!
//! Every attitude test here starts the filter level and lets it converge, rather
//! than seeding it at the answer: a sign error then shows up as the wrong sign,
//! not as a filter that has not moved far yet.

use elle_control::attitude::{AttitudePipeline, board_rotation};
use elle_control::level_cal::{compute_mount, level_in_airframe, mount_to_display_deg};
use nalgebra::{UnitQuaternion, Vector3};

const G: f32 = 9.81;
/// 60 s at 1 kHz: Madgwick at β 0.033 settles well within it from level.
const CONVERGE: usize = 60_000;

/// The eagle's mounting: turned round about the vertical.
fn yaw180() -> UnitQuaternion<f32> {
    UnitQuaternion::from_axis_angle(&Vector3::z_axis(), core::f32::consts::PI)
}

/// Gravity reaction in the airframe frame for an airframe at `roll`/`pitch`
/// (degrees, the AHRS's Euler convention: the published roll is its negation).
fn airframe_accel(roll_deg: f32, pitch_deg: f32) -> Vector3<f32> {
    UnitQuaternion::from_euler_angles(roll_deg.to_radians(), pitch_deg.to_radians(), 0.0).inverse()
        * Vector3::new(0.0, 0.0, G)
}

/// Run a still airframe through `p`, the sensor seeing `sensor_accel` and
/// `sensor_gyro`, from level until it settles.
fn converge(
    p: &mut AttitudePipeline,
    sensor_accel: Vector3<f32>,
    sensor_gyro: Vector3<f32>,
) -> elle_control::attitude::Attitude {
    let mut last = None;
    for _ in 0..CONVERGE {
        last = Some(p.fuse(p.debias(sensor_gyro), sensor_accel, None));
    }
    last.expect("AHRS accepted the samples")
}

fn deg(rad: f32) -> f32 {
    rad.to_degrees()
}

#[test]
fn board_rotation_matches_the_platform() {
    let b = board_rotation();
    #[cfg(feature = "platform-dart")]
    assert!(b.angle() < 1e-6, "dart board must be the identity: {b:?}");
    #[cfg(not(feature = "platform-dart"))]
    {
        assert!((b.angle() - core::f32::consts::PI).abs() < 1e-5, "{b:?}");
        let axis = b.axis().expect("a rotation has an axis");
        assert!(
            (axis.z.abs() - 1.0).abs() < 1e-5,
            "about the vertical: {axis:?}"
        );
    }
}

#[test]
fn turned_board_reads_nose_up_as_positive_pitch() {
    let board = yaw180();
    // The sensor sees the airframe's vectors through the inverse mounting.
    let sensor = board.inverse() * airframe_accel(0.0, 10.0);
    // Without the board rotation the reading is reversed: today's eagle.
    let raw = converge(&mut AttitudePipeline::new(), sensor, Vector3::zeros());
    assert!(deg(raw.pitch) < -9.5, "{raw:?}");
    // With it, nose up reads positive.
    let a = converge(
        &mut AttitudePipeline::with_board(board),
        sensor,
        Vector3::zeros(),
    );
    assert!((deg(a.pitch) - 10.0).abs() < 0.5, "{a:?}");
    assert!(deg(a.roll).abs() < 0.5, "{a:?}");
}

#[test]
fn turned_board_reads_right_wing_down_as_positive_roll() {
    let board = yaw180();
    // Published roll is the Euler roll negated: right wing down +10° is -10°.
    let sensor = board.inverse() * airframe_accel(-10.0, 0.0);
    let a = converge(
        &mut AttitudePipeline::with_board(board),
        sensor,
        Vector3::zeros(),
    );
    assert!((deg(a.roll) - 10.0).abs() < 0.5, "{a:?}");
    assert!(deg(a.pitch).abs() < 0.5, "{a:?}");
}

#[test]
fn turned_board_rates_follow_the_airframe() {
    let board = yaw180();
    // Airframe pitch rate +0.1 rad/s (nose coming up), roll rate +0.2 in the
    // published sense (sensor-convention x is negated), yaw rate +0.05.
    let airframe_gyro = Vector3::new(-0.2, 0.1, 0.05);
    let sensor_gyro = board.inverse() * airframe_gyro;
    let mut p = AttitudePipeline::with_board(board);
    let mut last = None;
    // Short: the rates settle through the 30 Hz low-pass long before the
    // attitude drifts far.
    for _ in 0..500 {
        last = Some(p.fuse(p.debias(sensor_gyro), Vector3::new(0.0, 0.0, G), None));
    }
    let a = last.unwrap();
    assert!((a.pitch_rate - 0.1).abs() < 1e-3, "{a:?}");
    assert!((a.roll_rate - 0.2).abs() < 1e-3, "{a:?}");
    assert!((a.yaw_rate - 0.05).abs() < 1e-3, "{a:?}");
}

#[test]
fn level_cal_tilt_composes_before_the_board() {
    let board = yaw180();
    // Board turned round and tilted on its mount: the sensor sees
    // v_sensor = tilt⁻¹ × board⁻¹ × v_airframe.
    let tilt =
        UnitQuaternion::from_euler_angles(2.0_f32.to_radians(), (-3.0_f32).to_radians(), 0.0);
    let to_sensor = |v: Vector3<f32>| tilt.inverse() * (board.inverse() * v);

    // Level cal on the level airframe recovers the tilt from raw sensor accel.
    let level = compute_mount(to_sensor(airframe_accel(0.0, 0.0)), 0.0, 0.1).unwrap();

    let mut p = AttitudePipeline::with_board(board);
    p.set_level_mount(Some(&level));
    let flat = converge(
        &mut p,
        to_sensor(airframe_accel(0.0, 0.0)),
        Vector3::zeros(),
    );
    assert!(
        deg(flat.pitch).abs() < 0.1 && deg(flat.roll).abs() < 0.1,
        "{flat:?}"
    );

    let mut p = AttitudePipeline::with_board(board);
    p.set_level_mount(Some(&level));
    let up = converge(
        &mut p,
        to_sensor(airframe_accel(0.0, 10.0)),
        Vector3::zeros(),
    );
    assert!((deg(up.pitch) - 10.0).abs() < 0.5, "{up:?}");

    // Clearing the level cal leaves the board orientation in place.
    p.set_level_mount(None);
    assert!(p.mount.angle_to(&board) < 1e-6);
}

#[test]
fn identity_board_is_the_plain_pipeline_bit_for_bit() {
    // The dart's case: with_board(identity) must fuse exactly as new() does.
    let mut a = AttitudePipeline::new();
    let mut b = AttitudePipeline::with_board(UnitQuaternion::identity());
    b.set_level_mount(None);
    for i in 0..2000 {
        let t = i as f32 * 1e-3;
        let gyro = Vector3::new(0.3 * t.sin(), 0.2 * t.cos(), 0.05);
        let accel = Vector3::new(0.5 * t.cos(), -0.4 * t.sin(), G);
        assert_eq!(
            a.fuse(a.debias(gyro), accel, None),
            b.fuse(b.debias(gyro), accel, None),
            "sample {i}"
        );
    }
}

#[test]
fn level_cal_display_is_in_the_airframe_frame() {
    // A board sitting 3° nose down on its mount, in the sensor's own frame.
    let tilt = UnitQuaternion::from_euler_angles(0.0, (-3.0_f32).to_radians(), 0.0);
    let level = compute_mount(tilt.inverse() * Vector3::new(0.0, 0.0, G), 0.0, 0.1).unwrap();
    let (r, p) = mount_to_display_deg(&level_in_airframe(&UnitQuaternion::identity(), &level));
    assert!(
        r.abs() < 0.01 && (p + 3.0).abs() < 0.01,
        "identity board: {r} {p}"
    );
    // Turned round, the sensor's nose-down is the airframe's nose-up.
    let (r, p) = mount_to_display_deg(&level_in_airframe(&yaw180(), &level));
    assert!(
        r.abs() < 0.01 && (p - 3.0).abs() < 0.01,
        "turned board: {r} {p}"
    );
}
