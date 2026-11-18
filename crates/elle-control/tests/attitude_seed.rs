//! Proposal 0009: `attitude::seed` is where the AHRS stays when it is still.
//!
//! A pipeline seeded from one still accel + mag sample, then fed that sample
//! for 60 s, must not move (the boot transient comes from starting elsewhere).

use elle_control::attitude::{
    AttitudePipeline, SEED_MIN_SIN, angles, board_rotation, seed, tilt_disagreement,
};
use nalgebra::{UnitQuaternion, Vector3};

/// Body-frame accel and mag for an airframe at (pitch, roll, heading), with an
/// earth field of the given dip (degrees, + = pointing up), earth frame north,
/// west, up.
fn still_sample(pitch: f32, roll: f32, heading: f32, dip: f32) -> (Vector3<f32>, Vector3<f32>) {
    // Firmware convention: roll is negated after the Euler conversion.
    let q = UnitQuaternion::from_euler_angles(
        (-roll).to_radians(),
        pitch.to_radians(),
        heading.to_radians(),
    );
    let up = Vector3::new(0.0, 0.0, 9.81);
    let d = dip.to_radians();
    let field = Vector3::new(d.cos(), 0.0, d.sin()) * 0.45;
    (q.inverse() * up, q.inverse() * field)
}

#[test]
fn seeded_pipeline_stays_put() {
    for heading in [0.0f32, 90.0, 180.0, -164.0] {
        for (pitch, roll) in [(0.0f32, 0.0f32), (20.0, 0.0), (0.0, -20.0), (-15.0, 10.0)] {
            for dip in [-60.0f32, 20.0, 63.0] {
                let (accel, mag) = still_sample(pitch, roll, heading, dip);
                let q = seed(&accel, &mag).expect("seed");
                let (p, r, y) = angles(&q);
                assert!(
                    (p.to_degrees() - pitch).abs() < 0.01 && (r.to_degrees() - roll).abs() < 0.01,
                    "seed tilt {p} {r} for {pitch} {roll}"
                );
                let dy = (y.to_degrees() - heading + 540.0).rem_euclid(360.0) - 180.0;
                assert!(
                    dy.abs() < 0.01,
                    "seed heading {} for {heading}",
                    y.to_degrees()
                );

                let mut pl = AttitudePipeline::with_board(UnitQuaternion::identity());
                pl.set_quat(q);
                let mut worst = 0f32;
                for _ in 0..60_000 {
                    let a = pl.fuse(Vector3::zeros(), accel, Some(&mag));
                    worst =
                        worst.max((a.pitch.to_degrees() - pitch).hypot(a.roll.to_degrees() - roll));
                }
                assert!(
                    worst < 0.1,
                    "heading {heading} tilt {pitch}/{roll} dip {dip}: drifted {worst}°"
                );
            }
        }
    }
}

#[test]
fn identity_start_shows_the_transient() {
    // The case the seed fixes: level, field 164° from north, 20° dip.
    let (accel, mag) = still_sample(0.0, 0.0, -164.0, 20.0);
    let mut pl = AttitudePipeline::with_board(UnitQuaternion::identity());
    let mut worst = 0f32;
    for _ in 0..60_000 {
        let a = pl.fuse(Vector3::zeros(), accel, Some(&mag));
        worst = worst.max(a.pitch.to_degrees().hypot(a.roll.to_degrees()));
    }
    assert!(worst > 20.0, "expected a large transient, got {worst}°");
}

#[test]
fn degenerate_inputs() {
    let up = Vector3::new(0.0, 0.0, 9.81);
    assert!(seed(&Vector3::zeros(), &Vector3::x()).is_none());
    assert!(seed(&up, &Vector3::zeros()).is_none());
    assert!(seed(&up, &Vector3::new(0.0, 0.0, 0.4)).is_none());
}

fn wrap_deg(d: f32) -> f32 {
    (d + 540.0).rem_euclid(360.0) - 180.0
}

/// The firmware's frames: the platform's board rotation (eagle 180°, dart 0°)
/// and a level-cal tilt. `seed_from` takes what `fuse` takes (accel in the
/// sensor frame, mag in the airframe frame) and must seed the airframe
/// attitude, which then holds.
#[test]
fn seed_from_in_the_firmware_frames() {
    let level = UnitQuaternion::from_euler_angles(2.0f32.to_radians(), -3.0f32.to_radians(), 0.0);
    let mount = board_rotation() * level;
    for heading in [0.0f32, 134.0, -164.0] {
        for (pitch, roll) in [(0.0f32, 0.0f32), (15.0, -10.0)] {
            let (accel_air, mag_air) = still_sample(pitch, roll, heading, -62.0);
            let accel_sensor = mount.inverse() * accel_air;

            let mut pl = AttitudePipeline::with_board(board_rotation());
            pl.set_level_mount(Some(&level));
            assert!((pl.accel_to_airframe(accel_sensor) - accel_air).norm() < 1e-4);
            let q = pl.seed_from(accel_sensor, &mag_air).expect("seed");
            let (p, r, y) = angles(&q);
            assert!(
                (p.to_degrees() - pitch).abs() < 0.05 && (r.to_degrees() - roll).abs() < 0.05,
                "seed tilt {} {} for {pitch} {roll}",
                p.to_degrees(),
                r.to_degrees()
            );
            assert!(
                wrap_deg(y.to_degrees() - heading).abs() < 0.05,
                "seed yaw {}",
                y.to_degrees()
            );

            let (mut worst_tilt, mut worst_yaw) = (0f32, 0f32);
            for _ in 0..60_000 {
                let a = pl.fuse(Vector3::zeros(), accel_sensor, Some(&mag_air));
                worst_tilt = worst_tilt
                    .max((a.pitch.to_degrees() - pitch).hypot(a.roll.to_degrees() - roll));
                worst_yaw = worst_yaw.max(wrap_deg(a.yaw.to_degrees() - heading).abs());
            }
            assert!(
                worst_tilt < 0.5 && worst_yaw < 0.5,
                "heading {heading} tilt {pitch}/{roll}: drifted tilt {worst_tilt}°, yaw {worst_yaw}°"
            );
        }
    }
}

/// Part B's guard: zero for the seed's own sample, the tilt offset otherwise,
/// in the firmware's frames.
#[test]
fn tilt_disagreement_in_the_firmware_frames() {
    let pl = AttitudePipeline::with_board(board_rotation());
    let (accel_air, mag_air) = still_sample(10.0, -5.0, 134.0, -62.0);
    let accel_sensor = board_rotation().inverse() * accel_air;
    let accel = pl.accel_to_airframe(accel_sensor);
    let q = seed(&accel, &mag_air).expect("seed");
    assert!(tilt_disagreement(&q, &accel).unwrap().to_degrees() < 0.01);
    // The filter tilted 7° about an earth-horizontal axis: the guard reads 7°.
    let off = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 7.0f32.to_radians());
    let d = tilt_disagreement(&(off * q), &accel).unwrap().to_degrees();
    assert!((d - 7.0).abs() < 0.01, "{d}");
    // Heading does not count.
    let yawed = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 1.0) * q;
    assert!(tilt_disagreement(&yawed, &accel).unwrap().to_degrees() < 0.01);
    assert!(tilt_disagreement(&q, &Vector3::zeros()).is_none());
}

/// A field within ~6° of the vertical has too little horizontal part for a
/// heading, whatever its units; just outside it, the seed is accepted.
#[test]
fn near_vertical_field_is_refused() {
    let up = Vector3::new(0.0, 0.0, 9.81);
    // Raw-count sized field, 3° off vertical: rejected (the old absolute
    // threshold accepted it).
    let a = 3.0f32.to_radians();
    assert!(seed(&up, &(Vector3::new(a.sin(), 0.0, -a.cos()) * 1.0e5)).is_none());
    // Accel and mag nearly parallel, the other way up.
    assert!(seed(&up, &(Vector3::new(a.sin(), 0.0, a.cos()) * 0.4)).is_none());
    // 8° off vertical (sin 0.139 > SEED_MIN_SIN): accepted.
    let b = 8.0f32.to_radians();
    assert!(b.sin() > SEED_MIN_SIN);
    assert!(seed(&up, &(Vector3::new(b.sin(), 0.0, -b.cos()) * 0.4)).is_some());
}
