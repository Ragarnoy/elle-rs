//! Proposal 0009: `attitude::seed` is where the AHRS stays when it is still.
//!
//! A pipeline seeded from one still accel + mag sample, then fed that sample
//! for 60 s, must not move (the boot transient comes from starting elsewhere).

use elle_control::attitude::{AttitudePipeline, angles, seed};
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
