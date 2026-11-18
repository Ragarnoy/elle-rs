//! Static check: constant accel (level) and a constant mag with a given dip;
//! start Madgwick at the TRIAD seed and at identity, run 120 s, print where
//! each ends. `cargo run -p elle-replay --example seed_static`
use core::time::Duration;
use elle_control::attitude::{angles, seed};
use nalgebra::{UnitQuaternion, Vector3};
use uf_ahrs::{Ahrs, Madgwick, MadgwickParams};
fn main() {
    let dt = Duration::from_micros(elle_config::AHRS_SAMPLE_PERIOD_US);
    let accel = Vector3::new(0.0, 0.0, 9.81);
    for dip_deg in [-60.0f32, -20.0, 0.0, 20.0, 60.0, 75.0] {
        for head_deg in [-164.0f32, 44.0, 90.0] {
            // Field in the body frame: horizontal part along `head` from x, z by dip (+ = up).
            let h = head_deg.to_radians();
            let dip = dip_deg.to_radians();
            let mag = Vector3::new(dip.cos() * h.cos(), dip.cos() * h.sin(), dip.sin());
            let q0 = seed(&accel, &mag).unwrap();
            let mut res = String::new();
            for (name, q) in [("seed", q0), ("ident", UnitQuaternion::identity())] {
                let mut f = Madgwick::new(
                    dt,
                    MadgwickParams {
                        beta: elle_config::AHRS_BETA,
                    },
                );
                f.set_orientation(q);
                let mut worst = 0f32;
                for _ in 0..120_000 {
                    let q = f.update(Vector3::zeros(), accel, mag);
                    let (p, r, _) = angles(&q);
                    worst = worst.max(p.hypot(r).to_degrees());
                }
                let (p, r, y) = angles(&f.orientation());
                res += &format!(
                    "  {name}: end p{:+5.1} r{:+5.1} y{:+7.1} worst tilt {:5.1}",
                    p.to_degrees(),
                    r.to_degrees(),
                    y.to_degrees(),
                    worst
                );
            }
            let (_, _, ys) = angles(&q0);
            println!(
                "dip {dip_deg:+4.0} field-heading {head_deg:+5.0} seed yaw {:+7.1}{res}",
                ys.to_degrees()
            );
        }
    }
}
