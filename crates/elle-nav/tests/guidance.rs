//! L1 guidance flown in closed loop against a point-mass wing, with wind.

use elle_nav::Ne;
use elle_nav::l1::{self, G, L1Config, Path, Phase};

const AIRSPEED: f32 = 15.0;
const DT: f32 = 0.04; // 25 Hz, the firmware's navigator rate

/// Coordinated-turn kinematics: bank follows the demand through a 0.3 s lag
/// and a 90°/s rate limit (the attitude loop), heading rate = g tan φ / V.
struct Wing {
    pos: Ne,
    heading: f32,
    bank: f32,
    wind: Ne,
}

impl Wing {
    fn ground_vel(&self) -> Ne {
        Ne::from_bearing(self.heading) * AIRSPEED + self.wind
    }

    fn step(&mut self, bank_demand_deg: f32) {
        let target = bank_demand_deg.to_radians();
        let rate = ((target - self.bank) / 0.3).clamp(-90f32.to_radians(), 90f32.to_radians());
        self.bank += rate * DT;
        self.heading += G * self.bank.tan() / AIRSPEED * DT;
        self.pos += self.ground_vel() * DT;
    }
}

fn cfg() -> L1Config {
    L1Config::from_config()
}

/// Fly `secs` seconds; return the track errors seen.
fn fly(w: &mut Wing, path: &Path, secs: f32) -> Vec<(f32, l1::Guidance)> {
    let mut out = Vec::new();
    let mut t = 0.0;
    while t < secs {
        let g = l1::guide(&cfg(), path, w.pos, w.ground_vel()).expect("flying fast enough");
        assert!(g.bank_deg.abs() <= cfg().max_bank_deg + 1e-3);
        out.push((t, g));
        w.step(g.bank_deg);
        t += DT;
    }
    out
}

fn line_east() -> Path {
    Path::Line {
        from: Ne::new(0.0, 0.0),
        to: Ne::new(0.0, 1000.0),
    }
}

#[test]
fn line_on_track_needs_no_bank() {
    let g = l1::guide(&cfg(), &line_east(), Ne::new(0.0, 10.0), Ne::new(0.0, 15.0)).unwrap();
    assert!(
        g.bank_deg.abs() < 1e-3 && g.track_error_m.abs() < 1e-3,
        "{g:?}"
    );
    assert_eq!(g.phase, Phase::Line);
}

#[test]
fn line_sign_convention() {
    // Flying east, 30 m south of the line: south is to the right of an
    // eastbound track, so the error is positive and the turn is left.
    let g = l1::guide(
        &cfg(),
        &line_east(),
        Ne::new(-30.0, 0.0),
        Ne::new(0.0, 15.0),
    )
    .unwrap();
    assert!(g.track_error_m > 29.0, "{g:?}");
    assert!(g.bank_deg < 0.0, "{g:?}");
}

#[test]
fn line_converges_from_offset_in_crosswind() {
    for wind in [Ne::ZERO, Ne::new(5.0, 0.0), Ne::new(-5.0, 2.0)] {
        let mut w = Wing {
            pos: Ne::new(-150.0, 0.0),
            heading: 90f32.to_radians(),
            bank: 0.0,
            wind,
        };
        let log = fly(&mut w, &line_east(), 60.0);
        let late: Vec<f32> = log
            .iter()
            .filter(|(t, _)| *t > 40.0)
            .map(|(_, g)| g.track_error_m)
            .collect();
        let worst = late.iter().fold(0f32, |m, e| m.max(e.abs()));
        assert!(
            worst < 2.0,
            "wind {wind:?}: track error after 40 s up to {worst} m"
        );
        // Crossing the line, the overshoot stays small (damped approach).
        let overshoot = log
            .iter()
            .map(|(_, g)| -g.track_error_m)
            .fold(0f32, f32::max);
        assert!(overshoot < 6.0, "wind {wind:?}: overshoot {overshoot} m");
    }
}

#[test]
fn line_start_from_behind() {
    // Well behind the start, heading away: fly back to it first.
    let g = l1::guide(
        &cfg(),
        &line_east(),
        Ne::new(0.0, -500.0),
        Ne::new(0.0, -15.0),
    )
    .unwrap();
    assert_eq!(g.phase, Phase::ToLineStart);
    assert!(g.bank_limited, "a reversal is a full-rate turn: {g:?}");
}

fn loiter(clockwise: bool) -> Path {
    Path::Loiter {
        center: Ne::ZERO,
        radius_m: elle_config::NAV_LOITER_RADIUS_M,
        clockwise,
    }
}

#[test]
fn loiter_captures_and_holds_the_circle() {
    for clockwise in [true, false] {
        for wind in [Ne::ZERO, Ne::new(4.0, -3.0)] {
            let mut w = Wing {
                pos: Ne::new(400.0, -250.0),
                heading: 0.0, // north, away from the circle
                bank: 0.0,
                wind,
            };
            let log = fly(&mut w, &loiter(clockwise), 150.0);
            let late: Vec<&l1::Guidance> = log
                .iter()
                .filter(|(t, _)| *t > 100.0)
                .map(|(_, g)| g)
                .collect();
            let worst = late.iter().fold(0f32, |m, g| m.max(g.track_error_m.abs()));
            assert!(
                worst < 5.0,
                "cw {clockwise} wind {wind:?}: radial error up to {worst} m"
            );
            assert!(late.iter().all(|g| g.phase == Phase::LoiterCircle));
            // No wind, the bank is the coordinated turn for the circle.
            if wind == Ne::ZERO {
                let expect = (AIRSPEED * AIRSPEED / (elle_config::NAV_LOITER_RADIUS_M * G))
                    .atan()
                    .to_degrees();
                let last = late.last().unwrap().bank_deg.abs();
                assert!(
                    (last - expect).abs() < 0.5,
                    "bank {last}, expected {expect}"
                );
            }
            // Going round the right way: clockwise = right bank on average.
            let mean_bank = late.iter().map(|g| g.bank_deg).sum::<f32>() / late.len() as f32;
            assert_eq!(mean_bank > 0.0, clockwise, "mean bank {mean_bank}");
        }
    }
}

#[test]
fn no_guidance_when_too_slow() {
    let slow = Ne::new(0.0, cfg().min_ground_speed_ms * 0.5);
    assert!(l1::guide(&cfg(), &loiter(true), Ne::new(10.0, 10.0), slow).is_none());
}
