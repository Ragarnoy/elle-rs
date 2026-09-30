//! The gyro-only reference, checked against simulated truth, and what it says
//! about the turn corrections.
//!
//!   cargo test -p elle-replay --target x86_64-unknown-linux-gnu --release --test reference

use elle_replay::reference::{self, Score};
use elle_replay::sim::{self, Config, Segment};
use elle_replay::ulog::ULog;
use elle_replay::variants::{self, Spec};

/// Turns both ways at 30°, a pull-up and a full circle, with straight legs
/// between them to anchor the reference (~75 s).
fn profile() -> Vec<Segment> {
    use Segment::{PullUp, Straight, Turn};
    vec![
        Straight { secs: 6.0 },
        Turn {
            bank_deg: 30.0,
            secs: 15.0,
        },
        Straight { secs: 6.0 },
        Turn {
            bank_deg: -30.0,
            secs: 15.0,
        },
        Straight { secs: 6.0 },
        PullUp {
            pitch_deg: 15.0,
            secs: 3.0,
        },
        Straight { secs: 6.0 },
        Turn {
            bank_deg: 20.0,
            secs: 14.0,
        },
        Straight { secs: 6.0 },
    ]
}

fn specs(names: &[&str]) -> Vec<Spec> {
    variants::default_specs()
        .into_iter()
        .filter(|s| names.contains(&s.name.as_str()))
        .collect()
}

struct Run {
    vs_truth: Vec<Score>,
    vs_reference: Vec<Score>,
    reference_vs_truth: Score,
}

fn run(cfg: &Config, names: &[&str]) -> Run {
    let f = sim::fly(cfg, &profile()).unwrap();
    let rep = elle_replay::replay_with(&ULog::parse(&f.ulog).unwrap(), &specs(names)).unwrap();
    let truth: Vec<Option<[f32; 3]>> = rep
        .samples
        .iter()
        .map(|s| {
            let t = f.truth[s.index as usize];
            Some([t.pitch as f32, t.roll as f32, t.yaw as f32])
        })
        .collect();
    let refr = reference::build(&rep);
    Run {
        reference_vs_truth: reference::score("reference", refr.iter().copied(), &truth),
        vs_truth: reference::score_all(&rep, &truth),
        vs_reference: reference::score_all(&rep, &refr),
    }
}

fn get<'a>(scores: &'a [Score], name: &str) -> &'a Score {
    scores.iter().find(|s| s.name == name).unwrap()
}

fn assert_reference_holds(r: &Run, tol_deg: f32) {
    let s = &r.reference_vs_truth;
    assert!(s.samples.iter().all(|n| *n > 1000), "turns scored: {s:?}");
    let worst = s
        .roll_rms_deg
        .into_iter()
        .chain(s.pitch_rms_deg)
        .fold(0f32, f32::max);
    assert!(
        worst < tol_deg,
        "reference off the truth by {worst:.2}° RMS: {s:?}"
    );
    // Scoring against it tells the same story as scoring against the truth.
    for (t, rf) in r.vs_truth.iter().zip(&r.vs_reference) {
        let d = (t.roll_rms_both() - rf.roll_rms_both()).abs();
        assert!(
            d < tol_deg,
            "{}: {:.2}° vs truth, {:.2}° vs reference",
            t.name,
            t.roll_rms_both(),
            rf.roll_rms_both()
        );
    }
}

#[test]
fn reference_matches_truth_in_calm_air() {
    let r = run(&Config::default(), &["madgwick-cc"]);
    assert_reference_holds(&r, 0.5);
}

#[test]
fn reference_survives_vibration_and_residual_gyro_bias() {
    let cfg = Config {
        vibration: 3.0,
        gyro_bias_residual: 0.05f64.to_radians(),
        ..Config::default()
    };
    let r = run(&cfg, &["madgwick-cc"]);
    assert_reference_holds(&r, 0.7);
}

#[test]
fn centripetal_correction_removes_the_turn_error_in_calm_air() {
    let r = run(&Config::default(), &["madgwick-cc", "madgwick-ce"]);
    let fw = get(&r.vs_reference, "firmware").roll_rms_both();
    let cc = get(&r.vs_reference, "madgwick-cc").roll_rms_both();
    let ce = get(&r.vs_reference, "madgwick-ce").roll_rms_both();
    assert!(fw > 5.0, "firmware {fw:.1}° RMS in turns");
    assert!(cc < 0.5, "centripetal {cc:.2}°");
    assert!(ce < 3.0, "earth-frame {ce:.2}°");
}

#[test]
fn in_wind_ground_speed_spoils_the_centripetal_correction() {
    // Documents the limit of correcting with ground speed: 6 m/s of wind
    // makes it wrong by ω × wind around the circle, while the earth-frame
    // correction (GNSS acceleration) does not care. Which matters more on
    // the aircraft is for the flight data to say.
    let cfg = Config {
        wind_ned: nalgebra::Vector3::new(0.0, 6.0, 0.0),
        ..Config::default()
    };
    let r = run(&cfg, &["madgwick-cc", "madgwick-ce"]);
    let cc = get(&r.vs_reference, "madgwick-cc").roll_rms_both();
    let ce = get(&r.vs_reference, "madgwick-ce").roll_rms_both();
    assert!(
        cc > 2.0 * ce,
        "centripetal {cc:.2}° vs earth-frame {ce:.2}° RMS"
    );
    assert!(ce < 3.0, "earth-frame {ce:.2}°");
}
