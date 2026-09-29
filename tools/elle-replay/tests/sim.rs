//! The simulator's physics and frame mapping, checked against the firmware's
//! conventions, and what it shows about the current filter in turns.
//!
//!   cargo test -p elle-replay --target x86_64-unknown-linux-gnu --release --test sim

use elle_replay::sim::{self, Config, Segment};
use elle_replay::ulog::ULog;

/// Noise-free sensors: the frame and physics checks should be exact-ish.
fn quiet() -> Config {
    Config {
        gyro_noise: 0.0,
        accel_noise: 0.0,
        gnss_vel_noise: 0.0,
        ..Config::default()
    }
}

fn deg(r: f64) -> f64 {
    r.to_degrees()
}

#[test]
fn truth_reads_back_as_flown_in_the_firmware_convention() {
    use Segment::{PullUp, Straight, Turn};
    let f = sim::fly(
        &quiet(),
        &[
            Straight { secs: 1.0 },
            Turn {
                bank_deg: 30.0,
                secs: 3.0,
            },
            Straight { secs: 2.0 },
            PullUp {
                pitch_deg: 20.0,
                secs: 2.0,
            },
        ],
    )
    .unwrap();
    let at = |ms: usize| f.truth[ms];
    assert!(deg(at(500).roll).abs() < 1e-9 && deg(at(500).pitch).abs() < 1e-9);
    // Right turn: right wing down reads positive roll, like the firmware.
    assert!((deg(at(2500).roll) - 30.0).abs() < 1e-6, "{:?}", at(2500));
    // Pull-up peaks at half its length: nose up reads positive pitch.
    assert!((deg(at(7000).pitch) - 20.0).abs() < 0.01, "{:?}", at(7000));
}

#[test]
fn the_firmware_filter_follows_the_simulated_motion() {
    // Straight flight, and the roll-in of a turn, where the motion is almost
    // all gyro: the accel correction (towards "level", at up to 2 beta rad/s,
    // ~3.8°/s) has only had fractions of a second to act.
    let f = sim::fly(
        &quiet(),
        &[
            Segment::Straight { secs: 5.0 },
            Segment::Turn {
                bank_deg: 30.0,
                secs: 3.0,
            },
        ],
    )
    .unwrap();
    let log = ULog::parse(&f.ulog).unwrap();
    let rep = elle_replay::replay(&log).unwrap();
    // 4000: straight; 5200 and 5400: 0.2 and 0.4 s into the roll-in.
    for k in [4000usize, 5200, 5400] {
        let (t, s) = (f.truth[k], rep.samples[k].att.unwrap());
        let err = (f64::from(s.roll) - t.roll)
            .abs()
            .max((f64::from(s.pitch) - t.pitch).abs());
        assert!(deg(err) < 2.0, "sample {k}: firmware {s:?} truth {t:?}");
    }
    // Mid roll-in the bank is growing the same way in both: a wrong gyro axis
    // or sign in the frame mapping would roll the filter the other way.
    assert!(rep.samples[5400].att.unwrap().roll > 0.25);
}

#[test]
fn replay_of_a_simulated_flight_is_exact() {
    let f = sim::fly(&Config::default(), &sim::standard_profile()).unwrap();
    let log = ULog::parse(&f.ulog).unwrap();
    let rep = elle_replay::replay(&log).unwrap();
    let fa = elle_replay::faithfulness(&log, &rep);
    assert!(fa.ok() && fa.mismatched == 0, "{fa:?}");
    // A trailing partial batch (under 10 samples) is never recorded.
    assert_eq!(rep.coverage.synced, f.truth.len() as u64 / 10 * 10);
    assert!(log.get("gnss_data").unwrap().records.len() > 5 * 180);
}

/// Mean firmware-minus-truth roll over the steady middle of a segment, degrees.
fn steady_roll_error(f: &sim::Flight, rep: &elle_replay::Replay, from_s: f64, to_s: f64) -> f64 {
    let (a, b) = ((from_s * 1000.0) as usize, (to_s * 1000.0) as usize);
    let sum: f64 = (a..b)
        .map(|k| f64::from(rep.samples[k].att.unwrap().roll) - f.truth[k].roll)
        .sum();
    deg(sum / (b - a) as f64)
}

#[test]
fn current_filter_levels_itself_in_a_sustained_turn() {
    // Not a requirement: this documents why the turn correction exists. The
    // accelerometer points through the belly in a coordinated turn, so the
    // filter corrects towards wings level, at up to 2 beta rad/s (~3.8°/s).
    let f = sim::fly(
        &Config::default(),
        &[
            Segment::Straight { secs: 5.0 },
            Segment::Turn {
                bank_deg: 15.0,
                secs: 20.0,
            },
        ],
    )
    .unwrap();
    let rep = elle_replay::replay(&ULog::parse(&f.ulog).unwrap()).unwrap();
    let early = steady_roll_error(&f, &rep, 5.4, 5.6);
    let late = steady_roll_error(&f, &rep, 18.0, 23.0);
    assert!(early.abs() < 2.5, "just after the roll-in: {early:.1}°");
    // Simulated: a 15° turn reads ~5° of bank after 15 s.
    assert!(
        late < -8.0,
        "15° turn after ~15 s reads {:.1}° of bank",
        15.0 + late
    );
}

/// Roll RMS error (firmware − truth) in steady turns, degrees.
fn turn_roll_rms(f: &sim::Flight, rep: &elle_replay::Replay) -> f64 {
    let (mut sq, mut n) = (0.0, 0.0);
    for (s, t) in rep.samples.iter().zip(&f.truth) {
        if t.roll.abs() < 10f64.to_radians() {
            continue;
        }
        let d = f64::from(s.att.unwrap().roll) - t.roll;
        sq += d * d;
        n += 1.0;
    }
    deg((sq / n).sqrt())
}

#[test]
fn firmware_turn_compensation_modes_fly_and_replay_exactly() {
    use elle_control::attitude::AhrsTurnComp;
    let profile = [
        Segment::Straight { secs: 5.0 },
        Segment::Turn {
            bank_deg: 30.0,
            secs: 15.0,
        },
        Segment::Straight { secs: 5.0 },
        Segment::Turn {
            bank_deg: -30.0,
            secs: 15.0,
        },
        Segment::Straight { secs: 5.0 },
    ];
    let wind = nalgebra::Vector3::new(0.0, 6.0, 0.0);
    let mut rms = Vec::new();
    for (mode, wind) in [
        (AhrsTurnComp::Off, nalgebra::Vector3::zeros()),
        (AhrsTurnComp::Centripetal, nalgebra::Vector3::zeros()),
        (AhrsTurnComp::GnssAccel, nalgebra::Vector3::zeros()),
        (AhrsTurnComp::Centripetal, wind),
        (AhrsTurnComp::GnssAccel, wind),
    ] {
        let cfg = Config {
            turn_comp: mode,
            wind_ned: wind,
            ..Config::default()
        };
        let f = sim::fly(&cfg, &profile).unwrap();
        let log = ULog::parse(&f.ulog).unwrap();
        let rep = elle_replay::replay(&log).unwrap();
        let fa = elle_replay::faithfulness(&log, &rep);
        assert!(fa.ok(), "{mode:?}: replay not exact: {fa:?}");
        if mode != AhrsTurnComp::Off {
            assert!(
                log.get("imu_raw_fix")
                    .is_some_and(|s| s.records.len() > 100)
            );
        }
        rms.push((mode, wind.norm(), turn_roll_rms(&f, &rep)));
    }
    let get = |m, w: f64| {
        rms.iter()
            .find(|(mm, ww, _)| *mm == m && (*ww - w).abs() < 1e-9)
            .unwrap()
            .2
    };
    let off = get(AhrsTurnComp::Off, 0.0);
    assert!(off > 5.0, "no compensation: {off:.1}°");
    assert!(get(AhrsTurnComp::Centripetal, 0.0) < 0.5, "{rms:?}");
    assert!(get(AhrsTurnComp::GnssAccel, 0.0) < 3.0, "{rms:?}");
    assert!(get(AhrsTurnComp::GnssAccel, 6.0) < 3.0, "{rms:?}");
    // Ground speed is not airspeed: in wind the centripetal mode loses most
    // of its gain (documented; the flight data decides between the modes).
    assert!(
        get(AhrsTurnComp::Centripetal, 6.0) > get(AhrsTurnComp::GnssAccel, 6.0),
        "{rms:?}"
    );
}
