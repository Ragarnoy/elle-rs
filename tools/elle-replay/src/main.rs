//! `elle-replay LOG_NNNN.ulg [--compare] [--csv out.csv]`
//! `elle-replay --simulate out.ulg [--wind-north N --wind-east E] [--vibration A] [--compare]`
//!
//! Replays a raw IMU log (`imu-raw-log` build) through the firmware's attitude
//! pipeline and checks it reproduces the attitude the aircraft logged; exits
//! non-zero when it does not. `--compare` also runs the alternative filters
//! and scores everything against a gyro-only reference through turns.
//! `--simulate` first writes a simulated flight (turns both ways at 15, 30 and
//! 45°, a pull-up, circles) to the given path, then analyses it the same way,
//! adding a score against the simulated truth.

use std::path::PathBuf;

use anyhow::{Context, Result, bail};
use clap::Parser;
use elle_replay::reference::{self, Score};
use elle_replay::sim;
use elle_replay::ulog::ULog;

#[derive(Parser)]
#[command(about = "Replay an Elle raw IMU log through the attitude pipeline")]
struct Args {
    /// ULog file from an imu-raw-log build (or, with --simulate, where to write one).
    log: PathBuf,
    /// Write every replayed sample to this CSV (for PlotJuggler), with the
    /// variants' angles when --compare is given.
    #[arg(long)]
    csv: Option<PathBuf>,
    /// Also run the alternative filters (Madgwick, Mahony, VQF; turn
    /// compensation; accel gating) and score them against the gyro reference.
    #[arg(long)]
    compare: bool,
    /// Accel gate threshold for the gated variants, g from 1 g.
    #[arg(long, default_value_t = elle_replay::variants::DEFAULT_GATE_G)]
    gate_g: f32,
    /// Write a simulated flight to LOG first, then analyse it.
    #[arg(long)]
    simulate: bool,
    /// Simulated wind towards north, m/s.
    #[arg(long, default_value_t = 0.0)]
    wind_north: f64,
    /// Simulated wind towards east, m/s.
    #[arg(long, default_value_t = 0.0)]
    wind_east: f64,
    /// Simulated motor vibration on the accel, m/s² (150 Hz).
    #[arg(long, default_value_t = 0.0)]
    vibration: f64,
    /// Simulated residual gyro bias, °/s.
    #[arg(long, default_value_t = 0.0)]
    gyro_bias_dps: f64,
}

fn print_scores(title: &str, scores: &[Score]) {
    println!("\n  {title}");
    println!(
        "  in turns (|bank| > 10°)       roll error mean R/L      roll RMS    pitch RMS R/L    samples R/L"
    );
    for s in scores {
        println!(
            "  {:<22} {:+7.2}° {:+7.2}°     {:6.2}°    {:5.2}° {:5.2}°    {:>6} {:>6}",
            s.name,
            s.roll_mean_deg[0],
            s.roll_mean_deg[1],
            s.roll_rms_both(),
            s.pitch_rms_deg[0],
            s.pitch_rms_deg[1],
            s.samples[0],
            s.samples[1]
        );
    }
}

fn main() -> Result<()> {
    let args = Args::parse();
    let truth = if args.simulate {
        let cfg = sim::Config {
            wind_ned: nalgebra::Vector3::new(args.wind_north, args.wind_east, 0.0),
            vibration: args.vibration,
            gyro_bias_residual: args.gyro_bias_dps.to_radians(),
            ..sim::Config::default()
        };
        let flight = sim::fly(&cfg, &sim::standard_profile())?;
        std::fs::write(&args.log, &flight.ulog)
            .with_context(|| format!("writing {}", args.log.display()))?;
        println!("simulated flight written to {}", args.log.display());
        Some(flight.truth)
    } else {
        None
    };

    let data =
        std::fs::read(&args.log).with_context(|| format!("reading {}", args.log.display()))?;
    let log = ULog::parse(&data)?;
    let specs: Vec<_> = if args.compare {
        elle_replay::variants::default_specs()
            .into_iter()
            .map(|mut s| {
                s.gate_g = s.gate_g.map(|_| args.gate_g);
                s
            })
            .collect()
    } else {
        Vec::new()
    };
    let replay = elle_replay::replay_with(&log, &specs)?;
    let c = &replay.coverage;

    println!("{}", args.log.display());
    if log.truncated {
        println!("  file ends inside a record (power cut?): read up to there");
    }
    println!(
        "  samples {} ({:.1} s), synced {} ({:.1}%), gaps {} ({} samples lost), ULog dropouts {}",
        c.samples,
        c.samples as f64 / 1000.0,
        c.synced,
        100.0 * c.synced as f64 / c.samples.max(1) as f64,
        c.gaps,
        c.lost,
        c.dropouts
    );
    println!(
        "  contexts {}, mag changes {}, encode round-trip errors {}",
        c.contexts, c.mag_changes, c.roundtrip_errors
    );

    let f = elle_replay::faithfulness(&log, &replay);
    println!(
        "  attitude_data: {} exact, {} mismatched, {} not checked (source sample may be unsynced or lost)",
        f.exact, f.mismatched, f.unchecked
    );

    if args.compare {
        let refr = reference::build(&replay);
        let covered = refr.iter().filter(|r| r.is_some()).count();
        println!(
            "\n  gyro reference: covers {:.0}% of the samples (needs straight-and-level stretches of {} s between manoeuvres)",
            100.0 * covered as f64 / refr.len().max(1) as f64,
            reference::ANCHOR_S
        );
        print_scores(
            "against the gyro reference",
            &reference::score_all(&replay, &refr),
        );
        if let Some(t) = &truth {
            let truth_angles: Vec<Option<[f32; 3]>> = replay
                .samples
                .iter()
                .map(|s| {
                    t.get(s.index as usize)
                        .map(|t| [t.pitch as f32, t.roll as f32, t.yaw as f32])
                })
                .collect();
            print_scores(
                "against the simulated truth",
                &reference::score_all(&replay, &truth_angles),
            );
            let r = reference::score("reference", refr.iter().copied(), &truth_angles);
            println!(
                "  (the reference itself is {:.2}° roll / {:.2}° pitch RMS off the truth in turns)",
                r.roll_rms_both(),
                r.pitch_rms_deg[0].max(r.pitch_rms_deg[1])
            );
        }
        println!("\n  samples fused without the accelerometer (gate)");
        for c in elle_replay::compare(&replay)
            .iter()
            .filter(|c| c.gated_share > 0.0)
        {
            println!("  {:<22} {:5.1}%", c.name, 100.0 * c.gated_share);
        }
    }

    if let Some(path) = &args.csv {
        let file =
            std::fs::File::create(path).with_context(|| format!("creating {}", path.display()))?;
        elle_replay::write_csv(&replay, std::io::BufWriter::new(file))?;
        println!("  wrote {}", path.display());
    }

    if c.roundtrip_errors > 0 {
        bail!("the logged integers do not reproduce the driver's floats (scale mismatch)");
    }
    if !f.ok() {
        bail!(
            "the replay does not reproduce the firmware (worst {:.4}°); results from it cannot be trusted",
            f.worst_deg
        );
    }
    println!("\nOK: the replay reproduces the firmware's attitude exactly");
    Ok(())
}
