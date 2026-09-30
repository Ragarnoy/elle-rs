//! `elle-replay LOG_NNNN.ulg [--compare] [--csv out.csv] [--json]`
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
use elle_replay::reference;
use elle_replay::report::{self, Report, ScoreRow};
use elle_replay::sim;

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
    /// Count samples as "turning" by gyro turn rate (> 6°/s) instead of by
    /// bank: for ground tests in a vehicle, which turns without banking.
    #[arg(long)]
    score_by_rate: bool,
    /// Print the report as JSON instead of text.
    #[arg(long)]
    json: bool,
}

fn print_scores(title: &str, scores: &[ScoreRow]) {
    println!("\n  {title}");
    println!(
        "  in turns                      roll error mean R/L      roll RMS    pitch RMS R/L    samples R/L"
    );
    for s in scores {
        println!(
            "  {:<22} {:+7.2}° {:+7.2}°     {:6.2}°    {:5.2}° {:5.2}°    {:>6} {:>6}",
            s.name,
            s.roll_mean_deg[0],
            s.roll_mean_deg[1],
            s.roll_rms_deg,
            s.pitch_rms_deg[0],
            s.pitch_rms_deg[1],
            s.samples[0],
            s.samples[1]
        );
    }
}

fn print_report(r: &Report) {
    println!("{}", r.log);
    if r.truncated {
        println!("  file ends inside a record (power cut?): read up to there");
    }
    let c = &r.coverage;
    println!(
        "  samples {} ({:.1} s), synced {} ({:.1}%), gaps {} ({} samples lost), ULog dropouts {}",
        c.samples, c.seconds, c.synced, c.synced_pct, c.gaps, c.lost, c.dropouts
    );
    println!(
        "  contexts {}, mag changes {}, encode round-trip errors {}",
        c.contexts, c.mag_changes, c.roundtrip_errors
    );
    let f = &r.faithfulness;
    println!(
        "  attitude_data: {} exact, {} mismatched, {} not checked (source sample may be unsynced or lost)",
        f.exact, f.mismatched, f.unchecked
    );
    if let Some(cmp) = &r.comparison {
        println!(
            "\n  gyro reference: covers {:.0}% of the samples (needs straight-and-level stretches of {} s between manoeuvres)",
            cmp.reference_coverage_pct,
            reference::ANCHOR_S
        );
        print_scores("against the gyro reference", &cmp.vs_reference);
        if let Some(t) = &cmp.vs_truth {
            print_scores("against the simulated truth", t);
        }
        if let Some(r) = &cmp.reference_vs_truth {
            println!(
                "  (the reference itself is {:.2}° roll / {:.2}° pitch RMS off the truth in turns)",
                r.roll_rms_deg,
                r.pitch_rms_deg[0].max(r.pitch_rms_deg[1])
            );
        }
        println!("\n  samples fused without the accelerometer (gate)");
        for (name, pct) in &cmp.gated_pct {
            println!("  {name:<22} {pct:5.1}%");
        }
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
        if !args.json {
            println!("simulated flight written to {}", args.log.display());
        }
        Some(flight.truth)
    } else {
        None
    };

    let data =
        std::fs::read(&args.log).with_context(|| format!("reading {}", args.log.display()))?;
    let opts = report::Options {
        compare: args.compare,
        gate_g: args.gate_g,
        score_by_rate: args.score_by_rate,
    };
    let (r, replay) = report::analyse(
        &args.log.display().to_string(),
        &data,
        &opts,
        truth.as_deref(),
    )?;

    if args.json {
        println!("{}", serde_json::to_string_pretty(&r)?);
    } else {
        print_report(&r);
    }

    if let Some(path) = &args.csv {
        let file =
            std::fs::File::create(path).with_context(|| format!("creating {}", path.display()))?;
        elle_replay::write_csv(&replay, std::io::BufWriter::new(file))?;
        if !args.json {
            println!("  wrote {}", path.display());
        }
    }

    if let Some(p) = r.problems.first() {
        bail!("{p}");
    }
    if !args.json {
        println!("\nOK: the replay reproduces the firmware's attitude exactly");
    }
    Ok(())
}
