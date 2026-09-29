//! `elle-replay LOG_NNNN.ulg [--csv out.csv]`
//!
//! Replays a raw IMU log (`imu-raw-log` build) through the firmware's attitude
//! pipeline and checks it reproduces the attitude the aircraft logged. Exits
//! non-zero when it does not.

use std::path::PathBuf;

use anyhow::{Context, Result};
use clap::Parser;
use elle_replay::ulog::ULog;

#[derive(Parser)]
#[command(about = "Replay an Elle raw IMU log through the attitude pipeline")]
struct Args {
    /// ULog file from an imu-raw-log build.
    log: PathBuf,
    /// Write every replayed sample to this CSV (for PlotJuggler), with the
    /// variants' angles when --compare is given.
    #[arg(long)]
    csv: Option<PathBuf>,
    /// Also run the alternative filters (Madgwick, Mahony, VQF, each with and
    /// without accel gating) and compare them with the firmware.
    #[arg(long)]
    compare: bool,
    /// Accel gate threshold for the gated variants, g from 1 g.
    #[arg(long, default_value_t = elle_replay::variants::DEFAULT_GATE_G)]
    gate_g: f32,
}

fn main() -> Result<()> {
    let args = Args::parse();
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
        println!(
            "\n  variant vs firmware (deg)      pitch mean/rms/max       roll mean/rms/max        yaw mean/rms/max    gated"
        );
        for c in elle_replay::compare(&replay) {
            let f = |k: usize| {
                format!(
                    "{:+7.3} {:6.3} {:6.3}",
                    c.mean_deg[k], c.rms_deg[k], c.max_deg[k]
                )
            };
            println!(
                "  {:<22} {}  {}  {}  {:5.1}%",
                c.name,
                f(0),
                f(1),
                f(2),
                100.0 * c.gated_share
            );
        }
        println!(
            "  (differences, not errors: which one is right needs a reference, e.g. gyro-only through turns)"
        );
    }

    if let Some(path) = &args.csv {
        let file =
            std::fs::File::create(path).with_context(|| format!("creating {}", path.display()))?;
        elle_replay::write_csv(&replay, std::io::BufWriter::new(file))?;
        println!("  wrote {}", path.display());
    }

    if c.roundtrip_errors > 0 {
        println!("FAIL: the logged integers do not reproduce the driver's floats (scale mismatch)");
        std::process::exit(1);
    }
    if !f.ok() {
        println!(
            "FAIL: the replay does not reproduce the firmware (worst {:.4}°); results from it cannot be trusted",
            f.worst_deg
        );
        std::process::exit(1);
    }
    println!("OK: the replay reproduces the firmware's attitude exactly");
    Ok(())
}
