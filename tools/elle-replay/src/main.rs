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
    /// Write every replayed sample to this CSV (for PlotJuggler).
    #[arg(long)]
    csv: Option<PathBuf>,
}

fn main() -> Result<()> {
    let args = Args::parse();
    let data =
        std::fs::read(&args.log).with_context(|| format!("reading {}", args.log.display()))?;
    let log = ULog::parse(&data)?;
    let replay = elle_replay::replay(&log)?;
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
        "  attitude_data: {} exact, {} mismatched, {} not checked (no synced replay nearby)",
        f.exact, f.mismatched, f.unchecked
    );

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
