//! Rebuild a flight-law input stream from a flight ULog, for proposal 0007's
//! equivalence oracle (`crates/elle-control/tests/flight_law_equivalence/`).
//!
//! ```sh
//! cargo run -p elle-replay --target x86_64-unknown-linux-gnu --example law_stream -- \
//!     LOG.ulg [OUT.stream --from S --secs N]
//! ```
//!
//! Without `OUT` it lists the log's armed intervals and modes, to pick a window.
//!
//! The log is not a capture of the controller's inputs: `commands` is the
//! normalized pilot command at 100 Hz, held for the 5 ms ticks in between, and
//! the raw RC channels are not logged, so the stream drives the normalized
//! path only. Arming follows `system_status.armed` through explicit arm and
//! disarm calls. `attitude_data` gives each tick's new sample, if one arrived
//! since the previous tick. The inputs are realistic, not what flew; the
//! oracle only needs both controllers to get the same ones.

use anyhow::{Context, Result, bail};
use elle_replay::ulog::ULog;
use std::fmt::Write as _;

const TICK_US: u64 = 5_000;

struct Cmd {
    t: u64,
    throttle: f32,
    pitch: f32,
    roll: f32,
    yaw: f32,
    mode: u8,
}

struct Att {
    t: u64,
    v: [f32; 6],
}

fn main() -> Result<()> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let Some(path) = args.first() else {
        bail!("usage: law_stream LOG.ulg [OUT.stream --from S --secs N]");
    };
    let flag = |name: &str| -> Option<f64> {
        let i = args.iter().position(|a| a == name)?;
        args.get(i + 1)?.parse().ok()
    };
    let log = ULog::parse(&std::fs::read(path).with_context(|| path.clone())?)?;

    let s = log.get("commands").context("no commands")?;
    let cmds: Vec<Cmd> = s
        .records
        .iter()
        .map(|r| Cmd {
            t: s.u64(r, "timestamp"),
            throttle: s.f32(r, "throttle"),
            pitch: s.f32(r, "pitch"),
            roll: s.f32(r, "roll"),
            yaw: s.f32(r, "yaw"),
            mode: s.u8(r, "attitude_mode"),
        })
        .collect();
    let s = log.get("attitude_data").context("no attitude_data")?;
    let mut atts: Vec<Att> = s
        .records
        .iter()
        .map(|r| Att {
            t: s.u64(r, "timestamp"),
            v: [
                "pitch",
                "roll",
                "yaw",
                "pitch_rate",
                "roll_rate",
                "yaw_rate",
            ]
            .map(|f| s.f32(r, f)),
        })
        .collect();
    atts.sort_by_key(|a| a.t);
    let s = log.get("system_status").context("no system_status")?;
    let armed: Vec<(u64, bool)> = s
        .records
        .iter()
        .map(|r| (s.u64(r, "timestamp"), s.u8(r, "armed") != 0))
        .collect();
    let gains = log.get("pid_gains").and_then(|s| {
        let r = s.records.first()?;
        Some(
            [
                "kp_pitch", "ki_pitch", "kd_pitch", "kp_roll", "ki_roll", "kd_roll", "i_limit",
                "scale",
            ]
            .map(|f| s.f32(r, f)),
        )
    });

    let t0 = cmds.first().context("empty commands")?.t;
    let Some(out) = args.get(1).filter(|a| !a.starts_with("--")) else {
        // Survey: armed intervals and time per mode.
        let mut last = None;
        for &(t, a) in &armed {
            if last != Some(a) {
                println!(
                    "{:8.1} s  {}",
                    (t - t0) as f64 / 1e6,
                    if a { "armed" } else { "disarmed" }
                );
                last = Some(a);
            }
        }
        let mut per_mode = [0usize; 3];
        for c in &cmds {
            per_mode[usize::from(c.mode.min(2))] += 1;
        }
        println!(
            "commands: Manual {:.0} s, Stabilized {:.0} s, AltitudeHold {:.0} s",
            per_mode[0] as f64 / 100.0,
            per_mode[1] as f64 / 100.0,
            per_mode[2] as f64 / 100.0
        );
        return Ok(());
    };

    let from = t0 + (flag("--from").unwrap_or(0.0) * 1e6) as u64;
    let to = from + (flag("--secs").unwrap_or(20.0) * 1e6) as u64;
    // Stream times are µs from the window's start (plus one tick, so that
    // samples older than the first tick stay at positive times).
    let base = from - 2 * TICK_US;
    let rel = |t: u64| t.saturating_sub(base);

    let mut text = format!(
        "# Proposal 0007 flight stream: {} from {:.1} s for {:.1} s, written by\n\
         # tools/elle-replay/examples/law_stream.rs.\n",
        std::path::Path::new(path)
            .file_name()
            .unwrap()
            .to_string_lossy(),
        (from - t0) as f64 / 1e6,
        (to - from) as f64 / 1e6
    );
    text.push_str("enable_supervisor\nexplicit 1\n");
    if let Some(g) = gains {
        text.push_str("gains");
        for v in g {
            write!(text, " {v:?}").unwrap();
        }
        text.push('\n');
    }

    let (mut ci, mut ai, mut si) = (0, 0, 0);
    // Start from the arming state at the window's start, not its history.
    let mut was_armed = false;
    while si < armed.len() && armed[si].0 <= from {
        was_armed = armed[si].1;
        si += 1;
    }
    if was_armed {
        text.push_str("arm\n");
    }
    let mut t = from;
    let mut prev = from - TICK_US;
    while t < to {
        while si < armed.len() && armed[si].0 <= t {
            let a = armed[si].1;
            if a != was_armed {
                text.push_str(if a { "arm\n" } else { "disarm\n" });
                was_armed = a;
            }
            si += 1;
        }
        while ci + 1 < cmds.len() && cmds[ci + 1].t <= t {
            ci += 1;
        }
        let c = &cmds[ci];
        if c.t > t {
            t += TICK_US;
            continue;
        }
        // The newest sample read since the previous tick.
        let mut att = None;
        while ai < atts.len() && atts[ai].t <= t {
            if atts[ai].t > prev {
                att = Some(&atts[ai]);
            }
            ai += 1;
        }
        let mode = ['M', 'S', 'A'][usize::from(c.mode.min(2))];
        write!(
            text,
            "tick {} {} norm {:?} {:?} {:?} {:?} {mode}",
            rel(t),
            rel(c.t),
            c.throttle,
            c.pitch,
            c.roll,
            c.yaw
        )
        .unwrap();
        if let Some(a) = att {
            write!(text, " att {}", rel(a.t)).unwrap();
            for v in a.v {
                write!(text, " {v:?}").unwrap();
            }
        }
        text.push_str("\nfailsafe\nsupervise 1\n");
        prev = t;
        t += TICK_US;
    }
    std::fs::write(out, text)?;
    println!("wrote {out}");
    Ok(())
}
