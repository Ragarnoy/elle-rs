//! Proposal 0009: does seeding the AHRS from accel + mag remove the boot
//! transient? `cargo run -p elle-replay --example seed -- LOG.ulg`
//!
//! From the first synced sample of a still raw-IMU log, runs the firmware's
//! filter three ways on the same inputs: as logged (the exact replay),
//! restarted at identity (what a boot does) and restarted at
//! `elle_control::attitude::seed`. The restarted runs are the replay's own
//! firmware variant (`variants::default_specs()[0]`, which `--compare` checks
//! against the exact replay), so they follow the build's filter. Prints the
//! angles every 2 s and each run's worst tilt error against the attitude the
//! log settles at.

use anyhow::{Context, Result, bail};
use elle_control::attitude::{angles, seed};
use elle_replay::ulog::ULog;
use elle_replay::variants::{Running, default_specs};
use nalgebra::UnitQuaternion;

/// The firmware's filter as a replay variant, started at `q`.
fn filter(q: UnitQuaternion<f32>) -> Running {
    let mut r = Running::new(default_specs().swap_remove(0));
    r.seed(q);
    r
}

fn main() -> Result<()> {
    let path = std::env::args().nth(1).context("usage: seed LOG.ulg")?;
    let data = std::fs::read(&path).with_context(|| format!("reading {path}"))?;
    let log = ULog::parse(&data)?;
    let rp = elle_replay::replay(&log)?;

    // SEED_AT_S: start all runs this long after the first synced sample
    // (e.g. once the stored mag offsets have reached the filter).
    let at_us = std::env::var("SEED_AT_S")
        .ok()
        .and_then(|v| v.parse::<f64>().ok())
        .map_or(0, |s| (s * 1e6) as u64);
    let first = rp
        .samples
        .iter()
        .position(|s| s.att.is_some())
        .context("no synced sample")?;
    let k0 = rp.samples[first..]
        .iter()
        .position(|s| s.att.is_some() && s.t_us >= rp.samples[first].t_us + at_us)
        .map(|k| first + k)
        .context("no synced sample at SEED_AT_S")?;
    let inp0 = &rp.inputs[k0];
    let mag0 = inp0.mag.context("no mag at the first synced sample")?;
    let q_seed = seed(&inp0.accel, &mag0).context("seed failed")?;
    let mut runs = [filter(UnitQuaternion::identity()), filter(q_seed)];

    let t0 = rp.samples[k0].t_us;
    let n = rp.samples.len();
    // Settled attitude: the logged mean over the last 10 s.
    let tail: Vec<_> = rp.samples[k0..]
        .iter()
        .filter(|s| s.t_us + 10_000_000 >= rp.samples[n - 1].t_us)
        .filter_map(|s| s.att)
        .collect();
    if tail.is_empty() {
        bail!("no synced attitude in the last 10 s: nothing to settle against");
    }
    let mean = |f: fn(&elle_control::attitude::Attitude) -> f32| {
        tail.iter().map(f).sum::<f32>() / tail.len() as f32
    };
    // Yaw as a circular mean: a heading dithering across ±180° must not
    // average to 0.
    let sy = tail.iter().fold((0.0f32, 0.0f32), |(s, c), a| {
        (s + a.yaw.sin(), c + a.yaw.cos())
    });
    let (sp, sr, sy) = (mean(|a| a.pitch), mean(|a| a.roll), sy.0.atan2(sy.1));
    let d = f32::to_degrees;
    println!(
        "{path}: from sample {} ({} samples); settled pitch {:+.1} roll {:+.1} yaw {:+.1}",
        rp.samples[k0].index,
        n - k0,
        d(sp),
        d(sr),
        d(sy)
    );
    let (p, r, y) = angles(&q_seed);
    println!(
        "seed: pitch {:+.1} roll {:+.1} yaw {:+.1}",
        d(p),
        d(r),
        d(y)
    );
    println!("   t |  logged p/r/y          |  identity p/r/y        |  seeded p/r/y");

    let tilt = |p: f32, r: f32| d((p - sp).hypot(r - sr));
    let mut worst = [0f32; 3];
    let mut last_bad = [0u64; 3];
    let mut next_print = 0u64;
    let mut prev_mag = None;
    for k in k0..n {
        let s = &rp.samples[k];
        let inp = &rp.inputs[k];
        let mut out = [(0.0, 0.0, 0.0); 3];
        if let Some(a) = s.att {
            out[0] = (a.pitch, a.roll, a.yaw);
        }
        let mag_new = inp.mag != prev_mag;
        prev_mag = inp.mag;
        for (i, f) in runs.iter_mut().enumerate() {
            let [p, r, y] = f.step(inp.gyro, inp.accel, inp.mag.as_ref(), mag_new, &inp.aid);
            out[i + 1] = (p, r, y);
        }
        let t = s.t_us - t0;
        if std::env::var_os("SEED_TRACE").is_some()
            && t % 5_000_000 < 1_000
            && let Some(m) = inp.mag
            && let Some(q) = seed(&inp.accel, &m)
        {
            let (p, r, y) = angles(&q);
            println!(
                "  trace {:5.1} s: seed p{:+5.1} r{:+5.1} y{:+7.1}  mag ({:+8.0} {:+8.0} {:+8.0})",
                t as f64 / 1e6,
                d(p),
                d(r),
                d(y),
                m.x,
                m.y,
                m.z
            );
        }
        for i in 0..3 {
            if i == 0 && s.att.is_none() {
                continue;
            }
            let e = tilt(out[i].0, out[i].1);
            worst[i] = worst[i].max(e);
            if e > 1.0 {
                last_bad[i] = t;
            }
        }
        if t >= next_print {
            next_print += 2_000_000;
            let f = |o: (f32, f32, f32)| format!("{:+6.1} {:+6.1} {:+7.1}", d(o.0), d(o.1), d(o.2));
            println!(
                "{:4.0} | {} | {} | {}",
                t as f64 / 1e6,
                f(out[0]),
                f(out[1]),
                f(out[2])
            );
        }
    }
    for (name, i) in [("logged", 0), ("identity", 1), ("seeded", 2)] {
        println!(
            "{name:9} worst tilt error {:5.1} deg, last > 1 deg at {:5.1} s",
            worst[i],
            last_bad[i] as f64 / 1e6
        );
    }
    Ok(())
}
