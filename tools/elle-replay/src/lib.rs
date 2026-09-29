//! Replay a raw IMU log (`imu-raw-log` build) through the firmware's attitude
//! pipeline on the host.
//!
//! The replay runs `elle_control::imu_raw::Replayer`, the same fusion code the
//! aircraft ran. Its first job is to prove that: every `attitude_data` the
//! firmware logged while the replay is synced must be reproduced exactly
//! ([`Faithfulness`]). Any mismatch means the harness does not reproduce the
//! firmware, and nothing it says about other filters can be trusted.

pub mod ulog;

use anyhow::{Result, bail};
use elle_control::attitude::Attitude;
use elle_control::imu_raw::{BATCH_SAMPLES, Ctx, Mag, Replayer, SAMPLE_BYTES, unpack};

use crate::ulog::ULog;

/// One replayed sample.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sample {
    pub index: u32,
    /// Estimated sample time, µs since boot: the batch's first-sample time
    /// plus 1 ms per sample.
    pub t_us: u64,
    /// The attitude while synced (firmware-exact angles); `None` before the
    /// first context and after a gap until the next one.
    pub att: Option<Attitude>,
}

/// What the log contained and what the replay covered.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Coverage {
    /// Samples recorded.
    pub samples: u64,
    /// Samples replayed with firmware-exact angles (after a context, before a gap).
    pub synced: u64,
    /// Breaks in the sample index, and the samples lost in them.
    pub gaps: u64,
    pub lost: u64,
    pub contexts: u64,
    pub mag_changes: u64,
    /// Largest `roundtrip_errors` reported: nonzero means the logged integers
    /// do not reproduce the driver's floats.
    pub roundtrip_errors: u32,
    /// ULog dropout records in the file.
    pub dropouts: usize,
}

/// Every recorded sample (synced or not) plus coverage.
#[derive(Clone, Debug, Default)]
pub struct Replay {
    pub samples: Vec<Sample>,
    pub coverage: Coverage,
}

/// How the replay compares with the attitude the firmware logged.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Faithfulness {
    /// Logged attitudes the replay reproduced bit for bit.
    pub exact: u64,
    /// Logged attitudes whose source sample may be missing or unsynced in the
    /// replay (before the first context, around a gap): not checked.
    pub unchecked: u64,
    /// Logged attitudes the replay had every candidate sample for, synced,
    /// and none matched.
    pub mismatched: u64,
    /// For mismatches: the largest angle difference to the nearest-in-time
    /// replayed sample, degrees.
    pub worst_deg: f32,
}

impl Faithfulness {
    #[must_use]
    pub const fn ok(&self) -> bool {
        self.mismatched == 0 && self.exact > 0
    }
}

enum Event {
    Ctx(Ctx),
    Mag(Mag),
    Sample {
        t_us: u64,
        gyro: [i32; 3],
        accel: [i32; 3],
    },
}

/// An event keyed for ordering: (sample index, rank within that index, event).
type Keyed = (u32, u8, Event);

fn events(log: &ULog) -> Result<(Vec<Keyed>, Coverage)> {
    let Some(raw) = log.get("imu_raw") else {
        bail!("no imu_raw records: not an imu-raw-log build, or recording never started");
    };
    let mut cov = Coverage {
        dropouts: log.dropouts.len(),
        ..Coverage::default()
    };
    let mut ev = Vec::new();
    for r in &raw.records {
        let first = raw.u32(r, "first_index");
        let t0 = raw.u64(r, "timestamp");
        let count = usize::from(raw.u8(r, "count")).min(BATCH_SAMPLES);
        let data = raw.blob(r, "data");
        for k in 0..count {
            let (gyro, accel) = unpack(&data[k * SAMPLE_BYTES..]);
            ev.push((
                first.wrapping_add(k as u32),
                2,
                Event::Sample {
                    t_us: t0 + 1000 * k as u64,
                    gyro,
                    accel,
                },
            ));
        }
        cov.samples += count as u64;
    }
    if let Some(c) = log.get("imu_raw_ctx") {
        for r in &c.records {
            let ctx = Ctx {
                index: c.u32(r, "index"),
                t_us: c.u64(r, "timestamp"),
                quat: c.f32s(r, "quat"),
                gyro_bias: c.f32s(r, "gyro_bias"),
                mount: c.f32s(r, "mount"),
                roundtrip_errors: c.u32(r, "roundtrip_errors"),
            };
            cov.roundtrip_errors = cov.roundtrip_errors.max(ctx.roundtrip_errors);
            cov.contexts += 1;
            ev.push((ctx.index, 0, Event::Ctx(ctx)));
        }
    }
    if let Some(m) = log.get("imu_raw_mag") {
        for r in &m.records {
            let mag = Mag {
                index: m.u32(r, "index"),
                t_us: m.u64(r, "timestamp"),
                mag: (m.u8(r, "valid") != 0).then(|| m.f32s(r, "mag")),
            };
            cov.mag_changes += 1;
            ev.push((mag.index, 1, Event::Mag(mag)));
        }
    }
    // Sample order; a context or mag change before the sample it applies to.
    ev.sort_by_key(|(i, k, _)| (*i, *k));
    Ok((ev, cov))
}

/// Replay every raw sample in the log.
pub fn replay(log: &ULog) -> Result<Replay> {
    let (ev, mut coverage) = events(log)?;
    let mut rp = Replayer::new();
    let mut samples = Vec::new();
    let mut last: Option<u32> = None;
    for (index, _, e) in ev {
        match e {
            Event::Ctx(c) => rp.ctx(&c),
            Event::Mag(m) => rp.mag(&m),
            Event::Sample { t_us, gyro, accel } => {
                if let Some(prev) = last
                    && index != prev.wrapping_add(1)
                {
                    coverage.gaps += 1;
                    coverage.lost += u64::from(index.wrapping_sub(prev).wrapping_sub(1));
                }
                last = Some(index);
                let att = rp.sample(index, gyro, accel);
                coverage.synced += u64::from(att.is_some());
                samples.push(Sample { index, t_us, att });
            }
        }
    }
    Ok(Replay { samples, coverage })
}

/// How far before its log time a logged attitude can have been sampled, µs:
/// the loop only logs an attitude younger than `IMU_MAX_AGE_MS`.
const MATCH_WINDOW_US: u64 = elle_config::IMU_MAX_AGE_MS * 1000;
/// Replayed sample times are estimates (batch time + 1 ms per sample).
const MATCH_SLACK_US: u64 = 5_000;

/// Compare the replay with every `attitude_data` the firmware logged.
///
/// `attitude_data` is stamped when logged, not when sampled, so each one is
/// matched by value: an exact (pitch, roll, yaw) among the samples of the
/// preceding [`MATCH_WINDOW_US`]. It only counts as mismatched when the replay
/// holds that whole window synced, with no gap in it or just after it;
/// otherwise its source sample may be one the replay could not reproduce, and
/// it is unchecked.
#[must_use]
pub fn faithfulness(log: &ULog, replay: &Replay) -> Faithfulness {
    let mut out = Faithfulness::default();
    let Some(att) = log.get("attitude_data") else {
        return out;
    };
    let s = &replay.samples;
    for r in &att.records {
        let t = att.u64(r, "timestamp");
        let want = (att.f32(r, "pitch"), att.f32(r, "roll"), att.f32(r, "yaw"));
        let from = t.saturating_sub(MATCH_WINDOW_US + MATCH_SLACK_US);
        let lo = s.partition_point(|x| x.t_us < from);
        let hi = s.partition_point(|x| x.t_us <= t + MATCH_SLACK_US);
        let window = &s[lo..hi];
        let angles = |x: &Sample| x.att.map(|a| (a.pitch, a.roll, a.yaw));
        if window.iter().any(|x| angles(x) == Some(want)) {
            out.exact += 1;
            continue;
        }
        // Complete: the window starts early enough, every sample in it is
        // synced, and the samples run on without a break through the one just
        // after it (a gap at the window's end could hide the source sample).
        let complete = window
            .first()
            .is_some_and(|f| f.t_us <= from + MATCH_SLACK_US)
            && window.iter().all(|x| x.att.is_some())
            && s.get(lo..=hi).is_some_and(|w| {
                w.windows(2)
                    .all(|p| p[1].index == p[0].index.wrapping_add(1))
            });
        if !complete {
            out.unchecked += 1;
            continue;
        }
        out.mismatched += 1;
        let nearest = window
            .iter()
            .filter_map(|x| x.att.map(|a| (x.t_us.abs_diff(t), a)))
            .min_by_key(|(d, _)| *d)
            .map(|(_, a)| a)
            .unwrap();
        let d = (nearest.pitch - want.0)
            .abs()
            .max((nearest.roll - want.1).abs())
            .max((nearest.yaw - want.2).abs());
        out.worst_deg = out.worst_deg.max(d.to_degrees());
    }
    out
}

/// The replay as CSV: sample index, time (s), angles (deg), rates (deg/s).
pub fn write_csv(replay: &Replay, mut w: impl std::io::Write) -> Result<()> {
    writeln!(
        w,
        "index,t_s,pitch_deg,roll_deg,yaw_deg,pitch_rate_dps,roll_rate_dps,yaw_rate_dps"
    )?;
    for s in &replay.samples {
        let Some(a) = &s.att else { continue };
        writeln!(
            w,
            "{},{:.6},{},{},{},{},{},{}",
            s.index,
            s.t_us as f64 * 1e-6,
            a.pitch.to_degrees(),
            a.roll.to_degrees(),
            a.yaw.to_degrees(),
            a.pitch_rate.to_degrees(),
            a.roll_rate.to_degrees(),
            a.yaw_rate.to_degrees()
        )?;
    }
    Ok(())
}
