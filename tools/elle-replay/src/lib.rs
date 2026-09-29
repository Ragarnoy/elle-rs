//! Replay a raw IMU log (`imu-raw-log` build) through the firmware's attitude
//! pipeline on the host.
//!
//! The replay runs `elle_control::imu_raw::Replayer`, the same fusion code the
//! aircraft ran. Its first job is to prove that: every `attitude_data` the
//! firmware logged while the replay is synced must be reproduced exactly
//! ([`Faithfulness`]). Any mismatch means the harness does not reproduce the
//! firmware, and nothing it says about other filters can be trusted.

pub mod sim;
pub mod ulog;
pub mod variants;

use anyhow::{Result, bail};
use elle_control::attitude::Attitude;
use elle_control::imu_raw::{
    ACCEL_SCALE, BATCH_SAMPLES, Ctx, GYRO_SCALE, Mag, Replayer, SAMPLE_BYTES, decode, unpack,
};
use nalgebra::{Quaternion, UnitQuaternion, Vector3};

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

/// One alternative filter's output, aligned with [`Replay::samples`].
#[derive(Clone, Debug, Default)]
pub struct Track {
    pub name: String,
    /// (pitch, roll, yaw), radians, firmware convention; `None` where the
    /// replay is not synced (the variant restarts from the firmware's state at
    /// each resynchronisation).
    pub angles: Vec<Option<[f32; 3]>>,
    /// Samples it fused without the accelerometer.
    pub gated: u64,
}

/// Every recorded sample (synced or not) plus coverage.
#[derive(Clone, Debug, Default)]
pub struct Replay {
    pub samples: Vec<Sample>,
    pub coverage: Coverage,
    pub tracks: Vec<Track>,
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
    replay_with(log, &[])
}

fn quat(v: [f32; 4]) -> UnitQuaternion<f32> {
    UnitQuaternion::new_unchecked(Quaternion::new(v[0], v[1], v[2], v[3]))
}

/// Replay every raw sample, and run each variant in `specs` alongside.
pub fn replay_with(log: &ULog, specs: &[variants::Spec]) -> Result<Replay> {
    let (ev, mut coverage) = events(log)?;
    let mut rp = Replayer::new();
    let mut runs: Vec<variants::Running> =
        specs.iter().cloned().map(variants::Running::new).collect();
    let mut tracks: Vec<Track> = specs
        .iter()
        .map(|s| Track {
            name: s.name.clone(),
            ..Track::default()
        })
        .collect();
    let mut samples = Vec::new();
    let mut last: Option<u32> = None;
    // The variants' inputs, kept the way the Replayer keeps its own.
    let mut bias = Vector3::zeros();
    let mut mount = UnitQuaternion::identity();
    let mut mag: Option<Vector3<f32>> = None;
    let mut mag_new = false;
    let mut seed: Option<UnitQuaternion<f32>> = None;
    let mut was_synced = false;
    for (index, _, e) in ev {
        match e {
            Event::Ctx(c) => {
                rp.ctx(&c);
                bias = Vector3::from(c.gyro_bias);
                mount = quat(c.mount);
                seed = Some(quat(c.quat));
            }
            Event::Mag(m) => {
                rp.mag(&m);
                mag = m.mag.map(Vector3::from);
                mag_new = true;
            }
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

                let synced = att.is_some();
                if synced
                    && !was_synced
                    && let Some(q) = seed
                {
                    runs.iter_mut().for_each(|r| r.seed(q));
                }
                was_synced = synced;
                let g = mount * (Vector3::from(gyro.map(|r| decode(r, GYRO_SCALE))) - bias);
                let a = mount * Vector3::from(accel.map(|r| decode(r, ACCEL_SCALE)));
                for (run, track) in runs.iter_mut().zip(&mut tracks) {
                    let out = run.step(g, a, mag.as_ref(), mag_new);
                    track.angles.push(synced.then_some(out));
                    track.gated = run.gated;
                }
                mag_new = false;
            }
        }
    }
    Ok(Replay {
        samples,
        coverage,
        tracks,
    })
}

/// A variant against the exact replay, over the samples both have.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Comparison {
    pub name: String,
    pub samples: u64,
    /// Mean, RMS and largest difference (variant − firmware), degrees, for
    /// pitch, roll and yaw.
    pub mean_deg: [f32; 3],
    pub rms_deg: [f32; 3],
    pub max_deg: [f32; 3],
    /// Share of samples fused without the accelerometer.
    pub gated_share: f32,
}

/// Compare every variant track with the firmware replay.
#[must_use]
pub fn compare(replay: &Replay) -> Vec<Comparison> {
    replay
        .tracks
        .iter()
        .map(|t| {
            let mut sum = [0f64; 3];
            let mut sq = [0f64; 3];
            let mut max = [0f32; 3];
            let mut n = 0u64;
            for (s, v) in replay.samples.iter().zip(&t.angles) {
                let (Some(a), Some(v)) = (s.att, v) else {
                    continue;
                };
                let fw = [a.pitch, a.roll, a.yaw];
                for k in 0..3 {
                    let mut d = v[k] - fw[k];
                    // Yaw wraps at ±180°.
                    if k == 2 {
                        d = (d + core::f32::consts::PI).rem_euclid(2.0 * core::f32::consts::PI)
                            - core::f32::consts::PI;
                    }
                    let d = d.to_degrees();
                    sum[k] += f64::from(d);
                    sq[k] += f64::from(d) * f64::from(d);
                    max[k] = max[k].max(d.abs());
                }
                n += 1;
            }
            let nf = n.max(1) as f64;
            Comparison {
                name: t.name.clone(),
                samples: n,
                mean_deg: sum.map(|x| (x / nf) as f32),
                rms_deg: sq.map(|x| (x / nf).sqrt() as f32),
                max_deg: max,
                gated_share: t.gated as f32 / replay.samples.len().max(1) as f32,
            }
        })
        .collect()
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

/// The replay as CSV: sample index, time (s), angles (deg), rates (deg/s),
/// then pitch/roll/yaw (deg) for each variant track.
pub fn write_csv(replay: &Replay, mut w: impl std::io::Write) -> Result<()> {
    write!(
        w,
        "index,t_s,pitch_deg,roll_deg,yaw_deg,pitch_rate_dps,roll_rate_dps,yaw_rate_dps"
    )?;
    for t in &replay.tracks {
        write!(w, ",{0}_pitch_deg,{0}_roll_deg,{0}_yaw_deg", t.name)?;
    }
    writeln!(w)?;
    for (i, s) in replay.samples.iter().enumerate() {
        let Some(a) = &s.att else { continue };
        write!(
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
        for t in &replay.tracks {
            match t.angles[i] {
                Some(v) => write!(
                    w,
                    ",{},{},{}",
                    v[0].to_degrees(),
                    v[1].to_degrees(),
                    v[2].to_degrees()
                )?,
                None => write!(w, ",,,")?,
            }
        }
        writeln!(w)?;
    }
    Ok(())
}
