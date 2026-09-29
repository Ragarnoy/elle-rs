//! A wind-independent attitude reference, and scoring against it.
//!
//! In straight, level, unaccelerated flight the accelerometer does measure
//! gravity alone, so its direction gives pitch and roll directly (not the
//! firmware's attitude, which after a turn is still recovering from the error
//! the turn put in it). From such a stretch the reference integrates the gyro
//! alone (debiased, as fused) for up
//! to [`MAX_PROPAGATION_S`]: over that span gyro drift is small (0.05°/s of
//! residual bias is 1° in 20 s), so through a turn or a pull-up the reference
//! stays close to the truth while an accel-corrected filter is pulled towards
//! "level". When a propagation ends at the next straight-and-level stretch,
//! the gap between propagated and anchored attitude is the drift it gathered;
//! it is removed linearly across the stretch (exact for a constant residual
//! bias). Filters are then scored against it, in turns only.
//!
//! On simulated flights the same scoring runs against the truth, which is how
//! the reference itself is checked.

use nalgebra::{UnitQuaternion, Vector3};

use crate::Replay;

/// Straight and level for at least this long anchors the reference, s.
pub const ANCHOR_S: f32 = 2.0;
/// ... and for at least this long after, s: the low-passed signals lag the
/// start of a manoeuvre, so an anchor right before one would take a tilt the
/// manoeuvre already disturbs.
pub const ANCHOR_GUARD_S: f32 = 0.5;
/// Longest gyro-only propagation from an anchor, s.
pub const MAX_PROPAGATION_S: f32 = 20.0;
/// "Straight and level": gyro under this, rad/s (3°/s) ...
const STILL_RATE: f32 = 0.052;
/// ... accel magnitude within this of 1 g, m/s² (0.05 g) ...
const STILL_ACCEL: f32 = 0.05 * 9.806_65;
/// ... and pitch and roll under these, radians (10° and 5°).
const LEVEL_PITCH: f32 = 0.175;
const LEVEL_ROLL: f32 = 0.087;
/// Scored samples: the reference is banked more than this, radians (10°) ...
pub const TURN_BANK: f32 = 0.175;
/// ... or, scoring by rate, the aircraft turns faster than this, rad/s (6°/s).
pub const TURN_RATE: f32 = 0.1;

/// Which samples count as "in a turn".
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Select {
    /// The reference is banked more than [`TURN_BANK`] (flight: coordinated
    /// turns bank). Direction: the bank's sign.
    Bank,
    /// The gyro turns faster than [`TURN_RATE`] (ground tests: a vehicle
    /// turns without banking, so the truth there is level). Direction: the
    /// turn's sign about the filter's up axis.
    Rate,
}
/// The straight-and-level test reads gyro and accel through a low-pass at this
/// corner, Hz, so motor vibration does not hide a steady stretch.
const STILL_LPF_HZ: f32 = 2.0;

const DT: f32 = elle_config::AHRS_SAMPLE_PERIOD_US as f32 / 1_000_000.0;

/// Filter-frame roll and pitch of the gravity direction in a (low-passed)
/// accel: level reads (0, 0, +g), so roll = atan2(y, z), pitch = atan2(−x, …).
fn tilt_of(acc: &Vector3<f32>) -> (f32, f32) {
    (acc.y.atan2(acc.z), (-acc.x).atan2(acc.y.hypot(acc.z)))
}

/// Filter-frame quaternion → firmware-convention (pitch, roll, yaw).
fn angles_of(q: &UnitQuaternion<f32>) -> [f32; 3] {
    let (roll, pitch, yaw) = q.euler_angles();
    [pitch, -roll, yaw]
}

/// The reference, aligned with `Replay::samples` (`None` where it is not
/// established: before the first anchor, after a gap, too long after one).
#[must_use]
pub fn build(replay: &Replay) -> Vec<Option<[f32; 3]>> {
    let anchor_n = (ANCHOR_S / DT) as usize;
    let guard_n = (ANCHOR_GUARD_S / DT) as usize;
    let max_n = (MAX_PROPAGATION_S / DT) as u32;
    let alpha = {
        let rc = 1.0 / (2.0 * core::f32::consts::PI * STILL_LPF_HZ);
        DT / (rc + DT)
    };

    // Pass 1: per sample, straight and level (on low-passed gyro and accel),
    // and the tilt of the low-passed accel.
    let mut level = Vec::with_capacity(replay.samples.len());
    let mut tilt = Vec::with_capacity(replay.samples.len());
    let mut lp: Option<(Vector3<f32>, Vector3<f32>)> = None;
    for (s, inp) in replay.samples.iter().zip(&replay.inputs) {
        if s.att.is_none() {
            lp = None;
            level.push(false);
            tilt.push((0.0, 0.0));
            continue;
        }
        let (g, acc) = match lp {
            Some((g, acc)) => (g + (inp.gyro - g) * alpha, acc + (inp.accel - acc) * alpha),
            None => (inp.gyro, inp.accel),
        };
        lp = Some((g, acc));
        let (roll_f, pitch_f) = tilt_of(&acc);
        level.push(
            g.norm() < STILL_RATE
                && (acc.norm() - 9.806_65).abs() < STILL_ACCEL
                && pitch_f.abs() < LEVEL_PITCH
                && roll_f.abs() < LEVEL_ROLL,
        );
        tilt.push((roll_f, pitch_f));
    }
    // Level samples before each index (prefix sums), to test windows quickly.
    let mut before = vec![0u32; level.len() + 1];
    for (i, l) in level.iter().enumerate() {
        before[i + 1] = before[i] + u32::from(*l);
    }
    let level_over = |from: usize, to: usize| -> bool {
        to <= level.len() && (before[to] - before[from]) as usize == to - from
    };

    // Pass 2: anchor where it was level for ANCHOR_S before and stays level
    // ANCHOR_GUARD_S after (the low-passed signals lag the start of a
    // manoeuvre), propagate the gyro in between.
    let mut out: Vec<Option<[f32; 3]>> = Vec::with_capacity(replay.samples.len());
    let mut q: Option<UnitQuaternion<f32>> = None;
    let mut since_anchor = 0u32;
    // Where the current propagation started in `out` (the anchor sample).
    let mut run_start: Option<usize> = None;
    for (i, (s, inp)) in replay.samples.iter().zip(&replay.inputs).enumerate() {
        let Some(a) = s.att else {
            // Not synced: no trusted gyro run.
            q = None;
            run_start = None;
            out.push(None);
            continue;
        };
        let anchor = i + 1 >= anchor_n && level_over(i + 1 - anchor_n, i + 1 + guard_n);
        if anchor {
            let (roll_f, pitch_f) = tilt[i];
            // Tilt from gravity; heading (not scored) from the firmware.
            let anchored = UnitQuaternion::from_euler_angles(roll_f, pitch_f, a.yaw);
            // A propagation arriving here: spread its end error back over it.
            if let (Some(start), Some(r)) = (run_start, q.as_ref())
                && since_anchor > 0
            {
                let prop = *r * UnitQuaternion::from_scaled_axis(inp.gyro * DT);
                let (end, fix) = (angles_of(&prop), angles_of(&anchored));
                let d = [fix[0] - end[0], fix[1] - end[1]];
                let len = (out.len() - start) as f32;
                for (k, v) in out[start..].iter_mut().enumerate() {
                    if let Some(v) = v {
                        let w = k as f32 / len;
                        v[0] += d[0] * w;
                        v[1] += d[1] * w;
                    }
                }
            }
            q = Some(anchored);
            since_anchor = 0;
            run_start = Some(out.len());
        } else if let Some(r) = q.as_mut() {
            *r *= UnitQuaternion::from_scaled_axis(inp.gyro * DT);
            since_anchor += 1;
            if since_anchor > max_n {
                q = None;
                run_start = None;
            }
        }
        out.push(q.as_ref().map(angles_of));
    }
    out
}

/// Errors against a reference over the samples where it is banked more than
/// [`TURN_BANK`], by turn direction.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Score {
    pub name: String,
    /// Samples scored in right and left turns.
    pub samples: [u64; 2],
    /// Mean and RMS roll error (filter − reference), degrees, right then left turns.
    pub roll_mean_deg: [f32; 2],
    pub roll_rms_deg: [f32; 2],
    /// RMS pitch error, degrees, right then left turns.
    pub pitch_rms_deg: [f32; 2],
}

impl Score {
    /// RMS roll error over both directions, degrees.
    #[must_use]
    pub fn roll_rms_both(&self) -> f32 {
        let n = (self.samples[0] + self.samples[1]).max(1) as f32;
        ((self.roll_rms_deg[0].powi(2) * self.samples[0] as f32
            + self.roll_rms_deg[1].powi(2) * self.samples[1] as f32)
            / n)
            .sqrt()
    }
}

/// Score `track` (per-sample angles, `None` to skip) against `reference`,
/// in turns by bank.
pub fn score(
    name: &str,
    track: impl Iterator<Item = Option<[f32; 3]>>,
    reference: &[Option<[f32; 3]>],
) -> Score {
    score_where(name, track, reference, |i| {
        reference[i].and_then(|r| (r[1].abs() >= TURN_BANK).then_some(r[1] < 0.0))
    })
}

/// Score `track` against `reference` over the samples `side` picks: `Some(false)`
/// for right turns, `Some(true)` for left, `None` to skip.
pub fn score_where(
    name: &str,
    track: impl Iterator<Item = Option<[f32; 3]>>,
    reference: &[Option<[f32; 3]>],
    side: impl Fn(usize) -> Option<bool>,
) -> Score {
    let mut n = [0u64; 2];
    let mut roll_sum = [0f64; 2];
    let mut roll_sq = [0f64; 2];
    let mut pitch_sq = [0f64; 2];
    for (i, (t, r)) in track.zip(reference).enumerate() {
        let (Some(t), Some(r), Some(left)) = (t, r, side(i)) else {
            continue;
        };
        let side = usize::from(left);
        let dr = f64::from((t[1] - r[1]).to_degrees());
        let dp = f64::from((t[0] - r[0]).to_degrees());
        n[side] += 1;
        roll_sum[side] += dr;
        roll_sq[side] += dr * dr;
        pitch_sq[side] += dp * dp;
    }
    let per = |x: [f64; 2], f: fn(f64) -> f64| -> [f32; 2] {
        [0, 1].map(|k| f(x[k] / n[k].max(1) as f64) as f32)
    };
    Score {
        name: name.to_string(),
        samples: n,
        roll_mean_deg: per(roll_sum, |x| x),
        roll_rms_deg: per(roll_sq, f64::sqrt),
        pitch_rms_deg: per(pitch_sq, f64::sqrt),
    }
}

/// Score the firmware and every variant against `reference`.
#[must_use]
pub fn score_all(replay: &Replay, reference: &[Option<[f32; 3]>]) -> Vec<Score> {
    score_all_by(replay, reference, Select::Bank)
}

/// As [`score_all`], choosing how turns are recognised.
#[must_use]
pub fn score_all_by(replay: &Replay, reference: &[Option<[f32; 3]>], select: Select) -> Vec<Score> {
    let side = |i: usize| -> Option<bool> {
        match select {
            Select::Bank => {
                reference[i].and_then(|r| (r[1].abs() >= TURN_BANK).then_some(r[1] < 0.0))
            }
            Select::Rate => {
                // Turn rate about the filter's up axis (+z): positive turns left.
                let r = replay.inputs[i].gyro.z;
                (r.abs() >= TURN_RATE).then_some(r > 0.0)
            }
        }
    };
    let fw = replay
        .samples
        .iter()
        .map(|s| s.att.map(|a| [a.pitch, a.roll, a.yaw]));
    let mut out = vec![score_where("firmware", fw, reference, side)];
    for t in &replay.tracks {
        out.push(score_where(
            &t.name,
            t.angles.iter().copied(),
            reference,
            side,
        ));
    }
    out
}
