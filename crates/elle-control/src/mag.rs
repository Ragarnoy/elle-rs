//! Magnetometer health and hard-iron calibration, on raw MMC5616WA counts.
//!
//! The Core 1 mag task feeds every reading through [`Despike`], then
//! [`Calibration`] while one runs, then [`MagHealth`], which decides whether the
//! offset-corrected field is fit to fuse. Pure math, no hardware.
//!
//! Why: the eagle's sensor sits in a ~15 G airframe field that steps by up to
//! ~3 G within a session, so no stored offset holds and Madgwick steered yaw to
//! the stray field. Its reads also show single-sample spikes of ~(0, +5000,
//! −6000) counts, which the old min/max calibration took at face value.

use elle_config::{
    MAG_CAL_MIN_SPAN_COUNTS, MAG_CAL_SAMPLES, MAG_COUNTS_PER_GAUSS, MAG_DROP_S, MAG_FIELD_MAX_G,
    MAG_FIELD_MIN_G, MAG_READ_HZ, MAG_RESTORE_S, MAG_SPIKE_COUNTS, MAG_STALE_S,
};

/// Readings in `secs` at the mag read rate, at least one.
fn readings(secs: f32) -> u16 {
    let n = libm::ceilf(secs * MAG_READ_HZ);
    if n < 1.0 { 1 } else { n as u16 }
}

fn norm(v: &[f32; 3]) -> f32 {
    libm::sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
}

/// Causal median of the last three readings, per axis. A single-sample spike
/// never reaches the output; a real step arrives one reading late.
#[derive(Clone, Copy, Debug, Default)]
pub struct Despike {
    prev: Option<[f32; 3]>,
    prev2: Option<[f32; 3]>,
}

impl Despike {
    #[must_use]
    pub const fn new() -> Self {
        Self {
            prev: None,
            prev2: None,
        }
    }

    /// The filtered reading, and whether `raw` was a spike (further than
    /// `MAG_SPIKE_COUNTS` from the median on some axis).
    pub fn push(&mut self, raw: [f32; 3]) -> ([f32; 3], bool) {
        let out = match (self.prev2, self.prev) {
            (Some(a), Some(b)) => core::array::from_fn(|i| median3(a[i], b[i], raw[i])),
            _ => raw,
        };
        self.prev2 = self.prev;
        self.prev = Some(raw);
        let spike = (0..3).any(|i| (raw[i] - out[i]).abs() > MAG_SPIKE_COUNTS);
        (out, spike)
    }
}

fn median3(a: f32, b: f32, c: f32) -> f32 {
    a.max(b).min(a.min(b).max(c))
}

/// Why a calibration produced no offsets.
#[derive(Clone, Copy, Debug, PartialEq, defmt::Format)]
pub enum CalFail {
    /// Some axis spanned less than `MAG_CAL_MIN_SPAN_COUNTS`: too little rotation.
    Rotation { min_span: f32 },
    /// The readings' RMS distance from the fitted centre is not an earth-sized
    /// field: something on the airframe moved, or a magnet was close.
    Field { radius_g: f32 },
}

/// One step of a running calibration.
#[derive(Clone, Copy, Debug, PartialEq, defmt::Format)]
pub enum CalStep {
    Collecting(u32),
    Done([f32; 3]),
    Failed(CalFail),
}

/// Hard-iron calibration: the centre of the min/max box over a rotation,
/// accepted when every axis moved and the readings sit on an earth-sized
/// sphere around it.
#[derive(Clone, Copy, Debug)]
pub struct Calibration {
    min: [f32; 3],
    max: [f32; 3],
    n: u32,
    // f64: raw counts reach 2.5e5, their squares summed over 300 readings
    // lose the radius to cancellation in f32.
    sum: [f64; 3],
    sum_sq: f64,
}

impl Default for Calibration {
    fn default() -> Self {
        Self::new()
    }
}

impl Calibration {
    #[must_use]
    pub const fn new() -> Self {
        Self {
            min: [f32::MAX; 3],
            max: [f32::MIN; 3],
            n: 0,
            sum: [0.0; 3],
            sum_sq: 0.0,
        }
    }

    /// Add a (despiked) reading; after `MAG_CAL_SAMPLES` the result.
    pub fn step(&mut self, v: &[f32; 3]) -> CalStep {
        for (i, &x) in v.iter().enumerate() {
            self.min[i] = self.min[i].min(x);
            self.max[i] = self.max[i].max(x);
            self.sum[i] += f64::from(x);
            self.sum_sq += f64::from(x) * f64::from(x);
        }
        self.n += 1;
        if self.n < MAG_CAL_SAMPLES {
            return CalStep::Collecting(self.n);
        }
        let min_span = (0..3)
            .map(|i| self.max[i] - self.min[i])
            .fold(f32::MAX, f32::min);
        if min_span < MAG_CAL_MIN_SPAN_COUNTS {
            return CalStep::Failed(CalFail::Rotation { min_span });
        }
        let c: [f32; 3] = core::array::from_fn(|i| (self.min[i] + self.max[i]) / 2.0);
        // Mean |v − c|² = mean |v|² − 2 c·mean(v) + |c|².
        let n = f64::from(self.n);
        let mut ms = self.sum_sq / n;
        for (&ci, &sum) in c.iter().zip(&self.sum) {
            let ci = f64::from(ci);
            ms += ci * ci - 2.0 * ci * sum / n;
        }
        let radius_g = (libm::sqrt(ms.max(0.0)) as f32) / MAG_COUNTS_PER_GAUSS;
        if !(MAG_FIELD_MIN_G..=MAG_FIELD_MAX_G).contains(&radius_g) {
            return CalStep::Failed(CalFail::Field { radius_g });
        }
        CalStep::Done(c)
    }
}

/// Why the field stopped being fused.
#[derive(Clone, Copy, Debug, PartialEq, defmt::Format)]
pub enum DropReason {
    /// Corrected magnitude outside `MAG_FIELD_MIN_G..=MAG_FIELD_MAX_G`.
    Implausible { gauss: f32 },
    /// The raw reading has not changed for `MAG_STALE_S`.
    Stale,
}

/// A change in whether the field is fused.
#[derive(Clone, Copy, Debug, PartialEq, defmt::Format)]
pub enum MagChange {
    Dropped(DropReason),
    /// Fused again after a reported drop.
    Restored,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    /// No verdict yet (boot, or new offsets).
    Pending,
    Fused,
    Dropped,
}

/// Decides whether the offset-corrected field is fit to fuse: dropped after
/// `MAG_DROP_S` of implausible or stale readings, fused again after
/// `MAG_RESTORE_S` of plausible ones. The first acceptance (boot, new
/// offsets) takes `MAG_DROP_S` and is silent.
#[derive(Clone, Copy, Debug)]
pub struct MagHealth {
    state: State,
    /// A drop was reported and no restore since.
    dropped_reported: bool,
    good: u16,
    bad: u16,
    last_raw: Option<[f32; 3]>,
    repeats: u16,
    drop_n: u16,
    restore_n: u16,
    stale_n: u16,
}

impl Default for MagHealth {
    fn default() -> Self {
        Self::new()
    }
}

impl MagHealth {
    #[must_use]
    pub fn new() -> Self {
        Self {
            state: State::Pending,
            dropped_reported: false,
            good: 0,
            bad: 0,
            last_raw: None,
            repeats: 0,
            drop_n: readings(MAG_DROP_S),
            restore_n: readings(MAG_RESTORE_S),
            stale_n: readings(MAG_STALE_S),
        }
    }

    /// Whether the field is fused now.
    #[must_use]
    pub fn fused(&self) -> bool {
        self.state == State::Fused
    }

    /// New offsets: judge the field afresh. A drop already reported is
    /// answered with `Restored` once it passes.
    pub fn reset(&mut self) {
        self.state = State::Pending;
        self.good = 0;
        self.bad = 0;
    }

    /// One reading: `raw` counts (for the stale check) and the corrected
    /// `field`. Returns a change in whether it is fused, if any.
    pub fn update(&mut self, raw: &[f32; 3], field: &[f32; 3]) -> Option<MagChange> {
        if self.last_raw == Some(*raw) {
            self.repeats = self.repeats.saturating_add(1);
        } else {
            self.repeats = 0;
        }
        self.last_raw = Some(*raw);

        let gauss = norm(field) / MAG_COUNTS_PER_GAUSS;
        let problem = if self.repeats >= self.stale_n.saturating_sub(1) {
            Some(DropReason::Stale)
        } else if !(MAG_FIELD_MIN_G..=MAG_FIELD_MAX_G).contains(&gauss) {
            Some(DropReason::Implausible { gauss })
        } else {
            None
        };
        if problem.is_some() {
            self.bad = self.bad.saturating_add(1);
            self.good = 0;
        } else {
            self.good = self.good.saturating_add(1);
            self.bad = 0;
        }

        match (self.state, problem) {
            (State::Pending | State::Fused, Some(reason)) if self.bad >= self.drop_n => {
                self.state = State::Dropped;
                if self.dropped_reported {
                    None
                } else {
                    self.dropped_reported = true;
                    Some(MagChange::Dropped(reason))
                }
            }
            (State::Pending, None) if self.good >= self.drop_n => self.fuse(),
            (State::Dropped, None) if self.good >= self.restore_n => self.fuse(),
            _ => None,
        }
    }

    fn fuse(&mut self) -> Option<MagChange> {
        self.state = State::Fused;
        if self.dropped_reported {
            self.dropped_reported = false;
            Some(MagChange::Restored)
        } else {
            None
        }
    }
}
