//! The whole analysis of one log as a value: what the CLI prints, and what
//! `--json` (and `elle mcp`'s `replay` tool) returns.

use anyhow::Result;
use serde::Serialize;

use crate::reference::{self, Score, Select};
use crate::sim::Truth;
use crate::ulog::ULog;
use crate::{Coverage, Faithfulness, Replay};

/// What to run.
#[derive(Clone, Debug)]
pub struct Options {
    /// Run the alternative filters and score them.
    pub compare: bool,
    /// Gate threshold for the gated variants, g from 1 g.
    pub gate_g: f32,
    /// Count turns by turn rate instead of bank (vehicle tests).
    pub score_by_rate: bool,
}

impl Default for Options {
    fn default() -> Self {
        Self {
            compare: false,
            gate_g: crate::variants::DEFAULT_GATE_G,
            score_by_rate: false,
        }
    }
}

#[derive(Clone, Debug, Serialize)]
pub struct CoverageReport {
    pub samples: u64,
    pub seconds: f64,
    pub synced: u64,
    pub synced_pct: f64,
    pub gaps: u64,
    pub lost: u64,
    pub contexts: u64,
    pub mag_changes: u64,
    pub roundtrip_errors: u32,
    pub dropouts: usize,
}

impl From<&Coverage> for CoverageReport {
    fn from(c: &Coverage) -> Self {
        Self {
            samples: c.samples,
            seconds: c.samples as f64 / 1000.0,
            synced: c.synced,
            synced_pct: 100.0 * c.synced as f64 / c.samples.max(1) as f64,
            gaps: c.gaps,
            lost: c.lost,
            contexts: c.contexts,
            mag_changes: c.mag_changes,
            roundtrip_errors: c.roundtrip_errors,
            dropouts: c.dropouts,
        }
    }
}

#[derive(Clone, Debug, Serialize)]
pub struct FaithfulnessReport {
    pub exact: u64,
    pub mismatched: u64,
    pub unchecked: u64,
    pub worst_deg: f32,
    pub ok: bool,
}

impl From<&Faithfulness> for FaithfulnessReport {
    fn from(f: &Faithfulness) -> Self {
        Self {
            exact: f.exact,
            mismatched: f.mismatched,
            unchecked: f.unchecked,
            worst_deg: f.worst_deg,
            ok: f.ok(),
        }
    }
}

/// One filter's errors in turns, degrees; `[right, left]`.
#[derive(Clone, Debug, Serialize)]
pub struct ScoreRow {
    pub name: String,
    pub roll_mean_deg: [f32; 2],
    pub roll_rms_deg: f32,
    pub pitch_rms_deg: [f32; 2],
    pub samples: [u64; 2],
}

impl From<&Score> for ScoreRow {
    fn from(s: &Score) -> Self {
        Self {
            name: s.name.clone(),
            roll_mean_deg: s.roll_mean_deg,
            roll_rms_deg: s.roll_rms_both(),
            pitch_rms_deg: s.pitch_rms_deg,
            samples: s.samples,
        }
    }
}

/// The filter comparison (`compare`).
#[derive(Clone, Debug, Serialize)]
pub struct Comparison {
    /// Share of samples the gyro reference covers, %.
    pub reference_coverage_pct: f64,
    /// Against the gyro reference, in turns.
    pub vs_reference: Vec<ScoreRow>,
    /// Against the simulated truth (simulations only).
    pub vs_truth: Option<Vec<ScoreRow>>,
    /// The reference itself against the truth (simulations only).
    pub reference_vs_truth: Option<ScoreRow>,
    /// Variants that gated samples, and the share, %.
    pub gated_pct: Vec<(String, f32)>,
}

#[derive(Clone, Debug, Serialize)]
pub struct Report {
    pub log: String,
    /// The file ends inside a record (power cut).
    pub truncated: bool,
    pub coverage: CoverageReport,
    pub faithfulness: FaithfulnessReport,
    pub comparison: Option<Comparison>,
    /// Why the replay cannot be trusted, if it cannot.
    pub problems: Vec<String>,
}

impl Report {
    /// The replay reproduces the firmware, so the rest can be trusted.
    #[must_use]
    pub fn ok(&self) -> bool {
        self.problems.is_empty()
    }
}

/// Replay `data` (the bytes of `name`) and, with `opts.compare`, score the
/// filters; `truth` for a simulated flight.
pub fn analyse(
    name: &str,
    data: &[u8],
    opts: &Options,
    truth: Option<&[Truth]>,
) -> Result<(Report, Replay)> {
    let log = ULog::parse(data)?;
    let specs: Vec<_> = if opts.compare {
        crate::variants::default_specs()
            .into_iter()
            .map(|mut s| {
                s.gate_g = s.gate_g.map(|_| opts.gate_g);
                s
            })
            .collect()
    } else {
        Vec::new()
    };
    let replay = crate::replay_with(&log, &specs)?;
    let f = crate::faithfulness(&log, &replay);

    let comparison = opts.compare.then(|| {
        let select = if opts.score_by_rate {
            Select::Rate
        } else {
            Select::Bank
        };
        let refr = reference::build(&replay);
        let covered = refr.iter().filter(|r| r.is_some()).count();
        let rows = |s: Vec<Score>| s.iter().map(ScoreRow::from).collect::<Vec<_>>();
        let truth_angles: Option<Vec<Option<[f32; 3]>>> = truth.map(|t| {
            replay
                .samples
                .iter()
                .map(|s| {
                    t.get(s.index as usize)
                        .map(|t| [t.pitch as f32, t.roll as f32, t.yaw as f32])
                })
                .collect()
        });
        Comparison {
            reference_coverage_pct: 100.0 * covered as f64 / refr.len().max(1) as f64,
            vs_reference: rows(reference::score_all_by(&replay, &refr, select)),
            vs_truth: truth_angles
                .as_ref()
                .map(|t| rows(reference::score_all_by(&replay, t, select))),
            reference_vs_truth: truth_angles
                .as_ref()
                .map(|t| ScoreRow::from(&reference::score("reference", refr.iter().copied(), t))),
            gated_pct: crate::compare(&replay)
                .iter()
                .filter(|c| c.gated_share > 0.0)
                .map(|c| (c.name.clone(), 100.0 * c.gated_share))
                .collect(),
        }
    });

    let mut problems = Vec::new();
    if replay.coverage.roundtrip_errors > 0 {
        problems.push(
            "the logged integers do not reproduce the driver's floats (scale mismatch)".into(),
        );
    }
    if !f.ok() {
        problems.push(format!(
            "the replay does not reproduce the firmware (worst {:.4}°); results from it cannot be trusted",
            f.worst_deg
        ));
    }
    let report = Report {
        log: name.to_string(),
        truncated: log.truncated,
        coverage: (&replay.coverage).into(),
        faithfulness: (&f).into(),
        comparison,
        problems,
    };
    Ok((report, replay))
}
