//! Post-session work for `elle mcp`: fetch ULog files off the SD card, run the
//! log tools on them (`elle_log.py`, `elle-replay`), and record test results.
//!
//! Everything lands in the repository's `logs/` (gitignored): the copied logs,
//! and the test results as `logs/test-runs/<date>.jsonl`, one line per result.

use std::path::{Path, PathBuf};
use std::process::Command;

use anyhow::{Context, Result, bail};
use serde::{Deserialize, Serialize};
use serde_json::Value;

use crate::target::repo_root;

/// Where the card mounts: `/run/media/$USER/<label>/`.
#[must_use]
pub fn card_root() -> PathBuf {
    let user = std::env::var("USER").unwrap_or_default();
    PathBuf::from("/run/media").join(user)
}

/// The repository's `logs/`.
#[must_use]
pub fn logs_dir() -> PathBuf {
    repo_root().join("logs")
}

#[derive(Clone, Debug, Serialize)]
pub struct LogFile {
    pub name: String,
    pub path: String,
    pub bytes: u64,
    /// Whether `logs/` already holds a copy of the same size.
    pub copied: bool,
}

fn is_ulog(name: &str) -> bool {
    name.starts_with("LOG_")
        && Path::new(name)
            .extension()
            .is_some_and(|e| e.eq_ignore_ascii_case("ulg"))
}

/// ULog files on every mounted card under `root`, oldest number first.
pub fn card_logs(root: &Path, logs: &Path) -> Result<Vec<LogFile>> {
    let mut out = Vec::new();
    let Ok(mounts) = std::fs::read_dir(root) else {
        bail!(
            "nothing mounted under {} (is the card in the reader?)",
            root.display()
        );
    };
    for mount in mounts.flatten() {
        let Ok(files) = std::fs::read_dir(mount.path()) else {
            continue;
        };
        for f in files.flatten() {
            let name = f.file_name().to_string_lossy().into_owned();
            if !is_ulog(&name) {
                continue;
            }
            let bytes = f.metadata().map(|m| m.len()).unwrap_or(0);
            let copied = std::fs::metadata(logs.join(&name)).is_ok_and(|m| m.len() == bytes);
            out.push(LogFile {
                name,
                path: f.path().display().to_string(),
                bytes,
                copied,
            });
        }
    }
    out.sort_by(|a, b| a.name.cmp(&b.name));
    Ok(out)
}

/// Copy card logs into `logs`: the ones named, else the newest `last`.
/// A file already there with the same size is skipped; one with a different
/// size is never overwritten (a card reused after renumbering).
pub fn copy_logs(
    card: &[LogFile],
    logs: &Path,
    names: &[String],
    last: Option<usize>,
) -> Result<Value> {
    let chosen: Vec<&LogFile> = if names.is_empty() {
        let n = last.unwrap_or(1);
        card.iter().skip(card.len().saturating_sub(n)).collect()
    } else {
        names
            .iter()
            .map(|n| {
                card.iter()
                    .find(|f| f.name.eq_ignore_ascii_case(n))
                    .with_context(|| format!("{n} is not on the card"))
            })
            .collect::<Result<_>>()?
    };
    std::fs::create_dir_all(logs)?;
    let (mut copied, mut skipped, mut refused) = (Vec::new(), Vec::new(), Vec::new());
    for f in chosen {
        let dest = logs.join(&f.name);
        match std::fs::metadata(&dest) {
            Ok(m) if m.len() == f.bytes => skipped.push(f.name.clone()),
            Ok(_) => refused.push(format!(
                "{}: logs/ already has a different file of that name",
                f.name
            )),
            Err(_) => {
                std::fs::copy(&f.path, &dest)
                    .with_context(|| format!("copying {} to {}", f.path, dest.display()))?;
                copied.push(dest.display().to_string());
            }
        }
    }
    Ok(
        serde_json::json!({ "copied": copied, "already_there": skipped, "not_overwritten": refused }),
    )
}

/// A log by name (`LOG_0061.ulg`, found in `logs/`) or path.
pub fn resolve_log(name: &str) -> Result<PathBuf> {
    let p = Path::new(name);
    if p.is_absolute() || p.exists() {
        return Ok(p.to_path_buf());
    }
    let in_logs = logs_dir().join(name);
    if in_logs.exists() {
        return Ok(in_logs);
    }
    bail!("{name} not found (copy it off the card with `copy_logs`)")
}

/// `elle_log.py` subcommands.
#[derive(Clone, Copy, Debug, Deserialize, schemars::JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum LogCommand {
    List,
    Summary,
    Timing,
    Esc,
    Sensors,
    Stages,
    Nav,
    Window,
}

impl LogCommand {
    const fn name(self) -> &'static str {
        match self {
            Self::List => "list",
            Self::Summary => "summary",
            Self::Timing => "timing",
            Self::Esc => "esc",
            Self::Sensors => "sensors",
            Self::Stages => "stages",
            Self::Nav => "nav",
            Self::Window => "window",
        }
    }
}

/// Run `elle_log.py <cmd> FILES [t0 t1]` with the `logs/.venv` Python (or
/// `python3`); its text output.
pub fn elle_log(cmd: LogCommand, files: &[PathBuf], window: Option<(f64, f64)>) -> Result<String> {
    let root = repo_root();
    let venv = root.join("logs/.venv/bin/python");
    let python = if venv.exists() {
        venv
    } else {
        PathBuf::from("python3")
    };
    let mut c = Command::new(&python);
    c.current_dir(&root)
        .arg(root.join(".claude/skills/flight-logs/elle_log.py"))
        .arg(cmd.name())
        .args(files);
    if let LogCommand::Window = cmd {
        let (t0, t1) = window.context("window needs t0_s and t1_s")?;
        if files.len() != 1 {
            bail!("window takes exactly one file");
        }
        c.arg(t0.to_string()).arg(t1.to_string());
    }
    let out = c
        .output()
        .with_context(|| format!("running {}", python.display()))?;
    let mut text = String::from_utf8_lossy(&out.stdout).into_owned();
    let err = String::from_utf8_lossy(&out.stderr);
    if !out.status.success() {
        bail!("elle_log.py {} failed:\n{text}{err}", cmd.name());
    }
    if !err.trim().is_empty() {
        text.push_str("\n[stderr]\n");
        text.push_str(&err);
    }
    Ok(text)
}

/// The host's target triple, for building the host tools in a workspace that
/// defaults to thumbv8m.
fn host_triple() -> &'static str {
    match (std::env::consts::ARCH, std::env::consts::OS) {
        ("aarch64", "linux") => "aarch64-unknown-linux-gnu",
        ("aarch64", "macos") => "aarch64-apple-darwin",
        ("x86_64", "macos") => "x86_64-apple-darwin",
        _ => "x86_64-unknown-linux-gnu",
    }
}

/// What `replay` runs.
#[derive(Clone, Debug, Default)]
pub struct ReplayOpts {
    pub compare: bool,
    pub score_by_rate: bool,
    /// Simulate a flight into the file first: (wind north, wind east, vibration, gyro bias °/s).
    pub simulate: Option<[f64; 4]>,
}

/// Run `elle-replay FILE --json` (built for the host on first use); its report.
/// A replay that does not reproduce the firmware still returns its report,
/// with `problems` filled in.
pub fn replay(file: &Path, opts: &ReplayOpts) -> Result<Value> {
    let mut c = Command::new(std::env::var("CARGO").unwrap_or_else(|_| "cargo".into()));
    c.current_dir(repo_root())
        .args(["run", "-q", "--release", "-p", "elle-replay", "--target"])
        .arg(host_triple())
        .arg("--")
        .arg(file)
        .arg("--json");
    if opts.compare {
        c.arg("--compare");
    }
    if opts.score_by_rate {
        c.arg("--score-by-rate");
    }
    if let Some([n, e, vib, bias]) = opts.simulate {
        c.arg("--simulate")
            .args(["--wind-north", &n.to_string()])
            .args(["--wind-east", &e.to_string()])
            .args(["--vibration", &vib.to_string()])
            .args(["--gyro-bias-dps", &bias.to_string()]);
    }
    let out = c.output().context("running elle-replay")?;
    let stdout = String::from_utf8_lossy(&out.stdout);
    match serde_json::from_str::<Value>(&stdout) {
        Ok(v) => Ok(v),
        Err(_) => {
            let err = String::from_utf8_lossy(&out.stderr);
            let tail: Vec<&str> = err.lines().rev().take(20).collect();
            bail!(
                "elle-replay failed:\n{}",
                tail.into_iter().rev().collect::<Vec<_>>().join("\n")
            )
        }
    }
}

// ---------------------------------------------------------------------------
// Test results

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum Outcome {
    Pass,
    Fail,
    Skip,
    /// Ran, but the result needs a person's judgement.
    Inconclusive,
}

/// One recorded result.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct TestResult {
    /// RFC 3339, local time.
    pub at: String,
    /// TEST_PLAN part, e.g. `7`.
    pub part: String,
    /// Row or section, e.g. `7.2` or `6.9 step 3`.
    pub row: String,
    pub outcome: Outcome,
    #[serde(default)]
    pub note: String,
    /// Evidence: ULog files, measured values.
    #[serde(default)]
    pub logs: Vec<String>,
    /// The firmware under test (`GetBuildInfo`), when known.
    #[serde(default)]
    pub build: Option<Value>,
}

/// `logs/test-runs/`.
#[must_use]
pub fn runs_dir(logs: &Path) -> PathBuf {
    logs.join("test-runs")
}

/// Append a result to today's file; the file's path.
pub fn record(logs: &Path, r: &TestResult) -> Result<PathBuf> {
    use std::io::Write;
    let dir = runs_dir(logs);
    std::fs::create_dir_all(&dir)?;
    let path = dir.join(format!("{}.jsonl", chrono::Local::now().format("%Y-%m-%d")));
    let mut f = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(&path)
        .with_context(|| format!("opening {}", path.display()))?;
    writeln!(f, "{}", serde_json::to_string(r)?)?;
    Ok(path)
}

/// The results of one day (`YYYY-MM-DD`, default the newest file): every
/// record, the latest outcome per row, and counts.
pub fn report(logs: &Path, date: Option<&str>) -> Result<Value> {
    let dir = runs_dir(logs);
    let path = match date {
        Some(d) => dir.join(format!("{d}.jsonl")),
        None => {
            let mut files: Vec<PathBuf> = std::fs::read_dir(&dir)
                .with_context(|| format!("no results yet ({})", dir.display()))?
                .flatten()
                .map(|e| e.path())
                .filter(|p| p.extension().is_some_and(|e| e == "jsonl"))
                .collect();
            files.sort();
            files.pop().context("no results yet")?
        }
    };
    let text =
        std::fs::read_to_string(&path).with_context(|| format!("reading {}", path.display()))?;
    let results: Vec<TestResult> = text
        .lines()
        .filter(|l| !l.trim().is_empty())
        .map(serde_json::from_str)
        .collect::<Result<_, _>>()
        .with_context(|| format!("parsing {}", path.display()))?;
    // Latest outcome per (part, row), in first-seen order.
    let mut latest: Vec<&TestResult> = Vec::new();
    for r in &results {
        match latest
            .iter_mut()
            .find(|l| l.part == r.part && l.row == r.row)
        {
            Some(l) => *l = r,
            None => latest.push(r),
        }
    }
    let count = |o| latest.iter().filter(|r| r.outcome == o).count();
    Ok(serde_json::json!({
        "file": path.display().to_string(),
        "counts": {
            "pass": count(Outcome::Pass),
            "fail": count(Outcome::Fail),
            "skip": count(Outcome::Skip),
            "inconclusive": count(Outcome::Inconclusive),
        },
        "latest": latest,
        "records": results.len(),
    }))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tmp(name: &str) -> PathBuf {
        let d = std::env::temp_dir().join(format!("elle-analysis-{name}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&d);
        std::fs::create_dir_all(&d).unwrap();
        d
    }

    #[test]
    fn card_listing_and_copy() {
        let root = tmp("card");
        let card = root.join("media/F22D-C1B3");
        let logs = root.join("logs");
        std::fs::create_dir_all(&card).unwrap();
        std::fs::create_dir_all(&logs).unwrap();
        std::fs::write(card.join("LOG_0002.ulg"), b"bb").unwrap();
        std::fs::write(card.join("LOG_0001.ULG"), b"a").unwrap();
        std::fs::write(card.join("notes.txt"), b"x").unwrap();
        // Same name, different content: must not be overwritten.
        std::fs::write(logs.join("LOG_0001.ULG"), b"zzz").unwrap();

        let files = card_logs(&root.join("media"), &logs).unwrap();
        let names: Vec<_> = files.iter().map(|f| f.name.as_str()).collect();
        assert_eq!(names, ["LOG_0001.ULG", "LOG_0002.ulg"]);

        let r = copy_logs(&files, &logs, &[], Some(5)).unwrap();
        assert_eq!(r["copied"].as_array().unwrap().len(), 1);
        assert_eq!(r["not_overwritten"].as_array().unwrap().len(), 1);
        assert_eq!(std::fs::read(logs.join("LOG_0001.ULG")).unwrap(), b"zzz");
        assert_eq!(std::fs::read(logs.join("LOG_0002.ulg")).unwrap(), b"bb");

        let again = copy_logs(&files, &logs, &["log_0002.ulg".into()], None).unwrap();
        assert_eq!(again["already_there"][0], "LOG_0002.ulg");
        assert!(copy_logs(&files, &logs, &["LOG_0009.ulg".into()], None).is_err());
    }

    #[test]
    fn record_and_report_keep_the_latest_outcome_per_row() {
        let logs = tmp("runs");
        let r = |row: &str, outcome| TestResult {
            at: "2026-09-30T10:00:00+02:00".into(),
            part: "7".into(),
            row: row.into(),
            outcome,
            note: String::new(),
            logs: Vec::new(),
            build: None,
        };
        record(&logs, &r("7.1", Outcome::Fail)).unwrap();
        record(&logs, &r("7.2", Outcome::Skip)).unwrap();
        record(&logs, &r("7.1", Outcome::Pass)).unwrap();
        let rep = report(&logs, None).unwrap();
        assert_eq!(rep["records"], 3);
        assert_eq!(rep["counts"]["pass"], 1);
        assert_eq!(rep["counts"]["fail"], 0);
        assert_eq!(rep["counts"]["skip"], 1);
        assert_eq!(rep["latest"][0]["row"], "7.1");
        assert_eq!(rep["latest"][0]["outcome"], "pass");
    }
}
