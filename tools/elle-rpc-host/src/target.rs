//! Build, flash and reset the flight controller through the probe, and read
//! a flight build's defmt log (flight builds have no RPC link).

use std::collections::VecDeque;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use anyhow::{Context, Result, bail};
use probe_rs::flashing::{ElfLoader, ElfOptions, download_file};
use probe_rs::probe::list::Lister;
use probe_rs::{Permissions, Session};
use serde::Serialize;

use crate::probe;

/// The workspace root (this crate is `tools/elle-rpc-host`).
#[must_use]
pub fn repo_root() -> PathBuf {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    root.canonicalize().unwrap_or(root)
}

/// Which airframe binary.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Airframe {
    Eagle,
    Dart,
}

impl Airframe {
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::Eagle => "eagle",
            Self::Dart => "dart",
        }
    }
}

/// A firmware build: `cargo build --release -p elle-<airframe>` with features.
#[derive(Clone, Debug)]
pub struct Build {
    pub airframe: Airframe,
    /// `--no-default-features` (the RPC builds use it; add `gnss` back).
    pub no_default_features: bool,
    pub features: Vec<String>,
}

impl Build {
    /// Whether the build talks RPC (else it logs defmt).
    #[must_use]
    pub fn is_rpc(&self) -> bool {
        self.features.iter().any(|f| f == "rpc-control")
    }

    /// Run cargo; the ELF path on success, cargo's last lines on failure.
    pub fn run(&self) -> Result<PathBuf> {
        let mut cmd = Command::new(std::env::var("CARGO").unwrap_or_else(|_| "cargo".into()));
        cmd.current_dir(repo_root())
            .args(["build", "--release", "-p"])
            .arg(format!("elle-{}", self.airframe.name()));
        if self.no_default_features {
            cmd.arg("--no-default-features");
        }
        if !self.features.is_empty() {
            cmd.arg("--features").arg(self.features.join(","));
        }
        let out = cmd.output().context("running cargo")?;
        if !out.status.success() {
            let err = String::from_utf8_lossy(&out.stderr);
            let tail: Vec<&str> = err.lines().rev().take(30).collect();
            bail!(
                "build failed:\n{}",
                tail.into_iter().rev().collect::<Vec<_>>().join("\n")
            );
        }
        Ok(repo_root()
            .join("target/thumbv8m.main-none-eabihf/release")
            .join(self.airframe.name()))
    }
}

fn open_session() -> Result<Session> {
    let probes = Lister::new().list_all();
    let Some(p) = probes.first() else {
        bail!("No debug probes found");
    };
    Ok(p.open()?.attach("RP235x", Permissions::default())?)
}

/// Write `elf` to the target's flash and reset it.
pub fn flash(elf: &Path) -> Result<()> {
    let mut session = open_session()?;
    download_file(&mut session, elf, ElfLoader(ElfOptions::default()))
        .with_context(|| format!("flashing {}", elf.display()))?;
    session.core(0)?.reset()?;
    Ok(())
}

/// Reset the target (the firmware reboots; the probe is released afterwards).
pub fn reset() -> Result<()> {
    open_session()?.core(0)?.reset()?;
    Ok(())
}

/// One decoded defmt line.
#[derive(Clone, Debug, Serialize)]
pub struct LogLine {
    pub seq: u64,
    /// Milliseconds since the reader started.
    pub t_ms: u64,
    pub level: String,
    /// The firmware's own timestamp, if it prints one.
    pub fw_time: Option<String>,
    pub text: String,
}

/// Lines kept.
const LOG_LEN: usize = 5000;

#[derive(Default)]
pub struct LogRing {
    next_seq: u64,
    lines: VecDeque<LogLine>,
}

impl LogRing {
    fn push(&mut self, started: Instant, level: String, fw_time: Option<String>, text: String) {
        self.next_seq += 1;
        self.lines.push_back(LogLine {
            seq: self.next_seq,
            t_ms: started.elapsed().as_millis() as u64,
            level,
            fw_time,
            text,
        });
        while self.lines.len() > LOG_LEN {
            self.lines.pop_front();
        }
    }

    #[must_use]
    pub fn after(&self, seq: u64, contains: Option<&str>) -> Vec<LogLine> {
        self.lines
            .iter()
            .filter(|l| l.seq > seq && contains.is_none_or(|c| l.text.contains(c)))
            .cloned()
            .collect()
    }

    #[must_use]
    pub const fn last_seq(&self) -> u64 {
        self.next_seq
    }
}

/// Reads a flight build's defmt log (RTT up channel 0) into a ring buffer.
pub struct DefmtReader {
    pub lines: Arc<Mutex<LogRing>>,
    pub elf: PathBuf,
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}

impl DefmtReader {
    /// Attach (waiting up to `limit` for RTT) and start decoding with `elf`'s
    /// defmt table.
    pub fn start(elf: &Path, limit: Duration) -> Result<Self> {
        let bytes = std::fs::read(elf).with_context(|| format!("reading {}", elf.display()))?;
        // Check the table now, so a wrong ELF fails the call, not the thread.
        defmt_decoder::Table::parse(&bytes)?
            .with_context(|| format!("{} has no defmt table (an RPC build?)", elf.display()))?;
        let (session, rtt) = probe::connect_within(Some(limit))?;
        let lines: Arc<Mutex<LogRing>> = Arc::default();
        let stop = Arc::new(AtomicBool::new(false));
        let (l, s) = (lines.clone(), stop.clone());
        let thread = std::thread::spawn(move || read_defmt(session, rtt, &bytes, &l, &s));
        Ok(Self {
            lines,
            elf: elf.to_path_buf(),
            stop,
            thread: Some(thread),
        })
    }

    #[must_use]
    pub fn alive(&self) -> bool {
        self.thread.as_ref().is_some_and(|t| !t.is_finished())
    }

    pub fn stop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

impl Drop for DefmtReader {
    fn drop(&mut self) {
        self.stop();
    }
}

fn read_defmt(
    mut session: Session,
    mut rtt: probe_rs::rtt::Rtt,
    elf: &[u8],
    lines: &Mutex<LogRing>,
    stop: &AtomicBool,
) {
    let Ok(Some(table)) = defmt_decoder::Table::parse(elf) else {
        return;
    };
    let mut decoder = table.new_stream_decoder();
    let Ok(mut core) = session.core(0) else {
        return;
    };
    let started = Instant::now();
    let mut buf = [0u8; 1024];
    while !stop.load(Ordering::Relaxed) {
        let Some(up) = rtt.up_channel(0) else {
            return;
        };
        let Ok(n) = up.read(&mut core, &mut buf) else {
            return;
        };
        if n == 0 {
            std::thread::sleep(Duration::from_millis(10));
            continue;
        }
        decoder.received(&buf[..n]);
        loop {
            match decoder.decode() {
                Ok(frame) => {
                    let level = frame
                        .level()
                        .map_or_else(|| "print".to_string(), |l| format!("{l:?}").to_lowercase());
                    let fw_time = frame.display_timestamp().map(|t| t.to_string());
                    let text = frame.display_message().to_string();
                    lines
                        .lock()
                        .unwrap_or_else(std::sync::PoisonError::into_inner)
                        .push(started, level, fw_time, text);
                }
                Err(defmt_decoder::DecodeError::UnexpectedEof) => break,
                // A corrupt frame (e.g. attached mid-frame): skip it.
                Err(defmt_decoder::DecodeError::Malformed) => {}
            }
        }
    }
}
