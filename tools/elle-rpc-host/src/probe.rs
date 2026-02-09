//! Probe-rs connection and RTT attach logic
//!
//! Shared between direct mode and TUI mode.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

use anyhow::{Context, Result};
use cobs::{decode_vec, encode_vec};
use probe_rs::probe::list::Lister;
use probe_rs::rtt::{Rtt, ScanRegion};
use probe_rs::{Permissions, Session};
use tokio::sync::mpsc;

/// Connect to a debug probe and attach RTT.
///
/// Returns the session and RTT instance ready for I/O.
pub fn connect() -> Result<(Session, Rtt)> {
    let probes = Lister::new().list_all();
    if probes.is_empty() {
        anyhow::bail!("No debug probes found");
    }

    eprintln!("Connecting to {:?}...", probes[0]);
    let probe = probes[0].open()?;
    let mut session = probe.attach("RP235x", Permissions::default())?;

    let rtt = {
        let mut core = session.core(0)?;
        core.halt(Duration::from_millis(500))
            .context("Failed to halt core")?;
        eprintln!("Core halted, attaching to RTT...");

        let rtt;
        loop {
            match Rtt::attach_region(&mut core, &ScanRegion::Ram) {
                Ok(r) => {
                    rtt = r;
                    break;
                }
                Err(_e) => {
                    eprintln!("RTT not ready, retrying...");
                    core.run().unwrap();
                    std::thread::sleep(Duration::from_millis(500));
                    core.halt(Duration::from_millis(500)).unwrap();
                }
            }
        }

        core.run().unwrap();
        eprintln!("RTT attached at {:#010x}", rtt.ptr());
        rtt
    };

    Ok((session, rtt))
}

/// Shared shutdown flag for the RTT worker thread.
pub fn shutdown_flag() -> Arc<AtomicBool> {
    Arc::new(AtomicBool::new(false))
}

/// RTT worker thread that bridges blocking probe-rs I/O to async mpsc channels.
///
/// Reads from RTT up channel 1, COBS-decodes frames, and sends them via `inc_tx`.
/// Receives outbound messages from `out_rx`, COBS-encodes, and writes to RTT down channel 0.
///
/// Exits when `shutdown` is set to `true` or `out_rx` disconnects.
pub fn rtt_worker(
    mut session: Session,
    mut rtt: Rtt,
    inc_tx: mpsc::Sender<Vec<u8>>,
    mut out_rx: mpsc::Receiver<Vec<u8>>,
    shutdown: Arc<AtomicBool>,
) {
    let mut core = session.core(0).unwrap();
    let mut buf = [0u8; 1024];
    let mut inc_staging = vec![];
    let mut pending_out: Option<Vec<u8>> = None;

    loop {
        if shutdown.load(Ordering::Relaxed) {
            return;
        }

        let mut progress = false;

        // Read from device (up channel 1 = RPC TX; channel 0 is defmt)
        let up = rtt.up_channel(1).unwrap();
        let got = up.read(&mut core, &mut buf).unwrap();
        if got != 0 {
            progress = true;
            let mut window = &buf[..got];
            while !window.is_empty() {
                if let Some(pos) = window.iter().position(|b| *b == 0) {
                    let (now, later) = window.split_at(pos + 1);
                    inc_staging.extend_from_slice(now);
                    if let Ok(frame) = decode_vec(&inc_staging) {
                        let _ = inc_tx.blocking_send(frame);
                    }
                    inc_staging.clear();
                    window = later;
                } else {
                    inc_staging.extend_from_slice(window);
                    window = &[];
                }
            }
        }

        // Send to device (down channel 0)
        if pending_out.is_none() {
            match out_rx.try_recv() {
                Ok(msg) => {
                    let mut out = encode_vec(&msg);
                    out.push(0);
                    pending_out = Some(out);
                }
                Err(mpsc::error::TryRecvError::Empty) => {}
                Err(mpsc::error::TryRecvError::Disconnected) => return,
            }
        }

        if let Some(tx) = pending_out.take() {
            let down = rtt.down_channel(0).unwrap();
            let ct = down.write(&mut core, &tx).unwrap();
            if ct == tx.len() {
                progress = true;
            } else if ct != 0 {
                progress = true;
                let remaining = tx[ct..].to_vec();
                pending_out = Some(remaining);
            } else {
                pending_out = Some(tx);
            }
        }

        if !progress {
            std::thread::sleep(Duration::from_millis(5));
        }
    }
}
