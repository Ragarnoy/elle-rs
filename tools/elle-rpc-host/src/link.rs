//! A postcard-RPC client over the probe's RTT link.
//!
//! [`Link::connect`] attaches to the first debug probe (without halting the
//! core), starts the RTT worker thread and builds the `HostClient`. Dropping
//! the link stops the worker and releases the probe.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread::JoinHandle;

use anyhow::Result;
use postcard_rpc::header::VarSeqKind;
use postcard_rpc::host_client::HostClient;
use postcard_rpc::standard_icd::WireError;
use tokio::sync::mpsc;

use crate::probe;
use crate::wire::{ProbeRttRx, ProbeRttTx, TokSpawn};

pub struct Link {
    pub client: HostClient<WireError>,
    shutdown: Arc<AtomicBool>,
    worker: Option<JoinHandle<()>>,
}

impl Link {
    /// Attach to the probe and the firmware's RPC channels. Blocks until the
    /// RTT control block is found.
    pub fn connect() -> Result<Self> {
        Self::connect_within(None)
    }

    /// As [`Self::connect`], giving up after `limit` without an RTT control block.
    pub fn connect_within(limit: Option<std::time::Duration>) -> Result<Self> {
        let (session, rtt) = probe::connect_within(limit)?;
        let (out_tx, out_rx) = mpsc::channel(64);
        let (inc_tx, inc_rx) = mpsc::channel(64);
        let shutdown = probe::shutdown_flag();
        let flag = shutdown.clone();
        let worker =
            std::thread::spawn(move || probe::rtt_worker(session, rtt, inc_tx, out_rx, flag));
        let client = HostClient::<WireError>::new_with_wire(
            ProbeRttTx { out: out_tx },
            ProbeRttRx { inc: inc_rx },
            TokSpawn,
            VarSeqKind::Seq2,
            "error",
            64,
        );
        Ok(Self {
            client,
            shutdown,
            worker: Some(worker),
        })
    }

    /// Whether the RTT worker is still running (it stops when the probe or
    /// the target goes away).
    #[must_use]
    pub fn alive(&self) -> bool {
        self.worker.as_ref().is_some_and(|w| !w.is_finished())
    }

    /// Stop the worker and release the probe.
    pub fn close(mut self) {
        self.stop();
    }

    fn stop(&mut self) {
        self.shutdown.store(true, Ordering::Relaxed);
        self.client.close();
        if let Some(w) = self.worker.take() {
            let _ = w.join();
        }
    }
}

impl Drop for Link {
    fn drop(&mut self) {
        self.stop();
    }
}
