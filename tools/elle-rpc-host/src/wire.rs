//! WireTx/WireRx/WireSpawn implementations for postcard-rpc HostClient
//!
//! Bridges the blocking probe-rs RTT I/O worker thread to the async tokio runtime
//! via mpsc channels.

use std::error::Error;
use std::fmt::{Debug, Display};

use postcard_rpc::host_client::{WireRx, WireSpawn, WireTx};
use tokio::sync::mpsc::{Receiver, Sender};

pub struct ProbeRttTx {
    pub out: Sender<Vec<u8>>,
}

#[derive(Debug)]
pub enum ProbeRttTxError {
    Closed,
}

impl Display for ProbeRttTxError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        <Self as Debug>::fmt(self, f)
    }
}

impl Error for ProbeRttTxError {}

#[derive(Debug)]
pub enum ProbeRttRxError {
    Closed,
}

impl Display for ProbeRttRxError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        <Self as Debug>::fmt(self, f)
    }
}

impl Error for ProbeRttRxError {}

pub struct ProbeRttRx {
    pub inc: Receiver<Vec<u8>>,
}

impl WireTx for ProbeRttTx {
    type Error = ProbeRttTxError;

    async fn send(&mut self, data: Vec<u8>) -> Result<(), Self::Error> {
        self.out.send(data).await.map_err(|_| ProbeRttTxError::Closed)
    }
}

impl WireRx for ProbeRttRx {
    type Error = ProbeRttRxError;

    async fn receive(&mut self) -> Result<Vec<u8>, Self::Error> {
        self.inc.recv().await.ok_or(ProbeRttRxError::Closed)
    }
}

pub struct TokSpawn;

impl WireSpawn for TokSpawn {
    fn spawn(&mut self, fut: impl Future<Output = ()> + Send + 'static) {
        _ = tokio::task::spawn(fut);
    }
}
