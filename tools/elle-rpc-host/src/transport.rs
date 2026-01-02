//! Transport layer implementations

use anyhow::{Context, Result};
use console::style;
use elle_rpc_icd::*;
use postcard_rpc::header::{VarHeader, VarKey};
use postcard_rpc::Endpoint;
use std::time::Duration;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::TcpStream;

use crate::protocol::encode_response;

/// Transport abstraction for RPC communication
pub trait Transport {
    fn transact(
        &mut self,
        request: &[u8],
        timeout: Duration,
    ) -> impl Future<Output = Result<Vec<u8>>> + Send;
}

/// TCP transport connecting to cargo-embed sockets
pub struct TcpTransport {
    rx_stream: TcpStream,
    tx_stream: TcpStream,
    rx_pending: Vec<u8>,
}

impl TcpTransport {
    pub async fn connect(rx_addr: &str, tx_addr: &str) -> Result<Self> {
        let rx_stream = TcpStream::connect(rx_addr)
            .await
            .context("Failed to connect (is cargo-embed running?)")?;
        let tx_stream = TcpStream::connect(tx_addr).await?;
        Ok(Self {
            rx_stream,
            tx_stream,
            rx_pending: Vec::new(),
        })
    }

    pub async fn connect_with_retry(rx_addr: &str, tx_addr: &str) -> Self {
        let mut delay = Duration::from_millis(100);
        loop {
            match Self::connect(rx_addr, tx_addr).await {
                Ok(t) => return t,
                Err(_) => {
                    println!(
                        "{}",
                        style(format!("Waiting for cargo-embed... ({delay:?})")).dim()
                    );
                    tokio::time::sleep(delay).await;
                    delay = (delay * 2).min(Duration::from_secs(1));
                }
            }
        }
    }
}

impl Transport for TcpTransport {
    async fn transact(&mut self, request: &[u8], timeout: Duration) -> Result<Vec<u8>> {
        self.tx_stream.write_all(request).await?;
        self.tx_stream.flush().await?;

        let mut buf = [0u8; 1024];
        let start = std::time::Instant::now();

        loop {
            if start.elapsed() > timeout {
                anyhow::bail!("Response timeout");
            }

            match tokio::time::timeout(Duration::from_millis(100), self.rx_stream.read(&mut buf))
                .await
            {
                Ok(Ok(n)) if n > 0 => {
                    self.rx_pending.extend_from_slice(&buf[..n]);
                    if let Some(pos) = self.rx_pending.iter().position(|&b| b == 0x00) {
                        let frame = self.rx_pending.drain(..pos).collect();
                        self.rx_pending.drain(..1);
                        return Ok(frame);
                    }
                }
                Ok(Ok(_)) => anyhow::bail!("Connection closed"),
                Ok(Err(e)) => anyhow::bail!("Read error: {e}"),
                Err(_) => {}
            }
        }
    }
}

/// Mock transport for dry-run mode
pub struct MockTransport;

impl Transport for MockTransport {
    async fn transact(&mut self, request: &[u8], _timeout: Duration) -> Result<Vec<u8>> {
        // Decode request to determine endpoint
        let mut decoded = vec![0u8; request.len()];
        let report = cobs::decode(request, &mut decoded)?;
        let (header, _) =
            VarHeader::take_from_slice(&decoded[..report.frame_size()]).context("Invalid header")?;

        // Simulate delay
        tokio::time::sleep(Duration::from_millis(10)).await;

        // Return fake response based on request key
        match header.key {
            VarKey::Key8(k) if k == PingEndpoint::REQ_KEY => encode_response::<PingEndpoint>(&()),
            VarKey::Key8(k) if k == GetVersionEndpoint::REQ_KEY => {
                encode_response::<GetVersionEndpoint>(&VersionResp {
                    major: 0,
                    minor: 1,
                    patch: 0,
                })
            }
            VarKey::Key8(k) if k == GetStatusEndpoint::REQ_KEY => {
                encode_response::<GetStatusEndpoint>(&StatusResp {
                    armed: false,
                    failsafe: false,
                    mode: ControlMode::Manual,
                    imu_calibrated: true,
                    imu_error_count: 0,
                })
            }
            VarKey::Key8(k) if k == GetAttitudeEndpoint::REQ_KEY => {
                encode_response::<GetAttitudeEndpoint>(&AttitudeResp {
                    pitch_cdeg: 250,
                    roll_cdeg: -150,
                    yaw_cdeg: 9000,
                    pitch_rate_cdeg: 10,
                    roll_rate_cdeg: -5,
                    yaw_rate_cdeg: 0,
                })
            }
            VarKey::Key8(k) if k == GetPerformanceEndpoint::REQ_KEY => {
                encode_response::<GetPerformanceEndpoint>(&PerformanceResp {
                    control_loop_avg_us: 450,
                    control_loop_max_us: 1200,
                    imu_avg_us: 280,
                    imu_max_us: 500,
                })
            }
            VarKey::Key8(k)
                if k == ArmEndpoint::REQ_KEY
                    || k == DisarmEndpoint::REQ_KEY
                    || k == SetThrottleEndpoint::REQ_KEY
                    || k == SetElevonsEndpoint::REQ_KEY
                    || k == EmergencyStopEndpoint::REQ_KEY =>
            {
                encode_response::<ArmEndpoint>(&AckResp {
                    success: true,
                    error_code: 0,
                })
            }
            _ => anyhow::bail!("Unknown endpoint in dry-run mode"),
        }
    }
}
