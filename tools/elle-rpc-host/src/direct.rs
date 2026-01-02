//! Direct probe-rs access mode (single commands, no cargo-embed needed)

use anyhow::{Context, Result};
use clap::Subcommand;
use elle_rpc_icd::*;
use postcard_rpc::Endpoint;
use probe_rs::{probe::list::Lister, rtt::Rtt, Permissions, Session};
use std::{
    sync::{Arc, Mutex},
    thread,
    time::Duration,
};

use crate::protocol::{decode_response, encode_request};

#[derive(Subcommand)]
pub enum DirectCommand {
    /// Ping the device
    Ping,
    /// Get firmware version
    Version,
    /// Get system status
    Status,
    /// Get attitude data
    Attitude,
    /// Set throttle (0-100%)
    Throttle {
        #[arg(value_parser = clap::value_parser!(u8).range(0..=100))]
        percent: u8,
    },
    /// Set elevon positions (-100 to 100)
    Elevon {
        #[arg(short, long, allow_hyphen_values = true)]
        left: i8,
        #[arg(short, long, allow_hyphen_values = true)]
        right: i8,
    },
    /// Arm motors
    Arm,
    /// Disarm motors
    Disarm,
    /// Emergency stop
    Stop,
    /// Get performance statistics
    Perf,
}

struct RttWorker {
    session: Arc<Mutex<Session>>,
    rtt: Rtt,
    rx_pending: Vec<u8>,
}

impl RttWorker {
    fn call<E: Endpoint>(&mut self, req: &E::Request, timeout: Duration) -> Result<E::Response>
    where
        E::Request: serde::Serialize,
        E::Response: serde::de::DeserializeOwned,
    {
        let request = encode_request::<E>(req)?;
        let mut session = self.session.lock()?;
        let mut core = session.core(0)?;

        self.rtt
            .down_channel(0)
            .context("No down channel")?
            .write(&mut core, &request)?;

        let start = std::time::Instant::now();
        let mut buf = [0u8; 1024];

        loop {
            if start.elapsed() > timeout {
                anyhow::bail!("Response timeout");
            }
            let up = self.rtt.up_channel(1).context("No up channel")?;
            let n = up.read(&mut core, &mut buf)?;
            if n > 0 {
                self.rx_pending.extend_from_slice(&buf[..n]);
                if let Some(pos) = self.rx_pending.iter().position(|&b| b == 0x00) {
                    let frame: Vec<u8> = self.rx_pending.drain(..pos).collect();
                    self.rx_pending.drain(..1);
                    return decode_response::<E>(&frame);
                }
            }
            thread::sleep(Duration::from_millis(1));
        }
    }
}

pub async fn run(cmd: DirectCommand) -> Result<()> {
    let probes = Lister::new().list_all();
    if probes.is_empty() {
        anyhow::bail!("No debug probes found");
    }

    println!("Connecting to {:?}...", probes[0]);
    let probe = probes[0].open()?;
    let session = Arc::new(Mutex::new(
        probe.attach("RP2350", Permissions::default())?,
    ));

    let rtt = {
        let mut s = session.lock()?;
        Rtt::attach(&mut s.core(0)?)?
    };

    let mut w = RttWorker {
        session,
        rtt,
        rx_pending: Vec::new(),
    };
    let t = Duration::from_secs(2);

    match cmd {
        DirectCommand::Ping => {
            w.call::<PingEndpoint>(&(), t)?;
            println!("Pong!");
        }
        DirectCommand::Version => {
            let v = w.call::<GetVersionEndpoint>(&(), t)?;
            println!("Firmware: {}.{}.{}", v.major, v.minor, v.patch);
        }
        DirectCommand::Status => {
            let s = w.call::<GetStatusEndpoint>(&(), t)?;
            println!(
                "Armed: {}, Failsafe: {}, Mode: {:?}, IMU: {}",
                s.armed, s.failsafe, s.mode, s.imu_calibrated
            );
        }
        DirectCommand::Attitude => {
            let a = w.call::<GetAttitudeEndpoint>(&(), t)?;
            println!(
                "Pitch: {:.1}, Roll: {:.1}, Yaw: {:.1}",
                a.pitch_cdeg as f32 / 100.0,
                a.roll_cdeg as f32 / 100.0,
                a.yaw_cdeg as f32 / 100.0
            );
        }
        DirectCommand::Throttle { percent } => {
            let ack = w.call::<SetThrottleEndpoint>(&SetThrottleReq { percent }, t)?;
            println!(
                "{}",
                if ack.success {
                    format!("Throttle: {percent}%")
                } else {
                    format!("Failed: {}", ack.error_code)
                }
            );
        }
        DirectCommand::Elevon { left, right } => {
            let ack = w.call::<SetElevonsEndpoint>(&SetElevonsReq { left, right }, t)?;
            println!(
                "{}",
                if ack.success {
                    format!("Elevons: {left}, {right}")
                } else {
                    format!("Failed: {}", ack.error_code)
                }
            );
        }
        DirectCommand::Arm => {
            let ack = w.call::<ArmEndpoint>(&(), t)?;
            println!("{}", if ack.success { "ARMED" } else { "Failed to arm" });
        }
        DirectCommand::Disarm => {
            let ack = w.call::<DisarmEndpoint>(&(), t)?;
            println!(
                "{}",
                if ack.success {
                    "DISARMED"
                } else {
                    "Failed to disarm"
                }
            );
        }
        DirectCommand::Stop => {
            let ack = w.call::<EmergencyStopEndpoint>(&(), t)?;
            println!(
                "{}",
                if ack.success {
                    "EMERGENCY STOP"
                } else {
                    "E-Stop failed"
                }
            );
        }
        DirectCommand::Perf => {
            let p = w.call::<GetPerformanceEndpoint>(&(), t)?;
            println!(
                "Control: {}us avg, {}us max | IMU: {}us avg",
                p.control_loop_avg_us, p.control_loop_max_us, p.imu_avg_us
            );
        }
    }
    Ok(())
}
