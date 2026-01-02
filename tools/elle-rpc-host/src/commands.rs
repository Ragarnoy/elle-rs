//! RPC command handlers with formatted output

use anyhow::Result;
use console::style;
use elle_rpc_icd::*;
use postcard_rpc::Endpoint;
use std::time::Duration;

use crate::protocol::{decode_response, encode_request};
use crate::transport::Transport;

/// Handler for executing RPC commands over a transport
pub struct CommandHandler<'a, T: Transport> {
    transport: &'a mut T,
    timeout: Duration,
}

impl<'a, T: Transport> CommandHandler<'a, T> {
    pub fn new(transport: &'a mut T) -> Self {
        Self {
            transport,
            timeout: Duration::from_secs(2),
        }
    }

    async fn call<E: Endpoint>(&mut self, req: &E::Request) -> Result<E::Response>
    where
        E::Request: serde::Serialize,
        E::Response: serde::de::DeserializeOwned,
    {
        let request = encode_request::<E>(req)?;
        let response = self.transport.transact(&request, self.timeout).await?;
        decode_response::<E>(&response)
    }

    pub async fn ping(&mut self) -> Result<()> {
        self.call::<PingEndpoint>(&()).await?;
        println!("{}", style("Pong!").green());
        Ok(())
    }

    pub async fn version(&mut self) -> Result<()> {
        let v = self.call::<GetVersionEndpoint>(&()).await?;
        println!(
            "Firmware: {}.{}.{}",
            style(v.major).cyan(),
            style(v.minor).cyan(),
            style(v.patch).cyan()
        );
        Ok(())
    }

    pub async fn status(&mut self) -> Result<()> {
        let s = self.call::<GetStatusEndpoint>(&()).await?;
        println!("{}", style("System Status:").bold());
        println!(
            "  Armed: {}",
            if s.armed {
                style("YES").red().bold()
            } else {
                style("no").green()
            }
        );
        println!(
            "  Failsafe: {}",
            if s.failsafe {
                style("ACTIVE").red().bold()
            } else {
                style("inactive").green()
            }
        );
        println!("  Mode: {:?}", s.mode);
        println!(
            "  IMU Calibrated: {}",
            if s.imu_calibrated {
                style("yes").green()
            } else {
                style("NO").yellow()
            }
        );
        println!("  IMU Errors: {}", s.imu_error_count);
        Ok(())
    }

    pub async fn attitude(&mut self) -> Result<()> {
        let a = self.call::<GetAttitudeEndpoint>(&()).await?;
        println!("{}", style("Attitude:").bold());
        println!(
            "  Pitch: {:>6.1} deg  (rate: {:>6.1} deg/s)",
            a.pitch_cdeg as f32 / 100.0,
            a.pitch_rate_cdeg as f32 / 100.0
        );
        println!(
            "  Roll:  {:>6.1} deg  (rate: {:>6.1} deg/s)",
            a.roll_cdeg as f32 / 100.0,
            a.roll_rate_cdeg as f32 / 100.0
        );
        println!(
            "  Yaw:   {:>6.1} deg  (rate: {:>6.1} deg/s)",
            a.yaw_cdeg as f32 / 100.0,
            a.yaw_rate_cdeg as f32 / 100.0
        );
        Ok(())
    }

    pub async fn perf(&mut self) -> Result<()> {
        let p = self.call::<GetPerformanceEndpoint>(&()).await?;
        println!("{}", style("Performance:").bold());
        println!(
            "  Control Loop: avg={}us, max={}us",
            p.control_loop_avg_us, p.control_loop_max_us
        );
        println!("  IMU Read:     avg={}us, max={}us", p.imu_avg_us, p.imu_max_us);
        Ok(())
    }

    pub async fn arm(&mut self) -> Result<()> {
        let ack = self.call::<ArmEndpoint>(&()).await?;
        if ack.success {
            println!("{}", style("Motors ARMED").red().bold());
        } else {
            println!(
                "{}",
                style(format!("Failed to arm (error: {})", ack.error_code)).red()
            );
        }
        Ok(())
    }

    pub async fn disarm(&mut self) -> Result<()> {
        let ack = self.call::<DisarmEndpoint>(&()).await?;
        if ack.success {
            println!("{}", style("Motors DISARMED").green());
        } else {
            println!(
                "{}",
                style(format!("Failed to disarm (error: {})", ack.error_code)).red()
            );
        }
        Ok(())
    }

    pub async fn throttle(&mut self, percent: u8) -> Result<()> {
        let ack = self
            .call::<SetThrottleEndpoint>(&SetThrottleReq { percent })
            .await?;
        if ack.success {
            println!("Throttle set to {}%", style(percent).cyan());
        } else {
            println!(
                "{}",
                style(format!("Failed (error: {})", ack.error_code)).red()
            );
        }
        Ok(())
    }

    pub async fn elevon(&mut self, left: i8, right: i8) -> Result<()> {
        let ack = self
            .call::<SetElevonsEndpoint>(&SetElevonsReq { left, right })
            .await?;
        if ack.success {
            println!(
                "Elevons: left={}, right={}",
                style(left).cyan(),
                style(right).cyan()
            );
        } else {
            println!(
                "{}",
                style(format!("Failed (error: {})", ack.error_code)).red()
            );
        }
        Ok(())
    }

    pub async fn emergency_stop(&mut self) -> Result<()> {
        let ack = self.call::<EmergencyStopEndpoint>(&()).await?;
        if ack.success {
            println!("{}", style("EMERGENCY STOP executed").red().bold());
        } else {
            println!(
                "{}",
                style(format!("E-Stop failed (error: {})", ack.error_code)).red()
            );
        }
        Ok(())
    }
}
