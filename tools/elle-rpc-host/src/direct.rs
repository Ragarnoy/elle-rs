//! Direct probe-rs access mode (single commands for scripting)

use std::time::Duration;

use anyhow::Result;
use clap::Subcommand;
use elle_rpc_icd::*;
use postcard_rpc::header::VarSeqKind;
use postcard_rpc::host_client::HostClient;
use postcard_rpc::standard_icd::WireError;
use tokio::sync::mpsc;
use tokio::time::timeout;

use crate::probe;
use crate::wire::{ProbeRttRx, ProbeRttTx, TokSpawn};

const CMD_TIMEOUT: Duration = Duration::from_secs(2);

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
    /// Read magnetometer (XYZ signed counts)
    Mag,
    /// Read GNSS position fix
    Gnss,
    /// Read engine RPM telemetry
    Engine,
    /// Magnetometer calibration
    MagCal {
        #[command(subcommand)]
        action: MagCalAction,
    },
}

#[derive(Subcommand)]
pub enum MagCalAction {
    /// Start calibration (rotate board for ~30s)
    Start,
    /// Clear calibration (zero offsets)
    Clear,
    /// Get calibration status and offsets
    Status,
}

struct ProbeConnection {
    client: HostClient<WireError>,
    shutdown: std::sync::Arc<std::sync::atomic::AtomicBool>,
    worker_handle: std::thread::JoinHandle<()>,
}

fn connect_client() -> Result<ProbeConnection> {
    let (session, rtt) = probe::connect()?;

    let (out_tx, out_rx) = mpsc::channel(64);
    let (inc_tx, inc_rx) = mpsc::channel(64);

    let shutdown = probe::shutdown_flag();
    let shutdown_clone = shutdown.clone();
    let worker_handle =
        std::thread::spawn(move || probe::rtt_worker(session, rtt, inc_tx, out_rx, shutdown_clone));

    let client = HostClient::<WireError>::new_with_wire(
        ProbeRttTx { out: out_tx },
        ProbeRttRx { inc: inc_rx },
        TokSpawn,
        VarSeqKind::Seq2,
        "error",
        64,
    );

    Ok(ProbeConnection {
        client,
        shutdown,
        worker_handle,
    })
}

pub async fn run(cmd: DirectCommand) -> Result<()> {
    let conn = connect_client()?;
    let client = &conn.client;

    match cmd {
        DirectCommand::Ping => {
            timeout(CMD_TIMEOUT, client.send_resp::<PingEndpoint>(&())).await??;
            println!("Pong!");
        }
        DirectCommand::Version => {
            let v = timeout(CMD_TIMEOUT, client.send_resp::<GetVersionEndpoint>(&())).await??;
            println!("Firmware: {}.{}.{}", v.major, v.minor, v.patch);
        }
        DirectCommand::Status => {
            let s = timeout(CMD_TIMEOUT, client.send_resp::<GetStatusEndpoint>(&())).await??;
            println!(
                "Armed: {}, Failsafe: {}, Mode: {:?}, IMU: {}",
                s.armed, s.failsafe, s.mode, s.imu_calibrated
            );
        }
        DirectCommand::Attitude => {
            let a = timeout(CMD_TIMEOUT, client.send_resp::<GetAttitudeEndpoint>(&())).await??;
            println!(
                "Pitch: {:.1}, Roll: {:.1}, Yaw: {:.1}",
                a.pitch_cdeg as f32 / 100.0,
                a.roll_cdeg as f32 / 100.0,
                a.yaw_cdeg as f32 / 100.0
            );
        }
        DirectCommand::Throttle { percent } => {
            let ack = timeout(
                CMD_TIMEOUT,
                client.send_resp::<SetThrottleEndpoint>(&SetThrottleReq { percent }),
            )
            .await??;
            if ack.success {
                println!("Throttle: {percent}%");
            } else {
                println!("Failed: {}", ack.error_code);
            }
        }
        DirectCommand::Elevon { left, right } => {
            let ack = timeout(
                CMD_TIMEOUT,
                client.send_resp::<SetElevonsEndpoint>(&SetElevonsReq { left, right }),
            )
            .await??;
            if ack.success {
                println!("Elevons: {left}, {right}");
            } else {
                println!("Failed: {}", ack.error_code);
            }
        }
        DirectCommand::Arm => {
            let ack = timeout(CMD_TIMEOUT, client.send_resp::<ArmEndpoint>(&())).await??;
            println!(
                "{}",
                if ack.success {
                    "ARMED"
                } else {
                    "Failed to arm"
                }
            );
        }
        DirectCommand::Disarm => {
            let ack = timeout(CMD_TIMEOUT, client.send_resp::<DisarmEndpoint>(&())).await??;
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
            let ack =
                timeout(CMD_TIMEOUT, client.send_resp::<EmergencyStopEndpoint>(&())).await??;
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
            let p = timeout(CMD_TIMEOUT, client.send_resp::<GetPerformanceEndpoint>(&())).await??;
            println!(
                "Control: {}us avg, {}us max | IMU: {}us avg",
                p.control_loop_avg_us, p.control_loop_max_us, p.imu_avg_us
            );
        }
        DirectCommand::Mag => {
            let m = timeout(
                CMD_TIMEOUT,
                client.send_resp::<GetMagnetometerEndpoint>(&()),
            )
            .await??;
            println!("Mag: X={} Y={} Z={}", m.x, m.y, m.z);
        }
        DirectCommand::Gnss => {
            let g = timeout(CMD_TIMEOUT, client.send_resp::<GetGnssEndpoint>(&())).await??;
            println!(
                "GNSS: {:.6},{:.6} alt={:.1}m fix={} sats={} hdop={:.1}",
                g.latitude, g.longitude, g.altitude_m, g.fix_quality, g.num_satellites, g.hdop
            );
            println!(
                "      spd={:.2}m/s trk={:.1}° vel_ned=({:.2},{:.2},{:.2})m/s",
                g.ground_speed_ms,
                g.heading_motion_deg,
                g.vel_n_ms,
                g.vel_e_ms,
                g.vel_d_ms
            );
            println!(
                "      hAcc={:.2}m vAcc={:.2}m sAcc={:.2}m/s",
                g.h_acc_m, g.v_acc_m, g.s_acc_ms
            );
        }
        DirectCommand::MagCal { action } => match action {
            MagCalAction::Start => {
                let ack =
                    timeout(CMD_TIMEOUT, client.send_resp::<StartMagCalEndpoint>(&())).await??;
                if ack.success {
                    println!("Mag cal started — rotate board in all orientations for ~30s");
                } else {
                    println!("Failed: {}", ack.error_code);
                }
            }
            MagCalAction::Clear => {
                let ack =
                    timeout(CMD_TIMEOUT, client.send_resp::<ClearMagCalEndpoint>(&())).await??;
                if ack.success {
                    println!("Mag cal cleared (offsets zeroed)");
                } else {
                    println!("Failed: {}", ack.error_code);
                }
            }
            MagCalAction::Status => {
                let cal =
                    timeout(CMD_TIMEOUT, client.send_resp::<GetMagCalEndpoint>(&())).await??;
                let status = if cal.collecting {
                    format!("COLLECTING ({} samples)", cal.samples)
                } else if cal.calibrated {
                    "CALIBRATED".into()
                } else {
                    "UNCALIBRATED".into()
                };
                println!(
                    "Mag cal: {} | Offsets: X={:.0} Y={:.0} Z={:.0}",
                    status, cal.offset_x, cal.offset_y, cal.offset_z
                );
            }
        },
        DirectCommand::Engine => {
            let e = timeout(CMD_TIMEOUT, client.send_resp::<GetEngineEndpoint>(&())).await??;
            let l_status = if e.left.valid { "OK" } else { "STALE" };
            let r_status = if e.right.valid { "OK" } else { "STALE" };
            let l_target = if e.left.target_erpm > 0 {
                format!(" target:{}", e.left.target_erpm)
            } else {
                String::new()
            };
            let r_target = if e.right.target_erpm > 0 {
                format!(" target:{}", e.right.target_erpm)
            } else {
                String::new()
            };
            println!(
                "Engine L: {} eRPM{} (cmd:{}) [{}] | R: {} eRPM{} (cmd:{}) [{}]",
                e.left.erpm,
                l_target,
                e.left.throttle,
                l_status,
                e.right.erpm,
                r_target,
                e.right.throttle,
                r_status,
            );
            if e.left.voltage_mv > 0 || e.right.voltage_mv > 0 {
                println!(
                    "  EDT L: {:.1}V {:.1}A {}°C | R: {:.1}V {:.1}A {}°C",
                    e.left.voltage_mv as f32 / 1000.0,
                    e.left.current_ma as f32 / 1000.0,
                    e.left.temperature,
                    e.right.voltage_mv as f32 / 1000.0,
                    e.right.current_ma as f32 / 1000.0,
                    e.right.temperature,
                );
            }
        }
    }

    // Signal the RTT worker to stop and wait for it to release the probe
    conn.shutdown
        .store(true, std::sync::atomic::Ordering::Relaxed);
    let _ = conn.worker_handle.join();

    Ok(())
}
