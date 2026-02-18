//! TUI command parsing and execution

use elle_rpc_icd::*;
use postcard_rpc::host_client::HostClient;
use postcard_rpc::standard_icd::WireError;
use std::time::Duration;
use tokio::io::AsyncWriteExt;
use tokio::time::timeout;

use super::state::AppState;

const CMD_TIMEOUT: Duration = Duration::from_secs(2);

pub enum CommandResult {
    Ok(String),
    Err(String),
    Quit,
    Unknown(String),
    /// Long-running task spawned in background (status message, join handle)
    Background(String, tokio::task::JoinHandle<String>),
}

pub async fn execute(
    input: &str,
    client: &HostClient<WireError>,
    state: &mut AppState,
) -> CommandResult {
    let parts: Vec<&str> = input.split_whitespace().collect();
    if parts.is_empty() {
        return CommandResult::Ok(String::new());
    }

    let cmd = parts[0].to_lowercase();
    match cmd.as_str() {
        "q" | "quit" | "exit" => CommandResult::Quit,
        "ping" => cmd_ping(client).await,
        "status" => cmd_status(client, state).await,
        "attitude" | "att" => cmd_attitude(client, state).await,
        "version" | "ver" => cmd_version(client, state).await,
        "perf" => cmd_perf(client, state).await,
        "arm" => cmd_arm(client).await,
        "disarm" => cmd_disarm(client).await,
        "estop" | "stop" => cmd_estop(client).await,
        "throttle" | "thr" => {
            if parts.len() < 2 {
                return CommandResult::Err("Usage: throttle <0-100>".into());
            }
            match parts[1].parse::<u8>() {
                Ok(p) if p <= 100 => cmd_throttle(client, p).await,
                _ => CommandResult::Err("Throttle must be 0-100".into()),
            }
        }
        "elevon" | "elv" => {
            if parts.len() < 3 {
                return CommandResult::Err("Usage: elevon <left> <right>".into());
            }
            let left = parts[1].parse::<i8>();
            let right = parts[2].parse::<i8>();
            match (left, right) {
                (Ok(l), Ok(r)) if (-100..=100).contains(&l) && (-100..=100).contains(&r) => {
                    cmd_elevon(client, l, r).await
                }
                _ => CommandResult::Err("Elevon values must be -100 to 100".into()),
            }
        }
        "mode" => {
            if parts.len() < 2 {
                return CommandResult::Err("Usage: mode <manual|mixed|auto>".into());
            }
            match parts[1].to_lowercase().as_str() {
                "manual" | "man" => cmd_mode(client, ControlMode::Manual).await,
                "mixed" | "mix" => cmd_mode(client, ControlMode::Mixed).await,
                "auto" | "autopilot" => cmd_mode(client, ControlMode::Autopilot).await,
                _ => CommandResult::Err("Mode must be: manual, mixed, or auto".into()),
            }
        }
        "ulog" => {
            if parts.len() < 2 {
                return CommandResult::Err(
                    "Usage: ulog <info|start|stop|extract [file]|erase>".into(),
                );
            }
            match parts[1].to_lowercase().as_str() {
                "info" => cmd_ulog_info(client).await,
                "start" => cmd_ulog_start(client).await,
                "stop" => cmd_ulog_stop(client).await,
                "extract" => {
                    let filename = parts.get(2).map(|s| s.to_string()).unwrap_or_else(|| {
                        format!(
                            "flight_{}.ulg",
                            chrono::Local::now().format("%Y%m%d_%H%M%S")
                        )
                    });
                    let client_clone = client.clone();
                    let fname = filename.clone();
                    let handle =
                        tokio::spawn(
                            async move { cmd_ulog_extract_bg(&client_clone, &fname).await },
                        );
                    CommandResult::Background(format!("Extracting ULog to {filename}..."), handle)
                }
                "erase" => cmd_ulog_erase(client).await,
                _ => CommandResult::Err(
                    "ulog subcommand must be: info, start, stop, extract, or erase".into(),
                ),
            }
        }
        "time" => cmd_time(client).await,
        "mag" => cmd_mag(client).await,
        "gnss" | "gps" => cmd_gnss(client).await,
        "help" | "?" => CommandResult::Ok(
            "Commands: ping status attitude version perf time mag gnss arm disarm estop \
             throttle <0-100> elevon <L> <R> mode <manual|mixed|auto> \
             ulog <info|start|stop|extract [file]|erase> quit"
                .into(),
        ),
        other => CommandResult::Unknown(other.into()),
    }
}

async fn cmd_ping(client: &HostClient<WireError>) -> CommandResult {
    match timeout(CMD_TIMEOUT, client.send_resp::<PingEndpoint>(&())).await {
        Ok(Ok(_)) => CommandResult::Ok("Pong!".into()),
        Ok(Err(e)) => CommandResult::Err(format!("Ping failed: {e}")),
        Err(_) => CommandResult::Err("Ping timeout".into()),
    }
}

async fn cmd_status(client: &HostClient<WireError>, state: &mut AppState) -> CommandResult {
    match timeout(CMD_TIMEOUT, client.send_resp::<GetStatusEndpoint>(&())).await {
        Ok(Ok(s)) => {
            state.status = Some(s);
            CommandResult::Ok("Status updated".into())
        }
        Ok(Err(e)) => CommandResult::Err(format!("Status failed: {e}")),
        Err(_) => CommandResult::Err("Status timeout".into()),
    }
}

async fn cmd_attitude(client: &HostClient<WireError>, _state: &mut AppState) -> CommandResult {
    match timeout(CMD_TIMEOUT, client.send_resp::<GetAttitudeEndpoint>(&())).await {
        Ok(Ok(a)) => {
            let msg = format!(
                "P:{:.1} R:{:.1} Y:{:.1}",
                a.pitch_cdeg as f32 / 100.0,
                a.roll_cdeg as f32 / 100.0,
                a.yaw_cdeg as f32 / 100.0,
            );
            CommandResult::Ok(msg)
        }
        Ok(Err(e)) => CommandResult::Err(format!("Attitude failed: {e}")),
        Err(_) => CommandResult::Err("Attitude timeout".into()),
    }
}

async fn cmd_version(client: &HostClient<WireError>, state: &mut AppState) -> CommandResult {
    match timeout(CMD_TIMEOUT, client.send_resp::<GetVersionEndpoint>(&())).await {
        Ok(Ok(v)) => {
            state.version = Some(v);
            CommandResult::Ok(format!("Firmware v{}.{}.{}", v.major, v.minor, v.patch))
        }
        Ok(Err(e)) => CommandResult::Err(format!("Version failed: {e}")),
        Err(_) => CommandResult::Err("Version timeout".into()),
    }
}

async fn cmd_perf(client: &HostClient<WireError>, state: &mut AppState) -> CommandResult {
    match timeout(CMD_TIMEOUT, client.send_resp::<GetPerformanceEndpoint>(&())).await {
        Ok(Ok(p)) => {
            state.performance = Some(p);
            CommandResult::Ok(format!(
                "Loop: {}us avg / {}us max | IMU: {}us avg",
                p.control_loop_avg_us, p.control_loop_max_us, p.imu_avg_us,
            ))
        }
        Ok(Err(e)) => CommandResult::Err(format!("Perf failed: {e}")),
        Err(_) => CommandResult::Err("Perf timeout".into()),
    }
}

async fn cmd_arm(client: &HostClient<WireError>) -> CommandResult {
    match timeout(CMD_TIMEOUT, client.send_resp::<ArmEndpoint>(&())).await {
        Ok(Ok(ack)) if ack.success => CommandResult::Ok("ARMED".into()),
        Ok(Ok(ack)) => CommandResult::Err(format!("Arm failed (error: {})", ack.error_code)),
        Ok(Err(e)) => CommandResult::Err(format!("Arm failed: {e}")),
        Err(_) => CommandResult::Err("Arm timeout".into()),
    }
}

async fn cmd_disarm(client: &HostClient<WireError>) -> CommandResult {
    match timeout(CMD_TIMEOUT, client.send_resp::<DisarmEndpoint>(&())).await {
        Ok(Ok(ack)) if ack.success => CommandResult::Ok("DISARMED".into()),
        Ok(Ok(ack)) => CommandResult::Err(format!("Disarm failed (error: {})", ack.error_code)),
        Ok(Err(e)) => CommandResult::Err(format!("Disarm failed: {e}")),
        Err(_) => CommandResult::Err("Disarm timeout".into()),
    }
}

async fn cmd_estop(client: &HostClient<WireError>) -> CommandResult {
    match timeout(CMD_TIMEOUT, client.send_resp::<EmergencyStopEndpoint>(&())).await {
        Ok(Ok(ack)) if ack.success => CommandResult::Ok("EMERGENCY STOP EXECUTED".into()),
        Ok(Ok(ack)) => CommandResult::Err(format!("E-Stop failed (error: {})", ack.error_code)),
        Ok(Err(e)) => CommandResult::Err(format!("E-Stop failed: {e}")),
        Err(_) => CommandResult::Err("E-Stop timeout".into()),
    }
}

async fn cmd_throttle(client: &HostClient<WireError>, percent: u8) -> CommandResult {
    match timeout(
        CMD_TIMEOUT,
        client.send_resp::<SetThrottleEndpoint>(&SetThrottleReq { percent }),
    )
    .await
    {
        Ok(Ok(ack)) if ack.success => CommandResult::Ok(format!("Throttle: {percent}%")),
        Ok(Ok(ack)) => CommandResult::Err(format!("Throttle failed (error: {})", ack.error_code)),
        Ok(Err(e)) => CommandResult::Err(format!("Throttle failed: {e}")),
        Err(_) => CommandResult::Err("Throttle timeout".into()),
    }
}

async fn cmd_elevon(client: &HostClient<WireError>, left: i8, right: i8) -> CommandResult {
    match timeout(
        CMD_TIMEOUT,
        client.send_resp::<SetElevonsEndpoint>(&SetElevonsReq { left, right }),
    )
    .await
    {
        Ok(Ok(ack)) if ack.success => CommandResult::Ok(format!("Elevons: L={left} R={right}")),
        Ok(Ok(ack)) => CommandResult::Err(format!("Elevon failed (error: {})", ack.error_code)),
        Ok(Err(e)) => CommandResult::Err(format!("Elevon failed: {e}")),
        Err(_) => CommandResult::Err("Elevon timeout".into()),
    }
}

async fn cmd_mode(client: &HostClient<WireError>, mode: ControlMode) -> CommandResult {
    match timeout(
        CMD_TIMEOUT,
        client.send_resp::<SetControlModeEndpoint>(&SetControlModeReq { mode }),
    )
    .await
    {
        Ok(Ok(ack)) if ack.success => CommandResult::Ok(format!("Mode: {mode:?}")),
        Ok(Ok(ack)) => CommandResult::Err(format!("Mode failed (error: {})", ack.error_code)),
        Ok(Err(e)) => CommandResult::Err(format!("Mode failed: {e}")),
        Err(_) => CommandResult::Err("Mode timeout".into()),
    }
}


async fn cmd_time(client: &HostClient<WireError>) -> CommandResult {
    match timeout(CMD_TIMEOUT, client.send_resp::<GetTimeEndpoint>(&())).await {
        Ok(Ok(ms)) => {
            let dt = chrono::DateTime::from_timestamp_millis(ms as i64);
            let time_str = match dt {
                Some(dt) => dt.format("%Y-%m-%d %H:%M:%S UTC").to_string(),
                None => "invalid".into(),
            };
            CommandResult::Ok(format!("{time_str} ({ms})"))
        }
        Ok(Err(e)) => CommandResult::Err(format!("Time failed: {e}")),
        Err(_) => CommandResult::Err("Time timeout".into()),
    }
}

async fn cmd_mag(client: &HostClient<WireError>) -> CommandResult {
    match timeout(
        CMD_TIMEOUT,
        client.send_resp::<GetMagnetometerEndpoint>(&()),
    )
    .await
    {
        Ok(Ok(m)) => CommandResult::Ok(format!("Mag: X={} Y={} Z={}", m.x, m.y, m.z)),
        Ok(Err(e)) => CommandResult::Err(format!("Mag failed: {e}")),
        Err(_) => CommandResult::Err("Mag timeout".into()),
    }
}

async fn cmd_gnss(client: &HostClient<WireError>) -> CommandResult {
    match timeout(CMD_TIMEOUT, client.send_resp::<GetGnssEndpoint>(&())).await {
        Ok(Ok(g)) => CommandResult::Ok(format!(
            "GNSS: {:.6},{:.6} alt={:.1}m fix={} sats={} hdop={:.1}",
            g.latitude, g.longitude, g.altitude_m, g.fix_quality, g.num_satellites, g.hdop
        )),
        Ok(Err(e)) => CommandResult::Err(format!("GNSS failed: {e}")),
        Err(_) => CommandResult::Err("GNSS timeout".into()),
    }
}


async fn cmd_ulog_info(client: &HostClient<WireError>) -> CommandResult {
    match timeout(CMD_TIMEOUT, client.send_resp::<GetULogInfoEndpoint>(&())).await {
        Ok(Ok(info)) => {
            let region_mb = info.region_total as f64 / (1024.0 * 1024.0);
            let used_kb = info.bytes_used as f64 / 1024.0;
            let remaining_mb =
                (info.region_total.saturating_sub(info.bytes_used)) as f64 / (1024.0 * 1024.0);
            let pct = if info.region_total > 0 {
                info.bytes_used as f64 / info.region_total as f64 * 100.0
            } else {
                0.0
            };
            let rec = if info.recording { "YES" } else { "no" };
            CommandResult::Ok(format!(
                "ULog: rec={rec} | {used_kb:.1} KB used ({pct:.1}%) | {remaining_mb:.1}/{region_mb:.1} MB free | {items} items",
                items = info.items_stored,
            ))
        }
        Ok(Err(e)) => CommandResult::Err(format!("ULog info failed: {e}")),
        Err(_) => CommandResult::Err("ULog info timeout".into()),
    }
}

async fn cmd_ulog_start(client: &HostClient<WireError>) -> CommandResult {
    match timeout(CMD_TIMEOUT, client.send_resp::<StartULogEndpoint>(&())).await {
        Ok(Ok(ack)) if ack.success => CommandResult::Ok("ULog recording started".into()),
        Ok(Ok(ack)) => CommandResult::Err(format!("ULog start failed (error: {})", ack.error_code)),
        Ok(Err(e)) => CommandResult::Err(format!("ULog start failed: {e}")),
        Err(_) => CommandResult::Err("ULog start timeout".into()),
    }
}

async fn cmd_ulog_stop(client: &HostClient<WireError>) -> CommandResult {
    match timeout(CMD_TIMEOUT, client.send_resp::<StopULogEndpoint>(&())).await {
        Ok(Ok(ack)) if ack.success => CommandResult::Ok("ULog recording stopped".into()),
        Ok(Ok(ack)) => CommandResult::Err(format!("ULog stop failed (error: {})", ack.error_code)),
        Ok(Err(e)) => CommandResult::Err(format!("ULog stop failed: {e}")),
        Err(_) => CommandResult::Err("ULog stop timeout".into()),
    }
}

/// Background ULog extraction — runs in a spawned tokio task, returns a status string.
async fn cmd_ulog_extract_bg(client: &HostClient<WireError>, filename: &str) -> String {
    const POLL_TIMEOUT: Duration = Duration::from_secs(10);
    const RETRY_DELAY: Duration = Duration::from_millis(50); // Flash ops complete in <20ms
    const MAX_RETRIES: u32 = 200; // 10 seconds max wait for pending (200 × 50ms)

    // Stop recording first to avoid signal contention on FLASH_REQUEST_SIGNAL
    let _ = timeout(CMD_TIMEOUT, client.send_resp::<StopULogEndpoint>(&())).await;
    // Brief delay to let firmware flush its buffer
    tokio::time::sleep(Duration::from_millis(500)).await;

    let mut file = match tokio::fs::File::create(filename).await {
        Ok(f) => f,
        Err(e) => return format!("ERROR: Failed to create file: {e}"),
    };

    let mut total_bytes: u64 = 0;
    let mut chunk_count: u32 = 0;
    let start_time = std::time::Instant::now();

    loop {
        // Poll with retries for pending/timeout states
        let mut retries = 0;
        let mut rpc_timeouts = 0u32;
        let resp = loop {
            match timeout(POLL_TIMEOUT, client.send_resp::<ReadULogChunkEndpoint>(&())).await {
                Ok(Ok(resp)) => {
                    if resp.pending {
                        retries += 1;
                        if retries >= MAX_RETRIES {
                            if total_bytes > 0 {
                                // Got some data before stalling — finish gracefully
                                break None;
                            }
                            return "ERROR: ULog read timed out (device not responding)".into();
                        }
                        tokio::time::sleep(RETRY_DELAY).await;
                        continue;
                    }
                    break Some(resp);
                }
                Ok(Err(e)) => return format!("ERROR: ULog read failed: {e}"),
                Err(_) => {
                    // RPC timeout — retry a few times before giving up
                    rpc_timeouts += 1;
                    if rpc_timeouts >= 3 {
                        if total_bytes > 0 {
                            break None; // finish with what we have
                        }
                        return "ERROR: ULog read timeout (no RPC response)".into();
                    }
                    tokio::time::sleep(RETRY_DELAY).await;
                    continue;
                }
            }
        };

        let Some(resp) = resp else { break };

        if !resp.data.is_empty() {
            if let Err(e) = file.write_all(&resp.data).await {
                return format!("ERROR: File write failed: {e}");
            }
            total_bytes += resp.data.len() as u64;
            chunk_count += 1;
        }

        if !resp.has_more {
            break;
        }
    }

    if let Err(e) = file.flush().await {
        return format!("ERROR: File flush failed: {e}");
    }

    let elapsed = start_time.elapsed();

    if total_bytes == 0 {
        let _ = tokio::fs::remove_file(filename).await;
        "No ULog data on device".into()
    } else {
        let rate_kbs = total_bytes as f64 / elapsed.as_secs_f64() / 1024.0;
        format!(
            "Extracted {} bytes ({} chunks) to {} in {:.1}s ({:.1} KB/s)",
            total_bytes,
            chunk_count,
            filename,
            elapsed.as_secs_f64(),
            rate_kbs
        )
    }
}

async fn cmd_ulog_erase(client: &HostClient<WireError>) -> CommandResult {
    // Erase can take a while (~14MB of flash), use longer timeout
    const ERASE_TIMEOUT: Duration = Duration::from_secs(30);

    match timeout(ERASE_TIMEOUT, client.send_resp::<EraseULogEndpoint>(&())).await {
        Ok(Ok(ack)) if ack.success => CommandResult::Ok("ULog flash erased".into()),
        Ok(Ok(ack)) => CommandResult::Err(format!("ULog erase failed (error: {})", ack.error_code)),
        Ok(Err(e)) => CommandResult::Err(format!("ULog erase failed: {e}")),
        Err(_) => CommandResult::Err("ULog erase timeout (30s)".into()),
    }
}
