//! TUI command parsing and execution

use elle_rpc_icd::*;
use postcard_rpc::host_client::HostClient;
use postcard_rpc::standard_icd::WireError;
use std::time::Duration;
use tokio::io::AsyncWriteExt;
use tokio::time::timeout;

use super::state::AppState;

const CMD_TIMEOUT: Duration = Duration::from_secs(2);

/// Common handler for RPC commands that return `AckResp`.
fn handle_ack(
    result: Result<Result<AckResp, impl std::fmt::Display>, tokio::time::error::Elapsed>,
    name: &str,
    success_msg: String,
) -> CommandResult {
    match result {
        Ok(Ok(ack)) if ack.success => CommandResult::Ok(success_msg),
        Ok(Ok(ack)) => CommandResult::Err(format!("{name} failed (error: {})", ack.error_code)),
        Ok(Err(e)) => CommandResult::Err(format!("{name} failed: {e}")),
        Err(_) => CommandResult::Err(format!("{name} timeout")),
    }
}

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
        "pid" => {
            if parts.len() < 9 {
                return CommandResult::Err(
                    "Usage: pid <Pkp> <Pki> <Pkd> <Rkp> <Rki> <Rkd> <scale> <ilimit>".into(),
                );
            }
            #[allow(clippy::type_complexity)]
            let parse = || -> Option<(f32, f32, f32, f32, f32, f32, f32, f32)> {
                Some((
                    parts[1].parse().ok()?,
                    parts[2].parse().ok()?,
                    parts[3].parse().ok()?,
                    parts[4].parse().ok()?,
                    parts[5].parse().ok()?,
                    parts[6].parse().ok()?,
                    parts[7].parse().ok()?,
                    parts[8].parse().ok()?,
                ))
            };
            match parse() {
                Some((pkp, pki, pkd, rkp, rki, rkd, scale, ilimit)) => {
                    cmd_set_pid_gains(client, pkp, pki, pkd, rkp, rki, rkd, scale, ilimit).await
                }
                None => CommandResult::Err("All PID arguments must be valid floats".into()),
            }
        }
        "setpoint" | "sp" => {
            if parts.len() < 3 {
                return CommandResult::Err("Usage: setpoint <pitch_deg> <roll_deg>".into());
            }
            let pitch: Result<f32, _> = parts[1].parse();
            let roll: Result<f32, _> = parts[2].parse();
            match (pitch, roll) {
                (Ok(p), Ok(r)) => cmd_set_attitude_setpoint(client, p, r).await,
                _ => CommandResult::Err("Setpoint values must be valid floats".into()),
            }
        }
        "time" => cmd_time(client).await,
        "autotune" | "atune" => {
            if parts.len() < 2 {
                return CommandResult::Err(
                    "Usage: autotune <pitch|roll> [relay_deg] [cycles] [tl|zn|so] | autotune abort"
                        .into(),
                );
            }
            match parts[1].to_lowercase().as_str() {
                "abort" | "stop" => cmd_autotune_abort(client).await,
                "pitch" | "roll" => {
                    let axis: u8 = if parts[1].to_lowercase() == "pitch" {
                        0
                    } else {
                        1
                    };
                    let relay_deg: f32 = parts
                        .get(2)
                        .and_then(|s| s.parse().ok())
                        .unwrap_or(5.0);
                    let cycles: u8 = parts
                        .get(3)
                        .and_then(|s| s.parse().ok())
                        .unwrap_or(6);
                    let rule: u8 = match parts.get(4).map(|s| s.to_lowercase()).as_deref() {
                        Some("zn") => 1,
                        Some("so") => 2,
                        _ => 0, // TyreusLuyben default
                    };
                    cmd_autotune_start(client, axis, relay_deg, cycles, rule).await
                }
                _ => CommandResult::Err(
                    "autotune subcommand must be: pitch, roll, or abort".into(),
                ),
            }
        }
        "savepid" => cmd_savepid(client).await,
        "mag" => {
            if parts.len() >= 2 && parts[1].to_lowercase() == "cal" {
                if parts.len() >= 3 {
                    match parts[2].to_lowercase().as_str() {
                        "start" => cmd_mag_cal_start(client).await,
                        "clear" => cmd_mag_cal_clear(client).await,
                        _ => CommandResult::Err("Usage: mag cal [start|clear]".into()),
                    }
                } else {
                    cmd_mag_cal_status(client).await
                }
            } else {
                CommandResult::Err("Usage: mag cal [start|clear]".into())
            }
        }
        "help" | "?" => CommandResult::Ok(
            "query: ping perf time | \
             safety: arm disarm estop | \
             ctrl: thr <0-100> elv <L> <R> mode <man|mix|auto> pid <8 floats> sp <P> <R> | \
             autotune: autotune <pitch|roll> [deg] [cycles] [tl|zn|so] | autotune abort | \
             savepid | ulog: ulog <info|start|stop|extract|erase> | \
             mag cal [start|clear] | quit"
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
    handle_ack(
        timeout(CMD_TIMEOUT, client.send_resp::<ArmEndpoint>(&())).await,
        "Arm", "ARMED".into(),
    )
}

async fn cmd_disarm(client: &HostClient<WireError>) -> CommandResult {
    handle_ack(
        timeout(CMD_TIMEOUT, client.send_resp::<DisarmEndpoint>(&())).await,
        "Disarm", "DISARMED".into(),
    )
}

async fn cmd_estop(client: &HostClient<WireError>) -> CommandResult {
    handle_ack(
        timeout(CMD_TIMEOUT, client.send_resp::<EmergencyStopEndpoint>(&())).await,
        "E-Stop", "EMERGENCY STOP EXECUTED".into(),
    )
}

async fn cmd_throttle(client: &HostClient<WireError>, percent: u8) -> CommandResult {
    handle_ack(
        timeout(CMD_TIMEOUT, client.send_resp::<SetThrottleEndpoint>(&SetThrottleReq { percent })).await,
        "Throttle", format!("Throttle: {percent}%"),
    )
}

async fn cmd_elevon(client: &HostClient<WireError>, left: i8, right: i8) -> CommandResult {
    handle_ack(
        timeout(CMD_TIMEOUT, client.send_resp::<SetElevonsEndpoint>(&SetElevonsReq { left, right })).await,
        "Elevon", format!("Elevons: L={left} R={right}"),
    )
}

async fn cmd_mode(client: &HostClient<WireError>, mode: ControlMode) -> CommandResult {
    handle_ack(
        timeout(CMD_TIMEOUT, client.send_resp::<SetControlModeEndpoint>(&SetControlModeReq { mode })).await,
        "Mode", format!("Mode: {mode:?}"),
    )
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

#[allow(clippy::too_many_arguments)]
async fn cmd_set_pid_gains(
    client: &HostClient<WireError>,
    pkp: f32,
    pki: f32,
    pkd: f32,
    rkp: f32,
    rki: f32,
    rkd: f32,
    scale: f32,
    ilimit: f32,
) -> CommandResult {
    let req = SetPidGainsReq {
        pitch_kp_x1000: (pkp * 1000.0) as i16,
        pitch_ki_x1000: (pki * 1000.0) as i16,
        pitch_kd_x1000: (pkd * 1000.0) as i16,
        roll_kp_x1000: (rkp * 1000.0) as i16,
        roll_ki_x1000: (rki * 1000.0) as i16,
        roll_kd_x1000: (rkd * 1000.0) as i16,
        scale_x10000: (scale * 10000.0) as i16,
        i_limit_x10: (ilimit * 10.0) as i16,
    };
    handle_ack(
        timeout(CMD_TIMEOUT, client.send_resp::<SetPidGainsEndpoint>(&req)).await,
        "PID set",
        format!("PID: P({pkp}/{pki}/{pkd}) R({rkp}/{rki}/{rkd}) scale={scale} ilim={ilimit}"),
    )
}

async fn cmd_set_attitude_setpoint(
    client: &HostClient<WireError>,
    pitch_deg: f32,
    roll_deg: f32,
) -> CommandResult {
    let req = SetAttitudeSetpointReq {
        pitch_cdeg: (pitch_deg * 100.0) as i16,
        roll_cdeg: (roll_deg * 100.0) as i16,
    };
    handle_ack(
        timeout(CMD_TIMEOUT, client.send_resp::<SetAttitudeSetpointEndpoint>(&req)).await,
        "Setpoint",
        format!("Setpoint: P={pitch_deg}° R={roll_deg}°"),
    )
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
    handle_ack(
        timeout(CMD_TIMEOUT, client.send_resp::<StartULogEndpoint>(&())).await,
        "ULog start", "ULog recording started".into(),
    )
}

async fn cmd_ulog_stop(client: &HostClient<WireError>) -> CommandResult {
    handle_ack(
        timeout(CMD_TIMEOUT, client.send_resp::<StopULogEndpoint>(&())).await,
        "ULog stop", "ULog recording stopped".into(),
    )
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
    const ERASE_TIMEOUT: Duration = Duration::from_secs(30);
    handle_ack(
        timeout(ERASE_TIMEOUT, client.send_resp::<EraseULogEndpoint>(&())).await,
        "ULog erase", "ULog flash erased".into(),
    )
}

async fn cmd_autotune_start(
    client: &HostClient<WireError>,
    axis: u8,
    relay_deg: f32,
    cycles: u8,
    rule: u8,
) -> CommandResult {
    let relay_deg_x10 = (relay_deg * 10.0).clamp(1.0, 255.0) as u8;
    let req = StartAutotuneReq {
        axis,
        relay_deg_x10,
        num_cycles: cycles,
        rule,
    };
    let axis_name = if axis == 0 { "pitch" } else { "roll" };
    let rule_name = match rule {
        1 => "ZieglerNichols",
        2 => "SomeOvershoot",
        _ => "TyreusLuyben",
    };
    handle_ack(
        timeout(CMD_TIMEOUT, client.send_resp::<StartAutotuneEndpoint>(&req)).await,
        "Autotune start",
        format!("Autotune started: {axis_name} relay={relay_deg}° cycles={cycles} rule={rule_name}"),
    )
}

async fn cmd_autotune_abort(client: &HostClient<WireError>) -> CommandResult {
    handle_ack(
        timeout(CMD_TIMEOUT, client.send_resp::<AbortAutotuneEndpoint>(&())).await,
        "Autotune abort", "Autotune aborted".into(),
    )
}

async fn cmd_savepid(client: &HostClient<WireError>) -> CommandResult {
    let req = StartAutotuneReq {
        axis: 0xFF, // Magic value = save current PID gains to flash
        relay_deg_x10: 0,
        num_cycles: 0,
        rule: 0,
    };
    handle_ack(
        timeout(CMD_TIMEOUT, client.send_resp::<StartAutotuneEndpoint>(&req)).await,
        "PID save", "PID gains save requested".into(),
    )
}

async fn cmd_mag_cal_start(client: &HostClient<WireError>) -> CommandResult {
    handle_ack(
        timeout(CMD_TIMEOUT, client.send_resp::<StartMagCalEndpoint>(&())).await,
        "Mag cal start",
        "Mag cal started — rotate board in all orientations for ~30s".into(),
    )
}

async fn cmd_mag_cal_clear(client: &HostClient<WireError>) -> CommandResult {
    handle_ack(
        timeout(CMD_TIMEOUT, client.send_resp::<ClearMagCalEndpoint>(&())).await,
        "Mag cal clear", "Mag cal cleared (offsets zeroed)".into(),
    )
}

async fn cmd_mag_cal_status(client: &HostClient<WireError>) -> CommandResult {
    match timeout(CMD_TIMEOUT, client.send_resp::<GetMagCalEndpoint>(&())).await {
        Ok(Ok(cal)) => {
            let status = if cal.collecting {
                format!("COLLECTING ({} samples)", cal.samples)
            } else if cal.calibrated {
                "CALIBRATED".into()
            } else {
                "UNCALIBRATED".into()
            };
            CommandResult::Ok(format!(
                "Mag cal: {} | Offsets: X={:.0} Y={:.0} Z={:.0}",
                status, cal.offset_x, cal.offset_y, cal.offset_z
            ))
        }
        Ok(Err(e)) => CommandResult::Err(format!("Mag cal status failed: {e}")),
        Err(_) => CommandResult::Err("Mag cal status timeout".into()),
    }
}
