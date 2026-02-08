//! TUI command parsing and execution

use elle_rpc_icd::*;
use postcard_rpc::host_client::HostClient;
use postcard_rpc::standard_icd::WireError;
use std::time::Duration;
use tokio::time::timeout;

use super::state::AppState;

const CMD_TIMEOUT: Duration = Duration::from_secs(2);

pub enum CommandResult {
    Ok(String),
    Err(String),
    Quit,
    Unknown(String),
}

pub async fn execute(
    input: &str,
    client: &HostClient<WireError>,
    state: &mut AppState,
) -> CommandResult {
    let parts: Vec<&str> = input.trim().split_whitespace().collect();
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
        "trim" => {
            if parts.len() < 3 {
                return CommandResult::Err("Usage: trim <left> <right>".into());
            }
            let left = parts[1].parse::<i8>();
            let right = parts[2].parse::<i8>();
            match (left, right) {
                (Ok(l), Ok(r)) if (-100..=100).contains(&l) && (-100..=100).contains(&r) => {
                    cmd_trim(client, l, r).await
                }
                _ => CommandResult::Err("Trim values must be -100 to 100".into()),
            }
        }
        "cal" => {
            if parts.len() < 2 {
                return CommandResult::Err("Usage: cal <save|clear>".into());
            }
            match parts[1].to_lowercase().as_str() {
                "save" => cmd_cal_save(client).await,
                "clear" => cmd_cal_clear(client).await,
                _ => CommandResult::Err("cal subcommand must be: save or clear".into()),
            }
        }
        "mag" => cmd_mag(client).await,
        "gnss" | "gps" => cmd_gnss(client).await,
        "help" | "?" => CommandResult::Ok(
            "Commands: ping status attitude version perf mag gnss arm disarm estop \
             throttle <0-100> elevon <L> <R> mode <manual|mixed|auto> \
             trim <L> <R> cal <save|clear> quit"
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
        Ok(Ok(ack)) if ack.success => {
            CommandResult::Ok(format!("Elevons: L={left} R={right}"))
        }
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

async fn cmd_trim(client: &HostClient<WireError>, left: i8, right: i8) -> CommandResult {
    match timeout(
        CMD_TIMEOUT,
        client.send_resp::<AdjustTrimEndpoint>(&AdjustTrimReq { left, right }),
    )
    .await
    {
        Ok(Ok(ack)) if ack.success => CommandResult::Ok(format!("Trim: L={left} R={right}")),
        Ok(Ok(ack)) => CommandResult::Err(format!("Trim failed (error: {})", ack.error_code)),
        Ok(Err(e)) => CommandResult::Err(format!("Trim failed: {e}")),
        Err(_) => CommandResult::Err("Trim timeout".into()),
    }
}

async fn cmd_cal_save(client: &HostClient<WireError>) -> CommandResult {
    match timeout(
        CMD_TIMEOUT,
        client.send_resp::<SaveCalibrationEndpoint>(&()),
    )
    .await
    {
        Ok(Ok(ack)) if ack.success => CommandResult::Ok("Calibration saved".into()),
        Ok(Ok(ack)) => CommandResult::Err(format!("Cal save failed (error: {})", ack.error_code)),
        Ok(Err(e)) => CommandResult::Err(format!("Cal save failed: {e}")),
        Err(_) => CommandResult::Err("Cal save timeout".into()),
    }
}

async fn cmd_mag(client: &HostClient<WireError>) -> CommandResult {
    match timeout(CMD_TIMEOUT, client.send_resp::<GetMagnetometerEndpoint>(&())).await {
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

async fn cmd_cal_clear(client: &HostClient<WireError>) -> CommandResult {
    match timeout(
        CMD_TIMEOUT,
        client.send_resp::<ClearCalibrationEndpoint>(&()),
    )
    .await
    {
        Ok(Ok(ack)) if ack.success => CommandResult::Ok("Calibration cleared".into()),
        Ok(Ok(ack)) => {
            CommandResult::Err(format!("Cal clear failed (error: {})", ack.error_code))
        }
        Ok(Err(e)) => CommandResult::Err(format!("Cal clear failed: {e}")),
        Err(_) => CommandResult::Err("Cal clear timeout".into()),
    }
}
