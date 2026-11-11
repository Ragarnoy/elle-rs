//! Direct probe-rs access mode (single commands for scripting)

use std::time::Duration;

use anyhow::Result;
use clap::Subcommand;
use elle_rpc_host::esc;
use elle_rpc_host::link::Link;
use elle_rpc_icd::*;
use postcard_rpc::host_client::HostClient;
use postcard_rpc::standard_icd::WireError;
use tokio::time::timeout;

const CMD_TIMEOUT: Duration = Duration::from_secs(2);
/// Keepalive period while holding the link: well inside the firmware's 200 ms
/// warning / 300 ms failsafe thresholds.
const KEEPALIVE_PERIOD: Duration = Duration::from_millis(100);
/// Consecutive failed keepalives before giving up (the firmware will have
/// failed safe by then anyway).
const KEEPALIVE_MAX_FAILURES: u32 = 5;

/// Keep the host link alive after a command that leaves the aircraft moving,
/// until Ctrl-C; then command throttle 0 and disarm before exiting.
async fn hold_link(client: &HostClient<WireError>) -> Result<()> {
    println!("Holding the link (the firmware fails safe if the host goes quiet).");
    println!("Ctrl-C to stop: sends throttle 0 and disarm.");
    let mut failures = 0;
    let mut ticker = tokio::time::interval(KEEPALIVE_PERIOD);
    loop {
        tokio::select! {
            _ = tokio::signal::ctrl_c() => break,
            _ = ticker.tick() => {
                match timeout(KEEPALIVE_PERIOD * 2, client.send_resp::<PingEndpoint>(&())).await {
                    Ok(Ok(())) => failures = 0,
                    _ => {
                        failures += 1;
                        if failures >= KEEPALIVE_MAX_FAILURES {
                            println!("Link lost: the firmware has failed safe.");
                            return Ok(());
                        }
                    }
                }
            }
        }
    }
    let _ = timeout(
        CMD_TIMEOUT,
        client.send_resp::<SetThrottleEndpoint>(&SetThrottleReq { percent: 0 }),
    )
    .await;
    let _ = timeout(CMD_TIMEOUT, client.send_resp::<DisarmEndpoint>(&())).await;
    println!("Throttle 0, disarmed.");
    Ok(())
}

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
    /// Level calibration (IMU mounting offset)
    LevelCal {
        #[command(subcommand)]
        action: LevelCalAction,
    },
    /// AM32 ESC configuration (RPC builds; see docs/OPERATIONS.md)
    #[command(subcommand)]
    Esc(EscAction),
}

#[derive(Subcommand)]
pub enum EscAction {
    /// Show this boot's mode (normal, or ESC config)
    Mode,
    /// Reboot into ESC-config mode (disarmed, throttle 0), then reattach
    Enter,
    /// Start the ESCs' apps and reboot normally, then reattach
    Exit,
    /// Bootloader handshake; prints the ESC's identity and firmware name
    Connect {
        #[arg(short, long, default_value = "left", value_parser = parse_engine)]
        engine: EscEngine,
    },
    /// Connect and print the decoded settings
    Show {
        #[arg(short, long, default_value = "left", value_parser = parse_engine)]
        engine: EscEngine,
    },
    /// Change settings (name=value ...), read back, print what changed
    Set {
        #[arg(short, long, default_value = "left", value_parser = parse_engine)]
        engine: EscEngine,
        /// e.g. beep_volume=5 timing=15
        #[arg(required = true)]
        settings: Vec<String>,
    },
    /// Save the 256 bytes at the EEPROM base (the esc-config-rs backup format)
    Backup {
        #[arg(short, long, default_value = "left", value_parser = parse_engine)]
        engine: EscEngine,
        /// Output file (default logs/esc/<engine>-<id>-<time>.bin)
        #[arg(short, long)]
        out: Option<std::path::PathBuf>,
    },
    /// Restore a backup (settings, melody, CAN; boot and version bytes kept)
    Restore {
        #[arg(short, long, default_value = "left", value_parser = parse_engine)]
        engine: EscEngine,
        file: std::path::PathBuf,
    },
    /// Leave the bootloader and start the ESC's app
    Run {
        #[arg(short, long, default_value = "left", value_parser = parse_engine)]
        engine: EscEngine,
    },
}

fn parse_engine(s: &str) -> Result<EscEngine, String> {
    esc::parse_engine(s).map_err(|e| e.to_string())
}

fn print_changes(changes: &[esc::Change]) {
    if changes.is_empty() {
        println!("No change: the ESC already had these settings.");
    }
    for c in changes {
        println!("  {}: {} -> {}", c.field, c.before, c.after);
    }
}

/// `elle direct esc ...`. Takes the link: `enter` / `exit` replace it.
async fn run_esc(link: Link, action: EscAction) -> Result<Link> {
    let client = &link.client;
    match action {
        EscAction::Mode => println!("{}", mode_text(esc::mode(client).await?)),
        EscAction::Enter | EscAction::Exit => {
            let entering = matches!(action, EscAction::Enter);
            if entering {
                esc::enter(client).await?;
                println!("Rebooting into ESC-config mode...");
            } else {
                esc::exit(client).await?;
                println!("ESCs started; rebooting normally...");
            }
            let (link, mode) = esc::reconnect(link).await?;
            let want = if entering {
                EscBootMode::EscConfig
            } else {
                EscBootMode::Normal
            };
            if mode != want {
                anyhow::bail!(
                    "the switch did not take: the flight controller is in {} \
                     (the control loop refuses it while armed or with an engine commanded)",
                    mode_text(mode)
                );
            }
            println!("{}", mode_text(mode));
            if entering {
                println!(
                    "The ESCs should now wait in their bootloader (an AM32 app resets \
                     after ~2 s without DShot). Next: elle direct esc connect -e left"
                );
            }
            return Ok(link);
        }
        EscAction::Connect { engine } => {
            let c = esc::connect(client, engine).await?;
            println!("{}", serde_json::to_string_pretty(&esc::info_json(&c))?);
        }
        EscAction::Show { engine } => {
            let c = esc::connect(client, engine).await?;
            let block = esc::read(client, engine).await?;
            println!(
                "{} ESC 0x{:04X} {}",
                esc::engine_name(engine),
                c.info.device_id,
                c.name
            );
            println!("{}", esc::config(&block));
        }
        EscAction::Set { engine, settings } => {
            let edits = esc::parse_edits(&settings)?;
            esc::connect(client, engine).await?;
            let changes = esc::set(client, engine, &edits).await?;
            print_changes(&changes);
        }
        EscAction::Backup { engine, out } => {
            let c = esc::connect(client, engine).await?;
            let data = esc::read_backup(client, engine, &c.info).await?;
            let path = match out {
                Some(p) => p,
                None => {
                    let dir = esc::backup_dir();
                    std::fs::create_dir_all(&dir)?;
                    dir.join(esc::backup_name(engine, &c.info))
                }
            };
            esc::save_backup(&path, &data)?;
            println!("Backup saved to {}", path.display());
        }
        EscAction::Restore { engine, file } => {
            let data = std::fs::read(&file)?;
            esc::connect(client, engine).await?;
            let changes = esc::restore(client, engine, &data).await?;
            print_changes(&changes);
        }
        EscAction::Run { engine } => {
            esc::connect(client, engine).await?;
            esc::run(client, engine).await?;
            println!(
                "Run sent: the {} ESC's app is starting.",
                esc::engine_name(engine)
            );
        }
    }
    Ok(link)
}

fn mode_text(m: EscBootMode) -> &'static str {
    match m {
        EscBootMode::Normal => "normal boot",
        EscBootMode::EscConfig => "ESC CONFIG",
    }
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

#[derive(Subcommand)]
pub enum LevelCalAction {
    /// Start calibration (hold the aircraft still at its reference attitude, ~3s)
    Start,
    /// Clear calibration (attitude uncorrected)
    Clear,
    /// Get calibration status and board offset
    Status,
}

pub async fn run(cmd: DirectCommand) -> Result<()> {
    let link = Link::connect()?;
    if let DirectCommand::Esc(action) = cmd {
        run_esc(link, action).await?.close();
        return Ok(());
    }
    let client = &link.client;

    // Commands that leave something moving. The firmware fails safe ~300 ms
    // after the host goes quiet, so these keep the link alive until Ctrl-C.
    let hold = matches!(
        cmd,
        DirectCommand::Throttle { percent } if percent > 0
    ) || matches!(cmd, DirectCommand::Elevon { .. } | DirectCommand::Arm);

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
                "GNSS: {:.6},{:.6} alt={:.1}m fix={} sats={} (in view {}) hdop={}",
                g.latitude,
                g.longitude,
                g.altitude_m,
                g.fix_quality,
                g.num_satellites,
                g.sats_in_view,
                // NAV-PVT carries no DOP; `hdop` only means something on the
                // GGA fallback path.
                if g.pvt_active {
                    "n/a".to_string()
                } else {
                    format!("{:.1}", g.hdop)
                }
            );
            println!(
                "      spd={:.2}m/s trk={:.1}° vel_ned=({:.2},{:.2},{:.2})m/s",
                g.ground_speed_ms, g.heading_motion_deg, g.vel_n_ms, g.vel_e_ms, g.vel_d_ms
            );
            println!(
                "      hAcc={:.2}m vAcc={:.2}m sAcc={:.2}m/s",
                g.h_acc_m, g.v_acc_m, g.s_acc_ms
            );
            println!(
                "      source={} link={} baud rate={} ms",
                if g.pvt_active {
                    "NAV-PVT"
                } else {
                    "NMEA (fallback)"
                },
                g.link_baud,
                g.nav_rate_ms
            );
            {
                use elle_rpc_icd::{GNSS_CFG_ABANDONED, GNSS_CFG_KEY_NAMES, GNSS_CFG_MASK_ALL};
                let abandoned = g.cfg_mask & GNSS_CFG_ABANDONED != 0;
                let applied = g.cfg_mask & GNSS_CFG_MASK_ALL;
                let missing: Vec<&str> = GNSS_CFG_KEY_NAMES
                    .iter()
                    .enumerate()
                    .filter(|(i, _)| applied & (1 << i) == 0)
                    .map(|(_, n)| *n)
                    .collect();
                if abandoned {
                    // These keys were never sent, so they were not refused —
                    // the fault is the link, not the settings.
                    println!(
                        "      config: ABANDONED after the module stopped answering; NOT APPLIED {}",
                        missing.join(", ")
                    );
                } else if missing.is_empty() {
                    println!(
                        "      config: all {} keys accepted",
                        GNSS_CFG_KEY_NAMES.len()
                    );
                } else {
                    println!("      config: REJECTED {}", missing.join(", "));
                }
            }
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
        DirectCommand::Esc(_) => unreachable!("handled above"),
        DirectCommand::LevelCal { action } => match action {
            LevelCalAction::Start => {
                let ack =
                    timeout(CMD_TIMEOUT, client.send_resp::<StartLevelCalEndpoint>(&())).await??;
                if ack.success {
                    println!(
                        "Level cal started — hold the aircraft still at its reference attitude (~3s)"
                    );
                } else {
                    println!("Failed: {}", ack.error_code);
                }
            }
            LevelCalAction::Clear => {
                let ack =
                    timeout(CMD_TIMEOUT, client.send_resp::<ClearLevelCalEndpoint>(&())).await??;
                if ack.success {
                    println!("Level cal cleared (attitude uncorrected)");
                } else {
                    println!("Failed: {}", ack.error_code);
                }
            }
            LevelCalAction::Status => {
                let cal =
                    timeout(CMD_TIMEOUT, client.send_resp::<GetLevelCalEndpoint>(&())).await??;
                println!("{}", crate::tui::commands::format_level_cal(&cal));
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

    if hold {
        hold_link(client).await?;
    }

    // Stop the RTT worker and release the probe.
    link.close();

    Ok(())
}
