//! RPC mode: the ground-test control loop driven by postcard-RPC commands from
//! the host (`rpc-control`), or by the RC receiver with the host monitoring
//! (`rpc-rc`).

use crate::engines::publish_engine_output;
use crate::logging::log_flight_data;
use crate::support::{
    erase_pid_from_flash, save_pid_to_flash, tap_cal_allowed, tap_selects_level_cal,
    validate_attitude,
};
use crate::{flight_state, rc_signal, rpc_app, rpc_handlers};

#[cfg(feature = "rpc-rc")]
use defmt::debug;
use defmt::info;
use elle_config::*;
use elle_control::autotune::{AutotuneAction, AutotuneAxis, Autotuner};
#[cfg(feature = "rpc-rc")]
use elle_control::commands::PilotCommands;
use elle_hardware::ULogLogger;
use elle_hardware::crsf::RC_COMMANDS;
use elle_hardware::dshot::DSHOT_THROTTLE;
use elle_hardware::imu::{ATTITUDE, IMU_STATUS, LED_COMMAND_CHANNEL};
use elle_hardware::led::{LedPattern, colors};
use elle_rpc_icd::ControlMode;
use elle_system::{
    FlightController, TimingMeasurement, log_performance_summary, update_control_loop_timing,
};
use embassy_time::{Duration, Instant, Ticker, Timer};

/// Run the RPC-mode control loop forever. `epoch_ms` is the wall-clock time at
/// boot (ms since the UNIX epoch), used to stamp new ULog files.
///
/// The controller is borrowed, not moved: a value moved into an async fn keeps
/// its storage in every enclosing future too. The ULog logger (~6 KB of buffers)
/// is created here for the same reason. It starts uninitialised: recording
/// starts on `ulog start`.
pub(crate) async fn run_rpc(fc: &mut FlightController<'static>, epoch_ms: u64) -> ! {
    let mut ulog_logger = ULogLogger::new();
    let mut loop_counter = 0u32;

    use core::sync::atomic::Ordering;
    use elle_config::profile::{FlashRequest, FlashResponse};
    #[cfg(not(feature = "rpc-rc"))]
    use elle_control::commands::AttitudeMode;
    use elle_control::commands::NormalizedCommands;
    #[cfg(not(feature = "rpc-rc"))]
    use elle_control::commands::PilotCommands;
    use elle_hardware::flash::{FLASH_REQUEST_SIGNAL, FLASH_RESPONSE_SIGNAL};
    use rpc_app::{
        ULOG_ENABLED, ULOG_ITEM_LEN, ULOG_ITEM_SIGNAL, ULOG_OFFSET, ULOG_STATE, ULogState,
    };
    use rpc_handlers::{RPC_CMD_CHANNEL, RpcCommand};

    #[cfg(feature = "rpc-rc")]
    info!("RPC MONITORING MODE - RC/CRSF flight control + RPC observability");
    #[cfg(not(feature = "rpc-rc"))]
    info!("GROUND TEST MODE - RPC Control (postcard-RPC over RTT)");
    info!("WARNING: This mode requires programmer connection");

    // RPC mode without RC: arming is via explicit arm/disarm commands only
    // With RC: use RC-based arming (throttle-low auto-arm)
    #[cfg(not(feature = "rpc-rc"))]
    fc.set_explicit_arming(true);

    // Ticker for consistent control loop timing
    let mut ticker = Ticker::every(Duration::from_millis(CONTROL_LOOP_PERIOD_MS));

    // RPC commands accumulator (updated by RPC handlers, unused when rc feature is active)
    #[cfg(not(feature = "rpc-rc"))]
    let mut rpc_throttle: f32 = 0.0;
    #[cfg(not(feature = "rpc-rc"))]
    let mut rpc_elevon_left: f32 = 0.0;
    #[cfg(not(feature = "rpc-rc"))]
    let mut rpc_elevon_right: f32 = 0.0;
    #[cfg(not(feature = "rpc-rc"))]
    let mut rpc_mode = AttitudeMode::Manual;

    // RC mode: track last commands from CRSF receiver
    #[cfg(feature = "rpc-rc")]
    let mut last_commands: Option<PilotCommands> = None;

    // Autotuner state (same pattern as flight mode)
    let mut autotuner = Autotuner::new();
    let mut autotune_tick: u32 = 0;
    let mut rpc_save_pending: Option<[u8; 32]> = None;
    let mut was_armed = false;
    let mut was_killed = false;
    let mut autotune_display = elle_hardware::crsf::AutotuneDisplay::Off;
    let mut autotune_display_timer: u32 = 0;
    const AUTOTUNE_DISPLAY_DURATION: u32 = CONTROL_LOOP_FREQUENCY_HZ * 3; // ~3 s

    loop {
        ticker.next().await;
        let loop_start = Instant::now();
        let loop_timer = TimingMeasurement::start();

        // Supervisor check
        let _supervisor_healthy = fc.supervisor_check();

        // Get latest attitude data
        let attitude = ATTITUDE.try_take();

        // Process all pending RPC commands
        while let Ok(cmd) = RPC_CMD_CHANNEL.try_receive() {
            match cmd {
                #[cfg(not(feature = "rpc-rc"))]
                RpcCommand::SetThrottle(percent) => {
                    rpc_throttle = (percent as f32 / 100.0).clamp(0.0, 1.0);
                }
                #[cfg(feature = "rpc-rc")]
                RpcCommand::SetThrottle(_) => {} // Ignored in RC mode
                #[cfg(not(feature = "rpc-rc"))]
                RpcCommand::SetElevons { left, right } => {
                    rpc_elevon_left = (left as f32 / 100.0).clamp(-1.0, 1.0);
                    rpc_elevon_right = (right as f32 / 100.0).clamp(-1.0, 1.0);
                }
                #[cfg(feature = "rpc-rc")]
                RpcCommand::SetElevons { .. } => {} // Ignored in RC mode
                #[cfg(not(feature = "rpc-rc"))]
                RpcCommand::SetMode(mode) => {
                    rpc_mode = match mode {
                        ControlMode::Manual => AttitudeMode::Manual,
                        ControlMode::Stabilized => AttitudeMode::Stabilized,
                        ControlMode::AltitudeHold => AttitudeMode::AltitudeHold,
                    };
                    info!(
                        "RPC: Control mode set to {}",
                        match mode {
                            ControlMode::Manual => "Manual",
                            ControlMode::Stabilized => "Stabilized",
                            ControlMode::AltitudeHold => "AltitudeHold",
                        }
                    );
                }
                #[cfg(feature = "rpc-rc")]
                RpcCommand::SetMode(_) => {} // Ignored in RC mode
                RpcCommand::Arm => {
                    fc.arm();
                    elle_hardware::elle_event!(
                        info,
                        elle_hardware::event::EVT_MOTORS_ARMED,
                        "Motors ARMED via RPC"
                    );
                }
                RpcCommand::Disarm => {
                    fc.disarm();
                    elle_hardware::elle_event!(
                        info,
                        elle_hardware::event::EVT_MOTORS_DISARMED,
                        "Motors DISARMED via RPC"
                    );
                }
                RpcCommand::EmergencyStop => {
                    elle_hardware::elle_event!(
                        warn,
                        elle_hardware::event::EVT_EMERGENCY_STOP,
                        "RPC: EMERGENCY STOP"
                    );
                    #[cfg(not(feature = "rpc-rc"))]
                    {
                        rpc_throttle = 0.0;
                        rpc_elevon_left = 0.0;
                        rpc_elevon_right = 0.0;
                    }
                    fc.disarm();
                    fc.apply_failsafe();
                }
                RpcCommand::SetPidGains { config } => {
                    fc.set_pid_gains(config);
                    info!(
                        "RPC: PID gains updated P({}/{}/{}) R({}/{}/{}) s={} il={}",
                        (config.kp_pitch * 1000.0) as i32,
                        (config.ki_pitch * 1000.0) as i32,
                        (config.kd_pitch * 1000.0) as i32,
                        (config.kp_roll * 1000.0) as i32,
                        (config.ki_roll * 1000.0) as i32,
                        (config.kd_roll * 1000.0) as i32,
                        (config.scale * 10000.0) as i32,
                        (config.i_limit * 10.0) as i32,
                    );
                }
                RpcCommand::StartULog => {
                    // Every Start opens a new SD file, and each file needs its
                    // own header — so always re-initialize, never reuse the
                    // previous session's logger state. Start goes first: the
                    // SD writer discards anything that arrives while it is idle.
                    let mut ok = false;
                    if elle_hardware::sd_writer::SD_READY.load(Ordering::Acquire) {
                        ULOG_ENABLED.store(false, Ordering::Release);
                        elle_hardware::sd_writer::SD_CMD_SIGNAL
                            .signal(elle_hardware::sd_writer::SdCommand::Start);
                        let wall_ms = epoch_ms + Instant::now().as_micros() / 1000;
                        ok = ulog_logger.initialize(wall_ms).await.is_ok();
                        if !ok {
                            elle_hardware::sd_writer::SD_CMD_SIGNAL
                                .signal(elle_hardware::sd_writer::SdCommand::Stop);
                        }
                    }
                    if ok {
                        ULOG_ENABLED.store(true, Ordering::Release);
                        elle_hardware::elle_event!(
                            info,
                            elle_hardware::event::EVT_ULOG_STARTED,
                            "ULog recording started"
                        );
                    } else {
                        elle_hardware::elle_event!(
                            error,
                            elle_hardware::event::EVT_ULOG_INIT_FAILED,
                            "ULog init failed"
                        );
                    }
                }
                RpcCommand::StopULog => {
                    ULOG_ENABLED.store(false, Ordering::Release);
                    let _ = ulog_logger.flush();
                    elle_hardware::sd_writer::SD_CMD_SIGNAL
                        .signal(elle_hardware::sd_writer::SdCommand::Stop);
                    elle_hardware::elle_event!(
                        info,
                        elle_hardware::event::EVT_ULOG_STOPPED,
                        "ULog recording stopped"
                    );
                }
                RpcCommand::ReadULogChunk => {
                    FLASH_REQUEST_SIGNAL.signal(FlashRequest::PeekULog);
                    match FLASH_RESPONSE_SIGNAL.wait().await {
                        FlashResponse::ULogData { data, len } => {
                            ULOG_ITEM_LEN.store(len as u16, Ordering::Release);
                            ULOG_ITEM_SIGNAL.signal((data, len));
                            ULOG_STATE.store(ULogState::Ready as u8, Ordering::Release);
                        }
                        FlashResponse::ULogEmpty => {
                            ULOG_STATE.store(ULogState::Empty as u8, Ordering::Release);
                        }
                        _ => {
                            ULOG_STATE.store(ULogState::Empty as u8, Ordering::Release);
                        }
                    }
                }
                RpcCommand::PopAndPeekULog => {
                    // Pop the item we just finished sending
                    FLASH_REQUEST_SIGNAL.signal(FlashRequest::PopULog);
                    let _ = FLASH_RESPONSE_SIGNAL.wait().await;
                    // Peek the next item
                    FLASH_REQUEST_SIGNAL.signal(FlashRequest::PeekULog);
                    match FLASH_RESPONSE_SIGNAL.wait().await {
                        FlashResponse::ULogData { data, len } => {
                            ULOG_ITEM_LEN.store(len as u16, Ordering::Release);
                            ULOG_ITEM_SIGNAL.signal((data, len));
                            ULOG_STATE.store(ULogState::Ready as u8, Ordering::Release);
                        }
                        FlashResponse::ULogEmpty => {
                            ULOG_STATE.store(ULogState::Empty as u8, Ordering::Release);
                        }
                        _ => {
                            ULOG_STATE.store(ULogState::Empty as u8, Ordering::Release);
                        }
                    }
                }
                RpcCommand::EraseULog => {
                    // Stop recording first
                    ULOG_ENABLED.store(false, Ordering::Release);
                    FLASH_REQUEST_SIGNAL.signal(FlashRequest::EraseULog);
                    let _ = FLASH_RESPONSE_SIGNAL.wait().await;
                    // Reset ULog transfer state
                    ULOG_STATE.store(ULogState::Idle as u8, Ordering::Release);
                    ULOG_OFFSET.store(0, Ordering::Release);
                    ULOG_ITEM_LEN.store(0, Ordering::Release);
                    elle_hardware::elle_event!(
                        info,
                        elle_hardware::event::EVT_ULOG_ERASED,
                        "ULog: flash erased"
                    );
                }
                RpcCommand::StartAutotune {
                    axis,
                    relay_deg_x10,
                    num_cycles,
                    rule,
                } => {
                    if axis == 0xFF {
                        // Magic value: save current PID gains to flash
                        let gains = fc.get_pid_gains();
                        save_pid_to_flash(gains.to_bytes(), "savepid").await;
                    } else if axis == 0xFE {
                        // Magic value: erase PID profile from flash
                        erase_pid_from_flash().await;
                    } else {
                        use elle_control::autotune::TuningRule;
                        if fc.is_armed() && fc.is_attitude_enabled() && !autotuner.is_active() {
                            let at_axis = if axis == 0 {
                                AutotuneAxis::Pitch
                            } else {
                                AutotuneAxis::Roll
                            };
                            let relay_deg = relay_deg_x10 as f32 / 10.0;
                            let at_rule = match rule {
                                1 => TuningRule::ZieglerNichols,
                                2 => TuningRule::SomeOvershoot,
                                _ => TuningRule::TyreusLuyben,
                            };
                            let current_gains = fc.get_pid_gains();
                            let test_gains = autotuner.start(
                                at_axis,
                                current_gains,
                                relay_deg,
                                num_cycles as usize,
                                at_rule,
                                autotune_tick,
                            );
                            fc.apply_saved_gains(&test_gains);
                            elle_hardware::elle_event!(
                                info,
                                elle_hardware::event::EVT_AUTOTUNE_STARTED,
                                "Autotune STARTED via RPC (axis={})",
                                if axis == 0 { "pitch" } else { "roll" }
                            );
                        } else {
                            info!(
                                "RPC: Autotune rejected (armed={} attitude={} active={})",
                                fc.is_armed(),
                                fc.is_attitude_enabled(),
                                autotuner.is_active()
                            );
                        }
                    }
                }
                RpcCommand::AbortAutotune => {
                    if let Some(saved) = autotuner.abort() {
                        fc.apply_saved_gains(&saved);
                        fc.clear_setpoint_override();
                        autotune_display = elle_hardware::crsf::AutotuneDisplay::Error;
                        autotune_display_timer = AUTOTUNE_DISPLAY_DURATION;
                        elle_hardware::elle_event!(
                            warn,
                            elle_hardware::event::EVT_AUTOTUNE_ABORTED,
                            "Autotune ABORTED via RPC"
                        );
                    }
                }
                RpcCommand::StartMagCal => {
                    elle_hardware::imu::MAG_CAL_START_SIGNAL.signal(());
                    rpc_app::MAG_CAL_STATUS.store(1, Ordering::Relaxed);
                    elle_hardware::elle_event!(
                        info,
                        elle_hardware::event::EVT_MAG_CAL_STARTED,
                        "Mag calibration started"
                    );
                }
                RpcCommand::ClearMagCal => {
                    // Save zeros to flash
                    FLASH_REQUEST_SIGNAL.signal(FlashRequest::SaveMagCal { data: [0; 12] });
                    let save_timeout = Timer::after(Duration::from_secs(5));
                    let _ =
                        embassy_futures::select::select(FLASH_RESPONSE_SIGNAL.wait(), save_timeout)
                            .await;
                    // Signal zero offsets to IMU
                    elle_hardware::imu::MAG_CALIBRATION_SIGNAL.signal((0.0, 0.0, 0.0));
                    rpc_app::MAG_CAL_STATUS.store(0, Ordering::Relaxed);
                    rpc_app::MAG_CAL_OFFSET.lock(|c| c.set((0.0, 0.0, 0.0)));
                    elle_hardware::elle_event!(
                        info,
                        elle_hardware::event::EVT_MAG_CAL_CLEARED,
                        "Mag calibration cleared"
                    );
                }
                RpcCommand::StartLevelCal => {
                    if fc.is_armed() {
                        elle_hardware::elle_event!(
                            warn,
                            elle_hardware::event::EVT_LEVEL_CAL_FAILED_MOVING,
                            "Level calibration refused: armed"
                        );
                    } else if !elle_hardware::imu::level_cal::is_collecting() {
                        elle_hardware::imu::level_cal::start();
                    }
                }
                RpcCommand::ClearLevelCal => {
                    elle_hardware::imu::level_cal::clear().await;
                }
                RpcCommand::SetHeadingHold {
                    enabled,
                    target_cdeg,
                } => {
                    if enabled {
                        let target_rad = (target_cdeg as f32 / 100.0).to_radians();
                        fc.engage_heading_hold(target_rad);
                        elle_hardware::elle_event!(
                            info,
                            elle_hardware::event::EVT_HEADING_HOLD_TARGET_SET,
                            "Heading hold target set via RPC"
                        );
                    } else {
                        fc.disengage_heading_hold();
                        elle_hardware::elle_event!(
                            info,
                            elle_hardware::event::EVT_HEADING_HOLD_DISENGAGED,
                            "Heading hold disengaged via RPC"
                        );
                    }
                }
            }
        }

        // Build pilot commands — from CRSF receiver (rc feature) or RPC accumulators
        #[cfg(feature = "rpc-rc")]
        let commands = {
            if let Some(commands) = RC_COMMANDS.try_take() {
                if let PilotCommands::Raw(raw) = &commands {
                    rc_signal::RC_SIGNAL.signal(raw.channels);
                    // Debug logging (~8Hz)
                    if loop_counter.is_multiple_of(CONTROL_LOOP_FREQUENCY_HZ / 10) {
                        debug!(
                            "RC: CH1:{} CH2:{} CH3:{} CH4:{} CH5:{}",
                            raw.channels[ROLL_CH],
                            raw.channels[PITCH_CH],
                            raw.channels[THROTTLE_CH],
                            raw.channels[YAW_CH],
                            raw.channels[ATTITUDE_ENABLE_CH],
                        );
                    }
                }
                fc.note_rc_packet(commands.timestamp());
                last_commands = Some(commands);
            }
            last_commands
        };

        #[cfg(not(feature = "rpc-rc"))]
        let commands = {
            if let Some(PilotCommands::Raw(raw)) = RC_COMMANDS.try_take().as_ref() {
                rc_signal::RC_SIGNAL.signal(raw.channels);
            }
            Some(PilotCommands::Normalized(NormalizedCommands {
                throttle: rpc_throttle,
                pitch: (rpc_elevon_left + rpc_elevon_right) / 2.0,
                roll: (rpc_elevon_right - rpc_elevon_left) / 2.0,
                yaw: 0.0,
                attitude_mode: rpc_mode,
                timestamp: Instant::now(),
            }))
        };

        // Kill switch: CH8 high = disarm (RPC+RC mode)
        #[cfg(feature = "rpc-rc")]
        let kill_active = last_commands.as_ref().is_some_and(|cmd| {
            if let PilotCommands::Raw(raw) = cmd {
                raw.channels[elle_config::KILL_SWITCH_CH] > elle_config::KILL_SWITCH_THRESHOLD
            } else {
                false
            }
        });
        #[cfg(not(feature = "rpc-rc"))]
        let kill_active = false;

        if kill_active != was_killed {
            if kill_active {
                elle_hardware::elle_event!(
                    warn,
                    elle_hardware::event::EVT_KILL_ENGAGED,
                    "Kill switch engaged (RPC)"
                );
            } else {
                elle_hardware::elle_event!(
                    info,
                    elle_hardware::event::EVT_KILL_RELEASED,
                    "Kill switch released (RPC)"
                );
            }
            was_killed = kill_active;
        }

        if kill_active {
            if fc.is_armed() {
                fc.disarm();
            }
            fc.set_safe_positions();
            DSHOT_THROTTLE.signal((0, 0));
        }

        // Double-tap calibration gesture (RPC mode) — only while the kill switch
        // is active (rpc-rc builds; inert in pure RPC builds where kill_active
        // is always false — use `mag cal start` / `level cal start` there instead).
        // CH7 out of its off position selects level cal, as in flight mode.
        let tapped = elle_hardware::imu::TAP_SIGNAL.try_take().is_some();
        if tapped
            && !fc.is_armed()
            && kill_active
            && rpc_app::MAG_CAL_STATUS.load(Ordering::Relaxed) != 1
            && !elle_hardware::imu::level_cal::is_collecting()
        {
            if !tap_cal_allowed(commands.as_ref(), attitude.as_ref()) {
                info!("Double-tap ignored: throttle not low or gyro not quiet");
            } else if tap_selects_level_cal(commands.as_ref()) {
                elle_hardware::imu::level_cal::start();
            } else {
                elle_hardware::imu::MAG_CAL_START_SIGNAL.signal(());
                rpc_app::MAG_CAL_STATUS.store(1, Ordering::Relaxed);
                elle_hardware::elle_event!(
                    info,
                    elle_hardware::event::EVT_MAG_CAL_STARTED,
                    "Double-tap: mag calibration started"
                );
            }
        }

        // Update flight controller
        let valid_attitude = validate_attitude(attitude);
        if let Some(ref commands) = commands
            && !kill_active
        {
            fc.update(commands, valid_attitude.as_ref());
        }

        // Send engine commands via DShot (governor converts eRPM target to DShot)
        if !kill_active {
            publish_engine_output(fc);
        }

        // Detect arm/disarm transitions → beep
        let now_armed = fc.is_armed();
        if now_armed && !was_armed {
            elle_hardware::dshot::BEEP_SIGNAL.signal(elle_hardware::dshot::BeepPattern::ArmBeep);
        } else if !now_armed && was_armed {
            elle_hardware::dshot::BEEP_SIGNAL.signal(elle_hardware::dshot::BeepPattern::DisarmBeep);
        }
        was_armed = now_armed;

        // Check for RC signal loss (only relevant when RC is the command source).
        // Packet arrival is stamped via note_rc_packet(), so this stays
        // accurate even while the kill switch blocks fc.update().
        #[cfg(feature = "rpc-rc")]
        fc.check_failsafe();

        // Autotuner per-tick update
        if autotuner.is_active()
            && let Some(att) = valid_attitude.as_ref()
        {
            let measurement_deg = match autotuner.axis() {
                AutotuneAxis::Pitch => att.pitch * (180.0 / core::f32::consts::PI),
                AutotuneAxis::Roll => att.roll * (180.0 / core::f32::consts::PI),
            };
            match autotuner.update(measurement_deg, autotune_tick) {
                AutotuneAction::None => {}
                AutotuneAction::SetpointOverride {
                    pitch_deg,
                    roll_deg,
                } => {
                    fc.set_setpoint_override(pitch_deg, roll_deg);
                }
                AutotuneAction::ApplyGains(gains) => {
                    fc.apply_saved_gains(&gains);
                }
                AutotuneAction::RestoreGains(gains) => {
                    fc.apply_saved_gains(&gains);
                    fc.clear_setpoint_override();
                    autotune_display = elle_hardware::crsf::AutotuneDisplay::Error;
                    autotune_display_timer = AUTOTUNE_DISPLAY_DURATION;
                    elle_hardware::elle_event!(
                        warn,
                        elle_hardware::event::EVT_AUTOTUNE_ESTOP,
                        "Autotune safety abort (timeout/amplitude)"
                    );
                }
                AutotuneAction::Completed(result) => {
                    if let Some(gains) = autotuner.computed_gains() {
                        fc.apply_saved_gains(&gains);
                        rpc_save_pending = Some(gains.to_bytes());
                    }
                    fc.clear_setpoint_override();
                    autotune_display = elle_hardware::crsf::AutotuneDisplay::Done;
                    autotune_display_timer = AUTOTUNE_DISPLAY_DURATION;
                    info!(
                        "Autotune COMPLETE: Ku={} Tu={}ms kp={} ki={} kd={} amp={}cdeg cycles={}",
                        (result.ku * 1000.0) as i32,
                        (result.tu_s * 1000.0) as i32,
                        (result.kp * 1000.0) as i32,
                        (result.ki * 1000.0) as i32,
                        (result.kd * 1000.0) as i32,
                        (result.amplitude_deg * 100.0) as i32,
                        result.cycles,
                    );
                    elle_hardware::elle_event!(
                        info,
                        elle_hardware::event::EVT_AUTOTUNE_COMPLETE,
                        "Autotune COMPLETE (RPC)"
                    );
                }
            }

            // Log autotune status to ULog (control-loop rate during active autotune)
            if ULOG_ENABLED.load(Ordering::Acquire) {
                let _ = ulog_logger.log_autotune(
                    autotuner.phase_u8(),
                    autotuner.axis() as u8,
                    autotuner.relay_positive(),
                    autotuner.current_setpoint_deg(),
                    measurement_deg,
                    autotuner.cycles_completed(),
                    autotuner.current_amplitude_deg(),
                );
            }
        }
        autotune_tick += 1;

        // Auto-save PID gains to flash after autotune completion
        if let Some(data) = rpc_save_pending.take() {
            save_pid_to_flash(data, "autotune").await;
        }

        // Poll level calibration result from IMU task
        elle_hardware::imu::level_cal::poll_result().await;

        // Poll mag calibration result from IMU task
        if let Some(result) = elle_hardware::imu::MAG_CAL_RESULT_SIGNAL.try_take() {
            match result {
                Some((ox, oy, oz)) => {
                    let data: [u8; 12] = bytemuck::cast([ox, oy, oz]);
                    FLASH_REQUEST_SIGNAL.signal(FlashRequest::SaveMagCal { data });
                    let save_timeout = Timer::after(Duration::from_secs(5));
                    match embassy_futures::select::select(
                        FLASH_RESPONSE_SIGNAL.wait(),
                        save_timeout,
                    )
                    .await
                    {
                        embassy_futures::select::Either::First(FlashResponse::MagCalSaved) => {
                            elle_hardware::elle_event!(
                                info,
                                elle_hardware::event::EVT_MAG_CAL_SAVED,
                                "Mag cal saved to flash"
                            );
                        }
                        _ => {
                            elle_hardware::elle_event!(
                                warn,
                                elle_hardware::event::EVT_MAG_CAL_FAILED,
                                "Mag cal flash save failed"
                            );
                        }
                    }
                    rpc_app::MAG_CAL_STATUS.store(2, Ordering::Relaxed);
                    rpc_app::MAG_CAL_OFFSET.lock(|c| c.set((ox, oy, oz)));
                    elle_hardware::elle_event!(
                        info,
                        elle_hardware::event::EVT_MAG_CAL_COMPLETE,
                        "Mag calibration complete"
                    );
                }
                None => {
                    rpc_app::MAG_CAL_STATUS.store(0, Ordering::Relaxed);
                    elle_hardware::elle_event!(
                        warn,
                        elle_hardware::event::EVT_MAG_CAL_FAILED,
                        "Mag calibration failed (insufficient rotation)"
                    );
                }
            }
        }

        // Update mag cal sample count for RPC visibility
        // (read from IMU side if collecting — approximated via status check)

        // Update autotune display state for CRSF telemetry
        if autotuner.is_active() {
            autotune_display = match autotuner.axis() {
                AutotuneAxis::Pitch => elle_hardware::crsf::AutotuneDisplay::Pitch,
                AutotuneAxis::Roll => elle_hardware::crsf::AutotuneDisplay::Roll,
            };
        } else if autotune_display_timer > 0 {
            autotune_display_timer -= 1;
            if autotune_display_timer == 0 {
                autotune_display = elle_hardware::crsf::AutotuneDisplay::Off;
            }
        }

        elle_hardware::crsf::CRSF_FLIGHT_MODE.signal(elle_hardware::crsf::CrsfFlightMode {
            armed: fc.is_armed(),
            failsafe: fc.is_failsafe(),
            mode: match fc.current_control_mode() {
                elle_system::ControlMode::Manual => elle_hardware::crsf::CrsfControlMode::Manual,
                elle_system::ControlMode::Stabilized => {
                    elle_hardware::crsf::CrsfControlMode::Stabilized
                }
                elle_system::ControlMode::AltitudeHold => {
                    elle_hardware::crsf::CrsfControlMode::AltitudeHold
                }
            },
            autotune: autotune_display,
            heading_hold: fc.is_heading_hold_active(),
        });

        // Publish flight state for RPC handlers
        let fs = flight_state::FlightState {
            armed: fc.is_armed(),
            failsafe: fc.is_failsafe(),
            mode: match fc.current_control_mode() {
                elle_system::ControlMode::Manual => ControlMode::Manual,
                elle_system::ControlMode::Stabilized => ControlMode::Stabilized,
                elle_system::ControlMode::AltitudeHold => ControlMode::AltitudeHold,
            },
            rc_age_ms: fc.rc_signal_age_ms(),
            autotune_state: if autotuner.is_active() {
                if autotuner.axis() == AutotuneAxis::Pitch {
                    1
                } else {
                    2
                }
            } else {
                0
            },
        };
        flight_state::FLIGHT_STATE.publish(fs);

        // Publish controller output for RPC observability
        {
            let out = fc.last_output();
            let co = flight_state::ControllerOutput {
                pitch_correction: out.pitch_correction,
                roll_correction: out.roll_correction,
                pitch_setpoint_deg: out.pitch_setpoint_deg,
                roll_setpoint_deg: out.roll_setpoint_deg,
                elevon_left_us: out.elevon_left_us,
                elevon_right_us: out.elevon_right_us,
                engine_left_dshot: out.engine_left_dshot,
                engine_right_dshot: out.engine_right_dshot,
                heading_hold_active: out.heading_hold_active,
                heading_target_deg: out.heading_target_deg,
                heading_error_deg: out.heading_error_deg,
            };
            flight_state::CONTROLLER_OUTPUT.publish(co);
        }

        // Log flight data to ULog flash storage (only when recording is active)
        let neutral = PilotCommands::Normalized(NormalizedCommands::neutral());
        let ulog_commands = commands.as_ref().unwrap_or(&neutral);
        if ULOG_ENABLED.load(Ordering::Acquire) {
            log_flight_data(
                &mut ulog_logger,
                valid_attitude.as_ref(),
                ulog_commands,
                loop_counter,
                loop_start.elapsed().as_micros() as u32,
                fc,
            );
        } else if loop_counter.is_multiple_of(STALE_EVENT_DRAIN_DIVISOR) {
            // Drain stale events when not recording
            while elle_hardware::event::ULOG_EVENT_CHANNEL
                .try_receive()
                .is_ok()
            {}
        }

        update_control_loop_timing(loop_timer.elapsed_us());
        loop_counter = loop_counter.saturating_add(1);

        if loop_counter.is_multiple_of(PERF_LOG_INTERVAL) {
            log_performance_summary();
        }

        if loop_counter.is_multiple_of(LED_UPDATE_INTERVAL) {
            loop_counter = 0;

            let imu_status = IMU_STATUS.read().await;
            let led_pattern = if fc.is_failsafe() {
                LedPattern::RapidFlash(colors::ORANGE)
            } else if elle_hardware::imu::level_cal::is_collecting() {
                LedPattern::FastBlink(colors::CYAN)
            } else if fc.is_armed() && fc.rc_link_state() == elle_system::RcLinkState::Warning {
                LedPattern::FastBlink(colors::ORANGE)
            } else if fc.is_armed() {
                if fc.is_heading_hold_active() {
                    LedPattern::Pulse(colors::BLUE)
                } else {
                    LedPattern::DoubleBlink(colors::PURPLE)
                }
            } else if imu_status.calibrated {
                LedPattern::Solid(colors::PURPLE)
            } else {
                LedPattern::Pulse(colors::PURPLE)
            };

            let _ = LED_COMMAND_CHANNEL.try_send(led_pattern);
            drop(imu_status);
        }
    }
}
