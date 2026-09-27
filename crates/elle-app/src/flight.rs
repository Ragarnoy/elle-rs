//! Flight mode: the CRSF/ELRS control loop (default builds, no `rpc-control`).

use crate::engines::publish_engine_output;
use crate::logging::log_flight_data;
use crate::support::{
    AUTOTUNE_DISPLAY_DURATION, FLASH_WRITE_TIMEOUT, RAD_TO_DEG, RC_DEBUG_LOG_DIVISOR,
    resync_after_stall, save_pid_to_flash, tap_cal_allowed, tap_selects_level_cal,
    validate_attitude,
};

use defmt::{debug, info};
use elle_config::*;
use elle_control::autotune::{AutotuneAction, AutotuneAxis, Autotuner};
use elle_control::commands::PilotCommands;
use elle_hardware::ULogLogger;
use elle_hardware::crsf::RC_COMMANDS;
use elle_hardware::dshot::DSHOT_THROTTLE;
use elle_hardware::imu::{ATTITUDE, IMU_STATUS, LED_COMMAND_CHANNEL};
use elle_hardware::led::{LedPattern, colors};
use elle_system::{
    FlightController, TimingMeasurement, log_performance_summary, update_control_loop_timing,
};
use embassy_time::{Duration, Instant, Ticker, Timer};

/// Relay amplitude for an autotune started from the RC switch (degrees).
const RC_AUTOTUNE_RELAY_DEG: f32 = 5.0;
/// Oscillation cycles measured by an autotune started from the RC switch.
const RC_AUTOTUNE_CYCLES: usize = 6;

/// Run the flight-mode control loop forever. `epoch_ms` is the wall-clock time
/// at boot (ms since the UNIX epoch), used to stamp new ULog files.
///
/// The controller is borrowed, not moved: a value moved into an async fn keeps
/// its storage in every enclosing future too. The ULog logger (~6 KB of buffers)
/// is created here for the same reason. It starts uninitialised: recording
/// starts once the SD card is ready.
pub(crate) async fn run_flight(fc: &mut FlightController<'static>, epoch_ms: u64) -> ! {
    let mut ulog_logger = ULogLogger::new();
    let mut loop_counter = 0u32;

    info!("FLIGHT MODE - CRSF/ELRS Control");

    let mut ulog_recording = false;

    // Autotune state
    let mut autotuner = Autotuner::new();
    let mut autotune_tick: u32 = 0;
    let mut autotune_debounce_pos: u8 = 0; // 0=Off, 1=Pitch, 2=Roll
    let mut autotune_debounce_count: u32 = 0;
    let mut autotune_stable_pos: u8 = 0;
    let mut pitch_tune_done: bool = false;
    let mut save_pending: Option<[u8; 32]> = None;
    // Autotune display for CRSF telemetry (DONE/ERR shown for ~3s then reverts to Off)
    let mut autotune_display = elle_hardware::crsf::AutotuneDisplay::Off;
    let mut autotune_display_timer: u32 = 0;

    // Heading-hold state (CH5, 2-pos switch, modifier active only while Stabilized)
    let mut heading_hold_switch_debounced: bool = false;
    let mut heading_hold_debounce_count: u32 = 0;
    let mut heading_hold_effective_prev: bool = false;

    // Mag calibration collection in progress (double-tap gesture)
    let mut mag_cal_collecting: bool = false;

    // Create ticker for the control loop period (CONTROL_LOOP_PERIOD_MS)
    let mut ticker = Ticker::every(Duration::from_millis(CONTROL_LOOP_PERIOD_MS));
    let mut previous_tick = None;

    // Track last commands for consistent update rate
    let mut last_commands: Option<PilotCommands> = None;
    let mut was_armed = false;
    let mut was_killed = false;

    loop {
        ticker.next().await; // Wait for next tick BEFORE processing
        let loop_start = Instant::now();
        resync_after_stall(&mut ticker, &mut previous_tick, loop_start);
        let loop_timer = TimingMeasurement::start();

        // Supervisor check - monitor core health and kick watchdog
        let _supervisor_healthy = fc.supervisor_check();

        // Get latest attitude data (non-blocking)
        let attitude = ATTITUDE.try_take();

        // Check for latest RC commands from dedicated CRSF receiver task (non-blocking)
        if let Some(commands) = RC_COMMANDS.try_take() {
            // Debug logging (~10 Hz)
            if loop_counter.is_multiple_of(RC_DEBUG_LOG_DIVISOR)
                && let PilotCommands::Raw(raw) = &commands
            {
                debug!(
                    "RC: CH1:{} CH2:{} CH3:{} CH4:{} CH5:{}",
                    raw.channels[ROLL_CH],
                    raw.channels[PITCH_CH],
                    raw.channels[THROTTLE_CH],
                    raw.channels[YAW_CH],
                    raw.channels[ATTITUDE_ENABLE_CH],
                );
            }

            fc.note_rc_packet(commands.timestamp());
            last_commands = Some(commands);
        }

        // Always update flight controller every tick for consistent PID timing
        // Use last known commands if no new packet arrived this iteration
        // Kill switch: CH8 high = disarm, block fc.update() to prevent re-arm
        let kill_active = last_commands.as_ref().is_some_and(|cmd| {
            if let PilotCommands::Raw(raw) = cmd {
                raw.channels[elle_config::KILL_SWITCH_CH] > elle_config::KILL_SWITCH_THRESHOLD
            } else {
                false
            }
        });

        if kill_active != was_killed {
            if kill_active {
                elle_hardware::elle_event!(
                    warn,
                    elle_hardware::event::EVT_KILL_ENGAGED,
                    "Kill switch engaged"
                );
            } else {
                elle_hardware::elle_event!(
                    info,
                    elle_hardware::event::EVT_KILL_RELEASED,
                    "Kill switch released"
                );
            }
            was_killed = kill_active;
        }

        if kill_active {
            // Kill switch active: disarm, center elevons, zero throttle
            if fc.is_armed() {
                fc.disarm();
            }
            fc.set_safe_positions();
            DSHOT_THROTTLE.signal((0, 0));
        }

        // Double-tap mag-cal gesture — only while the kill switch is active
        // (deliberate service state; motors locked out) so handling bumps
        // can't start a calibration. Drain the signal every iteration so a
        // tap detected outside kill mode can't stay latched and fire later.
        let tapped = elle_hardware::imu::TAP_SIGNAL.try_take().is_some();
        if tapped
            && !fc.is_armed()
            && kill_active
            && !mag_cal_collecting
            && !elle_hardware::imu::level_cal::is_collecting()
        {
            if !tap_cal_allowed(last_commands.as_ref(), attitude.as_ref()) {
                info!("Double-tap ignored: throttle not low or gyro not quiet");
            } else if tap_selects_level_cal(last_commands.as_ref()) {
                elle_hardware::imu::level_cal::start();
            } else {
                elle_hardware::imu::MAG_CAL_START_SIGNAL.signal(());
                mag_cal_collecting = true;
                elle_hardware::elle_event!(
                    info,
                    elle_hardware::event::EVT_MAG_CAL_STARTED,
                    "Double-tap: mag calibration started"
                );
            }
        }

        // Poll level calibration result from IMU task (tap-triggered collection)
        // Calibration results are applied on Core 1 at once; saving them
        // writes flash, which must wait until disarmed.
        if !fc.is_armed() {
            elle_hardware::imu::level_cal::poll_result().await;
        }

        // Poll mag calibration result from IMU task (tap-triggered collection)
        if !fc.is_armed()
            && let Some(result) = elle_hardware::imu::MAG_CAL_RESULT_SIGNAL.try_take()
        {
            mag_cal_collecting = false;
            match result {
                Some((ox, oy, oz)) => {
                    use elle_config::profile::{FlashRequest, FlashResponse};
                    use elle_hardware::flash::{FLASH_REQUEST_SIGNAL, FLASH_RESPONSE_SIGNAL};
                    let data: [u8; 12] = bytemuck::cast([ox, oy, oz]);
                    FLASH_REQUEST_SIGNAL.signal(FlashRequest::SaveMagCal { data });
                    let save_timeout = Timer::after(FLASH_WRITE_TIMEOUT);
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
                    elle_hardware::elle_event!(
                        info,
                        elle_hardware::event::EVT_MAG_CAL_COMPLETE,
                        "Mag calibration complete"
                    );
                }
                None => {
                    elle_hardware::elle_event!(
                        warn,
                        elle_hardware::event::EVT_MAG_CAL_FAILED,
                        "Mag calibration failed (insufficient rotation)"
                    );
                }
            }
        }

        if let Some(commands) = &last_commands {
            // Update with validated attitude (warns if stale)
            let valid_attitude = validate_attitude(attitude);
            if valid_attitude.is_none() && attitude.is_some() {
                elle_hardware::elle_event!(
                    warn,
                    elle_hardware::event::EVT_ATTITUDE_STALE,
                    "Stale attitude data, using manual control only"
                );
            }
            if !kill_active {
                fc.update(commands, valid_attitude.as_ref());
            }

            // Detect arm/disarm transitions → beep + event
            let now_armed = fc.is_armed();
            if now_armed && !was_armed {
                elle_hardware::dshot::BEEP_SIGNAL
                    .signal(elle_hardware::dshot::BeepPattern::ArmBeep);
                elle_hardware::elle_event!(
                    info,
                    elle_hardware::event::EVT_MOTORS_ARMED,
                    "Motors armed"
                );
            } else if !now_armed && was_armed {
                elle_hardware::dshot::BEEP_SIGNAL
                    .signal(elle_hardware::dshot::BeepPattern::DisarmBeep);
                elle_hardware::elle_event!(
                    warn,
                    elle_hardware::event::EVT_MOTORS_DISARMED,
                    "Motors disarmed"
                );
            }
            was_armed = now_armed;

            // Send engine commands via DShot (governor converts eRPM target to DShot)
            if !kill_active {
                publish_engine_output(fc);
            }

            // Update autotune display state for CRSF telemetry
            if autotuner.is_active() {
                autotune_display = if autotune_stable_pos == 1 {
                    elle_hardware::crsf::AutotuneDisplay::Pitch
                } else {
                    elle_hardware::crsf::AutotuneDisplay::Roll
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
                    elle_system::ControlMode::Manual => {
                        elle_hardware::crsf::CrsfControlMode::Manual
                    }
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

            // --- Autotune RC switch logic (CH9, 3-position with debounce) ---
            if let PilotCommands::Raw(raw) = commands {
                let ch9 = raw.channels[elle_config::AUTOTUNE_CH];
                let current_pos: u8 = if ch9 < elle_config::AUTOTUNE_OFF_THRESHOLD {
                    0 // Off
                } else if ch9 < elle_config::AUTOTUNE_PITCH_THRESHOLD {
                    1 // Pitch
                } else {
                    2 // Roll
                };

                // Debounce
                if current_pos == autotune_debounce_pos {
                    autotune_debounce_count += 1;
                } else {
                    autotune_debounce_pos = current_pos;
                    autotune_debounce_count = 0;
                }

                if autotune_debounce_count == elle_config::AUTOTUNE_DEBOUNCE_TICKS {
                    let new_pos = autotune_debounce_pos;

                    if new_pos == 0 && autotuner.is_active() {
                        // Abort: switch moved to off while active
                        if let Some(saved) = autotuner.abort() {
                            fc.apply_saved_gains(&saved);
                            fc.clear_setpoint_override();
                            autotune_display = elle_hardware::crsf::AutotuneDisplay::Error;
                            autotune_display_timer = AUTOTUNE_DISPLAY_DURATION;
                            elle_hardware::elle_event!(
                                warn,
                                elle_hardware::event::EVT_AUTOTUNE_ABORTED,
                                "Autotune ABORTED (RC switch off)"
                            );
                        }
                    } else if autotune_stable_pos == 0
                        && (new_pos == 1 && !pitch_tune_done || new_pos == 2)
                        && !autotuner.is_active()
                    {
                        // Start: from off to pitch (not locked) or roll
                        if fc.is_armed() && fc.is_attitude_enabled() {
                            let axis = if new_pos == 1 {
                                AutotuneAxis::Pitch
                            } else {
                                AutotuneAxis::Roll
                            };
                            let current_gains = fc.get_pid_gains();
                            let test_gains = autotuner.start(
                                axis,
                                current_gains,
                                RC_AUTOTUNE_RELAY_DEG,
                                RC_AUTOTUNE_CYCLES,
                                elle_control::autotune::TuningRule::TyreusLuyben,
                                autotune_tick,
                            );
                            fc.apply_saved_gains(&test_gains);
                            elle_hardware::elle_event!(
                                info,
                                elle_hardware::event::EVT_AUTOTUNE_STARTED,
                                "Autotune STARTED (axis={})",
                                if new_pos == 1 { "pitch" } else { "roll" }
                            );
                        }
                    }
                    // Completion lock: ignore mid position if pitch already done
                    // (no action needed, the conditions above skip it)

                    autotune_stable_pos = new_pos;
                }
            }

            // --- Heading-hold RC switch logic (CH5, 2-position, debounced) ---
            // Active only while Stabilized is selected; captures current heading
            // on the rising edge of (switch on AND mode == Stabilized).
            if let PilotCommands::Raw(raw) = commands {
                let ch5 = raw.channels[elle_config::HEADING_HOLD_CH];
                let switch_on = ch5 > elle_config::HEADING_HOLD_THRESHOLD;

                if switch_on == heading_hold_switch_debounced {
                    heading_hold_debounce_count = 0;
                } else {
                    heading_hold_debounce_count += 1;
                    if heading_hold_debounce_count >= elle_config::HEADING_HOLD_DEBOUNCE_TICKS {
                        heading_hold_switch_debounced = switch_on;
                        heading_hold_debounce_count = 0;
                    }
                }

                let heading_hold_effective = heading_hold_switch_debounced
                    && fc.current_control_mode() == elle_system::ControlMode::Stabilized;

                if heading_hold_effective && !heading_hold_effective_prev {
                    if let Some(att) = valid_attitude.as_ref() {
                        fc.engage_heading_hold(att.yaw);
                        elle_hardware::elle_event!(
                            info,
                            elle_hardware::event::EVT_HEADING_HOLD_ENGAGED,
                            "Heading hold ENGAGED"
                        );
                    }
                } else if !heading_hold_effective && heading_hold_effective_prev {
                    fc.disengage_heading_hold();
                    elle_hardware::elle_event!(
                        info,
                        elle_hardware::event::EVT_HEADING_HOLD_DISENGAGED,
                        "Heading hold DISENGAGED"
                    );
                }
                heading_hold_effective_prev = heading_hold_effective;
            }

            // Autotune state machine tick
            if autotuner.is_active() && valid_attitude.is_none() {
                // Attitude data lost during autotune — abort for safety
                if let Some(saved) = autotuner.abort() {
                    fc.apply_saved_gains(&saved);
                    fc.clear_setpoint_override();
                    autotune_display = elle_hardware::crsf::AutotuneDisplay::Error;
                    autotune_display_timer = AUTOTUNE_DISPLAY_DURATION;
                    elle_hardware::elle_event!(
                        error,
                        elle_hardware::event::EVT_AUTOTUNE_ESTOP,
                        "Autotune aborted: attitude data lost"
                    );
                }
            }
            if autotuner.is_active()
                && let Some(att) = valid_attitude.as_ref()
            {
                let measurement_deg = if autotune_stable_pos == 1 {
                    att.pitch * RAD_TO_DEG
                } else {
                    att.roll * RAD_TO_DEG
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
                    AutotuneAction::Rejected { reason, gains } => {
                        fc.apply_saved_gains(&gains);
                        fc.clear_setpoint_override();
                        autotune_display = elle_hardware::crsf::AutotuneDisplay::Error;
                        autotune_display_timer = AUTOTUNE_DISPLAY_DURATION;
                        elle_hardware::elle_event!(
                            warn,
                            elle_hardware::event::EVT_AUTOTUNE_REJECTED,
                            "Autotune result rejected ({}); original gains restored",
                            reason
                        );
                    }
                    AutotuneAction::Completed(result) => {
                        if let Some(gains) = autotuner.computed_gains() {
                            fc.apply_saved_gains(&gains);
                            save_pending = Some(gains.to_bytes());
                        }
                        fc.clear_setpoint_override();
                        if result.axis == AutotuneAxis::Pitch {
                            pitch_tune_done = true;
                        }
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
                            "Autotune COMPLETE"
                        );
                    }
                }

                // Log autotune status to ULog (control-loop rate during active autotune)
                if ulog_recording {
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

            // Persist autotuned gains once disarmed. They are already applied in
            // RAM; a flash write pauses Core 1 and blocks Core 0 (DShot included),
            // so it must never happen in the air.
            if !fc.is_armed()
                && let Some(data) = save_pending.take()
            {
                save_pid_to_flash(data, "autotune").await;
            }

            // Auto-start ULog on SD card ready (runs until power off)
            if !ulog_recording
                && elle_hardware::sd_writer::SD_READY.load(core::sync::atomic::Ordering::Acquire)
            {
                if !ulog_logger.is_initialized() {
                    // Start before the header is written: the SD writer
                    // discards anything that arrives while it is idle.
                    elle_hardware::sd_writer::SD_CMD_SIGNAL
                        .signal(elle_hardware::sd_writer::SdCommand::Start);
                    let wall_ms = epoch_ms + Instant::now().as_micros() / 1000;
                    if ulog_logger.initialize(wall_ms).await.is_err() {
                        elle_hardware::sd_writer::SD_CMD_SIGNAL
                            .signal(elle_hardware::sd_writer::SdCommand::Stop);
                    }
                }
                if ulog_logger.is_initialized() {
                    ulog_recording = true;
                    elle_hardware::elle_event!(
                        info,
                        elle_hardware::event::EVT_ULOG_RC_ON,
                        "ULog recording started (auto)"
                    );
                }
            }

            if ulog_recording {
                log_flight_data(
                    &mut ulog_logger,
                    valid_attitude.as_ref(),
                    commands,
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
        }

        // Check for failsafe (triggers after 300ms of no valid packets).
        // Packet arrival is stamped via note_rc_packet(), so this stays
        // accurate even while the kill switch blocks fc.update().
        fc.check_failsafe();

        update_control_loop_timing(loop_timer.elapsed_us());
        loop_counter = loop_counter.saturating_add(1);

        // Periodic status updates
        if loop_counter.is_multiple_of(PERF_LOG_INTERVAL) {
            log_performance_summary();
        }

        if loop_counter.is_multiple_of(LED_UPDATE_INTERVAL) {
            loop_counter = 0;

            let imu_status = IMU_STATUS.read().await;

            let led_pattern = if fc.is_failsafe() {
                LedPattern::RapidFlash(colors::ORANGE)
            } else if mag_cal_collecting {
                LedPattern::FastBlink(colors::YELLOW)
            } else if elle_hardware::imu::level_cal::is_collecting() {
                LedPattern::FastBlink(colors::CYAN)
            } else if fc.is_armed() && fc.rc_link_state() == elle_system::RcLinkState::Warning {
                LedPattern::FastBlink(colors::ORANGE)
            } else if fc.is_armed() {
                if fc.is_attitude_enabled() {
                    if fc.is_heading_hold_active() {
                        LedPattern::Pulse(colors::BLUE)
                    } else {
                        LedPattern::Pulse(colors::CYAN)
                    }
                } else {
                    LedPattern::DoubleBlink(colors::GREEN)
                }
            } else if imu_status.calibrated {
                LedPattern::Solid(colors::GREEN)
            } else {
                LedPattern::Pulse(colors::CYAN)
            };

            let _ = LED_COMMAND_CHANNEL.try_send(led_pattern);
            drop(imu_status);
        }
    }
}
