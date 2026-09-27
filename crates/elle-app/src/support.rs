//! Small helpers shared by the control loops: attitude freshness, the
//! autotune run guard, the calibration tap gesture, and PID profile persistence.

#[cfg(feature = "rpc-control")]
use defmt::info;
use defmt::warn;
use elle_config::IMU_MAX_AGE_MS;
use elle_control::autotune::{Autotuner, check_run_conditions};
use elle_hardware::imu::{AttitudeData, is_attitude_valid};
use elle_system::FlightController;
use embassy_time::{Duration, Timer};

/// How long a flash save or erase may take before the control loop gives up
/// waiting for the flash manager's answer.
pub(crate) const FLASH_WRITE_TIMEOUT: Duration = Duration::from_secs(5);

/// Radians to degrees, for the autotuner's per-tick attitude measurement.
pub(crate) const RAD_TO_DEG: f32 = 180.0 / core::f32::consts::PI;

/// Control-loop ticks between RC channel debug lines (~10 Hz).
#[cfg(any(not(feature = "rpc-control"), feature = "rpc-rc"))]
pub(crate) const RC_DEBUG_LOG_DIVISOR: u32 = elle_config::CONTROL_LOOP_FREQUENCY_HZ / 10;

/// Control-loop ticks the CRSF autotune display holds DONE/ERR before
/// reverting to Off (~3 s).
pub(crate) const AUTOTUNE_DISPLAY_DURATION: u32 = elle_config::CONTROL_LOOP_FREQUENCY_HZ * 3;

/// Stop the control-loop ticker from replaying missed ticks after a stall.
///
/// `Ticker::next()` returns immediately while it is behind schedule, so after a
/// long stall the loop would run every missed tick back to back — the PID
/// integrating the same stale attitude over and over at a fixed dt. When the
/// gap since the previous tick exceeds two periods, restart the ticker from now.
pub(crate) fn resync_after_stall(
    ticker: &mut embassy_time::Ticker,
    previous_tick: &mut Option<embassy_time::Instant>,
    now: embassy_time::Instant,
) {
    let period = Duration::from_millis(elle_config::CONTROL_LOOP_PERIOD_MS);
    if let Some(prev) = *previous_tick {
        let gap = now.saturating_duration_since(prev);
        if gap > period * 2 {
            ticker.reset();
            warn!(
                "Control loop stalled {} ms; ticker resynced",
                gap.as_millis()
            );
        }
    }
    *previous_tick = Some(now);
}

/// Helper to validate attitude data and return only if fresh
#[inline]
#[must_use]
pub(crate) fn validate_attitude(attitude: Option<AttitudeData>) -> Option<AttitudeData> {
    attitude.filter(|att| is_attitude_valid(att, Duration::from_millis(IMU_MAX_AGE_MS)))
}

/// Stop an active autotune run that no longer owns the aircraft: killed,
/// disarmed (including failsafe), attitude controller off (Manual, Core 1
/// unhealthy) or no valid attitude. Restores the saved gains and clears the
/// setpoint override, so returning to Stabilized can't resume the test.
/// Call it every tick before the autotuner update. Returns true when it
/// aborted a run.
pub(crate) fn guard_autotune(
    autotuner: &mut Autotuner,
    fc: &mut FlightController<'static>,
    killed: bool,
    has_attitude: bool,
) -> bool {
    if !autotuner.is_active() {
        return false;
    }
    let Err(reason) = check_run_conditions(
        fc.is_armed(),
        fc.is_attitude_enabled(),
        killed,
        has_attitude,
    ) else {
        return false;
    };
    if let Some(saved) = autotuner.abort() {
        fc.apply_saved_gains(&saved);
    }
    fc.clear_setpoint_override();
    elle_hardware::elle_event!(
        warn,
        elle_hardware::event::EVT_AUTOTUNE_ESTOP,
        "Autotune aborted: {}",
        reason
    );
    true
}

/// Gate for the double-tap mag-cal gesture: only start calibration when
/// throttle is commanded low and the gyro is quiet, so motor vibration or
/// handling can't trigger a spurious calibration.
#[must_use]
pub(crate) fn tap_cal_allowed(
    commands: Option<&elle_control::commands::PilotCommands>,
    attitude: Option<&AttitudeData>,
) -> bool {
    use elle_control::commands::PilotCommands;
    let throttle_low = commands.is_some_and(|cmd| match cmd {
        PilotCommands::Raw(raw) => {
            raw.channels[elle_config::THROTTLE_CH] < elle_config::TAP_CAL_THROTTLE_MAX_RAW
        }
        PilotCommands::Normalized(n) => n.throttle < elle_config::TAP_CAL_THROTTLE_MAX_NORM,
    });
    let gyro_quiet = attitude.is_some_and(|a| {
        a.pitch_rate.abs() < elle_config::TAP_CAL_MAX_GYRO_RAD_S
            && a.roll_rate.abs() < elle_config::TAP_CAL_MAX_GYRO_RAD_S
            && a.yaw_rate.abs() < elle_config::TAP_CAL_MAX_GYRO_RAD_S
    });
    throttle_low && gyro_quiet
}

/// Whether a calibration gesture means *level* cal rather than mag cal: the CH7
/// autotune switch is out of its off position. Safe to reuse because autotune only
/// starts while armed, and only on an off → on transition.
#[must_use]
pub(crate) fn tap_selects_level_cal(
    commands: Option<&elle_control::commands::PilotCommands>,
) -> bool {
    commands.is_some_and(|cmd| match cmd {
        elle_control::commands::PilotCommands::Raw(raw) => {
            raw.channels[elle_config::AUTOTUNE_CH] >= elle_config::AUTOTUNE_OFF_THRESHOLD
        }
        elle_control::commands::PilotCommands::Normalized(_) => false,
    })
}

/// Save PID gains to flash with timeout. Returns true on success.
pub(crate) async fn save_pid_to_flash(data: [u8; 32], context: &str) -> bool {
    if elle_config::IGNORE_PID_FLASH {
        warn!("PID flash ignored — skip save ({})", context);
        return false;
    }

    use elle_config::profile::{FlashRequest, FlashResponse};
    use elle_hardware::flash::{FLASH_REQUEST_SIGNAL, FLASH_RESPONSE_SIGNAL};

    FLASH_REQUEST_SIGNAL.signal(FlashRequest::SavePidProfile { data });
    let save_timeout = Timer::after(FLASH_WRITE_TIMEOUT);
    match embassy_futures::select::select(FLASH_RESPONSE_SIGNAL.wait(), save_timeout).await {
        embassy_futures::select::Either::First(FlashResponse::PidProfileSaved) => {
            elle_hardware::elle_event!(
                info,
                elle_hardware::event::EVT_PID_SAVED,
                "PID gains saved to flash ({})",
                context
            );
            true
        }
        _ => {
            elle_hardware::elle_event!(
                warn,
                elle_hardware::event::EVT_PID_SAVE_FAILED,
                "PID gains flash save failed ({})",
                context
            );
            false
        }
    }
}

/// Remove the saved PID gains; the next boot uses firmware defaults. Only the
/// PID entry goes — mag and level cal stay. No-op while `IGNORE_PID_FLASH` is
/// set, since PID flash is then neither loaded nor saved.
#[cfg(feature = "rpc-control")]
pub(crate) async fn clear_pid_from_flash() {
    use elle_config::profile::{FlashRequest, FlashResponse, ProfileEntry};
    use elle_hardware::flash::{FLASH_REQUEST_SIGNAL, FLASH_RESPONSE_SIGNAL};

    if elle_config::IGNORE_PID_FLASH {
        info!("PID flash ignored — skip clear");
        return;
    }

    FLASH_REQUEST_SIGNAL.signal(FlashRequest::ClearProfileEntry {
        entry: ProfileEntry::Pid,
    });
    let timeout = Timer::after(FLASH_WRITE_TIMEOUT);
    match embassy_futures::select::select(FLASH_RESPONSE_SIGNAL.wait(), timeout).await {
        embassy_futures::select::Either::First(FlashResponse::ProfileEntryCleared) => {
            info!("PID profile cleared from flash");
        }
        _ => {
            warn!("PID profile clear failed or timed out");
        }
    }
}
