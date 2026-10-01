//! Per-tick ULog recording, shared by the flight and RPC loops.

use elle_config::{
    ULOG_BARO_DIVISOR, ULOG_COMMANDS_DIVISOR, ULOG_CORE1_LOAD_DIVISOR, ULOG_ENGINE_DIVISOR,
    ULOG_ESC_HEALTH_DIVISOR, ULOG_MAG_DIVISOR, ULOG_STATUS_DIVISOR,
};
use elle_hardware::ULogLogger;
use elle_hardware::imu::{AttitudeData, BARO, IMU_STATUS, MAG};
use elle_system::{FlightController, TimingMeasurement, update_ulog_timing};

/// Log flight data to ULog flash storage
///
/// Rates (see the `ULOG_*_DIVISOR`s in elle-config):
/// - Attitude, controller cycle: every tick (control-loop rate; attitude 50 Hz in
///   `imu-raw-log` builds, which log every IMU sample as `imu_raw` instead)
/// - Commands, engine telemetry: 100 Hz
/// - PID gains: on change (and once per file)
/// - Status, loop stages: 8 Hz; mag 10 Hz; baro 5 Hz; ESC health, Core 1 load: ~1 Hz
pub(crate) fn log_flight_data(
    logger: &mut ULogLogger,
    attitude: Option<&AttitudeData>,
    commands: &elle_control::commands::PilotCommands,
    loop_counter: u32,
    loop_timer_us: u32,
    fc: &FlightController<'_>,
) {
    use elle_control::commands::PilotCommands;

    // Measure ULog logging performance
    let ulog_timer = TimingMeasurement::start();

    // Attitude at the loop rate (50 Hz in imu-raw-log builds: ULOG_ATTITUDE_DIVISOR)
    if let Some(att) = attitude
        && loop_counter.is_multiple_of(elle_config::ULOG_ATTITUDE_DIVISOR)
    {
        let _ = logger.log_attitude(
            att.pitch,
            att.roll,
            att.yaw,
            att.pitch_rate,
            att.roll_rate,
            att.yaw_rate,
        );
    }

    // Log commands at ULOG_COMMANDS_DIVISOR (100 Hz)
    // Convert to normalized for consistent logging
    let last_out = fc.last_output();
    let log_cmd = |logger: &mut ULogLogger, norm: &elle_control::commands::NormalizedCommands| {
        let _ = logger.log_commands(
            norm.throttle,
            norm.pitch,
            norm.roll,
            norm.yaw,
            norm.attitude_mode as u8,
            last_out.pitch_setpoint_deg,
            last_out.roll_setpoint_deg,
            last_out.pitch_correction,
            last_out.roll_correction,
            last_out.elevon_left_us,
            last_out.elevon_right_us,
        );
    };
    if loop_counter.is_multiple_of(ULOG_COMMANDS_DIVISOR) {
        match commands {
            PilotCommands::Normalized(norm) => log_cmd(logger, norm),
            PilotCommands::Raw(raw) => log_cmd(logger, &raw.to_normalized()),
        }
    }

    // Log what the controller actually did this tick, and the gains it used
    // (the gains only when they changed, or once per file).
    let _ = logger.log_controller(elle_ulog::ControllerMessage {
        timestamp: 0,
        dt_us: last_out.dt_us,
        att_age_us: last_out.att_age_us,
        pitch_sp_deg: last_out.pitch_sp_used_deg,
        roll_sp_deg: last_out.roll_sp_used_deg,
        pitch_p: last_out.pitch_terms.p,
        pitch_i: last_out.pitch_terms.i,
        pitch_d: last_out.pitch_terms.d,
        roll_p: last_out.roll_terms.p,
        roll_i: last_out.roll_terms.i,
        roll_d: last_out.roll_terms.d,
        saturation: last_out.saturation.bits(),
        elevon_left_pulse_us: last_out.elevon_left_pulse_us as u16,
        elevon_right_pulse_us: last_out.elevon_right_pulse_us as u16,
        yaw_damp: last_out.yaw_damp,
    });
    let gains = fc.pid_config();
    let _ = logger.log_pid_gains(
        fc.gains_version(),
        elle_ulog::PidGainsMessage {
            timestamp: 0,
            kp_pitch: gains.kp_pitch,
            ki_pitch: gains.ki_pitch,
            kd_pitch: gains.kd_pitch,
            kp_roll: gains.kp_roll,
            ki_roll: gains.ki_roll,
            kd_roll: gains.kd_roll,
            i_limit: gains.i_limit,
            scale: gains.scale,
        },
    );

    // Raw IMU capture: every record Core 1 queued since the last tick.
    #[cfg(feature = "imu-raw-log")]
    while let Ok(rec) = elle_hardware::imu::IMU_RAW_CHANNEL.try_receive() {
        let _ = logger.log_imu_raw(&rec);
    }

    // Log engine data at ULOG_ENGINE_DIVISOR (100 Hz), ESC link health at ~1 Hz
    let log_engine = loop_counter.is_multiple_of(ULOG_ENGINE_DIVISOR);
    let log_esc_health = loop_counter.is_multiple_of(ULOG_ESC_HEALTH_DIVISOR);
    if log_engine || log_esc_health {
        let eng = elle_hardware::dshot::ENGINE_CACHE.lock(|c| c.get());
        if log_engine {
            let _ = logger.log_engine(&eng);
        }
        if log_esc_health {
            let _ = logger.log_esc_health(&eng);
        }
    }

    // Core 1 load at ~1 Hz: the window the loop closed at the start of this tick.
    if loop_counter.is_multiple_of(ULOG_CORE1_LOAD_DIVISOR) {
        let load = elle_hardware::timing::core1_last_window();
        let _ = logger.log_core1_load(&load);
    }

    // Log status at reduced rate (~8Hz)
    if loop_counter.is_multiple_of(ULOG_STATUS_DIVISOR) {
        let imu_status = IMU_STATUS.try_read();
        let _ = logger.log_status(
            loop_timer_us,
            imu_status.as_ref().map(|s| s.error_count).unwrap_or(0),
            imu_status.as_ref().map(|s| s.calibrated).unwrap_or(false),
            fc.is_armed(),
            0.0, // CPU load - could calculate from timing data
            fc.rc_signal_age_ms(),
        );
    }

    // Log barometer at ~2Hz
    if loop_counter.is_multiple_of(ULOG_BARO_DIVISOR) {
        let baro = BARO.read_cached();
        let _ = logger.log_barometer(
            baro.pressure_hpa,
            baro.temperature_c,
            baro.altitude_m,
            baro.vario_ms,
        );
    }

    // Log magnetometer at ~10Hz
    if loop_counter.is_multiple_of(ULOG_MAG_DIVISOR) {
        let mag = MAG.read_cached();
        let _ = logger.log_magnetometer(mag.x as f32, mag.y as f32, mag.z as f32);
    }

    // Drain event channel into ULog
    while let Ok((level, code)) = elle_hardware::event::ULOG_EVENT_CHANNEL.try_receive() {
        let _ = logger.log_event(level, code);
    }

    // Update performance monitoring
    update_ulog_timing(ulog_timer.elapsed_us());
}

/// Log each new GNSS solution (5 Hz on NAV-PVT) and the navigator's output
/// (`nav`, 25 Hz), from `NavObserver::tick`.
#[cfg(feature = "gnss")]
pub(crate) fn log_nav(logger: &mut ULogLogger, tick: &crate::nav::NavTick) {
    if let Some(gnss) = &tick.gnss {
        let _ = logger.log_gnss(gnss);
    }
    if let Some(msg) = tick.nav {
        let _ = logger.log_nav(msg);
    }
}
