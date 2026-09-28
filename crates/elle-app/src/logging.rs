//! Per-tick ULog recording, shared by the flight and RPC loops.

use elle_config::{
    ULOG_BARO_DIVISOR, ULOG_CORE1_LOAD_DIVISOR, ULOG_ESC_HEALTH_DIVISOR, ULOG_MAG_DIVISOR,
    ULOG_STATUS_DIVISOR,
};
use elle_hardware::ULogLogger;
use elle_hardware::imu::{AttitudeData, BARO, IMU_STATUS, MAG};
use elle_system::{FlightController, TimingMeasurement, update_ulog_timing};

/// Log flight data to ULog flash storage
///
/// Logs attitude, commands, and periodic status updates at appropriate rates:
/// - Attitude: control-loop rate (every call)
/// - Commands: control-loop rate (every call)
/// - Controller cycle: control-loop rate; PID gains on change
/// - Status: ~8Hz (every 10th call)
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

    // Log attitude data at control-loop rate
    if let Some(att) = attitude {
        let _ = logger.log_attitude(
            att.pitch,
            att.roll,
            att.yaw,
            att.pitch_rate,
            att.roll_rate,
            att.yaw_rate,
        );
    }

    // Log commands at control-loop rate
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
    match commands {
        PilotCommands::Normalized(norm) => log_cmd(logger, norm),
        PilotCommands::Raw(raw) => log_cmd(logger, &raw.to_normalized()),
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

    // Bench vibration capture: every 1 kHz gyro sample queued by Core1.
    #[cfg(feature = "gyro-raw-log")]
    while let Ok(s) = elle_hardware::imu::GYRO_RAW_CHANNEL.try_receive() {
        let _ = logger.log_gyro_raw(&elle_ulog::GyroRawMessage {
            timestamp: s.timestamp.as_micros(),
            gyro_x: s.gyro[0],
            gyro_y: s.gyro[1],
            gyro_z: s.gyro[2],
        });
    }

    // Log engine data at control-loop rate, ESC link health at ~1 Hz
    {
        let eng = elle_hardware::dshot::ENGINE_CACHE.lock(|c| c.get());
        let _ = logger.log_engine(&eng);
        if loop_counter.is_multiple_of(ULOG_ESC_HEALTH_DIVISOR) {
            let _ = logger.log_esc_health(&eng);
        }
    }

    // Core 1 load at ~1 Hz; each record covers the window since the previous one.
    if loop_counter.is_multiple_of(ULOG_CORE1_LOAD_DIVISOR) {
        let load = elle_hardware::timing::CORE1_LOAD.take();
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

    // Log GNSS at ~1Hz — feature-gated
    #[cfg(feature = "gnss")]
    if loop_counter.is_multiple_of(elle_config::ULOG_GNSS_DIVISOR)
        && let Some(gnss) = elle_hardware::gnss::GNSS_SIGNAL.try_take()
    {
        elle_hardware::gnss::GNSS_SIGNAL.signal(gnss); // put back for other readers
        let _ = logger.log_gnss(&gnss);
    }

    // Drain event channel into ULog
    while let Ok((level, code)) = elle_hardware::event::ULOG_EVENT_CHANNEL.try_receive() {
        let _ = logger.log_event(level, code);
    }

    // Update performance monitoring
    update_ulog_timing(ulog_timer.elapsed_us());
}
