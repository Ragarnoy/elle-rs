//! ULog logger module
//!
//! Provides a high-level interface for logging flight data to flash using ULog format.

use defmt::*;
use elle_config::profile::{ULOG_LOGGER_BUFFER_SIZE, ULOG_WRITE_CHUNK_SIZE};
use elle_error::ULogError;
use embassy_time::Instant;
use heapless::Vec;

use crate::flash::ULOG_WRITE_CHANNEL;
use crate::flash::{ULOG_WRITE_CHANNEL_DEPTH, request_write_ulog, request_write_ulog_blocking};

/// Buffer fill level above which `buffer_writer_output` flushes.
const FLUSH_THRESHOLD: usize = ULOG_LOGGER_BUFFER_SIZE * 3 / 4;

// `flush` sends the whole buffer or nothing, so a full buffer must fit in the
// write channel at once, or every flush of it would be dropped.
const _: () = core::assert!(
    ULOG_LOGGER_BUFFER_SIZE.div_ceil(ULOG_WRITE_CHUNK_SIZE) <= ULOG_WRITE_CHANNEL_DEPTH
);

/// ULog logger state
pub struct ULogLogger {
    /// Reusable ULog writer (avoids 4KB allocation per log call)
    writer: elle_ulog::ULogWriter,
    /// Internal buffer for collecting log data before flushing to channel.
    /// Larger than channel chunk size to reduce flush frequency.
    buffer: Vec<u8, ULOG_LOGGER_BUFFER_SIZE>,
    /// Whether the logger has been initialized
    initialized: bool,
    /// Message IDs for subscriptions
    attitude_msg_id: Option<u16>,
    commands_msg_id: Option<u16>,
    status_msg_id: Option<u16>,
    barometer_msg_id: Option<u16>,
    magnetometer_msg_id: Option<u16>,
    gnss_msg_id: Option<u16>,
    engine_msg_id: Option<u16>,
    log_event_msg_id: Option<u16>,
    autotune_msg_id: Option<u16>,
    controller_msg_id: Option<u16>,
    pid_gains_msg_id: Option<u16>,
    esc_health_msg_id: Option<u16>,
    #[cfg(feature = "gyro-raw-log")]
    gyro_raw_msg_id: Option<u16>,
    /// Gains version last written to this file; `None` right after `initialize`.
    logged_gains_version: Option<u32>,
    /// Log start time
    start_time: Option<Instant>,
    /// When data first had to be dropped, until a flush gets through again.
    /// The next flush starts with a ULog dropout ('O') record covering it.
    dropout_start: Option<Instant>,
}

impl ULogLogger {
    /// Create a new ULog logger
    #[must_use]
    pub fn new() -> Self {
        Self {
            writer: elle_ulog::ULogWriter::new(),
            buffer: Vec::new(),
            initialized: false,
            attitude_msg_id: None,
            commands_msg_id: None,
            status_msg_id: None,
            barometer_msg_id: None,
            magnetometer_msg_id: None,
            gnss_msg_id: None,
            engine_msg_id: None,
            log_event_msg_id: None,
            autotune_msg_id: None,
            controller_msg_id: None,
            pid_gains_msg_id: None,
            esc_health_msg_id: None,
            #[cfg(feature = "gyro-raw-log")]
            gyro_raw_msg_id: None,
            logged_gains_version: None,
            start_time: None,
            dropout_start: None,
        }
    }

    /// Start a fresh log: write header, definitions and subscriptions.
    ///
    /// Call once per log *file*. Every SD `Start` opens a new file, and a ULog
    /// file without its own header is unreadable, so this always starts over —
    /// it is not a one-time init.
    ///
    /// `epoch_ms` is the wall-clock time in ms since UNIX epoch, used for
    /// the `sys_start_time_utc_ms` info message in the ULog header.
    pub async fn initialize(&mut self, epoch_ms: u64) -> Result<(), ULogError> {
        self.writer.reset();
        self.buffer.clear();
        self.initialized = false;
        self.logged_gains_version = None;
        self.dropout_start = None;

        info!("Initializing ULog logger");

        // Import ULog types
        use elle_ulog::{
            AttitudeMessage, AutotuneMessage, BarometerMessage, CommandsMessage, ControllerMessage,
            EngineMessage, GnssMessage, LogEventMessage, MagnetometerMessage, PidGainsMessage,
            StatusMessage,
        };

        let start_time = Instant::now();
        self.start_time = Some(start_time);

        // Initialize writer with header
        self.writer
            .initialize(start_time)
            .map_err(|_| ULogError::InitFailed)?;

        // Write definitions
        self.writer
            .write_definitions(
                "ELLE-RS",
                elle_config::PLATFORM_NAME,
                env!("CARGO_PKG_VERSION"),
                epoch_ms,
            )
            .map_err(|_| ULogError::InitFailed)?;

        // Add subscriptions
        self.attitude_msg_id = Some(
            self.writer
                .add_subscription(AttitudeMessage::NAME)
                .map_err(|_| ULogError::InitFailed)?,
        );
        self.commands_msg_id = Some(
            self.writer
                .add_subscription(CommandsMessage::NAME)
                .map_err(|_| ULogError::InitFailed)?,
        );
        self.status_msg_id = Some(
            self.writer
                .add_subscription(StatusMessage::NAME)
                .map_err(|_| ULogError::InitFailed)?,
        );
        self.barometer_msg_id = Some(
            self.writer
                .add_subscription(BarometerMessage::NAME)
                .map_err(|_| ULogError::InitFailed)?,
        );
        self.magnetometer_msg_id = Some(
            self.writer
                .add_subscription(MagnetometerMessage::NAME)
                .map_err(|_| ULogError::InitFailed)?,
        );
        self.gnss_msg_id = Some(
            self.writer
                .add_subscription(GnssMessage::NAME)
                .map_err(|_| ULogError::InitFailed)?,
        );
        self.engine_msg_id = Some(
            self.writer
                .add_subscription(EngineMessage::NAME)
                .map_err(|_| ULogError::InitFailed)?,
        );
        self.log_event_msg_id = Some(
            self.writer
                .add_subscription(LogEventMessage::NAME)
                .map_err(|_| ULogError::InitFailed)?,
        );
        self.autotune_msg_id = Some(
            self.writer
                .add_subscription(AutotuneMessage::NAME)
                .map_err(|_| ULogError::InitFailed)?,
        );
        self.controller_msg_id = Some(
            self.writer
                .add_subscription(ControllerMessage::NAME)
                .map_err(|_| ULogError::InitFailed)?,
        );
        self.pid_gains_msg_id = Some(
            self.writer
                .add_subscription(PidGainsMessage::NAME)
                .map_err(|_| ULogError::InitFailed)?,
        );
        self.esc_health_msg_id = Some(
            self.writer
                .add_subscription(elle_ulog::EscHealthMessage::NAME)
                .map_err(|_| ULogError::InitFailed)?,
        );
        // Only capture builds write gyro_raw, so only they subscribe to it.
        #[cfg(feature = "gyro-raw-log")]
        {
            self.gyro_raw_msg_id = Some(
                self.writer
                    .add_subscription(elle_ulog::GyroRawMessage::NAME)
                    .map_err(|_| ULogError::InitFailed)?,
            );
        }

        info!(
            "ULog subscriptions: attitude={}, commands={}, status={}, baro={}, mag={}, gnss={}, engine={}, event={}, autotune={}, controller={}, pid_gains={}, esc_health={}",
            self.attitude_msg_id.unwrap(),
            self.commands_msg_id.unwrap(),
            self.status_msg_id.unwrap(),
            self.barometer_msg_id.unwrap(),
            self.magnetometer_msg_id.unwrap(),
            self.gnss_msg_id.unwrap(),
            self.engine_msg_id.unwrap(),
            self.log_event_msg_id.unwrap(),
            self.autotune_msg_id.unwrap(),
            self.controller_msg_id.unwrap(),
            self.pid_gains_msg_id.unwrap(),
            self.esc_health_msg_id.unwrap()
        );
        #[cfg(feature = "gyro-raw-log")]
        info!(
            "ULog subscription: gyro_raw={}",
            self.gyro_raw_msg_id.unwrap()
        );

        // Flush header and definitions to flash
        let data = self.writer.buffer();
        if !data.is_empty() {
            info!("Flushing {} bytes of ULog header to flash", data.len());
            let success = request_write_ulog_blocking(data).await;
            if !success {
                error!("Failed to write ULog header to flash");
                return Err(ULogError::FlushFailed);
            }
            // Clear writer buffer after successful flush
            self.writer.clear_buffer();
        }

        self.initialized = true;
        info!("ULog logger initialized successfully");
        Ok(())
    }

    /// Write a message from the writer buffer into the internal buffer,
    /// flushing to flash if the buffer is getting full.
    fn buffer_writer_output(&mut self) -> Result<(), ULogError> {
        // After a dropped flush the buffer restarts empty; lead it with the
        // dropout record so readers see the gap instead of guessing.
        if self.buffer.is_empty()
            && let Some(start) = self.dropout_start
        {
            let ms = start.elapsed().as_millis().min(u64::from(u16::MAX)) as u16;
            let header =
                elle_ulog::format::MessageHeader::new(2, elle_ulog::MessageType::Dropout as u8);
            let _ = self.buffer.extend_from_slice(&header.to_bytes());
            let _ = self.buffer.extend_from_slice(&ms.to_le_bytes());
        }

        self.buffer
            .extend_from_slice(self.writer.buffer())
            .map_err(|_| ULogError::BufferFull)?;

        if self.buffer.len() > FLUSH_THRESHOLD {
            self.flush()?;
        }

        Ok(())
    }

    /// Log attitude data
    pub fn log_attitude(
        &mut self,
        pitch: f32,
        roll: f32,
        yaw: f32,
        pitch_rate: f32,
        roll_rate: f32,
        yaw_rate: f32,
    ) -> Result<(), ULogError> {
        if !self.initialized {
            return Err(ULogError::NotInitialized);
        }

        let msg = elle_ulog::AttitudeMessage::new(
            Instant::now(),
            pitch,
            roll,
            yaw,
            pitch_rate,
            roll_rate,
            yaw_rate,
        );

        self.writer.clear_buffer();
        self.writer
            .write_attitude(self.attitude_msg_id.unwrap(), &msg)
            .map_err(|_| ULogError::BufferFull)?;

        self.buffer_writer_output()
    }

    /// Log one controller cycle (timestamp is filled in here).
    pub fn log_controller(
        &mut self,
        mut msg: elle_ulog::ControllerMessage,
    ) -> Result<(), ULogError> {
        if !self.initialized {
            return Err(ULogError::NotInitialized);
        }
        msg.timestamp = Instant::now().as_micros();

        self.writer.clear_buffer();
        self.writer
            .write_controller(self.controller_msg_id.unwrap(), &msg)
            .map_err(|_| ULogError::BufferFull)?;

        self.buffer_writer_output()
    }

    /// Log one raw gyro sample (`gyro-raw-log` builds). Keeps its own timestamp.
    #[cfg(feature = "gyro-raw-log")]
    pub fn log_gyro_raw(&mut self, msg: &elle_ulog::GyroRawMessage) -> Result<(), ULogError> {
        if !self.initialized {
            return Err(ULogError::NotInitialized);
        }
        self.writer.clear_buffer();
        self.writer
            .write_gyro_raw(self.gyro_raw_msg_id.unwrap(), msg)
            .map_err(|_| ULogError::BufferFull)?;
        self.buffer_writer_output()
    }

    /// Log the active PID gains if `version` differs from what this file last
    /// recorded (always once per file). Timestamp is filled in here.
    pub fn log_pid_gains(
        &mut self,
        version: u32,
        mut msg: elle_ulog::PidGainsMessage,
    ) -> Result<(), ULogError> {
        if !self.initialized {
            return Err(ULogError::NotInitialized);
        }
        if self.logged_gains_version == Some(version) {
            return Ok(());
        }
        msg.timestamp = Instant::now().as_micros();

        self.writer.clear_buffer();
        self.writer
            .write_pid_gains(self.pid_gains_msg_id.unwrap(), &msg)
            .map_err(|_| ULogError::BufferFull)?;

        self.buffer_writer_output()?;
        self.logged_gains_version = Some(version);
        Ok(())
    }

    /// Log commands data
    #[allow(clippy::too_many_arguments)]
    pub fn log_commands(
        &mut self,
        throttle: f32,
        pitch: f32,
        roll: f32,
        yaw: f32,
        attitude_mode: u8,
        pitch_setpoint_deg: f32,
        roll_setpoint_deg: f32,
        pitch_correction: f32,
        roll_correction: f32,
        elevon_left_us: u32,
        elevon_right_us: u32,
    ) -> Result<(), ULogError> {
        if !self.initialized {
            return Err(ULogError::NotInitialized);
        }

        let msg = elle_ulog::CommandsMessage::new(
            Instant::now(),
            throttle,
            pitch,
            roll,
            yaw,
            attitude_mode,
            pitch_setpoint_deg,
            roll_setpoint_deg,
            pitch_correction,
            roll_correction,
            elevon_left_us,
            elevon_right_us,
        );

        self.writer.clear_buffer();
        self.writer
            .write_commands(self.commands_msg_id.unwrap(), &msg)
            .map_err(|_| ULogError::BufferFull)?;

        self.buffer_writer_output()
    }

    /// Log system status
    pub fn log_status(
        &mut self,
        loop_time_us: u32,
        imu_errors: u32,
        calibrated: bool,
        armed: bool,
        cpu_load: f32,
        rc_age_ms: u16,
    ) -> Result<(), ULogError> {
        if !self.initialized {
            return Err(ULogError::NotInitialized);
        }

        let msg = elle_ulog::StatusMessage::new(
            Instant::now(),
            loop_time_us,
            imu_errors,
            calibrated,
            armed,
            cpu_load,
            rc_age_ms,
        );

        self.writer.clear_buffer();
        self.writer
            .write_status(self.status_msg_id.unwrap(), &msg)
            .map_err(|_| ULogError::BufferFull)?;

        self.buffer_writer_output()
    }

    /// Log barometer data
    pub fn log_barometer(
        &mut self,
        pressure_hpa: f32,
        temperature_c: f32,
        altitude_m: f32,
        vario_ms: f32,
    ) -> Result<(), ULogError> {
        if !self.initialized {
            return Err(ULogError::NotInitialized);
        }

        let msg = elle_ulog::BarometerMessage::new(
            Instant::now(),
            pressure_hpa,
            temperature_c,
            altitude_m,
            vario_ms,
        );

        self.writer.clear_buffer();
        self.writer
            .write_barometer(self.barometer_msg_id.unwrap(), &msg)
            .map_err(|_| ULogError::BufferFull)?;

        self.buffer_writer_output()
    }

    /// Log magnetometer data
    pub fn log_magnetometer(
        &mut self,
        mag_x: f32,
        mag_y: f32,
        mag_z: f32,
    ) -> Result<(), ULogError> {
        if !self.initialized {
            return Err(ULogError::NotInitialized);
        }

        let msg = elle_ulog::MagnetometerMessage::new(Instant::now(), mag_x, mag_y, mag_z);

        self.writer.clear_buffer();
        self.writer
            .write_magnetometer(self.magnetometer_msg_id.unwrap(), &msg)
            .map_err(|_| ULogError::BufferFull)?;

        self.buffer_writer_output()
    }

    /// Log a GNSS solution.
    ///
    /// Takes the whole `GnssData` rather than positional arguments: the record
    /// carries fourteen fields since NAV-PVT was added, and a long argument
    /// list is easy to mis-order silently.
    #[cfg(feature = "gnss")]
    pub fn log_gnss(&mut self, gnss: &crate::gnss::GnssData) -> Result<(), ULogError> {
        if !self.initialized {
            return Err(ULogError::NotInitialized);
        }

        let msg = elle_ulog::GnssMessage::new(
            Instant::now(),
            gnss.latitude,
            gnss.longitude,
            gnss.altitude_m,
            gnss.fix_quality,
            gnss.num_satellites,
            gnss.hdop,
            gnss.vel_n_ms,
            gnss.vel_e_ms,
            gnss.vel_d_ms,
            gnss.ground_speed_ms,
            gnss.heading_motion_deg,
            gnss.h_acc_m,
            gnss.v_acc_m,
            gnss.s_acc_ms,
        );

        self.writer.clear_buffer();
        self.writer
            .write_gnss(self.gnss_msg_id.unwrap(), &msg)
            .map_err(|_| ULogError::BufferFull)?;

        self.buffer_writer_output()
    }

    /// Log engine data from `EngineReading` cache snapshot
    pub fn log_engine(&mut self, eng: &crate::dshot::EngineReading) -> Result<(), ULogError> {
        if !self.initialized {
            return Err(ULogError::NotInitialized);
        }

        let msg = elle_ulog::EngineMessage::new(
            Instant::now(),
            eng.left.erpm,
            eng.right.erpm,
            eng.left.throttle,
            eng.right.throttle,
            eng.left.target_erpm,
            eng.right.target_erpm,
            eng.left.temperature,
            eng.right.temperature,
            eng.left.voltage_mv,
            eng.right.voltage_mv,
            eng.left.current_ma,
            eng.right.current_ma,
        );

        self.writer.clear_buffer();
        self.writer
            .write_engine(self.engine_msg_id.unwrap(), &msg)
            .map_err(|_| ULogError::BufferFull)?;

        self.buffer_writer_output()
    }

    /// Log the ESC link health counters from an `EngineReading` snapshot.
    pub fn log_esc_health(&mut self, eng: &crate::dshot::EngineReading) -> Result<(), ULogError> {
        if !self.initialized {
            return Err(ULogError::NotInitialized);
        }

        let msg = elle_ulog::EscHealthMessage {
            timestamp: Instant::now().as_micros(),
            left_replies: eng.left.replies,
            right_replies: eng.right.replies,
            left_timeouts: eng.left.timeouts,
            right_timeouts: eng.right.timeouts,
            left_bad_frames: eng.left.bad_frames,
            right_bad_frames: eng.right.bad_frames,
            left_reconfigs: eng.left.reconfigs,
            right_reconfigs: eng.right.reconfigs,
        };

        self.writer.clear_buffer();
        self.writer
            .write_esc_health(self.esc_health_msg_id.unwrap(), &msg)
            .map_err(|_| ULogError::BufferFull)?;

        self.buffer_writer_output()
    }

    /// Log a discrete event (level + code)
    pub fn log_event(&mut self, level: u8, code: u16) -> Result<(), ULogError> {
        if !self.initialized {
            return Err(ULogError::NotInitialized);
        }

        let msg = elle_ulog::LogEventMessage::new(Instant::now(), level, code);

        self.writer.clear_buffer();
        self.writer
            .write_log_event(self.log_event_msg_id.unwrap(), &msg)
            .map_err(|_| ULogError::BufferFull)?;

        self.buffer_writer_output()
    }

    /// Log autotune status (called at the control-loop rate during active autotune only)
    #[allow(clippy::too_many_arguments)]
    pub fn log_autotune(
        &mut self,
        phase: u8,
        axis: u8,
        relay_positive: bool,
        setpoint_deg: f32,
        measurement_deg: f32,
        cycles_done: u8,
        amplitude_deg: f32,
    ) -> Result<(), ULogError> {
        if !self.initialized {
            return Err(ULogError::NotInitialized);
        }

        let msg = elle_ulog::AutotuneMessage::new(
            Instant::now(),
            phase,
            axis,
            relay_positive,
            setpoint_deg,
            measurement_deg,
            cycles_done,
            amplitude_deg,
        );

        self.writer.clear_buffer();
        self.writer
            .write_autotune(self.autotune_msg_id.unwrap(), &msg)
            .map_err(|_| ULogError::BufferFull)?;

        self.buffer_writer_output()
    }

    /// Flush buffered data to flash (fire-and-forget — returns immediately).
    /// Splits the buffer into ULOG_WRITE_CHUNK_SIZE messages for the channel.
    pub fn flush(&mut self) -> Result<(), ULogError> {
        if self.buffer.is_empty() {
            return Ok(());
        }

        // All or nothing. The buffer holds whole messages but the chunks cut
        // across them: sending only some chunks would leave a partial message
        // in the file, and the next flush would splice onto it (seen as garbage
        // records after a stall). Drop the whole buffer instead, and mark it.
        let needed = self.buffer.len().div_ceil(ULOG_WRITE_CHUNK_SIZE);
        let mut all_ok = ULOG_WRITE_CHANNEL.free_capacity() >= needed;
        if all_ok {
            for chunk in self.buffer.chunks(ULOG_WRITE_CHUNK_SIZE) {
                if !request_write_ulog(chunk) {
                    // Only this task produces, so capacity can't shrink under us.
                    all_ok = false;
                    break;
                }
            }
        }

        if all_ok {
            self.dropout_start = None;
        } else {
            self.dropout_start.get_or_insert_with(Instant::now);
            error!(
                "ULog flush: channel full, {} bytes dropped",
                self.buffer.len()
            );
        }

        // Clear buffer regardless to avoid re-sending stale data
        self.buffer.clear();
        if all_ok {
            Ok(())
        } else {
            Err(ULogError::FlushFailed)
        }
    }

    /// Check if the logger has been initialized
    #[must_use]
    pub const fn is_initialized(&self) -> bool {
        self.initialized
    }
}

impl Default for ULogLogger {
    fn default() -> Self {
        Self::new()
    }
}
