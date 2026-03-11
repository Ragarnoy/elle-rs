//! ULog logger module
//!
//! Provides a high-level interface for logging flight data to flash using ULog format.

use defmt::*;
use elle_config::profile::ULOG_CHUNK_SIZE;
use embassy_time::Instant;
use heapless::Vec;

use crate::sequential_flash_manager::request_write_ulog;

/// ULog logger state
pub struct ULogLogger {
    /// Reusable ULog writer (avoids 4KB allocation per log call)
    writer: elle_ulog::ULogWriter,
    /// Internal buffer for collecting log data
    buffer: Vec<u8, ULOG_CHUNK_SIZE>,
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
    /// Log start time
    start_time: Option<Instant>,
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
            start_time: None,
        }
    }

    /// Initialize the ULog logger (write header and definitions)
    ///
    /// `epoch_ms` is the wall-clock time in ms since UNIX epoch, used for
    /// the `sys_start_time_utc_ms` info message in the ULog header.
    pub async fn initialize(&mut self, epoch_ms: u64) -> Result<(), ()> {
        if self.initialized {
            info!("ULog logger already initialized");
            return Ok(());
        }

        info!("Initializing ULog logger");

        // Import ULog types
        use elle_ulog::{
            AttitudeMessage, BarometerMessage, CommandsMessage, EngineMessage, GnssMessage,
            LogEventMessage, MagnetometerMessage, StatusMessage,
        };

        let start_time = Instant::now();
        self.start_time = Some(start_time);

        // Initialize writer with header
        self.writer.initialize(start_time).map_err(|_| ())?;

        // Write definitions
        self.writer
            .write_definitions("ELLE-RS", "RP2350-XFly-Eagle", env!("CARGO_PKG_VERSION"), epoch_ms)
            .map_err(|_| ())?;

        // Add subscriptions
        self.attitude_msg_id = Some(
            self.writer
                .add_subscription(AttitudeMessage::NAME)
                .map_err(|_| ())?,
        );
        self.commands_msg_id = Some(
            self.writer
                .add_subscription(CommandsMessage::NAME)
                .map_err(|_| ())?,
        );
        self.status_msg_id = Some(
            self.writer
                .add_subscription(StatusMessage::NAME)
                .map_err(|_| ())?,
        );
        self.barometer_msg_id = Some(
            self.writer
                .add_subscription(BarometerMessage::NAME)
                .map_err(|_| ())?,
        );
        self.magnetometer_msg_id = Some(
            self.writer
                .add_subscription(MagnetometerMessage::NAME)
                .map_err(|_| ())?,
        );
        self.gnss_msg_id = Some(
            self.writer
                .add_subscription(GnssMessage::NAME)
                .map_err(|_| ())?,
        );
        self.engine_msg_id = Some(
            self.writer
                .add_subscription(EngineMessage::NAME)
                .map_err(|_| ())?,
        );
        self.log_event_msg_id = Some(
            self.writer
                .add_subscription(LogEventMessage::NAME)
                .map_err(|_| ())?,
        );

        info!(
            "ULog subscriptions: attitude={}, commands={}, status={}, baro={}, mag={}, gnss={}, engine={}, event={}",
            self.attitude_msg_id.unwrap(),
            self.commands_msg_id.unwrap(),
            self.status_msg_id.unwrap(),
            self.barometer_msg_id.unwrap(),
            self.magnetometer_msg_id.unwrap(),
            self.gnss_msg_id.unwrap(),
            self.engine_msg_id.unwrap(),
            self.log_event_msg_id.unwrap()
        );

        // Flush header and definitions to flash
        let data = self.writer.buffer();
        if !data.is_empty() {
            info!("Flushing {} bytes of ULog header to flash", data.len());
            let success = request_write_ulog(data).await;
            if !success {
                error!("Failed to write ULog header to flash");
                return Err(());
            }
            // Clear writer buffer after successful flush
            self.writer.clear_buffer();
        }

        self.initialized = true;
        info!("ULog logger initialized successfully");
        Ok(())
    }

    /// Log attitude data
    pub async fn log_attitude(
        &mut self,
        pitch: f32,
        roll: f32,
        yaw: f32,
        pitch_rate: f32,
        roll_rate: f32,
        yaw_rate: f32,
    ) -> Result<(), ()> {
        if !self.initialized {
            return Err(());
        }

        use elle_ulog::AttitudeMessage;

        let msg = AttitudeMessage::new(
            Instant::now(),
            pitch,
            roll,
            yaw,
            pitch_rate,
            roll_rate,
            yaw_rate,
        );

        // Clear writer and write message
        self.writer.clear_buffer();
        self.writer
            .write_attitude(self.attitude_msg_id.unwrap(), &msg)
            .map_err(|_| ())?;

        // Add to buffer
        self.buffer
            .extend_from_slice(self.writer.buffer())
            .map_err(|_| ())?;

        // Flush if buffer is getting full
        if self.buffer.len() > (ULOG_CHUNK_SIZE * 3 / 4) {
            self.flush().await?;
        }

        Ok(())
    }

    /// Log commands data
    #[allow(clippy::too_many_arguments)]
    pub async fn log_commands(
        &mut self,
        throttle: f32,
        pitch: f32,
        roll: f32,
        yaw: f32,
        attitude_mode: u8,
        pitch_setpoint_deg: f32,
        roll_setpoint_deg: f32,
    ) -> Result<(), ()> {
        if !self.initialized {
            return Err(());
        }

        use elle_ulog::CommandsMessage;

        let msg = CommandsMessage::new(
            Instant::now(),
            throttle,
            pitch,
            roll,
            yaw,
            attitude_mode,
            pitch_setpoint_deg,
            roll_setpoint_deg,
        );

        // Clear writer and write message
        self.writer.clear_buffer();
        self.writer
            .write_commands(self.commands_msg_id.unwrap(), &msg)
            .map_err(|_| ())?;

        // Add to buffer
        self.buffer
            .extend_from_slice(self.writer.buffer())
            .map_err(|_| ())?;

        // Flush if buffer is getting full
        if self.buffer.len() > (ULOG_CHUNK_SIZE * 3 / 4) {
            self.flush().await?;
        }

        Ok(())
    }

    /// Log system status
    pub async fn log_status(
        &mut self,
        loop_time_us: u32,
        imu_errors: u32,
        calibrated: bool,
        armed: bool,
        cpu_load: f32,
    ) -> Result<(), ()> {
        if !self.initialized {
            return Err(());
        }

        use elle_ulog::StatusMessage;

        let msg = StatusMessage::new(
            Instant::now(),
            loop_time_us,
            imu_errors,
            calibrated,
            armed,
            cpu_load,
        );

        // Clear writer and write message
        self.writer.clear_buffer();
        self.writer
            .write_status(self.status_msg_id.unwrap(), &msg)
            .map_err(|_| ())?;

        // Add to buffer
        self.buffer
            .extend_from_slice(self.writer.buffer())
            .map_err(|_| ())?;

        // Flush if buffer is getting full
        if self.buffer.len() > (ULOG_CHUNK_SIZE * 3 / 4) {
            self.flush().await?;
        }

        Ok(())
    }

    /// Log barometer data
    pub async fn log_barometer(
        &mut self,
        pressure_hpa: f32,
        temperature_c: f32,
        altitude_m: f32,
    ) -> Result<(), ()> {
        if !self.initialized {
            return Err(());
        }

        use elle_ulog::BarometerMessage;

        let msg = BarometerMessage::new(Instant::now(), pressure_hpa, temperature_c, altitude_m);

        self.writer.clear_buffer();
        self.writer
            .write_barometer(self.barometer_msg_id.unwrap(), &msg)
            .map_err(|_| ())?;

        self.buffer
            .extend_from_slice(self.writer.buffer())
            .map_err(|_| ())?;

        if self.buffer.len() > (ULOG_CHUNK_SIZE * 3 / 4) {
            self.flush().await?;
        }

        Ok(())
    }

    /// Log magnetometer data
    pub async fn log_magnetometer(&mut self, mag_x: f32, mag_y: f32, mag_z: f32) -> Result<(), ()> {
        if !self.initialized {
            return Err(());
        }

        use elle_ulog::MagnetometerMessage;

        let msg = MagnetometerMessage::new(Instant::now(), mag_x, mag_y, mag_z);

        self.writer.clear_buffer();
        self.writer
            .write_magnetometer(self.magnetometer_msg_id.unwrap(), &msg)
            .map_err(|_| ())?;

        self.buffer
            .extend_from_slice(self.writer.buffer())
            .map_err(|_| ())?;

        if self.buffer.len() > (ULOG_CHUNK_SIZE * 3 / 4) {
            self.flush().await?;
        }

        Ok(())
    }

    /// Log GNSS data
    #[allow(clippy::too_many_arguments)]
    pub async fn log_gnss(
        &mut self,
        latitude: f32,
        longitude: f32,
        altitude_m: f32,
        fix_quality: u8,
        num_satellites: u8,
        hdop: f32,
    ) -> Result<(), ()> {
        if !self.initialized {
            return Err(());
        }

        use elle_ulog::GnssMessage;

        let msg = GnssMessage::new(
            Instant::now(),
            latitude,
            longitude,
            altitude_m,
            fix_quality,
            num_satellites,
            hdop,
        );

        self.writer.clear_buffer();
        self.writer
            .write_gnss(self.gnss_msg_id.unwrap(), &msg)
            .map_err(|_| ())?;

        self.buffer
            .extend_from_slice(self.writer.buffer())
            .map_err(|_| ())?;

        if self.buffer.len() > (ULOG_CHUNK_SIZE * 3 / 4) {
            self.flush().await?;
        }

        Ok(())
    }

    /// Log engine data from `EngineReading` cache snapshot
    pub async fn log_engine(
        &mut self,
        eng: &crate::dshot::EngineReading,
    ) -> Result<(), ()> {
        if !self.initialized {
            return Err(());
        }

        use elle_ulog::EngineMessage;

        let msg = EngineMessage::new(
            Instant::now(),
            eng.left_erpm,
            eng.right_erpm,
            eng.left_throttle,
            eng.right_throttle,
            eng.left_target_erpm,
            eng.right_target_erpm,
            eng.left_temperature,
            eng.right_temperature,
            eng.left_voltage_mv,
            eng.right_voltage_mv,
            eng.left_current_ma,
            eng.right_current_ma,
        );

        self.writer.clear_buffer();
        self.writer
            .write_engine(self.engine_msg_id.unwrap(), &msg)
            .map_err(|_| ())?;

        self.buffer
            .extend_from_slice(self.writer.buffer())
            .map_err(|_| ())?;

        if self.buffer.len() > (ULOG_CHUNK_SIZE * 3 / 4) {
            self.flush().await?;
        }

        Ok(())
    }

    /// Log a discrete event (level + code)
    pub async fn log_event(&mut self, level: u8, code: u16) -> Result<(), ()> {
        if !self.initialized {
            return Err(());
        }

        use elle_ulog::LogEventMessage;

        let msg = LogEventMessage::new(Instant::now(), level, code);

        self.writer.clear_buffer();
        self.writer
            .write_log_event(self.log_event_msg_id.unwrap(), &msg)
            .map_err(|_| ())?;

        self.buffer
            .extend_from_slice(self.writer.buffer())
            .map_err(|_| ())?;

        if self.buffer.len() > (ULOG_CHUNK_SIZE * 3 / 4) {
            self.flush().await?;
        }

        Ok(())
    }

    /// Flush buffered data to flash
    pub async fn flush(&mut self) -> Result<(), ()> {
        if self.buffer.is_empty() {
            return Ok(());
        }

        let success = request_write_ulog(&self.buffer).await;
        if success {
            self.buffer.clear();
            Ok(())
        } else {
            error!("ULog flush failed");
            Err(())
        }
    }

    /// Check if the logger has been initialized
    #[must_use]
    pub fn is_initialized(&self) -> bool {
        self.initialized
    }

    /// Check if the logger needs flushing
    #[must_use]
    pub fn needs_flush(&self) -> bool {
        self.buffer.len() > (ULOG_CHUNK_SIZE / 2)
    }

    /// Get the buffer fill percentage
    #[must_use]
    pub fn buffer_fill_percent(&self) -> u8 {
        ((self.buffer.len() * 100) / ULOG_CHUNK_SIZE) as u8
    }
}

impl Default for ULogLogger {
    fn default() -> Self {
        Self::new()
    }
}
