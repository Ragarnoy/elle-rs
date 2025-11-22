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
    /// Log start time
    start_time: Option<Instant>,
}

impl ULogLogger {
    /// Create a new ULog logger
    pub fn new() -> Self {
        Self {
            writer: elle_ulog::ULogWriter::new(),
            buffer: Vec::new(),
            initialized: false,
            attitude_msg_id: None,
            commands_msg_id: None,
            status_msg_id: None,
            start_time: None,
        }
    }

    /// Initialize the ULog logger (write header and definitions)
    pub async fn initialize(&mut self) -> Result<(), ()> {
        if self.initialized {
            info!("ULog logger already initialized");
            return Ok(());
        }

        info!("Initializing ULog logger");

        // Import ULog types
        use elle_ulog::{AttitudeMessage, CommandsMessage, StatusMessage};

        let start_time = Instant::now();
        self.start_time = Some(start_time);

        // Initialize writer with header
        self.writer.initialize(start_time).map_err(|_| ())?;

        // Write definitions
        self.writer
            .write_definitions("ELLE-RS", "RP2350-XFly-Eagle", env!("CARGO_PKG_VERSION"))
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

        info!(
            "ULog subscriptions: attitude={}, commands={}, status={}",
            self.attitude_msg_id.unwrap(),
            self.commands_msg_id.unwrap(),
            self.status_msg_id.unwrap()
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

    /// Flush buffered data to flash
    pub async fn flush(&mut self) -> Result<(), ()> {
        if self.buffer.is_empty() {
            return Ok(());
        }

        info!("Flushing {} bytes of ULog data to flash", self.buffer.len());

        let success = request_write_ulog(&self.buffer).await;
        if success {
            self.buffer.clear();
            Ok(())
        } else {
            error!("Failed to flush ULog data to flash");
            Err(())
        }
    }

    /// Check if the logger needs flushing
    pub fn needs_flush(&self) -> bool {
        self.buffer.len() > (ULOG_CHUNK_SIZE / 2)
    }

    /// Get the buffer fill percentage
    pub fn buffer_fill_percent(&self) -> u8 {
        ((self.buffer.len() * 100) / ULOG_CHUNK_SIZE) as u8
    }
}

impl Default for ULogLogger {
    fn default() -> Self {
        Self::new()
    }
}
