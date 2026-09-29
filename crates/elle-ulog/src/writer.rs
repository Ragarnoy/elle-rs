//! ULog writer with buffering support
//!
//! Provides a buffered writer for ULog messages to minimize flash writes.

use crate::format::{
    FLAG_BITS_MSG, InfoMessage, MESSAGE_HEADER_SIZE, MessageHeader, SubscriptionMessage, ULogHeader,
};
use crate::messages::{
    AttitudeMessage, AutotuneMessage, BarometerMessage, CommandsMessage, ControllerMessage,
    Core1LoadMessage, EngineMessage, EscHealthMessage, GnssMessage, ImuRawCtxMessage,
    ImuRawMagMessage, ImuRawMessage, LogEventMessage, LoopStagesMessage, MagnetometerMessage,
    MessageType, NavMessage, PidGainsMessage, StatusMessage,
};
use embassy_time::Instant;
use heapless::Vec;

/// Maximum buffer size for ULog data (4KB)
pub(crate) const BUFFER_SIZE: usize = 4096;

/// Largest data ('D') message payload: 2-byte msg_id plus the serialized message
/// (`imu_raw`, 195 B, is the largest).
const MAX_DATA_PAYLOAD: usize = 256;
const _: () = assert!(2 + crate::messages::ImuRawMessage::SIZE <= MAX_DATA_PAYLOAD);

/// Write errors
#[derive(Debug, Clone, Copy, defmt::Format)]
pub enum WriteError {
    /// Buffer is full
    BufferFull,
    /// Invalid message
    InvalidMessage,
}

/// ULog writer with buffering
pub struct ULogWriter {
    /// Internal buffer
    buffer: Vec<u8, BUFFER_SIZE>,
    /// Next message ID to assign
    next_msg_id: u16,
    /// Whether header has been written
    header_written: bool,
    /// Whether definitions have been written
    definitions_written: bool,
}

impl ULogWriter {
    /// Create a new ULog writer
    #[must_use]
    pub const fn new() -> Self {
        Self {
            buffer: Vec::new(),
            next_msg_id: 0,
            header_written: false,
            definitions_written: false,
        }
    }

    /// Initialize the ULog with header and definitions
    pub fn initialize(&mut self, start_time: Instant) -> Result<(), WriteError> {
        if self.header_written {
            return Ok(());
        }

        // Write header
        let header = ULogHeader::from_instant(start_time);
        let header_bytes = header.to_bytes();
        self.buffer
            .extend_from_slice(&header_bytes)
            .map_err(|_| WriteError::BufferFull)?;

        self.header_written = true;
        Ok(())
    }

    /// Write definitions section (flag bits, formats, info)
    ///
    /// `epoch_ms` is the wall-clock time in ms since UNIX epoch at the start of recording.
    /// Written as `uint64_t sys_start_time_utc_ms` info message (PX4 convention).
    pub fn write_definitions(
        &mut self,
        sys_name: &str,
        ver_hw: &str,
        ver_sw: &str,
        epoch_ms: u64,
    ) -> Result<(), WriteError> {
        if self.definitions_written {
            return Ok(());
        }

        // Write flag bits (type 'B') - use pre-serialized message
        self.buffer
            .extend_from_slice(&FLAG_BITS_MSG)
            .map_err(|_| WriteError::BufferFull)?;

        // Write format definitions (type 'F') - use pre-serialized messages
        self.buffer
            .extend_from_slice(AttitudeMessage::FORMAT_MSG)
            .map_err(|_| WriteError::BufferFull)?;
        self.buffer
            .extend_from_slice(CommandsMessage::FORMAT_MSG)
            .map_err(|_| WriteError::BufferFull)?;
        self.buffer
            .extend_from_slice(StatusMessage::FORMAT_MSG)
            .map_err(|_| WriteError::BufferFull)?;
        self.buffer
            .extend_from_slice(BarometerMessage::FORMAT_MSG)
            .map_err(|_| WriteError::BufferFull)?;
        self.buffer
            .extend_from_slice(MagnetometerMessage::FORMAT_MSG)
            .map_err(|_| WriteError::BufferFull)?;
        self.buffer
            .extend_from_slice(GnssMessage::FORMAT_MSG)
            .map_err(|_| WriteError::BufferFull)?;
        self.buffer
            .extend_from_slice(LogEventMessage::FORMAT_MSG)
            .map_err(|_| WriteError::BufferFull)?;
        self.buffer
            .extend_from_slice(EngineMessage::FORMAT_MSG)
            .map_err(|_| WriteError::BufferFull)?;
        self.buffer
            .extend_from_slice(AutotuneMessage::FORMAT_MSG)
            .map_err(|_| WriteError::BufferFull)?;
        self.buffer
            .extend_from_slice(ControllerMessage::FORMAT_MSG)
            .map_err(|_| WriteError::BufferFull)?;
        self.buffer
            .extend_from_slice(PidGainsMessage::FORMAT_MSG)
            .map_err(|_| WriteError::BufferFull)?;
        self.buffer
            .extend_from_slice(ImuRawMessage::FORMAT_MSG)
            .map_err(|_| WriteError::BufferFull)?;
        self.buffer
            .extend_from_slice(ImuRawMagMessage::FORMAT_MSG)
            .map_err(|_| WriteError::BufferFull)?;
        self.buffer
            .extend_from_slice(ImuRawCtxMessage::FORMAT_MSG)
            .map_err(|_| WriteError::BufferFull)?;
        self.buffer
            .extend_from_slice(EscHealthMessage::FORMAT_MSG)
            .map_err(|_| WriteError::BufferFull)?;
        self.buffer
            .extend_from_slice(Core1LoadMessage::FORMAT_MSG)
            .map_err(|_| WriteError::BufferFull)?;
        self.buffer
            .extend_from_slice(LoopStagesMessage::FORMAT_MSG)
            .map_err(|_| WriteError::BufferFull)?;
        self.buffer
            .extend_from_slice(NavMessage::FORMAT_MSG)
            .map_err(|_| WriteError::BufferFull)?;

        // Write info messages (type 'I')
        self.write_info("char[] sys_name", sys_name)?;
        self.write_info("char[] ver_hw", ver_hw)?;
        self.write_info("char[] ver_sw", ver_sw)?;
        self.write_info_u64("uint64_t sys_start_time_utc_ms", epoch_ms)?;

        self.definitions_written = true;
        Ok(())
    }

    /// Add a subscription for a message type
    pub fn add_subscription(&mut self, message_name: &str) -> Result<u16, WriteError> {
        let msg_id = self.next_msg_id;
        self.next_msg_id += 1;

        let sub = SubscriptionMessage::new(0, msg_id, message_name);
        let mut buf = [0u8; 128];
        let len = sub.to_bytes(&mut buf);
        self.write_message(MessageType::AddLogged, &buf[..len])?;

        Ok(msg_id)
    }

    /// Write a data message with a given msg_id and serialized payload bytes.
    fn write_data_payload(&mut self, msg_id: u16, data_bytes: &[u8]) -> Result<(), WriteError> {
        // 2 bytes for msg_id + data_bytes; use a stack buffer sized for the largest message
        let total = 2 + data_bytes.len();
        if total > MAX_DATA_PAYLOAD {
            return Err(WriteError::InvalidMessage);
        }
        let mut payload = [0u8; MAX_DATA_PAYLOAD];
        payload[0..2].copy_from_slice(&msg_id.to_le_bytes());
        payload[2..total].copy_from_slice(data_bytes);
        self.write_message(MessageType::Data, &payload[..total])
    }

    /// Write an attitude data message
    pub fn write_attitude(
        &mut self,
        msg_id: u16,
        data: &AttitudeMessage,
    ) -> Result<(), WriteError> {
        self.write_data_payload(msg_id, &data.to_bytes())
    }

    /// Write a commands message
    pub fn write_commands(
        &mut self,
        msg_id: u16,
        data: &CommandsMessage,
    ) -> Result<(), WriteError> {
        self.write_data_payload(msg_id, &data.to_bytes())
    }

    /// Write a status message
    pub fn write_status(&mut self, msg_id: u16, data: &StatusMessage) -> Result<(), WriteError> {
        self.write_data_payload(msg_id, &data.to_bytes())
    }

    /// Write a barometer data message
    pub fn write_barometer(
        &mut self,
        msg_id: u16,
        data: &BarometerMessage,
    ) -> Result<(), WriteError> {
        self.write_data_payload(msg_id, &data.to_bytes())
    }

    /// Write a magnetometer data message
    pub fn write_magnetometer(
        &mut self,
        msg_id: u16,
        data: &MagnetometerMessage,
    ) -> Result<(), WriteError> {
        self.write_data_payload(msg_id, &data.to_bytes())
    }

    /// Write a GNSS data message
    pub fn write_gnss(&mut self, msg_id: u16, data: &GnssMessage) -> Result<(), WriteError> {
        self.write_data_payload(msg_id, &data.to_bytes())
    }

    /// Write an engine data message
    pub fn write_engine(&mut self, msg_id: u16, data: &EngineMessage) -> Result<(), WriteError> {
        self.write_data_payload(msg_id, &data.to_bytes())
    }

    /// Write a controller cycle data message
    pub fn write_controller(
        &mut self,
        msg_id: u16,
        data: &ControllerMessage,
    ) -> Result<(), WriteError> {
        self.write_data_payload(msg_id, &data.to_bytes())
    }

    /// Write a PID gains data message
    pub fn write_pid_gains(
        &mut self,
        msg_id: u16,
        data: &PidGainsMessage,
    ) -> Result<(), WriteError> {
        self.write_data_payload(msg_id, &data.to_bytes())
    }

    /// Write an ESC link health message
    pub fn write_esc_health(
        &mut self,
        msg_id: u16,
        data: &EscHealthMessage,
    ) -> Result<(), WriteError> {
        self.write_data_payload(msg_id, &data.to_bytes())
    }

    /// Write a Core 1 load message
    pub fn write_core1_load(
        &mut self,
        msg_id: u16,
        data: &Core1LoadMessage,
    ) -> Result<(), WriteError> {
        self.write_data_payload(msg_id, &data.to_bytes())
    }

    /// Write a loop stage timing message
    pub fn write_loop_stages(
        &mut self,
        msg_id: u16,
        data: &LoopStagesMessage,
    ) -> Result<(), WriteError> {
        self.write_data_payload(msg_id, &data.to_bytes())
    }

    /// Write a navigator output message
    pub fn write_nav(&mut self, msg_id: u16, data: &NavMessage) -> Result<(), WriteError> {
        self.write_data_payload(msg_id, &data.to_bytes())
    }

    /// Write a raw IMU batch message
    pub fn write_imu_raw(&mut self, msg_id: u16, data: &ImuRawMessage) -> Result<(), WriteError> {
        self.write_data_payload(msg_id, &data.to_bytes())
    }

    /// Write a raw IMU mag-change message
    pub fn write_imu_raw_mag(
        &mut self,
        msg_id: u16,
        data: &ImuRawMagMessage,
    ) -> Result<(), WriteError> {
        self.write_data_payload(msg_id, &data.to_bytes())
    }

    /// Write a raw IMU context message
    pub fn write_imu_raw_ctx(
        &mut self,
        msg_id: u16,
        data: &ImuRawCtxMessage,
    ) -> Result<(), WriteError> {
        self.write_data_payload(msg_id, &data.to_bytes())
    }

    /// Write a log event data message
    pub fn write_log_event(
        &mut self,
        msg_id: u16,
        data: &LogEventMessage,
    ) -> Result<(), WriteError> {
        self.write_data_payload(msg_id, &data.to_bytes())
    }

    /// Write an autotune status data message
    pub fn write_autotune(
        &mut self,
        msg_id: u16,
        data: &AutotuneMessage,
    ) -> Result<(), WriteError> {
        self.write_data_payload(msg_id, &data.to_bytes())
    }

    /// Write an info message with a string value
    fn write_info(&mut self, key: &str, value: &str) -> Result<(), WriteError> {
        let msg = InfoMessage::new(key, value);
        let mut buf = [0u8; 256];
        let len = msg.to_bytes(&mut buf);
        self.write_message(MessageType::Info, &buf[..len])
    }

    /// Write an info message with a binary u64 value (little-endian)
    fn write_info_u64(&mut self, key: &str, value: u64) -> Result<(), WriteError> {
        let key_len = key.len() as u8;
        let mut buf = [0u8; 256];
        buf[0] = key_len;
        buf[1..1 + key.len()].copy_from_slice(key.as_bytes());
        let val_start = 1 + key.len();
        buf[val_start..val_start + 8].copy_from_slice(&value.to_le_bytes());
        let total_len = val_start + 8;
        self.write_message(MessageType::Info, &buf[..total_len])
    }

    /// Write a message with header
    fn write_message(&mut self, msg_type: MessageType, payload: &[u8]) -> Result<(), WriteError> {
        if payload.len() > u16::MAX as usize {
            return Err(WriteError::InvalidMessage);
        }

        let header = MessageHeader::new(payload.len() as u16, msg_type as u8);
        let header_bytes = header.to_bytes();

        // Check if we have space
        if self.buffer.len() + MESSAGE_HEADER_SIZE + payload.len() > BUFFER_SIZE {
            return Err(WriteError::BufferFull);
        }

        self.buffer
            .extend_from_slice(&header_bytes)
            .map_err(|_| WriteError::BufferFull)?;
        self.buffer
            .extend_from_slice(payload)
            .map_err(|_| WriteError::BufferFull)?;

        Ok(())
    }

    /// Get the current buffer contents
    #[must_use]
    pub fn buffer(&self) -> &[u8] {
        &self.buffer
    }

    /// Clear the buffer after successful flush
    pub fn clear_buffer(&mut self) {
        self.buffer.clear();
    }

    /// Reset the writer (for new log file)
    pub fn reset(&mut self) {
        self.buffer.clear();
        self.next_msg_id = 0;
        self.header_written = false;
        self.definitions_written = false;
    }
}

impl Default for ULogWriter {
    fn default() -> Self {
        Self::new()
    }
}
