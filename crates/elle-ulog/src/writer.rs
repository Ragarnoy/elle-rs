//! ULog writer with buffering support
//!
//! Provides a buffered writer for ULog messages to minimize flash writes.

use crate::format::{
    InfoMessage, MessageHeader, SubscriptionMessage, ULogHeader, FLAG_BITS_MSG,
};
use crate::messages::{AttitudeMessage, CommandsMessage, MessageType, StatusMessage};
use embassy_time::Instant;
use heapless::Vec;

/// Maximum buffer size for ULog data (4KB)
pub const BUFFER_SIZE: usize = 4096;

/// Write errors
#[derive(Debug, Clone, Copy, defmt::Format)]
pub enum WriteError {
    /// Buffer is full
    BufferFull,
    /// Invalid message
    InvalidMessage,
    /// Flash write failed
    FlashError,
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
    pub fn new() -> Self {
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
    pub fn write_definitions(
        &mut self,
        sys_name: &str,
        ver_hw: &str,
        ver_sw: &str,
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

        // Write info messages (type 'I')
        self.write_info("char[] sys_name", sys_name)?;
        self.write_info("char[] ver_hw", ver_hw)?;
        self.write_info("char[] ver_sw", ver_sw)?;

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

    /// Write an attitude data message
    pub fn write_attitude(
        &mut self,
        msg_id: u16,
        data: &AttitudeMessage,
    ) -> Result<(), WriteError> {
        let mut payload = [0u8; AttitudeMessage::SIZE + 2];
        payload[0..2].copy_from_slice(&msg_id.to_le_bytes());
        payload[2..].copy_from_slice(&data.to_bytes());
        self.write_message(MessageType::Data, &payload)
    }

    /// Write a commands message
    pub fn write_commands(
        &mut self,
        msg_id: u16,
        data: &CommandsMessage,
    ) -> Result<(), WriteError> {
        let mut payload = [0u8; CommandsMessage::SIZE + 2];
        payload[0..2].copy_from_slice(&msg_id.to_le_bytes());
        payload[2..].copy_from_slice(&data.to_bytes());
        self.write_message(MessageType::Data, &payload)
    }

    /// Write a status message
    pub fn write_status(&mut self, msg_id: u16, data: &StatusMessage) -> Result<(), WriteError> {
        let mut payload = [0u8; StatusMessage::SIZE + 2];
        payload[0..2].copy_from_slice(&msg_id.to_le_bytes());
        payload[2..].copy_from_slice(&data.to_bytes());
        self.write_message(MessageType::Data, &payload)
    }

    /// Write an info message
    fn write_info(&mut self, key: &str, value: &str) -> Result<(), WriteError> {
        let msg = InfoMessage::new(key, value);
        let mut buf = [0u8; 256];
        let len = msg.to_bytes(&mut buf);
        self.write_message(MessageType::Info, &buf[..len])
    }

    /// Write a message with header
    fn write_message(&mut self, msg_type: MessageType, payload: &[u8]) -> Result<(), WriteError> {
        if payload.len() > u16::MAX as usize {
            return Err(WriteError::InvalidMessage);
        }

        let header = MessageHeader::new(payload.len() as u16, msg_type as u8);
        let header_bytes = header.to_bytes();

        // Check if we have space
        if self.buffer.len() + 3 + payload.len() > BUFFER_SIZE {
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
    pub fn buffer(&self) -> &[u8] {
        &self.buffer
    }

    /// Get the buffer size
    pub fn buffer_len(&self) -> usize {
        self.buffer.len()
    }

    /// Check if buffer needs flushing (>75% full)
    pub fn needs_flush(&self) -> bool {
        self.buffer.len() > (BUFFER_SIZE * 3 / 4)
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
