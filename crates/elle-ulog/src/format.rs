//! ULog file format structures and constants
//!
//! Implements the PX4 ULog binary format specification.

use embassy_time::Instant;

/// ULog magic bytes: "ULog" followed by version marker [0x01, 0x12, 0x35]
pub(crate) const ULOG_MAGIC: [u8; 7] = [0x55, 0x4c, 0x6f, 0x67, 0x01, 0x12, 0x35];

/// ULog format version (currently 1)
pub(crate) const ULOG_VERSION: u8 = 1;

/// ULog file header (16 bytes total)
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub(crate) struct ULogHeader {
    /// Magic bytes "ULog" + version marker
    magic: [u8; 7],
    /// Format version (1)
    version: u8,
    /// Timestamp in microseconds since epoch
    timestamp: u64,
}

impl ULogHeader {
    /// Create a new ULog header with the given timestamp
    #[must_use]
    pub const fn new(timestamp_us: u64) -> Self {
        Self {
            magic: ULOG_MAGIC,
            version: ULOG_VERSION,
            timestamp: timestamp_us,
        }
    }

    /// Create a new ULog header with timestamp from current Instant
    #[must_use]
    pub fn from_instant(instant: Instant) -> Self {
        Self::new(instant.as_micros())
    }

    /// Serialize the header to a byte buffer (16 bytes)
    #[must_use]
    pub fn to_bytes(self) -> [u8; 16] {
        let mut buf = [0u8; 16];
        buf[0..7].copy_from_slice(&self.magic);
        buf[7] = self.version;
        buf[8..16].copy_from_slice(&self.timestamp.to_le_bytes());
        buf
    }
}

/// Message header (3 bytes)
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct MessageHeader {
    /// Payload size (excluding this header)
    msg_size: u16,
    /// Message type identifier (single ASCII character)
    msg_type: u8,
}

impl MessageHeader {
    /// Create a new message header
    #[must_use]
    pub const fn new(msg_size: u16, msg_type: u8) -> Self {
        Self { msg_size, msg_type }
    }

    /// Serialize to bytes (3 bytes)
    #[must_use]
    pub fn to_bytes(&self) -> [u8; 3] {
        let mut buf = [0u8; 3];
        buf[0..2].copy_from_slice(&self.msg_size.to_le_bytes());
        buf[2] = self.msg_type;
        buf
    }
}

/// Pre-serialized default FlagBits message (header + payload, computed at compile time)
/// Header: msg_size=40 (0x28), msg_type='B'
/// Payload: all zeros (compat_flags, incompat_flags, appended_offsets)
pub(crate) const FLAG_BITS_MSG: [u8; 43] = [
    0x28, 0x00, b'B', // Header: size=40, type='B'
    0, 0, 0, 0, 0, 0, 0, 0, // compat_flags
    0, 0, 0, 0, 0, 0, 0, 0, // incompat_flags
    0, 0, 0, 0, 0, 0, 0, 0, // appended_offsets[0]
    0, 0, 0, 0, 0, 0, 0, 0, // appended_offsets[1]
    0, 0, 0, 0, 0, 0, 0, 0, // appended_offsets[2]
];

/// Information message key-value pair (type 'I')
#[derive(Debug, Clone)]
pub(crate) struct InfoMessage<'a> {
    key: &'a str,
    value: &'a str,
}

impl<'a> InfoMessage<'a> {
    /// Create a new info message
    #[must_use]
    pub fn new(key: &'a str, value: &'a str) -> Self {
        Self { key, value }
    }

    /// Serialize to bytes
    pub fn to_bytes(&self, buf: &mut [u8]) -> usize {
        let key_len = self.key.len() as u8;
        buf[0] = key_len;
        buf[1..1 + self.key.len()].copy_from_slice(self.key.as_bytes());
        buf[1 + self.key.len()..1 + self.key.len() + self.value.len()]
            .copy_from_slice(self.value.as_bytes());
        1 + self.key.len() + self.value.len()
    }
}

/// Subscription message (type 'A') - declares a message instance for logging
#[derive(Debug, Clone)]
pub(crate) struct SubscriptionMessage<'a> {
    /// Multi-instance ID (0 for primary)
    multi_id: u8,
    /// Unique message ID
    msg_id: u16,
    /// Message name (must match a format definition)
    message_name: &'a str,
}

impl<'a> SubscriptionMessage<'a> {
    /// Create a new subscription
    #[must_use]
    pub fn new(multi_id: u8, msg_id: u16, message_name: &'a str) -> Self {
        Self {
            multi_id,
            msg_id,
            message_name,
        }
    }

    /// Serialize to bytes
    pub fn to_bytes(&self, buf: &mut [u8]) -> usize {
        buf[0] = self.multi_id;
        buf[1..3].copy_from_slice(&self.msg_id.to_le_bytes());
        buf[3..3 + self.message_name.len()].copy_from_slice(self.message_name.as_bytes());
        3 + self.message_name.len()
    }
}
