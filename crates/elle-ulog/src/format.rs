//! ULog file format structures and constants
//!
//! Implements the PX4 ULog binary format specification.

use embassy_time::Instant;

/// ULog magic bytes: "ULog" followed by version marker [0x01, 0x12, 0x35]
pub const ULOG_MAGIC: [u8; 7] = [0x55, 0x4c, 0x6f, 0x67, 0x01, 0x12, 0x35];

/// ULog format version (currently 1)
pub const ULOG_VERSION: u8 = 1;

/// ULog file header (16 bytes total)
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct ULogHeader {
    /// Magic bytes "ULog" + version marker
    pub magic: [u8; 7],
    /// Format version (1)
    pub version: u8,
    /// Timestamp in microseconds since epoch
    pub timestamp: u64,
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
    pub fn to_bytes(&self) -> [u8; 16] {
        let mut buf = [0u8; 16];
        buf[0..7].copy_from_slice(&self.magic);
        buf[7] = self.version;
        buf[8..16].copy_from_slice(&self.timestamp.to_le_bytes());
        buf
    }

    /// Deserialize a header from bytes
    #[must_use]
    pub fn from_bytes(buf: &[u8; 16]) -> Option<Self> {
        if buf[0..7] != ULOG_MAGIC {
            return None;
        }
        let version = buf[7];
        if version != ULOG_VERSION {
            return None;
        }
        let timestamp = u64::from_le_bytes([
            buf[8], buf[9], buf[10], buf[11], buf[12], buf[13], buf[14], buf[15],
        ]);
        Some(Self {
            magic: ULOG_MAGIC,
            version,
            timestamp,
        })
    }
}

/// Message header (3 bytes)
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct MessageHeader {
    /// Payload size (excluding this header)
    pub msg_size: u16,
    /// Message type identifier (single ASCII character)
    pub msg_type: u8,
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

    /// Deserialize from bytes
    #[must_use]
    pub fn from_bytes(buf: &[u8; 3]) -> Self {
        let msg_size = u16::from_le_bytes([buf[0], buf[1]]);
        let msg_type = buf[2];
        Self { msg_size, msg_type }
    }
}

/// Flag bits message (type 'B') - must be first after header
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct FlagBits {
    /// Compatible features (backwards compatible changes)
    pub compat_flags: [u8; 8],
    /// Incompatible features (breaking changes)
    pub incompat_flags: [u8; 8],
    /// Offsets for appended data sections
    pub appended_offsets: [u64; 3],
}

/// Pre-serialized default FlagBits message (header + payload, computed at compile time)
/// Header: msg_size=40 (0x28), msg_type='B'
/// Payload: all zeros (compat_flags, incompat_flags, appended_offsets)
pub const FLAG_BITS_MSG: [u8; 43] = [
    0x28, 0x00, b'B', // Header: size=40, type='B'
    0, 0, 0, 0, 0, 0, 0, 0, // compat_flags
    0, 0, 0, 0, 0, 0, 0, 0, // incompat_flags
    0, 0, 0, 0, 0, 0, 0, 0, // appended_offsets[0]
    0, 0, 0, 0, 0, 0, 0, 0, // appended_offsets[1]
    0, 0, 0, 0, 0, 0, 0, 0, // appended_offsets[2]
];

impl FlagBits {
    /// Create new flag bits with default values
    #[must_use]
    pub const fn new() -> Self {
        Self {
            compat_flags: [0; 8],
            incompat_flags: [0; 8],
            appended_offsets: [0; 3],
        }
    }

    /// Serialize to bytes (40 bytes)
    #[must_use]
    pub fn to_bytes(&self) -> [u8; 40] {
        let mut buf = [0u8; 40];
        buf[0..8].copy_from_slice(&self.compat_flags);
        buf[8..16].copy_from_slice(&self.incompat_flags);
        for (i, offset) in self.appended_offsets.iter().enumerate() {
            buf[16 + i * 8..16 + (i + 1) * 8].copy_from_slice(&offset.to_le_bytes());
        }
        buf
    }
}

impl Default for FlagBits {
    fn default() -> Self {
        Self::new()
    }
}

/// Information message key-value pair (type 'I')
#[derive(Debug, Clone)]
pub struct InfoMessage<'a> {
    pub key: &'a str,
    pub value: &'a str,
}

impl<'a> InfoMessage<'a> {
    /// Create a new info message
    #[must_use]
    pub fn new(key: &'a str, value: &'a str) -> Self {
        Self { key, value }
    }

    /// Calculate total message size (excluding header)
    #[must_use]
    pub fn msg_size(&self) -> u16 {
        (1 + self.key.len() + self.value.len()) as u16
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

/// Format definition message (type 'F')
#[derive(Debug, Clone)]
pub struct FormatMessage<'a> {
    /// Format string: "message_name:field0;field1;...;fieldN"
    pub format: &'a str,
}

impl<'a> FormatMessage<'a> {
    /// Create a new format message
    #[must_use]
    pub fn new(format: &'a str) -> Self {
        Self { format }
    }

    /// Calculate message size
    #[must_use]
    pub fn msg_size(&self) -> u16 {
        self.format.len() as u16
    }

    /// Serialize to bytes
    pub fn to_bytes(&self, buf: &mut [u8]) -> usize {
        buf[..self.format.len()].copy_from_slice(self.format.as_bytes());
        self.format.len()
    }
}

/// Subscription message (type 'A') - declares a message instance for logging
#[derive(Debug, Clone)]
pub struct SubscriptionMessage<'a> {
    /// Multi-instance ID (0 for primary)
    pub multi_id: u8,
    /// Unique message ID
    pub msg_id: u16,
    /// Message name (must match a format definition)
    pub message_name: &'a str,
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

    /// Calculate message size
    #[must_use]
    pub fn msg_size(&self) -> u16 {
        (3 + self.message_name.len()) as u16
    }

    /// Serialize to bytes
    pub fn to_bytes(&self, buf: &mut [u8]) -> usize {
        buf[0] = self.multi_id;
        buf[1..3].copy_from_slice(&self.msg_id.to_le_bytes());
        buf[3..3 + self.message_name.len()].copy_from_slice(self.message_name.as_bytes());
        3 + self.message_name.len()
    }
}
