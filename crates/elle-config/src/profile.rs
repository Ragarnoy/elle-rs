pub const FLASH_SIZE: usize = 16 * 1024 * 1024; // 16MB total flash (128 Mbit)

/// Maximum ULog chunk size for flash writes
pub const ULOG_CHUNK_SIZE: usize = 4096;

// Flash operation requests and responses
#[derive(Debug)]
#[allow(clippy::large_enum_variant)] // WriteULog needs 4KB buffer for inter-core transfer
pub enum FlashRequest {
    WriteULog {
        data: [u8; ULOG_CHUNK_SIZE],
        len: usize,
    },
    PeekULog,
    PopULog,
    EraseULog,
}

#[derive(Clone, Copy, Debug)]
#[allow(clippy::large_enum_variant)] // Intentional: no_std + Copy, transferred via Signal
pub enum FlashResponse {
    ULogWriteSuccess,
    ULogWriteFailed,
    ULogData {
        data: [u8; ULOG_CHUNK_SIZE],
        len: usize,
    },
    ULogEmpty,
    ULogPopSuccess,
    ULogEraseSuccess,
    ULogEraseFailed,
}
