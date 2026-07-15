pub const FLASH_SIZE: usize = 16 * 1024 * 1024; // 16MB total flash (128 Mbit)

/// Maximum ULog chunk size for flash reads/extraction
pub const ULOG_CHUNK_SIZE: usize = 4096;

/// ULog write chunk size for channel messages. Each channel message carries up to
/// this many bytes. The flash manager batches multiple messages into a single
/// `queue.push()` to minimize `in_ram()` pause/resume cycles.
pub const ULOG_WRITE_CHUNK_SIZE: usize = 512;

/// ULog logger internal buffer size. Larger buffer = fewer flushes = fewer channel
/// messages = more data batched per flash write. At ~133 bytes/iteration (77Hz),
/// a 2KB buffer flushes roughly every 11 iterations (~7 flushes/sec).
pub const ULOG_LOGGER_BUFFER_SIZE: usize = 2048;

// Flash operation requests and responses
#[derive(Debug)]
pub enum FlashRequest {
    PeekULog,
    PopULog,
    EraseULog,
    SavePidProfile { data: [u8; 32] },
    LoadPidProfile,
    ErasePidProfile,
    SaveMagCal { data: [u8; 12] },
    LoadMagCal,
}

#[derive(Clone, Copy, Debug)]
#[allow(clippy::large_enum_variant)] // Intentional: no_std + Copy, transferred via Signal
pub enum FlashResponse {
    ULogData {
        data: [u8; ULOG_CHUNK_SIZE],
        len: usize,
    },
    ULogEmpty,
    ULogPopSuccess,
    ULogEraseSuccess,
    ULogEraseFailed,
    PidProfileSaved,
    PidProfileSaveFailed,
    PidProfileErased,
    PidProfileEraseFailed,
    PidProfileLoaded {
        data: [u8; 32],
    },
    PidProfileEmpty,
    MagCalSaved,
    MagCalSaveFailed,
    MagCalLoaded {
        data: [u8; 12],
    },
    MagCalEmpty,
}
