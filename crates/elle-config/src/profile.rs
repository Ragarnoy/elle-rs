pub const FLASH_SIZE: usize = 16 * 1024 * 1024; // 16MB total flash (128 Mbit)

/// Maximum ULog chunk size for flash reads/extraction
pub const ULOG_CHUNK_SIZE: usize = 4096;

/// ULog write chunk size — smaller than read chunks to avoid 4KB stack allocations
/// in the fire-and-forget write path. Header writes are split into multiple chunks.
pub const ULOG_WRITE_CHUNK_SIZE: usize = 512;

// Flash operation requests and responses
#[derive(Debug)]
pub enum FlashRequest {
    PeekULog,
    PopULog,
    EraseULog,
    SavePidProfile {
        data: [u8; 32],
    },
    LoadPidProfile,
    SaveMagCal {
        data: [u8; 12],
    },
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
