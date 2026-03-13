pub const FLASH_SIZE: usize = 16 * 1024 * 1024; // 16MB total flash (128 Mbit)

/// Maximum ULog chunk size for flash writes
pub const ULOG_CHUNK_SIZE: usize = 4096;

// Flash operation requests and responses
#[derive(Debug)]
#[allow(clippy::large_enum_variant)] // WriteULogBlocking needs 4KB buffer for inter-core transfer
pub enum FlashRequest {
    /// Blocking ULog write via Signal path — used only for header init where confirmation is needed.
    /// Regular ULog data writes use the fire-and-forget ULOG_WRITE_CHANNEL instead.
    WriteULogBlocking {
        data: [u8; ULOG_CHUNK_SIZE],
        len: usize,
    },
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
