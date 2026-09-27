pub const FLASH_SIZE: usize = 16 * 1024 * 1024; // 16MB total flash (128 Mbit)

/// Maximum ULog chunk size for flash reads/extraction
pub const ULOG_CHUNK_SIZE: usize = 4096;

/// ULog write chunk size for channel messages. Each channel message carries up to
/// this many bytes. The flash manager batches multiple messages into a single
/// `queue.push()` to minimize `in_ram()` pause/resume cycles.
pub const ULOG_WRITE_CHUNK_SIZE: usize = 512;

/// ULog logger internal buffer size. Larger buffer = fewer flushes = fewer channel
/// messages = more data batched per flash write. At ~133 bytes/iteration (83 Hz),
/// a 2KB buffer flushes roughly every 11 iterations (~7 flushes/sec).
pub const ULOG_LOGGER_BUFFER_SIZE: usize = 2048;

/// An entry in the flash profile map (sequential-storage `MapStorage`). Each
/// setting lives under its own key; clearing one removes only that key, and the
/// boot loader falls back to firmware defaults when a key is absent.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ProfileEntry {
    /// PID gains, 32 bytes
    Pid,
    /// Magnetometer hard-iron offsets, 12 bytes
    MagCal,
    /// Level-cal mount quaternion, 16 bytes
    LevelCal,
}

impl ProfileEntry {
    /// Map key. Stored data is addressed by these numbers — never renumber.
    #[must_use]
    pub const fn key(self) -> u8 {
        match self {
            Self::Pid => 1,
            Self::MagCal => 2,
            Self::LevelCal => 3,
        }
    }
}

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
    /// Remove one entry from the profile map, leaving the others intact
    ClearProfileEntry {
        entry: ProfileEntry,
    },
    SaveMagCal {
        data: [u8; 12],
    },
    LoadMagCal,
    SaveLevelCal {
        data: [u8; 16],
    },
    LoadLevelCal,
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
    ProfileEntryCleared,
    ProfileEntryClearFailed,
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
    LevelCalSaved,
    LevelCalSaveFailed,
    LevelCalLoaded {
        data: [u8; 16],
    },
    LevelCalEmpty,
}
