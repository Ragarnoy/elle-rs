pub const FLASH_SIZE: usize = 16 * 1024 * 1024; // 16MB total flash (128 Mbit)

/// Maximum ULog chunk size for flash reads/extraction
pub const ULOG_CHUNK_SIZE: usize = 4096;

/// ULog write chunk size for channel messages. Each channel message carries up to
/// this many bytes; the SD writer drains them into the open log file.
pub const ULOG_WRITE_CHUNK_SIZE: usize = 512;

/// ULog logger internal buffer size: records batch here and go to the channel in
/// 512 B chunks at 75 % full. At ~17.7 kB/s that is ~9 flushes a second.
pub const ULOG_LOGGER_BUFFER_SIZE: usize = 2048;
/// Chunks queued between the logger and the SD writer: 64 × 512 B = 32 KB, about
/// 1.8 s of recording at the ~17.7 kB/s measured in LOG_0058. It has to cover the
/// card's internal busy periods (garbage collection, FAT updates), which run to a
/// few hundred ms: with 8 slots (4 KB, ~230 ms) LOG_0058 lost 95–180 ms of records
/// four times in two seconds while the loop kept running. Static RAM, ~33 KB.
#[cfg(not(feature = "imu-raw-log"))]
pub const ULOG_WRITE_CHANNEL_DEPTH: usize = 64;
/// `imu-raw-log` builds add ~20 kB/s of raw IMU records on top of the ~30 kB/s
/// every build logs: twice the queue (~64 KB, ~1.3 s at ~50 kB/s) to ride out
/// the card's slow phase.
#[cfg(feature = "imu-raw-log")]
pub const ULOG_WRITE_CHANNEL_DEPTH: usize = 128;

/// SD card sector size: the FAT layer's block size and the unit the SD writer
/// keeps the file position aligned to.
pub const SD_SECTOR_SIZE: usize = 512;

/// Bytes the SD writer collects before a write: 8 sectors per card command.
///
/// A write reaches the card as one multi-block command (CMD25) only when the
/// file position is sector-aligned and the length is a whole number of sectors;
/// otherwise every sector is read back and written on its own (CMD17 + CMD24 +
/// CMD13 and one program-busy wait each; LOG_0091 got ~27 kB/s onto the card
/// that way, of ~47 kB/s produced). The writer batches queued chunks up to this
/// size and writes whole sectors only (`elle_control::write_batch`). A batch
/// only fills when chunks are waiting; with an empty queue it goes out as it
/// is, so data waits no longer than before. The ~47 kB/s of an `imu-raw-log`
/// build is under 3 full batches a second. Static RAM, 16 KB: 4 KB still lost
/// records while armed (LOG_0092), larger multi-block writes cost the writer
/// less Core 0 time per byte.
pub const SD_WRITE_BATCH_SIZE: usize = 16384;

// Whole sectors, and room for a chunk next to a kept tail (< 1 sector): the
// writer takes a chunk off the queue only when a whole one still fits.
const _: () = assert!(SD_WRITE_BATCH_SIZE.is_multiple_of(SD_SECTOR_SIZE));
const _: () = assert!(SD_WRITE_BATCH_SIZE >= SD_SECTOR_SIZE + ULOG_WRITE_CHUNK_SIZE);

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
