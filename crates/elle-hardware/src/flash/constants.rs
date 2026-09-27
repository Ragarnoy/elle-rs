//! Constants for flash memory operations
//!
//! These constants define the characteristics of the flash memory used in the system.
//! Using these values correctly helps ensure reliable flash operations and reduces the chance of errors.

/// Flash erase size.
/// This is the size of a flash sector that must be erased at once.
pub(crate) const ERASE_SIZE: usize = 4096;

/// Flash memory layout (16MB total = 0x000000 - 0xFFFFFF):
///     - 0x000000 - 0x1FFFFF: Program code (2MB)
///     - 0x200000 - 0x20FFFF: Reserved (64KB)
///     - 0x210000 - 0xFFFFFF: ULog storage (14,680,064 bytes ≈ 14MB)
///     - ULog capacity: ~14MB ÷ 6KB/s = ~2,446 seconds ≈ 40 minutes of flight logging
///
/// PID profile flash region start (64KB for map storage)
pub(crate) const PROFILE_FLASH_START: u32 = 0x200000;

/// PID profile flash region end
pub(crate) const PROFILE_FLASH_END: u32 = 0x210000;

/// ULog flash region start
pub(crate) const ULOG_FLASH_START: u32 = 0x210000;

/// ULog flash region end (end of flash, inclusive)
pub(crate) const ULOG_FLASH_END: u32 = 0xFFFFFF;

/// ULog flash region end (exclusive, for range-based APIs like sequential-storage)
pub(crate) const ULOG_FLASH_END_EXCL: u32 = 0x1000000;

/// ULog flash region size (~14MB)
pub const ULOG_FLASH_SIZE: usize = (ULOG_FLASH_END - ULOG_FLASH_START + 1) as usize;

// Layout invariants. sequential-storage and the erase path need every region
// boundary on an erase-sector edge; the two regions must not overlap and must
// fit in the part.
const _: () = assert!(ULOG_FLASH_END_EXCL == ULOG_FLASH_END + 1);
const _: () = assert!(PROFILE_FLASH_START < PROFILE_FLASH_END);
const _: () = assert!(PROFILE_FLASH_END <= ULOG_FLASH_START);
const _: () = assert!(ULOG_FLASH_END_EXCL as usize <= elle_config::profile::FLASH_SIZE);
const _: () = assert!((PROFILE_FLASH_START as usize).is_multiple_of(ERASE_SIZE));
const _: () = assert!((PROFILE_FLASH_END as usize).is_multiple_of(ERASE_SIZE));
const _: () = assert!((ULOG_FLASH_START as usize).is_multiple_of(ERASE_SIZE));
const _: () = assert!((ULOG_FLASH_END_EXCL as usize).is_multiple_of(ERASE_SIZE));
