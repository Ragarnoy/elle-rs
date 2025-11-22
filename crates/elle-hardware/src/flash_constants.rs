//! Constants for flash memory operations
//!
//! These constants define the characteristics of the flash memory used in the system.
//! Using these values correctly helps ensure reliable flash operations and reduces the chance of errors.

/// Flash page size.
/// This is the smallest unit that can be programmed at once.
pub const PAGE_SIZE: usize = 256;

/// Flash write size.
/// This is the smallest unit that can be written at once.
pub const WRITE_SIZE: usize = 1;

/// Flash read size.
/// This is the smallest unit that can be read at once.
pub const READ_SIZE: usize = 1;

/// Flash erase size.
/// This is the size of a flash sector that must be erased at once.
pub const ERASE_SIZE: usize = 4096;

/// Flash DMA read size.
/// This is the optimal size for DMA read operations.
pub const ASYNC_READ_SIZE: usize = 4;

/// Flash memory layout (16MB total = 0x000000 - 0xFFFFFF):
///     - 0x000000 - 0x1FFFFF: Program code (2MB)
///     - 0x200000 - 0x20FFFF: Calibration storage (64KB)
///     - 0x210000 - 0xFFFFFF: ULog storage (14,680,064 bytes ≈ 14MB)
///     - ULog capacity: ~14MB ÷ 6KB/s = ~2,446 seconds ≈ 40 minutes of flight logging
///
/// Calibration flash region start
pub const CALIBRATION_FLASH_START: u32 = 0x200000;

/// Calibration flash region end
pub const CALIBRATION_FLASH_END: u32 = 0x20FFFF;

/// Calibration flash region size (64KB)
pub const CALIBRATION_FLASH_SIZE: usize =
    (CALIBRATION_FLASH_END - CALIBRATION_FLASH_START + 1) as usize;

/// ULog flash region start (after calibration area)
pub const ULOG_FLASH_START: u32 = 0x210000;

/// ULog flash region end (end of flash)
pub const ULOG_FLASH_END: u32 = 0xFFFFFF;

/// ULog flash region size (~14MB)
pub const ULOG_FLASH_SIZE: usize = (ULOG_FLASH_END - ULOG_FLASH_START + 1) as usize;
