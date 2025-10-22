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
