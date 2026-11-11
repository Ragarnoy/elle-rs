//! Which mode a boot runs in (proposal 0002).
//!
//! A normal boot flies or runs the ground-test loop. An ESC-config boot (RPC
//! builds only) puts the one-wire UART on the engine pins instead of DShot, so
//! the ESCs can be configured through their AM32 bootloader. The request to
//! enter it survives one reboot in two watchdog scratch registers; this module
//! decides, from what they held and why the chip reset, which mode this boot is.

use elle_config::ESC_CONFIG_MAGIC;

/// The mode of this boot.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum BootMode {
    /// DShot on the engine pins; the normal flight or RPC loop.
    #[default]
    Normal,
    /// One-wire UART on the engine pins, no DShot, arming locked.
    EscConfig,
}

/// Why the chip last reset, as the watchdog block reports it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ResetCause {
    /// Power-on, probe or reset pin: the watchdog did not cause it.
    Other,
    /// Software asked the watchdog for a reset (`trigger_reset`).
    Forced,
    /// The watchdog timed out (the loop stopped feeding it).
    TimedOut,
}

/// The two scratch words that request an ESC-config boot.
pub const fn esc_config_words() -> (u32, u32) {
    (ESC_CONFIG_MAGIC, !ESC_CONFIG_MAGIC)
}

/// Decide this boot's mode from the scratch words and the reset cause.
///
/// ESC-config only for the exact magic pair **and** a forced reset. A watchdog
/// timeout (a hung loop, possibly in flight) or a power-on never enters it,
/// whatever the registers hold, and the caller clears the words so the boot
/// after this one is normal.
pub fn decide(scratch0: u32, scratch1: u32, cause: ResetCause) -> BootMode {
    if (scratch0, scratch1) == esc_config_words() && cause == ResetCause::Forced {
        BootMode::EscConfig
    } else {
        BootMode::Normal
    }
}

/// Bytes of the AM32 settings structure.
pub const ESC_EEPROM_SIZE: usize = 192;

/// First offset where a written ESC settings block and its read-back differ,
/// ignoring byte 2: the bootloader writes its own version there on every write
/// at the EEPROM base.
pub fn esc_write_mismatch(
    written: &[u8; ESC_EEPROM_SIZE],
    read: &[u8; ESC_EEPROM_SIZE],
) -> Option<usize> {
    (0..ESC_EEPROM_SIZE).find(|&i| i != 2 && written[i] != read[i])
}
