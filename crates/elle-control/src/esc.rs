//! AM32 ESC settings checks (proposal 0002), host-testable.

use elle_config::{ESC_BOOTLOADER_VERSION_OFFSET, ESC_EEPROM_SIZE};

/// First offset where a written ESC settings block and its read-back differ,
/// ignoring `ESC_BOOTLOADER_VERSION_OFFSET`: the bootloader writes its own
/// version there on every write at the EEPROM base.
pub fn esc_write_mismatch(
    written: &[u8; ESC_EEPROM_SIZE],
    read: &[u8; ESC_EEPROM_SIZE],
) -> Option<usize> {
    (0..ESC_EEPROM_SIZE).find(|&i| i != ESC_BOOTLOADER_VERSION_OFFSET && written[i] != read[i])
}
