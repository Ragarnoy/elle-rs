//! ESC-config boot selection (proposal 0002): only the exact scratch pair after a
//! forced reset enters it.
//!
//!   cargo test -p elle-control --target x86_64-unknown-linux-gnu --test boot_mode

use elle_control::boot_mode::{BootMode, ResetCause, decide, esc_config_words};

#[test]
fn magic_pair_after_forced_reset_enters_esc_config() {
    let (a, b) = esc_config_words();
    assert_eq!(decide(a, b, ResetCause::Forced), BootMode::EscConfig);
}

#[test]
fn watchdog_timeout_or_power_on_is_normal() {
    let (a, b) = esc_config_words();
    assert_eq!(decide(a, b, ResetCause::TimedOut), BootMode::Normal);
    assert_eq!(decide(a, b, ResetCause::Other), BootMode::Normal);
}

#[test]
fn wrong_words_are_normal() {
    let (a, b) = esc_config_words();
    assert_eq!(decide(a, a, ResetCause::Forced), BootMode::Normal);
    assert_eq!(decide(0, 0, ResetCause::Forced), BootMode::Normal);
    assert_eq!(decide(a ^ 1, b, ResetCause::Forced), BootMode::Normal);
    assert_eq!(decide(b, a, ResetCause::Forced), BootMode::Normal);
}

#[test]
fn words_are_complements() {
    let (a, b) = esc_config_words();
    assert_eq!(a, !b);
}

#[test]
fn esc_write_mismatch_ignores_the_bootloader_version_byte() {
    use elle_control::boot_mode::{ESC_EEPROM_SIZE, esc_write_mismatch};
    let written = [0x11u8; ESC_EEPROM_SIZE];
    let mut read = written;
    read[2] = 0x99;
    assert_eq!(esc_write_mismatch(&written, &read), None);
    read[40] = 0;
    assert_eq!(esc_write_mismatch(&written, &read), Some(40));
    read[0] = 0;
    assert_eq!(esc_write_mismatch(&written, &read), Some(0));
}
