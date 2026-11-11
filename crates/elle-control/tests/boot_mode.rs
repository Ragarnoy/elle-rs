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
