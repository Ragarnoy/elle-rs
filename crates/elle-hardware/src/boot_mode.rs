//! This boot's mode (proposal 0002), read from the watchdog block once, first
//! thing in `main()`, and held in a static every task reads.
//!
//! The decision itself is `elle_control::boot_mode::decide` (host-tested). Here
//! are the registers: two scratch words that survive the forced reset into
//! ESC-config mode, and the watchdog's reset reason. Both words are cleared on
//! read, so any later reset is a normal boot.

use core::sync::atomic::{AtomicBool, Ordering};
pub use elle_control::boot_mode::BootMode;
use elle_control::boot_mode::{ResetCause, decide};
use embassy_rp::watchdog::{ResetReason, Watchdog};

static ESC_CONFIG: AtomicBool = AtomicBool::new(false);

/// Read and clear the request, decide the mode, and remember it.
///
/// Only builds with the `esc-config` feature can enter ESC-config mode; the
/// others still clear the words and always boot normally.
pub fn init(watchdog: &mut Watchdog<'_>) -> BootMode {
    let (s0, s1) = (watchdog.scratch(0), watchdog.scratch(1));
    let cause = match watchdog.reset_reason() {
        Some(ResetReason::Forced) => ResetCause::Forced,
        Some(ResetReason::TimedOut) => ResetCause::TimedOut,
        None => ResetCause::Other,
    };
    watchdog.set_scratch(0, 0);
    watchdog.set_scratch(1, 0);
    let mode = if cfg!(feature = "esc-config") {
        decide(s0, s1, cause)
    } else {
        BootMode::Normal
    };
    ESC_CONFIG.store(mode == BootMode::EscConfig, Ordering::Relaxed);
    mode
}

/// The mode [`init`] decided.
pub fn current() -> BootMode {
    if ESC_CONFIG.load(Ordering::Relaxed) {
        BootMode::EscConfig
    } else {
        BootMode::Normal
    }
}

/// Whether this is an ESC-config boot.
pub fn is_esc_config() -> bool {
    current() == BootMode::EscConfig
}
