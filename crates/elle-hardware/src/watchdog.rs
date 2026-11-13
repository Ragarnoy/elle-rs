//! The hardware watchdog, shared between the control loop and the flash manager.
//!
//! The control loop feeds it every tick with the normal short timeout. A flash
//! operation pauses Core 1 and runs blocking erase/program routines on Core 0,
//! so the loop cannot feed it for as long as the operation takes (a PID or
//! profile write can run well past the 500 ms timeout). The flash manager
//! therefore feeds a long budget before each request. Flash writes are only
//! issued while disarmed, so the long budget never covers flight.

use core::cell::RefCell;
use embassy_rp::watchdog::Watchdog;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::Duration;

static WATCHDOG: Mutex<CriticalSectionRawMutex, RefCell<Option<Watchdog<'static>>>> =
    Mutex::new(RefCell::new(None));

/// Start the watchdog with `timeout` and make it available to [`feed`].
pub fn install(mut watchdog: Watchdog<'static>, timeout: Duration) {
    watchdog.start(timeout);
    WATCHDOG.lock(|w| *w.borrow_mut() = Some(watchdog));
}

/// Reload the watchdog with `timeout`. No-op until [`install`] has run.
pub fn feed(timeout: Duration) {
    WATCHDOG.lock(|w| {
        if let Some(wd) = w.borrow_mut().as_mut() {
            wd.feed(timeout);
        }
    });
}

/// Reboot into `mode` now, through the installed watchdog (a forced reset).
///
/// For [`BootMode::EscConfig`](crate::boot_mode::BootMode) the request goes in
/// watchdog scratch 0 and 1 first; `boot_mode::init` reads it back after the
/// reset. Only call it disarmed: it cuts every output. Does nothing before
/// [`install`].
pub fn reboot_into(mode: crate::boot_mode::BootMode) {
    WATCHDOG.lock(|w| {
        if let Some(wd) = w.borrow_mut().as_mut() {
            let (s0, s1) = match mode {
                crate::boot_mode::BootMode::EscConfig => {
                    elle_control::boot_mode::esc_config_words()
                }
                crate::boot_mode::BootMode::Normal => (0, 0),
            };
            wd.set_scratch(0, s0);
            wd.set_scratch(1, s1);
            wd.trigger_reset();
        }
    });
}
