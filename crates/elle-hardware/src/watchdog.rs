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
