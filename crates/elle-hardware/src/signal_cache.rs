//! Combined Signal + Mutex<Cell<>> for publish-subscribe with non-consuming reads.
//!
//! Replaces the repeated pattern of maintaining a Signal (for async consumers) and
//! a Mutex<Cell<>> cache (for non-consuming reads) side-by-side.

use core::cell::Cell;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

/// A combined signal + cache for sharing data between tasks.
///
/// - `publish()` updates both the signal and cache atomically.
/// - `read_cached()` reads the cache without consuming (for RPC handlers, telemetry).
/// - `try_take()` consumes the signal (for the primary consumer).
pub struct SignalCache<T: Copy + Send> {
    signal: Signal<CriticalSectionRawMutex, T>,
    cache: Mutex<CriticalSectionRawMutex, Cell<T>>,
}

impl<T: Copy + Send> SignalCache<T> {
    pub const fn new(default: T) -> Self {
        Self {
            signal: Signal::new(),
            cache: Mutex::new(Cell::new(default)),
        }
    }

    /// Publish a value: updates both the signal and cache.
    pub fn publish(&self, value: T) {
        self.signal.signal(value);
        self.cache.lock(|c| c.set(value));
    }

    /// Read the cached value (non-consuming).
    pub fn read_cached(&self) -> T {
        self.cache.lock(|c| c.get())
    }

    /// Try to take the signal value (consuming, non-blocking).
    pub fn try_take(&self) -> Option<T> {
        self.signal.try_take()
    }
}
