//! Execution-time statistics for tasks that live in this crate (the Core 1 IMU
//! loop, the flash manager). `elle-system`'s performance monitor cannot be called
//! from here — it depends on this crate, not the other way round — so these are
//! lock-free atomics it reads instead. One writer per instance; `reset` from
//! another core may race a concurrent `record`, which is fine for diagnostics.

use core::sync::atomic::{AtomicU32, Ordering};

/// Min / running-average / max execution time in µs, plus a sample count.
pub struct AtomicTiming {
    min_us: AtomicU32,
    avg_us: AtomicU32,
    max_us: AtomicU32,
    samples: AtomicU32,
}

/// A point-in-time copy of an [`AtomicTiming`].
#[derive(Clone, Copy, Default)]
pub struct TimingSnapshot {
    pub min_us: u32,
    pub avg_us: u32,
    pub max_us: u32,
    pub samples: u32,
}

impl AtomicTiming {
    #[must_use]
    pub const fn new() -> Self {
        Self {
            min_us: AtomicU32::new(u32::MAX),
            avg_us: AtomicU32::new(0),
            max_us: AtomicU32::new(0),
            samples: AtomicU32::new(0),
        }
    }

    /// Record one execution (call from the timed task only).
    pub fn record(&self, elapsed_us: u32) {
        let n = self.samples.load(Ordering::Relaxed);
        self.min_us.fetch_min(elapsed_us, Ordering::Relaxed);
        self.max_us.fetch_max(elapsed_us, Ordering::Relaxed);
        // Same running average as elle-system's TaskTiming: equal weights for the
        // first 100 samples, then an EMA with 1/100 weight.
        let avg = if n == 0 {
            elapsed_us
        } else {
            let weight = (n + 1).min(100);
            let prev = self.avg_us.load(Ordering::Relaxed);
            ((u64::from(prev) * u64::from(weight - 1) + u64::from(elapsed_us)) / u64::from(weight))
                as u32
        };
        self.avg_us.store(avg, Ordering::Relaxed);
        self.samples.store(n.saturating_add(1), Ordering::Relaxed);
    }

    #[must_use]
    pub fn snapshot(&self) -> TimingSnapshot {
        TimingSnapshot {
            min_us: self.min_us.load(Ordering::Relaxed),
            avg_us: self.avg_us.load(Ordering::Relaxed),
            max_us: self.max_us.load(Ordering::Relaxed),
            samples: self.samples.load(Ordering::Relaxed),
        }
    }

    pub fn reset(&self) {
        self.min_us.store(u32::MAX, Ordering::Relaxed);
        self.avg_us.store(0, Ordering::Relaxed);
        self.max_us.store(0, Ordering::Relaxed);
        self.samples.store(0, Ordering::Relaxed);
    }
}

impl Default for AtomicTiming {
    fn default() -> Self {
        Self::new()
    }
}

/// Core 1 busy time per IMU DATA_RDY wake-up: FIFO drain, fusion, publish and
/// the mag/baro/tap housekeeping that follows it. One wake-up per 1 kHz sample.
pub static IMU_TIMING: AtomicTiming = AtomicTiming::new();

/// Duration of each flash manager request (profile load/save, ULog peek/pop/erase).
pub static FLASH_TIMING: AtomicTiming = AtomicTiming::new();
