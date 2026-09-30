//! Execution-time statistics for tasks that live in this crate (the Core 1 IMU
//! loop, the flash manager). `CORE1_LOAD` is always on and goes to ULog; the
//! `AtomicTiming` instances feed `performance-monitoring` builds only. `elle-system`'s performance monitor cannot be called
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

/// Core 1 load over one logging window, accumulated by the IMU and I2C sensor
/// tasks and taken (and reset) by the ULog writer on Core 0 about once a second.
///
/// Busy time is wall time per IMU DATA_RDY wake-up, from the wake to the next
/// wait: FIFO drain, fusion, publish, tap polling. The deadline is one sample
/// period (1 ms); a wake that runs past it leaves samples queued, which shows up
/// as `max_drain` > 1. The mag and baro read durations come from the separate
/// I2C task and are not part of the busy time.
pub struct Core1Load {
    wakes: AtomicU32,
    busy_sum_us: AtomicU32,
    busy_max_us: AtomicU32,
    mag_max_us: AtomicU32,
    baro_max_us: AtomicU32,
    max_drain: AtomicU32,
}

/// One window of [`Core1Load`].
#[derive(Clone, Copy, Default, defmt::Format)]
pub struct Core1LoadSnapshot {
    pub wakes: u32,
    pub busy_sum_us: u32,
    pub busy_max_us: u32,
    /// Longest MMC5616WA read (I2C0, in `i2c_sensors`, not the IMU wake).
    pub mag_max_us: u32,
    /// Longest BMP390 read (I2C0, in `i2c_sensors`, not the IMU wake).
    pub baro_max_us: u32,
    /// Most FIFO samples drained in one wake-up.
    pub max_drain: u32,
}

impl Core1LoadSnapshot {
    /// Mean busy time per wake-up in the window.
    #[must_use]
    pub const fn busy_avg_us(&self) -> u32 {
        match self.busy_sum_us.checked_div(self.wakes) {
            Some(avg) => avg,
            None => 0,
        }
    }
}

impl Core1Load {
    #[must_use]
    pub const fn new() -> Self {
        Self {
            wakes: AtomicU32::new(0),
            busy_sum_us: AtomicU32::new(0),
            busy_max_us: AtomicU32::new(0),
            mag_max_us: AtomicU32::new(0),
            baro_max_us: AtomicU32::new(0),
            max_drain: AtomicU32::new(0),
        }
    }

    /// One DATA_RDY wake-up took `busy_us`.
    pub fn record_wake(&self, busy_us: u32) {
        self.wakes.fetch_add(1, Ordering::Relaxed);
        self.busy_sum_us.fetch_add(busy_us, Ordering::Relaxed);
        self.busy_max_us.fetch_max(busy_us, Ordering::Relaxed);
    }

    pub fn record_mag(&self, us: u32) {
        self.mag_max_us.fetch_max(us, Ordering::Relaxed);
    }

    pub fn record_baro(&self, us: u32) {
        self.baro_max_us.fetch_max(us, Ordering::Relaxed);
    }

    pub fn record_drain(&self, samples: u32) {
        self.max_drain.fetch_max(samples, Ordering::Relaxed);
    }

    /// Read the window and start a new one. A wake-up recorded between the
    /// individual swaps lands half in each window, which is fine for diagnostics.
    pub fn take(&self) -> Core1LoadSnapshot {
        Core1LoadSnapshot {
            wakes: self.wakes.swap(0, Ordering::Relaxed),
            busy_sum_us: self.busy_sum_us.swap(0, Ordering::Relaxed),
            busy_max_us: self.busy_max_us.swap(0, Ordering::Relaxed),
            mag_max_us: self.mag_max_us.swap(0, Ordering::Relaxed),
            baro_max_us: self.baro_max_us.swap(0, Ordering::Relaxed),
            max_drain: self.max_drain.swap(0, Ordering::Relaxed),
        }
    }
}

impl Default for Core1Load {
    fn default() -> Self {
        Self::new()
    }
}

/// Core 1 IMU task load, logged to ULog as `core1_load`.
pub static CORE1_LOAD: Core1Load = Core1Load::new();

/// The last complete [`CORE1_LOAD`] window, for ULog and the RPC query.
static CORE1_LOAD_LAST: embassy_sync::blocking_mutex::Mutex<
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    core::cell::Cell<Core1LoadSnapshot>,
> = embassy_sync::blocking_mutex::Mutex::new(core::cell::Cell::new(Core1LoadSnapshot {
    wakes: 0,
    busy_sum_us: 0,
    busy_max_us: 0,
    mag_max_us: 0,
    baro_max_us: 0,
    max_drain: 0,
}));

/// Close the current Core 1 load window (the control loop does, ~1 Hz,
/// whether or not ULog records) and keep it as [`core1_last_window`].
pub fn take_core1_window() -> Core1LoadSnapshot {
    let s = CORE1_LOAD.take();
    CORE1_LOAD_LAST.lock(|c| c.set(s));
    s
}

/// The last window closed by [`take_core1_window`].
pub fn core1_last_window() -> Core1LoadSnapshot {
    CORE1_LOAD_LAST.lock(core::cell::Cell::get)
}

/// Time Core 0 spends in the DShot executor's interrupt (SWI_IRQ_0), which
/// preempts the control loop. Recorded by the binaries' handler, taken with the
/// loop's stage timing. Each poll is only a few µs, so the 1 µs timer rounds
/// each one; treat the sum as ±1 µs per poll.
pub struct IrqTime {
    busy_us: AtomicU32,
    runs: AtomicU32,
}

impl IrqTime {
    #[must_use]
    pub const fn new() -> Self {
        Self {
            busy_us: AtomicU32::new(0),
            runs: AtomicU32::new(0),
        }
    }

    /// One run of the handler took `us`.
    pub fn record(&self, us: u32) {
        self.busy_us.fetch_add(us, Ordering::Relaxed);
        self.runs.fetch_add(1, Ordering::Relaxed);
    }

    /// (µs busy, handler runs) since the last take.
    pub fn take(&self) -> (u32, u32) {
        (
            self.busy_us.swap(0, Ordering::Relaxed),
            self.runs.swap(0, Ordering::Relaxed),
        )
    }
}

impl Default for IrqTime {
    fn default() -> Self {
        Self::new()
    }
}

/// DShot executor (SWI_IRQ_0) time on Core 0.
pub static DSHOT_EXEC_TIME: IrqTime = IrqTime::new();
