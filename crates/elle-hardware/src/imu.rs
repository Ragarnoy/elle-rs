//! ICM-42686-P IMU integration with AHRS sensor fusion

use defmt::*;
use elle_error::ElleResult;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::Instant;

use crate::signal_cache::SignalCache;

// ============================================================================
// COMMON TYPES AND STATICS (available in both modes)
// ============================================================================

/// AHRS attitude and filtered rates. The type is the flight law's own
/// (`elle_control::flight_law::Attitude`), so the controller builds on the host.
pub use elle_control::flight_law::Attitude as AttitudeData;

#[derive(Clone, Copy, Debug, Format)]
pub struct ImuStatus {
    pub initialized: bool,
    pub calibrated: bool,
    pub error_count: u32,
    pub(crate) last_update: Instant,
}

impl Default for ImuStatus {
    fn default() -> Self {
        Self::new()
    }
}

impl ImuStatus {
    #[must_use]
    pub(crate) const fn new() -> Self {
        Self {
            initialized: false,
            calibrated: false,
            error_count: 0,
            last_update: Instant::from_ticks(0),
        }
    }
}

/// Shared attitude data between cores/tasks.
/// Use `publish()` to update, `read_cached()` for non-consuming reads (RPC handlers),
/// `try_take()`/`wait()` for the primary async consumer (control loop).
pub static ATTITUDE: SignalCache<AttitudeData> = SignalCache::new(AttitudeData::zero());

/// Global signal for Core 1 (IMU) heartbeat
pub static CORE1_HEARTBEAT: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Helper function to check if attitude data is valid and recent
#[must_use]
pub fn is_attitude_valid(attitude: &AttitudeData, max_age: embassy_time::Duration) -> bool {
    attitude.timestamp != Instant::from_ticks(0) && attitude.timestamp.elapsed() < max_age
}

/// Largest raw magnetometer count either way: the MMC5616WA's 20-bit output,
/// signed about its null field (2^19). A hard-iron offset can be anywhere in it.
pub const MAG_FULL_SCALE_COUNTS: f32 = mmc5616wa::registers::NULL_FIELD_OUTPUT as f32;

/// Magnetometer data (populated by MMC5616WA).
pub static MAG: SignalCache<MagReading> = SignalCache::new(MagReading { x: 0, y: 0, z: 0 });

/// Magnetometer reading (signed counts from MMC5616WA)
#[derive(Clone, Copy, Debug, Format)]
pub struct MagReading {
    pub x: i32,
    pub y: i32,
    pub z: i32,
}

/// Barometer data (populated by BMP390).
pub static BARO: SignalCache<BaroReading> = SignalCache::new(BaroReading {
    pressure_hpa: 0.0,
    temperature_c: 0.0,
    altitude_m: 0.0,
    vario_ms: 0.0,
    sample_us: 0,
});

/// Barometer reading from BMP390
#[derive(Clone, Copy, Debug, Format)]
pub struct BaroReading {
    pub pressure_hpa: f32,
    pub temperature_c: f32,
    pub altitude_m: f32,
    /// Vertical speed in m/s (positive = climbing, negative = sinking).
    /// Computed from altitude differentiation with EMA smoothing.
    pub vario_ms: f32,
    /// When the measurement was read, µs since boot; 0 before the first one.
    pub sample_us: u64,
}

/// Cross-core: Core0 signals loaded offsets at boot → Core1 IMU applies them
pub static MAG_CALIBRATION_SIGNAL: Signal<CriticalSectionRawMutex, (f32, f32, f32)> = Signal::new();

/// Main loop signals "start calibration" → IMU begins min/max tracking
pub static MAG_CAL_START_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// IMU signals result back → main loop saves to flash. None = failed (insufficient rotation).
pub static MAG_CAL_RESULT_SIGNAL: Signal<CriticalSectionRawMutex, Option<(f32, f32, f32)>> =
    Signal::new();

/// Live sample count during mag calibration collection (Core1 → RPC status handler).
/// Reset to 0 when collection starts; final count remains after completion.
pub static MAG_CAL_PROGRESS: core::sync::atomic::AtomicU16 = core::sync::atomic::AtomicU16::new(0);

/// Double-tap gesture detected by ICM-42686 APEX tap detection (Core1 → Core0).
/// Core0 acts on this only when disarmed.
pub static TAP_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Core1 → Core0 queue of raw IMU records (`imu-raw-log` builds): batches of 10
/// samples, plus mag and context changes. 16 records hold well over a control
/// tick (a batch is 10 ms); the ULog path drains it every tick, and records are
/// dropped (never blocked on) if it fills, which the sample index makes visible.
#[cfg(feature = "imu-raw-log")]
pub static IMU_RAW_CHANNEL: embassy_sync::channel::Channel<
    CriticalSectionRawMutex,
    elle_control::imu_raw::Record,
    16,
> = embassy_sync::channel::Channel::new();

/// Channel for LED pattern updates
pub static LED_COMMAND_CHANNEL: embassy_sync::channel::Channel<
    CriticalSectionRawMutex,
    crate::led::LedPattern,
    8,
> = embassy_sync::channel::Channel::new();

/// RwLock-protected IMU status for safe access
pub static IMU_STATUS: embassy_sync::rwlock::RwLock<CriticalSectionRawMutex, ImuStatus> =
    embassy_sync::rwlock::RwLock::new(ImuStatus::new());

mod driver;
pub mod i2c_sensors;
pub mod level_cal;
pub use driver::Imu;
