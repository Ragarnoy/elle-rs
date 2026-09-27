//! Unified event logging — single call site dispatches to defmt, RPC LogTopic, and ULog.
//!
//! Use the [`elle_event!`] macro to emit events. Each invocation:
//! 1. Calls the corresponding `defmt` macro (info!, warn!, etc.) → RTT ch0
//! 2. Sends `(level, code)` to [`EVENT_CHANNEL`] → consumed by `log_publisher_task` → RPC LogTopic
//! 3. Sends `(level, code)` to [`ULOG_EVENT_CHANNEL`] → consumed by `log_flight_data` → ULog flash
//!
//! Channels use `try_send` — if no consumer is running (feature not enabled), events are silently dropped.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;

// ---------------------------------------------------------------------------
// Channels
// ---------------------------------------------------------------------------

/// Events destined for the RPC LogTopic (consumed by `log_publisher_task`).
pub static EVENT_CHANNEL: Channel<CriticalSectionRawMutex, (u8, u16), 16> = Channel::new();

/// Events destined for ULog flash recording (consumed by `log_flight_data`).
pub static ULOG_EVENT_CHANNEL: Channel<CriticalSectionRawMutex, (u8, u16), 16> = Channel::new();

/// Enqueue an event to both channels. Drops silently if either channel is full.
#[inline]
pub fn send(level: u8, code: u16) {
    let _ = EVENT_CHANNEL.try_send((level, code));
    let _ = ULOG_EVENT_CHANNEL.try_send((level, code));
}

// ---------------------------------------------------------------------------
// Event code registry
// ---------------------------------------------------------------------------

// GNSS (1–9)
#[cfg(feature = "gnss")]
pub(crate) const EVT_GNSS_FIRST_FIX: u16 = 1;
#[cfg(feature = "gnss")]
pub(crate) const EVT_GNSS_PERIODIC: u16 = 2;
#[cfg(feature = "gnss")]
pub(crate) const EVT_GNSS_UART_ERROR: u16 = 3;
#[cfg(feature = "gnss")]
pub(crate) const EVT_GNSS_CFG_NAK: u16 = 4;
#[cfg(feature = "gnss")]
pub(crate) const EVT_GNSS_PVT_ACQUIRED: u16 = 5;
#[cfg(feature = "gnss")]
pub(crate) const EVT_GNSS_NMEA_FALLBACK: u16 = 6;
#[cfg(feature = "gnss")]
pub(crate) const EVT_GNSS_BAUD_SWITCHED: u16 = 7;
#[cfg(feature = "gnss")]
pub(crate) const EVT_GNSS_BAUD_FALLBACK: u16 = 8;
#[cfg(feature = "gnss")]
pub(crate) const EVT_GNSS_NO_DATA: u16 = 9;

// GNSS, continued (140–149) — the 1–9 block is full
#[cfg(feature = "gnss")]
pub(crate) const EVT_GNSS_CFG_TIMEOUT: u16 = 140;
#[cfg(feature = "gnss")]
pub(crate) const EVT_GNSS_CFG_PARTIAL: u16 = 141;

// Safety (10–19)
pub const EVT_MOTORS_ARMED: u16 = 10;
pub const EVT_MOTORS_DISARMED: u16 = 11;
pub const EVT_EMERGENCY_STOP: u16 = 12;
pub const EVT_RC_WARNING: u16 = 13;
pub const EVT_RC_SIGNAL_LOST: u16 = 14;
pub const EVT_RC_RESTORED: u16 = 15;
pub const EVT_KILL_ENGAGED: u16 = 16;
pub const EVT_KILL_RELEASED: u16 = 17;

// CRSF telemetry (20–29)
pub(crate) const EVT_CRSF_TX_STARTED: u16 = 20;
pub(crate) const EVT_CRSF_TX_FIRST_SEC: u16 = 21;
pub(crate) const EVT_CRSF_TX_ERROR: u16 = 22;
pub(crate) const EVT_CRSF_TX_STATS: u16 = 23;

// ULog (30–39)
pub const EVT_ULOG_STARTED: u16 = 30;
pub const EVT_ULOG_INIT_FAILED: u16 = 31;
// 32 retired: "ULog not compiled in" (ULog is always compiled in now)
pub const EVT_ULOG_STOPPED: u16 = 33;
pub const EVT_ULOG_ERASED: u16 = 34;

// IMU / sensors (40–49)
pub(crate) const EVT_IMU_INIT_FAILED: u16 = 40;
pub(crate) const EVT_IMU_FIFO_OVERFLOW: u16 = 41;
pub(crate) const EVT_IMU_READ_ERRORS: u16 = 42;
pub(crate) const EVT_MAG_INIT_FAILED: u16 = 43;
pub(crate) const EVT_BARO_INIT_FAILED: u16 = 44;
/// Core1 fell behind and drained several queued IMU samples in one wake-up.
pub(crate) const EVT_IMU_CATCHUP: u16 = 45;
/// Boot-time gyro bias measured; the IMU now reports calibrated.
pub(crate) const EVT_GYRO_BIAS_DONE: u16 = 46;
/// No still window (or an implausible offset) at boot; flying with zero bias.
pub(crate) const EVT_GYRO_BIAS_FAILED: u16 = 47;

// CRSF receiver (50–59)
pub(crate) const EVT_CRSF_RX_FIRST_FRAME: u16 = 50;
pub(crate) const EVT_CRSF_RX_UART_ERROR: u16 = 51;

// Flash storage (60–69)
// 60 retired: ULog push to flash failed (ULog writes go to the SD card now)
pub(crate) const EVT_FLASH_ULOG_ERASE_FAILED: u16 = 61;
pub(crate) const EVT_FLASH_ULOG_WRITE_TIMEOUT: u16 = 62;

// Supervisor / system (70–79)
pub const EVT_CORE1_UNHEALTHY: u16 = 70;
pub const EVT_CORE1_RESTORED: u16 = 71;

// Flight state (80–89)
pub const EVT_ATTITUDE_STALE: u16 = 80;
pub const EVT_ULOG_RC_ON: u16 = 81;
// 82 retired: ULog RC switch off (recording runs until power-off, no RC switch)

// Autotune (90–99)
pub const EVT_AUTOTUNE_STARTED: u16 = 90;
pub const EVT_AUTOTUNE_COMPLETE: u16 = 91;
pub const EVT_AUTOTUNE_ABORTED: u16 = 92;
pub const EVT_AUTOTUNE_ESTOP: u16 = 93;

// PID profile persistence (100–109)
pub const EVT_PID_SAVED: u16 = 100;
pub const EVT_PID_SAVE_FAILED: u16 = 101;
pub const EVT_PID_LOADED: u16 = 102;
pub const EVT_PID_LOAD_EMPTY: u16 = 103;

// Mag calibration (110–119)
pub const EVT_MAG_CAL_STARTED: u16 = 110;
pub const EVT_MAG_CAL_COMPLETE: u16 = 111;
pub const EVT_MAG_CAL_FAILED: u16 = 112;
pub const EVT_MAG_CAL_SAVED: u16 = 113;
pub const EVT_MAG_CAL_CLEARED: u16 = 114;
pub const EVT_MAG_CAL_LOADED: u16 = 115;
pub const EVT_MAG_CAL_LOAD_EMPTY: u16 = 116;

// Tap detection (120–129)
pub(crate) const EVT_DOUBLE_TAP: u16 = 120;

// Heading hold (130–139)
pub const EVT_HEADING_HOLD_ENGAGED: u16 = 130;
pub const EVT_HEADING_HOLD_DISENGAGED: u16 = 131;
pub const EVT_HEADING_HOLD_TARGET_SET: u16 = 132;

// Level calibration (150–159) — IMU mounting offset
pub(crate) const EVT_LEVEL_CAL_STARTED: u16 = 150;
pub(crate) const EVT_LEVEL_CAL_COMPLETE: u16 = 151;
pub const EVT_LEVEL_CAL_FAILED_MOVING: u16 = 152;
pub(crate) const EVT_LEVEL_CAL_FAILED_TILTED: u16 = 153;
pub(crate) const EVT_LEVEL_CAL_SAVED: u16 = 154;
pub(crate) const EVT_LEVEL_CAL_SAVE_FAILED: u16 = 155;
pub(crate) const EVT_LEVEL_CAL_CLEARED: u16 = 156;
pub(crate) const EVT_LEVEL_CAL_LOADED: u16 = 157;
pub(crate) const EVT_LEVEL_CAL_LOAD_EMPTY: u16 = 158;

// ---------------------------------------------------------------------------
// Macro
// ---------------------------------------------------------------------------

/// Unified event macro — dispatches to defmt + EVENT_CHANNEL + ULOG_EVENT_CHANNEL.
///
/// ```ignore
/// elle_event!(info, EVT_MOTORS_ARMED, "Motors ARMED via RPC");
/// elle_event!(warn, EVT_GNSS_UART_ERROR, "GNSS UART error (total={})", count);
/// ```
#[macro_export]
macro_rules! elle_event {
    (trace, $code:expr, $($arg:tt)*) => {{
        defmt::trace!($($arg)*);
        $crate::event::send(0, $code);
    }};
    (debug, $code:expr, $($arg:tt)*) => {{
        defmt::debug!($($arg)*);
        $crate::event::send(1, $code);
    }};
    (info, $code:expr, $($arg:tt)*) => {{
        defmt::info!($($arg)*);
        $crate::event::send(2, $code);
    }};
    (warn, $code:expr, $($arg:tt)*) => {{
        defmt::warn!($($arg)*);
        $crate::event::send(3, $code);
    }};
    (error, $code:expr, $($arg:tt)*) => {{
        defmt::error!($($arg)*);
        $crate::event::send(4, $code);
    }};
}
