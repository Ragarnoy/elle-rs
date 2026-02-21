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
pub const EVT_GNSS_FIRST_FIX: u16 = 1;
pub const EVT_GNSS_PERIODIC: u16 = 2;
pub const EVT_GNSS_UART_ERROR: u16 = 3;

// Safety (10–19)
pub const EVT_MOTORS_ARMED: u16 = 10;
pub const EVT_MOTORS_DISARMED: u16 = 11;
pub const EVT_EMERGENCY_STOP: u16 = 12;

// CRSF telemetry (20–29)
pub const EVT_CRSF_TX_STARTED: u16 = 20;
pub const EVT_CRSF_TX_FIRST_SEC: u16 = 21;
pub const EVT_CRSF_TX_ERROR: u16 = 22;
pub const EVT_CRSF_TX_STATS: u16 = 23;

// ULog (30–39)
pub const EVT_ULOG_STARTED: u16 = 30;
pub const EVT_ULOG_INIT_FAILED: u16 = 31;
pub const EVT_ULOG_NOT_COMPILED: u16 = 32;
pub const EVT_ULOG_STOPPED: u16 = 33;
pub const EVT_ULOG_ERASED: u16 = 34;

// IMU / sensors (40–49)
pub const EVT_IMU_INIT_FAILED: u16 = 40;
pub const EVT_IMU_FIFO_OVERFLOW: u16 = 41;
pub const EVT_IMU_READ_ERRORS: u16 = 42;
pub const EVT_MAG_INIT_FAILED: u16 = 43;
pub const EVT_BARO_INIT_FAILED: u16 = 44;

// CRSF receiver (50–59)
pub const EVT_CRSF_RX_FIRST_FRAME: u16 = 50;
pub const EVT_CRSF_RX_UART_ERROR: u16 = 51;

// Flash storage (60–69)
pub const EVT_FLASH_ULOG_PUSH_FAILED: u16 = 60;
pub const EVT_FLASH_ULOG_ERASE_FAILED: u16 = 61;
pub const EVT_FLASH_ULOG_WRITE_TIMEOUT: u16 = 62;

// Supervisor / system (70–79)
pub const EVT_CORE1_UNHEALTHY: u16 = 70;
pub const EVT_CORE1_RESTORED: u16 = 71;

// Flight state (80–89)
pub const EVT_ATTITUDE_STALE: u16 = 80;
pub const EVT_ULOG_RC_ON: u16 = 81;
pub const EVT_ULOG_RC_OFF: u16 = 82;

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
