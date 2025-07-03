#![allow(dead_code)]

// PWM timing parameters
pub const REFRESH_INTERVAL_US: u32 = 20_000; // 50Hz servo refresh rate

// Servo range (standard 1000-2000μs)
pub const SERVO_MIN_PULSE_US: u32 = 1_000;
pub const SERVO_MAX_PULSE_US: u32 = 2_000;
pub const SERVO_CENTER_US: u32 = 1_500;

// ESC range
pub const ENGINE_MIN_PULSE_US: u32 = 1_000;      // Absolute minimum (motors off)
pub const ENGINE_START_PULSE_US: u32 = 1_150;    // Actual point where motors start spinning
pub const ENGINE_MAX_PULSE_US: u32 = 1_600;      // Maximum throttle
pub const ENGINE_IDLE_PULSE_US: u32 = 1_100;     // Idle throttle for init sequence

// Throttle curve
pub const THROTTLE_DEADZONE: u32 = 200;          // SBUS values 0-200 = motors off
pub const THROTTLE_START_POINT: u32 = 300;       // SBUS value where motors start

// Arming parameters
pub const ENGINE_ARM_THRESHOLD: u32 = 1_100;      // Must have low throttle to arm
pub const ARM_DURATION_MS: u32 = 2_000;          // Hold at min for 2 seconds during init

// Differential thrust parameters
pub const DIFF_NEUTRAL_MIN: u16 = 1_000;
pub const DIFF_NEUTRAL_MAX: u16 = 1_010;
pub const DIFF_MAX_PERCENT: i32 = 20;

// Engine sync tuning
pub const ENGINE_RIGHT_OFFSET_US: u32 = 2;

// SBUS parameters
pub const SBUS_BAUD: u32 = 100_000;
pub const SBUS_TIMEOUT_MS: u64 = 100;

// Channel mapping (0-indexed)
pub const ELEVON_LEFT_CH: usize = 0;
pub const ELEVON_RIGHT_CH: usize = 1;
pub const ENGINE_CH: usize = 2;
pub const DIFFERENTIAL_CH: usize = 3;

// Pin assignments
pub const PIN_ELEVON_LEFT: u8 = 16;
pub const PIN_ELEVON_RIGHT: u8 = 17;
pub const PIN_ENGINE_LEFT: u8 = 10;
pub const PIN_ENGINE_RIGHT: u8 = 11;
pub const PIN_SBUS_RX: u8 = 5;