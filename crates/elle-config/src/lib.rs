#![no_std]

pub mod lut;
pub mod profile;

// Re-export LUT functions for easy access
pub use lut::*;
pub use profile::*;

// PWM timing parameters
pub const REFRESH_INTERVAL_US: u32 = 20_000; // 50Hz servo refresh rate

// Servo range (standard 1000-2000μs)
pub const SERVO_MIN_PULSE_US: u32 = 1_000;
pub const SERVO_MAX_PULSE_US: u32 = 2_000;
pub const SERVO_CENTER_US: u32 = 1_500;

// ESC range
pub const ENGINE_MIN_PULSE_US: u32 = 1_000; // Absolute minimum (motors off)
pub const ENGINE_START_PULSE_US: u32 = 1_150; // Actual point where motors start spinning
pub const ENGINE_MAX_PULSE_US: u32 = 1_600; // Maximum throttle

// Throttle curve
pub const THROTTLE_DEADZONE: u32 = 200; // RC values 0-200 = motors off
pub const THROTTLE_START_POINT: u32 = 300; // RC value where motors start

// Arming parameters
pub const ENGINE_ARM_THRESHOLD: u32 = 1_100; // Must have low throttle to arm
pub const ARM_DURATION_MS: u32 = 2_000; // Hold at min for 2 seconds during init

// Differential thrust parameters
pub const DIFF_NEUTRAL_MIN: u16 = 1_000;
pub const DIFF_NEUTRAL_MAX: u16 = 1_010;
pub const DIFF_MAX_PERCENT: i32 = 20;

// DShot configuration
pub const DSHOT_THROTTLE_MAX: u16 = 1999;

// RC parameters (protocol-independent, values in 0–2047 range)
pub const RC_WARNING_MS: u64 = 200;
pub const RC_TIMEOUT_MS: u64 = 300;
pub const RC_CENTER: u16 = 1024; // CRSF center 992 scaled to 0–2047

// Control loop timing parameters
pub const CONTROL_LOOP_FREQUENCY_HZ: u32 = 77; // Actual measured rate (~13ms)
pub const CONTROL_LOOP_PERIOD_MS: u64 = 1000 / CONTROL_LOOP_FREQUENCY_HZ as u64; // 13ms
pub const CONTROL_LOOP_DT: f32 = 0.013; // 13ms actual timing for PID stability
pub const IMU_UPDATE_FREQUENCY_HZ: u32 = 1000; // IMU reads at 1kHz
pub const RC_MAX_LATENCY_MS: u64 = 100; // Max acceptable RC packet age

// ULog sub-sampling divisors (relative to CONTROL_LOOP_FREQUENCY_HZ)
pub const ULOG_STATUS_DIVISOR: u32 = 10; // 77/10 ≈ 7.7 Hz
pub const ULOG_MAG_DIVISOR: u32 = 8; // 77/8 ≈ 9.6 Hz
pub const ULOG_BARO_DIVISOR: u32 = 19; // 77/19 ≈ 4 Hz
pub const ULOG_GNSS_DIVISOR: u32 = CONTROL_LOOP_FREQUENCY_HZ; // ~1 Hz
pub const STALE_EVENT_DRAIN_DIVISOR: u32 = CONTROL_LOOP_FREQUENCY_HZ; // ~1 Hz

// LED update interval (iterations at CONTROL_LOOP_FREQUENCY_HZ)
pub const LED_UPDATE_INTERVAL: u32 = CONTROL_LOOP_FREQUENCY_HZ * 4; // ~4s

// Performance log interval
pub const PERF_LOG_INTERVAL: u32 = CONTROL_LOOP_FREQUENCY_HZ * 10; // ~10s

// GNSS error log throttling
pub const GNSS_ERROR_LOG_INITIAL: u32 = 3; // Log first N errors
pub const GNSS_ERROR_LOG_INTERVAL: u32 = 1000; // Then every Nth error

// Flight control channel mapping (0-indexed)
pub const ROLL_CH: usize = 0; // Aileron/roll input
pub const PITCH_CH: usize = 1; // Elevator/pitch input
pub const THROTTLE_CH: usize = 2; // Engine throttle
pub const YAW_CH: usize = 3; // Rudder/yaw input

// Channel inversion (sign flip for reversed servo/engine direction)
pub const PITCH_INVERT: f32 = -1.0;
pub const ROLL_INVERT: f32 = 1.0;
pub const YAW_INVERT: f32 = -1.0;

// Legacy direct elevon channels (if needed for fallback)
pub const ELEVON_LEFT_CH: usize = 0;
pub const ELEVON_RIGHT_CH: usize = 1;
pub const ENGINE_CH: usize = 2;
pub const DIFFERENTIAL_CH: usize = 3;

// Elevon mixing parameters
pub const ELEVON_PITCH_GAIN: f32 = 1.0; // How much pitch affects elevons
pub const ELEVON_ROLL_GAIN: f32 = 1.0; // How much roll affects elevons
pub const YAW_TO_DIFF_GAIN: f32 = 1.0; // How much yaw affects differential thrust
pub const YAW_TO_ELEVON_GAIN: f32 = 0.1; // Small yaw contribution to elevons for coordination

// Control mode selection
pub const USE_MIXING_MODE: bool = true; // Set to false for direct elevon control

// IMU parameters
pub const IMU_I2C_FREQ: u32 = 400_000; // 400kHz I2C fast mode (MMC5616WA + BMP390)
pub const IMU_MAX_AGE_MS: u64 = 100; // Max age for valid attitude data
pub const IMU_CALIBRATION_TIMEOUT_S: u64 = 120; // Calibration timeout

// IMU SPI parameters (ICM-42686-P)
pub const IMU_SPI_FREQ: u32 = 8_000_000; // 8 MHz SPI clock (ICM-42686-P rated to 24 MHz reads)
pub const AHRS_SAMPLE_PERIOD_US: u64 = 1000; // 1ms (matches 1 kHz ICM ODR)
/// Madgwick AHRS filter gain (higher = faster convergence, more noise)
pub const AHRS_BETA: f32 = 0.033;
/// Magnetometer read interval in IMU ticks (100 = 10Hz at 1kHz IMU rate)
pub const MAG_READ_INTERVAL_TICKS: u32 = 100;
/// Barometer read interval in IMU ticks (50 = 20Hz at 1kHz IMU rate)
pub const BARO_READ_INTERVAL_TICKS: u32 = 50;

// Supervisor parameters
pub const WATCHDOG_TIMEOUT_MS: u64 = 500; // Hardware watchdog timeout
pub const CORE1_HEALTH_TIMEOUT_MS: u64 = 2000; // Core 1 health check timeout
pub const SUPERVISOR_CHECK_INTERVAL_MS: u64 = 50; // How often to check supervisor health

pub const ELEVON_LEFT_TRIM_US: i32 = 100; // Raises left elevon
pub const ELEVON_RIGHT_TRIM_US: i32 = -50;

// Individual servo center positions after trim
pub const ELEVON_LEFT_CENTER_US: u32 = (SERVO_CENTER_US as i32 + ELEVON_LEFT_TRIM_US) as u32;
pub const ELEVON_RIGHT_CENTER_US: u32 = (SERVO_CENTER_US as i32 + ELEVON_RIGHT_TRIM_US) as u32;

// Safety bounds for trim values
pub const MAX_TRIM_US: i32 = 100; // Maximum trim adjustment

pub const ROLL_KP: f32 = 0.2;
pub const ROLL_KI: f32 = 0.05;
pub const ROLL_KD: f32 = 0.08;

pub const PITCH_KP: f32 = 0.2;
pub const PITCH_KI: f32 = 0.03;
pub const PITCH_KD: f32 = 0.1;

// PID operating scale and integral limit (must match SavedGains validation ranges)
pub const PID_SCALE: f32 = 5.0;
pub const PID_I_LIMIT: f32 = 0.5;

// Control authority limits (0.0 to 1.0)
pub const ATTITUDE_MAX_AUTHORITY: f32 = 0.8; // Increased authority for better response

// RC aux channel assignments:
//   CH5 (idx 4) = 2-pos switch left  → Unused
//   CH6 (idx 5) = 3-pos switch left  → Attitude mode (Manual/Stabilized/AltitudeHold)
//   CH7 (idx 6) = 3-pos switch right → Autotune (off/pitch/roll)
//   CH8 (idx 7) = 2-pos switch right → Kill switch (high = disarm, must re-arm)

// Kill switch
pub const KILL_SWITCH_CH: usize = 7; // CH8 - 2-pos switch right: kill (high = disarm)
pub const KILL_SWITCH_THRESHOLD: u16 = 1500; // Above this = kill active

// Attitude control mode
pub const ATTITUDE_ENABLE_CH: usize = 5; // CH6 - 3-pos: Manual/Stabilized/AltitudeHold

// Control mode switch thresholds (3-state switch on CH6)
pub const MANUAL_MODE_THRESHOLD: u16 = 500; // Below this = Full Manual (~306)
pub const STABILIZED_MODE_THRESHOLD: u16 = 1300; // Above this = Stabilized (~1000), above next = AltitudeHold

// Stabilized mode: stick deflection maps to attitude angle
pub const STABILIZED_MAX_PITCH_DEG: f32 = 25.0; // Full stick = ±25° pitch
pub const STABILIZED_MAX_ROLL_DEG: f32 = 45.0; // Full stick = ±45° roll

// Autotune RC switch (3-position on CH7)
pub const AUTOTUNE_CH: usize = 6; // CH7 - 3-pos: off/pitch/roll
pub const AUTOTUNE_OFF_THRESHOLD: u16 = 500; // Below = off
pub const AUTOTUNE_PITCH_THRESHOLD: u16 = 1300; // Below = pitch, above = roll
pub const AUTOTUNE_DEBOUNCE_TICKS: u32 = 38; // 0.5s at 77Hz

// Setpoint smoothing parameters
pub const SETPOINT_FILTER_ALPHA: f32 = 0.15; // Low-pass filter for setpoint smoothing (0.1-0.3)
pub const MAX_SETPOINT_RATE_DEG_S: f32 = 30.0; // Max rate of setpoint change (degrees/second)

// Motor specs: 5000KV, 14-pole, 3S/4S, rated 20A/500g/330W (manufacturer test prop)
// Actual setup: 12-blade EDF, draws ~10A at max thrust (well within motor limits)
// Left engine saturates at ~21,865 RPM (DShot ~1498), right at ~21,430 RPM (DShot ~1473)
// MAX_RPM capped at slower engine (right) to avoid asymmetric thrust
pub const MOTOR_POLES: u8 = 14;
pub const MAX_RPM: u32 = 21_400; // Measured: right engine saturation under EDF load (4S)
pub const MAX_ERPM: u32 = MAX_RPM * (MOTOR_POLES as u32 / 2); // = 149,800
/// Governor PI gains — normalized to eRPM scale.
/// Kp=0.01 gives ~14 DShot counts per 1000 eRPM error — fast enough to reduce
/// overshoot settling time without oscillation.
pub const GOVERNOR_KP: f32 = 0.01;
pub const GOVERNOR_KI: f32 = 0.002;
pub const GOVERNOR_DT: f32 = 0.001; // 1ms (1kHz task rate)
pub const GOVERNOR_DEADBAND_ERPM: f32 = 500.0; // Ignore errors below this (absorbs EDT noise)
pub const GOVERNOR_ERPM_MAX_JUMP: u32 = 20_000; // Reject telemetry jumps larger than this

// ---------------------------------------------------------------------------
// Compile-time validation
// ---------------------------------------------------------------------------
// These mirror the SavedGains::from_bytes() validation ranges (elle-control).
// If a const assert fires, the default config would fail to round-trip through
// flash persistence — exactly the bug we had with scale=5.0 vs range 0..=1.

// PID gains must fit SavedGains range 0.0..=100.0
const _: () = assert!(PITCH_KP >= 0.0 && PITCH_KP <= 100.0);
const _: () = assert!(PITCH_KI >= 0.0 && PITCH_KI <= 100.0);
const _: () = assert!(PITCH_KD >= 0.0 && PITCH_KD <= 100.0);
const _: () = assert!(ROLL_KP >= 0.0 && ROLL_KP <= 100.0);
const _: () = assert!(ROLL_KI >= 0.0 && ROLL_KI <= 100.0);
const _: () = assert!(ROLL_KD >= 0.0 && ROLL_KD <= 100.0);

// Scale must fit SavedGains range 0.0..=100.0
const _: () = assert!(PID_SCALE >= 0.0 && PID_SCALE <= 100.0);

// I-limit must fit SavedGains range 0.0..=1000.0
const _: () = assert!(PID_I_LIMIT >= 0.0 && PID_I_LIMIT <= 1000.0);

// Inversions must be exactly ±1.0 (not arbitrary floats)
const _: () = assert!(PITCH_INVERT == 1.0 || PITCH_INVERT == -1.0);
const _: () = assert!(ROLL_INVERT == 1.0 || ROLL_INVERT == -1.0);
const _: () = assert!(YAW_INVERT == 1.0 || YAW_INVERT == -1.0);

// Mode thresholds must be ordered
const _: () = assert!(MANUAL_MODE_THRESHOLD < STABILIZED_MODE_THRESHOLD);
const _: () = assert!(AUTOTUNE_OFF_THRESHOLD < AUTOTUNE_PITCH_THRESHOLD);

// Channel indices must be distinct and in 0..16
const _: () = assert!(ROLL_CH < 16 && PITCH_CH < 16 && THROTTLE_CH < 16 && YAW_CH < 16);
const _: () = assert!(ATTITUDE_ENABLE_CH < 16 && AUTOTUNE_CH < 16 && KILL_SWITCH_CH < 16);

// CONTROL_LOOP_DT must be consistent with CONTROL_LOOP_FREQUENCY_HZ (±1ms tolerance).
// These are defined independently — if one changes and the other doesn't, PID integrator
// and autotuner timing silently break.
const _: () = {
    let expected_ms = 1000 / CONTROL_LOOP_FREQUENCY_HZ; // integer ms
    let dt_ms = (CONTROL_LOOP_DT * 1000.0) as u32;
    assert!(dt_ms >= expected_ms - 1 && dt_ms <= expected_ms + 1);
};

// Motor poles must be even (eRPM = RPM × poles/2)
const _: () = assert!(MOTOR_POLES % 2 == 0);

// Servo range: MIN < CENTER < MAX
const _: () = assert!(SERVO_MIN_PULSE_US < SERVO_CENTER_US);
const _: () = assert!(SERVO_CENTER_US < SERVO_MAX_PULSE_US);

// Trim must not push center outside servo range
const _: () = assert!(ELEVON_LEFT_CENTER_US >= SERVO_MIN_PULSE_US);
const _: () = assert!(ELEVON_LEFT_CENTER_US <= SERVO_MAX_PULSE_US);
const _: () = assert!(ELEVON_RIGHT_CENTER_US >= SERVO_MIN_PULSE_US);
const _: () = assert!(ELEVON_RIGHT_CENTER_US <= SERVO_MAX_PULSE_US);
