#![no_std]

pub mod lut;
pub mod profile;

// Re-export LUT functions for easy access
pub use lut::*;
pub use profile::*;

// Platform identification (used for ULog ver_hw, etc.)
#[cfg(not(feature = "platform-dart"))]
pub const PLATFORM_NAME: &str = "RP2350-XFly-Eagle";
#[cfg(feature = "platform-dart")]
pub const PLATFORM_NAME: &str = "RP2350-Elle-Dart";

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
#[cfg(not(feature = "platform-dart"))]
pub const YAW_TO_DIFF_GAIN: f32 = 1.0; // How much yaw affects differential thrust
#[cfg(feature = "platform-dart")]
pub const YAW_TO_DIFF_GAIN: f32 = 0.0; // Single engine — no yaw via differential thrust
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

#[cfg(not(feature = "platform-dart"))]
pub const ELEVON_LEFT_TRIM_US: i32 = 100; // Raises left elevon
#[cfg(not(feature = "platform-dart"))]
pub const ELEVON_RIGHT_TRIM_US: i32 = -50;

#[cfg(feature = "platform-dart")]
pub const ELEVON_LEFT_TRIM_US: i32 = 5; // New airframe — start at zero
#[cfg(feature = "platform-dart")]
pub const ELEVON_RIGHT_TRIM_US: i32 = 0;

// Individual servo center positions after trim
pub const ELEVON_LEFT_CENTER_US: u32 = (SERVO_CENTER_US as i32 + ELEVON_LEFT_TRIM_US) as u32;
pub const ELEVON_RIGHT_CENTER_US: u32 = (SERVO_CENTER_US as i32 + ELEVON_RIGHT_TRIM_US) as u32;

// Safety bounds for trim values
pub const MAX_TRIM_US: i32 = 100; // Maximum trim adjustment

#[cfg(not(feature = "platform-dart"))]
pub const ROLL_KP: f32 = 0.5;
#[cfg(not(feature = "platform-dart"))]
pub const ROLL_KI: f32 = 0.03;
#[cfg(not(feature = "platform-dart"))]
pub const ROLL_KD: f32 = 0.12;

#[cfg(feature = "platform-dart")]
pub const ROLL_KP: f32 = 0.25;
#[cfg(feature = "platform-dart")]
pub const ROLL_KI: f32 = 0.012;
#[cfg(feature = "platform-dart")]
pub const ROLL_KD: f32 = 0.07;

#[cfg(not(feature = "platform-dart"))]
pub const PITCH_KP: f32 = 1.0;
#[cfg(not(feature = "platform-dart"))]
pub const PITCH_KI: f32 = 0.1;
#[cfg(not(feature = "platform-dart"))]
pub const PITCH_KD: f32 = 0.25;

// Dart: inverted from LOG_0026 / 0032 / 0035 holds (docs/DART_PID.md).
// 0.25/0.125 flew but sagged; 0.50/0.40 punched / blew roll. I stays low.
#[cfg(feature = "platform-dart")]
pub const PITCH_KP: f32 = 0.45;
#[cfg(feature = "platform-dart")]
pub const PITCH_KI: f32 = 0.020;
#[cfg(feature = "platform-dart")]
pub const PITCH_KD: f32 = 0.16;

// PID operating scale and integral limit (must match SavedGains validation ranges)
pub const PID_SCALE: f32 = 5.0;
pub const PID_I_LIMIT: f32 = 0.5;

// Skip flash PID load/save/erase. Firmware defaults always apply.
// `clearpid` erases the whole 64KB profile region (mag cal is in the same
// map) and has crashed the MCU — leave this on until that path is fixed.
#[cfg(feature = "platform-dart")]
pub const IGNORE_PID_FLASH: bool = true;
#[cfg(not(feature = "platform-dart"))]
pub const IGNORE_PID_FLASH: bool = false;

// Control authority limits (0.0 to 1.0).
// NOT APPLIED anywhere: `AttitudeController::update` returns an unclamped
// `scale * (P + I + D)` per axis, and `mix_elevons` clips each axis at +/-1 and
// then each surface at +/-1. Leave it unused until a hop shows one axis
// starving the other (docs/DART_PID.md).
pub const ATTITUDE_MAX_AUTHORITY: f32 = 0.8;

// RC aux channel assignments:
//   CH5 (idx 4) = 2-pos switch left  → Heading hold (modifier on top of Stabilized)
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

// Heading-hold RC switch (2-position on CH5) — modifier active only while
// AttitudeMode::Stabilized is selected; captures current heading on engage.
pub const HEADING_HOLD_CH: usize = 4; // CH5 - 2-pos: off/on
pub const HEADING_HOLD_THRESHOLD: u16 = 1024; // Above this = engaged
pub const HEADING_HOLD_DEBOUNCE_TICKS: u32 = 38; // 0.5s at 77Hz, matches AUTOTUNE_DEBOUNCE_TICKS

// Heading-hold outer loop: P/PI on heading error (deg) -> roll setpoint (deg)
pub const HEADING_HOLD_KP: f32 = 1.2;
pub const HEADING_HOLD_KI: f32 = 0.0; // start P-only; enable after flight-test tuning
pub const HEADING_HOLD_I_LIMIT_DEG: f32 = 10.0; // integral clamp, in roll-degrees
pub const HEADING_HOLD_MAX_ROLL_DEG: f32 = 25.0; // bank angle clamp (< STABILIZED_MAX_ROLL_DEG)
pub const HEADING_HOLD_MAX_ROLL_RATE_DEG_S: f32 = 15.0; // output slew-rate limiter

// Double-tap mag-cal gesture gating (motor vibration can trip the APEX tap detector,
// so the gesture only starts calibration when the aircraft is demonstrably idle)
pub const TAP_CAL_MAX_GYRO_RAD_S: f32 = 0.5; // ~30°/s — gyro must be quiet
pub const TAP_CAL_THROTTLE_MAX_RAW: u16 = 200; // Raw CRSF throttle must be below this
pub const TAP_CAL_THROTTLE_MAX_NORM: f32 = 0.05; // Normalized throttle must be below this

// Level calibration (IMU mounting offset). Samples are at the 1 kHz IMU rate.
/// Samples discarded before averaging, so the tap that triggered a field
/// calibration has died out before collection starts.
pub const LEVEL_CAL_SETTLE_SAMPLES: u32 = 500;
/// Samples averaged into the gravity vector (2 s).
pub const LEVEL_CAL_SAMPLES: u32 = 2000;
/// Any gyro magnitude above this during collection fails the run as "moving".
/// ~6°/s: well above sensor noise on a still airframe, well below handling.
pub const LEVEL_CAL_MAX_GYRO_RAD_S: f32 = 0.1;
/// A mounting offset larger than this means the aircraft was not at its reference
/// attitude, not that the board is mounted crooked; the run fails as "tilted".
pub const LEVEL_CAL_MAX_TILT_DEG: f32 = 15.0;

// Setpoint smoothing parameters
pub const SETPOINT_FILTER_ALPHA: f32 = 0.15; // Low-pass filter for setpoint smoothing (0.1-0.3)
pub const MAX_SETPOINT_RATE_DEG_S: f32 = 30.0; // Max rate of setpoint change (degrees/second)

// Motor specs (eagle): 5000KV, 14-pole, 3S/4S, rated 20A/500g/330W (manufacturer test prop)
// Actual setup: 12-blade EDF, draws ~10A at max thrust (well within motor limits)
// rpm_range sweep 26 Sep 2026, 4S at 16.25 V under load: left engine peaks at
// 21,389 RPM (DShot 1498) and falls above; right flattens out at ~20,340 RPM
// from DShot ~1400. MAX_RPM is capped at the slower engine (right) to avoid
// asymmetric thrust. The engines are ~5% apart at the top (the previous
// sweep had them ~2% apart), so the right side lost more top-end.
#[cfg(not(feature = "platform-dart"))]
pub const MOTOR_POLES: u8 = 14;
#[cfg(not(feature = "platform-dart"))]
pub const MAX_RPM: u32 = 20_300; // right engine plateau, rpm_range sweep 2026-09-26
#[cfg(not(feature = "platform-dart"))]
pub const MAX_ERPM: u32 = MAX_RPM * (MOTOR_POLES as u32 / 2); // = 142,100

// Motor specs (dart). MOTOR_POLES = 14 (confirmed by the rpm_range sweep).
// 3-blade prop, reversed spin. MAX_RPM is the full-throttle point of the
// 26 Sep 2026 rpm_range sweep (15,360 RPM at DShot 1998, 3S at 11.5 V under
// load), rounded down. It matches the ~103–107 k eRPM ceiling seen in flight
// logs LOG_0020–0035. The curve is still rising at full throttle, so there is
// no stall region above it.
#[cfg(feature = "platform-dart")]
pub const MOTOR_POLES: u8 = 14;
#[cfg(feature = "platform-dart")]
pub const MAX_RPM: u32 = 15_300; // 3-blade, rpm_range sweep 2026-09-26
#[cfg(feature = "platform-dart")]
pub const MAX_ERPM: u32 = MAX_RPM * (MOTOR_POLES as u32 / 2); // 107,100

/// ESC spin direction. The dart's prop needs the ESC reversed (sweep-confirmed on the
/// 3-blade: normal direction peaks at DShot ~1773 and then falls). Commanded on every
/// boot, session-only, never `SettingsSave`: no EEPROM wear, and still correct after
/// an ESC swap or factory reset.
#[cfg(feature = "platform-dart")]
pub const ENGINE_SPIN_REVERSED: bool = true;
#[cfg(not(feature = "platform-dart"))]
pub const ENGINE_SPIN_REVERSED: bool = false;

/// Governor PI gains — normalized to eRPM scale.
/// Kp=0.01 gives ~14 DShot counts per 1000 eRPM error — fast enough to reduce
/// overshoot settling time without oscillation.
#[cfg(not(feature = "platform-dart"))]
pub const GOVERNOR_KP: f32 = 0.01;
#[cfg(not(feature = "platform-dart"))]
pub const GOVERNOR_KI: f32 = 0.002;
pub const GOVERNOR_DT: f32 = 0.001; // 1ms (1kHz task rate) — universal
#[cfg(not(feature = "platform-dart"))]
pub const GOVERNOR_DEADBAND_ERPM: f32 = 500.0; // Ignore errors below this (absorbs EDT noise)
#[cfg(not(feature = "platform-dart"))]
pub const GOVERNOR_ERPM_MAX_JUMP: u32 = 20_000; // Reject telemetry jumps larger than this

#[cfg(feature = "platform-dart")]
pub const GOVERNOR_KP: f32 = 0.01;
#[cfg(feature = "platform-dart")]
pub const GOVERNOR_KI: f32 = 0.002;
#[cfg(feature = "platform-dart")]
pub const GOVERNOR_DEADBAND_ERPM: f32 = 500.0;
#[cfg(feature = "platform-dart")]
pub const GOVERNOR_ERPM_MAX_JUMP: u32 = 20_000;

/// Hard ceiling on governor DShot output — independent of `DSHOT_THROTTLE_MAX`.
/// Must match the last `GOVERNOR_FF_TABLE` entry: the PI correction must never push
/// output past the point the feedforward table itself refuses to cross, or the
/// integrator can wind up past it under normal RPM sag (battery/thermal/prop wash),
/// driving DShot further into a region where RPM falls as throttle rises — a runaway
/// positive-feedback loop (eagle left engine peaks at DShot 1498 and falls above;
/// the dart's 3-blade rises through DShot 1998, the last measured point).
#[cfg(not(feature = "platform-dart"))]
pub const GOVERNOR_DSHOT_MAX: u16 = 1_498;
#[cfg(feature = "platform-dart")]
pub const GOVERNOR_DSHOT_MAX: u16 = 1_998;

// The doc comment above is a real contract: enforce it so the two can't drift.
const _: () = assert!(GOVERNOR_DSHOT_MAX == lut::governor_ff_max_dshot());

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
const _: () = assert!(HEADING_HOLD_CH < 16);

// Heading-hold gains/limits must be sane and bank angle must not exceed the
// pilot's own Stabilized-mode authority.
const _: () =
    assert!(HEADING_HOLD_KP >= 0.0 && HEADING_HOLD_KI >= 0.0 && HEADING_HOLD_I_LIMIT_DEG >= 0.0);
const _: () = assert!(
    HEADING_HOLD_MAX_ROLL_DEG > 0.0 && HEADING_HOLD_MAX_ROLL_DEG <= STABILIZED_MAX_ROLL_DEG
);
const _: () = assert!(HEADING_HOLD_MAX_ROLL_RATE_DEG_S > 0.0);

// CONTROL_LOOP_DT must be consistent with CONTROL_LOOP_FREQUENCY_HZ (±1ms tolerance).
// These are defined independently — if one changes and the other doesn't, PID integrator
// and autotuner timing silently break.
const _: () = {
    let expected_ms = 1000 / CONTROL_LOOP_FREQUENCY_HZ; // integer ms
    let dt_ms = (CONTROL_LOOP_DT * 1000.0) as u32;
    assert!(dt_ms >= expected_ms - 1 && dt_ms <= expected_ms + 1);
};

// Motor poles must be even (eRPM = RPM × poles/2)
const _: () = assert!(MOTOR_POLES.is_multiple_of(2));

// Servo range: MIN < CENTER < MAX
const _: () = assert!(SERVO_MIN_PULSE_US < SERVO_CENTER_US);
const _: () = assert!(SERVO_CENTER_US < SERVO_MAX_PULSE_US);

// Trim must not push center outside servo range
const _: () = assert!(ELEVON_LEFT_CENTER_US >= SERVO_MIN_PULSE_US);
const _: () = assert!(ELEVON_LEFT_CENTER_US <= SERVO_MAX_PULSE_US);
const _: () = assert!(ELEVON_RIGHT_CENTER_US >= SERVO_MIN_PULSE_US);
const _: () = assert!(ELEVON_RIGHT_CENTER_US <= SERVO_MAX_PULSE_US);
