#![no_std]

pub mod lut;
pub mod profile;

// Re-export LUT functions for easy access
pub use lut::*;
pub use profile::*;

use defmt::Format;

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
pub const ENGINE_IDLE_PULSE_US: u32 = 1_100; // Idle throttle for init sequence

// Throttle curve
pub const THROTTLE_DEADZONE: u32 = 200; // SBUS values 0-200 = motors off
pub const THROTTLE_START_POINT: u32 = 300; // SBUS value where motors start

// Arming parameters
pub const ENGINE_ARM_THRESHOLD: u32 = 1_100; // Must have low throttle to arm
pub const ARM_DURATION_MS: u32 = 2_000; // Hold at min for 2 seconds during init

// Differential thrust parameters
pub const DIFF_NEUTRAL_MIN: u16 = 1_000;
pub const DIFF_NEUTRAL_MAX: u16 = 1_010;
pub const DIFF_MAX_PERCENT: i32 = 20;

// Engine sync tuning
pub const ENGINE_RIGHT_OFFSET_US: u32 = 36;

// SBUS parameters
pub const SBUS_BAUD: u32 = 100_000;
pub const SBUS_TIMEOUT_MS: u64 = 300;
pub const SBUS_CENTER_TOLERANCE: u16 = 10; // Max deviation
pub const SBUS_ROLL_CENTER: u16 = 999;
pub const SBUS_PITCH_CENTER: u16 = 999; // Adjust if needed
pub const SBUS_YAW_CENTER: u16 = 1003; // Adjust if needed

// Control loop timing parameters - adjusted for SBUS-limited rate
pub const CONTROL_LOOP_FREQUENCY_HZ: u32 = 77; // Actual measured rate (~13ms)
pub const CONTROL_LOOP_PERIOD_MS: u64 = 1000 / CONTROL_LOOP_FREQUENCY_HZ as u64; // 13ms
pub const CONTROL_LOOP_DT: f32 = 0.013; // 13ms actual timing for PID stability
pub const IMU_UPDATE_FREQUENCY_HZ: u32 = 1000; // IMU reads at 1kHz
pub const SBUS_MAX_LATENCY_MS: u64 = 100; // Max acceptable SBUS packet age

// NEW: Flight control channel mapping (0-indexed)
pub const ROLL_CH: usize = 0; // Aileron/roll input
pub const PITCH_CH: usize = 1; // Elevator/pitch input
pub const THROTTLE_CH: usize = 2; // Engine throttle
pub const YAW_CH: usize = 3; // Rudder/yaw input

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
pub const IMU_I2C_FREQ: u32 = 200_000; // 200kHz I2C
pub const IMU_MAX_AGE_MS: u64 = 100; // Max age for valid attitude data
pub const IMU_CALIBRATION_TIMEOUT_S: u64 = 120; // Calibration timeout

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

// Control authority limits (0.0 to 1.0)
pub const ATTITUDE_MAX_AUTHORITY: f32 = 0.8; // Increased authority for better response

// Attitude control channels
pub const ATTITUDE_ENABLE_CH: usize = 4; // CH5 - Attitude hold enable
pub const ATTITUDE_PITCH_SETPOINT_CH: usize = 5; // CH6 - Desired pitch angle
pub const ATTITUDE_ROLL_SETPOINT_CH: usize = 7; // CH8 - Desired roll angle

// Control mode switch thresholds (3-state switch on CH5)
pub const MANUAL_MODE_THRESHOLD: u16 = 500; // Below this = Full Manual (~306)
pub const MIXED_MODE_THRESHOLD: u16 = 1300; // Above this but below AUTOPILOT = Mixed (~1000)
pub const AUTOPILOT_MODE_THRESHOLD: u16 = 1500; // Above this = Full Autopilot (~1694)
pub const ATTITUDE_PITCH_MIN_DEG: f32 = -15.0; // Min commandable pitch angle
pub const ATTITUDE_PITCH_MAX_DEG: f32 = 25.0; // Max commandable pitch angle
pub const ATTITUDE_ROLL_MIN_DEG: f32 = -45.0; // Min commandable roll angle (flying wings can roll more)
pub const ATTITUDE_ROLL_MAX_DEG: f32 = 45.0; // Max commandable roll angle

// Setpoint smoothing parameters
pub const SETPOINT_FILTER_ALPHA: f32 = 0.15; // Low-pass filter for setpoint smoothing (0.1-0.3)
pub const MAX_SETPOINT_RATE_DEG_S: f32 = 30.0; // Max rate of setpoint change (degrees/second)

// Mixed mode control blending
pub const MIXED_MODE_AUTOPILOT_WEIGHT: f32 = 0.6; // 60% autopilot, 40% pilot in mixed mode

/// IMU calibration quality levels
#[derive(Clone, Copy, Debug, Format)]
pub struct CalibrationLevels {
    pub sys: u8,
    pub gyro: u8,
    pub accel: u8,
    pub mag: u8,
}

impl CalibrationLevels {
    pub const fn new() -> Self {
        Self {
            sys: 0,
            gyro: 0,
            accel: 0,
            mag: 0,
        }
    }

    pub fn is_flight_ready(&self) -> bool {
        // Relaxed requirements for flight - magnetometer not critical
        self.sys >= 2 && self.gyro >= 3 && self.accel >= 2
    }

    pub fn hash_quality(&self) -> u32 {
        // Simple quality metric: prioritize sys and gyro, accel important, mag less so
        (self.sys as u32 * 100)
            + (self.gyro as u32 * 50)
            + (self.accel as u32 * 25)
            + (self.mag as u32 * 10)
    }
}
