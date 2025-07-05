// src/config.rs - Updated channel mapping
#![allow(dead_code)]

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
pub const ENGINE_RIGHT_OFFSET_US: u32 = 2;

// SBUS parameters
pub const SBUS_BAUD: u32 = 100_000;
pub const SBUS_TIMEOUT_MS: u64 = 100;
pub const SBUS_CENTER_TOLERANCE: u16 = 10; // Max deviation
pub const SBUS_ROLL_CENTER: u16 = 999;
pub const SBUS_PITCH_CENTER: u16 = 999; // Adjust if needed
pub const SBUS_YAW_CENTER: u16 = 1005; // Adjust if needed

// NEW: Flight control channel mapping (0-indexed)
pub const PITCH_CH: usize = 0; // Elevator/pitch input
pub const ROLL_CH: usize = 1; // Aileron/roll input
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

// Pin assignments
pub const PIN_ELEVON_LEFT: u8 = 16;
pub const PIN_ELEVON_RIGHT: u8 = 17;
pub const PIN_ENGINE_LEFT: u8 = 10;
pub const PIN_ENGINE_RIGHT: u8 = 11;
pub const PIN_SBUS_RX: u8 = 5;

// IMU parameters
pub const IMU_I2C_FREQ: u32 = 400_000; // 400kHz I2C
pub const PIN_IMU_SDA: u8 = 6; // GPIO6
pub const PIN_IMU_SCL: u8 = 7; // GPIO7
pub const IMU_MAX_AGE_MS: u64 = 100; // Max age for valid attitude data
pub const IMU_CALIBRATION_TIMEOUT_S: u64 = 120; // Calibration timeout

pub const ELEVON_LEFT_TRIM_US: i32 = 100; // Raises left elevon
pub const ELEVON_RIGHT_TRIM_US: i32 = -50;

// Individual servo center positions after trim
pub const ELEVON_LEFT_CENTER_US: u32 = (SERVO_CENTER_US as i32 + ELEVON_LEFT_TRIM_US) as u32;
pub const ELEVON_RIGHT_CENTER_US: u32 = (SERVO_CENTER_US as i32 + ELEVON_RIGHT_TRIM_US) as u32;

// Safety bounds for trim values
pub const MAX_TRIM_US: i32 = 100; // Maximum trim adjustment

// PID tuning parameters (start conservative!)
pub const PITCH_KP: f32 = 0.5; // Proportional gain
pub const PITCH_KI: f32 = 0.05; // Integral gain
pub const PITCH_KD: f32 = 0.1; // Derivative gain
pub const PITCH_MAX_RATE: f32 = 45.0; // Max pitch rate in deg/s

pub const ROLL_KP: f32 = 0.4; // Usually lower than pitch for flying wings
pub const ROLL_KI: f32 = 0.05;
pub const ROLL_KD: f32 = 0.08;
pub const ROLL_MAX_RATE: f32 = 90.0; // Flying wings can roll faster

// Control authority limits (0.0 to 1.0)
pub const ATTITUDE_MAX_AUTHORITY: f32 = 0.5; // Limit attitude controller to 50% of stick authority

// Attitude control channels
pub const ATTITUDE_ENABLE_CH: usize = 4; // CH5 - Attitude hold enable
pub const ATTITUDE_SETPOINT_CH: usize = 5; // CH6 - Desired pitch angle

// Attitude control thresholds
pub const ATTITUDE_ENABLE_THRESHOLD: u16 = 1200; // SBUS value above which attitude hold is enabled
pub const ATTITUDE_PITCH_MIN_DEG: f32 = -15.0; // Min commandable pitch angle
pub const ATTITUDE_PITCH_MAX_DEG: f32 = 25.0; // Max commandable pitch angle
