//! BNO055 IMU integration for attitude sensing with RGB LED status

use crate::config::profile::StoredCalibration;
use crate::hardware::flash_manager::{request_load_calibration, request_save_calibration};
use crate::hardware::led::{LedPattern, colors};
#[cfg(feature = "performance-monitoring")]
use crate::system::{TimingMeasurement, update_imu_timing};
use crate::system::CORE1_HEARTBEAT;

#[cfg(not(feature = "performance-monitoring"))]
struct TimingMeasurement;

#[cfg(not(feature = "performance-monitoring"))]
impl TimingMeasurement {
    fn start() -> Self {
        Self
    }
    fn elapsed_us(&self) -> u32 {
        0
    }
}

#[cfg(not(feature = "performance-monitoring"))]
fn update_imu_timing(_elapsed_us: u32) {}
use bno055::{BNO055_CALIB_SIZE, BNO055AxisSign, BNO055Calibration, mint};
use defmt::*;
use embassy_rp::i2c::{Blocking, I2c};
use embassy_rp::peripherals::I2C0;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, Sender};
use embassy_sync::rwlock::RwLock;
use embassy_sync::signal::Signal;
use embassy_time::{Delay, Duration, Instant, Timer};

/// Shared attitude data between cores/tasks
pub static ATTITUDE_SIGNAL: Signal<CriticalSectionRawMutex, AttitudeData> = Signal::new();

/// Mutex-protected IMU status for safe access
pub static IMU_STATUS: RwLock<CriticalSectionRawMutex, ImuStatus> = RwLock::new(ImuStatus::new());

/// Channel for LED pattern updates
pub static LED_COMMAND_CHANNEL: Channel<CriticalSectionRawMutex, LedPattern, 8> = Channel::new();

#[derive(Clone, Copy, Debug, Format)]
pub struct AttitudeData {
    pub pitch: f32,      // radians
    pub roll: f32,       // radians
    pub yaw: f32,        // radians
    pub pitch_rate: f32, // rad/s
    pub roll_rate: f32,  // rad/s
    pub yaw_rate: f32,   // rad/s
    pub timestamp: Instant,
}

impl AttitudeData {
    const fn zero() -> Self {
        Self {
            pitch: 0.0,
            roll: 0.0,
            yaw: 0.0,
            pitch_rate: 0.0,
            roll_rate: 0.0,
            yaw_rate: 0.0,
            timestamp: Instant::from_ticks(0),
        }
    }
}

#[derive(Clone, Copy, Debug, Format)]
pub struct ImuStatus {
    pub initialized: bool,
    pub calibrated: bool,
    pub calibration_status: CalibrationLevels,
    pub error_count: u32,
    pub last_update: Instant,
}

impl ImuStatus {
    const fn new() -> Self {
        Self {
            initialized: false,
            calibrated: false,
            calibration_status: CalibrationLevels::new(),
            error_count: 0,
            last_update: Instant::from_ticks(0),
        }
    }
}

#[derive(Clone, Copy, Debug, Format)]
pub struct CalibrationLevels {
    pub sys: u8,
    pub gyro: u8,
    pub accel: u8,
    pub mag: u8,
}

impl CalibrationLevels {
    const fn new() -> Self {
        Self {
            sys: 0,
            gyro: 0,
            accel: 0,
            mag: 0,
        }
    }

    pub(crate) fn is_flight_ready(&self) -> bool {
        // Relaxed requirements for flight - magnetometer not critical
        self.sys >= 2 && self.gyro >= 3 && self.accel >= 2
    }
}

pub struct BnoImu<'a> {
    bno: bno055::Bno055<I2c<'a, I2C0, Blocking>>,
    led_sender: Sender<'a, CriticalSectionRawMutex, LedPattern, 8>,
    last_attitude: AttitudeData,
    error_threshold: u32,
    last_cal_log: CalibrationLevels,
    last_cal_request: Option<Instant>,
    calibration_loaded: bool,
}

impl<'a> BnoImu<'a> {
    pub fn new(
        i2c: I2c<'a, I2C0, Blocking>,
        led_sender: Sender<'a, CriticalSectionRawMutex, LedPattern, 8>,
    ) -> Self {
        let bno = bno055::Bno055::new(i2c).with_alternative_address();

        Self {
            bno,
            led_sender,
            last_attitude: AttitudeData::zero(),
            error_threshold: 10,
            last_cal_log: CalibrationLevels::new(),
            last_cal_request: None,
            calibration_loaded: false,
        }
    }

    /// Send LED pattern update
    async fn set_led_pattern(&self, pattern: LedPattern) {
        let _ = self.led_sender.try_send(pattern);
    }

    /// Try to load saved calibration via inter-core communication
    async fn load_calibration(&mut self) -> Result<bool, &'static str> {
        info!("Core1: Requesting calibration load from Core0");

        if let Some(profile_data) = request_load_calibration().await {
            let calib = BNO055Calibration::from_buf(&profile_data);

            let mut delay = Delay;
            match self.bno.set_calibration_profile(calib, &mut delay) {
                Ok(_) => {
                    info!("Core1: Successfully applied saved calibration");
                    self.calibration_loaded = true;
                    return Ok(true);
                }
                Err(e) => {
                    warn!(
                        "Core1: Failed to apply saved calibration: {:?}",
                        Debug2Format(&e)
                    );
                }
            }
        } else {
            info!("Core1: No saved calibration available");
        }

        Ok(false)
    }

    /// Request calibration save via inter-core communication
    /// Only saves calibration if imu-save-calibration feature is enabled
    async fn save_calibration(&mut self, levels: &CalibrationLevels) -> Result<(), &'static str> {
        // Check if saving calibration is enabled
        #[cfg(not(feature = "imu-save-calibration"))]
        {
            info!("Core1: Calibration saving is disabled, skipping save");
            return Ok(());
        }

        // Rate limiting - only request save once per 10 minutes
        if let Some(last_request) = self.last_cal_request
            && last_request.elapsed() < Duration::from_secs(600)
        {
            return Ok(());
        }

        if !levels.is_flight_ready() {
            return Ok(());
        }

        info!("Core1: Requesting calibration save to Core0");

        // Get current calibration profile
        let mut delay = Delay;
        let profile = match self.bno.calibration_profile(&mut delay) {
            Ok(profile) => profile,
            Err(e) => {
                warn!(
                    "Core1: Failed to get calibration profile: {:?}",
                    Debug2Format(&e)
                );
                return Err("Failed to get calibration profile");
            }
        };

        let profile_bytes = profile.as_bytes();
        let mut profile_array = [0u8; BNO055_CALIB_SIZE];
        profile_array.copy_from_slice(&profile_bytes[..BNO055_CALIB_SIZE.min(profile_bytes.len())]);

        let success =
            request_save_calibration(profile_array, *levels, Instant::now().as_ticks()).await;

        if success {
            info!("Core1: Calibration save requested successfully");
            self.last_cal_request = Some(Instant::now());
            Ok(())
        } else {
            warn!("Core1: Calibration save request failed");
            Err("Save request failed")
        }
    }

    /// Initialize IMU with flash calibration loading
    /// Attempts to load calibration from flash first, and only proceeds with full calibration if no saved calibration is found
    pub async fn initialize(&mut self) -> Result<(), &'static str> {
        info!("Core1: Initializing BNO055 IMU...");
        self.set_led_pattern(LedPattern::SlowBlink(colors::BLUE))
            .await;

        // Initialize BNO055 hardware
        let mut delay = Delay;
        for attempt in 0..3 {
            match self.bno.init(&mut delay) {
                Ok(_) => {
                    info!("Core1: BNO055 initialized on attempt {}", attempt + 1);
                    break;
                }
                Err(e) => {
                    error!(
                        "Core1: Init attempt {} failed: {:?}",
                        attempt + 1,
                        Debug2Format(&e)
                    );
                    if attempt == 2 {
                        self.set_led_pattern(LedPattern::RapidFlash(colors::RED))
                            .await;
                        return Err("Failed to initialize BNO055 after 3 attempts");
                    }
                    Timer::after(Duration::from_millis(100)).await;
                }
            }
        }

        // Configure BNO055
        self.bno
            .set_axis_sign(BNO055AxisSign::Y_NEGATIVE | BNO055AxisSign::Z_NEGATIVE)
            .map_err(|_| "Failed to set axis sign")?;

        self.bno
            .set_mode(bno055::BNO055OperationMode::NDOF, &mut delay)
            .map_err(|_| "Failed to set NDOF mode")?;

        // Try to load saved calibration
        match self.load_calibration().await {
            Ok(true) => {
                info!("Core1: Using saved calibration - shorter wait time");
                // Update status
                {
                    let mut status = IMU_STATUS.write().await;
                    status.initialized = true;
                    status.last_update = Instant::now();
                }
                self.set_led_pattern(LedPattern::Solid(colors::GREEN)).await;
                return Ok(());
            }
            Ok(false) => {
                info!("Core1: No saved calibration, will perform full calibration");
            }
            Err(e) => {
                warn!("Core1: Failed to load calibration: {}", e);
            }
        }

        // Update status for normal calibration path
        {
            let mut status = IMU_STATUS.write().await;
            status.initialized = true;
            status.last_update = Instant::now();
        }

        self.set_led_pattern(LedPattern::Pulse(colors::CYAN)).await;
        Ok(())
    }

    /// Wait for calibration with optional auto-save based on configuration
    /// Always performs calibration, but only saves to flash if imu-save-calibration feature is enabled
    pub async fn wait_for_calibration(&mut self, timeout_secs: u64) -> Result<(), &'static str> {
        info!("Core1: Waiting for IMU calibration...");

        #[cfg(feature = "imu-save-calibration")]
        info!("Core1: Calibration saving is enabled");
        #[cfg(not(feature = "imu-save-calibration"))]
        info!("Core1: Calibration saving is disabled");

        let start = Instant::now();
        let timeout = Duration::from_secs(timeout_secs);
        let mut best_quality = CalibrationLevels::new();

        loop {
            if start.elapsed() > timeout {
                warn!("Core1: Calibration timeout - proceeding with partial calibration");
                break;
            }

            match self.update_calibration_status().await {
                Ok(levels) => {
                    // Update LED to show calibration progress
                    self.set_led_pattern(LedPattern::Pulse(if levels.is_flight_ready() {
                        colors::GREEN
                    } else if levels.sys >= 2 {
                        colors::YELLOW
                    } else {
                        colors::ORANGE
                    }))
                    .await;

                    // Check if this is the best calibration we've seen
                    let current_quality = StoredCalibration::hash_quality(&levels);
                    let best_quality_hash = StoredCalibration::hash_quality(&best_quality);

                    if current_quality > best_quality_hash {
                        best_quality = levels;

                        // Auto-save improved calibration if enabled
                        if levels.is_flight_ready() && !self.calibration_loaded {
                            // Don't save if we just loaded calibration from flash
                            // save_calibration will check imu-save-calibration feature internally
                            if let Err(e) = self.save_calibration(&levels).await {
                                warn!("Core1: Failed to save calibration: {}", e);
                            }
                        } else if levels.is_flight_ready() && self.calibration_loaded {
                            info!(
                                "Core1: Skipping immediate save after loading calibration from flash"
                            );
                        }
                    }

                    if levels.is_flight_ready() {
                        info!("Core1: IMU calibration sufficient for flight!");
                        self.set_led_pattern(LedPattern::Solid(colors::GREEN)).await;

                        // Try to save final calibration if enabled, but not if we just loaded it
                        // save_calibration will check imu-save-calibration feature internally
                        if !self.calibration_loaded {
                            if let Err(e) = self.save_calibration(&levels).await {
                                warn!("Core1: Failed to save calibration: {}", e);
                            }
                        } else {
                            info!(
                                "Core1: Skipping immediate save after loading calibration from flash"
                            );
                        }
                        return Ok(());
                    }
                }
                Err(e) => error!("Core1: Calibration check error: {}", e),
            }

            Timer::after(Duration::from_millis(250)).await;
        }

        // Save best calibration achieved even if timeout, but not if we just loaded it
        if best_quality.is_flight_ready() {
            if !self.calibration_loaded {
                // save_calibration will check imu-save-calibration feature internally
                if let Err(e) = self.save_calibration(&best_quality).await {
                    warn!("Core1: Failed to save final calibration: {}", e);
                }

                #[cfg(feature = "imu-save-calibration")]
                info!("Core1: Final calibration saved to flash");
                #[cfg(not(feature = "imu-save-calibration"))]
                info!("Core1: Final calibration achieved but not saved (saving disabled)");
            } else {
                info!("Core1: Skipping immediate save after loading calibration from flash");
            }
        }

        Ok(())
    }

    async fn update_calibration_status(&mut self) -> Result<CalibrationLevels, &'static str> {
        match self.bno.get_calibration_status() {
            Ok(status) => {
                let levels = CalibrationLevels {
                    sys: status.sys,
                    gyro: status.gyr,
                    accel: status.acc,
                    mag: status.mag,
                };

                {
                    let mut imu_status = IMU_STATUS.write().await;
                    imu_status.calibration_status = levels;
                    imu_status.calibrated = levels.is_flight_ready();
                }

                // Only log when values change significantly
                if levels.sys != self.last_cal_log.sys
                    || levels.gyro != self.last_cal_log.gyro
                    || levels.accel != self.last_cal_log.accel
                    || levels.mag != self.last_cal_log.mag
                {
                    info!(
                        "Core1: Cal - Sys:{}/3 Gyro:{}/3 Acc:{}/3 Mag:{}/3",
                        levels.sys, levels.gyro, levels.accel, levels.mag
                    );
                    self.last_cal_log = levels;
                }

                Ok(levels)
            }
            Err(_) => Err("Failed to read calibration status"),
        }
    }

    pub async fn read_attitude(&mut self) -> Result<AttitudeData, &'static str> {
        // Read quaternion (more reliable than Euler angles)
        let quat = self
            .bno
            .quaternion()
            .map_err(|_| "Failed to read quaternion")?;

        // Read gyroscope for rates
        let gyro = self
            .bno
            .gyro_data()
            .map_err(|_| "Failed to read gyroscope")?;

        // Convert quaternion to Euler angles
        let (yaw, pitch, roll) = quaternion_to_euler(&quat);

        let now = Instant::now();
        let attitude = AttitudeData {
            pitch: -pitch,
            roll,
            yaw,
            pitch_rate: gyro.y, // Pitch rate around Y axis
            roll_rate: gyro.x,  // Roll rate around X axis
            yaw_rate: gyro.z,   // Yaw rate around Z axis
            timestamp: now,
        };

        // Update status
        {
            let mut status = IMU_STATUS.write().await;
            status.last_update = now;
            status.error_count = 0; // Reset on successful read
        }

        self.last_attitude = attitude;
        Ok(attitude)
    }

    pub async fn run(&mut self) {
        info!("Core1: Starting IMU reading task");

        let mut consecutive_errors = 0u32;
        let mut led_cycle = 0u32;
        let mut last_debug = Instant::now();
        let mut last_cal_check = Instant::now();

        // Set normal operation LED pattern
        self.set_led_pattern(LedPattern::DoubleBlink(colors::GREEN))
            .await;

        loop {
            match self.read_attitude().await {
                Ok(attitude) => {
                    consecutive_errors = 0;

                    let process_timer = TimingMeasurement::start();
                    // Signal new attitude data
                    ATTITUDE_SIGNAL.signal(attitude);
                    
                    // Send heartbeat signal to Core 0 for health monitoring
                    CORE1_HEARTBEAT.signal(());

                    // Update LED pattern based on attitude (optional visual feedback)
                    if led_cycle.is_multiple_of(500) {
                        // Check if we're level or tilted
                        let color = if attitude.roll.abs() > 0.5 || attitude.pitch.abs() > 0.5 {
                            colors::YELLOW // Significant tilt
                        } else {
                            colors::GREEN // Level flight
                        };
                        self.set_led_pattern(LedPattern::DoubleBlink(color)).await;
                    }
                    led_cycle = led_cycle.wrapping_add(1);

                    // Debug output every 100ms (10Hz)
                    if last_debug.elapsed() > Duration::from_millis(300) {
                        trace!(
                            "Core1: Pitch: {}°, Roll: {}°, Rate: {}°/s",
                            (attitude.pitch * 180.0 / core::f32::consts::PI) as i16,
                            (attitude.roll * 180.0 / core::f32::consts::PI) as i16,
                            attitude.pitch_rate
                        );
                        last_debug = Instant::now();
                    }

                    // Check calibration periodically
                    if last_cal_check.elapsed() > Duration::from_secs(60) {
                        info!("Core1: Checking IMU calibration status...");
                        let _ = self.update_calibration_status().await;

                        // Reset calibration_loaded flag after 60 seconds
                        // This allows saving calibration during periodic checks
                        if self.calibration_loaded {
                            info!(
                                "Core1: Resetting calibration_loaded flag to allow periodic saves"
                            );
                            self.calibration_loaded = false;
                        }

                        last_cal_check = Instant::now();
                    }

                    // Update performance metrics (only processing time, not sleep)
                    update_imu_timing(process_timer.elapsed_us());
                }
                Err(e) => {
                    consecutive_errors += 1;
                    error!("Core1: IMU read error ({}): {}", consecutive_errors, e);

                    // Rapid flash on errors
                    self.set_led_pattern(LedPattern::RapidFlash(colors::RED))
                        .await;

                    {
                        let mut status = IMU_STATUS.write().await;
                        status.error_count = consecutive_errors;
                    }

                    if consecutive_errors >= self.error_threshold {
                        error!("Core1: IMU failure threshold exceeded!");
                        // Signal invalid data to trigger failsafe
                        let mut failed_attitude = self.last_attitude;
                        failed_attitude.timestamp = Instant::from_ticks(0); // Invalid timestamp
                        ATTITUDE_SIGNAL.signal(failed_attitude);

                        // Try to recover
                        Timer::after(Duration::from_secs(1)).await;
                        consecutive_errors = 0;
                    }
                }
            }

            // Run at 4000Hz for smooth control
            Timer::after(Duration::from_micros(250)).await;
        }
    }
}

/// Convert quaternion to Euler angles (ZYX convention)
fn quaternion_to_euler(q: &mint::Quaternion<f32>) -> (f32, f32, f32) {
    let w = q.s;
    let x = q.v.x;
    let y = q.v.y;
    let z = q.v.z;

    // Roll (x-axis rotation)
    let sinr_cosp = 2.0 * (w * x + y * z);
    let cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    let roll = libm::atan2f(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    let sinp = 2.0 * (w * y - z * x);
    let pitch = if sinp.abs() >= 1.0 {
        libm::copysignf(core::f32::consts::PI / 2.0, sinp)
    } else {
        libm::asinf(sinp)
    };

    // Yaw (z-axis rotation)
    let siny_cosp = 2.0 * (w * z + x * y);
    let cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    let yaw = libm::atan2f(siny_cosp, cosy_cosp);

    (yaw, pitch, roll)
}

/// Helper function to check if attitude data is valid and recent
pub fn is_attitude_valid(attitude: &AttitudeData, max_age: Duration) -> bool {
    attitude.timestamp != Instant::from_ticks(0) && attitude.timestamp.elapsed() < max_age
}
