//! BNO055 IMU integration for attitude sensing with LED status

use bno055::{BNO055AxisSign, mint};
use defmt::*;
use embassy_rp::Peri;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::i2c::{Blocking, I2c};
use embassy_rp::peripherals::{I2C0, PIN_25};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::rwlock::RwLock;
use embassy_sync::signal::Signal;
use embassy_time::{Delay, Duration, Instant, Timer};

/// Shared attitude data between cores/tasks
pub static ATTITUDE_SIGNAL: Signal<CriticalSectionRawMutex, AttitudeData> = Signal::new();

/// Mutex-protected IMU status for safe access
pub static IMU_STATUS: RwLock<CriticalSectionRawMutex, ImuStatus> = RwLock::new(ImuStatus::new());

#[derive(Clone, Copy, Debug, Format)]
pub struct AttitudeData {
    pub pitch: f32,      // radians
    pub roll: f32,       // radians
    pub yaw: f32,        // radians
    pub pitch_rate: f32, // rad/s
    pub timestamp: Instant,
}

impl AttitudeData {
    const fn zero() -> Self {
        Self {
            pitch: 0.0,
            roll: 0.0,
            yaw: 0.0,
            pitch_rate: 0.0,
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

    fn is_flight_ready(&self) -> bool {
        // Relaxed requirements for flight - magnetometer not critical
        self.sys >= 2 && self.gyro >= 3 && self.accel >= 2
    }
}

/// LED blink patterns for different states
#[derive(Clone, Copy)]
enum LedPattern {
    Off,
    Solid,
    SlowBlink,   // 1Hz - Initializing
    FastBlink,   // 4Hz - Calibrating
    DoubleBlink, // Double flash - Normal operation
    RapidFlash,  // 10Hz - Error
}

pub struct BnoImu<'a> {
    bno: bno055::Bno055<I2c<'a, I2C0, Blocking>>,
    led: Output<'a>,
    last_attitude: AttitudeData,
    error_threshold: u32,
    last_cal_log: CalibrationLevels,
}

impl<'a> BnoImu<'a> {
    pub fn new(i2c: I2c<'a, I2C0, Blocking>, led_pin: Peri<'a, PIN_25>) -> Self {
        // Use default address (0x29) or alternative (0x28) based on your hardware
        let bno = bno055::Bno055::new(i2c).with_alternative_address();
        let led = Output::new(led_pin, Level::Low);

        Self {
            bno,
            led,
            last_attitude: AttitudeData::zero(),
            error_threshold: 10, // Max consecutive errors before failsafe
            last_cal_log: CalibrationLevels::new(),
        }
    }

    async fn set_led_pattern(&mut self, pattern: LedPattern) {
        match pattern {
            LedPattern::Off => self.led.set_low(),
            LedPattern::Solid => self.led.set_high(),
            LedPattern::SlowBlink => {
                // Spawned as separate task to not block IMU reading
                self.led.toggle();
            }
            LedPattern::FastBlink => {
                self.led.toggle();
            }
            LedPattern::DoubleBlink => {
                // Quick double flash
                self.led.set_high();
                Timer::after(Duration::from_millis(50)).await;
                self.led.set_low();
                Timer::after(Duration::from_millis(50)).await;
                self.led.set_high();
                Timer::after(Duration::from_millis(50)).await;
                self.led.set_low();
            }
            LedPattern::RapidFlash => {
                self.led.toggle();
            }
        }
    }

    pub async fn initialize(&mut self) -> Result<(), &'static str> {
        info!("Core1: Initializing BNO055 IMU...");

        // LED pattern: slow blink during init
        let mut delay = Delay;

        // Initialize with retries
        for attempt in 0..3 {
            self.set_led_pattern(LedPattern::SlowBlink).await;

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
                        self.set_led_pattern(LedPattern::RapidFlash).await;
                        return Err("Failed to initialize BNO055 after 3 attempts");
                    }
                    Timer::after(Duration::from_millis(100)).await;
                }
            }
        }

        // BNO is inverted by default, so we need to flip the sign of the Y and Z axes
        self.bno
            .set_axis_sign(BNO055AxisSign::Y_NEGATIVE | BNO055AxisSign::Z_NEGATIVE)
            .map_err(|_| "Failed to set axis sign")?;

        // Set to NDOF mode for full sensor fusion
        Timer::after(Duration::from_millis(50)).await;

        match self
            .bno
            .set_mode(bno055::BNO055OperationMode::NDOF, &mut delay)
        {
            Ok(_) => info!("Core1: BNO055 set to NDOF mode"),
            Err(e) => {
                error!("Core1: Failed to set NDOF mode: {:?}", Debug2Format(&e));
                self.set_led_pattern(LedPattern::RapidFlash).await;
                return Err("Failed to set NDOF mode");
            }
        }

        // Update status
        {
            let mut status = IMU_STATUS.write().await;
            status.initialized = true;
            status.last_update = Instant::now();
        }

        self.led.set_high(); // Solid on when initialized
        Timer::after(Duration::from_millis(50)).await;
        Ok(())
    }

    pub async fn wait_for_calibration(&mut self, timeout_secs: u64) -> Result<(), &'static str> {
        info!("Core1: Waiting for IMU calibration...");
        info!("Core1: Move the aircraft through figure-8 patterns for magnetometer");
        info!("Core1: Keep on flat surface for accelerometer calibration");

        let start = Instant::now();
        let timeout = Duration::from_secs(timeout_secs);
        let mut blink_counter = 0u8;

        loop {
            if start.elapsed() > timeout {
                warn!("Core1: Calibration timeout - proceeding with partial calibration");
                break;
            }

            // Fast blink during calibration
            if blink_counter % 2 == 0 {
                self.set_led_pattern(LedPattern::FastBlink).await;
            }
            blink_counter = blink_counter.wrapping_add(1);

            match self.update_calibration_status().await {
                Ok(levels) => {
                    if levels.is_flight_ready() {
                        info!("Core1: IMU calibration sufficient for flight!");

                        // Save calibration profile
                        let mut delay = Delay;
                        match self.bno.calibration_profile(&mut delay) {
                            Ok(profile) => {
                                info!("Core1: Calibration profile saved");
                                // You could store this in flash for next boot
                            }
                            Err(e) => {
                                warn!("Core1: Failed to save calibration: {:?}", Debug2Format(&e))
                            }
                        }

                        self.led.set_high(); // Solid when calibrated
                        return Ok(());
                    }
                }
                Err(e) => error!("Core1: Calibration check error: {}", e),
            }

            Timer::after(Duration::from_millis(250)).await;
        }

        Ok(()) // Proceed even if not fully calibrated
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
            pitch,
            roll,
            yaw,
            pitch_rate: gyro.y, // Pitch rate around Y axis
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

        loop {
            match self.read_attitude().await {
                Ok(attitude) => {
                    consecutive_errors = 0;

                    // Signal new attitude data
                    ATTITUDE_SIGNAL.signal(attitude);

                    // LED pattern: double blink every second during normal operation
                    if led_cycle % 100 == 0 {
                        self.set_led_pattern(LedPattern::DoubleBlink).await;
                    }
                    led_cycle = led_cycle.wrapping_add(1);

                    // Debug output every 100ms (10Hz)
                    if last_debug.elapsed() > Duration::from_millis(100) {
                        debug!(
                            "Core1: Pitch: {}°, Roll: {}°, Rate: {}°/s",
                            (attitude.pitch * 180.0 / core::f32::consts::PI) as i16,
                            (attitude.roll * 180.0 / core::f32::consts::PI) as i16,
                            (attitude.pitch_rate * 180.0 / core::f32::consts::PI) as i16
                        );
                        last_debug = Instant::now();
                    }

                    // Check calibration periodically
                    if last_cal_check.elapsed() > Duration::from_secs(5) {
                        let _ = self.update_calibration_status().await;
                        last_cal_check = Instant::now();
                    }
                }
                Err(e) => {
                    consecutive_errors += 1;
                    error!("Core1: IMU read error ({}): {}", consecutive_errors, e);

                    // Rapid flash on errors
                    self.set_led_pattern(LedPattern::RapidFlash).await;

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

            // Run at 100Hz for smooth control
            Timer::after(Duration::from_micros(2500)).await;
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
