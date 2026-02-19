//! ICM-42686-P IMU integration with AHRS sensor fusion
//!
//! When the `disable-imu` feature is enabled, this module provides stub implementations
//! that return synthetic attitude data for debugging without hardware.

use core::cell::Cell;
use defmt::*;
use elle_error::ElleResult;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::Instant;

// ============================================================================
// COMMON TYPES AND STATICS (available in both modes)
// ============================================================================

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
    #[must_use]
    pub const fn zero() -> Self {
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
    pub error_count: u32,
    pub last_update: Instant,
}

impl Default for ImuStatus {
    fn default() -> Self {
        Self::new()
    }
}

impl ImuStatus {
    #[must_use]
    pub const fn new() -> Self {
        Self {
            initialized: false,
            calibrated: false,
            error_count: 0,
            last_update: Instant::from_ticks(0),
        }
    }
}

/// Shared attitude data between cores/tasks
pub static ATTITUDE_SIGNAL: Signal<CriticalSectionRawMutex, AttitudeData> = Signal::new();

/// Non-consuming attitude cache for RPC queries.
/// Written alongside `ATTITUDE_SIGNAL` by the IMU task; read (without consuming) by
/// RPC handlers so they never race with the control loop's `try_take()`.
pub static ATTITUDE_CACHE: Mutex<CriticalSectionRawMutex, Cell<AttitudeData>> =
    Mutex::new(Cell::new(AttitudeData::zero()));

/// Global signal for Core 1 (IMU) heartbeat
pub static CORE1_HEARTBEAT: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Helper function to check if attitude data is valid and recent
#[must_use]
pub fn is_attitude_valid(attitude: &AttitudeData, max_age: embassy_time::Duration) -> bool {
    attitude.timestamp != Instant::from_ticks(0) && attitude.timestamp.elapsed() < max_age
}

/// Magnetometer data for RPC handler (populated by MMC5616WA)
pub static MAG_SIGNAL: Signal<CriticalSectionRawMutex, MagReading> = Signal::new();

/// Magnetometer reading (signed counts from MMC5616WA)
#[derive(Clone, Copy, Debug, Format)]
pub struct MagReading {
    pub x: i32,
    pub y: i32,
    pub z: i32,
}

/// Barometer data for RPC handler (populated by BMP390)
pub static BARO_SIGNAL: Signal<CriticalSectionRawMutex, BaroReading> = Signal::new();

/// Barometer reading from BMP390
#[derive(Clone, Copy, Debug, Format)]
pub struct BaroReading {
    pub pressure_hpa: f32,
    pub temperature_c: f32,
    pub altitude_m: f32,
}

/// Channel for LED pattern updates
pub static LED_COMMAND_CHANNEL: embassy_sync::channel::Channel<
    CriticalSectionRawMutex,
    crate::led::LedPattern,
    8,
> = embassy_sync::channel::Channel::new();

// ============================================================================
// REAL IMU IMPLEMENTATION (when disable-imu feature is NOT enabled)
// ============================================================================
#[cfg(not(feature = "disable-imu"))]
use embassy_sync::rwlock::RwLock;

#[cfg(not(feature = "disable-imu"))]
/// RwLock-protected IMU status for safe access
pub static IMU_STATUS: RwLock<CriticalSectionRawMutex, ImuStatus> = RwLock::new(ImuStatus::new());

#[cfg(not(feature = "disable-imu"))]
use crate::led::{LedPattern, colors};
#[cfg(not(feature = "disable-imu"))]
use ahrs::Ahrs;
#[cfg(not(feature = "disable-imu"))]
use core::cell::RefCell;
#[cfg(not(feature = "disable-imu"))]
use elle_error::ImuError;
#[cfg(not(feature = "disable-imu"))]
use embassy_rp::gpio::Output;
#[cfg(not(feature = "disable-imu"))]
use embassy_rp::i2c;
#[cfg(not(feature = "disable-imu"))]
use embassy_rp::peripherals::{I2C0, SPI0};
#[cfg(not(feature = "disable-imu"))]
use embassy_rp::spi;
#[cfg(not(feature = "disable-imu"))]
use embassy_sync::channel::{Sender, TrySendError};
#[cfg(not(feature = "disable-imu"))]
use embassy_time::{Duration, Timer};
#[cfg(not(feature = "disable-imu"))]
use embedded_hal_bus::i2c::RefCellDevice as I2cRefCellDevice;
#[cfg(not(feature = "disable-imu"))]
use embedded_hal_bus::spi::ExclusiveDevice;

#[cfg(not(feature = "disable-imu"))]
type I2cBus<'a> = i2c::I2c<'a, I2C0, i2c::Blocking>;
#[cfg(not(feature = "disable-imu"))]
type SharedI2c<'a> = I2cRefCellDevice<'a, I2cBus<'a>>;
#[cfg(not(feature = "disable-imu"))]
type SpiDev<'a> =
    ExclusiveDevice<spi::Spi<'a, SPI0, spi::Blocking>, Output<'a>, embassy_time::Delay>;

#[cfg(not(feature = "disable-imu"))]
pub struct Imu<'a> {
    icm: Option<icm426xx::ICM42686<SpiDev<'a>, icm426xx::Ready>>,
    spi_dev: Option<SpiDev<'a>>,
    ahrs: ahrs::Madgwick<f32>,
    mag: mmc5616wa::Mmc5616wa<SharedI2c<'a>>,
    baro: Option<bmp390::sync::Bmp390<SharedI2c<'a>>>,
    i2c_bus: &'a RefCell<I2cBus<'a>>,
    led_sender: Sender<'a, CriticalSectionRawMutex, LedPattern, 8>,
    last_attitude: AttitudeData,
    last_mag: nalgebra::Vector3<f32>,
    has_mag: bool,
    mag_ok: bool,
    error_threshold: u32,
}

#[cfg(not(feature = "disable-imu"))]
impl<'a> Imu<'a> {
    pub fn new(
        spi_dev: SpiDev<'a>,
        i2c_bus: &'a RefCell<I2cBus<'a>>,
        led_sender: Sender<'a, CriticalSectionRawMutex, LedPattern, 8>,
    ) -> Self {
        let mag_i2c = I2cRefCellDevice::new(i2c_bus);
        Self {
            icm: None,
            spi_dev: Some(spi_dev),
            ahrs: ahrs::Madgwick::new(
                elle_config::AHRS_SAMPLE_PERIOD_US as f32 / 1_000_000.0, // sample period in seconds
                0.033, // Madgwick beta (conservative)
            ),
            mag: mmc5616wa::Mmc5616wa::new_default(mag_i2c),
            baro: None,
            i2c_bus,
            led_sender,
            last_attitude: AttitudeData::zero(),
            last_mag: nalgebra::Vector3::zeros(),
            has_mag: false,
            mag_ok: false,
            error_threshold: 10,
        }
    }

    /// Send LED pattern update
    async fn set_led_pattern(&self, pattern: LedPattern) {
        if let Err(TrySendError::Full(_)) = self.led_sender.try_send(pattern) {
            warn!("Core1: LED channel full, falling back to awaited send");
            let _ = self.led_sender.send(pattern).await;
        }
    }

    /// Initialize ICM-42686-P, MMC5616WA magnetometer, and BMP390 barometer
    pub async fn initialize(&mut self) -> ElleResult<()> {
        info!("Core1: Initializing ICM-42686-P IMU...");
        self.set_led_pattern(LedPattern::SlowBlink(colors::BLUE))
            .await;

        // 1. Initialize ICM-42686-P on SPI0
        let spi_dev = self.spi_dev.take().expect("SPI device already consumed");
        let icm_uninit = icm426xx::ICM42686::new(spi_dev);
        let config = icm426xx::Config {
            rate: icm426xx::OutputDataRate::Hz1000,
            ..Default::default()
        };
        match icm_uninit.initialize(embassy_time::Delay, config) {
            Ok(icm) => {
                self.icm = Some(icm);
                info!("ICM-42686: initialized (WHO_AM_I OK, 1 kHz ODR)");
            }
            Err(e) => {
                crate::elle_event!(
                    error,
                    crate::event::EVT_IMU_INIT_FAILED,
                    "ICM-42686: init failed: {:?}",
                    Debug2Format(&e)
                );
                return Err(ImuError::InitializationFailed.into());
            }
        }

        // Mark IMU as initialized — ICM-42686 is factory-calibrated
        // Do this before I2C sensors so a hanging mag/baro doesn't block Core0
        {
            let mut status = IMU_STATUS.write().await;
            status.initialized = true;
            status.calibrated = true;
            status.last_update = Instant::now();
        }

        // 2. Initialize MMC5616WA magnetometer on I2C0
        let mut delay = embassy_time::Delay;
        let mag_init_ok = if let Err(e) = self.mag.soft_reset(&mut delay) {
            crate::elle_event!(
                warn,
                crate::event::EVT_MAG_INIT_FAILED,
                "MMC5616WA: soft reset failed: {}",
                e
            );
            false
        } else if let Err(e) = self.mag.init(&mut delay) {
            crate::elle_event!(
                warn,
                crate::event::EVT_MAG_INIT_FAILED,
                "MMC5616WA: init failed: {}",
                e
            );
            false
        } else if let Err(e) = self.mag.validate() {
            crate::elle_event!(
                warn,
                crate::event::EVT_MAG_INIT_FAILED,
                "MMC5616WA: chip ID validation failed: {}",
                e
            );
            false
        } else if let Err(e) = self.mag.start_continuous(255) {
            crate::elle_event!(
                warn,
                crate::event::EVT_MAG_INIT_FAILED,
                "MMC5616WA: start continuous failed: {}",
                e
            );
            false
        } else {
            info!("MMC5616WA: initialized, continuous mode (chip ID OK)");
            true
        };
        self.mag_ok = mag_init_ok;

        // 3. Initialize BMP390 barometer on I2C0 at address 0x76
        // Standard resolution config (datasheet Section 3.5): osrs_p=x8, osrs_t=x1, IIR=coef_3, ODR=50Hz
        let baro_config = bmp390::Configuration {
            iir_filter: bmp390::Config {
                iir_filter: bmp390::IirFilter::coef_3,
            },
            ..bmp390::Configuration::default()
        };
        let baro_i2c = I2cRefCellDevice::new(self.i2c_bus);
        match bmp390::sync::Bmp390::try_new(
            baro_i2c,
            bmp390::Address::Down,
            embassy_time::Delay,
            &baro_config,
        ) {
            Ok(baro) => {
                self.baro = Some(baro);
                info!("BMP390: initialized at 0x76 (pressure + temperature)");
            }
            Err(e) => {
                crate::elle_event!(
                    warn,
                    crate::event::EVT_BARO_INIT_FAILED,
                    "BMP390: init failed at 0x76: {}",
                    e
                );
            }
        }

        self.set_led_pattern(LedPattern::Solid(colors::GREEN)).await;
        Ok(())
    }

    /// ICM-42686 has no user calibration (factory-calibrated MEMS)
    pub async fn wait_for_calibration(&mut self, _timeout_secs: u64) -> ElleResult<()> {
        info!("ICM-42686: no calibration needed (factory-calibrated MEMS)");
        Ok(())
    }

    /// Run continuous IMU reading with AHRS sensor fusion at 1 kHz
    pub async fn run(&mut self) -> ! {
        info!("Core1: Starting ICM-42686 + AHRS fusion loop");

        let icm = self.icm.as_mut().expect("ICM not initialized");

        // Flush FIFO — it accumulated samples during the supervisor barrier wait
        if let Err(e) = icm.reset_fifo() {
            warn!("ICM-42686: FIFO flush failed: {:?}", Debug2Format(&e));
        }

        let mut consecutive_errors: u32 = 0;
        let mut mag_counter: u32 = 0;
        let mut baro_counter: u32 = 0;

        loop {
            // 1. Read ICM-42686 FIFO sample
            match icm.read_sample() {
                Ok(Some((sample, _more))) => {
                    consecutive_errors = 0;

                    let (ax, ay, az) = sample.accel.unwrap_or((0.0, 0.0, 0.0));
                    let (gx, gy, gz) = sample.gyro.unwrap_or((0.0, 0.0, 0.0));

                    let gyro = nalgebra::Vector3::new(gx, gy, gz);
                    let accel = nalgebra::Vector3::new(ax, ay, az);

                    // 2. Update AHRS (9-DOF with mag, or 6-DOF if no mag yet)
                    let q_result = if self.has_mag {
                        self.ahrs.update(&gyro, &accel, &self.last_mag)
                    } else {
                        self.ahrs.update_imu(&gyro, &accel)
                    };

                    let q = match q_result {
                        Ok(q) => q,
                        Err(_) => {
                            // AHRS normalization error — skip this sample
                            continue;
                        }
                    };

                    // 3. Extract Euler angles from quaternion
                    let (roll, pitch, yaw) = q.euler_angles();

                    // NOTE: Axis mapping may need sign adjustment on hardware.
                    // Verify: board flat → pitch≈0, roll≈0; nose up → pitch>0; right wing down → roll>0.
                    let attitude = AttitudeData {
                        pitch,
                        roll,
                        yaw,
                        pitch_rate: gy,
                        roll_rate: gx,
                        yaw_rate: gz,
                        timestamp: Instant::now(),
                    };

                    ATTITUDE_SIGNAL.signal(attitude);
                    ATTITUDE_CACHE.lock(|c| c.set(attitude));
                    #[cfg(feature = "crsf-telemetry")]
                    crate::crsf_telemetry::TELEMETRY_ATTITUDE.signal(attitude);

                    CORE1_HEARTBEAT.signal(());
                    self.last_attitude = attitude;

                    {
                        let mut status = IMU_STATUS.write().await;
                        status.last_update = Instant::now();
                        status.error_count = 0;
                    }
                }
                Ok(None) => {
                    // FIFO empty — yield until next sample arrives
                    Timer::after(Duration::from_micros(500)).await;
                }
                Err(icm426xx::Error::FifoOverflow) => {
                    // FIFO overflowed — flush and restart
                    let _ = icm.reset_fifo();
                    crate::elle_event!(
                        warn,
                        crate::event::EVT_IMU_FIFO_OVERFLOW,
                        "ICM-42686: FIFO overflow, flushed"
                    );
                }
                Err(e) => {
                    consecutive_errors += 1;
                    if consecutive_errors % 100 == 1 {
                        crate::elle_event!(
                            warn,
                            crate::event::EVT_IMU_READ_ERRORS,
                            "ICM-42686: read error ({}): {:?}",
                            consecutive_errors,
                            Debug2Format(&e)
                        );
                    }
                    if consecutive_errors >= self.error_threshold {
                        let mut failed = self.last_attitude;
                        failed.timestamp = Instant::from_ticks(0);
                        ATTITUDE_SIGNAL.signal(failed);
                        ATTITUDE_CACHE.lock(|c| c.set(failed));
                        #[cfg(feature = "crsf-telemetry")]
                        crate::crsf_telemetry::TELEMETRY_ATTITUDE.signal(failed);
                        Timer::after(Duration::from_secs(1)).await;
                        consecutive_errors = 0;
                    }
                }
            }

            // 4. Read MMC5616WA at ~10 Hz (every 100 iterations at 1 kHz)
            mag_counter += 1;
            if self.mag_ok && mag_counter >= 100 {
                mag_counter = 0;
                match self.mag.read_magnetic() {
                    Ok(data) => {
                        self.last_mag =
                            nalgebra::Vector3::new(data.x as f32, data.y as f32, data.z as f32);
                        self.has_mag = true;
                        MAG_SIGNAL.signal(MagReading {
                            x: data.x,
                            y: data.y,
                            z: data.z,
                        });
                    }
                    Err(e) => warn!("MMC5616WA: read error: {}", e),
                }
            }

            // 5. Read BMP390 at ~2 Hz (every 500 iterations at 1 kHz)
            baro_counter += 1;
            if baro_counter >= 500 {
                baro_counter = 0;
                if let Some(baro) = &mut self.baro {
                    match baro.measure() {
                        Ok(m) => {
                            use uom::si::length::meter;
                            use uom::si::pressure::hectopascal;
                            use uom::si::thermodynamic_temperature::degree_celsius;
                            BARO_SIGNAL.signal(BaroReading {
                                pressure_hpa: m.pressure.get::<hectopascal>(),
                                temperature_c: m.temperature.get::<degree_celsius>(),
                                altitude_m: m.altitude.get::<meter>(),
                            });
                        }
                        Err(e) => warn!("BMP390: measure error: {}", e),
                    }
                }
            }

            // Yield to other tasks briefly (only when FIFO had data — busy-drain)
            embassy_futures::yield_now().await;
        }
    }
}

// ============================================================================
// STUB IMU IMPLEMENTATION (when disable-imu feature IS enabled)
// ============================================================================
#[cfg(feature = "disable-imu")]
use embassy_sync::rwlock::RwLock;

#[cfg(feature = "disable-imu")]
/// RwLock-protected IMU status for safe access (stub version)
pub static IMU_STATUS: RwLock<CriticalSectionRawMutex, ImuStatus> = RwLock::new(ImuStatus::new());

#[cfg(feature = "disable-imu")]
type I2cBus<'a> =
    embassy_rp::i2c::I2c<'a, embassy_rp::peripherals::I2C0, embassy_rp::i2c::Blocking>;
#[cfg(feature = "disable-imu")]
type SharedI2c<'a> = embedded_hal_bus::i2c::RefCellDevice<'a, I2cBus<'a>>;

#[cfg(feature = "disable-imu")]
pub struct Imu<'a> {
    mag: mmc5616wa::Mmc5616wa<SharedI2c<'a>>,
    baro: Option<bmp390::sync::Bmp390<SharedI2c<'a>>>,
    i2c_bus: &'a core::cell::RefCell<I2cBus<'a>>,
}

#[cfg(feature = "disable-imu")]
impl<'a> Imu<'a> {
    /// Create stub IMU with MMC5616WA magnetometer and BMP390 barometer on shared I2C bus.
    pub fn new(
        i2c_bus: &'a core::cell::RefCell<I2cBus<'a>>,
        _led_sender: embassy_sync::channel::Sender<
            'a,
            CriticalSectionRawMutex,
            crate::led::LedPattern,
            8,
        >,
    ) -> Self {
        info!("IMU: DISABLED (stub with MMC5616WA + BMP390 on shared I2C)");
        let mag_i2c = embedded_hal_bus::i2c::RefCellDevice::new(i2c_bus);
        Self {
            mag: mmc5616wa::Mmc5616wa::new_default(mag_i2c),
            baro: None,
            i2c_bus,
        }
    }

    /// Initialize IMU (stub - immediately returns success, inits MMC5616WA + BMP390)
    pub async fn initialize(&mut self) -> ElleResult<()> {
        info!("IMU: Stub initialization - marking as ready");

        let mut delay = embassy_time::Delay;
        if let Err(e) = self.mag.soft_reset(&mut delay) {
            warn!("MMC5616WA: soft reset failed: {}", e);
        }
        if let Err(e) = self.mag.init(&mut delay) {
            warn!("MMC5616WA: init failed: {}", e);
        } else if let Err(e) = self.mag.validate() {
            warn!("MMC5616WA: chip ID validation failed: {}", e);
        } else if let Err(e) = self.mag.start_continuous(255) {
            warn!("MMC5616WA: start continuous failed: {}", e);
        } else {
            info!("MMC5616WA: initialized, continuous mode (chip ID OK)");
        }

        // Initialize BMP390 barometer — try Address::Up (0x77) first, fall back to Down (0x76)
        // Standard resolution config (datasheet Section 3.5): osrs_p=x8, osrs_t=x1, IIR=coef_3, ODR=50Hz
        let baro_config = bmp390::Configuration {
            iir_filter: bmp390::Config {
                iir_filter: bmp390::IirFilter::coef_3,
            },
            ..bmp390::Configuration::default()
        };
        let baro_i2c = embedded_hal_bus::i2c::RefCellDevice::new(self.i2c_bus);
        match bmp390::sync::Bmp390::try_new(
            baro_i2c,
            bmp390::Address::Up,
            embassy_time::Delay,
            &baro_config,
        ) {
            Ok(baro) => {
                self.baro = Some(baro);
                info!("BMP390: initialized at 0x77 (pressure + temperature)");
            }
            Err(_) => {
                info!("BMP390: 0x77 failed, trying 0x76...");
                let baro_i2c = embedded_hal_bus::i2c::RefCellDevice::new(self.i2c_bus);
                match bmp390::sync::Bmp390::try_new(
                    baro_i2c,
                    bmp390::Address::Down,
                    embassy_time::Delay,
                    &baro_config,
                ) {
                    Ok(baro) => {
                        self.baro = Some(baro);
                        info!("BMP390: initialized at 0x76 (pressure + temperature)");
                    }
                    Err(e) => warn!("BMP390: init failed on both addresses: {}", e),
                }
            }
        }

        // Mark IMU as initialized and calibrated immediately
        {
            let mut status = IMU_STATUS.write().await;
            status.initialized = true;
            status.calibrated = true;
            status.last_update = Instant::now();
        }

        Ok(())
    }

    /// Wait for calibration (stub - immediately returns success)
    pub async fn wait_for_calibration(&mut self, _timeout_secs: u64) -> ElleResult<()> {
        info!("IMU: Stub calibration - already 'calibrated'");
        Ok(())
    }

    /// Run continuous IMU reading (stub - generates synthetic test data + real mag + real baro)
    pub async fn run(&mut self) -> ! {
        info!("IMU: Starting stub IMU loop (synthetic test data)");

        let mut tick: u32 = 0;
        let mut mag_counter: u32 = 0;
        let mut baro_counter: u32 = 0;

        loop {
            // Slow sine waves: full cycle every ~16 s (4 kHz × 65536 ticks)
            let phase = (tick as f32) * (2.0 * core::f32::consts::PI / 65536.0);
            let attitude = AttitudeData {
                pitch: libm::sinf(phase) * 0.26,
                roll: libm::sinf(phase * 1.5) * 0.17,
                yaw: phase % (2.0 * core::f32::consts::PI),
                pitch_rate: libm::cosf(phase) * 0.04,
                roll_rate: libm::cosf(phase * 1.5) * 0.03,
                yaw_rate: 0.01,
                timestamp: Instant::now(),
            };
            tick = tick.wrapping_add(1);

            // Signal synthetic attitude data
            ATTITUDE_SIGNAL.signal(attitude);
            ATTITUDE_CACHE.lock(|c| c.set(attitude));
            #[cfg(feature = "crsf-telemetry")]
            crate::crsf_telemetry::TELEMETRY_ATTITUDE.signal(attitude);

            // Read MMC5616WA magnetometer at ~10 Hz (every 400 ticks at 4 kHz)
            mag_counter += 1;
            if mag_counter >= 400 {
                mag_counter = 0;
                match self.mag.read_magnetic() {
                    Ok(data) => {
                        MAG_SIGNAL.signal(MagReading {
                            x: data.x,
                            y: data.y,
                            z: data.z,
                        });
                    }
                    Err(e) => {
                        warn!("MMC5616WA: read error: {}", e);
                    }
                }
            }

            // Read BMP390 barometer at ~2 Hz (every 2000 ticks at 4 kHz)
            baro_counter += 1;
            if baro_counter >= 2000 {
                baro_counter = 0;
                if let Some(baro) = &mut self.baro {
                    match baro.measure() {
                        Ok(m) => {
                            use uom::si::length::meter;
                            use uom::si::pressure::hectopascal;
                            use uom::si::thermodynamic_temperature::degree_celsius;
                            BARO_SIGNAL.signal(BaroReading {
                                pressure_hpa: m.pressure.get::<hectopascal>(),
                                temperature_c: m.temperature.get::<degree_celsius>(),
                                altitude_m: m.altitude.get::<meter>(),
                            });
                        }
                        Err(e) => warn!("BMP390: measure error: {}", e),
                    }
                }
            }

            // Send heartbeat signal
            CORE1_HEARTBEAT.signal(());

            // Update status
            {
                let mut status = IMU_STATUS.write().await;
                status.last_update = Instant::now();
            }

            // Run at same rate as real IMU (4kHz)
            embassy_time::Timer::after(embassy_time::Duration::from_micros(250)).await;
        }
    }
}
