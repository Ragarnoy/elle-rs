use super::*;
use crate::led::{LedPattern, colors};
use ahrs::Ahrs;
use core::cell::RefCell;
use elle_error::ImuError;
use embassy_rp::gpio::{Input, Output};
use embassy_rp::i2c;
use embassy_rp::peripherals::{I2C0, SPI0};
use embassy_rp::spi;
use embassy_sync::channel::{Sender, TrySendError};
use embassy_time::{Duration, Timer};
use embedded_hal_bus::i2c::RefCellDevice as I2cRefCellDevice;
use embedded_hal_bus::spi::ExclusiveDevice;

type I2cBus<'a> = i2c::I2c<'a, I2C0, i2c::Blocking>;
type SharedI2c<'a> = I2cRefCellDevice<'a, I2cBus<'a>>;
type SpiDev<'a> =
    ExclusiveDevice<spi::Spi<'a, SPI0, spi::Blocking>, Output<'a>, embassy_time::Delay>;

pub struct Imu<'a> {
    icm: Option<icm426xx::ICM42686<SpiDev<'a>, icm426xx::Ready>>,
    spi_dev: Option<SpiDev<'a>>,
    ahrs: ahrs::Madgwick<f32>,
    mag: mmc5616wa::Mmc5616wa<SharedI2c<'a>>,
    baro: Option<bmp390::sync::Bmp390<SharedI2c<'a>>>,
    i2c_bus: &'a RefCell<I2cBus<'a>>,
    led_sender: Sender<'a, CriticalSectionRawMutex, LedPattern, 8>,
    int1: Input<'a>,
    last_attitude: AttitudeData,
    last_mag: nalgebra::Vector3<f32>,
    has_mag: bool,
    mag_ok: bool,
    error_threshold: u32,
    mag_offset: nalgebra::Vector3<f32>,
    mag_cal_active: bool,
    mag_cal_min: [f32; 3],
    mag_cal_max: [f32; 3],
    mag_cal_samples: u32,
    prev_baro_alt: f32,
    prev_baro_time: Option<Instant>,
    vario_filtered: f32,
}

impl<'a> Imu<'a> {
    pub fn new(
        spi_dev: SpiDev<'a>,
        i2c_bus: &'a RefCell<I2cBus<'a>>,
        led_sender: Sender<'a, CriticalSectionRawMutex, LedPattern, 8>,
        int1: Input<'a>,
    ) -> Self {
        let mag_i2c = I2cRefCellDevice::new(i2c_bus);
        Self {
            icm: None,
            spi_dev: Some(spi_dev),
            ahrs: ahrs::Madgwick::new(
                elle_config::AHRS_SAMPLE_PERIOD_US as f32 / 1_000_000.0, // sample period in seconds
                elle_config::AHRS_BETA,
            ),
            mag: mmc5616wa::Mmc5616wa::new_default(mag_i2c),
            baro: None,
            i2c_bus,
            led_sender,
            int1,
            last_attitude: AttitudeData::zero(),
            last_mag: nalgebra::Vector3::zeros(),
            has_mag: false,
            mag_ok: false,
            error_threshold: 10,
            mag_offset: nalgebra::Vector3::zeros(),
            mag_cal_active: false,
            mag_cal_min: [f32::MAX; 3],
            mag_cal_max: [f32::MIN; 3],
            mag_cal_samples: 0,
            prev_baro_alt: 0.0,
            prev_baro_time: None,
            vario_filtered: 0.0,
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

        // Configure INT1 for DATA_RDY instead of FIFO threshold.
        // The driver init already set INT_CONFIG (push-pull, active-high, latched)
        // and INT_CONFIG1 (int_async_reset=0). We just need to switch the source.
        {
            let icm = self.icm.as_mut().unwrap();
            let mut bank0 = icm.ll().bank::<0>();
            // Disable FIFO threshold interrupt, enable data-ready interrupt.
            // INT1 is already configured as push-pull, active-high, latched
            // (driver defaults). Latched = stays asserted until INT_STATUS read,
            // which read_sample() does as part of its SPI transaction.
            bank0
                .int_source0()
                .modify(|_, w| w.fifo_ths_int1_en(0).ui_drdy_int1_en(1))
                .map_err(|_| ImuError::InitializationFailed)?;
            info!("ICM-42686: INT1 configured for DATA_RDY (active-high, latched)");
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

    /// Run continuous IMU reading with AHRS sensor fusion at 1 kHz.
    /// Polls INT1 pin (DATA_RDY, active-high, latched) — no async GPIO IRQ needed on Core1.
    pub async fn run(&mut self) -> ! {
        info!("Core1: Starting ICM-42686 + AHRS fusion loop (INT1 poll)");

        let icm = self.icm.as_mut().expect("ICM not initialized");

        // Flush FIFO — it accumulated samples during the supervisor barrier wait
        if let Err(e) = icm.reset_fifo() {
            warn!("ICM-42686: FIFO flush failed: {:?}", Debug2Format(&e));
        }

        let mut consecutive_errors: u32 = 0;
        let mut mag_counter: u32 = 0;
        let mut baro_counter: u32 = 0;

        loop {
            // Wait for DATA_RDY: INT1 goes high when new sample is ready
            // (active-high, latched — cleared on INT_STATUS read inside read_sample).
            // Uses true async GPIO (embassy-rp multicore executor enables cross-core IRQ wakeup).
            self.int1.wait_for_high().await;

            // 1. Read ICM-42686 FIFO sample (guaranteed to have data after INT1 high)
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

                    ATTITUDE.publish(attitude);
                    crate::crsf::TELEMETRY_ATTITUDE.signal(attitude);

                    CORE1_HEARTBEAT.signal(());
                    self.last_attitude = attitude;

                    {
                        let mut status = IMU_STATUS.write().await;
                        status.last_update = Instant::now();
                        status.error_count = 0;
                    }
                }
                Ok(None) => {
                    // Shouldn't happen with INT1 DATA_RDY — but yield just in case
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
                        ATTITUDE.publish(failed);
                        crate::crsf::TELEMETRY_ATTITUDE.signal(failed);
                        Timer::after(Duration::from_secs(1)).await;
                        consecutive_errors = 0;
                    }
                }
            }

            // Check for loaded mag cal offsets from Core0
            if let Some((ox, oy, oz)) = MAG_CALIBRATION_SIGNAL.try_take() {
                self.mag_offset = nalgebra::Vector3::new(ox, oy, oz);
                info!(
                    "Core1: Mag cal offsets applied: ({}, {}, {})",
                    ox as i32, oy as i32, oz as i32
                );
            }

            // Check for calibration start request
            if MAG_CAL_START_SIGNAL.try_take().is_some() {
                self.mag_cal_active = true;
                self.mag_cal_min = [f32::MAX; 3];
                self.mag_cal_max = [f32::MIN; 3];
                self.mag_cal_samples = 0;
                info!("Core1: Mag calibration started — rotate board in all orientations");
            }

            // 4. Read MMC5616WA at ~10 Hz
            mag_counter += 1;
            if self.mag_ok && mag_counter >= elle_config::MAG_READ_INTERVAL_TICKS {
                mag_counter = 0;
                match self.mag.read_magnetic() {
                    Ok(data) => {
                        let raw = [data.x as f32, data.y as f32, data.z as f32];

                        // Publish raw counts for diagnostics
                        let reading = MagReading {
                            x: data.x,
                            y: data.y,
                            z: data.z,
                        };
                        MAG.publish(reading);

                        // Calibration min/max tracking
                        if self.mag_cal_active {
                            for (i, &val) in raw.iter().enumerate() {
                                if val < self.mag_cal_min[i] {
                                    self.mag_cal_min[i] = val;
                                }
                                if val > self.mag_cal_max[i] {
                                    self.mag_cal_max[i] = val;
                                }
                            }
                            self.mag_cal_samples += 1;

                            // After 300 samples (~30s at 10Hz): compute and validate
                            if self.mag_cal_samples >= 300 {
                                const MIN_RANGE: f32 = 5000.0;
                                let ranges_ok = (0..3).all(|i| {
                                    (self.mag_cal_max[i] - self.mag_cal_min[i]) >= MIN_RANGE
                                });

                                if ranges_ok {
                                    let ox = (self.mag_cal_min[0] + self.mag_cal_max[0]) / 2.0;
                                    let oy = (self.mag_cal_min[1] + self.mag_cal_max[1]) / 2.0;
                                    let oz = (self.mag_cal_min[2] + self.mag_cal_max[2]) / 2.0;
                                    self.mag_offset = nalgebra::Vector3::new(ox, oy, oz);
                                    info!(
                                        "Core1: Mag cal complete: offsets ({}, {}, {})",
                                        ox as i32, oy as i32, oz as i32
                                    );
                                    MAG_CAL_RESULT_SIGNAL.signal(Some((ox, oy, oz)));
                                } else {
                                    warn!("Core1: Mag cal FAILED — insufficient rotation");
                                    MAG_CAL_RESULT_SIGNAL.signal(None);
                                }
                                self.mag_cal_active = false;
                            }
                        }

                        // Apply offsets before feeding AHRS
                        self.last_mag = nalgebra::Vector3::new(
                            raw[0] - self.mag_offset[0],
                            raw[1] - self.mag_offset[1],
                            raw[2] - self.mag_offset[2],
                        );
                        self.has_mag = true;
                    }
                    Err(e) => warn!("MMC5616WA: read error: {}", e),
                }
            }

            // 5. Read BMP390 at ~20 Hz
            baro_counter += 1;
            if baro_counter >= elle_config::BARO_READ_INTERVAL_TICKS {
                baro_counter = 0;
                if let Some(baro) = &mut self.baro {
                    match baro.measure() {
                        Ok(m) => {
                            use uom::si::length::meter;
                            use uom::si::pressure::hectopascal;
                            use uom::si::thermodynamic_temperature::degree_celsius;
                            let alt = m.altitude.get::<meter>();

                            // Compute vario from altitude differentiation + EMA filter
                            let now = Instant::now();
                            let raw_vario = if let Some(prev_time) = self.prev_baro_time {
                                let dt_s = (now - prev_time).as_micros() as f32 / 1_000_000.0;
                                if dt_s > 0.001 {
                                    (alt - self.prev_baro_alt) / dt_s
                                } else {
                                    0.0
                                }
                            } else {
                                0.0
                            };
                            // EMA: alpha=0.3 gives ~150ms effective time constant at 20Hz
                            self.vario_filtered = self.vario_filtered * 0.7 + raw_vario * 0.3;
                            self.prev_baro_alt = alt;
                            self.prev_baro_time = Some(now);

                            let reading = BaroReading {
                                pressure_hpa: m.pressure.get::<hectopascal>(),
                                temperature_c: m.temperature.get::<degree_celsius>(),
                                altitude_m: alt,
                                vario_ms: self.vario_filtered,
                            };
                            BARO.publish(reading);
                        }
                        Err(e) => warn!("BMP390: measure error: {}", e),
                    }
                }
            }

        }
    }
}
