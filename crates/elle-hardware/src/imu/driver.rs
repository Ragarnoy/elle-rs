use super::*;
use crate::led::{LedPattern, colors};
use ahrs::Ahrs;
use core::cell::RefCell;
use elle_error::ImuError;
use embassy_rp::gpio::{Input, Output};
use embassy_rp::i2c;
use embassy_rp::mode::Blocking;
use embassy_rp::spi;
use embassy_sync::channel::{Sender, TrySendError};
use embassy_time::{Duration, Timer};
use embedded_hal_bus::i2c::RefCellDevice as I2cRefCellDevice;
use embedded_hal_bus::spi::ExclusiveDevice;

/// Running sums for a level calibration (raw sensor frame).
struct LevelCalAccum {
    /// Samples seen so far, settle period included.
    seen: u32,
    accel_sum: nalgebra::Vector3<f32>,
    max_gyro: f32,
}

/// Feed one raw sample to an in-progress level calibration. After the settle
/// period and `LEVEL_CAL_SAMPLES` averaged samples it finishes: on success the new
/// mount is applied immediately, on failure the old one stays; either way the
/// result goes to Core0 to report and persist.
fn level_cal_step(
    acc: &mut Option<LevelCalAccum>,
    mount: &mut nalgebra::UnitQuaternion<f32>,
    raw_accel: &nalgebra::Vector3<f32>,
    raw_gyro: &nalgebra::Vector3<f32>,
) {
    use elle_config::{LEVEL_CAL_MAX_GYRO_RAD_S, LEVEL_CAL_SAMPLES, LEVEL_CAL_SETTLE_SAMPLES};

    let Some(a) = acc.as_mut() else {
        return;
    };
    a.seen += 1;
    if a.seen <= LEVEL_CAL_SETTLE_SAMPLES {
        return;
    }
    a.accel_sum += raw_accel;
    a.max_gyro = a.max_gyro.max(raw_gyro.norm());
    if a.seen < LEVEL_CAL_SETTLE_SAMPLES + LEVEL_CAL_SAMPLES {
        return;
    }

    let mean = a.accel_sum / LEVEL_CAL_SAMPLES as f32;
    let result = elle_control::level_cal::compute_mount(mean, a.max_gyro, LEVEL_CAL_MAX_GYRO_RAD_S);
    match result {
        Ok(m) => {
            *mount = m;
            info!("Core1: Level cal complete");
        }
        Err(e) => warn!("Core1: Level cal failed: {} (max gyro {})", e, a.max_gyro),
    }
    level_cal::LEVEL_CAL_RESULT_SIGNAL.signal(result);
    *acc = None;
}

/// Fuse one raw FIFO sample: feed any running level calibration, rotate into the
/// airframe frame, and step the AHRS (9-DOF when `mag` is given, else 6-DOF).
/// `None` when the AHRS rejects the sample (normalisation failure).
fn fuse_sample(
    ahrs: &mut ahrs::Madgwick<f32>,
    level_cal: &mut Option<LevelCalAccum>,
    mount: &mut nalgebra::UnitQuaternion<f32>,
    mag: Option<&nalgebra::Vector3<f32>>,
    sample: &icm426xx::Sample,
) -> Option<AttitudeData> {
    let (ax, ay, az) = sample.accel.unwrap_or((0.0, 0.0, 0.0));
    let (gx, gy, gz) = sample.gyro.unwrap_or((0.0, 0.0, 0.0));

    let raw_gyro = nalgebra::Vector3::new(gx, gy, gz);
    let raw_accel = nalgebra::Vector3::new(ax, ay, az);
    level_cal_step(level_cal, mount, &raw_accel, &raw_gyro);

    // Into the airframe frame; everything downstream (AHRS, rates)
    // then sees a level-mounted IMU.
    let gyro = *mount * raw_gyro;
    let accel = *mount * raw_accel;

    let q = match mag {
        Some(mag) => ahrs.update(&gyro, &accel, mag),
        None => ahrs.update_imu(&gyro, &accel),
    }
    .ok()?;

    let (roll, pitch, yaw) = q.euler_angles();

    // Board flat → pitch≈0, roll≈0; nose up → pitch>0; right wing down → roll>0.
    // Roll sign is inverted relative to the ICM-42686's raw frame on this PCB
    // orientation — field-confirmed rolling the wrong way with identity mapping.
    Some(AttitudeData {
        pitch,
        roll: -roll,
        yaw,
        pitch_rate: gyro.y,
        roll_rate: -gyro.x,
        yaw_rate: gyro.z,
        timestamp: Instant::now(),
    })
}

type I2cBus<'a> = i2c::I2c<'a, Blocking>;
type SharedI2c<'a> = I2cRefCellDevice<'a, I2cBus<'a>>;
type SpiDev<'a> = ExclusiveDevice<spi::Spi<'a, Blocking>, Output<'a>, embassy_time::Delay>;

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
    /// Level calibration: rotation from the IMU frame to the airframe frame,
    /// applied to every sensor vector before the AHRS. Identity = uncorrected.
    mount: nalgebra::UnitQuaternion<f32>,
    /// In-progress level calibration, if one is collecting.
    level_cal: Option<LevelCalAccum>,
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
            mount: nalgebra::UnitQuaternion::identity(),
            level_cal: None,
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
            bank0
                .int_source0()
                .modify(|_, w| w.fifo_ths_int1_en(0).ui_drdy_int1_en(1))
                .map_err(|_| ImuError::InitializationFailed)?;
            info!("ICM-42686: INT1 configured for DATA_RDY (active-high, latched)");

            // Enable APEX tap detection. dmp_power_save defaults to 1 (off), must be
            // cleared or the DMP never runs and tap_enable has no effect.
            // Tap sensitivity runs on chip defaults (jerk-based; APEX_CONFIG7,
            // TAP_MIN_JERK_THR=17) — the ICM-42686-P has no amplitude threshold.
            bank0
                .apex_config0()
                .modify(|_, w| w.tap_enable(1).dmp_power_save(0))
                .map_err(|_| ImuError::InitializationFailed)?;

            info!("ICM-42686: APEX tap detection enabled (chip default sensitivity, DMP active)");
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

        // 3. Initialize BMP390/BMP384 barometer on I2C0 — try 0x77 then 0x76
        // Standard resolution config (datasheet Section 3.5): osrs_p=x8, osrs_t=x1, IIR=coef_3, ODR=50Hz
        let baro_config = bmp390::Configuration {
            iir_filter: bmp390::Config {
                iir_filter: bmp390::IirFilter::coef_3,
            },
            ..bmp390::Configuration::default()
        };
        let addresses = [
            (bmp390::Address::Up, "0x77"),
            (bmp390::Address::Down, "0x76"),
        ];
        for (addr, addr_str) in addresses {
            let baro_i2c = I2cRefCellDevice::new(self.i2c_bus);
            match bmp390::sync::Bmp390::try_new(baro_i2c, addr, embassy_time::Delay, &baro_config) {
                Ok(baro) => {
                    self.baro = Some(baro);
                    info!(
                        "BMP390: initialized at {} (pressure + temperature)",
                        addr_str
                    );
                    break;
                }
                Err(e) => {
                    crate::elle_event!(
                        warn,
                        crate::event::EVT_BARO_INIT_FAILED,
                        "BMP390: init failed at {}: {}",
                        addr_str,
                        e
                    );
                }
            }
        }

        self.set_led_pattern(LedPattern::Solid(colors::GREEN)).await;
        Ok(())
    }

    /// Permanently disable magnetometer reads. Call before `run()` on platforms
    /// where the I2C bus is unreliable — prevents blocking hangs in the Core1 loop.
    pub fn disable_mag(&mut self) {
        self.mag_ok = false;
    }

    /// ICM-42686 has no user calibration (factory-calibrated MEMS)
    pub async fn wait_for_calibration(&mut self, _timeout_secs: u64) -> ElleResult<()> {
        info!("ICM-42686: no calibration needed (factory-calibrated MEMS)");
        Ok(())
    }

    /// Run continuous IMU reading with AHRS sensor fusion at 1 kHz.
    /// Waits on INT1 (DATA_RDY, active-high, latched) via async GPIO, then drains the FIFO.
    pub async fn run(&mut self) -> ! {
        info!("Core1: Starting ICM-42686 + AHRS fusion loop (INT1 DATA_RDY)");

        let icm = self.icm.as_mut().expect("ICM not initialized");

        // Flush FIFO — it accumulated samples during the supervisor barrier wait
        if let Err(e) = icm.reset_fifo() {
            warn!("ICM-42686: FIFO flush failed: {:?}", Debug2Format(&e));
        }

        let mut consecutive_errors: u32 = 0;
        let mut mag_counter: u32 = 0;
        let mut baro_counter: u32 = 0;
        let mut tap_counter: u32 = 0;
        let mut last_catchup_event: Option<Instant> = None;

        loop {
            // Wait for DATA_RDY: INT1 goes high when new sample is ready
            // (active-high, latched — cleared on INT_STATUS read inside read_sample).
            // Uses true async GPIO (embassy-rp multicore executor enables cross-core IRQ wakeup).
            self.int1.wait_for_high().await;

            // 1. Drain the ICM-42686 FIFO. Every queued sample goes through the
            // AHRS in order (it integrates at a fixed 1 kHz step, so none may be
            // skipped); only the newest attitude is published. Reading one sample
            // per DATA_RDY would leave any backlog from a slow iteration (mag/baro
            // I2C) queued for good, and the published attitude that much older
            // than its timestamp.
            let mut drained: u32 = 0;
            let mut latest: Option<AttitudeData> = None;
            while drained < elle_config::IMU_MAX_DRAIN {
                match icm.read_sample() {
                    Ok(Some((sample, more))) => {
                        consecutive_errors = 0;
                        drained += 1;
                        let fused = fuse_sample(
                            &mut self.ahrs,
                            &mut self.level_cal,
                            &mut self.mount,
                            self.has_mag.then_some(&self.last_mag),
                            &sample,
                        );
                        if fused.is_some() {
                            latest = fused;
                        }
                        if !more {
                            break;
                        }
                    }
                    Ok(None) => break,
                    Err(icm426xx::Error::FifoOverflow) => {
                        // FIFO overflowed — flush and restart
                        let _ = icm.reset_fifo();
                        crate::elle_event!(
                            warn,
                            crate::event::EVT_IMU_FIFO_OVERFLOW,
                            "ICM-42686: FIFO overflow, flushed"
                        );
                        break;
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
                        break;
                    }
                }
            }

            if let Some(attitude) = latest {
                ATTITUDE.publish(attitude);
                crate::crsf::TELEMETRY_ATTITUDE.signal(attitude);

                CORE1_HEARTBEAT.signal(());
                self.last_attitude = attitude;

                let mut status = IMU_STATUS.write().await;
                status.last_update = Instant::now();
                status.error_count = 0;
            }

            if drained > 1 {
                let now = Instant::now();
                if last_catchup_event.is_none_or(|t| now - t >= Duration::from_secs(1)) {
                    last_catchup_event = Some(now);
                    crate::elle_event!(
                        info,
                        crate::event::EVT_IMU_CATCHUP,
                        "ICM-42686: caught up {} queued samples",
                        drained
                    );
                }
            }
            // Housekeeping below runs on sensor time: advance by samples consumed.
            let ticks = drained.max(1);

            // Check for loaded mag cal offsets from Core0
            if let Some((ox, oy, oz)) = MAG_CALIBRATION_SIGNAL.try_take() {
                self.mag_offset = nalgebra::Vector3::new(ox, oy, oz);
                info!(
                    "Core1: Mag cal offsets applied: ({}, {}, {})",
                    ox as i32, oy as i32, oz as i32
                );
            }

            // Level calibration: loaded/cleared mount from Core0, or a start request
            if let Some(mount) = level_cal::LEVEL_CALIBRATION_SIGNAL.try_take() {
                self.mount = mount;
                info!("Core1: Level cal mount applied");
            }
            if level_cal::LEVEL_CAL_START_SIGNAL.try_take().is_some() {
                self.level_cal = Some(LevelCalAccum {
                    seen: 0,
                    accel_sum: nalgebra::Vector3::zeros(),
                    max_gyro: 0.0,
                });
                info!("Core1: Level calibration collecting");
            }

            // Poll APEX tap detection every TAP_POLL_INTERVAL samples (~50ms).
            // INT_STATUS3.tap_det_int clears on read, so no separate clear needed.
            const TAP_POLL_INTERVAL: u32 = 50;
            tap_counter += ticks;
            if tap_counter >= TAP_POLL_INTERVAL {
                tap_counter = 0;
                let mut bank0 = icm.ll().bank::<0>();
                if let Ok(s) = bank0.int_status3().read()
                    && s.tap_det_int() != 0
                    && let Ok(d) = bank0.apex_data4().read()
                {
                    let num = d.tap_num();
                    // num: 1=single, 2=double
                    if num == 2 {
                        crate::imu::TAP_SIGNAL.signal(());
                        info!("ICM-42686: double-tap detected");
                    } else if num == 1 {
                        info!("ICM-42686: single tap (num={})", num);
                    }
                }
            }

            // Check for calibration start request
            if MAG_CAL_START_SIGNAL.try_take().is_some() {
                self.mag_cal_active = true;
                self.mag_cal_min = [f32::MAX; 3];
                self.mag_cal_max = [f32::MIN; 3];
                self.mag_cal_samples = 0;
                crate::imu::MAG_CAL_PROGRESS.store(0, core::sync::atomic::Ordering::Relaxed);
                info!("Core1: Mag calibration started — rotate board in all orientations");
            }

            // 4. Read MMC5616WA at ~10 Hz
            mag_counter += ticks;
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
                            crate::imu::MAG_CAL_PROGRESS.store(
                                self.mag_cal_samples as u16,
                                core::sync::atomic::Ordering::Relaxed,
                            );

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
                        // (hard-iron offsets are sensor-frame), then into the airframe frame.
                        self.last_mag = self.mount
                            * nalgebra::Vector3::new(
                                raw[0] - self.mag_offset[0],
                                raw[1] - self.mag_offset[1],
                                raw[2] - self.mag_offset[2],
                            );
                        self.has_mag = true;
                    }
                    Err(e) => {
                        // Any I2C error leaves the bus stuck on this hardware — the next call
                        // would block Core1 indefinitely. Disable mag immediately, and drop
                        // has_mag so the AHRS falls back to 6-DOF instead of fusing the
                        // frozen last_mag vector forever (which drags yaw to a stale heading).
                        warn!("MMC5616WA: read error: {}", e);
                        self.mag_ok = false;
                        self.has_mag = false;
                        crate::elle_event!(
                            warn,
                            crate::event::EVT_IMU_INIT_FAILED,
                            "MMC5616WA: disabling mag after I2C error — falling back to 6-DOF"
                        );
                    }
                }
            }

            // 5. Read BMP390 at ~20 Hz
            baro_counter += ticks;
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
