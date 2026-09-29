use super::*;
use crate::led::{LedPattern, colors};
use elle_control::attitude::AttitudePipeline;
use elle_error::ImuError;
use embassy_rp::gpio::{Input, Output};
use embassy_rp::mode::Blocking;
use embassy_rp::spi;
use embassy_sync::channel::{Sender, TrySendError};
use embassy_time::{Duration, Timer};
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

/// Fuse one raw FIFO sample: remove the gyro bias, feed any running level
/// calibration, then step the shared pipeline (`elle_control::attitude`:
/// mount rotation, AHRS, rate filter). `mag` is already in the airframe frame.
fn fuse_sample(
    pipeline: &mut AttitudePipeline,
    level_cal: &mut Option<LevelCalAccum>,
    mag: Option<&nalgebra::Vector3<f32>>,
    sample: &icm426xx::Sample,
) -> AttitudeData {
    let (ax, ay, az) = sample.accel.unwrap_or((0.0, 0.0, 0.0));
    let (gx, gy, gz) = sample.gyro.unwrap_or((0.0, 0.0, 0.0));

    let raw_gyro = pipeline.debias(nalgebra::Vector3::new(gx, gy, gz));
    let raw_accel = nalgebra::Vector3::new(ax, ay, az);
    level_cal_step(level_cal, &mut pipeline.mount, &raw_accel, &raw_gyro);

    let a = pipeline.fuse(raw_gyro, raw_accel, mag);
    AttitudeData {
        pitch: a.pitch,
        roll: a.roll,
        yaw: a.yaw,
        pitch_rate: a.pitch_rate,
        roll_rate: a.roll_rate,
        yaw_rate: a.yaw_rate,
        timestamp: Instant::now(),
    }
}

/// Consecutive FIFO read errors before the attitude is published as stale and
/// the loop backs off for a second.
const IMU_ERROR_THRESHOLD: u32 = 10;

type SpiDev<'a> = ExclusiveDevice<spi::Spi<'a, Blocking>, Output<'a>, embassy_time::Delay>;

pub struct Imu<'a> {
    icm: Option<icm426xx::ICM42686<SpiDev<'a>, icm426xx::Ready>>,
    spi_dev: Option<SpiDev<'a>>,
    /// Bias, mount, AHRS and rate filter (`elle_control::attitude`).
    pipeline: AttitudePipeline,
    led_sender: Sender<'a, CriticalSectionRawMutex, LedPattern, 8>,
    int1: Input<'a>,
    last_attitude: AttitudeData,
    /// In-progress level calibration, if one is collecting.
    level_cal: Option<LevelCalAccum>,
    /// Raw IMU capture for host replay (`imu-raw-log` builds).
    #[cfg(feature = "imu-raw-log")]
    raw_recorder: elle_control::imu_raw::Recorder,
    /// Boot-time gyro bias estimate, while it is still collecting.
    bias_est: Option<elle_control::gyro_bias::GyroBiasEstimator>,
}

impl<'a> Imu<'a> {
    pub fn new(
        spi_dev: SpiDev<'a>,
        led_sender: Sender<'a, CriticalSectionRawMutex, LedPattern, 8>,
        int1: Input<'a>,
    ) -> Self {
        Self {
            icm: None,
            spi_dev: Some(spi_dev),
            pipeline: AttitudePipeline::new(),
            #[cfg(feature = "imu-raw-log")]
            raw_recorder: elle_control::imu_raw::Recorder::new(),
            led_sender,
            int1,
            last_attitude: AttitudeData::zero(),
            level_cal: None,
            bias_est: Some(elle_control::gyro_bias::GyroBiasEstimator::new()),
        }
    }

    /// Send LED pattern update
    async fn set_led_pattern(&self, pattern: LedPattern) {
        if let Err(TrySendError::Full(_)) = self.led_sender.try_send(pattern) {
            warn!("Core1: LED channel full, falling back to awaited send");
            let _ = self.led_sender.send(pattern).await;
        }
    }

    /// Initialize the ICM-42686-P. The magnetometer and barometer start in their
    /// own task (`i2c_sensors`).
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

        // Mark IMU as initialized. `calibrated` stays false until the gyro bias
        // has been measured at the start of `run()`.
        {
            let mut status = IMU_STATUS.write().await;
            status.initialized = true;
            status.last_update = Instant::now();
        }

        self.set_led_pattern(LedPattern::Solid(colors::GREEN)).await;
        Ok(())
    }

    /// Nothing to wait for here: the gyro bias is measured on the first still
    /// second of `run()`, without holding up the boot barrier.
    pub async fn wait_for_calibration(&mut self, _timeout_secs: u64) -> ElleResult<()> {
        info!("ICM-42686: gyro bias will be measured once running (keep still)");
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
        let mut tap_counter: u32 = 0;
        let mut last_catchup_event: Option<Instant> = None;

        // Busy time of the previous wake-up, recorded just before the next wait.
        let mut wake_started: Option<Instant> = None;

        loop {
            // Wait for DATA_RDY: INT1 goes high when new sample is ready
            // (active-high, latched — cleared on INT_STATUS read inside read_sample).
            // Uses true async GPIO (embassy-rp multicore executor enables cross-core IRQ wakeup).
            if let Some(started) = wake_started.take() {
                let busy_us = started.elapsed().as_micros() as u32;
                crate::timing::CORE1_LOAD.record_wake(busy_us);
                #[cfg(feature = "performance-monitoring")]
                crate::timing::IMU_TIMING.record(busy_us);
            }
            self.int1.wait_for_high().await;
            wake_started = Some(Instant::now());

            // 1. Drain the ICM-42686 FIFO. Every queued sample goes through the
            // AHRS in order (it integrates at a fixed 1 kHz step, so none may be
            // skipped); only the newest attitude is published. Reading one sample
            // per DATA_RDY would leave any backlog from a slow iteration (Core 1
            // paused for a flash operation, say) queued for good, and the
            // published attitude that much older than its timestamp.
            //
            // Latest mag field from the I2C task, into the airframe frame.
            let mag = i2c_sensors::MAG_FIELD.lock(|c| c.get()).map(|m| {
                self.pipeline
                    .mag_to_airframe(nalgebra::Vector3::new(m[0], m[1], m[2]))
            });
            let mut drained: u32 = 0;
            let mut latest: Option<AttitudeData> = None;
            let mut bias_result = None;
            while drained < elle_config::IMU_MAX_DRAIN {
                match icm.read_sample() {
                    Ok(Some((sample, more))) => {
                        consecutive_errors = 0;
                        drained += 1;
                        if let Some(est) = self.bias_est.as_mut()
                            && let Some((gx, gy, gz)) = sample.gyro
                            && let Some(result) = est.push(nalgebra::Vector3::new(gx, gy, gz))
                        {
                            self.bias_est = None;
                            bias_result = Some(result);
                            if let Ok(bias) = result {
                                self.pipeline.gyro_bias = bias;
                            }
                        }
                        #[cfg(feature = "imu-raw-log")]
                        let quat_before = self.pipeline.quat();
                        let fused = fuse_sample(
                            &mut self.pipeline,
                            &mut self.level_cal,
                            mag.as_ref(),
                            &sample,
                        );
                        #[cfg(feature = "imu-raw-log")]
                        self.raw_recorder.sample(
                            Instant::now().as_micros(),
                            sample.gyro.unwrap_or((0.0, 0.0, 0.0)),
                            sample.accel.unwrap_or((0.0, 0.0, 0.0)),
                            sample.temperature_celsius,
                            mag.as_ref(),
                            &quat_before,
                            &self.pipeline.gyro_bias,
                            &self.pipeline.mount,
                            |r| {
                                let _ = super::IMU_RAW_CHANNEL.try_send(r);
                            },
                        );
                        latest = Some(fused);
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
                        if consecutive_errors >= IMU_ERROR_THRESHOLD {
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

            match bias_result {
                Some(Ok(b)) => {
                    IMU_STATUS.write().await.calibrated = true;
                    crate::elle_event!(
                        info,
                        crate::event::EVT_GYRO_BIAS_DONE,
                        "ICM-42686: gyro bias ({}, {}, {}) mrad/s",
                        (b.x * 1000.0) as i32,
                        (b.y * 1000.0) as i32,
                        (b.z * 1000.0) as i32
                    );
                }
                Some(Err(e)) => {
                    crate::elle_event!(
                        warn,
                        crate::event::EVT_GYRO_BIAS_FAILED,
                        "ICM-42686: gyro bias not measured ({}), flying uncorrected",
                        e
                    );
                }
                None => {}
            }

            crate::timing::CORE1_LOAD.record_drain(drained);
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

            // Level calibration: loaded/cleared mount from Core0, or a start request
            if let Some(mount) = level_cal::LEVEL_CALIBRATION_SIGNAL.try_take() {
                self.pipeline.mount = mount;
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
                        crate::elle_event!(
                            info,
                            crate::event::EVT_DOUBLE_TAP,
                            "ICM-42686: double-tap detected"
                        );
                    } else if num == 1 {
                        info!("ICM-42686: single tap (num={})", num);
                    }
                }
            }
        }
    }
}
