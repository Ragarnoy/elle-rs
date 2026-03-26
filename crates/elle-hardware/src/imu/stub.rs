use super::*;

type I2cBus<'a> =
    embassy_rp::i2c::I2c<'a, embassy_rp::peripherals::I2C0, embassy_rp::i2c::Blocking>;
type SharedI2c<'a> = embedded_hal_bus::i2c::RefCellDevice<'a, I2cBus<'a>>;

pub struct Imu<'a> {
    mag: mmc5616wa::Mmc5616wa<SharedI2c<'a>>,
    baro: Option<bmp390::sync::Bmp390<SharedI2c<'a>>>,
    i2c_bus: &'a core::cell::RefCell<I2cBus<'a>>,
}

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
            ATTITUDE.publish(attitude);
            crate::crsf::TELEMETRY_ATTITUDE.signal(attitude);

            // Read MMC5616WA magnetometer at ~10 Hz (every 400 ticks at 4 kHz)
            mag_counter += 1;
            if mag_counter >= 400 {
                mag_counter = 0;
                match self.mag.read_magnetic() {
                    Ok(data) => {
                        let reading = MagReading {
                            x: data.x,
                            y: data.y,
                            z: data.z,
                        };
                        MAG.publish(reading);
                    }
                    Err(e) => {
                        warn!("MMC5616WA: read error: {}", e);
                    }
                }
            }

            // Read BMP390 barometer at ~20 Hz (every 200 ticks at 4 kHz)
            baro_counter += 1;
            if baro_counter >= 200 {
                baro_counter = 0;
                if let Some(baro) = &mut self.baro {
                    match baro.measure() {
                        Ok(m) => {
                            use uom::si::length::meter;
                            use uom::si::pressure::hectopascal;
                            use uom::si::thermodynamic_temperature::degree_celsius;
                            let reading = BaroReading {
                                pressure_hpa: m.pressure.get::<hectopascal>(),
                                temperature_c: m.temperature.get::<degree_celsius>(),
                                altitude_m: m.altitude.get::<meter>(),
                                vario_ms: 0.0, // stub — no vario in disable-imu mode
                            };
                            BARO.publish(reading);
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
