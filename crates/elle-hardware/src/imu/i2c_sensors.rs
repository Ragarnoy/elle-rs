//! Magnetometer (MMC5616WA) and barometer (BMP390) on I2C0, as their own task
//! on Core 1.
//!
//! These used to be blocking reads inside the 1 kHz IMU wake-up: ~250 µs each
//! at 400 kHz, a quarter of the IMU's 1 ms slot, during which it could not read
//! new samples. Here the bus is interrupt-driven: while bytes are on the wire
//! this task is suspended and the IMU task runs. Every operation is bounded by
//! a timeout, so a stuck bus disables both sensors instead of hanging Core 1.
//!
//! Outputs: raw counts to [`MAG`], offset-corrected field (sensor frame) to
//! [`MAG_FIELD`] for the AHRS, and [`BARO`]. Mag calibration runs here, on the
//! raw readings.

use core::cell::Cell;

use defmt::{Debug2Format, info, warn};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_rp::i2c::I2c;
use embassy_rp::mode::Async;
use embassy_sync::blocking_mutex::Mutex as BlockingMutex;
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, Ticker, with_timeout};
use mmc5616wa::asynch::Mmc5616waAsync;
use static_cell::StaticCell;

use super::{
    BARO, BaroReading, MAG, MAG_CAL_PROGRESS, MAG_CAL_RESULT_SIGNAL, MAG_CAL_START_SIGNAL,
    MAG_CALIBRATION_SIGNAL, MagReading,
};

/// Latest offset-corrected magnetic field in the sensor frame, for the AHRS.
/// `None` until the first reading, and again once the bus has failed (the AHRS
/// then runs 6-DOF instead of fusing a frozen vector). The IMU task applies the
/// level-cal mount when it fuses.
pub static MAG_FIELD: BlockingMutex<CriticalSectionRawMutex, Cell<Option<[f32; 3]>>> =
    BlockingMutex::new(Cell::new(None));

/// Magnetometer samples collected per hard-iron calibration (~30 s at 10 Hz).
const MAG_CAL_SAMPLES: u32 = 300;
// The live count is published through an `AtomicU16`.
const _: () = core::assert!(MAG_CAL_SAMPLES <= u16::MAX as u32);

/// Longest a single register read or write may take. A 9-byte read is ~0.3 ms
/// at 400 kHz; anything near this bound means the bus is stuck.
const I2C_OP_TIMEOUT: Duration = Duration::from_millis(20);

/// Baro read period (the loop period) and mag read period, from the intervals
/// the IMU used to count in its 1 kHz ticks.
const BARO_PERIOD_US: u64 = elle_config::BARO_READ_INTERVAL_TICKS as u64 * 1_000_000
    / elle_config::IMU_UPDATE_FREQUENCY_HZ as u64;
const MAG_PERIOD_US: u64 = elle_config::MAG_READ_INTERVAL_TICKS as u64 * 1_000_000
    / elle_config::IMU_UPDATE_FREQUENCY_HZ as u64;
const _: () = core::assert!(BARO_PERIOD_US > 0 && MAG_PERIOD_US >= BARO_PERIOD_US);

type Bus = Mutex<NoopRawMutex, I2c<'static, Async>>;
type Dev = I2cDevice<'static, NoopRawMutex, I2c<'static, Async>>;

static BUS: StaticCell<Bus> = StaticCell::new();

/// Hard-iron calibration in progress: running min/max per axis of raw counts.
struct MagCal {
    min: [f32; 3],
    max: [f32; 3],
    samples: u32,
}

/// Run the I2C sensors forever. Call from a Core 1 task.
pub async fn run(i2c: I2c<'static, Async>) -> ! {
    let bus: &'static Bus = BUS.init(Mutex::new(i2c));

    let mut mag = Mmc5616waAsync::new_default(I2cDevice::new(bus));
    let mut mag_ok = init_mag(&mut mag).await;
    let mut baro = init_baro(bus).await;

    let mut mag_offset = [0.0f32; 3];
    let mut mag_cal: Option<MagCal> = None;
    let mut prev_baro: Option<(f32, Instant)> = None;
    let mut vario_filtered = 0.0f32;
    let mut since_mag_us = MAG_PERIOD_US; // read the mag on the first pass

    let mut ticker = Ticker::every(Duration::from_micros(BARO_PERIOD_US));
    loop {
        ticker.next().await;

        // Offsets loaded or cleared by Core 0, and calibration requests.
        if let Some((ox, oy, oz)) = MAG_CALIBRATION_SIGNAL.try_take() {
            mag_offset = [ox, oy, oz];
            info!(
                "Core1: Mag cal offsets applied: ({}, {}, {})",
                ox as i32, oy as i32, oz as i32
            );
        }
        if MAG_CAL_START_SIGNAL.try_take().is_some() {
            mag_cal = Some(MagCal {
                min: [f32::MAX; 3],
                max: [f32::MIN; 3],
                samples: 0,
            });
            MAG_CAL_PROGRESS.store(0, core::sync::atomic::Ordering::Relaxed);
            info!("Core1: Mag calibration started — rotate board in all orientations");
        }

        let mut failed: Option<&str> = None;

        since_mag_us += BARO_PERIOD_US;
        if mag_ok && since_mag_us >= MAG_PERIOD_US {
            since_mag_us = 0;
            let started = Instant::now();
            let read = with_timeout(I2C_OP_TIMEOUT, mag.read_magnetic()).await;
            crate::timing::CORE1_LOAD.record_mag(started.elapsed().as_micros() as u32);
            match read {
                Ok(Ok(data)) => {
                    MAG.publish(MagReading {
                        x: data.x,
                        y: data.y,
                        z: data.z,
                    });
                    let raw = [data.x as f32, data.y as f32, data.z as f32];
                    if let Some(cal) = mag_cal.as_mut()
                        && let Some(offsets) = mag_cal_step(cal, &raw)
                    {
                        if let Some(o) = offsets {
                            mag_offset = o;
                        }
                        mag_cal = None;
                    }
                    // Hard-iron offsets are sensor-frame; the mount is applied at fusion.
                    let field = [
                        raw[0] - mag_offset[0],
                        raw[1] - mag_offset[1],
                        raw[2] - mag_offset[2],
                    ];
                    MAG_FIELD.lock(|c| c.set(Some(field)));
                }
                Ok(Err(e)) => {
                    warn!("MMC5616WA: read error: {}", Debug2Format(&e));
                    failed = Some("MMC5616WA");
                }
                Err(_) => {
                    warn!("MMC5616WA: read timed out");
                    failed = Some("MMC5616WA");
                }
            }
        }

        if failed.is_none()
            && let Some(b) = baro.as_mut()
        {
            let started = Instant::now();
            let read = with_timeout(I2C_OP_TIMEOUT, b.measure()).await;
            crate::timing::CORE1_LOAD.record_baro(started.elapsed().as_micros() as u32);
            match read {
                Ok(Ok(m)) => {
                    use uom::si::length::meter;
                    use uom::si::pressure::hectopascal;
                    use uom::si::thermodynamic_temperature::degree_celsius;
                    let alt = m.altitude.get::<meter>();
                    let now = Instant::now();
                    // Vario from altitude differentiation + EMA (alpha 0.3, ~150 ms at 20 Hz)
                    let raw_vario = match prev_baro {
                        Some((prev_alt, prev_time)) => {
                            let dt_s = (now - prev_time).as_micros() as f32 / 1_000_000.0;
                            if dt_s > 0.001 {
                                (alt - prev_alt) / dt_s
                            } else {
                                0.0
                            }
                        }
                        None => 0.0,
                    };
                    vario_filtered = vario_filtered * 0.7 + raw_vario * 0.3;
                    prev_baro = Some((alt, now));
                    BARO.publish(BaroReading {
                        pressure_hpa: m.pressure.get::<hectopascal>(),
                        temperature_c: m.temperature.get::<degree_celsius>(),
                        altitude_m: alt,
                        vario_ms: vario_filtered,
                    });
                }
                Ok(Err(e)) => {
                    warn!("BMP390: measure error: {}", Debug2Format(&e));
                    failed = Some("BMP390");
                }
                Err(_) => {
                    warn!("BMP390: measure timed out");
                    failed = Some("BMP390");
                }
            }
        }

        if let Some(who) = failed {
            // One error can leave the shared bus stuck, so both devices go.
            mag_ok = false;
            baro = None;
            MAG_FIELD.lock(|c| c.set(None));
            crate::elle_event!(
                error,
                crate::event::EVT_I2C_BUS_FAILED,
                "I2C0 error on {}: mag and baro disabled, AHRS on 6-DOF",
                who
            );
        }
    }
}

/// Feed one raw reading to a running calibration. `Some(result)` when it has
/// finished: the new offsets on success, `None` if the rotation was too small.
fn mag_cal_step(cal: &mut MagCal, raw: &[f32; 3]) -> Option<Option<[f32; 3]>> {
    for (i, &v) in raw.iter().enumerate() {
        cal.min[i] = cal.min[i].min(v);
        cal.max[i] = cal.max[i].max(v);
    }
    cal.samples += 1;
    MAG_CAL_PROGRESS.store(cal.samples as u16, core::sync::atomic::Ordering::Relaxed);
    if cal.samples < MAG_CAL_SAMPLES {
        return None;
    }

    const MIN_RANGE: f32 = 5000.0;
    if (0..3).all(|i| cal.max[i] - cal.min[i] >= MIN_RANGE) {
        let o = [
            (cal.min[0] + cal.max[0]) / 2.0,
            (cal.min[1] + cal.max[1]) / 2.0,
            (cal.min[2] + cal.max[2]) / 2.0,
        ];
        info!(
            "Core1: Mag cal complete: offsets ({}, {}, {})",
            o[0] as i32, o[1] as i32, o[2] as i32
        );
        MAG_CAL_RESULT_SIGNAL.signal(Some((o[0], o[1], o[2])));
        Some(Some(o))
    } else {
        warn!("Core1: Mag cal FAILED — insufficient rotation");
        MAG_CAL_RESULT_SIGNAL.signal(None);
        Some(None)
    }
}

/// Reset, identify and start the magnetometer in continuous mode.
async fn init_mag(mag: &mut Mmc5616waAsync<Dev>) -> bool {
    let mut delay = embassy_time::Delay;
    let step = async {
        mag.soft_reset(&mut delay)
            .await
            .map_err(|e| ("soft reset", e))?;
        mag.init(&mut delay).await.map_err(|e| ("init", e))?;
        mag.validate()
            .await
            .map_err(|e| ("chip ID validation", e))?;
        mag.start_continuous(255)
            .await
            .map_err(|e| ("start continuous", e))
    };
    match with_timeout(Duration::from_millis(200), step).await {
        Ok(Ok(())) => {
            info!("MMC5616WA: initialized, continuous mode (chip ID OK)");
            true
        }
        Ok(Err((what, e))) => {
            crate::elle_event!(
                warn,
                crate::event::EVT_MAG_INIT_FAILED,
                "MMC5616WA: {} failed: {}",
                what,
                Debug2Format(&e)
            );
            false
        }
        Err(_) => {
            crate::elle_event!(
                warn,
                crate::event::EVT_MAG_INIT_FAILED,
                "MMC5616WA: init timed out"
            );
            false
        }
    }
}

/// Find the barometer at 0x77 or 0x76 and configure it (osrs_p x8, osrs_t x1,
/// IIR coef 3, ODR 50 Hz: datasheet standard resolution).
async fn init_baro(bus: &'static Bus) -> Option<bmp390::Bmp390<Dev>> {
    let config = bmp390::Configuration {
        iir_filter: bmp390::Config {
            iir_filter: bmp390::IirFilter::coef_3,
        },
        ..bmp390::Configuration::default()
    };
    for (addr, addr_str) in [
        (bmp390::Address::Up, "0x77"),
        (bmp390::Address::Down, "0x76"),
    ] {
        let attempt =
            bmp390::Bmp390::try_new(I2cDevice::new(bus), addr, embassy_time::Delay, &config);
        match with_timeout(Duration::from_millis(200), attempt).await {
            Ok(Ok(baro)) => {
                info!(
                    "BMP390: initialized at {} (pressure + temperature)",
                    addr_str
                );
                return Some(baro);
            }
            Ok(Err(e)) => {
                crate::elle_event!(
                    warn,
                    crate::event::EVT_BARO_INIT_FAILED,
                    "BMP390: init failed at {}: {}",
                    addr_str,
                    Debug2Format(&e)
                );
            }
            Err(_) => {
                crate::elle_event!(
                    warn,
                    crate::event::EVT_BARO_INIT_FAILED,
                    "BMP390: init timed out at {}",
                    addr_str
                );
            }
        }
    }
    None
}
