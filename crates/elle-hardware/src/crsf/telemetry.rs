//! CRSF telemetry TX — sends attitude, flight mode, and GPS frames to the radio.
//!
//! Frame format: `[0xC8 | len | type | payload... | crc8_dvb_s2]`
//! where `len` = payload_len + 2 (type + CRC bytes), CRC covers type + payload.

use crate::imu::{AttitudeData, BARO, MAG};
use crc::{CRC_8_DVB_S2, Crc};
use defmt::warn;
use embassy_rp::mode::Async;
use embassy_rp::uart::UartTx;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Ticker};

const CRSF_SYNC_BYTE: u8 = 0xC8;
const CRSF_FRAMETYPE_GPS: u8 = 0x02;
const CRSF_FRAMETYPE_BATTERY: u8 = 0x08;
const CRSF_FRAMETYPE_BARO: u8 = 0x09;
const CRSF_FRAMETYPE_ATTITUDE: u8 = 0x1E;
const CRSF_FRAMETYPE_FLIGHT_MODE: u8 = 0x21;

static CRC8: Crc<u8> = Crc::<u8>::new(&CRC_8_DVB_S2);

// ---------------------------------------------------------------------------
// Signals
// ---------------------------------------------------------------------------

/// Dedicated attitude signal for the telemetry task (separate from ATTITUDE
/// which is consumed by try_take in the control loop).
pub static TELEMETRY_ATTITUDE: Signal<CriticalSectionRawMutex, AttitudeData> = Signal::new();

/// Flight-mode signal, written by the main control loop after each fc.update().
pub static CRSF_FLIGHT_MODE: Signal<CriticalSectionRawMutex, CrsfFlightMode> = Signal::new();

/// Autotune display state for CRSF telemetry.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum AutotuneDisplay {
    /// No autotune activity
    Off,
    /// Pitch autotune running
    Pitch,
    /// Roll autotune running
    Roll,
    /// Autotune completed successfully (shown for a few seconds)
    Done,
    /// Autotune failed (shown for a few seconds)
    Error,
}

/// Flight mode for CRSF telemetry display.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum CrsfControlMode {
    Manual,
    Stabilized,
    AltitudeHold,
}

/// Snapshot of the current flight mode for CRSF telemetry.
#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct CrsfFlightMode {
    pub armed: bool,
    pub failsafe: bool,
    pub mode: CrsfControlMode,
    pub autotune: AutotuneDisplay,
    /// True while the heading-hold modifier is engaged (only meaningful when `mode == Stabilized`).
    pub heading_hold: bool,
}

/// Lightweight GPS data for telemetry (avoids dependency on elle-rpc-icd).
#[cfg(feature = "gnss")]
#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct TelemetryGpsData {
    pub latitude: f32,
    pub longitude: f32,
    pub altitude_m: f32,
    pub num_satellites: u8,
    /// Ground speed in m/s, from UBX-NAV-PVT.
    pub ground_speed_ms: f32,
}

/// Dedicated GNSS signal for telemetry (populated by the GNSS task in elle-eagle).
#[cfg(feature = "gnss")]
pub static TELEMETRY_GNSS: Signal<CriticalSectionRawMutex, TelemetryGpsData> = Signal::new();

// ---------------------------------------------------------------------------
// Frame builders
// ---------------------------------------------------------------------------

/// Build a CRSF Attitude frame (type 0x1E).
/// Payload: pitch:i16 + roll:i16 + yaw:i16 (radians × 10000, big-endian).
fn build_attitude_frame(buf: &mut [u8; 10], pitch: f32, roll: f32, yaw: f32) {
    let p = (pitch * 10000.0) as i16;
    let r = (roll * 10000.0) as i16;
    let y = (yaw * 10000.0) as i16;

    buf[0] = CRSF_SYNC_BYTE;
    buf[1] = 8; // len = 6 payload + type + crc
    buf[2] = CRSF_FRAMETYPE_ATTITUDE;
    buf[3..5].copy_from_slice(&p.to_be_bytes());
    buf[5..7].copy_from_slice(&r.to_be_bytes());
    buf[7..9].copy_from_slice(&y.to_be_bytes());
    buf[9] = CRC8.checksum(&buf[2..9]);
}

/// Build a CRSF Flight Mode frame (type 0x21).
/// Payload: null-terminated ASCII string.
/// Returns total frame length (variable, max 14).
fn build_flight_mode_frame(buf: &mut [u8; 14], mode: &CrsfFlightMode) -> usize {
    let mode_str: &[u8] = if mode.failsafe {
        b"!FS!\0"
    } else if !mode.armed {
        b"WAIT\0"
    } else {
        match mode.autotune {
            AutotuneDisplay::Pitch => b"AT P\0",
            AutotuneDisplay::Roll => b"AT R\0",
            AutotuneDisplay::Done => b"DONE\0",
            AutotuneDisplay::Error => b"ERR!\0",
            AutotuneDisplay::Off => {
                if mode.heading_hold && mode.mode == CrsfControlMode::Stabilized {
                    b"HDG \0"
                } else {
                    match mode.mode {
                        CrsfControlMode::Manual => b"MANU\0",
                        CrsfControlMode::Stabilized => b"STAB\0",
                        CrsfControlMode::AltitudeHold => b"AHLD\0",
                    }
                }
            }
        }
    };

    let payload_len = mode_str.len(); // includes NUL
    let frame_len = payload_len + 4; // sync + len + type + payload + crc

    buf[0] = CRSF_SYNC_BYTE;
    buf[1] = (payload_len + 2) as u8; // type + payload + crc
    buf[2] = CRSF_FRAMETYPE_FLIGHT_MODE;
    buf[3..3 + payload_len].copy_from_slice(mode_str);
    let crc_end = 3 + payload_len;
    buf[crc_end] = CRC8.checksum(&buf[2..crc_end]);

    frame_len
}

/// Build a CRSF GPS frame (type 0x02).
/// Payload (15 bytes, all big-endian):
///   lat:i32 (deg × 1e7) + lon:i32 (deg × 1e7) + groundspeed:u16 (km/h × 10)
///   + heading:u16 (centideg, deg × 100) + altitude:u16 (m + 1000) + satellites:u8
///
/// `heading_rad`: magnetic heading in radians (-π..π) from the magnetometer —
/// deliberately not GNSS course over ground, which is meaningless at rest.
/// GNSS fields populated when `gnss` feature is enabled and data is available.
fn build_gps_frame(
    buf: &mut [u8; 19],
    heading_rad: f32,
    #[cfg(feature = "gnss")] gps: Option<&TelemetryGpsData>,
) {
    #[cfg(feature = "gnss")]
    let (lat_i, lon_i, alt, sats, speed) = if let Some(gps) = gps {
        (
            (gps.latitude as f64 * 1e7) as i32,
            (gps.longitude as f64 * 1e7) as i32,
            (gps.altitude_m + 1000.0).max(0.0) as u16,
            gps.num_satellites,
            // The frame carries km/h × 10; NAV-PVT reports m/s.
            (gps.ground_speed_ms * (3.6 * 10.0)).clamp(0.0, f32::from(u16::MAX)) as u16,
        )
    } else {
        (0i32, 0i32, 1000u16, 0u8, 0u16)
    };
    #[cfg(not(feature = "gnss"))]
    let (lat_i, lon_i, alt, sats, speed) = (0i32, 0i32, 1000u16, 0u8, 0u16);
    // Convert radians (-π..π) to signed centidegrees (-18000..18000).
    // EdgeTX reads this field as i16 and divides by 100 to get degrees.
    let cdeg = (heading_rad * (18000.0 / core::f32::consts::PI)) as i16;

    buf[0] = CRSF_SYNC_BYTE;
    buf[1] = 17; // 15 payload + type + crc
    buf[2] = CRSF_FRAMETYPE_GPS;
    buf[3..7].copy_from_slice(&lat_i.to_be_bytes());
    buf[7..11].copy_from_slice(&lon_i.to_be_bytes());
    buf[11..13].copy_from_slice(&speed.to_be_bytes());
    buf[13..15].copy_from_slice(&cdeg.to_be_bytes());
    buf[15..17].copy_from_slice(&alt.to_be_bytes());
    buf[17] = sats;
    buf[18] = CRC8.checksum(&buf[2..18]);
}

/// Build a CRSF Barometric Altitude frame (type 0x09).
/// Payload: altitude_dm:u16 (decimeters, offset +10000) + vario_cm:i16 (cm/s).
fn build_baro_frame(buf: &mut [u8; 8], altitude_m: f32, vario_ms: f32) {
    // CRSF barometric altitude = decimeters + 10000 offset
    let alt_dm = ((altitude_m * 10.0) as i32 + 10000).clamp(0, 65535) as u16;
    // Vario in cm/s, clamped to i16 range
    let vario = (vario_ms * 100.0).clamp(-32768.0, 32767.0) as i16;
    buf[0] = CRSF_SYNC_BYTE;
    buf[1] = 6; // type + 4 payload + crc
    buf[2] = CRSF_FRAMETYPE_BARO;
    buf[3..5].copy_from_slice(&alt_dm.to_be_bytes());
    buf[5..7].copy_from_slice(&vario.to_be_bytes());
    buf[7] = CRC8.checksum(&buf[2..7]);
}

/// Build a CRSF Battery Sensor frame (type 0x08).
/// Payload (8 bytes, big-endian):
///   voltage:u16 (100mV/LSB) + current:u16 (100mA/LSB) + capacity:u24 (mAh) + remaining:u8 (%)
///
/// Voltage/current from EDT (averaged across both engines since they share the same battery).
fn build_battery_frame(buf: &mut [u8; 12], voltage_mv: u32, current_ma: u32) {
    // EDT voltage is in mV, CRSF wants 100mV units
    let voltage_crsf = (voltage_mv / 100) as u16;
    // EDT current is in mA, CRSF wants 100mA units. Sum both engines.
    let current_crsf = (current_ma / 100) as u16;
    let capacity: u32 = 0; // no mAh tracking yet
    let remaining: u8 = 0; // no remaining % yet

    buf[0] = CRSF_SYNC_BYTE;
    buf[1] = 10; // 8 payload + type + crc
    buf[2] = CRSF_FRAMETYPE_BATTERY;
    buf[3..5].copy_from_slice(&voltage_crsf.to_be_bytes());
    buf[5..7].copy_from_slice(&current_crsf.to_be_bytes());
    // capacity is 24-bit big-endian
    buf[7] = (capacity >> 16) as u8;
    buf[8] = (capacity >> 8) as u8;
    buf[9] = capacity as u8;
    buf[10] = remaining;
    buf[11] = CRC8.checksum(&buf[2..11]);
}

// ---------------------------------------------------------------------------
// Telemetry task
// ---------------------------------------------------------------------------

/// CRSF telemetry transmit task.
///
/// Runs at 50 Hz, round-robining through 5 frame types each tick:
/// attitude → flight_mode → gps → baro → battery (~10 Hz each).
/// The GPS frame always carries magnetic heading from the MMC5616WA;
/// when the `gnss` feature is enabled, it also carries position data.
/// The battery frame carries EDT voltage/current from the ESCs (0 if EDT unsupported).
#[embassy_executor::task]
pub async fn crsf_telemetry_task(mut tx: UartTx<'static, Async>) {
    crate::elle_event!(
        info,
        crate::event::EVT_CRSF_TX_STARTED,
        "CRSF telemetry TX task starting (50 Hz)"
    );

    let mut last_attitude = AttitudeData::zero();
    let mut last_heading: f32 = 0.0; // magnetic heading from MMC5616WA (radians)
    let mut last_baro_alt: f32; // barometric altitude from BMP390
    let mut last_mode = CrsfFlightMode {
        armed: false,
        failsafe: false,
        mode: CrsfControlMode::Manual,
        autotune: AutotuneDisplay::Off,
        heading_hold: false,
    };

    #[cfg(feature = "gnss")]
    let mut last_gps: Option<TelemetryGpsData> = None;

    let mut ticker = Ticker::every(Duration::from_millis(20)); // 50 Hz
    let mut slot: u8 = 0;
    let num_slots: u8 = 5; // attitude, flight_mode, gps, baro, battery

    let mut frame_count: u32 = 0;
    let mut error_count: u32 = 0;

    loop {
        ticker.next().await;

        // Drain latest signal values (non-blocking)
        if let Some(att) = TELEMETRY_ATTITUDE.try_take() {
            last_attitude = att;
        }
        {
            let mag = MAG.read_cached();
            if mag.x != 0 || mag.y != 0 || mag.z != 0 {
                last_heading = libm::atan2f(mag.y as f32, mag.x as f32);
            }
        }
        if let Some(mode) = CRSF_FLIGHT_MODE.try_take() {
            last_mode = mode;
        }
        let last_vario_ms;
        {
            let baro = BARO.read_cached();
            last_baro_alt = baro.altitude_m;
            last_vario_ms = baro.vario_ms;
        }

        let mut tx_ok = true;

        match slot {
            0 => {
                let mut buf = [0u8; 10];
                build_attitude_frame(
                    &mut buf,
                    last_attitude.pitch,
                    last_attitude.roll,
                    last_attitude.yaw,
                );
                if let Err(e) = tx.write(&buf).await {
                    warn!("CRSF TX attitude error: {}", e);
                    tx_ok = false;
                }
            }
            1 => {
                let mut buf = [0u8; 14];
                let len = build_flight_mode_frame(&mut buf, &last_mode);
                if let Err(e) = tx.write(&buf[..len]).await {
                    warn!("CRSF TX flight_mode error: {}", e);
                    tx_ok = false;
                }
            }
            2 => {
                #[cfg(feature = "gnss")]
                if let Some(gps) = TELEMETRY_GNSS.try_take() {
                    last_gps = Some(gps);
                }
                let mut buf = [0u8; 19];
                build_gps_frame(
                    &mut buf,
                    last_heading,
                    #[cfg(feature = "gnss")]
                    last_gps.as_ref(),
                );
                if let Err(e) = tx.write(&buf).await {
                    warn!("CRSF TX gps error: {}", e);
                    tx_ok = false;
                }
            }
            3 => {
                let mut buf = [0u8; 8];
                build_baro_frame(&mut buf, last_baro_alt, last_vario_ms);
                if let Err(e) = tx.write(&buf).await {
                    warn!("CRSF TX baro error: {}", e);
                    tx_ok = false;
                }
            }
            4 => {
                let eng = crate::dshot::ENGINE_CACHE.lock(|c| c.get());
                // Use voltage from whichever engine has it (same battery),
                // sum current from both engines
                let voltage_mv = eng.left.voltage_mv.max(eng.right.voltage_mv);
                let current_ma = eng.left.current_ma + eng.right.current_ma;
                let mut buf = [0u8; 12];
                build_battery_frame(&mut buf, voltage_mv, current_ma);
                if let Err(e) = tx.write(&buf).await {
                    warn!("CRSF TX battery error: {}", e);
                    tx_ok = false;
                }
            }
            _ => {}
        }

        if !tx_ok {
            error_count += 1;
            if error_count <= 3 || error_count.is_multiple_of(500) {
                crate::event::send(3, crate::event::EVT_CRSF_TX_ERROR);
            }
        }

        slot = (slot + 1) % num_slots;
        frame_count += 1;

        if frame_count == 50 {
            crate::elle_event!(
                debug,
                crate::event::EVT_CRSF_TX_FIRST_SEC,
                "CRSF TX: first second complete"
            );
        }
        if frame_count.is_multiple_of(5000) {
            crate::elle_event!(
                debug,
                crate::event::EVT_CRSF_TX_STATS,
                "CRSF TX: {} frames sent",
                frame_count
            );
        }
    }
}
