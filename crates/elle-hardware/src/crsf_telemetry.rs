//! CRSF telemetry TX — sends attitude, flight mode, and GPS frames to the radio.
//!
//! Frame format: `[0xC8 | len | type | payload... | crc8_dvb_s2]`
//! where `len` = payload_len + 2 (type + CRC bytes), CRC covers type + payload.

use crate::imu::{AttitudeData, MAG_SIGNAL};
use crc::{Crc, CRC_8_DVB_S2};
use defmt::{info, warn};
use embassy_rp::uart::{Async, UartTx};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Ticker};

const CRSF_SYNC_BYTE: u8 = 0xC8;
const CRSF_FRAMETYPE_GPS: u8 = 0x02;
const CRSF_FRAMETYPE_ATTITUDE: u8 = 0x1E;
const CRSF_FRAMETYPE_FLIGHT_MODE: u8 = 0x21;

static CRC8: Crc<u8> = Crc::<u8>::new(&CRC_8_DVB_S2);

// ---------------------------------------------------------------------------
// Signals
// ---------------------------------------------------------------------------

/// Dedicated attitude signal for the telemetry task (separate from ATTITUDE_SIGNAL
/// which is consumed by try_take in the control loop).
pub static TELEMETRY_ATTITUDE: Signal<CriticalSectionRawMutex, AttitudeData> = Signal::new();

/// Flight-mode signal, written by the main control loop after each fc.update().
pub static CRSF_FLIGHT_MODE: Signal<CriticalSectionRawMutex, CrsfFlightMode> = Signal::new();

/// Snapshot of the current flight mode for CRSF telemetry.
#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct CrsfFlightMode {
    pub armed: bool,
    pub failsafe: bool,
    pub attitude_mode: bool,
}

/// Lightweight GPS data for telemetry (avoids dependency on elle-rpc-icd).
#[cfg(feature = "gnss")]
#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct TelemetryGpsData {
    pub latitude: f32,
    pub longitude: f32,
    pub altitude_m: f32,
    pub num_satellites: u8,
}

/// Dedicated GNSS signal for telemetry (populated by the GNSS task in elle-eagle).
#[cfg(feature = "gnss")]
pub static TELEMETRY_GNSS: Signal<CriticalSectionRawMutex, TelemetryGpsData> = Signal::new();

/// Log event from the telemetry task. The eagle crate forwards these to LogTopic.
/// Encoded as `(level, code)` — same convention as `log_channel::send()`.
pub static TELEMETRY_LOG: Signal<CriticalSectionRawMutex, (u8, u16)> = Signal::new();

// Log code constants for CRSF telemetry events
pub const LOG_CRSF_TX_STARTED: u16 = 20;
pub const LOG_CRSF_TX_FIRST_SEC: u16 = 21;
pub const LOG_CRSF_TX_ERROR: u16 = 22;
pub const LOG_CRSF_TX_STATS: u16 = 23;

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
    } else if mode.attitude_mode {
        b"STAB\0"
    } else {
        b"MANU\0"
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
/// `heading_rad`: magnetic heading in radians (-π..π) from magnetometer.
/// GNSS fields populated when `gnss` feature is enabled and data is available.
fn build_gps_frame(
    buf: &mut [u8; 19],
    heading_rad: f32,
    #[cfg(feature = "gnss")] gps: Option<&TelemetryGpsData>,
) {
    #[cfg(feature = "gnss")]
    let (lat_i, lon_i, alt, sats) = if let Some(gps) = gps {
        (
            (gps.latitude as f64 * 1e7) as i32,
            (gps.longitude as f64 * 1e7) as i32,
            (gps.altitude_m + 1000.0).max(0.0) as u16,
            gps.num_satellites,
        )
    } else {
        (0i32, 0i32, 1000u16, 0u8)
    };
    #[cfg(not(feature = "gnss"))]
    let (lat_i, lon_i, alt, sats) = (0i32, 0i32, 1000u16, 0u8);

    let speed: u16 = 0;
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

// ---------------------------------------------------------------------------
// Telemetry task
// ---------------------------------------------------------------------------

/// CRSF telemetry transmit task.
///
/// Runs at 50 Hz, round-robining through 3 frame types each tick:
/// attitude → flight_mode → gps → … (~17 Hz each).
/// The GPS frame always carries magnetic heading from the MMC5616WA;
/// when the `gnss` feature is enabled, it also carries position data.
#[embassy_executor::task]
pub async fn crsf_telemetry_task(mut tx: UartTx<'static, Async>) {
    info!("CRSF telemetry TX task starting (50 Hz)");
    TELEMETRY_LOG.signal((2, LOG_CRSF_TX_STARTED));

    let mut last_attitude = AttitudeData::zero();
    let mut last_heading: f32 = 0.0; // magnetic heading from MMC5616WA (radians)
    let mut last_mode = CrsfFlightMode {
        armed: false,
        failsafe: false,
        attitude_mode: false,
    };

    #[cfg(feature = "gnss")]
    let mut last_gps: Option<TelemetryGpsData> = None;

    let mut ticker = Ticker::every(Duration::from_millis(20)); // 50 Hz
    let mut slot: u8 = 0;
    let num_slots: u8 = 3; // attitude, flight_mode, gps

    let mut frame_count: u32 = 0;
    let mut error_count: u32 = 0;

    loop {
        ticker.next().await;

        // Drain latest signal values (non-blocking)
        if let Some(att) = TELEMETRY_ATTITUDE.try_take() {
            last_attitude = att;
        }
        if let Some(mag) = MAG_SIGNAL.try_take() {
            MAG_SIGNAL.signal(mag); // put back for RPC handler
            // Use f64 for atan2 to preserve full i32 count precision
            last_heading = libm::atan2(mag.y as f64, mag.x as f64) as f32;
        }
        if let Some(mode) = CRSF_FLIGHT_MODE.try_take() {
            last_mode = mode;
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
            _ => {}
        }

        if !tx_ok {
            error_count += 1;
            if error_count <= 3 || error_count % 500 == 0 {
                TELEMETRY_LOG.signal((3, LOG_CRSF_TX_ERROR));
            }
        }

        slot = (slot + 1) % num_slots;
        frame_count += 1;

        if frame_count == 50 {
            TELEMETRY_LOG.signal((1, LOG_CRSF_TX_FIRST_SEC));
        }
        if frame_count % 5000 == 0 {
            TELEMETRY_LOG.signal((1, LOG_CRSF_TX_STATS));
        }
    }
}
