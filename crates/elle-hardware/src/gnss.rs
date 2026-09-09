//! GNSS receiver task, shared by both airframes.
//!
//! Position comes from UBX-NAV-PVT, which carries velocity, ground speed and
//! accuracy estimates that NMEA GGA does not. GGA stays enabled as a fallback:
//! if configuration fails or PVT stops arriving, the aircraft keeps a position
//! fix rather than losing GNSS entirely.
//!
//! The module boots at 9600 baud, which at its default sentence rate leaves no
//! headroom above 1 Hz. The task reconfigures it to 115200 and a 5 Hz solution.
//! Configuration is applied to the **RAM layer only**, so every boot starts from
//! the module's known power-on defaults instead of inheriting unknown state.

use embassy_rp::uart::{BufferedUart, BufferedUartRx, BufferedUartTx};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer, with_timeout};

use sam_m10q::asynch::{AckStatus, SamM10q};
use sam_m10q::nmea::ParseResult;
use sam_m10q::types::Frame;
use sam_m10q::ubx::cfg::{CfgVal, LAYER_RAM, NavDynamicModel};
use sam_m10q::ubx::{self, nav};

use crate::elle_event;
use crate::event;

/// GNSS solution, independent of the RPC ICD types.
#[derive(Clone, Copy, Debug, Default)]
pub struct GnssData {
    pub latitude: f32,
    pub longitude: f32,
    pub altitude_m: f32,
    /// 0 = no fix, 1 = GPS, 2 = DGPS, 3 = other.
    pub fix_quality: u8,
    pub num_satellites: u8,
    /// Horizontal dilution of precision. Only meaningful on the GGA fallback
    /// path; NAV-PVT reports pDOP instead, and supplies `h_acc_m` in its place.
    pub hdop: f32,
    /// Velocity north, m/s. NAV-PVT only.
    pub vel_n_ms: f32,
    /// Velocity east, m/s. NAV-PVT only.
    pub vel_e_ms: f32,
    /// Velocity down, m/s. NAV-PVT only.
    pub vel_d_ms: f32,
    /// Ground speed over the ellipsoid, m/s. NAV-PVT only.
    pub ground_speed_ms: f32,
    /// Heading of motion (course over ground), degrees. NAV-PVT only.
    pub heading_motion_deg: f32,
    /// Horizontal accuracy estimate, m. The honest fix-quality gate — unlike
    /// `hdop`, which describes satellite geometry rather than error.
    pub h_acc_m: f32,
    /// Vertical accuracy estimate, m. NAV-PVT only.
    pub v_acc_m: f32,
    /// Speed accuracy estimate, m/s. NAV-PVT only.
    pub s_acc_ms: f32,
}

pub static GNSS_SIGNAL: Signal<CriticalSectionRawMutex, GnssData> = Signal::new();

/// Baud rate the module powers up at.
pub const DEFAULT_BAUD: u32 = 9600;
/// Baud rate we switch to. 9600 is 960 B/s; the default sentence set already
/// uses most of that at 1 Hz, leaving no room for a faster solution.
pub const TARGET_BAUD: u32 = 115_200;
/// Solution interval at 115200 baud — 200 ms is 5 Hz.
const NAV_RATE_MS: u16 = 200;
/// Solution interval to use if we are stuck at 9600 baud.
///
/// 1 Hz keeps NAV-PVT + GGA at roughly 180 B/s, comfortably inside the 960 B/s
/// a 9600 link carries. Asking for 5 Hz here saturates the port and the module
/// silently drops frames.
const SLOW_NAV_RATE_MS: u16 = 1_000;

/// How long a NAV-PVT stays authoritative before GGA takes over again.
const PVT_STALE: Duration = Duration::from_millis(3_000);
/// Budget for the module to answer a configuration message.
const ACK_TIMEOUT: Duration = Duration::from_millis(1_500);
/// How long to wait for traffic after a baud change before declaring it failed.
const BAUD_PROBE_TIMEOUT: Duration = Duration::from_millis(2_000);
/// Settling time after the cold-start reset.
const COLD_START_SETTLE: Duration = Duration::from_millis(500);
/// Upper bound on waiting for the UART to clock out a queued message.
const TX_DRAIN_TIMEOUT: Duration = Duration::from_millis(200);
/// How long the module may say nothing before we report the link as silent.
///
/// Comfortably longer than the slowest configured solution interval, so a
/// healthy 1 Hz link never trips it.
const SILENCE_TIMEOUT: Duration = Duration::from_millis(3_000);
/// After the first few, report continued silence only every Nth period.
const GNSS_SILENCE_LOG_INTERVAL: u32 = 20;

type Gnss<'a> = SamM10q<&'a mut BufferedUartRx, &'a mut BufferedUartTx>;

/// Every setting we apply, in one CFG-VALSET.
///
/// Key IDs and value types come from `ublox`'s typed `CfgVal`, so they are not
/// transcribed by hand. The dynamic model matters: the default is *Portable*,
/// whose motion assumptions fight a fixed-wing aircraft's velocity estimate.
///
/// `rate_ms` is the solution interval, which **must** match the link speed we
/// actually achieved. NAV-PVT is 100 bytes and GGA about 80; at 5 Hz that is
/// 900 B/s against the 960 B/s a 9600-baud link can carry, which the module
/// answers by dropping messages. See [`slow_rate_ms`].
fn config_items(rate_ms: u16) -> [CfgVal; 10] {
    [
        CfgVal::NavSpgDynModel(NavDynamicModel::AirborneWithLess4gAcceleration),
        CfgVal::RateMeas(rate_ms),
        CfgVal::RateNav(1),
        CfgVal::MsgOutUbxNavPvtUart1(1),
        // GGA is the fallback source; the rest is bandwidth we never read.
        CfgVal::MsgOutNmeaIdGgaUart1(1),
        CfgVal::MsgOutNmeaIdGllUart1(0),
        CfgVal::MsgOutNmeaIdGsaUart1(0),
        CfgVal::MsgOutNmeaIdGsvUart1(0),
        CfgVal::MsgOutNmeaIdVtgUart1(0),
        CfgVal::MsgOutNmeaIdRmcUart1(0),
    ]
}

/// Send a CFG-VALSET and wait for the module to acknowledge it.
async fn apply_valset(gnss: &mut Gnss<'_>, items: &[CfgVal]) -> bool {
    if gnss.send_valset(LAYER_RAM, items).await.is_err() {
        return false;
    }
    match with_timeout(
        ACK_TIMEOUT,
        gnss.wait_for_ack(ubx::class::CFG, ubx::cfg::VALSET),
    )
    .await
    {
        Ok(Ok(AckStatus::Ack)) => true,
        Ok(Ok(AckStatus::Nak)) => {
            elle_event!(
                warn,
                event::EVT_GNSS_CFG_NAK,
                "GNSS: module rejected configuration (NAK)"
            );
            false
        }
        // Timed out, or the link failed.
        _ => {
            elle_event!(
                warn,
                event::EVT_GNSS_CFG_NAK,
                "GNSS: no answer to configuration"
            );
            false
        }
    }
}

/// Wait until the UART has physically transmitted everything queued.
///
/// `embedded_io_async::Write::flush` returns once the software ring buffer is
/// empty; `busy()` is what covers the hardware FIFO and shift register.
async fn drain_tx(uart: &mut BufferedUart) {
    let deadline = Instant::now() + TX_DRAIN_TIMEOUT;
    while uart.busy() && Instant::now() < deadline {
        Timer::after(Duration::from_millis(1)).await;
    }
    // A little margin for the final stop bit to clear the shift register.
    Timer::after(Duration::from_millis(5)).await;
}

/// Wait for any decodable frame, to prove the link works at the current baud.
async fn link_alive(gnss: &mut Gnss<'_>) -> bool {
    matches!(
        with_timeout(BAUD_PROBE_TIMEOUT, gnss.next_frame()).await,
        Ok(Ok(_))
    )
}

#[embassy_executor::task]
pub async fn gnss_task(mut uart: BufferedUart) {
    // Cold start, so acquisition does not depend on stale almanac state.
    {
        let (tx, rx) = uart.split_ref();
        let mut gnss = SamM10q::new(rx, tx);
        // UBX-CFG-RST: navBbrMask = 0x0000, resetMode = 0x02 (GNSS-only
        // software reset). 0x0000 is a *hot* start — ephemeris and almanac are
        // kept, which is what we want: it gives the fastest time to first fix.
        // (A cold start would be 0xFFFF; the previous comment here said "cold"
        // and was simply wrong.)
        if gnss
            .send_ubx(ubx::class::CFG, ubx::cfg::RST, &[0x00, 0x00, 0x02, 0x00])
            .await
            .is_err()
        {
            elle_event!(
                warn,
                event::EVT_GNSS_UART_ERROR,
                "GNSS: cold start send failed"
            );
        }
    }
    Timer::after(COLD_START_SETTLE).await;

    // Change baud FIRST, then configure at whatever rate we actually achieved.
    // The reverse order is a trap: a solution rate that only fits at 115200,
    // applied before a baud switch that then fails, leaves the module pushing
    // more bytes than a 9600 link can carry and silently dropping frames.
    let sent = {
        let (tx, rx) = uart.split_ref();
        let mut gnss = SamM10q::new(rx, tx);
        gnss.send_valset(LAYER_RAM, &[CfgVal::Uart1Baudrate(TARGET_BAUD)])
            .await
            .is_ok()
    };

    let mut fast = false;
    if sent {
        // `flush()` only drains the software ring buffer; the hardware FIFO and
        // shift register can still hold ~32 bytes, which is 33 ms at 9600.
        // Changing the divisor before they are on the wire corrupts the tail of
        // the message we just sent, and the module never switches.
        drain_tx(&mut uart).await;
        uart.set_baudrate(TARGET_BAUD);

        let (tx, rx) = uart.split_ref();
        let mut gnss = SamM10q::new(rx, tx);
        fast = link_alive(&mut gnss).await;
    }

    if !fast {
        // The module never answered at the new rate, so it is still at its
        // power-on baud. Go back and stay there.
        uart.set_baudrate(DEFAULT_BAUD);
    }

    let rate_ms = if fast { NAV_RATE_MS } else { SLOW_NAV_RATE_MS };
    let configured = {
        let (tx, rx) = uart.split_ref();
        let mut gnss = SamM10q::new(rx, tx);
        apply_valset(&mut gnss, &config_items(rate_ms)).await
    };

    if fast {
        elle_event!(
            info,
            event::EVT_GNSS_BAUD_SWITCHED,
            "GNSS: {} baud, {} ms solution",
            TARGET_BAUD,
            rate_ms
        );
    } else {
        elle_event!(
            warn,
            event::EVT_GNSS_BAUD_FALLBACK,
            "GNSS: {} baud, {} ms solution (baud switch failed)",
            DEFAULT_BAUD,
            rate_ms
        );
    }
    if !configured {
        // Nothing was applied, so the module is at its factory defaults: 1 Hz
        // NMEA, no NAV-PVT. The GGA fallback path carries us.
        elle_event!(
            warn,
            event::EVT_GNSS_CFG_NAK,
            "GNSS: running on module defaults, NMEA only"
        );
    }

    run(&mut uart).await;
}

/// Steady-state receive loop.
async fn run(uart: &mut BufferedUart) {
    let (tx, rx) = uart.split_ref();
    let mut gnss = SamM10q::new(rx, tx);

    let mut data = GnssData {
        hdop: 99.9,
        ..GnssData::default()
    };
    let mut last_pvt: Option<Instant> = None;
    let mut announced_pvt = false;
    let mut announced_fallback = false;
    let mut fix_count: u32 = 0;
    let mut uart_errors: u32 = 0;
    let mut silent_periods: u32 = 0;

    loop {
        // A silent link and a link with no satellite fix look identical from
        // the outside — both just never report a position. Time out the read so
        // "the module is not talking to us at all" is a distinct, visible state.
        let frame = match with_timeout(SILENCE_TIMEOUT, gnss.next_frame()).await {
            Err(_) => {
                silent_periods += 1;
                if silent_periods <= elle_config::GNSS_ERROR_LOG_INITIAL
                    || silent_periods.is_multiple_of(GNSS_SILENCE_LOG_INTERVAL)
                {
                    elle_event!(
                        warn,
                        event::EVT_GNSS_NO_DATA,
                        "GNSS: no data for {} ms (x{})",
                        SILENCE_TIMEOUT.as_millis(),
                        silent_periods
                    );
                }
                continue;
            }
            Ok(Ok(frame)) => frame,
            Ok(Err(_)) => {
                uart_errors += 1;
                if uart_errors <= elle_config::GNSS_ERROR_LOG_INITIAL
                    || uart_errors.is_multiple_of(elle_config::GNSS_ERROR_LOG_INTERVAL)
                {
                    elle_event!(
                        warn,
                        event::EVT_GNSS_UART_ERROR,
                        "GNSS read error (total={})",
                        uart_errors
                    );
                }
                Timer::after(Duration::from_millis(10)).await;
                continue;
            }
        };
        uart_errors = 0;
        silent_periods = 0;

        let now = Instant::now();
        let pvt_fresh = last_pvt.is_some_and(|t| now.duration_since(t) < PVT_STALE);

        let updated = match frame {
            Frame::Ubx(u) => match nav::parse_pvt(u.class, u.id, u.payload) {
                Some(pvt) => {
                    apply_pvt(&mut data, &pvt);
                    last_pvt = Some(now);
                    if !announced_pvt {
                        announced_pvt = true;
                        announced_fallback = false;
                        elle_event!(
                            info,
                            event::EVT_GNSS_PVT_ACQUIRED,
                            "GNSS: NAV-PVT stream acquired"
                        );
                    }
                    true
                }
                None => false,
            },
            // GGA only steers while PVT is absent or stale, so the two sources
            // never fight over the same fields.
            Frame::Nmea(n) if !pvt_fresh => {
                if announced_pvt && !announced_fallback {
                    announced_fallback = true;
                    announced_pvt = false;
                    elle_event!(
                        warn,
                        event::EVT_GNSS_NMEA_FALLBACK,
                        "GNSS: NAV-PVT stale, falling back to NMEA"
                    );
                }
                match n.parse() {
                    Some(ParseResult::GGA(gga)) => {
                        apply_gga(&mut data, &gga);
                        true
                    }
                    _ => false,
                }
            }
            Frame::Nmea(_) => false,
        };

        if !updated {
            continue;
        }

        fix_count = fix_count.wrapping_add(1);
        if fix_count == 1 {
            elle_event!(
                info,
                event::EVT_GNSS_FIRST_FIX,
                "GNSS: first fix (quality={}, sats={})",
                data.fix_quality,
                data.num_satellites
            );
        } else if fix_count.is_multiple_of(GNSS_PERIODIC_LOG_EVERY) {
            elle_event!(
                debug,
                event::EVT_GNSS_PERIODIC,
                "GNSS: {} fixes (quality={}, sats={})",
                fix_count,
                data.fix_quality,
                data.num_satellites
            );
        }

        GNSS_SIGNAL.signal(data);
        crate::crsf::TELEMETRY_GNSS.signal(crate::crsf::TelemetryGpsData {
            latitude: data.latitude,
            longitude: data.longitude,
            altitude_m: data.altitude_m,
            num_satellites: data.num_satellites,
            ground_speed_ms: data.ground_speed_ms,
        });
    }
}

/// Log a periodic summary roughly once a minute at 5 Hz.
const GNSS_PERIODIC_LOG_EVERY: u32 = 300;

fn apply_pvt(data: &mut GnssData, pvt: &nav::NavPvtRef<'_>) {
    // NAV-PVT fixType: 0 none, 1 dead reckoning, 2 2-D, 3 3-D, 4 GNSS+DR, 5 time.
    let fix_type = pvt.fix_type() as u8;
    data.fix_quality = match fix_type {
        2 | 3 => 1,
        4 => 2,
        _ => 0,
    };
    data.num_satellites = pvt.num_satellites();

    // Hold the last good position through a dropout rather than snapping to
    // zero, matching what the GGA path has always done.
    if data.fix_quality > 0 {
        data.latitude = pvt.latitude() as f32;
        data.longitude = pvt.longitude() as f32;
        data.altitude_m = pvt.height_msl() as f32;
    }

    data.vel_n_ms = pvt.vel_north() as f32;
    data.vel_e_ms = pvt.vel_east() as f32;
    data.vel_d_ms = pvt.vel_down() as f32;
    data.ground_speed_ms = pvt.ground_speed_2d() as f32;
    data.heading_motion_deg = pvt.heading_motion() as f32;
    data.h_acc_m = pvt.horizontal_accuracy() as f32;
    data.v_acc_m = pvt.vertical_accuracy() as f32;
    data.s_acc_ms = pvt.speed_accuracy() as f32;
}

fn apply_gga(data: &mut GnssData, gga: &sam_m10q::nmea::sentences::GgaData) {
    use sam_m10q::nmea::sentences::FixType;

    data.fix_quality = match gga.fix_type {
        Some(FixType::Invalid) | None => 0,
        Some(FixType::Gps) => 1,
        Some(FixType::DGps) => 2,
        Some(_) => 3,
    };
    data.num_satellites = gga.fix_satellites.unwrap_or(0) as u8;
    data.hdop = gga.hdop.unwrap_or(99.9);

    if data.fix_quality > 0 {
        if let Some(lat) = gga.latitude {
            data.latitude = lat as f32;
        }
        if let Some(lon) = gga.longitude {
            data.longitude = lon as f32;
        }
        if let Some(alt) = gga.altitude {
            data.altitude_m = alt;
        }
    }

    // GGA carries none of the velocity or accuracy fields; leave whatever the
    // last NAV-PVT left behind rather than publishing invented zeroes.
}
