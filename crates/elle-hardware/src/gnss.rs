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
use sam_m10q::ubx::cfg::{CfgVal, LAYER_RAM, NavDynamicModel, NavFixMode};
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
    /// True while NAV-PVT is arriving; false when the GGA fallback is driving.
    ///
    /// Polled rather than inferred from a boot event: the RTT attach takes long
    /// enough that one-shot boot messages are easily missed, and `LogTopic` is
    /// `BlockIfFull` with a 16-slot upstream channel, so early events can be
    /// dropped outright when no host is draining yet.
    pub pvt_active: bool,
    /// Link speed the boot sequence settled on, in baud.
    pub link_baud: u32,
    /// Solution interval the module actually accepted, in milliseconds.
    pub nav_rate_ms: u16,
    /// Bitmask of configuration keys the module acknowledged at boot.
    pub cfg_mask: u16,
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
///
/// Acknowledgements normally arrive in well under 100 ms. Kept short because
/// keys are applied one at a time, so this is paid per key.
const ACK_TIMEOUT: Duration = Duration::from_millis(400);
/// Give up on configuration after this many consecutive unanswered keys.
const MAX_SILENT_KEYS: u8 = 3;
/// Every bit of the configuration mask set — a fully applied configuration.
const CFG_MASK_ALL: u16 = (1 << 10) - 1;
/// Bit position of the solution-rate key within the configuration mask.
const CFG_BIT_RATE_MEAS: u16 = 1 << 1;
/// The module's own default solution interval, used when our rate key did not
/// take, so the reported rate is what the receiver is really doing.
const MODULE_DEFAULT_RATE_MS: u16 = 1_000;
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

/// Number of configuration groups applied at boot.
pub const CFG_GROUP_COUNT: usize = 10;

/// The settings we apply, grouped by what must be applied together.
///
/// Key IDs and value types come from `ublox`'s typed `CfgVal`, so they are not
/// transcribed by hand.
///
/// **Grouping is not cosmetic.** A CFG-VALSET aimed at the RAM layer is
/// validity-checked as a whole, and rejected outright if the resulting
/// configuration is inconsistent — not merely if a key is unknown. The dynamic
/// model and the fix mode are exactly such a pair: airborne models do not
/// support 2D fixes, so `DYNMODEL=AIR4` against the default `FIXMODE=AUTO` is
/// refused. Sent in either order as separate messages, one of them always
/// passes through an invalid intermediate state and NAKs. Sent together, the
/// configuration is valid at the moment it is checked.
///
/// 3D-only is what an aircraft wants regardless: a 2D fix invents an altitude.
///
/// `rate_ms` is the solution interval, which **must** match the link speed we
/// actually achieved. NAV-PVT is 100 bytes and GGA about 80; at 5 Hz that is
/// 900 B/s against the 960 B/s a 9600-baud link can carry, which the module
/// answers by dropping messages.
fn config_groups(rate_ms: u16) -> [CfgGroup; CFG_GROUP_COUNT] {
    [
        // Must travel together — see the note above.
        CfgGroup::pair(
            CfgVal::NavSpgDynModel(NavDynamicModel::AirborneWithLess4gAcceleration),
            CfgVal::NavSpgFixMode(NavFixMode::Only3D),
        ),
        CfgGroup::one(CfgVal::RateMeas(rate_ms)),
        CfgGroup::one(CfgVal::RateNav(1)),
        CfgGroup::one(CfgVal::MsgOutUbxNavPvtUart1(1)),
        // GGA is the fallback source; the rest is bandwidth we never read.
        CfgGroup::one(CfgVal::MsgOutNmeaIdGgaUart1(1)),
        CfgGroup::one(CfgVal::MsgOutNmeaIdGllUart1(0)),
        CfgGroup::one(CfgVal::MsgOutNmeaIdGsaUart1(0)),
        CfgGroup::one(CfgVal::MsgOutNmeaIdGsvUart1(0)),
        CfgGroup::one(CfgVal::MsgOutNmeaIdVtgUart1(0)),
        CfgGroup::one(CfgVal::MsgOutNmeaIdRmcUart1(0)),
    ]
}

/// One or two configuration keys that must be applied in a single message.
struct CfgGroup {
    items: [CfgVal; 2],
    len: usize,
}

impl CfgGroup {
    fn one(item: CfgVal) -> Self {
        Self {
            items: [item, item],
            len: 1,
        }
    }

    fn pair(a: CfgVal, b: CfgVal) -> Self {
        Self {
            items: [a, b],
            len: 2,
        }
    }

    fn as_slice(&self) -> &[CfgVal] {
        &self.items[..self.len]
    }
}

/// What the module said about one configuration message.
#[derive(Clone, Copy, PartialEq, Eq)]
enum CfgOutcome {
    /// Applied.
    Ack,
    /// Understood and refused — an unknown or unsupported key.
    Nak,
    /// Nothing came back. A different fault from a refusal: the link may be
    /// wrong rather than the key.
    NoAnswer,
}

/// Send a CFG-VALSET and wait for the module to acknowledge it.
async fn apply_valset(gnss: &mut Gnss<'_>, items: &[CfgVal]) -> CfgOutcome {
    if gnss.send_valset(LAYER_RAM, items).await.is_err() {
        return CfgOutcome::NoAnswer;
    }
    match with_timeout(
        ACK_TIMEOUT,
        gnss.wait_for_ack(ubx::class::CFG, ubx::cfg::VALSET),
    )
    .await
    {
        Ok(Ok(AckStatus::Ack)) => CfgOutcome::Ack,
        Ok(Ok(AckStatus::Nak)) => CfgOutcome::Nak,
        // Timed out, or the link failed.
        _ => CfgOutcome::NoAnswer,
    }
}

/// Apply each configuration group on its own, reporting which ones stuck.
///
/// A CFG-VALSET is rejected in full if the module dislikes any part of it, so
/// sending everything at once would cost us the dynamic model, the solution
/// rate and NAV-PVT together, with a NAK that names no culprit. One group per
/// message trades a slightly longer boot for a precise answer and keeps
/// whatever does work. Keys that must agree with each other share a group.
///
/// Returns a bitmask of accepted groups, one bit per [`config_groups`] entry.
async fn apply_config(gnss: &mut Gnss<'_>, rate_ms: u16) -> u16 {
    let groups = config_groups(rate_ms);
    let mut mask: u16 = 0;
    let mut silent = 0u8;

    for (i, group) in groups.iter().enumerate() {
        match apply_valset(gnss, group.as_slice()).await {
            CfgOutcome::Ack => {
                mask |= 1 << i;
                silent = 0;
            }
            CfgOutcome::Nak => {
                silent = 0;
                elle_event!(
                    warn,
                    event::EVT_GNSS_CFG_NAK,
                    "GNSS: key {} rejected (NAK)",
                    i
                );
            }
            CfgOutcome::NoAnswer => {
                silent += 1;
                elle_event!(
                    warn,
                    event::EVT_GNSS_CFG_TIMEOUT,
                    "GNSS: key {} unanswered",
                    i
                );
                // The module is not talking to us; stop rather than spend the
                // per-key timeout another seven times over.
                if silent >= MAX_SILENT_KEYS {
                    break;
                }
            }
        }
    }
    mask
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

    let requested_rate_ms = if fast { NAV_RATE_MS } else { SLOW_NAV_RATE_MS };
    let cfg_mask = {
        let (tx, rx) = uart.split_ref();
        let mut gnss = SamM10q::new(rx, tx);
        apply_config(&mut gnss, requested_rate_ms).await
    };
    // Report the rate the module actually runs at, not the one we asked for.
    let rate_ms = if cfg_mask & CFG_BIT_RATE_MEAS != 0 {
        requested_rate_ms
    } else {
        MODULE_DEFAULT_RATE_MS
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
    if cfg_mask != CFG_MASK_ALL {
        elle_event!(
            warn,
            event::EVT_GNSS_CFG_PARTIAL,
            "GNSS: configuration partially applied (mask {:#06x})",
            cfg_mask
        );
    }

    run(&mut uart, fast, rate_ms, cfg_mask).await;
}

/// Steady-state receive loop.
///
/// `fast` records which link speed the boot sequence settled on, so it can be
/// re-announced periodically. The boot events themselves fire about a second
/// after power-up and `LogTopic` keeps no backlog, so a host that attaches
/// later would otherwise never learn which mode the receiver is in.
async fn run(uart: &mut BufferedUart, fast: bool, nav_rate_ms: u16, cfg_mask: u16) {
    let (tx, rx) = uart.split_ref();
    let mut gnss = SamM10q::new(rx, tx);

    let mut data = GnssData {
        hdop: 99.9,
        link_baud: if fast { TARGET_BAUD } else { DEFAULT_BAUD },
        nav_rate_ms,
        cfg_mask,
        ..GnssData::default()
    };
    // Publish once up front so the link state is visible even before the first
    // fix — a receiver that never gets a fix should still report its baud rate
    // rather than leaving the host with nothing to read.
    GNSS_SIGNAL.signal(data);
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
                    data.pvt_active = true;
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
                data.pvt_active = false;
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
            // Re-announce the link mode for anyone who attached after boot.
            if fast {
                elle_event!(
                    debug,
                    event::EVT_GNSS_BAUD_SWITCHED,
                    "GNSS: {} baud",
                    TARGET_BAUD
                );
            } else {
                elle_event!(
                    warn,
                    event::EVT_GNSS_BAUD_FALLBACK,
                    "GNSS: {} baud",
                    DEFAULT_BAUD
                );
            }
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

/// Log a periodic summary every N fixes — 20 s at 5 Hz, 100 s at 1 Hz.
///
/// Kept short enough to be useful for diagnosis: this is how a host that
/// attached after boot learns the receiver is alive and which link mode it is
/// running in.
const GNSS_PERIODIC_LOG_EVERY: u32 = 100;

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
