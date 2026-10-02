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
use embassy_time::{Duration, Instant, Timer, with_timeout};

use sam_m10q::asynch::{AckStatus, SamM10q};
use sam_m10q::nmea::{ParseResult, sentences::GnssType};
use sam_m10q::types::Frame;
use sam_m10q::ubx::cfg::{CfgVal, LAYER_RAM, NavDynamicModel, NavFixMode};
use sam_m10q::ubx::{self, nav};

use crate::elle_event;
use crate::event;
use crate::signal_cache::SignalCache;

/// GNSS solution, independent of the RPC ICD types.
///
/// Fields that the source of the current fix does not provide are NaN, not
/// the last value another source left behind: on the GGA fallback that is
/// every velocity and accuracy field.
#[derive(Clone, Copy, Debug)]
pub struct GnssData {
    /// When the frame that last updated the fix was received, µs since boot;
    /// 0 before the first one. Consumers use it to age the fix and to tell a
    /// new solution from the same one read twice.
    pub sample_us: u64,
    /// Latitude, degrees × 10⁷ (the NAV-PVT encoding; `f32` degrees would
    /// round to ~0.5 m).
    pub lat_e7: i32,
    /// Longitude, degrees × 10⁷.
    pub lon_e7: i32,
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
    /// Satellites *in view*, summed across constellations, from NMEA GSV.
    ///
    /// Distinct from `num_satellites`, which counts satellites *used in the
    /// fix* and therefore reads zero throughout acquisition. Zero unless the
    /// `gnss-gsv` feature is on and the fast link was achieved.
    pub sats_in_view: u8,
}

impl GnssData {
    /// No fix yet: position zero, everything the receiver has not reported NaN.
    pub const EMPTY: Self = Self {
        sample_us: 0,
        lat_e7: 0,
        lon_e7: 0,
        altitude_m: 0.0,
        fix_quality: 0,
        num_satellites: 0,
        hdop: 99.9,
        vel_n_ms: f32::NAN,
        vel_e_ms: f32::NAN,
        vel_d_ms: f32::NAN,
        ground_speed_ms: f32::NAN,
        heading_motion_deg: f32::NAN,
        h_acc_m: f32::NAN,
        v_acc_m: f32::NAN,
        s_acc_ms: f32::NAN,
        pvt_active: false,
        link_baud: 0,
        nav_rate_ms: 0,
        cfg_mask: 0,
        sats_in_view: 0,
    };

    /// Latitude in degrees.
    #[must_use]
    pub fn latitude_deg(&self) -> f64 {
        f64::from(self.lat_e7) * 1e-7
    }

    /// Longitude in degrees.
    #[must_use]
    pub fn longitude_deg(&self) -> f64 {
        f64::from(self.lon_e7) * 1e-7
    }
}

/// Satellites in view, tracked per constellation and summed.
///
/// Each constellation sends its own GSV set with its own count, so a single
/// sentence never carries the total.
#[derive(Default)]
struct SatsInView {
    /// One count per [`GnssType`] we might hear from, indexed by `gnss_index`.
    per_gnss: [u8; GNSS_KINDS],
}

/// Number of constellations [`SatsInView`] tracks separately.
const GNSS_KINDS: usize = 6;

/// Stable index for a constellation, so counts can be kept side by side.
const fn gnss_index(kind: GnssType) -> usize {
    match kind {
        GnssType::Gps => 0,
        GnssType::Galileo => 1,
        GnssType::Glonass => 2,
        GnssType::Beidou => 3,
        GnssType::Qzss => 4,
        GnssType::NavIC => 5,
    }
}

impl SatsInView {
    /// Record one constellation's count and return the running total.
    fn update(&mut self, kind: GnssType, in_view: u16) -> u8 {
        self.per_gnss[gnss_index(kind)] = in_view.min(u16::from(u8::MAX)) as u8;
        self.per_gnss
            .iter()
            .fold(0u8, |acc, n| acc.saturating_add(*n))
    }
}

/// The latest solution (and link state), for the navigator, ULog, RPC and CRSF.
pub static GNSS: SignalCache<GnssData> = SignalCache::new(GnssData::EMPTY);

/// Baud rate the module powers up at.
pub const DEFAULT_BAUD: u32 = 9600;
/// Baud rate we switch to. 9600 is 960 B/s; the default sentence set already
/// uses most of that at 1 Hz, leaving no room for a faster solution.
pub(crate) const TARGET_BAUD: u32 = 115_200;
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
///
/// Derived from [`CFG_GROUP_COUNT`] rather than written out, so adding a
/// configuration group cannot leave "cfg ok" permanently unreachable.
const CFG_MASK_ALL: u16 = (1 << CFG_GROUP_COUNT) - 1;
/// Set in the configuration mask when configuration was given up on partway
/// through, rather than run to the end.
///
/// Without this a group that was never attempted is indistinguishable from one
/// the module refused, and the host names it as rejected — pointing diagnosis
/// at, say, `MSGOUT-GGA` when the real fault was the link going silent three
/// keys earlier. Lives in the top bit so it can never collide with a group.
const CFG_ABANDONED: u16 = 1 << 15;

// The abandoned marker must stay clear of the per-group bits.
const _: () = assert!(CFG_GROUP_COUNT < 15);
/// Bit position of the solution-rate key within the configuration mask: the
/// `RateMeas` group must stay at index 1 of [`config_groups`].
const CFG_BIT_RATE_MEAS: u16 = 1 << 1;
/// The module's own default solution interval, used when our rate key did not
/// take, so the reported rate is what the receiver is really doing.
const MODULE_DEFAULT_RATE_MS: u16 = 1_000;
/// How long to wait for traffic after a baud change before declaring it failed.
const BAUD_PROBE_TIMEOUT: Duration = Duration::from_millis(2_000);
/// Settling time after the cold-start reset.
const RESET_SETTLE: Duration = Duration::from_millis(500);
/// Upper bound on waiting for the UART to clock out a queued message.
const TX_DRAIN_TIMEOUT: Duration = Duration::from_millis(200);
/// How long the module may say nothing before we report the link as silent.
///
/// Comfortably longer than the slowest configured solution interval, so a
/// healthy 1 Hz link never trips it.
const SILENCE_TIMEOUT: Duration = Duration::from_millis(3_000);
/// After the first few, report continued silence only every Nth period.
const GNSS_SILENCE_LOG_INTERVAL: u32 = 20;

type Gnss<'a> = SamM10q<BufferedUartRx<'a>, BufferedUartTx<'a>>;

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
/// `gsv` asks for NMEA satellites-in-view. It is the only way to observe
/// satellites being *tracked* — every fix message reports satellites *used*,
/// which stays at zero right up until a fix appears, so acquisition is
/// otherwise invisible. It is expensive: roughly six sentences per epoch
/// across GPS/SBAS, Galileo and QZSS, about 480 bytes, or 2.4 kB/s at 5 Hz.
/// That is 29% of a 115200 link and would be 69% of a 9600 one, so the caller
/// only asks for it on the fast link.
///
/// `rate_ms` is the solution interval, which **must** match the link speed we
/// actually achieved. NAV-PVT is 100 bytes and GGA about 80; at 5 Hz that is
/// 900 B/s against the 960 B/s a 9600-baud link can carry, which the module
/// answers by dropping messages.
fn config_groups(rate_ms: u16, gsv: bool) -> [CfgGroup; CFG_GROUP_COUNT] {
    [
        // Must travel together — see the note above.
        CfgGroup::pair(
            CfgVal::NavSpgDynModel(NavDynamicModel::AirborneWithLess4gAcceleration),
            CfgVal::NavSpgFixMode(NavFixMode::Only3D),
        ),
        // Index 1: `CFG_BIT_RATE_MEAS` reports whether this group took.
        CfgGroup::one(CfgVal::RateMeas(rate_ms)),
        CfgGroup::one(CfgVal::RateNav(1)),
        CfgGroup::one(CfgVal::MsgOutUbxNavPvtUart1(1)),
        // GGA is the fallback source; the rest is bandwidth we never read.
        CfgGroup::one(CfgVal::MsgOutNmeaIdGgaUart1(1)),
        CfgGroup::one(CfgVal::MsgOutNmeaIdGllUart1(0)),
        CfgGroup::one(CfgVal::MsgOutNmeaIdGsaUart1(0)),
        CfgGroup::one(CfgVal::MsgOutNmeaIdGsvUart1(u8::from(gsv))),
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
    const fn one(item: CfgVal) -> Self {
        Self {
            items: [item, item],
            len: 1,
        }
    }

    const fn pair(a: CfgVal, b: CfgVal) -> Self {
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
    /// Nothing came back within [`ACK_TIMEOUT`]. A different fault from a
    /// refusal: the link may be wrong rather than the key. `uart_errors` counts
    /// the read errors skipped while waiting, which tells a noisy link from a
    /// silent one.
    NoAnswer { uart_errors: u16 },
}

/// Send a CFG-VALSET and wait for the module to acknowledge it.
///
/// UART read errors while waiting are skipped, as in [`link_alive`]: right
/// after the baud switch the UART still holds framing / overrun flags latched
/// from the old rate, and giving up on the first one abandoned every key within
/// a few milliseconds of a power-on boot. Only the timeout ends the wait.
async fn apply_valset(gnss: &mut Gnss<'_>, items: &[CfgVal]) -> CfgOutcome {
    if gnss.send_valset(LAYER_RAM, items).await.is_err() {
        return CfgOutcome::NoAnswer { uart_errors: 0 };
    }
    let mut uart_errors: u16 = 0;
    let wait = async {
        loop {
            match gnss.wait_for_ack(ubx::class::CFG, ubx::cfg::VALSET).await {
                Ok(AckStatus::Ack) => return CfgOutcome::Ack,
                Ok(AckStatus::Nak) => return CfgOutcome::Nak,
                Err(_) => {
                    uart_errors = uart_errors.saturating_add(1);
                    // An error is normally reported once and cleared; the pause
                    // keeps a flag that does not clear from spinning Core 0
                    // for the whole timeout.
                    Timer::after(Duration::from_millis(1)).await;
                }
            }
        }
    };
    match with_timeout(ACK_TIMEOUT, wait).await {
        Ok(outcome) => outcome,
        Err(_) => CfgOutcome::NoAnswer { uart_errors },
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
async fn apply_config(gnss: &mut Gnss<'_>, rate_ms: u16, gsv: bool) -> u16 {
    let groups = config_groups(rate_ms, gsv);
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
            CfgOutcome::NoAnswer { uart_errors } => {
                silent += 1;
                elle_event!(
                    warn,
                    event::EVT_GNSS_CFG_TIMEOUT,
                    "GNSS: key {} unanswered ({} UART errors)",
                    i,
                    uart_errors
                );
                // The module is not talking to us; stop rather than spend the
                // per-key timeout another seven times over. Flag it, so the
                // groups we never got to are not reported as refusals.
                if silent >= MAX_SILENT_KEYS {
                    mask |= CFG_ABANDONED;
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
async fn drain_tx(uart: &mut BufferedUart<'_>) {
    let deadline = Instant::now() + TX_DRAIN_TIMEOUT;
    while uart.busy() && Instant::now() < deadline {
        Timer::after(Duration::from_millis(1)).await;
    }
    // A little margin for the final stop bit to clear the shift register.
    Timer::after(Duration::from_millis(5)).await;
}

/// Wait for any decodable frame, to prove the link works at the current baud.
///
/// UART errors inside the window are skipped, not taken as a dead link. The RX
/// ring still holds whatever arrived at the previous baud, and the UART latches
/// its framing / break / overrun flags until that data has been read. After an
/// MCU-only reset (`cargo run`) the module is still at 115200 from the last
/// session, so the 9600 settle before the switch fills the ring with garbage and
/// the first read at 115200 returns that stale error; giving up there fell back
/// to 9600 against a module sending at 115200, and GNSS stayed dead for the
/// session. UBX and NMEA checksums keep misread bytes from passing as a frame.
async fn link_alive(gnss: &mut Gnss<'_>) -> bool {
    let probe = async {
        // Each error is reported once and then cleared, so this waits for new
        // data between attempts rather than spinning.
        while gnss.next_frame().await.is_err() {}
    };
    with_timeout(BAUD_PROBE_TIMEOUT, probe).await.is_ok()
}

#[embassy_executor::task]
pub async fn gnss_task(mut uart: BufferedUart<'static>) {
    // GNSS-only hot reset, so the receiver starts from its power-on defaults.
    {
        let (tx, rx) = uart.split_ref();
        let mut gnss = SamM10q::new(rx, tx);
        // UBX-CFG-RST: navBbrMask = 0x0000, resetMode = 0x02 (GNSS-only
        // software reset). 0x0000 is a *hot* start — ephemeris and almanac are
        // kept, which is what we want: it gives the fastest time to first fix.
        // (A cold start would be 0xFFFF.)
        if gnss
            .send_ubx(ubx::class::CFG, ubx::cfg::RST, &[0x00, 0x00, 0x02, 0x00])
            .await
            .is_err()
        {
            elle_event!(warn, event::EVT_GNSS_UART_ERROR, "GNSS: reset send failed");
        }
    }
    Timer::after(RESET_SETTLE).await;

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
    // Satellites-in-view is a bench aid, and only affordable on the fast link:
    // on the 9600 fallback it would take roughly 69% of the port.
    let want_gsv = cfg!(feature = "gnss-gsv") && fast;
    let cfg_mask = {
        let (tx, rx) = uart.split_ref();
        let mut gnss = SamM10q::new(rx, tx);
        apply_config(&mut gnss, requested_rate_ms, want_gsv).await
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
/// Push a sample to every consumer: the RPC/ULog signal and CRSF telemetry.
///
/// The two must not drift apart — a stale CRSF frame is a wrong home point on
/// the radio, which is worse than a stale TUI panel.
fn publish(data: &GnssData) {
    GNSS.publish(*data);
    crate::crsf::TELEMETRY_GNSS.signal(crate::crsf::TelemetryGpsData {
        lat_e7: data.lat_e7,
        lon_e7: data.lon_e7,
        altitude_m: data.altitude_m,
        num_satellites: data.num_satellites,
        ground_speed_ms: data.ground_speed_ms,
    });
}

/// Mark the fix as no longer trustworthy, keeping the last known position.
///
/// `GnssData` carries no timestamp, so a consumer cannot tell a live fix from
/// one frozen minutes ago. Without this, a receiver that stops talking leaves
/// `handle_get_gnss` returning a confident fix forever and CRSF still feeding
/// the radio a position. Holding the coordinates matches what the GGA path has
/// always done; zeroing the satellite count and quality is what says "do not
/// trust this any more" to every consumer, including the radio.
fn mark_fix_lost(data: &mut GnssData) {
    data.fix_quality = 0;
    data.num_satellites = 0;
    data.pvt_active = false;
}

/// `fast` records which link speed the boot sequence settled on, so it can be
/// re-announced periodically. The boot events themselves fire about a second
/// after power-up and `LogTopic` keeps no backlog, so a host that attaches
/// later would otherwise never learn which mode the receiver is in.
async fn run(uart: &mut BufferedUart<'_>, fast: bool, nav_rate_ms: u16, cfg_mask: u16) {
    let (tx, rx) = uart.split_ref();
    let mut gnss = SamM10q::new(rx, tx);

    let mut data = GnssData {
        link_baud: if fast { TARGET_BAUD } else { DEFAULT_BAUD },
        nav_rate_ms,
        cfg_mask,
        ..GnssData::EMPTY
    };
    // Publish once up front so the link state is visible even before the first
    // fix — a receiver that never gets a fix should still report its baud rate
    // rather than leaving the host with nothing to read.
    GNSS.publish(data);
    let mut last_pvt: Option<Instant> = None;
    // Last time *either* source updated the fix. `last_pvt` alone decides which
    // source steers; this decides whether the fix has gone stale.
    let mut last_fix_update: Option<Instant> = None;
    let mut in_view = SatsInView::default();
    let mut announced_pvt = false;
    let mut announced_fallback = false;
    let mut fix_count: u32 = 0;
    let mut uart_errors: u32 = 0;
    let mut silent_periods: u32 = 0;
    // Whether consumers have already been told the fix went away, so the
    // transition is published once rather than every timeout.
    let mut fix_lost = false;

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
                if !fix_lost {
                    fix_lost = true;
                    mark_fix_lost(&mut data);
                    publish(&data);
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
            // Parsed once: `parse()` runs the whole NMEA parser, so the
            // sentence type is matched here rather than in a guard.
            Frame::Nmea(n) => match n.parse() {
                // Satellites in view is diagnostic and never competes with the
                // fix, so it is read whichever source is driving position.
                // Gating it on the fallback path would discard it exactly when
                // it is enabled — on a healthy PVT link — costing bandwidth
                // for nothing.
                Some(ParseResult::GSV(gsv)) => {
                    data.sats_in_view = in_view.update(gsv.gnss_type, gsv.sats_in_view);
                    false
                }
                // GGA only steers while PVT is absent or stale, so the two
                // sources never fight over the same fields.
                Some(ParseResult::GGA(gga)) if !pvt_fresh => {
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
                    apply_gga(&mut data, &gga);
                    true
                }
                _ => false,
            },
        };

        if updated {
            last_fix_update = Some(now);
            data.sample_us = now.as_micros();
        }
        let fix_fresh = last_fix_update.is_some_and(|t| now.duration_since(t) < PVT_STALE);

        if !updated {
            // The link is alive — GSV or some other sentence is still arriving
            // — but nothing has driven the fix for `PVT_STALE`. That happens
            // whenever NAV-PVT stops and GGA is not enabled to take over, which
            // is exactly the state `apply_config` is allowed to leave us in.
            // The silence timeout never fires here, so without this the last
            // good sample would stand indefinitely.
            //
            // Keyed on the last update from *either* source, not on PVT: during
            // GGA fallback PVT is stale by definition, so checking it here wiped
            // every GGA fix on the very next non-GGA sentence (GSA/GSV/RMC follow
            // GGA within milliseconds on the default NMEA set).
            if data.fix_quality > 0 && !fix_fresh && !fix_lost {
                fix_lost = true;
                mark_fix_lost(&mut data);
                publish(&data);
            }
            continue;
        }
        fix_lost = false;

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

        publish(&data);
    }
}

/// Log a periodic summary every N fixes — 20 s at 5 Hz, 100 s at 1 Hz.
///
/// Kept short enough to be useful for diagnosis: this is how a host that
/// attached after boot learns the receiver is alive and which link mode it is
/// running in.
const GNSS_PERIODIC_LOG_EVERY: u32 = 100;

fn apply_pvt(data: &mut GnssData, pvt: &nav::NavPvtRef<'_>) {
    // Honours `gnssFixOK` as well as `fixType` — see `nav::fix_quality`. An
    // acquiring M10 will claim `fixType = 3` before its solution passes the
    // DOP/accuracy masks, and that position must not reach the radio.
    data.fix_quality = nav::fix_quality(pvt);
    data.num_satellites = pvt.num_satellites();
    // `hdop` is deliberately left at its "unavailable" seed: NAV-PVT carries no
    // DOP, and hAcc/vAcc are the real fix-quality gate on this path.

    // Hold the last good position through a dropout rather than snapping to
    // zero, matching what the GGA path has always done.
    if data.fix_quality > 0 {
        data.lat_e7 = pvt.latitude_raw();
        data.lon_e7 = pvt.longitude_raw();
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
            data.lat_e7 = libm::round(lat * 1e7) as i32;
        }
        if let Some(lon) = gga.longitude {
            data.lon_e7 = libm::round(lon * 1e7) as i32;
        }
        if let Some(alt) = gga.altitude {
            data.altitude_m = alt;
        }
    }

    // GGA carries none of the velocity or accuracy fields. Clear them rather
    // than leave the last NAV-PVT values looking current (a stale velocity
    // would steer the navigator); NaN also fails every accuracy gate.
    data.vel_n_ms = f32::NAN;
    data.vel_e_ms = f32::NAN;
    data.vel_d_ms = f32::NAN;
    data.ground_speed_ms = f32::NAN;
    data.heading_motion_deg = f32::NAN;
    data.h_acc_m = f32::NAN;
    data.v_acc_m = f32::NAN;
    data.s_acc_ms = f32::NAN;
}
