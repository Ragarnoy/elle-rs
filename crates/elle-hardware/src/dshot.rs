use core::cell::Cell;

use elle_control::dshot_pace::next_deadline_us;
use elle_control::esc_link::{EdtAction, EdtWatch, EscLink, EscLinkEvent};
use elle_control::governor::RpmGovernor;
use embassy_dshot::rp::BidirDshotPio;
use embassy_dshot::{Command, DshotError, ExtendedTelemetry};
use embassy_rp::peripherals::{PIO1, PIO2};
use embassy_rp::pio::Instance;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, TimeoutError, Timer};

use crate::{elle_event, event};

/// Target eRPM per engine (left, right). Governor PI converts to DShot at 1kHz.
pub static DSHOT_THROTTLE: Signal<CriticalSectionRawMutex, (u32, u32)> = Signal::new();

/// Beep request signal. The DShot task checks this each tick and plays the beep sequence.
pub static BEEP_SIGNAL: Signal<CriticalSectionRawMutex, BeepPattern> = Signal::new();

/// Beep patterns available via DShot commands.
#[derive(Clone, Copy, defmt::Format)]
pub enum BeepPattern {
    /// Short single beep (arm confirmation)
    ArmBeep,
    /// Two short beeps (disarm confirmation)
    DisarmBeep,
}

/// Consecutive bidir telemetry failure threshold before falling back to fire-and-forget.
const BIDIR_FALLBACK_THRESHOLD: u16 = 100;

/// Timeout for a single bidir telemetry read. DShot300 frame + response takes ~120µs;
/// 2ms is generous but prevents indefinite hangs if the ESC doesn't respond.
const BIDIR_READ_TIMEOUT: Duration = Duration::from_millis(2);

/// Number of times to send ExtendedTelemetryEnable command (DShot protocol requirement).
const EDT_ENABLE_REPEAT: u8 = 6;

/// Number of times to send the spin-direction command (DShot settings commands need 6).
const SPIN_DIRECTION_REPEAT: u8 = 6;

/// Frames sent after the boot configuration to see whether each ESC was
/// listening. One that answers is marked configured; one that stays silent is
/// configured again once it starts answering.
const BOOT_PROBE_FRAMES: u32 = 20;

/// Spin direction asserted at arm time, from the platform config. Sent every boot and
/// never followed by `SettingsSave`: no ESC EEPROM wear, and the direction is correct
/// even after an ESC swap or factory reset.
const SPIN_DIRECTION_CMD: Command = if elle_config::ENGINE_SPIN_REVERSED {
    Command::SpinDirectonReversed
} else {
    Command::SpinDirectionNormal
};

/// Log a failed configuration send. These run at boot or after an ESC restart,
/// so a warning per failure is cheap and the only sign that the ESC may be left
/// unarmed or misconfigured. Per-tick sends in the 1kHz loop are not logged.
fn warn_on_err(what: &str, side: Side, result: Result<(), DshotError>) {
    if let Err(e) = result {
        defmt::warn!("DShot: {} ({}) failed: {}", what, side.name(), e);
    }
}

/// Which engine: event codes and log text carry the side.
#[derive(Clone, Copy, defmt::Format)]
enum Side {
    Left,
    Right,
}

impl Side {
    const fn name(self) -> &'static str {
        match self {
            Self::Left => "left",
            Self::Right => "right",
        }
    }

    const fn silent_event(self) -> u16 {
        match self {
            Self::Left => event::EVT_ESC_LEFT_SILENT,
            Self::Right => event::EVT_ESC_RIGHT_SILENT,
        }
    }

    const fn reconfigured_event(self) -> u16 {
        match self {
            Self::Left => event::EVT_ESC_LEFT_RECONFIGURED,
            Self::Right => event::EVT_ESC_RIGHT_RECONFIGURED,
        }
    }

    const fn no_edt_event(self) -> u16 {
        match self {
            Self::Left => event::EVT_ESC_LEFT_NO_EDT,
            Self::Right => event::EVT_ESC_RIGHT_NO_EDT,
        }
    }
}

/// Per-engine telemetry snapshot.
#[derive(Clone, Copy, Default, defmt::Format)]
pub struct EngineUnitReading {
    pub erpm: u32,
    pub throttle: u16,
    pub valid: bool,
    pub target_erpm: u32,
    /// ESC temperature in °C (1°C/LSB, from EDT)
    pub temperature: u8,
    /// Supply voltage in millivolts (from EDT, 250mV/LSB)
    pub voltage_mv: u32,
    /// Current draw in milliamps (from EDT, 1A/LSB)
    pub current_ma: u32,
    /// Valid telemetry replies since boot.
    pub replies: u32,
    /// Telemetry requests with no reply since boot.
    pub timeouts: u32,
    /// Replies that failed GCR decoding or CRC since boot: the line-noise indicator.
    pub bad_frames: u32,
    /// Extended-telemetry frames (temperature, voltage, ...) since boot. Zero
    /// while replies climb means the EDT enable did not take.
    pub edt_frames: u32,
    /// Times the ESC was re-sent its configuration: after going silent, after
    /// missing the boot configuration, or after delivering no EDT.
    pub reconfigs: u16,
}

impl EngineUnitReading {
    #[must_use]
    pub(crate) const fn new() -> Self {
        Self {
            erpm: 0,
            throttle: 0,
            valid: false,
            target_erpm: 0,
            temperature: 0,
            voltage_mv: 0,
            current_ma: 0,
            replies: 0,
            timeouts: 0,
            bad_frames: 0,
            edt_frames: 0,
            reconfigs: 0,
        }
    }

    fn apply_edt(&mut self, edt: &ExtendedTelemetry) {
        match *edt {
            ExtendedTelemetry::Temperature(t) => self.temperature = t,
            ExtendedTelemetry::Voltage(mv) => self.voltage_mv = mv,
            // ESCs without a current sensor emit garbage near the 8-bit ceiling
            // (240-255A observed); anything >=200A is impossible on this airframe
            ExtendedTelemetry::Current(ma) if ma < 200_000 => self.current_ma = ma,
            _ => {}
        }
    }
}

/// Engine RPM + throttle + extended telemetry snapshot from the DShot task.
#[derive(Clone, Copy, Default, defmt::Format)]
pub struct EngineReading {
    pub left: EngineUnitReading,
    pub right: EngineUnitReading,
}

impl EngineReading {
    #[must_use]
    pub(crate) const fn new() -> Self {
        Self {
            left: EngineUnitReading::new(),
            right: EngineUnitReading::new(),
        }
    }
}

/// Non-consuming cache for engine telemetry (Mutex<Cell<>> pattern).
pub static ENGINE_CACHE: Mutex<CriticalSectionRawMutex, Cell<EngineReading>> =
    Mutex::new(Cell::new(EngineReading::new()));

/// What came back for one frame.
#[derive(Clone, Copy)]
enum Reply {
    Telemetry(ExtendedTelemetry),
    /// A reply arrived but failed GCR decoding or CRC.
    Corrupt,
    /// Telemetry was requested and nothing came back in time.
    Missing,
    /// Fire-and-forget frame (bidir fallback while running).
    NotRequested,
}

impl Reply {
    fn from_read(result: Result<Result<ExtendedTelemetry, DshotError>, TimeoutError>) -> Self {
        match result {
            Ok(Ok(edt)) => Self::Telemetry(edt),
            Ok(Err(DshotError::GcrDecodeError | DshotError::InvalidTelemetryCrc)) => Self::Corrupt,
            _ => Self::Missing,
        }
    }

    /// An extended-telemetry frame (anything but eRPM).
    const fn is_edt(self) -> bool {
        matches!(self, Self::Telemetry(t) if !matches!(t, ExtendedTelemetry::Erpm { .. }))
    }

    /// `Some(answered)` when telemetry was requested. A corrupt reply still
    /// proves the ESC is alive.
    const fn answered(self) -> Option<bool> {
        match self {
            Self::Telemetry(_) | Self::Corrupt => Some(true),
            Self::Missing => Some(false),
            Self::NotRequested => None,
        }
    }
}

/// Per-engine bidir telemetry state for auto-fallback, plus the ESC link.
struct EngineState {
    side: Side,
    fail_count: u16,
    bidir_enabled: bool,
    governor: RpmGovernor,
    link: EscLink,
    /// Set when the ESC (re)appeared: send it the configuration once it has been
    /// answering for `ESC_RECONFIGURE_SETTLE_FRAMES` while stopped.
    reconfigure_in: Option<u32>,
    /// Whether the last configuration visibly took (EDT frames arriving).
    edt: EdtWatch,
}

impl EngineState {
    const fn new(side: Side) -> Self {
        Self {
            side,
            fail_count: 0,
            bidir_enabled: true,
            governor: RpmGovernor::new(),
            link: EscLink::new(elle_config::ESC_SILENT_FRAMES),
            reconfigure_in: None,
            edt: EdtWatch::new(
                elle_config::ESC_EDT_CONFIRM_FRAMES,
                elle_config::ESC_EDT_MAX_RETRIES,
            ),
        }
    }

    fn record_success(&mut self) {
        self.fail_count = 0;
    }

    fn record_failure(&mut self) {
        self.fail_count = self.fail_count.saturating_add(1);
        if self.fail_count >= BIDIR_FALLBACK_THRESHOLD && self.bidir_enabled {
            self.bidir_enabled = false;
            self.governor.reset();
            defmt::warn!(
                "DShot {}: bidir fallback triggered after {} failures, governor reset",
                self.side.name(),
                self.fail_count
            );
        }
    }

    /// Track the ESC link. On `Appeared`, schedule the configuration and, if the
    /// bidir fallback had given up on this ESC, take telemetry back: it answers.
    fn track_link(&mut self, answered: bool) {
        match self.link.update(answered) {
            Some(EscLinkEvent::Lost) => {
                elle_event!(
                    warn,
                    self.side.silent_event(),
                    "DShot {}: ESC stopped replying (power loss or restart?)",
                    self.side.name()
                );
            }
            Some(EscLinkEvent::Appeared) => {
                self.reconfigure_in = Some(elle_config::ESC_RECONFIGURE_SETTLE_FRAMES);
                self.edt.reset();
                if !self.bidir_enabled {
                    self.bidir_enabled = true;
                    self.fail_count = 0;
                    defmt::info!(
                        "DShot {}: ESC answering, bidir re-enabled",
                        self.side.name()
                    );
                }
            }
            None => {}
        }
    }

    /// Whether the pending configuration is due now. Counts down answered
    /// frames while stopped, so a restarted ESC gets time to finish booting.
    fn reconfigure_due(&mut self, answered: bool, stopped: bool) -> bool {
        match self.reconfigure_in {
            Some(0) if stopped => {
                self.reconfigure_in = None;
                true
            }
            Some(n) if stopped && answered => {
                self.reconfigure_in = Some(n - 1);
                false
            }
            _ => false,
        }
    }
}

/// Update a single engine's reading from one frame's reply.
fn update_engine_unit(
    unit: &mut EngineUnitReading,
    state: &mut EngineState,
    reply: Reply,
    dshot_value: u16,
    target: u32,
) {
    unit.throttle = dshot_value;
    unit.target_erpm = target;
    match reply {
        Reply::Telemetry(ExtendedTelemetry::Erpm { erpm, .. }) => {
            unit.erpm = erpm;
            unit.valid = true;
            state.record_success();
        }
        Reply::Telemetry(ref edt) => {
            unit.apply_edt(edt);
            state.record_success();
        }
        // Only a commanded stop (target 0) means the engine is genuinely winding
        // down to zero. A governor output of 0 does NOT: it means the PI wants less
        // than the feedforward can express while the prop is still spinning fast,
        // and fabricating erpm=0 there feeds a bogus measurement straight back into
        // the loop (and into ULog).
        _ if target == 0 => {
            unit.erpm = 0;
            unit.valid = true;
        }
        _ if state.bidir_enabled => {
            state.record_failure();
        }
        _ => {
            unit.valid = false;
        }
    }
    match reply {
        Reply::Telemetry(_) => {
            unit.replies = unit.replies.wrapping_add(1);
            if reply.is_edt() {
                unit.edt_frames = unit.edt_frames.wrapping_add(1);
            }
        }
        Reply::Corrupt => unit.bad_frames = unit.bad_frames.wrapping_add(1),
        Reply::Missing => unit.timeouts = unit.timeouts.wrapping_add(1),
        Reply::NotRequested => {}
    }
    if let Some(answered) = reply.answered() {
        state.track_link(answered);
    }
}

/// Send one frame to one ESC.
///
/// A stopped engine (`stop`, from the *target* eRPM being zero, not from the
/// governor output being zero — see `update_engine_unit`) gets `MotorStop` with
/// a telemetry request, always: the reply proves the ESC is alive and carries
/// EDT at idle, and waiting for it keeps frames from overlapping on the wire. A
/// governor output of 0 is the protocol's minimum spin value (frame 48), which
/// still returns telemetry. Running with bidir disabled falls back to
/// fire-and-forget. Reads are bounded by `BIDIR_READ_TIMEOUT`.
async fn send_frame<PIO: Instance, const SM: usize>(
    esc: &mut BidirDshotPio<'_, PIO, SM>,
    value: u16,
    bidir: bool,
    stop: bool,
) -> Reply {
    if stop {
        return Reply::from_read(
            embassy_time::with_timeout(
                BIDIR_READ_TIMEOUT,
                esc.command_with_extended_telemetry(Command::MotorStop),
            )
            .await,
        );
    }
    if !bidir {
        let _ = esc.throttle_async(value).await;
        return Reply::NotRequested;
    }
    Reply::from_read(
        embassy_time::with_timeout(BIDIR_READ_TIMEOUT, esc.read_extended_telemetry(value)).await,
    )
}

/// Send the per-boot configuration: spin direction, then extended telemetry.
/// The ESC must be stopped (settings commands require it).
async fn configure<PIO: Instance, const SM: usize>(
    esc: &mut BidirDshotPio<'_, PIO, SM>,
    side: Side,
) {
    for _ in 0..SPIN_DIRECTION_REPEAT {
        warn_on_err(
            "spin direction",
            side,
            esc.send_command_async(SPIN_DIRECTION_CMD).await,
        );
        Timer::after(Duration::from_micros(300)).await;
    }
    // Extended telemetry must be enabled with 6 sends per the DShot protocol.
    for _ in 0..EDT_ENABLE_REPEAT {
        warn_on_err(
            "EDT enable",
            side,
            esc.send_command_async(Command::ExtendedTelemetryEnable)
                .await,
        );
        Timer::after(Duration::from_micros(300)).await;
    }
    defmt::info!(
        "DShot {}: spin direction (reversed: {}) and EDT enable sent",
        side.name(),
        elle_config::ENGINE_SPIN_REVERSED
    );
}

/// After the boot configuration, see whether the ESC answers. One that does
/// took the configuration; one that doesn't is configured when it appears.
async fn probe_after_boot<PIO: Instance, const SM: usize>(
    esc: &mut BidirDshotPio<'_, PIO, SM>,
    state: &mut EngineState,
) {
    for _ in 0..BOOT_PROBE_FRAMES {
        if send_frame(esc, 0, true, true).await.answered() == Some(true) {
            state.link.mark_up();
            return;
        }
        Timer::after(Duration::from_millis(1)).await;
    }
    defmt::warn!(
        "DShot {}: no reply after boot configuration; will configure when the ESC answers",
        state.side.name()
    );
}

/// Run the pending configuration for one ESC if it is due: the ESC reappeared
/// (restart, or late power), or it answers but has sent no extended telemetry
/// since it was last configured, so that configuration did not take. An ESC
/// replies with eRPM whether or not EDT is enabled, so replies alone can't tell.
async fn reconfigure_if_due<PIO: Instance, const SM: usize>(
    esc: &mut BidirDshotPio<'_, PIO, SM>,
    state: &mut EngineState,
    unit: &mut EngineUnitReading,
    reply: Reply,
    stopped: bool,
) {
    let answered = reply.answered() == Some(true);
    let appeared = state.reconfigure_due(answered, stopped);
    let edt = state.edt.update(answered, reply.is_edt(), stopped);
    if appeared || edt == Some(EdtAction::Reconfigure) {
        configure(esc, state.side).await;
        state.edt.configured();
        unit.reconfigs = unit.reconfigs.wrapping_add(1);
        elle_event!(
            info,
            state.side.reconfigured_event(),
            "DShot {}: configuration re-sent ({})",
            state.side.name(),
            if appeared {
                "ESC answering again"
            } else {
                "no EDT"
            }
        );
    } else if edt == Some(EdtAction::GiveUp) {
        elle_event!(
            warn,
            state.side.no_edt_event(),
            "DShot {}: still no EDT after {} re-sends; spin direction unconfirmed",
            state.side.name(),
            elle_config::ESC_EDT_MAX_RETRIES
        );
    }
}

/// Wrapper around two bidirectional DShot ESCs (left and right engines).
struct DshotEngines<'a> {
    left: BidirDshotPio<'a, PIO1, 0>,
    right: BidirDshotPio<'a, PIO2, 0>,
}

impl<'a> DshotEngines<'a> {
    const fn new(left: BidirDshotPio<'a, PIO1, 0>, right: BidirDshotPio<'a, PIO2, 0>) -> Self {
        Self { left, right }
    }

    /// Arm both ESCs, configure them, and check they answered.
    async fn arm(&mut self, duration: Duration, left: &mut EngineState, right: &mut EngineState) {
        let (l, r) = embassy_futures::join::join(
            self.left.arm_async(duration),
            self.right.arm_async(duration),
        )
        .await;
        warn_on_err("arm", Side::Left, l);
        warn_on_err("arm", Side::Right, r);

        // Configure right after the MotorStop burst: the ESCs are stopped, which
        // settings commands require.
        embassy_futures::join::join(
            configure(&mut self.left, Side::Left),
            configure(&mut self.right, Side::Right),
        )
        .await;
        embassy_futures::join::join(
            probe_after_boot(&mut self.left, left),
            probe_after_boot(&mut self.right, right),
        )
        .await;
    }
}

/// MotorStop burst sent at startup so the ESCs arm (`elle_config::ARM_DURATION_MS`).
const ARM_DURATION: Duration = Duration::from_millis(elle_config::ARM_DURATION_MS as u64);

/// Wait for the next frame slot (`DSHOT_LOOP_PERIOD_US`). Never replays ticks
/// missed during a stall — a `Ticker` replays them back to back, truncating
/// frames on the wire — and keeps frames `DSHOT_MIN_FRAME_GAP_US` apart.
async fn pace(deadline: &mut Instant) {
    *deadline = Instant::from_micros(next_deadline_us(
        deadline.as_micros(),
        Instant::now().as_micros(),
        elle_config::DSHOT_LOOP_PERIOD_US,
        elle_config::DSHOT_MIN_FRAME_GAP_US,
    ));
    Timer::at(*deadline).await;
}

/// Single-engine variant: arms ESC on startup, then resends latest `DSHOT_THROTTLE` at ~1kHz
/// (paced, see `pace`). Only the left/primary eRPM value from `DSHOT_THROTTLE` is used;
/// right stays zero.
#[cfg(feature = "single-engine")]
#[embassy_executor::task]
pub async fn dshot_single_task(engine: BidirDshotPio<'static, PIO1, 0>) {
    defmt::info!(
        "DShot single-engine task: arming ESC ({}ms)",
        ARM_DURATION.as_millis()
    );
    let mut engine = engine;
    let mut state = EngineState::new(Side::Left);
    warn_on_err("arm", Side::Left, engine.arm_async(ARM_DURATION).await);
    configure(&mut engine, Side::Left).await;
    probe_after_boot(&mut engine, &mut state).await;
    defmt::info!("DShot single-engine task: armed, entering 1kHz loop");

    let mut target_erpm = 0u32;
    let mut deadline = Instant::now();
    let mut reading = EngineReading::default();

    loop {
        // Check for beep request (non-blocking)
        if let Some(pattern) = BEEP_SIGNAL.try_take() {
            match pattern {
                BeepPattern::ArmBeep => {
                    for _ in 0..10 {
                        let _ = engine.send_command_async(Command::Beep1).await;
                        Timer::after(Duration::from_millis(1)).await;
                    }
                    Timer::after(Duration::from_millis(200)).await;
                }
                BeepPattern::DisarmBeep => {
                    for _ in 0..10 {
                        let _ = engine.send_command_async(Command::Beep2).await;
                        Timer::after(Duration::from_millis(1)).await;
                    }
                    Timer::after(Duration::from_millis(100)).await;
                    for _ in 0..10 {
                        let _ = engine.send_command_async(Command::Beep2).await;
                        Timer::after(Duration::from_millis(1)).await;
                    }
                    Timer::after(Duration::from_millis(200)).await;
                }
            }
        }

        if let Some(t) = DSHOT_THROTTLE.try_take() {
            target_erpm = t.0; // single engine uses only the left/primary value
        }

        let dshot_val = state.governor.update(
            target_erpm,
            reading.left.erpm,
            state.bidir_enabled && reading.left.valid,
        );

        // MotorStop only on a commanded stop. A governor output of 0 is the
        // protocol's minimum spin value, not a stop — cutting the ESC there loses
        // telemetry and leaves the loop with no way back up.
        let stopped = target_erpm == 0;
        let reply = send_frame(&mut engine, dshot_val, state.bidir_enabled, stopped).await;

        update_engine_unit(&mut reading.left, &mut state, reply, dshot_val, target_erpm);
        reconfigure_if_due(&mut engine, &mut state, &mut reading.left, reply, stopped).await;
        // right stays zeroed (single engine)
        ENGINE_CACHE.lock(|c| c.set(reading));
        pace(&mut deadline).await;
    }
}

/// Arms ESCs on startup, then resends latest `DSHOT_THROTTLE` values at ~1kHz
/// (paced, see `pace`).
#[embassy_executor::task]
pub async fn dshot_task(
    engine_left: BidirDshotPio<'static, PIO1, 0>,
    engine_right: BidirDshotPio<'static, PIO2, 0>,
) {
    defmt::info!("DShot task: arming ESCs ({}ms)", ARM_DURATION.as_millis());
    let mut engines = DshotEngines::new(engine_left, engine_right);
    let mut left_state = EngineState::new(Side::Left);
    let mut right_state = EngineState::new(Side::Right);
    engines
        .arm(ARM_DURATION, &mut left_state, &mut right_state)
        .await;
    defmt::info!("DShot task: armed, entering 1kHz send loop");

    let mut target = (0u32, 0u32);
    let mut deadline = Instant::now();
    let mut reading = EngineReading::default();

    loop {
        // Check for beep request (non-blocking)
        if let Some(pattern) = BEEP_SIGNAL.try_take() {
            match pattern {
                BeepPattern::ArmBeep => {
                    // Single beep: Beep1 repeated 10x (DShot spec)
                    for _ in 0..10 {
                        let _ = engines.left.send_command_async(Command::Beep1).await;
                        let _ = engines.right.send_command_async(Command::Beep1).await;
                        Timer::after(Duration::from_millis(1)).await;
                    }
                    Timer::after(Duration::from_millis(200)).await;
                }
                BeepPattern::DisarmBeep => {
                    // Two short beeps
                    for _ in 0..10 {
                        let _ = engines.left.send_command_async(Command::Beep2).await;
                        let _ = engines.right.send_command_async(Command::Beep2).await;
                        Timer::after(Duration::from_millis(1)).await;
                    }
                    Timer::after(Duration::from_millis(100)).await;
                    for _ in 0..10 {
                        let _ = engines.left.send_command_async(Command::Beep2).await;
                        let _ = engines.right.send_command_async(Command::Beep2).await;
                        Timer::after(Duration::from_millis(1)).await;
                    }
                    Timer::after(Duration::from_millis(200)).await;
                }
            }
        }

        if let Some(t) = DSHOT_THROTTLE.try_take() {
            target = t;
        }

        // Governor PI converts eRPM targets to DShot values
        let left_dshot = left_state.governor.update(
            target.0,
            reading.left.erpm,
            left_state.bidir_enabled && reading.left.valid,
        );
        let right_dshot = right_state.governor.update(
            target.1,
            reading.right.erpm,
            right_state.bidir_enabled && reading.right.valid,
        );

        // Both engines at once: they are on separate PIO blocks.
        let (left_stopped, right_stopped) = (target.0 == 0, target.1 == 0);
        let (l_reply, r_reply) = embassy_futures::join::join(
            send_frame(
                &mut engines.left,
                left_dshot,
                left_state.bidir_enabled,
                left_stopped,
            ),
            send_frame(
                &mut engines.right,
                right_dshot,
                right_state.bidir_enabled,
                right_stopped,
            ),
        )
        .await;

        update_engine_unit(
            &mut reading.left,
            &mut left_state,
            l_reply,
            left_dshot,
            target.0,
        );
        update_engine_unit(
            &mut reading.right,
            &mut right_state,
            r_reply,
            right_dshot,
            target.1,
        );
        reconfigure_if_due(
            &mut engines.left,
            &mut left_state,
            &mut reading.left,
            l_reply,
            left_stopped,
        )
        .await;
        reconfigure_if_due(
            &mut engines.right,
            &mut right_state,
            &mut reading.right,
            r_reply,
            right_stopped,
        )
        .await;

        ENGINE_CACHE.lock(|c| c.set(reading));
        pace(&mut deadline).await;
    }
}
