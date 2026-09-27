use core::cell::Cell;

use elle_control::governor::RpmGovernor;
use embassy_dshot::ExtendedTelemetry;
use embassy_dshot::rp::BidirDshotPio;
use embassy_rp::peripherals::{PIO1, PIO2};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Ticker, Timer};

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

/// Spin direction asserted at arm time, from the platform config. Sent every boot and
/// never followed by `SettingsSave`: no ESC EEPROM wear, and the direction is correct
/// even after an ESC swap or factory reset.
const SPIN_DIRECTION_CMD: embassy_dshot::Command = if elle_config::ENGINE_SPIN_REVERSED {
    embassy_dshot::Command::SpinDirectonReversed
} else {
    embassy_dshot::Command::SpinDirectionNormal
};

/// Log a failed arm-time send. These run once at boot, so a warning per failure is
/// cheap and the only sign that the ESC may be left unarmed or misconfigured.
/// Per-tick sends in the 1kHz loop are deliberately not logged.
fn warn_on_err(what: &str, result: Result<(), embassy_dshot::DshotError>) {
    if let Err(e) = result {
        defmt::warn!("DShot: {} failed: {}", what, e);
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

/// Per-engine bidir telemetry state for auto-fallback.
struct EngineState {
    fail_count: u16,
    bidir_enabled: bool,
    governor: RpmGovernor,
}

impl EngineState {
    const fn new() -> Self {
        Self {
            fail_count: 0,
            bidir_enabled: true,
            governor: RpmGovernor::new(),
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
                "DShot: bidir fallback triggered after {} failures, governor reset",
                self.fail_count
            );
        }
    }
}

/// Update a single engine's reading from DShot telemetry result.
fn update_engine_unit(
    unit: &mut EngineUnitReading,
    state: &mut EngineState,
    edt: Option<ExtendedTelemetry>,
    dshot_value: u16,
    target: u32,
) {
    unit.throttle = dshot_value;
    unit.target_erpm = target;
    match edt {
        Some(ExtendedTelemetry::Erpm { erpm, .. }) => {
            unit.erpm = erpm;
            unit.valid = true;
            state.record_success();
        }
        Some(ref edt) => {
            unit.apply_edt(edt);
            state.record_success();
        }
        // Only a commanded stop (target 0) means the engine is genuinely winding
        // down to zero. A governor output of 0 does NOT: it means the PI wants less
        // than the feedforward can express while the prop is still spinning fast,
        // and fabricating erpm=0 there feeds a bogus measurement straight back into
        // the loop (and into ULog).
        None if target == 0 => {
            unit.erpm = 0;
            unit.valid = true;
        }
        None if state.bidir_enabled => {
            state.record_failure();
        }
        None => {
            unit.valid = false;
        }
    }
}

/// Wrapper around two bidirectional DShot ESCs (left and right engines).
struct DshotEngines<'a> {
    left: BidirDshotPio<'a, PIO1, 0>,
    right: BidirDshotPio<'a, PIO2, 0>,
}

impl<'a> DshotEngines<'a> {
    fn new(left: BidirDshotPio<'a, PIO1, 0>, right: BidirDshotPio<'a, PIO2, 0>) -> Self {
        Self { left, right }
    }

    /// Arm both ESCs and enable extended telemetry.
    async fn arm(&mut self, duration: Duration) {
        let (l, r) = embassy_futures::join::join(
            self.left.arm_async(duration),
            self.right.arm_async(duration),
        )
        .await;
        warn_on_err("arm (left)", l);
        warn_on_err("arm (right)", r);

        // Assert spin direction before enabling telemetry — the ESC is stopped here
        // (arm_async just spent its whole duration sending MotorStop), which is what
        // settings commands require.
        for _ in 0..SPIN_DIRECTION_REPEAT {
            let (l, r) = embassy_futures::join::join(
                self.left.send_command_async(SPIN_DIRECTION_CMD),
                self.right.send_command_async(SPIN_DIRECTION_CMD),
            )
            .await;
            warn_on_err("spin direction (left)", l);
            warn_on_err("spin direction (right)", r);
            Timer::after(Duration::from_micros(300)).await;
        }
        defmt::info!(
            "DShot: spin direction set (reversed: {})",
            elle_config::ENGINE_SPIN_REVERSED
        );

        // Enable extended telemetry (must send command 6 times per DShot protocol)
        for _ in 0..EDT_ENABLE_REPEAT {
            let (l, r) = embassy_futures::join::join(
                self.left
                    .send_command_async(embassy_dshot::Command::ExtendedTelemetryEnable),
                self.right
                    .send_command_async(embassy_dshot::Command::ExtendedTelemetryEnable),
            )
            .await;
            warn_on_err("EDT enable (left)", l);
            warn_on_err("EDT enable (right)", r);
            Timer::after(Duration::from_micros(300)).await;
        }
        defmt::info!("DShot: extended telemetry enabled");
    }

    /// Send throttle commands to both engines and return extended telemetry results.
    ///
    /// `left_stop`/`right_stop` come from the *target* eRPM being zero, not from the
    /// governor output being zero — see `update_engine_unit`. A governor output of 0
    /// is the protocol's minimum spin value (frame 48), which still returns telemetry.
    ///
    /// Returns `(Option<ExtendedTelemetry>, Option<ExtendedTelemetry>)` per engine.
    /// When bidir is disabled for an engine, uses fire-and-forget (no telemetry).
    /// Bidir reads are wrapped with a timeout to prevent the task from hanging
    /// if ESCs don't respond (e.g. bidir DShot not supported/wired).
    async fn set_throttle(
        &mut self,
        left: u16,
        right: u16,
        left_bidir: bool,
        right_bidir: bool,
        left_stop: bool,
        right_stop: bool,
    ) -> (Option<ExtendedTelemetry>, Option<ExtendedTelemetry>) {
        match (left_stop, right_stop) {
            (true, true) => {
                let _ = embassy_futures::join::join(
                    self.left
                        .send_command_async(embassy_dshot::Command::MotorStop),
                    self.right
                        .send_command_async(embassy_dshot::Command::MotorStop),
                )
                .await;
                (None, None)
            }
            _ => {
                let l_edt = self.send_one_engine_left(left, left_bidir, left_stop).await;
                let r_edt = self
                    .send_one_engine_right(right, right_bidir, right_stop)
                    .await;
                (l_edt, r_edt)
            }
        }
    }

    /// Send throttle to left engine with optional bidir telemetry + timeout.
    async fn send_one_engine_left(
        &mut self,
        value: u16,
        bidir: bool,
        stop: bool,
    ) -> Option<ExtendedTelemetry> {
        if stop {
            let _ = self
                .left
                .send_command_async(embassy_dshot::Command::MotorStop)
                .await;
            return None;
        }
        if !bidir {
            let _ = self.left.throttle_async(value).await;
            return None;
        }
        match embassy_time::with_timeout(
            BIDIR_READ_TIMEOUT,
            self.left.read_extended_telemetry(value),
        )
        .await
        {
            Ok(Ok(edt)) => Some(edt),
            _ => None,
        }
    }

    /// Send throttle to right engine with optional bidir telemetry + timeout.
    async fn send_one_engine_right(
        &mut self,
        value: u16,
        bidir: bool,
        stop: bool,
    ) -> Option<ExtendedTelemetry> {
        if stop {
            let _ = self
                .right
                .send_command_async(embassy_dshot::Command::MotorStop)
                .await;
            return None;
        }
        if !bidir {
            let _ = self.right.throttle_async(value).await;
            return None;
        }
        match embassy_time::with_timeout(
            BIDIR_READ_TIMEOUT,
            self.right.read_extended_telemetry(value),
        )
        .await
        {
            Ok(Ok(edt)) => Some(edt),
            _ => None,
        }
    }
}

/// Single-engine variant: arms ESC on startup, then resends latest `DSHOT_THROTTLE` at ~1kHz.
/// Only the left/primary eRPM value from `DSHOT_THROTTLE` is used; right stays zero.
#[cfg(feature = "single-engine")]
#[embassy_executor::task]
pub async fn dshot_single_task(engine: BidirDshotPio<'static, PIO1, 0>) {
    defmt::info!("DShot single-engine task: arming ESC (2s)");
    let mut engine = engine;
    warn_on_err("arm", engine.arm_async(Duration::from_secs(2)).await);

    // Assert spin direction before enabling telemetry — the ESC is stopped here
    // (arm_async just spent its whole duration sending MotorStop), which is what
    // settings commands require.
    for _ in 0..SPIN_DIRECTION_REPEAT {
        warn_on_err(
            "spin direction",
            engine.send_command_async(SPIN_DIRECTION_CMD).await,
        );
        Timer::after(Duration::from_micros(300)).await;
    }
    defmt::info!(
        "DShot: spin direction set (reversed: {})",
        elle_config::ENGINE_SPIN_REVERSED
    );

    // Enable extended telemetry (must send command 6 times per DShot protocol)
    for _ in 0..EDT_ENABLE_REPEAT {
        warn_on_err(
            "EDT enable",
            engine
                .send_command_async(embassy_dshot::Command::ExtendedTelemetryEnable)
                .await,
        );
        Timer::after(Duration::from_micros(300)).await;
    }
    defmt::info!("DShot single-engine task: armed, entering 1kHz loop");

    let mut target_erpm = 0u32;
    let mut ticker = Ticker::every(Duration::from_millis(1));
    let mut state = EngineState::new();
    let mut reading = EngineReading::default();

    loop {
        // Check for beep request (non-blocking)
        if let Some(pattern) = BEEP_SIGNAL.try_take() {
            use embassy_dshot::Command;
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
        let edt = if target_erpm == 0 {
            let _ = engine
                .send_command_async(embassy_dshot::Command::MotorStop)
                .await;
            None
        } else if !state.bidir_enabled {
            let _ = engine.throttle_async(dshot_val).await;
            None
        } else {
            match embassy_time::with_timeout(
                BIDIR_READ_TIMEOUT,
                engine.read_extended_telemetry(dshot_val),
            )
            .await
            {
                Ok(Ok(e)) => Some(e),
                _ => None,
            }
        };

        update_engine_unit(&mut reading.left, &mut state, edt, dshot_val, target_erpm);
        // right stays zeroed (single engine)
        ENGINE_CACHE.lock(|c| c.set(reading));
        ticker.next().await;
    }
}

/// Arms ESCs on startup, then resends latest `DSHOT_THROTTLE` values at ~1kHz.
#[embassy_executor::task]
pub async fn dshot_task(
    engine_left: BidirDshotPio<'static, PIO1, 0>,
    engine_right: BidirDshotPio<'static, PIO2, 0>,
) {
    defmt::info!("DShot task: arming ESCs (2s)");
    let mut engines = DshotEngines::new(engine_left, engine_right);
    engines.arm(Duration::from_secs(2)).await;
    defmt::info!("DShot task: armed, entering 1kHz send loop");

    let mut target = (0u32, 0u32);
    let mut ticker = Ticker::every(Duration::from_millis(1));
    let mut left_state = EngineState::new();
    let mut right_state = EngineState::new();
    let mut reading = EngineReading::default();

    loop {
        // Check for beep request (non-blocking)
        if let Some(pattern) = BEEP_SIGNAL.try_take() {
            use embassy_dshot::Command;
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

        let (l_edt, r_edt) = engines
            .set_throttle(
                left_dshot,
                right_dshot,
                left_state.bidir_enabled,
                right_state.bidir_enabled,
                target.0 == 0,
                target.1 == 0,
            )
            .await;

        update_engine_unit(
            &mut reading.left,
            &mut left_state,
            l_edt,
            left_dshot,
            target.0,
        );
        update_engine_unit(
            &mut reading.right,
            &mut right_state,
            r_edt,
            right_dshot,
            target.1,
        );

        ENGINE_CACHE.lock(|c| c.set(reading));
        ticker.next().await;
    }
}
