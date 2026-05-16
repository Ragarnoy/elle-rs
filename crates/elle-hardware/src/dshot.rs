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
    pub const fn new() -> Self {
        Self {
            erpm: 0, throttle: 0, valid: false, target_erpm: 0,
            temperature: 0, voltage_mv: 0, current_ma: 0,
        }
    }

    fn apply_edt(&mut self, edt: &ExtendedTelemetry) {
        match *edt {
            ExtendedTelemetry::Temperature(t) => self.temperature = t,
            ExtendedTelemetry::Voltage(mv) => self.voltage_mv = mv,
            ExtendedTelemetry::Current(ma) => self.current_ma = ma,
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
    pub const fn new() -> Self {
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
        None if dshot_value == 0 => {
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
    left: BidirDshotPio<'a, PIO1>,
    right: BidirDshotPio<'a, PIO2>,
}

impl<'a> DshotEngines<'a> {
    fn new(left: BidirDshotPio<'a, PIO1>, right: BidirDshotPio<'a, PIO2>) -> Self {
        Self { left, right }
    }

    /// Arm both ESCs and enable extended telemetry.
    async fn arm(&mut self, duration: Duration) {
        embassy_futures::join::join(
            self.left.arm_async(duration),
            self.right.arm_async(duration),
        )
        .await;

        // Enable extended telemetry (must send command 6 times per DShot protocol)
        for _ in 0..EDT_ENABLE_REPEAT {
            embassy_futures::join::join(
                self.left
                    .send_command_async(embassy_dshot::Command::ExtendedTelemetryEnable),
                self.right
                    .send_command_async(embassy_dshot::Command::ExtendedTelemetryEnable),
            )
            .await;
            Timer::after(Duration::from_micros(300)).await;
        }
        defmt::info!("DShot: extended telemetry enabled");
    }

    /// Send throttle commands to both engines and return extended telemetry results.
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
    ) -> (Option<ExtendedTelemetry>, Option<ExtendedTelemetry>) {
        match (left == 0, right == 0) {
            (true, true) => {
                embassy_futures::join::join(
                    self.left
                        .send_command_async(embassy_dshot::Command::MotorStop),
                    self.right
                        .send_command_async(embassy_dshot::Command::MotorStop),
                )
                .await;
                (None, None)
            }
            _ => {
                let l_edt = self.send_one_engine_left(left, left_bidir).await;
                let r_edt = self.send_one_engine_right(right, right_bidir).await;
                (l_edt, r_edt)
            }
        }
    }

    /// Send throttle to left engine with optional bidir telemetry + timeout.
    async fn send_one_engine_left(
        &mut self,
        value: u16,
        bidir: bool,
    ) -> Option<ExtendedTelemetry> {
        if value == 0 {
            self.left
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
    ) -> Option<ExtendedTelemetry> {
        if value == 0 {
            self.right
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
pub async fn dshot_single_task(engine: BidirDshotPio<'static, PIO1>) {
    defmt::info!("DShot single-engine task: arming ESC (2s)");
    let mut engine = engine;
    engine.arm_async(Duration::from_secs(2)).await;

    // Enable extended telemetry (must send command 6 times per DShot protocol)
    for _ in 0..EDT_ENABLE_REPEAT {
        engine
            .send_command_async(embassy_dshot::Command::ExtendedTelemetryEnable)
            .await;
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
                        engine.send_command_async(Command::Beep1).await;
                        Timer::after(Duration::from_millis(1)).await;
                    }
                    Timer::after(Duration::from_millis(200)).await;
                }
                BeepPattern::DisarmBeep => {
                    for _ in 0..10 {
                        engine.send_command_async(Command::Beep2).await;
                        Timer::after(Duration::from_millis(1)).await;
                    }
                    Timer::after(Duration::from_millis(100)).await;
                    for _ in 0..10 {
                        engine.send_command_async(Command::Beep2).await;
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

        let edt = if dshot_val == 0 {
            engine
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
    engine_left: BidirDshotPio<'static, PIO1>,
    engine_right: BidirDshotPio<'static, PIO2>,
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
                        engines.left.send_command_async(Command::Beep1).await;
                        engines.right.send_command_async(Command::Beep1).await;
                        Timer::after(Duration::from_millis(1)).await;
                    }
                    Timer::after(Duration::from_millis(200)).await;
                }
                BeepPattern::DisarmBeep => {
                    // Two short beeps
                    for _ in 0..10 {
                        engines.left.send_command_async(Command::Beep2).await;
                        engines.right.send_command_async(Command::Beep2).await;
                        Timer::after(Duration::from_millis(1)).await;
                    }
                    Timer::after(Duration::from_millis(100)).await;
                    for _ in 0..10 {
                        engines.left.send_command_async(Command::Beep2).await;
                        engines.right.send_command_async(Command::Beep2).await;
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
            )
            .await;

        update_engine_unit(&mut reading.left, &mut left_state, l_edt, left_dshot, target.0);
        update_engine_unit(&mut reading.right, &mut right_state, r_edt, right_dshot, target.1);

        ENGINE_CACHE.lock(|c| c.set(reading));
        ticker.next().await;
    }
}
