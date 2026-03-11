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

/// Consecutive bidir telemetry failure threshold before falling back to fire-and-forget.
const BIDIR_FALLBACK_THRESHOLD: u16 = 100;

/// Number of times to send ExtendedTelemetryEnable command (DShot protocol requirement).
const EDT_ENABLE_REPEAT: u8 = 6;

/// Engine RPM + throttle + extended telemetry snapshot from the DShot task.
#[derive(Clone, Copy, Default, defmt::Format)]
pub struct EngineReading {
    pub left_erpm: u32,
    pub right_erpm: u32,
    pub left_throttle: u16,
    pub right_throttle: u16,
    pub left_valid: bool,
    pub right_valid: bool,
    pub left_target_erpm: u32,
    pub right_target_erpm: u32,
    /// ESC temperature in °C (1°C/LSB, from EDT)
    pub left_temperature: u8,
    pub right_temperature: u8,
    /// Supply voltage in millivolts (from EDT, 250mV/LSB)
    pub left_voltage_mv: u32,
    pub right_voltage_mv: u32,
    /// Current draw in milliamps (from EDT, 1A/LSB)
    pub left_current_ma: u32,
    pub right_current_ma: u32,
}

/// Non-consuming cache for engine telemetry (Mutex<Cell<>> pattern).
pub static ENGINE_CACHE: Mutex<CriticalSectionRawMutex, Cell<EngineReading>> =
    Mutex::new(Cell::new(EngineReading {
        left_erpm: 0,
        right_erpm: 0,
        left_throttle: 0,
        right_throttle: 0,
        left_valid: false,
        right_valid: false,
        left_target_erpm: 0,
        right_target_erpm: 0,
        left_temperature: 0,
        right_temperature: 0,
        left_voltage_mv: 0,
        right_voltage_mv: 0,
        left_current_ma: 0,
        right_current_ma: 0,
    }));

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

/// Apply an extended telemetry reading to the left engine fields of an `EngineReading`.
fn apply_edt_left(reading: &mut EngineReading, edt: &ExtendedTelemetry) {
    match *edt {
        ExtendedTelemetry::Temperature(t) => reading.left_temperature = t,
        ExtendedTelemetry::Voltage(mv) => reading.left_voltage_mv = mv,
        ExtendedTelemetry::Current(ma) => reading.left_current_ma = ma,
        _ => {}
    }
}

/// Apply an extended telemetry reading to the right engine fields of an `EngineReading`.
fn apply_edt_right(reading: &mut EngineReading, edt: &ExtendedTelemetry) {
    match *edt {
        ExtendedTelemetry::Temperature(t) => reading.right_temperature = t,
        ExtendedTelemetry::Voltage(mv) => reading.right_voltage_mv = mv,
        ExtendedTelemetry::Current(ma) => reading.right_current_ma = ma,
        _ => {}
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
            (false, false) => match (left_bidir, right_bidir) {
                (true, true) => {
                    let (l_res, r_res) = embassy_futures::join::join(
                        self.left.read_extended_telemetry(left),
                        self.right.read_extended_telemetry(right),
                    )
                    .await;
                    (l_res.ok(), r_res.ok())
                }
                (true, false) => {
                    let (l_res, r_res) = embassy_futures::join::join(
                        self.left.read_extended_telemetry(left),
                        self.right.throttle_async(right),
                    )
                    .await;
                    if let Err(e) = &r_res {
                        defmt::warn!("DShot right send error: {}", e);
                    }
                    (l_res.ok(), None)
                }
                (false, true) => {
                    let (l_res, r_res) = embassy_futures::join::join(
                        self.left.throttle_async(left),
                        self.right.read_extended_telemetry(right),
                    )
                    .await;
                    if let Err(e) = &l_res {
                        defmt::warn!("DShot left send error: {}", e);
                    }
                    (None, r_res.ok())
                }
                (false, false) => {
                    let (l_res, r_res) = embassy_futures::join::join(
                        self.left.throttle_async(left),
                        self.right.throttle_async(right),
                    )
                    .await;
                    if let Err(e) = l_res {
                        defmt::warn!("DShot left send error: {}", e);
                    }
                    if let Err(e) = r_res {
                        defmt::warn!("DShot right send error: {}", e);
                    }
                    (None, None)
                }
            },
            _ => {
                // Mixed: handle each side independently
                let l_edt = if left == 0 {
                    self.left
                        .send_command_async(embassy_dshot::Command::MotorStop)
                        .await;
                    None
                } else if left_bidir {
                    self.left.read_extended_telemetry(left).await.ok()
                } else {
                    if let Err(e) = self.left.throttle_async(left).await {
                        defmt::warn!("DShot left send error: {}", e);
                    }
                    None
                };

                let r_edt = if right == 0 {
                    self.right
                        .send_command_async(embassy_dshot::Command::MotorStop)
                        .await;
                    None
                } else if right_bidir {
                    self.right.read_extended_telemetry(right).await.ok()
                } else {
                    if let Err(e) = self.right.throttle_async(right).await {
                        defmt::warn!("DShot right send error: {}", e);
                    }
                    None
                };

                (l_edt, r_edt)
            }
        }
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
        if let Some(t) = DSHOT_THROTTLE.try_take() {
            target = t;
        }

        // Governor PI converts eRPM targets to DShot values
        let left_dshot = left_state.governor.update(
            target.0,
            reading.left_erpm,
            left_state.bidir_enabled && reading.left_valid,
        );
        let right_dshot = right_state.governor.update(
            target.1,
            reading.right_erpm,
            right_state.bidir_enabled && reading.right_valid,
        );

        let (l_edt, r_edt) = engines
            .set_throttle(
                left_dshot,
                right_dshot,
                left_state.bidir_enabled,
                right_state.bidir_enabled,
            )
            .await;

        // Update left engine reading
        reading.left_throttle = left_dshot;
        reading.left_target_erpm = target.0;
        match l_edt {
            Some(ExtendedTelemetry::Erpm { erpm, .. }) => {
                reading.left_erpm = erpm;
                reading.left_valid = true;
                left_state.record_success();
            }
            Some(ref edt) => {
                // EDT frame (temp/voltage/current) — eRPM stays at last known value
                apply_edt_left(&mut reading, edt);
                left_state.record_success();
            }
            None if left_dshot == 0 => {
                reading.left_erpm = 0;
                reading.left_valid = true;
            }
            None if left_state.bidir_enabled => {
                left_state.record_failure();
            }
            None => {
                reading.left_valid = false;
            }
        }

        // Update right engine reading
        reading.right_throttle = right_dshot;
        reading.right_target_erpm = target.1;
        match r_edt {
            Some(ExtendedTelemetry::Erpm { erpm, .. }) => {
                reading.right_erpm = erpm;
                reading.right_valid = true;
                right_state.record_success();
            }
            Some(ref edt) => {
                apply_edt_right(&mut reading, edt);
                right_state.record_success();
            }
            None if right_dshot == 0 => {
                reading.right_erpm = 0;
                reading.right_valid = true;
            }
            None if right_state.bidir_enabled => {
                right_state.record_failure();
            }
            None => {
                reading.right_valid = false;
            }
        }

        ENGINE_CACHE.lock(|c| c.set(reading));
        ticker.next().await;
    }
}
