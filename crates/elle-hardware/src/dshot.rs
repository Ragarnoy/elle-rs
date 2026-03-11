use core::cell::Cell;

use elle_control::governor::RpmGovernor;
use embassy_dshot::rp::BidirDshotPio;
use embassy_rp::peripherals::{PIO1, PIO2};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Ticker};

/// Target eRPM per engine (left, right). Governor PI converts to DShot at 1kHz.
pub static DSHOT_THROTTLE: Signal<CriticalSectionRawMutex, (u32, u32)> = Signal::new();

/// Consecutive bidir telemetry failure threshold before falling back to fire-and-forget.
const BIDIR_FALLBACK_THRESHOLD: u16 = 100;

/// Engine RPM + throttle snapshot from the DShot task.
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
            defmt::warn!("DShot: bidir fallback triggered after {} failures, governor reset", self.fail_count);
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

    /// Arm both ESCs by sending MotorStop at ~1kHz for the given duration.
    async fn arm(&mut self, duration: Duration) {
        embassy_futures::join::join(
            self.left.arm_async(duration),
            self.right.arm_async(duration),
        )
        .await;
    }

    /// Send throttle commands to both engines and return telemetry results.
    ///
    /// Returns `(Option<u32>, Option<u32>)` — eRPM for each engine, or None on failure.
    /// When bidir is disabled for an engine, uses fire-and-forget (no telemetry).
    async fn set_throttle(
        &mut self,
        left: u16,
        right: u16,
        left_bidir: bool,
        right_bidir: bool,
    ) -> (Option<u32>, Option<u32>) {
        match (left == 0, right == 0) {
            (true, true) => {
                // Both idle: send MotorStop concurrently
                embassy_futures::join::join(
                    self.left
                        .send_command_async(embassy_dshot::Command::MotorStop),
                    self.right
                        .send_command_async(embassy_dshot::Command::MotorStop),
                )
                .await;
                (None, None)
            }
            (false, false) => {
                // Both active: choose bidir or fire-and-forget per engine
                match (left_bidir, right_bidir) {
                    (true, true) => {
                        let (l_res, r_res) = embassy_futures::join::join(
                            self.left.throttle_with_telemetry(left),
                            self.right.throttle_with_telemetry(right),
                        )
                        .await;
                        (
                            l_res.ok().map(|t| t.erpm),
                            r_res.ok().map(|t| t.erpm),
                        )
                    }
                    (true, false) => {
                        let (l_res, r_res) = embassy_futures::join::join(
                            self.left.throttle_with_telemetry(left),
                            self.right.throttle_async(right),
                        )
                        .await;
                        if let Err(e) = &r_res {
                            defmt::warn!("DShot right send error: {}", e);
                        }
                        (l_res.ok().map(|t| t.erpm), None)
                    }
                    (false, true) => {
                        let (l_res, r_res) = embassy_futures::join::join(
                            self.left.throttle_async(left),
                            self.right.throttle_with_telemetry(right),
                        )
                        .await;
                        if let Err(e) = &l_res {
                            defmt::warn!("DShot left send error: {}", e);
                        }
                        (None, r_res.ok().map(|t| t.erpm))
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
                }
            }
            _ => {
                // Mixed (rare with differential thrust): handle each side independently
                let l_erpm = if left == 0 {
                    self.left
                        .send_command_async(embassy_dshot::Command::MotorStop)
                        .await;
                    None
                } else if left_bidir {
                    self.left
                        .throttle_with_telemetry(left)
                        .await
                        .ok()
                        .map(|t| t.erpm)
                } else {
                    if let Err(e) = self.left.throttle_async(left).await {
                        defmt::warn!("DShot left send error: {}", e);
                    }
                    None
                };

                let r_erpm = if right == 0 {
                    self.right
                        .send_command_async(embassy_dshot::Command::MotorStop)
                        .await;
                    None
                } else if right_bidir {
                    self.right
                        .throttle_with_telemetry(right)
                        .await
                        .ok()
                        .map(|t| t.erpm)
                } else {
                    if let Err(e) = self.right.throttle_async(right).await {
                        defmt::warn!("DShot right send error: {}", e);
                    }
                    None
                };

                (l_erpm, r_erpm)
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

        let (l_erpm, r_erpm) = engines
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
        if let Some(erpm) = l_erpm {
            reading.left_erpm = erpm;
            reading.left_valid = true;
            left_state.record_success();
        } else if left_dshot == 0 {
            reading.left_erpm = 0;
            reading.left_valid = true;
        } else if left_state.bidir_enabled {
            // Transient failure: hold last erpm, mark via state
            left_state.record_failure();
        } else {
            reading.left_valid = false;
        }

        // Update right engine reading
        reading.right_throttle = right_dshot;
        reading.right_target_erpm = target.1;
        if let Some(erpm) = r_erpm {
            reading.right_erpm = erpm;
            reading.right_valid = true;
            right_state.record_success();
        } else if right_dshot == 0 {
            reading.right_erpm = 0;
            reading.right_valid = true;
        } else if right_state.bidir_enabled {
            right_state.record_failure();
        } else {
            reading.right_valid = false;
        }

        ENGINE_CACHE.lock(|c| c.set(reading));
        ticker.next().await;
    }
}
