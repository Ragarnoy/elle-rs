use embassy_dshot::rp::BidirDshotPio;
use embassy_rp::peripherals::{PIO1, PIO2};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Ticker};

/// Throttle values (left, right) from the control loop, resent at ~1kHz by `dshot_task`.
pub static DSHOT_THROTTLE: Signal<CriticalSectionRawMutex, (u16, u16)> = Signal::new();

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

    /// Send throttle commands to both engines (DShot values 0-1999).
    ///
    /// Value 0 sends MotorStop (motors stay still).
    /// Values 1-1999 send throttle frames.
    /// Common cases (both idle or both active) are sent concurrently.
    async fn set_throttle(&mut self, left: u16, right: u16) {
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
            }
            (false, false) => {
                // Both active: send throttle concurrently
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
            }
            _ => {
                // Mixed (rare with differential thrust): send sequentially
                if left == 0 {
                    self.left
                        .send_command_async(embassy_dshot::Command::MotorStop)
                        .await;
                } else if let Err(e) = self.left.throttle_async(left).await {
                    defmt::warn!("DShot left send error: {}", e);
                }
                if right == 0 {
                    self.right
                        .send_command_async(embassy_dshot::Command::MotorStop)
                        .await;
                } else if let Err(e) = self.right.throttle_async(right).await {
                    defmt::warn!("DShot right send error: {}", e);
                }
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

    let mut current = (0u16, 0u16);
    let mut ticker = Ticker::every(Duration::from_millis(1));

    loop {
        if let Some(throttle) = DSHOT_THROTTLE.try_take() {
            current = throttle;
        }
        engines.set_throttle(current.0, current.1).await;
        ticker.next().await;
    }
}
