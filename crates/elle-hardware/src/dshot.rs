use embassy_dshot::rp::BidirDshotPio;
use embassy_rp::pio::Instance;
use embassy_time::Duration;

/// Wrapper around two bidirectional DShot ESCs (left and right engines).
pub struct DshotEngines<'a, PIO1: Instance, PIO2: Instance> {
    left: BidirDshotPio<'a, PIO1>,
    right: BidirDshotPio<'a, PIO2>,
}

impl<'a, PIO1: Instance, PIO2: Instance> DshotEngines<'a, PIO1, PIO2> {
    pub fn new(left: BidirDshotPio<'a, PIO1>, right: BidirDshotPio<'a, PIO2>) -> Self {
        Self { left, right }
    }

    /// Arm both ESCs by sending MotorStop at ~1kHz for the given duration.
    pub async fn arm(&mut self, duration: Duration) {
        // Arm concurrently
        embassy_futures::join::join(
            self.left.arm_async(duration),
            self.right.arm_async(duration),
        )
        .await;
    }

    /// Send throttle commands to both engines (DShot values 0-1999).
    pub async fn set_throttle(&mut self, left: u16, right: u16) {
        // Send concurrently (fire-and-forget, no telemetry read)
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

    /// Send MotorStop command to both engines.
    pub async fn stop(&mut self) {
        embassy_futures::join::join(
            self.left.send_command_async(embassy_dshot::Command::MotorStop),
            self.right.send_command_async(embassy_dshot::Command::MotorStop),
        )
        .await;
    }
}
