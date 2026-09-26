//! Boot sequence shared by every airframe: wait for the IMU, pass the
//! supervisor start barrier, load saved PID / mag-cal / level-cal data, then run
//! the control loop for this build (flight or RPC).

use defmt::{info, warn};
use elle_control::autotune::SavedGains;
use elle_hardware::imu::{IMU_STATUS, LED_COMMAND_CHANNEL};
use elle_hardware::led::{LedPattern, colors};
use elle_hardware::pwm::PwmOutputs;
#[cfg(feature = "performance-monitoring")]
use elle_system::TimingMeasurement;
use elle_system::{FlightController, SUP_FC_READY, SUP_START_FC};
use embassy_rp::watchdog::Watchdog;
use embassy_time::{Duration, Timer};

/// Bring the flight controller up and run the control loop forever.
///
/// Call at the end of `main()`, after every task is spawned. `epoch_ms` is the
/// wall-clock time at boot (ms since the UNIX epoch), used to stamp ULog files.
///
/// Takes the elevon outputs rather than a built `FlightController` so the
/// controller is constructed here and lives in this future only.
pub async fn run(pwm: PwmOutputs<'static>, watchdog: Watchdog<'static>, epoch_ms: u64) -> ! {
    let mut fc = FlightController::new(pwm);

    // Wait for IMU to be ready
    info!("Core0: Waiting for IMU initialization...");
    let mut wait_counter = 0u32;
    loop {
        let status = IMU_STATUS.read().await;
        if status.initialized {
            info!("Core0: IMU ready! Calibrated: {}", status.calibrated);
            // Signal LED to show system ready
            let _ = LED_COMMAND_CHANNEL.try_send(if status.calibrated {
                LedPattern::Solid(colors::GREEN)
            } else {
                LedPattern::Pulse(colors::CYAN)
            });
            break;
        }
        drop(status);
        wait_counter += 1;
        if wait_counter.is_multiple_of(50) {
            // Log every 5 seconds
            warn!("Core0: Still waiting for IMU... ({}s)", wait_counter / 10);
        }
        Timer::after(Duration::from_millis(100)).await;
    }

    // Initialize supervisor components (but keep health monitoring disabled)
    info!("Core0: Initializing supervisor (watchdog only)");
    fc.initialize_supervisor(watchdog);

    // Signal supervisor that Flight Controller is ready and wait for start
    SUP_FC_READY.signal(());
    info!("Core0: Waiting for Supervisor start barrier");
    SUP_START_FC.wait().await;

    // Now enable health monitoring after all tasks are ready
    fc.enable_supervisor_monitoring();

    // Update LED based on IMU calibration status at start
    let status = IMU_STATUS.read().await;
    let _ = LED_COMMAND_CHANNEL.try_send(if status.calibrated {
        LedPattern::Solid(colors::GREEN)
    } else {
        LedPattern::Pulse(colors::CYAN)
    });
    drop(status);

    // Boot-time PID profile load from flash
    if elle_config::IGNORE_PID_FLASH {
        info!("Core0: IGNORE_PID_FLASH — using firmware PID defaults");
    } else {
        use elle_config::profile::{FlashRequest, FlashResponse};
        use elle_hardware::flash::{FLASH_REQUEST_SIGNAL, FLASH_RESPONSE_SIGNAL};

        info!("Core0: Loading PID profile from flash");
        FLASH_REQUEST_SIGNAL.signal(FlashRequest::LoadPidProfile);
        let load_timeout = Timer::after(Duration::from_secs(2));
        match embassy_futures::select::select(FLASH_RESPONSE_SIGNAL.wait(), load_timeout).await {
            embassy_futures::select::Either::First(FlashResponse::PidProfileLoaded { data }) => {
                if let Some(gains) = SavedGains::from_bytes(&data) {
                    fc.apply_saved_gains(&gains);
                    info!(
                        "Core0: PID loaded P({}/{}/{}) R({}/{}/{}) s={} il={}",
                        (gains.pitch_kp * 1000.0) as i32,
                        (gains.pitch_ki * 1000.0) as i32,
                        (gains.pitch_kd * 1000.0) as i32,
                        (gains.roll_kp * 1000.0) as i32,
                        (gains.roll_ki * 1000.0) as i32,
                        (gains.roll_kd * 1000.0) as i32,
                        (gains.scale * 10000.0) as i32,
                        (gains.i_limit * 10.0) as i32,
                    );
                    elle_hardware::elle_event!(
                        info,
                        elle_hardware::event::EVT_PID_LOADED,
                        "PID profile loaded from flash"
                    );
                } else {
                    warn!("PID profile: invalid data in flash, using defaults");
                }
            }
            embassy_futures::select::Either::First(FlashResponse::PidProfileEmpty) => {
                info!("Core0: No PID profile saved, using defaults");
                elle_hardware::elle_event!(
                    info,
                    elle_hardware::event::EVT_PID_LOAD_EMPTY,
                    "No PID profile in flash"
                );
            }
            embassy_futures::select::Either::First(_) => {
                warn!("Core0: Unexpected flash response for PID load");
            }
            embassy_futures::select::Either::Second(_) => {
                warn!("Core0: PID profile load timeout, using defaults");
            }
        }
    }

    // Boot-time mag cal load from flash
    {
        use elle_config::profile::{FlashRequest, FlashResponse};
        use elle_hardware::flash::{FLASH_REQUEST_SIGNAL, FLASH_RESPONSE_SIGNAL};

        info!("Core0: Loading mag calibration from flash");
        FLASH_REQUEST_SIGNAL.signal(FlashRequest::LoadMagCal);
        let load_timeout = Timer::after(Duration::from_secs(2));
        match embassy_futures::select::select(FLASH_RESPONSE_SIGNAL.wait(), load_timeout).await {
            embassy_futures::select::Either::First(FlashResponse::MagCalLoaded { data }) => {
                let [ox, oy, oz]: [f32; 3] = bytemuck::pod_read_unaligned(&data);

                // Validate: finite and reasonable range (raw counts, max ~65535)
                if ox.is_finite()
                    && oy.is_finite()
                    && oz.is_finite()
                    && ox.abs() < 100_000.0
                    && oy.abs() < 100_000.0
                    && oz.abs() < 100_000.0
                    && (ox != 0.0 || oy != 0.0 || oz != 0.0)
                {
                    elle_hardware::imu::MAG_CALIBRATION_SIGNAL.signal((ox, oy, oz));
                    info!(
                        "Core0: Mag cal loaded ({}, {}, {})",
                        ox as i32, oy as i32, oz as i32
                    );
                    #[cfg(feature = "rpc-control")]
                    {
                        crate::rpc_app::MAG_CAL_OFFSET.lock(|c| c.set((ox, oy, oz)));
                        crate::rpc_app::MAG_CAL_STATUS
                            .store(2, core::sync::atomic::Ordering::Relaxed);
                    }
                    elle_hardware::elle_event!(
                        info,
                        elle_hardware::event::EVT_MAG_CAL_LOADED,
                        "Mag cal loaded from flash"
                    );
                } else {
                    warn!("Mag cal: invalid data in flash, ignoring");
                }
            }
            embassy_futures::select::Either::First(FlashResponse::MagCalEmpty) => {
                info!("Core0: No mag cal saved, using zero offsets");
                elle_hardware::elle_event!(
                    info,
                    elle_hardware::event::EVT_MAG_CAL_LOAD_EMPTY,
                    "No mag cal in flash"
                );
            }
            embassy_futures::select::Either::First(_) => {
                warn!("Core0: Unexpected flash response for mag cal load");
            }
            embassy_futures::select::Either::Second(_) => {
                warn!("Core0: Mag cal load timeout, using zero offsets");
            }
        }
    }

    // Boot-time level cal (IMU mounting offset) load from flash
    elle_hardware::imu::level_cal::load_from_flash().await;

    info!("Core0: Starting main control loop");

    // Test timing precision (only when performance monitoring is enabled)
    #[cfg(feature = "performance-monitoring")]
    {
        let test_timer = TimingMeasurement::start();
        let mut test_var = 0u32;
        for _ in 0..1000 {
            test_var = test_var.wrapping_add(1);
        }
        info!(
            "Timing test: {}μs for 1000 additions (result: {})",
            test_timer.elapsed_us(),
            test_var
        );
    }

    #[cfg(not(feature = "rpc-control"))]
    crate::flight::run_flight(&mut fc, epoch_ms).await;
    #[cfg(feature = "rpc-control")]
    crate::rpc::run_rpc(&mut fc, epoch_ms).await
}
