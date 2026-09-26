#![no_std]
#![no_main]
#![allow(clippy::too_many_arguments)] // embassy task macros generate wrapper fns

//! Firmware for the Elle Dart single-engine flying wing (RP2350, Embassy async)

// Feature gate guards
#[cfg(all(feature = "defmt-logging", feature = "rpc-control"))]
compile_error!("defmt-logging and rpc-control are mutually exclusive (both define _SEGGER_RTT)");

#[cfg(feature = "defmt-logging")]
use defmt_rtt as _;

use defmt::{info, warn};
use elle_config::profile::FLASH_SIZE;
#[allow(unused_imports)]
use elle_config::{
    CONTROL_LOOP_FREQUENCY_HZ, CONTROL_LOOP_PERIOD_MS, IMU_CALIBRATION_TIMEOUT_S, IMU_I2C_FREQ,
    IMU_MAX_AGE_MS, LED_UPDATE_INTERVAL, PERF_LOG_INTERVAL, STALE_EVENT_DRAIN_DIVISOR,
    ULOG_BARO_DIVISOR, ULOG_MAG_DIVISOR, ULOG_STATUS_DIVISOR,
};

#[cfg(not(feature = "rpc-control"))]
use defmt::debug;
#[cfg(not(feature = "rpc-control"))]
use elle_config::{ATTITUDE_ENABLE_CH, PITCH_CH, ROLL_CH, THROTTLE_CH, YAW_CH};
use elle_control::autotune::{AutotuneAction, AutotuneAxis, Autotuner, SavedGains};
#[cfg(not(feature = "rpc-control"))]
use elle_control::commands::PilotCommands;
use elle_hardware::imu::{
    ATTITUDE, AttitudeData, IMU_STATUS, Imu, LED_COMMAND_CHANNEL, is_attitude_valid,
};
use elle_hardware::imu::{BARO, MAG};
use elle_hardware::led::{LedPattern, StatusLed, colors};
use elle_hardware::{
    dshot::{DSHOT_THROTTLE, dshot_single_task},
    flash::SequentialFlashManager,
    pwm::{PwmOutputs, PwmPins},
};

use elle_hardware::ULogLogger;

use elle_hardware::crsf::{CrsfReceiver, RC_COMMANDS, crsf_receiver_task, crsf_uart_config};

#[cfg(feature = "rpc-control")]
use elle_rpc_icd::ControlMode;
#[cfg(feature = "rpc-control")]
use elle_system::rpc::init_rtt_rpc;
#[cfg(feature = "rpc-control")]
pub mod flight_state;
#[cfg(feature = "rpc-control")]
mod rpc_app;

use elle_system::{
    FlightController, SUP_FC_READY, SUP_IMU_READY, SUP_LED_READY, SUP_START_FC, SUP_START_IMU,
    TimingMeasurement, log_performance_summary, supervisor_task, update_control_loop_timing,
    update_led_timing, update_ulog_timing,
};
use embassy_executor::Spawner;
use embassy_rp::aon_timer::{AlarmWakeMode, AonTimer, ClockSource, Config as AonConfig};
use embassy_rp::clocks::{ClockConfig, CoreVoltage};
use embassy_rp::executor::Executor;
use embassy_rp::flash::Flash;
use embassy_rp::i2c::{Config, I2c};
use embassy_rp::mode::Async;
use embassy_rp::multicore::{Stack, spawn_core1};
use embassy_rp::peripherals::{
    DMA_CH2, I2C0, PIN_0, PIN_1, PIN_2, PIN_3, PIN_5, PIN_8, PIN_9, PIN_10, PIO0, PIO1, SPI0,
    UART0, UART1,
};
use embassy_rp::pio::{InterruptHandler as PioIrqHandler, Pio};
use embassy_rp::uart::BufferedInterruptHandler as BufferedUartIrqHandler;
#[cfg(feature = "gnss")]
use embassy_rp::uart::BufferedUart;
use embassy_rp::uart::InterruptHandler as UartIrqHandler;
use embassy_rp::watchdog::Watchdog;
use embassy_rp::{Peri, bind_interrupts};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Receiver;
use embassy_time::{Duration, Instant, Ticker, Timer};
use panic_probe as _;
use static_cell::StaticCell;

/// Helper to validate attitude data and return only if fresh
#[inline]
fn validate_attitude(attitude: Option<AttitudeData>) -> Option<AttitudeData> {
    attitude.filter(|att| is_attitude_valid(att, Duration::from_millis(IMU_MAX_AGE_MS)))
}

/// Gate for the double-tap mag-cal gesture: only start calibration when
/// throttle is commanded low and the gyro is quiet, so motor vibration or
/// handling can't trigger a spurious calibration.
fn tap_cal_allowed(
    commands: Option<&elle_control::commands::PilotCommands>,
    attitude: Option<&AttitudeData>,
) -> bool {
    use elle_control::commands::PilotCommands;
    let throttle_low = commands.is_some_and(|cmd| match cmd {
        PilotCommands::Raw(raw) => {
            raw.channels[elle_config::THROTTLE_CH] < elle_config::TAP_CAL_THROTTLE_MAX_RAW
        }
        PilotCommands::Normalized(n) => n.throttle < elle_config::TAP_CAL_THROTTLE_MAX_NORM,
    });
    let gyro_quiet = attitude.is_some_and(|a| {
        a.pitch_rate.abs() < elle_config::TAP_CAL_MAX_GYRO_RAD_S
            && a.roll_rate.abs() < elle_config::TAP_CAL_MAX_GYRO_RAD_S
            && a.yaw_rate.abs() < elle_config::TAP_CAL_MAX_GYRO_RAD_S
    });
    throttle_low && gyro_quiet
}

/// Whether a calibration gesture means *level* cal rather than mag cal: the CH7
/// autotune switch is out of its off position. Safe to reuse because autotune only
/// starts while armed, and only on an off → on transition.
fn tap_selects_level_cal(commands: Option<&elle_control::commands::PilotCommands>) -> bool {
    commands.is_some_and(|cmd| match cmd {
        elle_control::commands::PilotCommands::Raw(raw) => {
            raw.channels[elle_config::AUTOTUNE_CH] >= elle_config::AUTOTUNE_OFF_THRESHOLD
        }
        elle_control::commands::PilotCommands::Normalized(_) => false,
    })
}

/// Save PID gains to flash with timeout. Returns true on success.
async fn save_pid_to_flash(data: [u8; 32], context: &str) -> bool {
    if elle_config::IGNORE_PID_FLASH {
        warn!("PID flash ignored — skip save ({})", context);
        return false;
    }

    use elle_config::profile::{FlashRequest, FlashResponse};
    use elle_hardware::flash::{FLASH_REQUEST_SIGNAL, FLASH_RESPONSE_SIGNAL};

    FLASH_REQUEST_SIGNAL.signal(FlashRequest::SavePidProfile { data });
    let save_timeout = Timer::after(Duration::from_secs(5));
    match embassy_futures::select::select(FLASH_RESPONSE_SIGNAL.wait(), save_timeout).await {
        embassy_futures::select::Either::First(FlashResponse::PidProfileSaved) => {
            elle_hardware::elle_event!(
                info,
                elle_hardware::event::EVT_PID_SAVED,
                "PID gains saved to flash ({})",
                context
            );
            true
        }
        _ => {
            elle_hardware::elle_event!(
                warn,
                elle_hardware::event::EVT_PID_SAVE_FAILED,
                "PID gains flash save failed ({})",
                context
            );
            false
        }
    }
}

/// Erase the PID profile map entry. No-op while `IGNORE_PID_FLASH` is set —
/// the erase is a 64KB sector wipe (shared with mag cal) and has crashed the MCU.
#[cfg(feature = "rpc-control")]
async fn erase_pid_from_flash() {
    use elle_config::profile::{FlashRequest, FlashResponse};
    use elle_hardware::flash::{FLASH_REQUEST_SIGNAL, FLASH_RESPONSE_SIGNAL};

    if elle_config::IGNORE_PID_FLASH {
        info!("PID flash ignored — skip erase");
        return;
    }

    FLASH_REQUEST_SIGNAL.signal(FlashRequest::ErasePidProfile);
    let timeout = Timer::after(Duration::from_secs(5));
    match embassy_futures::select::select(FLASH_RESPONSE_SIGNAL.wait(), timeout).await {
        embassy_futures::select::Either::First(FlashResponse::PidProfileErased) => {
            info!("PID profile erased from flash");
        }
        _ => {
            warn!("PID profile erase failed or timed out");
        }
    }
}

/// Log flight data to ULog flash storage
///
/// Logs attitude, commands, and periodic status updates at appropriate rates:
/// - Attitude: control-loop rate (every call)
/// - Commands: control-loop rate (every call)
/// - Status: 7.7Hz (every 10th call)
fn log_flight_data(
    logger: &mut ULogLogger,
    attitude: Option<&AttitudeData>,
    commands: &elle_control::commands::PilotCommands,
    loop_counter: u32,
    loop_timer_us: u32,
    fc: &FlightController<'_>,
) {
    use elle_control::commands::PilotCommands;

    // Measure ULog logging performance
    let ulog_timer = TimingMeasurement::start();

    // Log attitude data at control-loop rate
    if let Some(att) = attitude {
        let _ = logger.log_attitude(
            att.pitch,
            att.roll,
            att.yaw,
            att.pitch_rate,
            att.roll_rate,
            att.yaw_rate,
        );
    }

    // Log commands at control-loop rate
    // Convert to normalized for consistent logging
    let last_out = fc.last_output();
    let log_cmd = |logger: &mut ULogLogger, norm: &elle_control::commands::NormalizedCommands| {
        let _ = logger.log_commands(
            norm.throttle,
            norm.pitch,
            norm.roll,
            norm.yaw,
            norm.attitude_mode as u8,
            last_out.pitch_setpoint_deg,
            last_out.roll_setpoint_deg,
            last_out.pitch_correction,
            last_out.roll_correction,
            last_out.elevon_left_us,
            last_out.elevon_right_us,
        );
    };
    match commands {
        PilotCommands::Normalized(norm) => log_cmd(logger, norm),
        PilotCommands::Raw(raw) => log_cmd(logger, &raw.to_normalized()),
    }

    // Log engine data at control-loop rate
    {
        let eng = elle_hardware::dshot::ENGINE_CACHE.lock(|c| c.get());
        let _ = logger.log_engine(&eng);
    }

    // Log status at reduced rate (~7.7Hz)
    if loop_counter.is_multiple_of(ULOG_STATUS_DIVISOR) {
        let imu_status = IMU_STATUS.try_read();
        let _ = logger.log_status(
            loop_timer_us,
            imu_status.as_ref().map(|s| s.error_count).unwrap_or(0),
            imu_status.as_ref().map(|s| s.calibrated).unwrap_or(false),
            fc.is_armed(),
            0.0, // CPU load - could calculate from timing data
            fc.rc_signal_age_ms(),
        );
    }

    // Log barometer at ~2Hz
    if loop_counter.is_multiple_of(ULOG_BARO_DIVISOR) {
        let baro = BARO.read_cached();
        let _ = logger.log_barometer(
            baro.pressure_hpa,
            baro.temperature_c,
            baro.altitude_m,
            baro.vario_ms,
        );
    }

    // Log magnetometer at ~10Hz
    if loop_counter.is_multiple_of(ULOG_MAG_DIVISOR) {
        let mag = MAG.read_cached();
        let _ = logger.log_magnetometer(mag.x as f32, mag.y as f32, mag.z as f32);
    }

    // Log GNSS at ~1Hz — feature-gated
    #[cfg(feature = "gnss")]
    if loop_counter.is_multiple_of(elle_config::ULOG_GNSS_DIVISOR)
        && let Some(gnss) = elle_hardware::gnss::GNSS_SIGNAL.try_take()
    {
        elle_hardware::gnss::GNSS_SIGNAL.signal(gnss); // put back for other readers
        let _ = logger.log_gnss(&gnss);
    }

    // Drain event channel into ULog
    while let Ok((level, code)) = elle_hardware::event::ULOG_EVENT_CHANNEL.try_receive() {
        let _ = logger.log_event(level, code);
    }

    // Update performance monitoring
    update_ulog_timing(ulog_timer.elapsed_us());
}

bind_interrupts!(
    struct Irqs {
        PIO0_IRQ_0 => PioIrqHandler<PIO0>;
        PIO1_IRQ_0 => PioIrqHandler<PIO1>;
        UART0_IRQ => BufferedUartIrqHandler<UART0>;
        UART1_IRQ => UartIrqHandler<UART1>;
        POWMAN_IRQ_TIMER => embassy_rp::aon_timer::InterruptHandler;
        DMA_IRQ_0 => embassy_rp::dma::InterruptHandler<embassy_rp::peripherals::DMA_CH0>,
            embassy_rp::dma::InterruptHandler<embassy_rp::peripherals::DMA_CH1>,
            embassy_rp::dma::InterruptHandler<embassy_rp::peripherals::DMA_CH2>,
            embassy_rp::dma::InterruptHandler<embassy_rp::peripherals::DMA_CH3>,
            embassy_rp::dma::InterruptHandler<embassy_rp::peripherals::DMA_CH4>,
            embassy_rp::dma::InterruptHandler<embassy_rp::peripherals::DMA_CH5>,
            embassy_rp::dma::InterruptHandler<embassy_rp::peripherals::DMA_CH6>,
            embassy_rp::dma::InterruptHandler<embassy_rp::peripherals::DMA_CH7>;
    }
);

static mut CORE1_STACK: Stack<16384> = Stack::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();
static AON_TIMER: StaticCell<AonTimer<'static>> = StaticCell::new();

// RC channel signal for RPC handler
#[cfg(feature = "rpc-control")]
pub mod rc_signal {
    use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
    use embassy_sync::signal::Signal;

    pub static RC_SIGNAL: Signal<CriticalSectionRawMutex, [u16; 16]> = Signal::new();
}

#[cfg(feature = "rpc-control")]
mod rpc_handlers;

#[embassy_executor::main(executor = "Executor", entry = "cortex_m_rt::entry")]
async fn main(spawner: Spawner) {
    let mut config =
        embassy_rp::config::Config::new(ClockConfig::system_freq(200_000_000).unwrap());
    config.clocks.core_voltage = CoreVoltage::V1_15;

    let p = embassy_rp::init(config);

    // Initialize AON timer with compile-time UNIX epoch for wall-clock reference
    let epoch_ms = compile_time::unix!() * 1000;
    let aon_config = AonConfig {
        clock_source: ClockSource::Xosc,
        clock_freq_khz: 12_000,
        alarm_wake_mode: AlarmWakeMode::Disabled,
    };
    let mut aon = AonTimer::new(p.POWMAN, Irqs, aon_config);
    aon.set_counter(epoch_ms);
    aon.start();
    info!("AON: seeded with epoch {}ms", epoch_ms);
    #[allow(unused_variables)]
    let aon_ref = AON_TIMER.init(aon);

    info!("Core0: Starting flash manager");
    // Create flash manager on Core 0 before spawning Core 1
    let flash = embassy_rp::flash::Flash::<Async, { FLASH_SIZE }>::new(p.FLASH, p.DMA_CH1, Irqs);
    // Small delay to let debug probe settle
    Timer::after_millis(10).await;
    spawner.spawn(flash_manager_task(flash).unwrap());

    // SD card writer on SPI1 with DMA (async)
    info!("Core0: Starting SD card writer");
    {
        let mut sd_spi_config = embassy_rp::spi::Config::default();
        sd_spi_config.frequency = embassy_rp::time::Hertz::khz(400); // 400kHz for SD card init
        let sd_spi = embassy_rp::spi::Spi::new(
            p.SPI1,
            p.PIN_26,
            p.PIN_27,
            p.PIN_24,
            p.DMA_CH5,
            p.DMA_CH6,
            Irqs,
            sd_spi_config,
        )
        .unwrap();
        let sd_cs = embassy_rp::gpio::Output::new(p.PIN_25, embassy_rp::gpio::Level::High);
        let sd_detect = embassy_rp::gpio::Input::new(p.PIN_23, embassy_rp::gpio::Pull::Up);
        spawner.spawn(
            elle_hardware::sd_writer::sd_writer_task(sd_spi, sd_cs, sd_detect, epoch_ms).unwrap(),
        );
    }

    // Elevons on hardware PWM slice 6; WS2812B LED on PIO0 SM2
    info!("Core0: Setting up flight control hardware + LED on PIO0");
    let mut pwm = PwmOutputs::new(PwmPins {
        slice: p.PWM_SLICE6,
        elevon_left: p.PIN_13,
        elevon_right: p.PIN_12,
    });
    pwm.set_safe_positions();

    let Pio {
        common, sm2: led_sm, ..
    } = Pio::new(p.PIO0, Irqs);

    spawner.spawn(led_task(common, led_sm, p.DMA_CH2, Irqs, p.PIN_10).unwrap());

    // Setup DShot300 single engine on PIO1 (PIN_14)
    info!("Core0: Setting up DShot300 single engine");
    // The program is only borrowed while the driver is built; the block's `Common`
    // and unused state machines drop here, as they did inside the pre-0.5 constructor.
    let Pio {
        mut common, sm0, ..
    } = Pio::new(p.PIO1, Irqs);
    let prog = embassy_dshot::rp::BidirDshotProgram::new(&mut common);
    let engine = embassy_dshot::rp::BidirDshotPio::new(
        sm0,
        &mut common,
        p.PIN_14,
        &prog,
        embassy_dshot::rp::DshotSpeed::DShot300,
    );
    spawner.spawn(dshot_single_task(engine).unwrap());

    #[cfg(feature = "rpc-control")]
    {
        info!("Core0: Starting RPC server");
        spawner.spawn(rpc_server_task(spawner, aon_ref).unwrap());
    }

    // Start supervisor to coordinate task startup
    info!("Core0: Spawning Supervisor");
    spawner.spawn(supervisor_task().unwrap());

    info!("Core0: Spawning Core1 for IMU");
    // Spawn IMU task on core1
    spawn_core1(
        p.CORE1,
        unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
        move || {
            let executor1 = EXECUTOR1.init(Executor::new());
            executor1.run(|spawner| {
                spawner.spawn(
                    imu_task(
                        spawner, p.I2C0, p.PIN_8, p.PIN_9, p.SPI0, p.PIN_0, p.PIN_1, p.PIN_2,
                        p.PIN_3, p.PIN_5,
                    )
                    .unwrap(),
                );
            })
        },
    );

    // CRSF receiver on UART1. DMA_CH3 unconditionally: the old cfg split that
    // reserved DMA_CH0 for GNSS is obsolete now that the GNSS UART is
    // interrupt-buffered and uses no DMA channel at all.
    {
        info!("Core0: Starting CRSF receiver task (UART1, GPIO21)");
        let config = crsf_uart_config();

        // Split UART1: RX for CRSF receiver, TX for telemetry
        let uart = embassy_rp::uart::Uart::new(
            p.UART1, p.PIN_20, p.PIN_21, p.DMA_CH4, p.DMA_CH3, Irqs, config,
        );
        let (tx, rx) = uart.split();
        let crsf = CrsfReceiver::new(rx);
        spawner.spawn(crsf_receiver_task(crsf).unwrap());

        info!("Core0: Starting CRSF telemetry TX task (PIN_20, DMA_CH4)");
        spawner.spawn(elle_hardware::crsf::crsf_telemetry_task(tx).unwrap());
    }

    #[cfg(feature = "gnss")]
    {
        info!("Core0: Starting GNSS task (UART0, GPIO28/29)");
        // BufferedUart, not the DMA Uart: only the interrupt-buffered variant
        // implements embedded-io-async, and its partial reads suit a bursty
        // GNSS stream, and frees DMA_CH0 and DMA_CH7.
        static GNSS_TX_BUF: StaticCell<[u8; 256]> = StaticCell::new();
        static GNSS_RX_BUF: StaticCell<[u8; 512]> = StaticCell::new();
        let mut gnss_config = embassy_rp::uart::Config::default();
        gnss_config.baudrate = elle_hardware::gnss::DEFAULT_BAUD;
        let gnss_uart = BufferedUart::new(
            p.UART0,
            p.PIN_28,
            p.PIN_29,
            Irqs,
            GNSS_TX_BUF.init([0; 256]),
            GNSS_RX_BUF.init([0; 512]),
            gnss_config,
        );
        spawner.spawn(elle_hardware::gnss::gnss_task(gnss_uart).unwrap());
    }
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
    let watchdog = Watchdog::new(p.WATCHDOG);
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

    // ULog logger created but NOT initialized — user starts recording via `ulog start`
    let mut ulog_logger = ULogLogger::new();

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
                // Deserialize 3x f32 from 12 bytes (little-endian)
                let ox = f32::from_le_bytes([data[0], data[1], data[2], data[3]]);
                let oy = f32::from_le_bytes([data[4], data[5], data[6], data[7]]);
                let oz = f32::from_le_bytes([data[8], data[9], data[10], data[11]]);

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
                        rpc_app::MAG_CAL_OFFSET.lock(|c| c.set((ox, oy, oz)));
                        rpc_app::MAG_CAL_STATUS.store(2, core::sync::atomic::Ordering::Relaxed);
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

    // State for control loop
    let mut loop_counter = 0u32;

    #[cfg(not(feature = "rpc-control"))]
    {
        info!("FLIGHT MODE - CRSF/ELRS Control");

        let mut ulog_recording = false;

        // Autotune state
        let mut autotuner = Autotuner::new();
        let mut autotune_tick: u32 = 0;
        let mut autotune_debounce_pos: u8 = 0; // 0=Off, 1=Pitch, 2=Roll
        let mut autotune_debounce_count: u32 = 0;
        let mut autotune_stable_pos: u8 = 0;
        let mut pitch_tune_done: bool = false;
        let mut save_pending: Option<[u8; 32]> = None;
        // Autotune display for CRSF telemetry (DONE/ERR shown for ~3s then reverts to Off)
        let mut autotune_display = elle_hardware::crsf::AutotuneDisplay::Off;
        let mut autotune_display_timer: u32 = 0;
        const AUTOTUNE_DISPLAY_DURATION: u32 = CONTROL_LOOP_FREQUENCY_HZ * 3; // ~3 s

        // Heading-hold state (CH5, 2-pos switch, modifier active only while Stabilized)
        let mut heading_hold_switch_debounced: bool = false;
        let mut heading_hold_debounce_count: u32 = 0;
        let mut heading_hold_effective_prev: bool = false;

        // Mag calibration collection in progress (double-tap gesture)
        let mut mag_cal_collecting: bool = false;

        // Create ticker for the control loop period (CONTROL_LOOP_PERIOD_MS)
        let mut ticker = Ticker::every(Duration::from_millis(CONTROL_LOOP_PERIOD_MS));

        // Track last commands for consistent update rate
        let mut last_commands: Option<PilotCommands> = None;
        let mut was_armed = false;
        let mut was_killed = false;

        loop {
            ticker.next().await; // Wait for next tick BEFORE processing
            let loop_start = Instant::now();
            let loop_timer = TimingMeasurement::start();

            // Supervisor check - monitor core health and kick watchdog
            let _supervisor_healthy = fc.supervisor_check();

            // Get latest attitude data (non-blocking)
            let attitude = ATTITUDE.try_take();

            // Check for latest RC commands from dedicated CRSF receiver task (non-blocking)
            if let Some(commands) = RC_COMMANDS.try_take() {
                // Debug logging (~8Hz)
                if loop_counter.is_multiple_of(CONTROL_LOOP_FREQUENCY_HZ / 10)
                    && let PilotCommands::Raw(raw) = &commands
                {
                    debug!(
                        "RC: CH1:{} CH2:{} CH3:{} CH4:{} CH5:{}",
                        raw.channels[ROLL_CH],
                        raw.channels[PITCH_CH],
                        raw.channels[THROTTLE_CH],
                        raw.channels[YAW_CH],
                        raw.channels[ATTITUDE_ENABLE_CH],
                    );
                }

                fc.note_rc_packet(commands.timestamp());
                last_commands = Some(commands);
            }

            // Always update flight controller every tick for consistent PID timing
            // Use last known commands if no new packet arrived this iteration
            // Kill switch: CH8 high = disarm, block fc.update() to prevent re-arm
            let kill_active = last_commands.as_ref().is_some_and(|cmd| {
                if let PilotCommands::Raw(raw) = cmd {
                    raw.channels[elle_config::KILL_SWITCH_CH] > elle_config::KILL_SWITCH_THRESHOLD
                } else {
                    false
                }
            });

            if kill_active != was_killed {
                if kill_active {
                    elle_hardware::elle_event!(
                        warn,
                        elle_hardware::event::EVT_KILL_ENGAGED,
                        "Kill switch engaged"
                    );
                } else {
                    elle_hardware::elle_event!(
                        info,
                        elle_hardware::event::EVT_KILL_RELEASED,
                        "Kill switch released"
                    );
                }
                was_killed = kill_active;
            }

            if kill_active {
                // Kill switch active: disarm, center elevons, zero throttle
                if fc.is_armed() {
                    fc.disarm();
                }
                fc.set_safe_positions();
                DSHOT_THROTTLE.signal((0, 0));
            }

            // Double-tap mag-cal gesture — only while the kill switch is active
            // (deliberate service state; motors locked out) so handling bumps
            // can't start a calibration. Drain the signal every iteration so a
            // tap detected outside kill mode can't stay latched and fire later.
            let tapped = elle_hardware::imu::TAP_SIGNAL.try_take().is_some();
            if tapped
                && !fc.is_armed()
                && kill_active
                && !mag_cal_collecting
                && !elle_hardware::imu::level_cal::is_collecting()
            {
                if !tap_cal_allowed(last_commands.as_ref(), attitude.as_ref()) {
                    info!("Double-tap ignored: throttle not low or gyro not quiet");
                } else if tap_selects_level_cal(last_commands.as_ref()) {
                    elle_hardware::imu::level_cal::start();
                } else {
                    elle_hardware::imu::MAG_CAL_START_SIGNAL.signal(());
                    mag_cal_collecting = true;
                    elle_hardware::elle_event!(
                        info,
                        elle_hardware::event::EVT_MAG_CAL_STARTED,
                        "Double-tap: mag calibration started"
                    );
                }
            }

            // Poll level calibration result from IMU task (tap-triggered collection)
            elle_hardware::imu::level_cal::poll_result().await;

            // Poll mag calibration result from IMU task (tap-triggered collection)
            if let Some(result) = elle_hardware::imu::MAG_CAL_RESULT_SIGNAL.try_take() {
                mag_cal_collecting = false;
                match result {
                    Some((ox, oy, oz)) => {
                        use elle_config::profile::{FlashRequest, FlashResponse};
                        use elle_hardware::flash::{FLASH_REQUEST_SIGNAL, FLASH_RESPONSE_SIGNAL};
                        let mut data = [0u8; 12];
                        data[0..4].copy_from_slice(&ox.to_le_bytes());
                        data[4..8].copy_from_slice(&oy.to_le_bytes());
                        data[8..12].copy_from_slice(&oz.to_le_bytes());
                        FLASH_REQUEST_SIGNAL.signal(FlashRequest::SaveMagCal { data });
                        let save_timeout = Timer::after(Duration::from_secs(5));
                        match embassy_futures::select::select(
                            FLASH_RESPONSE_SIGNAL.wait(),
                            save_timeout,
                        )
                        .await
                        {
                            embassy_futures::select::Either::First(FlashResponse::MagCalSaved) => {
                                elle_hardware::elle_event!(
                                    info,
                                    elle_hardware::event::EVT_MAG_CAL_SAVED,
                                    "Mag cal saved to flash"
                                );
                            }
                            _ => {
                                elle_hardware::elle_event!(
                                    warn,
                                    elle_hardware::event::EVT_MAG_CAL_FAILED,
                                    "Mag cal flash save failed"
                                );
                            }
                        }
                        elle_hardware::elle_event!(
                            info,
                            elle_hardware::event::EVT_MAG_CAL_COMPLETE,
                            "Mag calibration complete"
                        );
                    }
                    None => {
                        elle_hardware::elle_event!(
                            warn,
                            elle_hardware::event::EVT_MAG_CAL_FAILED,
                            "Mag calibration failed (insufficient rotation)"
                        );
                    }
                }
            }

            if let Some(commands) = &last_commands {
                // Update with validated attitude (warns if stale)
                let valid_attitude = validate_attitude(attitude);
                if valid_attitude.is_none() && attitude.is_some() {
                    elle_hardware::elle_event!(
                        warn,
                        elle_hardware::event::EVT_ATTITUDE_STALE,
                        "Stale attitude data, using manual control only"
                    );
                }
                if !kill_active {
                    fc.update(commands, valid_attitude.as_ref());
                }

                // Detect arm/disarm transitions → beep + event
                let now_armed = fc.is_armed();
                if now_armed && !was_armed {
                    elle_hardware::dshot::BEEP_SIGNAL
                        .signal(elle_hardware::dshot::BeepPattern::ArmBeep);
                    elle_hardware::elle_event!(
                        info,
                        elle_hardware::event::EVT_MOTORS_ARMED,
                        "Motors armed"
                    );
                } else if !now_armed && was_armed {
                    elle_hardware::dshot::BEEP_SIGNAL
                        .signal(elle_hardware::dshot::BeepPattern::DisarmBeep);
                    elle_hardware::elle_event!(
                        warn,
                        elle_hardware::event::EVT_MOTORS_DISARMED,
                        "Motors disarmed"
                    );
                }
                was_armed = now_armed;

                // Send engine commands via DShot (governor converts eRPM target to DShot)
                if !kill_active {
                    let (engine_l, _) = fc.engine_output();
                    let l_erpm = (engine_l as u32 * elle_config::MAX_ERPM)
                        / elle_config::DSHOT_THROTTLE_MAX as u32;
                    DSHOT_THROTTLE.signal((l_erpm, 0));
                }

                // Update autotune display state for CRSF telemetry
                if autotuner.is_active() {
                    autotune_display = if autotune_stable_pos == 1 {
                        elle_hardware::crsf::AutotuneDisplay::Pitch
                    } else {
                        elle_hardware::crsf::AutotuneDisplay::Roll
                    };
                } else if autotune_display_timer > 0 {
                    autotune_display_timer -= 1;
                    if autotune_display_timer == 0 {
                        autotune_display = elle_hardware::crsf::AutotuneDisplay::Off;
                    }
                }

                elle_hardware::crsf::CRSF_FLIGHT_MODE.signal(elle_hardware::crsf::CrsfFlightMode {
                    armed: fc.is_armed(),
                    failsafe: fc.is_failsafe(),
                    mode: match fc.current_control_mode() {
                        elle_system::ControlMode::Manual => {
                            elle_hardware::crsf::CrsfControlMode::Manual
                        }
                        elle_system::ControlMode::Stabilized => {
                            elle_hardware::crsf::CrsfControlMode::Stabilized
                        }
                        elle_system::ControlMode::AltitudeHold => {
                            elle_hardware::crsf::CrsfControlMode::AltitudeHold
                        }
                    },
                    autotune: autotune_display,
                    heading_hold: fc.is_heading_hold_active(),
                });

                // --- Autotune RC switch logic (CH9, 3-position with debounce) ---
                if let PilotCommands::Raw(raw) = commands {
                    let ch9 = raw.channels[elle_config::AUTOTUNE_CH];
                    let current_pos: u8 = if ch9 < elle_config::AUTOTUNE_OFF_THRESHOLD {
                        0 // Off
                    } else if ch9 < elle_config::AUTOTUNE_PITCH_THRESHOLD {
                        1 // Pitch
                    } else {
                        2 // Roll
                    };

                    // Debounce
                    if current_pos == autotune_debounce_pos {
                        autotune_debounce_count += 1;
                    } else {
                        autotune_debounce_pos = current_pos;
                        autotune_debounce_count = 0;
                    }

                    if autotune_debounce_count == elle_config::AUTOTUNE_DEBOUNCE_TICKS {
                        let new_pos = autotune_debounce_pos;

                        if new_pos == 0 && autotuner.is_active() {
                            // Abort: switch moved to off while active
                            if let Some(saved) = autotuner.abort() {
                                fc.apply_saved_gains(&saved);
                                fc.clear_setpoint_override();
                                autotune_display = elle_hardware::crsf::AutotuneDisplay::Error;
                                autotune_display_timer = AUTOTUNE_DISPLAY_DURATION;
                                elle_hardware::elle_event!(
                                    warn,
                                    elle_hardware::event::EVT_AUTOTUNE_ABORTED,
                                    "Autotune ABORTED (RC switch off)"
                                );
                            }
                        } else if autotune_stable_pos == 0
                            && ((new_pos == 1 && !pitch_tune_done) || new_pos == 2)
                            && !autotuner.is_active()
                        {
                            // Start: from off to pitch (not locked) or roll
                            if fc.is_armed() && fc.is_attitude_enabled() {
                                let axis = if new_pos == 1 {
                                    AutotuneAxis::Pitch
                                } else {
                                    AutotuneAxis::Roll
                                };
                                let current_gains = fc.get_pid_gains();
                                let test_gains = autotuner.start(
                                    axis,
                                    current_gains,
                                    5.0, // relay_deg
                                    6,   // num_cycles
                                    elle_control::autotune::TuningRule::TyreusLuyben,
                                    autotune_tick,
                                );
                                fc.apply_saved_gains(&test_gains);
                                elle_hardware::elle_event!(
                                    info,
                                    elle_hardware::event::EVT_AUTOTUNE_STARTED,
                                    "Autotune STARTED (axis={})",
                                    if new_pos == 1 { "pitch" } else { "roll" }
                                );
                            }
                        }
                        // Completion lock: ignore mid position if pitch already done
                        // (no action needed, the conditions above skip it)

                        autotune_stable_pos = new_pos;
                    }
                }

                // --- Heading-hold RC switch logic (CH5, 2-position, debounced) ---
                // Active only while Stabilized is selected; captures current heading
                // on the rising edge of (switch on AND mode == Stabilized).
                if let PilotCommands::Raw(raw) = commands {
                    let ch5 = raw.channels[elle_config::HEADING_HOLD_CH];
                    let switch_on = ch5 > elle_config::HEADING_HOLD_THRESHOLD;

                    if switch_on == heading_hold_switch_debounced {
                        heading_hold_debounce_count = 0;
                    } else {
                        heading_hold_debounce_count += 1;
                        if heading_hold_debounce_count >= elle_config::HEADING_HOLD_DEBOUNCE_TICKS {
                            heading_hold_switch_debounced = switch_on;
                            heading_hold_debounce_count = 0;
                        }
                    }

                    let heading_hold_effective = heading_hold_switch_debounced
                        && fc.current_control_mode() == elle_system::ControlMode::Stabilized;

                    if heading_hold_effective && !heading_hold_effective_prev {
                        if let Some(att) = valid_attitude.as_ref() {
                            fc.engage_heading_hold(att.yaw);
                            elle_hardware::elle_event!(
                                info,
                                elle_hardware::event::EVT_HEADING_HOLD_ENGAGED,
                                "Heading hold ENGAGED"
                            );
                        }
                    } else if !heading_hold_effective && heading_hold_effective_prev {
                        fc.disengage_heading_hold();
                        elle_hardware::elle_event!(
                            info,
                            elle_hardware::event::EVT_HEADING_HOLD_DISENGAGED,
                            "Heading hold DISENGAGED"
                        );
                    }
                    heading_hold_effective_prev = heading_hold_effective;
                }

                // Autotune state machine tick
                if autotuner.is_active() && valid_attitude.is_none() {
                    // Attitude data lost during autotune — abort for safety
                    if let Some(saved) = autotuner.abort() {
                        fc.apply_saved_gains(&saved);
                        fc.clear_setpoint_override();
                        autotune_display = elle_hardware::crsf::AutotuneDisplay::Error;
                        autotune_display_timer = AUTOTUNE_DISPLAY_DURATION;
                        elle_hardware::elle_event!(
                            error,
                            elle_hardware::event::EVT_AUTOTUNE_ESTOP,
                            "Autotune aborted: attitude data lost"
                        );
                    }
                }
                if autotuner.is_active()
                    && let Some(att) = valid_attitude.as_ref()
                {
                    let measurement_deg = if autotune_stable_pos == 1 {
                        att.pitch * (180.0 / core::f32::consts::PI)
                    } else {
                        att.roll * (180.0 / core::f32::consts::PI)
                    };

                    match autotuner.update(measurement_deg, autotune_tick) {
                        AutotuneAction::None => {}
                        AutotuneAction::SetpointOverride {
                            pitch_deg,
                            roll_deg,
                        } => {
                            fc.set_setpoint_override(pitch_deg, roll_deg);
                        }
                        AutotuneAction::ApplyGains(gains) => {
                            fc.apply_saved_gains(&gains);
                        }
                        AutotuneAction::RestoreGains(gains) => {
                            fc.apply_saved_gains(&gains);
                            fc.clear_setpoint_override();
                            autotune_display = elle_hardware::crsf::AutotuneDisplay::Error;
                            autotune_display_timer = AUTOTUNE_DISPLAY_DURATION;
                            elle_hardware::elle_event!(
                                warn,
                                elle_hardware::event::EVT_AUTOTUNE_ESTOP,
                                "Autotune safety abort (timeout/amplitude)"
                            );
                        }
                        AutotuneAction::Completed(result) => {
                            if let Some(gains) = autotuner.computed_gains() {
                                fc.apply_saved_gains(&gains);
                                save_pending = Some(gains.to_bytes());
                            }
                            fc.clear_setpoint_override();
                            if result.axis == AutotuneAxis::Pitch {
                                pitch_tune_done = true;
                            }
                            autotune_display = elle_hardware::crsf::AutotuneDisplay::Done;
                            autotune_display_timer = AUTOTUNE_DISPLAY_DURATION;
                            info!(
                                "Autotune COMPLETE: Ku={} Tu={}ms kp={} ki={} kd={} amp={}cdeg cycles={}",
                                (result.ku * 1000.0) as i32,
                                (result.tu_s * 1000.0) as i32,
                                (result.kp * 1000.0) as i32,
                                (result.ki * 1000.0) as i32,
                                (result.kd * 1000.0) as i32,
                                (result.amplitude_deg * 100.0) as i32,
                                result.cycles,
                            );
                            elle_hardware::elle_event!(
                                info,
                                elle_hardware::event::EVT_AUTOTUNE_COMPLETE,
                                "Autotune COMPLETE"
                            );
                        }
                    }

                    // Log autotune status to ULog (control-loop rate during active autotune)
                    if ulog_recording {
                        let _ = ulog_logger.log_autotune(
                            autotuner.phase_u8(),
                            autotuner.axis() as u8,
                            autotuner.relay_positive(),
                            autotuner.current_setpoint_deg(),
                            measurement_deg,
                            autotuner.cycles_completed(),
                            autotuner.current_amplitude_deg(),
                        );
                    }
                }

                autotune_tick += 1;

                // Auto-save PID gains to flash after autotune completion
                if let Some(data) = save_pending.take() {
                    save_pid_to_flash(data, "autotune").await;
                }

                // Auto-start ULog on SD card ready (runs until power off)
                if !ulog_recording
                    && elle_hardware::sd_writer::SD_READY
                        .load(core::sync::atomic::Ordering::Acquire)
                {
                    if !ulog_logger.is_initialized() {
                        // Start before the header is written: the SD writer
                        // discards anything that arrives while it is idle.
                        elle_hardware::sd_writer::SD_CMD_SIGNAL
                            .signal(elle_hardware::sd_writer::SdCommand::Start);
                        let wall_ms =
                            compile_time::unix!() * 1000 + Instant::now().as_micros() / 1000;
                        if ulog_logger.initialize(wall_ms).await.is_err() {
                            elle_hardware::sd_writer::SD_CMD_SIGNAL
                                .signal(elle_hardware::sd_writer::SdCommand::Stop);
                        }
                    }
                    if ulog_logger.is_initialized() {
                        ulog_recording = true;
                        elle_hardware::elle_event!(
                            info,
                            elle_hardware::event::EVT_ULOG_RC_ON,
                            "ULog recording started (auto)"
                        );
                    }
                }

                if ulog_recording {
                    log_flight_data(
                        &mut ulog_logger,
                        valid_attitude.as_ref(),
                        commands,
                        loop_counter,
                        loop_start.elapsed().as_micros() as u32,
                        &fc,
                    );
                } else if loop_counter.is_multiple_of(STALE_EVENT_DRAIN_DIVISOR) {
                    // Drain stale events when not recording
                    while elle_hardware::event::ULOG_EVENT_CHANNEL
                        .try_receive()
                        .is_ok()
                    {}
                }
            }

            // Check for failsafe (triggers after 300ms of no valid packets).
            // Packet arrival is stamped via note_rc_packet(), so this stays
            // accurate even while the kill switch blocks fc.update().
            fc.check_failsafe();

            update_control_loop_timing(loop_timer.elapsed_us());
            loop_counter = loop_counter.saturating_add(1);

            // Periodic status updates
            if loop_counter.is_multiple_of(PERF_LOG_INTERVAL) {
                log_performance_summary();
            }

            if loop_counter.is_multiple_of(LED_UPDATE_INTERVAL) {
                loop_counter = 0;

                let imu_status = IMU_STATUS.read().await;

                let led_pattern = if fc.is_failsafe() {
                    LedPattern::RapidFlash(colors::ORANGE)
                } else if mag_cal_collecting {
                    LedPattern::FastBlink(colors::YELLOW)
                } else if elle_hardware::imu::level_cal::is_collecting() {
                    LedPattern::FastBlink(colors::CYAN)
                } else if fc.is_armed() && fc.rc_link_state() == elle_system::RcLinkState::Warning {
                    LedPattern::FastBlink(colors::ORANGE)
                } else if fc.is_armed() {
                    if fc.is_attitude_enabled() {
                        if fc.is_heading_hold_active() {
                            LedPattern::Pulse(colors::BLUE)
                        } else {
                            LedPattern::Pulse(colors::CYAN)
                        }
                    } else {
                        LedPattern::DoubleBlink(colors::GREEN)
                    }
                } else if imu_status.calibrated {
                    LedPattern::Solid(colors::GREEN)
                } else {
                    LedPattern::Pulse(colors::CYAN)
                };

                let _ = LED_COMMAND_CHANNEL.try_send(led_pattern);
                drop(imu_status);
            }
        }
    }

    #[cfg(feature = "rpc-control")]
    {
        use core::sync::atomic::Ordering;
        use elle_config::profile::{FlashRequest, FlashResponse};
        use elle_control::commands::AttitudeMode;
        use elle_control::commands::NormalizedCommands;
        use elle_control::commands::PilotCommands;
        use elle_hardware::flash::{FLASH_REQUEST_SIGNAL, FLASH_RESPONSE_SIGNAL};
        use rpc_app::{
            ULOG_ENABLED, ULOG_ITEM_LEN, ULOG_ITEM_SIGNAL, ULOG_OFFSET, ULOG_STATE, ULogState,
        };
        use rpc_handlers::{RPC_CMD_CHANNEL, RpcCommand};

        info!("GROUND TEST MODE - RPC Control (postcard-RPC over RTT)");
        info!("WARNING: This mode requires programmer connection");

        // RPC mode: arming is via explicit arm/disarm commands only
        fc.set_explicit_arming(true);

        // Ticker for consistent control loop timing
        let mut ticker = Ticker::every(Duration::from_millis(CONTROL_LOOP_PERIOD_MS));

        // RPC commands accumulator
        let mut rpc_throttle: f32 = 0.0;
        let mut rpc_elevon_left: f32 = 0.0;
        let mut rpc_elevon_right: f32 = 0.0;
        let mut rpc_mode = AttitudeMode::Manual;

        // Autotuner state (same pattern as flight mode)
        let mut autotuner = Autotuner::new();
        let mut autotune_tick: u32 = 0;
        let mut rpc_save_pending: Option<[u8; 32]> = None;
        let mut was_armed = false;
        let mut was_killed = false;
        let mut autotune_display = elle_hardware::crsf::AutotuneDisplay::Off;
        let mut autotune_display_timer: u32 = 0;
        const AUTOTUNE_DISPLAY_DURATION: u32 = CONTROL_LOOP_FREQUENCY_HZ * 3; // ~3 s

        loop {
            ticker.next().await;
            let loop_start = Instant::now();
            let loop_timer = TimingMeasurement::start();

            // Supervisor check
            let _supervisor_healthy = fc.supervisor_check();

            // Get latest attitude data
            let attitude = ATTITUDE.try_take();

            // Process all pending RPC commands
            while let Ok(cmd) = RPC_CMD_CHANNEL.try_receive() {
                match cmd {
                    RpcCommand::SetThrottle(percent) => {
                        rpc_throttle = (percent as f32 / 100.0).clamp(0.0, 1.0);
                    }
                    RpcCommand::SetElevons { left, right } => {
                        rpc_elevon_left = (left as f32 / 100.0).clamp(-1.0, 1.0);
                        rpc_elevon_right = (right as f32 / 100.0).clamp(-1.0, 1.0);
                    }
                    RpcCommand::SetMode(mode) => {
                        rpc_mode = match mode {
                            ControlMode::Manual => AttitudeMode::Manual,
                            ControlMode::Stabilized => AttitudeMode::Stabilized,
                            ControlMode::AltitudeHold => AttitudeMode::AltitudeHold,
                        };
                        info!(
                            "RPC: Control mode set to {}",
                            match mode {
                                ControlMode::Manual => "Manual",
                                ControlMode::Stabilized => "Stabilized",
                                ControlMode::AltitudeHold => "AltitudeHold",
                            }
                        );
                    }
                    RpcCommand::Arm => {
                        fc.arm();
                        elle_hardware::elle_event!(
                            info,
                            elle_hardware::event::EVT_MOTORS_ARMED,
                            "Motors ARMED via RPC"
                        );
                    }
                    RpcCommand::Disarm => {
                        fc.disarm();
                        elle_hardware::elle_event!(
                            info,
                            elle_hardware::event::EVT_MOTORS_DISARMED,
                            "Motors DISARMED via RPC"
                        );
                    }
                    RpcCommand::EmergencyStop => {
                        elle_hardware::elle_event!(
                            warn,
                            elle_hardware::event::EVT_EMERGENCY_STOP,
                            "RPC: EMERGENCY STOP"
                        );
                        rpc_throttle = 0.0;
                        rpc_elevon_left = 0.0;
                        rpc_elevon_right = 0.0;
                        fc.disarm();
                        fc.apply_failsafe();
                    }
                    RpcCommand::SetPidGains {
                        pitch_kp,
                        pitch_ki,
                        pitch_kd,
                        roll_kp,
                        roll_ki,
                        roll_kd,
                        scale,
                        i_limit,
                    } => {
                        let config = elle_control::PidConfig {
                            kp_pitch: pitch_kp,
                            ki_pitch: pitch_ki,
                            kd_pitch: pitch_kd,
                            kp_roll: roll_kp,
                            ki_roll: roll_ki,
                            kd_roll: roll_kd,
                            scale,
                            i_limit,
                        };
                        fc.set_pid_gains(config);
                        info!(
                            "RPC: PID gains updated P({}/{}/{}) R({}/{}/{}) s={} il={}",
                            (pitch_kp * 1000.0) as i32,
                            (pitch_ki * 1000.0) as i32,
                            (pitch_kd * 1000.0) as i32,
                            (roll_kp * 1000.0) as i32,
                            (roll_ki * 1000.0) as i32,
                            (roll_kd * 1000.0) as i32,
                            (scale * 10000.0) as i32,
                            (i_limit * 10.0) as i32,
                        );
                    }
                    RpcCommand::StartULog => {
                        // Every Start opens a new SD file, and each file needs its
                        // own header — so always re-initialize, never reuse the
                        // previous session's logger state. Start goes first: the
                        // SD writer discards anything that arrives while it is idle.
                        let mut ok = false;
                        if elle_hardware::sd_writer::SD_READY.load(Ordering::Acquire) {
                            ULOG_ENABLED.store(false, Ordering::Release);
                            elle_hardware::sd_writer::SD_CMD_SIGNAL
                                .signal(elle_hardware::sd_writer::SdCommand::Start);
                            let wall_ms =
                                compile_time::unix!() * 1000 + Instant::now().as_micros() / 1000;
                            ok = ulog_logger.initialize(wall_ms).await.is_ok();
                            if !ok {
                                elle_hardware::sd_writer::SD_CMD_SIGNAL
                                    .signal(elle_hardware::sd_writer::SdCommand::Stop);
                            }
                        }
                        if ok {
                            ULOG_ENABLED.store(true, Ordering::Release);
                            elle_hardware::elle_event!(
                                info,
                                elle_hardware::event::EVT_ULOG_STARTED,
                                "ULog recording started"
                            );
                        } else {
                            elle_hardware::elle_event!(
                                error,
                                elle_hardware::event::EVT_ULOG_INIT_FAILED,
                                "ULog init failed"
                            );
                        }
                    }
                    RpcCommand::StopULog => {
                        ULOG_ENABLED.store(false, Ordering::Release);
                        let _ = ulog_logger.flush();
                        elle_hardware::sd_writer::SD_CMD_SIGNAL
                            .signal(elle_hardware::sd_writer::SdCommand::Stop);
                        elle_hardware::elle_event!(
                            info,
                            elle_hardware::event::EVT_ULOG_STOPPED,
                            "ULog recording stopped"
                        );
                    }
                    RpcCommand::ReadULogChunk => {
                        FLASH_REQUEST_SIGNAL.signal(FlashRequest::PeekULog);
                        match FLASH_RESPONSE_SIGNAL.wait().await {
                            FlashResponse::ULogData { data, len } => {
                                ULOG_ITEM_LEN.store(len as u16, Ordering::Release);
                                ULOG_ITEM_SIGNAL.signal((data, len));
                                ULOG_STATE.store(ULogState::Ready as u8, Ordering::Release);
                            }
                            FlashResponse::ULogEmpty => {
                                ULOG_STATE.store(ULogState::Empty as u8, Ordering::Release);
                            }
                            _ => {
                                ULOG_STATE.store(ULogState::Empty as u8, Ordering::Release);
                            }
                        }
                    }
                    RpcCommand::PopAndPeekULog => {
                        // Pop the item we just finished sending
                        FLASH_REQUEST_SIGNAL.signal(FlashRequest::PopULog);
                        let _ = FLASH_RESPONSE_SIGNAL.wait().await;
                        // Peek the next item
                        FLASH_REQUEST_SIGNAL.signal(FlashRequest::PeekULog);
                        match FLASH_RESPONSE_SIGNAL.wait().await {
                            FlashResponse::ULogData { data, len } => {
                                ULOG_ITEM_LEN.store(len as u16, Ordering::Release);
                                ULOG_ITEM_SIGNAL.signal((data, len));
                                ULOG_STATE.store(ULogState::Ready as u8, Ordering::Release);
                            }
                            FlashResponse::ULogEmpty => {
                                ULOG_STATE.store(ULogState::Empty as u8, Ordering::Release);
                            }
                            _ => {
                                ULOG_STATE.store(ULogState::Empty as u8, Ordering::Release);
                            }
                        }
                    }
                    RpcCommand::EraseULog => {
                        // Stop recording first
                        ULOG_ENABLED.store(false, Ordering::Release);
                        FLASH_REQUEST_SIGNAL.signal(FlashRequest::EraseULog);
                        let _ = FLASH_RESPONSE_SIGNAL.wait().await;
                        // Reset ULog transfer state
                        ULOG_STATE.store(ULogState::Idle as u8, Ordering::Release);
                        ULOG_OFFSET.store(0, Ordering::Release);
                        ULOG_ITEM_LEN.store(0, Ordering::Release);
                        elle_hardware::elle_event!(
                            info,
                            elle_hardware::event::EVT_ULOG_ERASED,
                            "ULog: flash erased"
                        );
                    }
                    RpcCommand::StartAutotune {
                        axis,
                        relay_deg_x10,
                        num_cycles,
                        rule,
                    } => {
                        if axis == 0xFF {
                            // Magic value: save current PID gains to flash
                            let gains = fc.get_pid_gains();
                            save_pid_to_flash(gains.to_bytes(), "savepid").await;
                        } else if axis == 0xFE {
                            // Magic value: erase PID profile from flash
                            erase_pid_from_flash().await;
                        } else {
                            use elle_control::autotune::TuningRule;
                            if fc.is_armed() && fc.is_attitude_enabled() && !autotuner.is_active() {
                                let at_axis = if axis == 0 {
                                    AutotuneAxis::Pitch
                                } else {
                                    AutotuneAxis::Roll
                                };
                                let relay_deg = relay_deg_x10 as f32 / 10.0;
                                let at_rule = match rule {
                                    1 => TuningRule::ZieglerNichols,
                                    2 => TuningRule::SomeOvershoot,
                                    _ => TuningRule::TyreusLuyben,
                                };
                                let current_gains = fc.get_pid_gains();
                                let test_gains = autotuner.start(
                                    at_axis,
                                    current_gains,
                                    relay_deg,
                                    num_cycles as usize,
                                    at_rule,
                                    autotune_tick,
                                );
                                fc.apply_saved_gains(&test_gains);
                                elle_hardware::elle_event!(
                                    info,
                                    elle_hardware::event::EVT_AUTOTUNE_STARTED,
                                    "Autotune STARTED via RPC (axis={})",
                                    if axis == 0 { "pitch" } else { "roll" }
                                );
                            } else {
                                info!(
                                    "RPC: Autotune rejected (armed={} attitude={} active={})",
                                    fc.is_armed(),
                                    fc.is_attitude_enabled(),
                                    autotuner.is_active()
                                );
                            }
                        }
                    }
                    RpcCommand::AbortAutotune => {
                        if let Some(saved) = autotuner.abort() {
                            fc.apply_saved_gains(&saved);
                            fc.clear_setpoint_override();
                            autotune_display = elle_hardware::crsf::AutotuneDisplay::Error;
                            autotune_display_timer = AUTOTUNE_DISPLAY_DURATION;
                            elle_hardware::elle_event!(
                                warn,
                                elle_hardware::event::EVT_AUTOTUNE_ABORTED,
                                "Autotune ABORTED via RPC"
                            );
                        }
                    }
                    RpcCommand::SavePidProfile { data } => {
                        save_pid_to_flash(data, "RPC").await;
                    }
                    RpcCommand::ClearPidProfile => {
                        erase_pid_from_flash().await;
                    }
                    RpcCommand::StartMagCal => {
                        elle_hardware::imu::MAG_CAL_START_SIGNAL.signal(());
                        rpc_app::MAG_CAL_STATUS.store(1, Ordering::Relaxed);
                        elle_hardware::elle_event!(
                            info,
                            elle_hardware::event::EVT_MAG_CAL_STARTED,
                            "Mag calibration started"
                        );
                    }
                    RpcCommand::ClearMagCal => {
                        // Save zeros to flash
                        FLASH_REQUEST_SIGNAL.signal(FlashRequest::SaveMagCal { data: [0; 12] });
                        let save_timeout = Timer::after(Duration::from_secs(5));
                        let _ = embassy_futures::select::select(
                            FLASH_RESPONSE_SIGNAL.wait(),
                            save_timeout,
                        )
                        .await;
                        // Signal zero offsets to IMU
                        elle_hardware::imu::MAG_CALIBRATION_SIGNAL.signal((0.0, 0.0, 0.0));
                        rpc_app::MAG_CAL_STATUS.store(0, Ordering::Relaxed);
                        rpc_app::MAG_CAL_OFFSET.lock(|c| c.set((0.0, 0.0, 0.0)));
                        elle_hardware::elle_event!(
                            info,
                            elle_hardware::event::EVT_MAG_CAL_CLEARED,
                            "Mag calibration cleared"
                        );
                    }
                    RpcCommand::StartLevelCal => {
                        if fc.is_armed() {
                            elle_hardware::elle_event!(
                                warn,
                                elle_hardware::event::EVT_LEVEL_CAL_FAILED_MOVING,
                                "Level calibration refused: armed"
                            );
                        } else if !elle_hardware::imu::level_cal::is_collecting() {
                            elle_hardware::imu::level_cal::start();
                        }
                    }
                    RpcCommand::ClearLevelCal => {
                        elle_hardware::imu::level_cal::clear().await;
                    }
                    RpcCommand::SetHeadingHold {
                        enabled,
                        target_cdeg,
                    } => {
                        if enabled {
                            let target_rad = (target_cdeg as f32 / 100.0).to_radians();
                            fc.engage_heading_hold(target_rad);
                            elle_hardware::elle_event!(
                                info,
                                elle_hardware::event::EVT_HEADING_HOLD_TARGET_SET,
                                "Heading hold target set via RPC"
                            );
                        } else {
                            fc.disengage_heading_hold();
                            elle_hardware::elle_event!(
                                info,
                                elle_hardware::event::EVT_HEADING_HOLD_DISENGAGED,
                                "Heading hold disengaged via RPC"
                            );
                        }
                    }
                }
            }

            // Build pilot commands from RPC accumulators
            let commands = {
                if let Some(PilotCommands::Raw(raw)) = RC_COMMANDS.try_take().as_ref() {
                    rc_signal::RC_SIGNAL.signal(raw.channels);
                }
                Some(PilotCommands::Normalized(NormalizedCommands {
                    throttle: rpc_throttle,
                    pitch: (rpc_elevon_left + rpc_elevon_right) / 2.0,
                    roll: (rpc_elevon_right - rpc_elevon_left) / 2.0,
                    yaw: 0.0,
                    attitude_mode: rpc_mode,
                    timestamp: Instant::now(),
                }))
            };

            let kill_active = false;

            if kill_active != was_killed {
                if kill_active {
                    elle_hardware::elle_event!(
                        warn,
                        elle_hardware::event::EVT_KILL_ENGAGED,
                        "Kill switch engaged (RPC)"
                    );
                } else {
                    elle_hardware::elle_event!(
                        info,
                        elle_hardware::event::EVT_KILL_RELEASED,
                        "Kill switch released (RPC)"
                    );
                }
                was_killed = kill_active;
            }

            if kill_active {
                if fc.is_armed() {
                    fc.disarm();
                }
                fc.set_safe_positions();
                DSHOT_THROTTLE.signal((0, 0));
            }

            // Double-tap calibration gesture (RPC mode) — only while the kill switch
            // is active (inert in pure RPC builds where kill_active is always
            // false — use `mag cal start` / `level cal start` there instead).
            // CH7 out of its off position selects level cal, as in flight mode.
            let tapped = elle_hardware::imu::TAP_SIGNAL.try_take().is_some();
            if tapped
                && !fc.is_armed()
                && kill_active
                && rpc_app::MAG_CAL_STATUS.load(Ordering::Relaxed) != 1
                && !elle_hardware::imu::level_cal::is_collecting()
            {
                if !tap_cal_allowed(commands.as_ref(), attitude.as_ref()) {
                    info!("Double-tap ignored: throttle not low or gyro not quiet");
                } else if tap_selects_level_cal(commands.as_ref()) {
                    elle_hardware::imu::level_cal::start();
                } else {
                    elle_hardware::imu::MAG_CAL_START_SIGNAL.signal(());
                    rpc_app::MAG_CAL_STATUS.store(1, Ordering::Relaxed);
                    elle_hardware::elle_event!(
                        info,
                        elle_hardware::event::EVT_MAG_CAL_STARTED,
                        "Double-tap: mag calibration started"
                    );
                }
            }

            // Update flight controller
            let valid_attitude = validate_attitude(attitude);
            if let Some(ref commands) = commands
                && !kill_active
            {
                fc.update(commands, valid_attitude.as_ref());
            }

            // Send engine commands via DShot (governor converts eRPM target to DShot)
            if !kill_active {
                let (engine_l, _) = fc.engine_output();
                let l_erpm = (engine_l as u32 * elle_config::MAX_ERPM)
                    / elle_config::DSHOT_THROTTLE_MAX as u32;
                DSHOT_THROTTLE.signal((l_erpm, 0));
            }

            // Detect arm/disarm transitions → beep
            let now_armed = fc.is_armed();
            if now_armed && !was_armed {
                elle_hardware::dshot::BEEP_SIGNAL
                    .signal(elle_hardware::dshot::BeepPattern::ArmBeep);
            } else if !now_armed && was_armed {
                elle_hardware::dshot::BEEP_SIGNAL
                    .signal(elle_hardware::dshot::BeepPattern::DisarmBeep);
            }
            was_armed = now_armed;

            // No failsafe check in RPC mode — no RC link to monitor

            // Autotuner per-tick update
            if autotuner.is_active()
                && let Some(att) = valid_attitude.as_ref()
            {
                let measurement_deg = match autotuner.axis() {
                    AutotuneAxis::Pitch => att.pitch * (180.0 / core::f32::consts::PI),
                    AutotuneAxis::Roll => att.roll * (180.0 / core::f32::consts::PI),
                };
                match autotuner.update(measurement_deg, autotune_tick) {
                    AutotuneAction::None => {}
                    AutotuneAction::SetpointOverride {
                        pitch_deg,
                        roll_deg,
                    } => {
                        fc.set_setpoint_override(pitch_deg, roll_deg);
                    }
                    AutotuneAction::ApplyGains(gains) => {
                        fc.apply_saved_gains(&gains);
                    }
                    AutotuneAction::RestoreGains(gains) => {
                        fc.apply_saved_gains(&gains);
                        fc.clear_setpoint_override();
                        autotune_display = elle_hardware::crsf::AutotuneDisplay::Error;
                        autotune_display_timer = AUTOTUNE_DISPLAY_DURATION;
                        elle_hardware::elle_event!(
                            warn,
                            elle_hardware::event::EVT_AUTOTUNE_ESTOP,
                            "Autotune safety abort (timeout/amplitude)"
                        );
                    }
                    AutotuneAction::Completed(result) => {
                        if let Some(gains) = autotuner.computed_gains() {
                            fc.apply_saved_gains(&gains);
                            rpc_save_pending = Some(gains.to_bytes());
                        }
                        fc.clear_setpoint_override();
                        autotune_display = elle_hardware::crsf::AutotuneDisplay::Done;
                        autotune_display_timer = AUTOTUNE_DISPLAY_DURATION;
                        info!(
                            "Autotune COMPLETE: Ku={} Tu={}ms kp={} ki={} kd={} amp={}cdeg cycles={}",
                            (result.ku * 1000.0) as i32,
                            (result.tu_s * 1000.0) as i32,
                            (result.kp * 1000.0) as i32,
                            (result.ki * 1000.0) as i32,
                            (result.kd * 1000.0) as i32,
                            (result.amplitude_deg * 100.0) as i32,
                            result.cycles,
                        );
                        elle_hardware::elle_event!(
                            info,
                            elle_hardware::event::EVT_AUTOTUNE_COMPLETE,
                            "Autotune COMPLETE (RPC)"
                        );
                    }
                }

                // Log autotune status to ULog (control-loop rate during active autotune)
                if ULOG_ENABLED.load(Ordering::Acquire) {
                    let _ = ulog_logger.log_autotune(
                        autotuner.phase_u8(),
                        autotuner.axis() as u8,
                        autotuner.relay_positive(),
                        autotuner.current_setpoint_deg(),
                        measurement_deg,
                        autotuner.cycles_completed(),
                        autotuner.current_amplitude_deg(),
                    );
                }
            }
            autotune_tick += 1;

            // Auto-save PID gains to flash after autotune completion
            if let Some(data) = rpc_save_pending.take() {
                save_pid_to_flash(data, "autotune").await;
            }

            // Poll level calibration result from IMU task
            elle_hardware::imu::level_cal::poll_result().await;

            // Poll mag calibration result from IMU task
            if let Some(result) = elle_hardware::imu::MAG_CAL_RESULT_SIGNAL.try_take() {
                match result {
                    Some((ox, oy, oz)) => {
                        // Serialize offsets to 12 bytes (3x f32 little-endian)
                        let mut data = [0u8; 12];
                        data[0..4].copy_from_slice(&ox.to_le_bytes());
                        data[4..8].copy_from_slice(&oy.to_le_bytes());
                        data[8..12].copy_from_slice(&oz.to_le_bytes());

                        FLASH_REQUEST_SIGNAL.signal(FlashRequest::SaveMagCal { data });
                        let save_timeout = Timer::after(Duration::from_secs(5));
                        match embassy_futures::select::select(
                            FLASH_RESPONSE_SIGNAL.wait(),
                            save_timeout,
                        )
                        .await
                        {
                            embassy_futures::select::Either::First(FlashResponse::MagCalSaved) => {
                                elle_hardware::elle_event!(
                                    info,
                                    elle_hardware::event::EVT_MAG_CAL_SAVED,
                                    "Mag cal saved to flash"
                                );
                            }
                            _ => {
                                elle_hardware::elle_event!(
                                    warn,
                                    elle_hardware::event::EVT_MAG_CAL_FAILED,
                                    "Mag cal flash save failed"
                                );
                            }
                        }
                        rpc_app::MAG_CAL_STATUS.store(2, Ordering::Relaxed);
                        rpc_app::MAG_CAL_OFFSET.lock(|c| c.set((ox, oy, oz)));
                        elle_hardware::elle_event!(
                            info,
                            elle_hardware::event::EVT_MAG_CAL_COMPLETE,
                            "Mag calibration complete"
                        );
                    }
                    None => {
                        rpc_app::MAG_CAL_STATUS.store(0, Ordering::Relaxed);
                        elle_hardware::elle_event!(
                            warn,
                            elle_hardware::event::EVT_MAG_CAL_FAILED,
                            "Mag calibration failed (insufficient rotation)"
                        );
                    }
                }
            }

            // Update mag cal sample count for RPC visibility
            // (read from IMU side if collecting — approximated via status check)

            // Update autotune display state for CRSF telemetry
            if autotuner.is_active() {
                autotune_display = match autotuner.axis() {
                    AutotuneAxis::Pitch => elle_hardware::crsf::AutotuneDisplay::Pitch,
                    AutotuneAxis::Roll => elle_hardware::crsf::AutotuneDisplay::Roll,
                };
            } else if autotune_display_timer > 0 {
                autotune_display_timer -= 1;
                if autotune_display_timer == 0 {
                    autotune_display = elle_hardware::crsf::AutotuneDisplay::Off;
                }
            }

            elle_hardware::crsf::CRSF_FLIGHT_MODE.signal(elle_hardware::crsf::CrsfFlightMode {
                armed: fc.is_armed(),
                failsafe: fc.is_failsafe(),
                mode: match fc.current_control_mode() {
                    elle_system::ControlMode::Manual => {
                        elle_hardware::crsf::CrsfControlMode::Manual
                    }
                    elle_system::ControlMode::Stabilized => {
                        elle_hardware::crsf::CrsfControlMode::Stabilized
                    }
                    elle_system::ControlMode::AltitudeHold => {
                        elle_hardware::crsf::CrsfControlMode::AltitudeHold
                    }
                },
                autotune: autotune_display,
                heading_hold: fc.is_heading_hold_active(),
            });

            // Publish flight state for RPC handlers
            let fs = flight_state::FlightState {
                armed: fc.is_armed(),
                failsafe: fc.is_failsafe(),
                mode: match fc.current_control_mode() {
                    elle_system::ControlMode::Manual => ControlMode::Manual,
                    elle_system::ControlMode::Stabilized => ControlMode::Stabilized,
                    elle_system::ControlMode::AltitudeHold => ControlMode::AltitudeHold,
                },
                rc_age_ms: fc.rc_signal_age_ms(),
                autotune_state: if autotuner.is_active() {
                    if autotuner.axis() == AutotuneAxis::Pitch {
                        1
                    } else {
                        2
                    }
                } else {
                    0
                },
            };
            flight_state::FLIGHT_STATE.publish(fs);

            // Publish controller output for RPC observability
            {
                let out = fc.last_output();
                let co = flight_state::ControllerOutput {
                    pitch_correction: out.pitch_correction,
                    roll_correction: out.roll_correction,
                    pitch_setpoint_deg: out.pitch_setpoint_deg,
                    roll_setpoint_deg: out.roll_setpoint_deg,
                    elevon_left_us: out.elevon_left_us,
                    elevon_right_us: out.elevon_right_us,
                    engine_left_dshot: out.engine_left_dshot,
                    engine_right_dshot: out.engine_right_dshot,
                    heading_hold_active: out.heading_hold_active,
                    heading_target_deg: out.heading_target_deg,
                    heading_error_deg: out.heading_error_deg,
                };
                flight_state::CONTROLLER_OUTPUT.publish(co);
            }

            // Log flight data to ULog flash storage (only when recording is active)
            let neutral = PilotCommands::Normalized(NormalizedCommands::neutral());
            let ulog_commands = commands.as_ref().unwrap_or(&neutral);
            if ULOG_ENABLED.load(Ordering::Acquire) {
                log_flight_data(
                    &mut ulog_logger,
                    valid_attitude.as_ref(),
                    ulog_commands,
                    loop_counter,
                    loop_start.elapsed().as_micros() as u32,
                    &fc,
                );
            } else if loop_counter.is_multiple_of(STALE_EVENT_DRAIN_DIVISOR) {
                // Drain stale events when not recording
                while elle_hardware::event::ULOG_EVENT_CHANNEL
                    .try_receive()
                    .is_ok()
                {}
            }

            update_control_loop_timing(loop_timer.elapsed_us());
            loop_counter = loop_counter.saturating_add(1);

            if loop_counter.is_multiple_of(PERF_LOG_INTERVAL) {
                log_performance_summary();
            }

            if loop_counter.is_multiple_of(LED_UPDATE_INTERVAL) {
                loop_counter = 0;

                let imu_status = IMU_STATUS.read().await;
                let led_pattern = if fc.is_failsafe() {
                    LedPattern::RapidFlash(colors::ORANGE)
                } else if elle_hardware::imu::level_cal::is_collecting() {
                    LedPattern::FastBlink(colors::CYAN)
                } else if fc.is_armed() && fc.rc_link_state() == elle_system::RcLinkState::Warning {
                    LedPattern::FastBlink(colors::ORANGE)
                } else if fc.is_armed() {
                    if fc.is_heading_hold_active() {
                        LedPattern::Pulse(colors::BLUE)
                    } else {
                        LedPattern::DoubleBlink(colors::PURPLE)
                    }
                } else if imu_status.calibrated {
                    LedPattern::Solid(colors::PURPLE)
                } else {
                    LedPattern::Pulse(colors::PURPLE)
                };

                let _ = LED_COMMAND_CHANNEL.try_send(led_pattern);
                drop(imu_status);
            }
        }
    }
}

#[allow(clippy::too_many_arguments)]
#[embassy_executor::task]
async fn imu_task(
    _spawner: Spawner,
    // I2C (mag + baro)
    i2c: Peri<'static, I2C0>,
    sda: Peri<'static, PIN_8>,
    scl: Peri<'static, PIN_9>,
    // SPI (ICM-42686)
    spi: Peri<'static, SPI0>,
    spi_miso: Peri<'static, PIN_0>,
    spi_cs: Peri<'static, PIN_1>,
    spi_sck: Peri<'static, PIN_2>,
    spi_mosi: Peri<'static, PIN_3>,
    // INT1 (DATA_RDY interrupt)
    int1_pin: Peri<'static, PIN_5>,
) {
    info!("Core1: IMU task starting");

    // I2C bus — always wrapped in RefCell (both cfg paths use shared I2C)
    let mut i2c_config = Config::default();
    i2c_config.frequency = embassy_rp::time::Hertz(IMU_I2C_FREQ);
    let i2c_bus = I2c::new_blocking(i2c, scl, sda, i2c_config);

    use core::cell::RefCell;
    static I2C_BUS: StaticCell<RefCell<I2c<'static, embassy_rp::mode::Blocking>>> =
        StaticCell::new();
    let i2c_ref = I2C_BUS.init(RefCell::new(i2c_bus));

    let led_sender = LED_COMMAND_CHANNEL.sender();

    let mut imu = {
        use embassy_rp::gpio::{Input, Level, Output, Pull};
        use embassy_rp::spi as rp_spi;
        use embedded_hal_bus::spi::ExclusiveDevice;

        let mut spi_config = rp_spi::Config::default();
        spi_config.frequency = embassy_rp::time::Hertz(elle_config::IMU_SPI_FREQ);
        // SPI Mode 0 (CPOL=0, CPHA=0) — ICM-42686 default
        spi_config.polarity = rp_spi::Polarity::IdleLow;
        spi_config.phase = rp_spi::Phase::CaptureOnFirstTransition;

        let spi_bus =
            rp_spi::Spi::new_blocking(spi, spi_sck, spi_mosi, spi_miso, spi_config).unwrap();
        let cs = Output::new(spi_cs, Level::High);
        let spi_dev = ExclusiveDevice::new(spi_bus, cs, embassy_time::Delay).unwrap();
        let int1 = Input::new(int1_pin, Pull::None);

        Imu::new(spi_dev, i2c_ref, led_sender, int1)
    };

    // Initialize sensors
    match imu.initialize().await {
        Ok(_) => info!("Core1: IMU initialized"),
        Err(e) => {
            defmt::panic!("Core1: IMU init failed: {}", e);
        }
    }

    // Notify supervisor that IMU is initialized
    SUP_IMU_READY.signal(());

    // Calibration wait (ICM-42686 returns immediately — factory calibrated)
    if let Err(e) = imu.wait_for_calibration(IMU_CALIBRATION_TIMEOUT_S).await {
        warn!("Core1: IMU calibration incomplete: {}", e);
    }

    // Wait for supervisor start before entering main IMU run loop
    info!("Core1: Waiting for Supervisor start barrier");
    SUP_START_IMU.wait().await;

    // Run continuous IMU reading
    imu.run().await;
}

#[embassy_executor::task]
async fn flash_manager_task(flash: Flash<'static, Async, { FLASH_SIZE }>) {
    info!("Core0: Flash manager task starting");
    let mut manager = SequentialFlashManager::new(flash);
    manager.run().await;
}

#[embassy_executor::task]
async fn led_task(
    mut common: embassy_rp::pio::Common<'static, PIO0>,
    sm2: embassy_rp::pio::StateMachine<'static, PIO0, 2>,
    dma: Peri<'static, DMA_CH2>,
    irq: Irqs,
    pin: Peri<'static, PIN_10>,
) {
    info!("Core0: LED task starting");

    let mut led = StatusLed::new(&mut common, sm2, pin, dma, irq);
    let receiver: Receiver<'static, CriticalSectionRawMutex, LedPattern, 8> =
        LED_COMMAND_CHANNEL.receiver();

    // Set initial pattern
    led.set_pattern(LedPattern::SlowBlink(colors::BLUE)).await;

    // Notify supervisor that LED task is initialized
    SUP_LED_READY.signal(());

    // Main LED update loop
    loop {
        let led_timer = TimingMeasurement::start();

        // Check for new patterns
        if let Ok(pattern) = receiver.try_receive() {
            led.set_pattern(pattern).await;
        }

        // Update LED animation
        led.update().await;

        // Update performance metrics
        update_led_timing(led_timer.elapsed_us());

        // Small delay for animation timing
        Timer::after(Duration::from_millis(10)).await;
    }
}

#[cfg(feature = "rpc-control")]
#[embassy_executor::task]
async fn log_publisher_task(sender: postcard_rpc::server::Sender<elle_system::rpc::RttTx>) {
    use postcard_rpc::header::VarSeq;

    let mut seq: u16 = 0;
    loop {
        let (level, code) = elle_hardware::event::EVENT_CHANNEL.receive().await;
        let msg = elle_rpc_icd::LogMsg { level, code };
        let _ = sender
            .publish::<elle_rpc_icd::LogTopic>(VarSeq::Seq2(seq), &msg)
            .await;
        seq = seq.wrapping_add(1);
    }
}

/// RPC server task using postcard-RPC over RTT with define_dispatch!
#[cfg(feature = "rpc-control")]
#[embassy_executor::task]
async fn rpc_server_task(
    spawner: Spawner,
    aon_timer: &'static mut embassy_rp::aon_timer::AonTimer<'static>,
) {
    use elle_system::rpc::ElleWireSpawn;
    use postcard_rpc::server::{Dispatch, Server};
    use rpc_handlers::RPC_CMD_CHANNEL;

    info!("Core0: RPC server task starting");

    // Initialize RTT channels for RPC
    let channels = init_rtt_rpc();

    // RX buffer for incoming messages
    static RX_BUF: StaticCell<[u8; 1024]> = StaticCell::new();
    let rx_buf: &'static mut [u8] = RX_BUF.init([0u8; 1024]);

    // Create dispatch context with command channel sender
    let context = rpc_app::RpcContext {
        cmd_sender: RPC_CMD_CHANNEL.sender(),
        aon_timer,
    };

    // Create dispatcher via define_dispatch!-generated type
    let dispatcher = rpc_app::ElleApp::new(context, ElleWireSpawn);
    let vkk = dispatcher.min_key_len();

    // Create and run the server
    let mut server = Server::new(channels.tx, channels.rx, rx_buf, dispatcher, vkk);

    let sender = server.sender();
    spawner.spawn(log_publisher_task(sender).unwrap());

    info!("Core0: RPC server ready, waiting for commands");

    loop {
        // run() returns on fatal error; just restart
        let _err = server.run().await;
    }
}
