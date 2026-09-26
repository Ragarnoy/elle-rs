#![no_std]
#![no_main]
#![allow(clippy::too_many_arguments)] // embassy task macros generate wrapper fns

//! Firmware for the XFly Eagle Testbed with WS2812B LED

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

use elle_control::autotune::SavedGains;
use elle_hardware::imu::{IMU_STATUS, Imu, LED_COMMAND_CHANNEL};
use elle_hardware::led::{LedPattern, StatusLed, colors};
use elle_hardware::{
    dshot::dshot_task,
    flash::SequentialFlashManager,
    pwm::{PwmOutputs, PwmPins},
};

use elle_hardware::ULogLogger;

use elle_hardware::crsf::{CrsfReceiver, crsf_receiver_task, crsf_uart_config};

#[cfg(feature = "rpc-control")]
use elle_system::rpc::init_rtt_rpc;

use elle_system::{
    FlightController, SUP_FC_READY, SUP_IMU_READY, SUP_LED_READY, SUP_START_FC, SUP_START_IMU,
    TimingMeasurement, supervisor_task, update_led_timing,
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
    DMA_CH2, I2C0, PIN_0, PIN_1, PIN_2, PIN_3, PIN_5, PIN_8, PIN_9, PIN_10, PIO0, PIO1, PIO2, SPI0,
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
use embassy_time::{Duration, Timer};
use panic_probe as _;

#[cfg(feature = "rpc-control")]
use elle_app::{rpc_app, rpc_handlers};
use static_cell::StaticCell;

bind_interrupts!(
    struct Irqs {
        PIO0_IRQ_0 => PioIrqHandler<PIO0>;
        PIO1_IRQ_0 => PioIrqHandler<PIO1>;
        PIO2_IRQ_0 => PioIrqHandler<PIO2>;
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
        common,
        sm2: led_sm,
        ..
    } = Pio::new(p.PIO0, Irqs);

    spawner.spawn(led_task(common, led_sm, p.DMA_CH2, Irqs, p.PIN_10).unwrap());

    // Setup DShot300 engines on PIO1 (left, PIN_11) and PIO2 (right, PIN_15)
    info!("Core0: Setting up DShot300 engines");
    // One engine per PIO block. The program is only borrowed while the driver is
    // built; the block's `Common` and unused state machines drop here, as they did
    // inside the pre-0.5 constructor.
    let Pio {
        mut common, sm0, ..
    } = Pio::new(p.PIO1, Irqs);
    let prog = embassy_dshot::rp::BidirDshotProgram::new(&mut common);
    let engine_left = embassy_dshot::rp::BidirDshotPio::new(
        sm0,
        &mut common,
        p.PIN_14,
        &prog,
        embassy_dshot::rp::DshotSpeed::DShot300,
    );
    let Pio {
        mut common, sm0, ..
    } = Pio::new(p.PIO2, Irqs);
    let prog = embassy_dshot::rp::BidirDshotProgram::new(&mut common);
    let engine_right = embassy_dshot::rp::BidirDshotPio::new(
        sm0,
        &mut common,
        p.PIN_11,
        &prog,
        embassy_dshot::rp::DshotSpeed::DShot300,
    );
    spawner.spawn(dshot_task(engine_left, engine_right).unwrap());

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
    let ulog_logger = ULogLogger::new();

    // Boot-time PID profile load from flash
    {
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

    #[cfg(not(feature = "rpc-control"))]
    elle_app::flight::run_flight(fc, ulog_logger, epoch_ms).await;
    #[cfg(feature = "rpc-control")]
    elle_app::rpc::run_rpc(fc, ulog_logger, epoch_ms).await;
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
