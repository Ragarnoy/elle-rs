#![no_std]
#![no_main]
#![allow(clippy::too_many_arguments)] // embassy task macros generate wrapper fns

//! Firmware for the Elle Dart single-engine flying wing (RP2350, Embassy async)

// Feature gate guards
#[cfg(all(feature = "defmt-logging", feature = "rpc-control"))]
compile_error!("defmt-logging and rpc-control are mutually exclusive (both define _SEGGER_RTT)");

#[cfg(feature = "defmt-logging")]
use defmt_rtt as _;

use defmt::info;
use elle_config::profile::FLASH_SIZE;
use elle_hardware::crsf::{CrsfReceiver, crsf_receiver_task, crsf_uart_config};
use elle_hardware::imu::LED_COMMAND_CHANNEL;
use elle_hardware::led::{LedPattern, StatusLed, colors};
use elle_hardware::{
    dshot::dshot_single_task,
    pwm::{PwmOutputs, PwmPins},
};
use elle_system::{SUP_LED_READY, TimingMeasurement, supervisor_task, update_led_timing};
use embassy_executor::Spawner;
use embassy_rp::aon_timer::{AlarmWakeMode, AonTimer, ClockSource, Config as AonConfig};
use embassy_rp::clocks::{ClockConfig, CoreVoltage};
use embassy_rp::executor::{Executor, InterruptExecutor};
use embassy_rp::interrupt;
use embassy_rp::interrupt::{InterruptExt, Priority};
use embassy_rp::mode::Async;
use embassy_rp::multicore::{Stack, spawn_core1};
use embassy_rp::peripherals::{DMA_CH2, PIN_10, PIO0, PIO1, UART0, UART1};
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
use static_cell::StaticCell;

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

/// DShot runs on its own interrupt-mode executor (SWI_IRQ_0, priority P2), so a
/// long poll on the thread executor (control loop, SD, GNSS, RPC) can't hold up
/// its 1 kHz frames. Everything it shares with Core 0 tasks is behind a
/// `CriticalSectionRawMutex` (`DSHOT_THROTTLE`, `BEEP_SIGNAL`, `ENGINE_CACHE`),
/// and it only waits on Core 0 wakers (PIO, timer). Flash operations run with
/// interrupts off, so DShot still pauses during them — they only happen disarmed.
static EXECUTOR_DSHOT: InterruptExecutor = InterruptExecutor::new();

#[interrupt]
unsafe fn SWI_IRQ_0() {
    let started = embassy_time::Instant::now();
    // SAFETY: the SWI_IRQ_0 handler, and EXECUTOR_DSHOT is started in main()
    // before this interrupt is unmasked.
    unsafe { EXECUTOR_DSHOT.on_interrupt() }
    elle_hardware::timing::DSHOT_EXEC_TIME.record(started.elapsed().as_micros() as u32);
}

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
    spawner.spawn(elle_app::tasks::flash_manager_task(flash).unwrap());

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
    // Priority must be set before start(), which unmasks the interrupt.
    interrupt::SWI_IRQ_0.set_priority(Priority::P2);
    let dshot_spawner = EXECUTOR_DSHOT.start(interrupt::SWI_IRQ_0);
    dshot_spawner.spawn(dshot_single_task(engine).unwrap());

    #[cfg(feature = "rpc-control")]
    {
        info!("Core0: Starting RPC server");
        spawner.spawn(elle_app::tasks::rpc_server_task(spawner, aon_ref).unwrap());
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
                    elle_app::tasks::imu_task(
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
    elle_app::boot::run(pwm, Watchdog::new(p.WATCHDOG), epoch_ms).await
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
