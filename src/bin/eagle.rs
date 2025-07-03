#![no_std]
#![no_main]

//! Firmware for the XFly Eagle Testbed

use defmt::info;
use defmt_rtt as _;
use panic_probe as _;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::{PIO0, UART1};
use embassy_rp::pio::{InterruptHandler as PioIrqHandler, Pio};
use embassy_rp::uart::InterruptHandler as UartIrqHandler;

use elle::hardware::{pwm::{PwmOutputs, PwmPins}, sbus::SbusReceiver};
use elle::system::FlightController;

bind_interrupts!(
    struct Irqs {
        PIO0_IRQ_0 => PioIrqHandler<PIO0>;
        UART1_IRQ => UartIrqHandler<UART1>;
    }
);

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let mut pwm_pins = PwmPins {
        elevon_left: p.PIN_16,
        elevon_right: p.PIN_17,
        engine_left: p.PIN_10,
        engine_right: p.PIN_11,
    };
    
    // Initialize hardware
    let Pio { mut common, sm0, sm1, sm2, sm3, .. } = Pio::new(p.PIO0, Irqs);


    let mut pwm = PwmOutputs::new(&mut common, sm0, sm1, sm2, sm3, &mut pwm_pins);
    pwm.set_safe_positions();

    // Initialize SBUS receiver
    let mut sbus = SbusReceiver::new(p.UART1, p.PIN_5, Irqs, p.DMA_CH0);

    // Create flight controller
    let mut fc = FlightController::new(pwm);

    // Initialize ESCs
    fc.initialize_escs().await;

    // Main control loop
    let mut loop_counter = 0u32;

    loop {
        if let Some(packet) = sbus.read_packet().await {
            fc.update(&packet);
        }

        fc.check_failsafe();

        // Status output
        loop_counter += 1;
        if loop_counter >= 10000 {
            loop_counter = 0;
            if fc.is_armed() {
                info!("Status: ARMED");
            } else if fc.is_failsafe() {
                info!("Status: FAILSAFE");
            } else {
                info!("Status: DISARMED");
            }
        }
    }
}