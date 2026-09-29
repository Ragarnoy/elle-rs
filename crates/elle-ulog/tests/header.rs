//! The ULog header must fit the writer's 4 KB buffer: `ULogLogger::initialize`
//! writes the definitions, info records and every subscription into it before
//! the first flush, and an overflow fails initialisation, which records nothing
//! at all.
//!
//!   cargo test -p elle-ulog --target x86_64-unknown-linux-gnu

use elle_ulog::*;
use embassy_time::Instant;

/// Room that must stay free, so adding a message fails here, not on the aircraft.
const SPARE: usize = 256;
const BUFFER_SIZE: usize = 4096;

#[test]
fn header_with_every_subscription_fits() {
    let mut w = ULogWriter::new();
    w.initialize(Instant::from_micros(0)).unwrap();
    // The longest values the firmware passes (elle-hardware ULogLogger::initialize).
    w.write_definitions("ELLE-RS", "RP2350-XFly-Eagle", "0.0.0-dev", u64::MAX)
        .unwrap();
    let names = [
        AttitudeMessage::NAME,
        CommandsMessage::NAME,
        StatusMessage::NAME,
        BarometerMessage::NAME,
        MagnetometerMessage::NAME,
        GnssMessage::NAME,
        EngineMessage::NAME,
        LogEventMessage::NAME,
        AutotuneMessage::NAME,
        ControllerMessage::NAME,
        PidGainsMessage::NAME,
        EscHealthMessage::NAME,
        Core1LoadMessage::NAME,
        LoopStagesMessage::NAME,
        NavMessage::NAME,
        ImuRawMessage::NAME,
        ImuRawMagMessage::NAME,
        ImuRawCtxMessage::NAME,
        ImuRawFixMessage::NAME,
    ];
    for name in names {
        w.add_subscription(name).unwrap();
    }
    let used = w.buffer().len();
    assert!(
        used + SPARE <= BUFFER_SIZE,
        "ULog header is {used} B of {BUFFER_SIZE}: less than {SPARE} B spare"
    );
}
