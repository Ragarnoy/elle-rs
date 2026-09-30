//! Labels for the firmware's numeric event codes (`LogTopic`, ULog `log_event`).
//!
//! One line per `EVT_*` constant in `elle-hardware/src/event.rs`; add one for
//! every new code. `elle_log.py` reads its labels from this file.

/// A short description of an event code ("unknown" for codes not listed).
pub const fn label(code: u16) -> &'static str {
    match code {
        // GNSS (1–9)
        1 => "GNSS: first fix",
        2 => "GNSS: periodic update",
        3 => "GNSS: UART error",
        4 => "GNSS: config rejected (NAK)",
        5 => "GNSS: NAV-PVT acquired",
        6 => "GNSS: PVT stale, NMEA fallback",
        7 => "GNSS: 115200 baud, 5Hz",
        8 => "GNSS: baud switch failed, 9600",
        140 => "GNSS: config key unanswered",
        141 => "GNSS: config only partly applied",
        9 => "GNSS: NO DATA from module",
        // Safety (10–19)
        10 => "Motors ARMED",
        11 => "Motors DISARMED",
        12 => "EMERGENCY STOP",
        13 => "RC: signal warning",
        14 => "RC: SIGNAL LOST",
        15 => "RC: signal restored",
        16 => "Kill switch ENGAGED",
        17 => "Kill switch released",
        18 => "Arm refused: throttle not at zero",
        // CRSF telemetry TX (20–29)
        20 => "CRSF TX: telemetry started",
        21 => "CRSF TX: first second OK",
        22 => "CRSF TX: UART write error",
        23 => "CRSF TX: running (periodic)",
        // ULog (30–39)
        30 => "ULog: recording started",
        31 => "ULog: init failed",
        33 => "ULog: recording stopped",
        34 => "ULog: flash erased",
        // IMU / sensors (40–49)
        40 => "IMU: init failed",
        41 => "IMU: FIFO overflow",
        42 => "IMU: read errors",
        43 => "Magnetometer: init failed",
        44 => "Barometer: init failed",
        45 => "IMU: caught up queued samples",
        46 => "IMU: gyro bias measured",
        47 => "IMU: gyro bias failed (keep still at boot)",
        48 => "I2C0 error: mag and baro disabled (AHRS 6-DOF)",
        // CRSF receiver (50–59)
        50 => "CRSF RX: first frame received",
        51 => "CRSF RX: UART error",
        // Flash storage (60–69)
        61 => "Flash: ULog erase failed",
        62 => "Flash: ULog write timeout",
        63 => "Flash: command refused while armed (disarm first)",
        // Supervisor (70–79)
        70 => "Supervisor: Core1 unhealthy",
        71 => "Supervisor: Core1 restored",
        // Flight state (80–89)
        80 => "Attitude data stale",
        81 => "ULog: recording started (auto, SD)",
        // Autotune (90–99)
        90 => "Autotune: started",
        91 => "Autotune: complete",
        92 => "Autotune: aborted (RC)",
        93 => "Autotune: safety abort",
        94 => "Autotune: result rejected (gains restored)",
        // PID profile persistence (100–109)
        100 => "PID: saved to flash",
        101 => "PID: save failed",
        102 => "PID: loaded from flash",
        103 => "PID: no saved profile",
        // Mag calibration (110–119)
        110 => "Mag cal: started",
        111 => "Mag cal: complete",
        112 => "Mag cal: failed",
        113 => "Mag cal: saved to flash",
        114 => "Mag cal: cleared",
        115 => "Mag cal: loaded from flash",
        116 => "Mag cal: no saved cal",
        // Tap detection (120–129)
        120 => "IMU: double-tap detected (calibration gesture)",
        // Heading hold (130–139)
        130 => "Heading hold: engaged",
        131 => "Heading hold: disengaged",
        132 => "Heading hold: target set",
        // Level calibration (150–159)
        150 => "Level cal: started",
        151 => "Level cal: complete",
        152 => "Level cal: failed (moved, or refused while armed)",
        153 => "Level cal: failed (tilt beyond limit)",
        154 => "Level cal: saved to flash",
        155 => "Level cal: flash save failed",
        156 => "Level cal: cleared",
        157 => "Level cal: loaded from flash",
        158 => "Level cal: no saved cal",
        // ESC link (160–169)
        160 => "ESC left: stopped replying (power loss or restart?)",
        161 => "ESC right: stopped replying (power loss or restart?)",
        162 => "ESC left: spin direction + EDT re-sent (reappeared, or no EDT)",
        163 => "ESC right: spin direction + EDT re-sent (reappeared, or no EDT)",
        164 => "ESC left: still no EDT after retries, spin direction unconfirmed",
        165 => "ESC right: still no EDT after retries, spin direction unconfirmed",
        _ => "unknown",
    }
}
