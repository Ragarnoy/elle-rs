/// Default I2C 7-bit address for the MMC5616WA.
pub const DEFAULT_ADDRESS: u8 = 0x30;

// --- Output registers ---
pub const XOUT0: u8 = 0x00;
pub const XOUT1: u8 = 0x01;
pub const YOUT0: u8 = 0x02;
pub const YOUT1: u8 = 0x03;
pub const ZOUT0: u8 = 0x04;
pub const ZOUT1: u8 = 0x05;
pub const XOUT2: u8 = 0x06;
pub const YOUT2: u8 = 0x07;
pub const ZOUT2: u8 = 0x08;
pub const TOUT: u8 = 0x09;

// --- Status ---
pub const STATUS1: u8 = 0x18;

// --- Control ---
pub const ODR: u8 = 0x1A;
pub const CTRL0: u8 = 0x1B;
pub const CTRL1: u8 = 0x1C;
pub const CTRL2: u8 = 0x1D;
/// Self-test thresholds, X/Y/Z (write-only, p. 15).
pub const ST_X_TH: u8 = 0x1E;
/// Factory self-test values, X/Y/Z (p. 16).
pub const ST_X: u8 = 0x27;

// --- Identity ---
pub const CHIP_ID: u8 = 0x21;
/// Expected value for Chip ID register (Rev I).
pub const CHIP_ID_VALUE: u8 = 0xD2;
pub const PRODUCT_ID: u8 = 0x39;
/// Product ID 1 reset value (p. 16).
pub const PRODUCT_ID_VALUE: u8 = 0x11;

// --- Status1 bit masks (datasheet v1.6 p. 12) ---
/// Magnetic measurement done (I3C IBI flag). Cleared by a new Take Measurement
/// command, by reading the data registers, or by reading Status1: the right
/// flag for polling a one-shot measurement.
pub const MEAS_M_DONE_INT: u8 = 1 << 0;
/// Temperature measurement done (I3C IBI flag), cleared like
/// [`MEAS_M_DONE_INT`].
pub const MEAS_T_DONE_INT: u8 = 1 << 1;
/// OTP memory read successfully (power-up, software reset).
pub const OTP_READ_DONE: u8 = 1 << 4;
/// Self-test signal: stays low once the device passes the self-test.
pub const SAT_SENSOR: u8 = 1 << 5;
/// A magnetic measurement is done and unread; cleared only by reading the
/// data registers. (These masks used to name bits 0 and 1 `MEAS_M_DONE` /
/// `MEAS_T_DONE`.)
pub const MEAS_M_DONE: u8 = 1 << 6;
/// A temperature measurement is done and unread; cleared only by reading the
/// temperature register.
pub const MEAS_T_DONE: u8 = 1 << 7;

// --- Ctrl0 (Internal Control 0) bit masks ---
// Datasheet v1.7 (2025-12-22), p. 13. Bit 2 (Start_MDT) is factory use only.
pub const TM_M: u8 = 1 << 0;
pub const TM_T: u8 = 1 << 1;
pub const DO_SET: u8 = 1 << 3;
pub const DO_RESET: u8 = 1 << 4;
pub const AUTO_SR_EN: u8 = 1 << 5;
/// Automatic self-test (thresholds in 0x1E..0x20 must be set first).
pub const AUTO_ST_EN: u8 = 1 << 6;
/// Start computing the continuous-mode measurement period from ODR. Must be
/// set before continuous mode starts. (Was defined as bit 6, which is
/// `AUTO_ST_EN`: continuous mode never picked up the ODR and ran at ~1 Hz.)
pub const CMM_FREQ_EN: u8 = 1 << 7;

/// Mask for self-clearing bits in Ctrl0 that must be cleared from the shadow
/// after a write to prevent accidental re-triggering on subsequent RMW.
pub const CTRL0_SELF_CLEARING: u8 = TM_M | TM_T | DO_SET | DO_RESET | AUTO_ST_EN | CMM_FREQ_EN;

// --- Ctrl1 (Internal Control 1) bit masks ---
pub const BW0: u8 = 1 << 0;
pub const BW1: u8 = 1 << 1;
pub const BW_MASK: u8 = BW0 | BW1;
pub const SW_RESET: u8 = 1 << 7;

// --- Ctrl2 (Internal Control 2) bit masks ---
pub const CMM_EN: u8 = 1 << 4;

// --- Sensor constants ---
/// Null field output for 20-bit mode (2^19).
pub const NULL_FIELD_OUTPUT: i32 = 524_288;
/// Sensitivity: counts per Gauss (20-bit mode).
pub const COUNTS_PER_GAUSS: f32 = 16384.0;
/// Number of bytes in a magnetic data burst read (Xout0..Zout2).
pub const MAG_DATA_LEN: usize = 9;
