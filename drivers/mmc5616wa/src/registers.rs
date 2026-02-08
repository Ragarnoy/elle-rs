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

// --- Identity ---
pub const CHIP_ID: u8 = 0x21;
/// Expected value for Chip ID register (Rev I).
pub const CHIP_ID_VALUE: u8 = 0xD2;
pub const PRODUCT_ID: u8 = 0x39;

// --- Status1 bit masks ---
pub const MEAS_M_DONE: u8 = 0x01;
pub const MEAS_T_DONE: u8 = 0x02;

// --- Ctrl0 (Internal Control 0) bit masks ---
pub const TM_M: u8 = 1 << 0;
pub const TM_T: u8 = 1 << 1;
pub const DO_SET: u8 = 1 << 3;
pub const DO_RESET: u8 = 1 << 4;
pub const AUTO_SR_EN: u8 = 1 << 5;
pub const CMM_FREQ_EN: u8 = 1 << 6;

/// Mask for self-clearing bits in Ctrl0 that must be cleared from the shadow
/// after a write to prevent accidental re-triggering on subsequent RMW.
pub const CTRL0_SELF_CLEARING: u8 = TM_M | TM_T | DO_SET | DO_RESET;

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
