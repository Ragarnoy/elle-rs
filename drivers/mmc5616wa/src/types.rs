use crate::registers::{COUNTS_PER_GAUSS, NULL_FIELD_OUTPUT};

/// Raw 20-bit unsigned magnetic data from the sensor.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RawMagData {
    pub x: u32,
    pub y: u32,
    pub z: u32,
}

/// Signed magnetic data in counts (null field subtracted).
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MagData {
    pub x: i32,
    pub y: i32,
    pub z: i32,
}

/// Measurement bandwidth setting, controlling the filter and measurement time.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Bandwidth {
    /// BW=00: 6.6 ms measurement time
    Bw00,
    /// BW=01: 3.5 ms measurement time
    Bw01,
    /// BW=10: 2.0 ms measurement time
    Bw10,
    /// BW=11: 1.2 ms measurement time
    Bw11,
}

impl Bandwidth {
    /// Returns the register bits for BW1:BW0.
    #[must_use]
    pub fn bits(self) -> u8 {
        match self {
            Self::Bw00 => 0b00,
            Self::Bw01 => 0b01,
            Self::Bw10 => 0b10,
            Self::Bw11 => 0b11,
        }
    }

    /// Returns the typical measurement time in microseconds for this bandwidth.
    #[must_use]
    pub fn measurement_time_us(self) -> u32 {
        match self {
            Self::Bw00 => 6600,
            Self::Bw01 => 3500,
            Self::Bw10 => 2000,
            Self::Bw11 => 1200,
        }
    }
}

/// Reconstruct a 20-bit unsigned value from three output register bytes.
///
/// - `out0`: high byte (bits 19:12)
/// - `out1`: mid byte (bits 11:4)
/// - `out2`: low nibble in upper 4 bits (bits 3:0)
#[must_use]
pub fn reconstruct_20bit(out0: u8, out1: u8, out2: u8) -> u32 {
    ((out0 as u32) << 12) | ((out1 as u32) << 4) | ((out2 as u32) >> 4)
}

/// Convert a raw 20-bit unsigned value to signed counts by subtracting the null field offset.
#[must_use]
pub fn to_signed(raw: u32) -> i32 {
    raw as i32 - NULL_FIELD_OUTPUT
}

/// Convert signed counts to Gauss.
#[must_use]
pub fn to_gauss(signed: i32) -> f32 {
    signed as f32 / COUNTS_PER_GAUSS
}

/// Convert the raw Tout register value to degrees Celsius.
///
/// Formula: T(°C) = -75.0 + 0.8 * Tout
#[must_use]
pub fn tout_to_celsius(tout: u8) -> f32 {
    -75.0 + 0.8 * tout as f32
}
