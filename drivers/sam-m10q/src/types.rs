/// Maximum internal buffer size for a single frame.
pub const MAX_FRAME_SIZE: usize = 256;

/// A decoded frame from the GNSS module.
#[derive(Debug)]
pub enum Frame<'a> {
    /// A UBX binary protocol frame.
    Ubx(UbxFrame<'a>),
    /// An NMEA ASCII sentence.
    Nmea(NmeaFrame<'a>),
}

/// A decoded UBX binary frame.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct UbxFrame<'a> {
    /// UBX message class.
    pub class: u8,
    /// UBX message ID.
    pub id: u8,
    /// Payload bytes (excluding sync, header, and checksum).
    pub payload: &'a [u8],
}

/// A decoded NMEA sentence with optional parsed data.
#[derive(Debug)]
pub struct NmeaFrame<'a> {
    /// Raw sentence bytes (including '$' through '*XX' but excluding \r\n).
    pub raw: &'a [u8],
    /// Parsed sentence data, if the `nmea` crate recognized the sentence type.
    pub parsed: Option<nmea::ParseResult>,
}

#[cfg(feature = "defmt")]
impl defmt::Format for NmeaFrame<'_> {
    fn format(&self, fmt: defmt::Formatter<'_>) {
        defmt::write!(fmt, "NmeaFrame {{ raw: {:?}, parsed: {} }}", self.raw, self.parsed.is_some());
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for Frame<'_> {
    fn format(&self, fmt: defmt::Formatter<'_>) {
        match self {
            Frame::Ubx(f) => defmt::write!(fmt, "Frame::Ubx({:?})", f),
            Frame::Nmea(f) => defmt::write!(fmt, "Frame::Nmea({:?})", f),
        }
    }
}
