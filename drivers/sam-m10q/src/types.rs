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

/// A decoded NMEA sentence.
///
/// Parsing is deferred: [`parse`](Self::parse) runs the `nmea` crate on demand.
/// `nmea::ParseResult` is a large enum, so storing it inline would bloat every
/// `Frame` — including UBX ones — and would pay the parsing cost even for
/// callers that only want [`raw`](Self::raw).
#[derive(Debug)]
pub struct NmeaFrame<'a> {
    /// Raw sentence bytes (including '$' through '*XX' but excluding \r\n).
    pub raw: &'a [u8],
}

impl NmeaFrame<'_> {
    /// Parse the sentence, if the `nmea` crate recognizes its type.
    ///
    /// Returns `None` for a sentence type the crate does not know, or one whose
    /// body it cannot parse. The `*hh` checksum has already been verified by the
    /// decoder before a frame is emitted.
    #[must_use]
    pub fn parse(&self) -> Option<nmea::ParseResult> {
        core::str::from_utf8(self.raw)
            .ok()
            .and_then(|s| nmea::parse_bytes(s.as_bytes()).ok())
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for NmeaFrame<'_> {
    fn format(&self, fmt: defmt::Formatter<'_>) {
        defmt::write!(fmt, "NmeaFrame {{ raw: {:?} }}", self.raw);
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
