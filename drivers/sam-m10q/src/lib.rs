//! Driver for the u-blox SAM-M10Q GNSS module.
//!
//! `no_std` compatible driver using `embedded-io` (UART) and `embedded-hal` (GPIO, delay) traits.
//! Handles streaming decode of interleaved UBX binary and NMEA ASCII frames on the same UART.
//! NMEA sentence parsing is delegated to the `nmea` crate.

#![no_std]

pub mod asynch;
pub mod decoder;
pub mod error;
pub mod types;
pub mod ubx;

pub use nmea;

use decoder::{Decoder, FeedResult};
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use embedded_io::{Read, Write};
use error::Error;
use types::Frame;

/// Duration in milliseconds to hold RESET_N low.
///
/// The datasheet requires at least 1 ms; 2 ms gives margin.
const RESET_PULSE_MS: u32 = 2;

/// Duration in milliseconds to wait after releasing RESET_N before reading.
const RESET_SETTLE_MS: u32 = 500;

/// Driver for the u-blox SAM-M10Q GNSS module over UART.
///
/// Generic over `U` which must implement `embedded_io::Read` and `embedded_io::Write`.
pub struct SamM10q<U> {
    uart: U,
    decoder: Decoder,
}

impl<U> SamM10q<U> {
    /// Create a new driver wrapping the given UART peripheral.
    pub fn new(uart: U) -> Self {
        Self {
            uart,
            decoder: Decoder::new(),
        }
    }

    /// Consume the driver and return the underlying UART peripheral.
    pub fn destroy(self) -> U {
        self.uart
    }

    /// Borrow the underlying UART peripheral.
    pub fn uart(&self) -> &U {
        &self.uart
    }

    /// Mutably borrow the underlying UART peripheral.
    pub fn uart_mut(&mut self) -> &mut U {
        &mut self.uart
    }
}

impl<U: Read + Write> SamM10q<U> {
    /// Read one byte from the UART and feed it to the decoder.
    ///
    /// Returns `Ok(Some(Frame))` when a complete frame has been decoded,
    /// `Ok(None)` when more data is needed, or `Err` on UART or decode errors.
    ///
    /// The returned `Frame` borrows from the decoder's internal buffer and
    /// must be consumed before calling `poll()` again.
    pub fn poll(&mut self) -> Result<Option<Frame<'_>>, Error<U::Error>> {
        let mut byte = [0u8; 1];
        match self.uart.read(&mut byte) {
            // A zero-length read means the buffer is empty, not that anything
            // failed — the caller should simply poll again.
            Ok(0) => return Ok(None),
            Ok(_) => {}
            Err(e) => return Err(Error::Uart(e)),
        }

        match self.decoder.feed(byte[0]) {
            FeedResult::Pending => Ok(None),
            FeedResult::FrameReady => Ok(self.decoder.take_frame()),
            FeedResult::Error(
                decoder::DecodeError::UbxChecksum | decoder::DecodeError::NmeaChecksum,
            ) => Err(Error::Checksum),
            FeedResult::Error(decoder::DecodeError::FrameTooLarge) => Err(Error::FrameTooLarge),
            FeedResult::Error(decoder::DecodeError::InvalidSync) => {
                // Invalid sync is non-fatal — just means garbage, treat as no frame
                Ok(None)
            }
        }
    }

    /// Block until a complete frame is decoded.
    ///
    /// Reads bytes from the UART in a loop, feeding them to the decoder
    /// until a complete frame is available. Checksum errors are silently
    /// skipped (the next valid frame is returned instead).
    pub fn read_frame(&mut self) -> Result<Frame<'_>, Error<U::Error>> {
        loop {
            let mut byte = [0u8; 1];
            match self.uart.read(&mut byte) {
                Ok(0) => return Err(Error::Timeout),
                Ok(_) => {}
                Err(e) => return Err(Error::Uart(e)),
            }

            match self.decoder.feed(byte[0]) {
                FeedResult::Pending => continue,
                FeedResult::FrameReady => break,
                FeedResult::Error(
                    decoder::DecodeError::UbxChecksum | decoder::DecodeError::NmeaChecksum,
                ) => continue,
                FeedResult::Error(decoder::DecodeError::FrameTooLarge) => {
                    return Err(Error::FrameTooLarge);
                }
                FeedResult::Error(decoder::DecodeError::InvalidSync) => continue,
            }
        }

        // `FrameReady` guarantees the decoder holds a frame, so the `None` arm
        // is unreachable — it is mapped to an error rather than a panic.
        self.decoder.take_frame().ok_or(Error::Timeout)
    }

    /// Send a UBX command to the module.
    ///
    /// Builds the complete UBX frame (sync bytes, header, payload, checksum)
    /// on the stack and writes it to the UART.
    pub fn send_ubx(&mut self, class: u8, id: u8, payload: &[u8]) -> Result<(), Error<U::Error>> {
        let mut buf = [0u8; types::MAX_FRAME_SIZE + ubx::HEADER_SIZE + ubx::CHECKSUM_SIZE];
        let len = ubx::build_frame(&mut buf, class, id, payload).ok_or(Error::FrameTooLarge)?;
        self.uart.write_all(&buf[..len])?;
        Ok(())
    }

    /// Send an NMEA sentence to the module.
    ///
    /// `sentence` should be a complete sentence including the leading '$' and
    /// checksum (e.g., `b"$PUBX,40,GGA,0,1,0,0,0,0*5A"`). A `\r\n` terminator
    /// is appended if not already present.
    pub fn send_nmea(&mut self, sentence: &[u8]) -> Result<(), Error<U::Error>> {
        self.uart.write_all(sentence)?;
        if !sentence.ends_with(b"\r\n") {
            self.uart.write_all(b"\r\n")?;
        }
        Ok(())
    }

    /// Perform a hardware reset via the RESET_N pin.
    ///
    /// Holds RESET_N low for `RESET_PULSE_MS` (the datasheet requires at least
    /// 1 ms), releases it, then waits `RESET_SETTLE_MS` for the module to boot
    /// before returning — reading immediately after release yields garbage.
    /// Also resets the internal decoder state.
    ///
    /// # Errors
    ///
    /// Returns [`Error::Pin`] if driving RESET_N fails.
    pub fn reset<P: OutputPin, D: DelayNs>(
        &mut self,
        pin: &mut P,
        delay: &mut D,
    ) -> Result<(), Error<U::Error>> {
        // RESET_N is active low.
        pin.set_low().map_err(|_| Error::Pin)?;
        delay.delay_ms(RESET_PULSE_MS);
        pin.set_high().map_err(|_| Error::Pin)?;
        delay.delay_ms(RESET_SETTLE_MS);
        self.decoder.reset();
        Ok(())
    }
}

#[cfg(test)]
extern crate alloc;

#[cfg(test)]
mod tests {
    use alloc::vec;
    use alloc::vec::Vec;

    use super::*;
    use crate::ubx;
    use embedded_io::{ErrorKind, ErrorType};

    /// Mock UART backed by a `Vec<u8>` for reads and a `Vec<u8>` sink for writes.
    struct MockUart {
        rx_buf: Vec<u8>,
        rx_pos: usize,
        tx_buf: Vec<u8>,
    }

    impl MockUart {
        fn new(rx_data: &[u8]) -> Self {
            Self {
                rx_buf: rx_data.to_vec(),
                rx_pos: 0,
                tx_buf: Vec::new(),
            }
        }

        fn written(&self) -> &[u8] {
            &self.tx_buf
        }
    }

    #[derive(Debug)]
    struct MockError;

    impl core::fmt::Display for MockError {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            write!(f, "MockError")
        }
    }

    impl core::error::Error for MockError {}

    impl embedded_io::Error for MockError {
        fn kind(&self) -> ErrorKind {
            ErrorKind::Other
        }
    }

    impl ErrorType for MockUart {
        type Error = MockError;
    }

    impl Read for MockUart {
        fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
            if self.rx_pos >= self.rx_buf.len() {
                return Ok(0);
            }
            let n = core::cmp::min(buf.len(), self.rx_buf.len() - self.rx_pos);
            buf[..n].copy_from_slice(&self.rx_buf[self.rx_pos..self.rx_pos + n]);
            self.rx_pos += n;
            Ok(n)
        }
    }

    impl Write for MockUart {
        fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
            self.tx_buf.extend_from_slice(buf);
            Ok(buf.len())
        }

        fn flush(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }
    }

    #[test]
    fn poll_nmea_frame() {
        let data = b"$GPGGA,test*6C\r\n";
        let uart = MockUart::new(data);
        let mut gnss = SamM10q::new(uart);

        // Poll until we get a frame
        loop {
            match gnss.poll() {
                Ok(Some(Frame::Nmea(f))) => {
                    assert!(f.raw.starts_with(b"$GPGGA"));
                    break;
                }
                Ok(None) => continue,
                Ok(Some(_)) => panic!("unexpected UBX frame"),
                Err(e) => panic!("unexpected error: {:?}", e),
            }
        }
    }

    #[test]
    fn poll_ubx_frame() {
        let mut frame_data = vec![0u8; 16];
        let payload = [0x06, 0x01];
        let len =
            ubx::build_frame(&mut frame_data, ubx::class::ACK, ubx::ack::ACK, &payload).unwrap();
        let uart = MockUart::new(&frame_data[..len]);
        let mut gnss = SamM10q::new(uart);

        loop {
            match gnss.poll() {
                Ok(Some(Frame::Ubx(f))) => {
                    assert_eq!(f.class, ubx::class::ACK);
                    assert_eq!(f.id, ubx::ack::ACK);
                    assert_eq!(f.payload, &[0x06, 0x01]);
                    break;
                }
                Ok(None) => continue,
                Ok(Some(_)) => panic!("unexpected NMEA frame"),
                Err(e) => panic!("unexpected error: {:?}", e),
            }
        }
    }

    /// An empty UART is "nothing yet", not an error — `poll()` is meant to be
    /// called in a loop against a non-blocking port.
    #[test]
    fn poll_returns_none_on_empty() {
        let uart = MockUart::new(&[]);
        let mut gnss = SamM10q::new(uart);

        match gnss.poll() {
            Ok(None) => {}
            other => panic!("expected Ok(None), got {:?}", other),
        }
    }

    #[test]
    fn send_ubx_produces_valid_frame() {
        let uart = MockUart::new(&[]);
        let mut gnss = SamM10q::new(uart);

        gnss.send_ubx(
            ubx::class::CFG,
            ubx::cfg::RATE,
            &[0xE8, 0x03, 0x01, 0x00, 0x01, 0x00],
        )
        .unwrap();

        let written = gnss.uart().written();
        // Verify sync bytes
        assert_eq!(written[0], ubx::SYNC1);
        assert_eq!(written[1], ubx::SYNC2);
        // Verify class/id
        assert_eq!(written[2], ubx::class::CFG);
        assert_eq!(written[3], ubx::cfg::RATE);
        // Verify length
        assert_eq!(written[4], 6); // len_lo
        assert_eq!(written[5], 0); // len_hi
        // Verify payload
        assert_eq!(&written[6..12], &[0xE8, 0x03, 0x01, 0x00, 0x01, 0x00]);
        // Verify checksum is valid
        let (ck_a, ck_b) = ubx::checksum(&written[2..12]);
        assert_eq!(written[12], ck_a);
        assert_eq!(written[13], ck_b);
    }

    #[test]
    fn send_nmea_appends_crlf() {
        let uart = MockUart::new(&[]);
        let mut gnss = SamM10q::new(uart);

        gnss.send_nmea(b"$PUBX,40,GGA,0,1,0,0,0,0*5A").unwrap();

        let written = gnss.uart().written();
        assert!(written.ends_with(b"\r\n"));
        assert!(written.starts_with(b"$PUBX"));
    }

    #[test]
    fn send_nmea_no_double_crlf() {
        let uart = MockUart::new(&[]);
        let mut gnss = SamM10q::new(uart);

        gnss.send_nmea(b"$PUBX,40,GGA,0,1,0,0,0,0*5A\r\n").unwrap();

        let written = gnss.uart().written();
        // Should not have double \r\n — sentence already ended with \r\n
        assert_eq!(written, b"$PUBX,40,GGA,0,1,0,0,0,0*5A\r\n");
    }

    #[test]
    fn destroy_returns_uart() {
        let uart = MockUart::new(&[1, 2, 3]);
        let gnss = SamM10q::new(uart);
        let recovered = gnss.destroy();
        assert_eq!(&recovered.rx_buf, &[1, 2, 3]);
    }

    #[test]
    fn read_frame_skips_garbage() {
        let mut data = vec![0xFF, 0x00, 0x42]; // garbage
        data.extend_from_slice(b"$GPRMC,test*71\r\n");
        let uart = MockUart::new(&data);
        let mut gnss = SamM10q::new(uart);

        match gnss.read_frame() {
            Ok(Frame::Nmea(f)) => {
                assert!(f.raw.starts_with(b"$GPRMC"));
            }
            other => panic!("expected NMEA frame, got {:?}", other),
        }
    }

    #[test]
    fn gga_with_fix_parses_position() {
        // Real GGA sentence with a valid GPS fix
        let data = b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,47.0,M,,*4F\r\n";
        let uart = MockUart::new(data);
        let mut gnss = SamM10q::new(uart);

        match gnss.read_frame() {
            Ok(Frame::Nmea(f)) => {
                assert!(f.raw.starts_with(b"$GPGGA"));
                let parsed = f.parse().expect("GGA should parse");
                match parsed {
                    nmea::ParseResult::GGA(gga) => {
                        let lat = gga.latitude.expect("should have latitude");
                        let lon = gga.longitude.expect("should have longitude");
                        let alt = gga.altitude.expect("should have altitude");
                        let sats = gga.fix_satellites.expect("should have sat count");
                        let hdop = gga.hdop.expect("should have hdop");

                        // 48°07.038'N ≈ 48.1173°
                        assert!((lat - 48.1173).abs() < 0.001, "lat={lat}");
                        // 011°31.000'E ≈ 11.5167°
                        assert!((lon - 11.5167).abs() < 0.001, "lon={lon}");
                        assert!((alt - 545.4).abs() < 0.1, "alt={alt}");
                        assert_eq!(sats, 8);
                        assert!((hdop - 0.9).abs() < 0.01, "hdop={hdop}");

                        assert!(matches!(gga.fix_type, Some(nmea::sentences::FixType::Gps)));
                    }
                    other => panic!("expected GGA, got {:?}", other),
                }
            }
            other => panic!("expected NMEA frame, got {:?}", other),
        }
    }

    #[test]
    fn gga_no_fix_parses_empty() {
        // GGA sentence with no fix (what you see indoors)
        let data = b"$GPGGA,235959.00,,,,,0,00,99.99,,,,,,*67\r\n";
        let uart = MockUart::new(data);
        let mut gnss = SamM10q::new(uart);

        match gnss.read_frame() {
            Ok(Frame::Nmea(f)) => {
                assert!(f.raw.starts_with(b"$GPGGA"));
                let parsed = f.parse().expect("GGA should parse");
                match parsed {
                    nmea::ParseResult::GGA(gga) => {
                        assert!(gga.latitude.is_none());
                        assert!(gga.longitude.is_none());
                        assert!(gga.altitude.is_none());
                        assert!(matches!(
                            gga.fix_type,
                            Some(nmea::sentences::FixType::Invalid)
                        ));
                    }
                    other => panic!("expected GGA, got {:?}", other),
                }
            }
            other => panic!("expected NMEA frame, got {:?}", other),
        }
    }
}
