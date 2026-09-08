use crate::types::{Frame, MAX_FRAME_SIZE, NmeaFrame, UbxFrame};
use crate::ubx;

/// Internal decoder state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum State {
    /// Waiting for a frame start byte ('$' or 0xB5).
    Idle,
    /// Accumulating NMEA sentence bytes after '$'.
    NmeaBody,
    /// Received UBX SYNC1 (0xB5), expecting SYNC2 (0x62).
    UbxSync2,
    /// Receiving UBX header bytes (class, id, len_lo, len_hi).
    UbxHeader { header_idx: u8 },
    /// Receiving UBX payload bytes.
    UbxPayload,
    /// Receiving UBX checksum bytes (ck_a, ck_b).
    UbxChecksum { ck_idx: u8 },
    /// Discarding a known number of bytes belonging to a frame we cannot buffer.
    ///
    /// Used after an oversized UBX payload length is announced: the declared
    /// payload plus checksum are dropped without being rescanned, so payload
    /// bytes that happen to equal `$` or `0xB5` cannot start a phantom frame.
    Skipping { remaining: u32 },
    /// Discarding bytes until the next `\n`, used for over-long NMEA sentences.
    SkipToNewline,
}

/// Tracks which frame type was last decoded.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum FrameType {
    None,
    Nmea,
    Ubx,
}

/// Result of feeding a single byte to the decoder.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FeedResult {
    /// More data needed.
    Pending,
    /// A complete frame is ready — call `take_frame()`.
    FrameReady,
    /// A decode error occurred. The decoder has been reset to Idle.
    Error(DecodeError),
}

/// Decode errors that can occur within the state machine.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DecodeError {
    /// UBX checksum mismatch.
    UbxChecksum,
    /// Frame exceeded the internal buffer.
    FrameTooLarge,
    /// Invalid UBX sync byte (0xB5 not followed by 0x62).
    InvalidSync,
    /// NMEA `*hh` checksum mismatch, or a malformed checksum field.
    NmeaChecksum,
}

/// Streaming decoder for interleaved UBX and NMEA frames.
///
/// Feed bytes one at a time via `feed()`. When `FeedResult::FrameReady` is
/// returned, call `take_frame()` to borrow the decoded frame data.
pub struct Decoder {
    state: State,
    buf: [u8; MAX_FRAME_SIZE],
    pos: usize,
    last_frame: FrameType,
    // UBX header fields (populated during UbxHeader state)
    ubx_class: u8,
    ubx_id: u8,
    ubx_payload_len: u16,
    // UBX checksum accumulators
    ubx_ck_a: u8,
    ubx_ck_b: u8,
    // Received checksum bytes
    ubx_rx_ck_a: u8,
}

impl Decoder {
    /// Create a new decoder in the Idle state.
    #[must_use]
    pub fn new() -> Self {
        Self {
            state: State::Idle,
            buf: [0; MAX_FRAME_SIZE],
            pos: 0,
            last_frame: FrameType::None,
            ubx_class: 0,
            ubx_id: 0,
            ubx_payload_len: 0,
            ubx_ck_a: 0,
            ubx_ck_b: 0,
            ubx_rx_ck_a: 0,
        }
    }

    /// Reset the decoder to Idle, discarding any partial frame.
    pub fn reset(&mut self) {
        self.state = State::Idle;
        self.pos = 0;
        self.last_frame = FrameType::None;
    }

    /// Feed a single byte into the decoder.
    pub fn feed(&mut self, byte: u8) -> FeedResult {
        match self.state {
            State::Idle => self.feed_idle(byte),
            State::NmeaBody => self.feed_nmea(byte),
            State::UbxSync2 => self.feed_ubx_sync2(byte),
            State::UbxHeader { header_idx } => self.feed_ubx_header(byte, header_idx),
            State::UbxPayload => self.feed_ubx_payload(byte),
            State::UbxChecksum { ck_idx } => self.feed_ubx_checksum(byte, ck_idx),
            State::Skipping { remaining } => self.feed_skipping(remaining),
            State::SkipToNewline => self.feed_skip_to_newline(byte),
        }
    }

    /// Retrieve the last decoded frame.
    ///
    /// Returns `None` if no frame has been decoded yet — call this only after
    /// `feed()` returns `FeedResult::FrameReady`. The returned `Frame` borrows
    /// from the decoder's internal buffer, so the borrow checker enforces that
    /// it is consumed before the next call to `feed()`.
    #[must_use]
    pub fn take_frame(&self) -> Option<Frame<'_>> {
        match self.last_frame {
            FrameType::Ubx => Some(Frame::Ubx(UbxFrame {
                class: self.ubx_class,
                id: self.ubx_id,
                payload: &self.buf[..self.ubx_payload_len as usize],
            })),
            FrameType::Nmea => Some(Frame::Nmea(NmeaFrame {
                raw: &self.buf[..self.pos],
            })),
            FrameType::None => None,
        }
    }

    // --- State handlers ---

    fn feed_idle(&mut self, byte: u8) -> FeedResult {
        match byte {
            b'$' => {
                self.pos = 0;
                self.buf[0] = byte;
                self.pos = 1;
                self.state = State::NmeaBody;
                FeedResult::Pending
            }
            ubx::SYNC1 => {
                self.state = State::UbxSync2;
                FeedResult::Pending
            }
            _ => {
                // Garbage byte, stay in Idle
                FeedResult::Pending
            }
        }
    }

    fn feed_nmea(&mut self, byte: u8) -> FeedResult {
        match byte {
            b'\n' => {
                // Strip trailing \r if present
                if self.pos > 0 && self.buf[self.pos - 1] == b'\r' {
                    self.pos -= 1;
                }
                self.state = State::Idle;
                if nmea_checksum_ok(&self.buf[..self.pos]) {
                    self.last_frame = FrameType::Nmea;
                    FeedResult::FrameReady
                } else {
                    self.pos = 0;
                    FeedResult::Error(DecodeError::NmeaChecksum)
                }
            }
            _ => {
                if self.pos >= MAX_FRAME_SIZE {
                    // Drop the rest of the sentence rather than rescanning it:
                    // returning to Idle mid-sentence risks a stray byte
                    // starting a phantom frame.
                    self.state = State::SkipToNewline;
                    self.pos = 0;
                    return FeedResult::Error(DecodeError::FrameTooLarge);
                }
                self.buf[self.pos] = byte;
                self.pos += 1;
                FeedResult::Pending
            }
        }
    }

    fn feed_skipping(&mut self, remaining: u32) -> FeedResult {
        let left = remaining - 1;
        self.state = if left == 0 {
            State::Idle
        } else {
            State::Skipping { remaining: left }
        };
        FeedResult::Pending
    }

    fn feed_skip_to_newline(&mut self, byte: u8) -> FeedResult {
        if byte == b'\n' {
            self.state = State::Idle;
        }
        FeedResult::Pending
    }

    fn feed_ubx_sync2(&mut self, byte: u8) -> FeedResult {
        if byte == ubx::SYNC2 {
            // Valid sync sequence, start header
            self.ubx_class = 0;
            self.ubx_id = 0;
            self.ubx_payload_len = 0;
            self.ubx_ck_a = 0;
            self.ubx_ck_b = 0;
            self.ubx_rx_ck_a = 0;
            self.pos = 0;
            self.state = State::UbxHeader { header_idx: 0 };
            FeedResult::Pending
        } else {
            // False sync — re-feed byte through Idle
            self.state = State::Idle;
            self.feed_idle(byte)
        }
    }

    fn feed_ubx_header(&mut self, byte: u8, header_idx: u8) -> FeedResult {
        // Accumulate checksum over all header bytes
        self.ubx_ck_a = self.ubx_ck_a.wrapping_add(byte);
        self.ubx_ck_b = self.ubx_ck_b.wrapping_add(self.ubx_ck_a);

        match header_idx {
            0 => {
                self.ubx_class = byte;
                self.state = State::UbxHeader { header_idx: 1 };
            }
            1 => {
                self.ubx_id = byte;
                self.state = State::UbxHeader { header_idx: 2 };
            }
            2 => {
                self.ubx_payload_len = byte as u16;
                self.state = State::UbxHeader { header_idx: 3 };
            }
            3 => {
                self.ubx_payload_len |= (byte as u16) << 8;
                if self.ubx_payload_len as usize > MAX_FRAME_SIZE {
                    // Skip the announced payload and its checksum instead of
                    // resyncing blind — payload bytes equal to `$` or SYNC1
                    // would otherwise start a phantom frame and swallow the
                    // next genuine one.
                    self.pos = 0;
                    // + 2 for the trailing ck_a / ck_b (`ubx::CHECKSUM_SIZE`).
                    self.state = State::Skipping {
                        remaining: u32::from(self.ubx_payload_len) + 2,
                    };
                    return FeedResult::Error(DecodeError::FrameTooLarge);
                }
                if self.ubx_payload_len == 0 {
                    self.state = State::UbxChecksum { ck_idx: 0 };
                } else {
                    self.pos = 0;
                    self.state = State::UbxPayload;
                }
            }
            _ => unreachable!(),
        }
        FeedResult::Pending
    }

    fn feed_ubx_payload(&mut self, byte: u8) -> FeedResult {
        self.ubx_ck_a = self.ubx_ck_a.wrapping_add(byte);
        self.ubx_ck_b = self.ubx_ck_b.wrapping_add(self.ubx_ck_a);

        self.buf[self.pos] = byte;
        self.pos += 1;

        if self.pos >= self.ubx_payload_len as usize {
            self.state = State::UbxChecksum { ck_idx: 0 };
        }
        FeedResult::Pending
    }

    fn feed_ubx_checksum(&mut self, byte: u8, ck_idx: u8) -> FeedResult {
        match ck_idx {
            0 => {
                self.ubx_rx_ck_a = byte;
                self.state = State::UbxChecksum { ck_idx: 1 };
                FeedResult::Pending
            }
            1 => {
                self.state = State::Idle;
                if self.ubx_rx_ck_a == self.ubx_ck_a && byte == self.ubx_ck_b {
                    self.last_frame = FrameType::Ubx;
                    FeedResult::FrameReady
                } else {
                    FeedResult::Error(DecodeError::UbxChecksum)
                }
            }
            _ => unreachable!(),
        }
    }
}

/// Verify an NMEA sentence's `*hh` checksum.
///
/// `raw` is the sentence from `$` up to but excluding the line terminator.
/// Per the SAM-M10Q driver spec the checksum field is optional: a sentence
/// without a `*` is accepted. A `*` present but malformed, or a mismatch,
/// is rejected.
fn nmea_checksum_ok(raw: &[u8]) -> bool {
    let Some(star) = raw.iter().rposition(|&b| b == b'*') else {
        // No checksum field — nothing to verify.
        return true;
    };
    // Exactly two hex digits must follow the '*'.
    if raw.len() != star + 3 {
        return false;
    }
    let (Some(hi), Some(lo)) = (hex_digit(raw[star + 1]), hex_digit(raw[star + 2])) else {
        return false;
    };
    // The checksum covers the bytes between '$' and '*'.
    ubx::nmea_checksum(&raw[1..star]) == (hi << 4) | lo
}

/// Decode a single ASCII hex digit.
fn hex_digit(b: u8) -> Option<u8> {
    match b {
        b'0'..=b'9' => Some(b - b'0'),
        b'A'..=b'F' => Some(b - b'A' + 10),
        b'a'..=b'f' => Some(b - b'a' + 10),
        _ => None,
    }
}

impl Default for Decoder {
    fn default() -> Self {
        Self::new()
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

    /// Helper: feed all bytes and collect FeedResults.
    fn feed_all(decoder: &mut Decoder, data: &[u8]) -> Vec<FeedResult> {
        data.iter().map(|&b| decoder.feed(b)).collect()
    }

    /// Helper: feed all bytes, returning the index of the first FrameReady.
    fn feed_until_frame(decoder: &mut Decoder, data: &[u8]) -> Option<usize> {
        for (i, &b) in data.iter().enumerate() {
            if decoder.feed(b) == FeedResult::FrameReady {
                return Some(i);
            }
        }
        None
    }

    /// Build a valid UBX frame for testing.
    fn build_ubx(class: u8, id: u8, payload: &[u8]) -> Vec<u8> {
        let mut buf = vec![0u8; ubx::HEADER_SIZE + payload.len() + ubx::CHECKSUM_SIZE];
        ubx::build_frame(&mut buf, class, id, payload).unwrap();
        buf
    }

    #[test]
    fn simple_nmea_sentence() {
        let mut dec = Decoder::new();
        let sentence = b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,47.0,M,,*4F\r\n";
        feed_until_frame(&mut dec, sentence).expect("should produce a frame");

        match dec.take_frame().expect("frame ready") {
            Frame::Nmea(f) => {
                // raw should not contain \r\n
                assert!(f.raw.starts_with(b"$GPGGA"));
                assert!(f.raw.ends_with(b"*4F"));
                assert!(!f.raw.contains(&b'\r'));
                assert!(!f.raw.contains(&b'\n'));
            }
            _ => panic!("expected NMEA frame"),
        }
    }

    #[test]
    fn ubx_ack_frame() {
        let mut dec = Decoder::new();
        let frame = build_ubx(ubx::class::ACK, ubx::ack::ACK, &[0x06, 0x01]);
        feed_until_frame(&mut dec, &frame).expect("should produce a frame");

        match dec.take_frame().expect("frame ready") {
            Frame::Ubx(f) => {
                assert_eq!(f.class, ubx::class::ACK);
                assert_eq!(f.id, ubx::ack::ACK);
                assert_eq!(f.payload, &[0x06, 0x01]);
            }
            _ => panic!("expected UBX frame"),
        }
    }

    #[test]
    fn ubx_bad_checksum() {
        let mut dec = Decoder::new();
        let mut frame = build_ubx(ubx::class::ACK, ubx::ack::ACK, &[0x06, 0x01]);
        // Corrupt the checksum
        let last = frame.len() - 1;
        frame[last] ^= 0xFF;

        let results = feed_all(&mut dec, &frame);
        assert!(results.contains(&FeedResult::Error(DecodeError::UbxChecksum)));
    }

    #[test]
    fn garbage_before_frame() {
        let mut dec = Decoder::new();
        let mut data = vec![0xFF, 0x00, 0x42, 0x13]; // garbage
        data.extend_from_slice(b"$GPRMC,test*71\r\n");

        feed_until_frame(&mut dec, &data).expect("should produce a frame");
        match dec.take_frame().expect("frame ready") {
            Frame::Nmea(f) => {
                assert!(f.raw.starts_with(b"$GPRMC"));
            }
            _ => panic!("expected NMEA frame"),
        }
    }

    #[test]
    fn mixed_nmea_and_ubx() {
        let mut dec = Decoder::new();

        // First: an NMEA sentence
        let nmea = b"$GPGGA,test*6C\r\n";
        feed_until_frame(&mut dec, nmea).expect("NMEA frame");
        match dec.take_frame().expect("frame ready") {
            Frame::Nmea(_) => {}
            _ => panic!("expected NMEA"),
        }

        // Second: a UBX frame
        let ubx_data = build_ubx(ubx::class::NAV, ubx::nav::PVT, &[1, 2, 3, 4]);
        feed_until_frame(&mut dec, &ubx_data).expect("UBX frame");
        match dec.take_frame().expect("frame ready") {
            Frame::Ubx(f) => {
                assert_eq!(f.class, ubx::class::NAV);
                assert_eq!(f.id, ubx::nav::PVT);
                assert_eq!(f.payload, &[1, 2, 3, 4]);
            }
            _ => panic!("expected UBX"),
        }
    }

    #[test]
    fn split_across_reads() {
        let mut dec = Decoder::new();
        let frame = build_ubx(ubx::class::ACK, ubx::ack::ACK, &[0x06, 0x01]);

        // Feed first half
        let mid = frame.len() / 2;
        for &b in &frame[..mid] {
            assert_eq!(dec.feed(b), FeedResult::Pending);
        }

        // Feed second half
        feed_until_frame(&mut dec, &frame[mid..]).expect("should complete");
        match dec.take_frame().expect("frame ready") {
            Frame::Ubx(f) => {
                assert_eq!(f.payload, &[0x06, 0x01]);
            }
            _ => panic!("expected UBX"),
        }
    }

    #[test]
    fn false_ubx_sync() {
        let mut dec = Decoder::new();
        // 0xB5 followed by something other than 0x62
        let data = [ubx::SYNC1, b'$'];

        // Feed 0xB5 -> goes to UbxSync2
        assert_eq!(dec.feed(data[0]), FeedResult::Pending);
        // Feed '$' -> false sync, re-feeds through Idle, starts NMEA
        assert_eq!(dec.feed(data[1]), FeedResult::Pending);

        // Now we should be in NMEA mode — feed rest of sentence
        let rest = b"GPGGA,test*6C\r\n";
        feed_until_frame(&mut dec, rest).expect("NMEA frame");
        match dec.take_frame().expect("frame ready") {
            Frame::Nmea(f) => {
                assert!(f.raw.starts_with(b"$GPGGA"));
            }
            _ => panic!("expected NMEA"),
        }
    }

    #[test]
    fn frame_too_large_nmea() {
        let mut dec = Decoder::new();
        // Start NMEA
        assert_eq!(dec.feed(b'$'), FeedResult::Pending);
        // Feed MAX_FRAME_SIZE - 1 bytes (the '$' already took one slot)
        for _ in 1..MAX_FRAME_SIZE {
            assert_eq!(dec.feed(b'A'), FeedResult::Pending);
        }
        // Next byte should trigger FrameTooLarge
        assert_eq!(
            dec.feed(b'A'),
            FeedResult::Error(DecodeError::FrameTooLarge)
        );
    }

    #[test]
    fn frame_too_large_ubx() {
        let mut dec = Decoder::new();
        // Craft a UBX header claiming a payload larger than MAX_FRAME_SIZE
        assert_eq!(dec.feed(ubx::SYNC1), FeedResult::Pending);
        assert_eq!(dec.feed(ubx::SYNC2), FeedResult::Pending);
        assert_eq!(dec.feed(0x01), FeedResult::Pending); // class
        assert_eq!(dec.feed(0x07), FeedResult::Pending); // id
        // Length = 0x0200 = 512 > 256
        assert_eq!(dec.feed(0x00), FeedResult::Pending); // len_lo
        assert_eq!(
            dec.feed(0x02),
            FeedResult::Error(DecodeError::FrameTooLarge)
        ); // len_hi
    }

    #[test]
    fn zero_length_ubx_payload() {
        let mut dec = Decoder::new();
        let frame = build_ubx(ubx::class::CFG, ubx::cfg::CFG, &[]);
        feed_until_frame(&mut dec, &frame).expect("should produce frame");
        match dec.take_frame().expect("frame ready") {
            Frame::Ubx(f) => {
                assert_eq!(f.class, ubx::class::CFG);
                assert_eq!(f.id, ubx::cfg::CFG);
                assert!(f.payload.is_empty());
            }
            _ => panic!("expected UBX"),
        }
    }

    #[test]
    fn reset_discards_partial() {
        let mut dec = Decoder::new();
        // Start an NMEA sentence
        dec.feed(b'$');
        dec.feed(b'G');
        dec.feed(b'P');

        // Reset mid-frame
        dec.reset();

        // Now feed a complete UBX frame
        let frame = build_ubx(ubx::class::ACK, ubx::ack::ACK, &[0x05, 0x01]);
        feed_until_frame(&mut dec, &frame).expect("should produce frame");
        match dec.take_frame().expect("frame ready") {
            Frame::Ubx(f) => {
                assert_eq!(f.class, ubx::class::ACK);
            }
            _ => panic!("expected UBX"),
        }
    }

    /// A real GGA sentence with a valid checksum, used as the "next good frame"
    /// in the resync tests.
    const GGA: &[u8] = b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,47.0,M,,*4F\r\n";
    /// `GGA` as the decoder reports it — no line terminator.
    const GGA_RAW: &[u8] = b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,47.0,M,,*4F";

    /// Feed bytes, returning the raw bytes of every frame that completed.
    fn collect_frames(dec: &mut Decoder, data: &[u8]) -> Vec<Vec<u8>> {
        let mut out = Vec::new();
        for &b in data {
            if dec.feed(b) == FeedResult::FrameReady {
                match dec.take_frame().expect("frame ready") {
                    Frame::Nmea(f) => out.push(f.raw.to_vec()),
                    Frame::Ubx(f) => out.push(f.payload.to_vec()),
                }
            }
        }
        out
    }

    /// A UBX frame too large to buffer must be skipped wholesale, not rescanned.
    ///
    /// The payload here is deliberately seeded with `$` and the UBX sync bytes:
    /// resyncing byte-by-byte through `Idle` would start phantom frames and
    /// swallow the genuine sentence that follows.
    #[test]
    fn oversized_ubx_does_not_desync() {
        let mut dec = Decoder::new();

        // Header announcing a 512-byte payload (> MAX_FRAME_SIZE).
        let header = [
            ubx::SYNC1,
            ubx::SYNC2,
            ubx::class::NAV,
            ubx::nav::SAT,
            0x00,
            0x02,
        ];
        for &b in &header[..header.len() - 1] {
            assert_eq!(dec.feed(b), FeedResult::Pending);
        }
        assert_eq!(
            dec.feed(header[header.len() - 1]),
            FeedResult::Error(DecodeError::FrameTooLarge),
            "oversized length should be reported once, at the header"
        );

        // The 512 payload bytes plus checksum, then a valid sentence.
        let mut rest = Vec::new();
        for i in 0..512u32 {
            rest.push(match i % 4 {
                0 => b'$',
                1 => ubx::SYNC1,
                2 => ubx::SYNC2,
                _ => b'A',
            });
        }
        rest.extend_from_slice(&[0x00, 0x00]); // checksum of the skipped frame
        rest.extend_from_slice(GGA);

        let frames = collect_frames(&mut dec, &rest);
        assert_eq!(frames.len(), 1, "expected only the trailing GGA, got {frames:?}");
        assert_eq!(frames[0], GGA_RAW);
    }

    /// An over-long NMEA sentence must be skipped to its line terminator.
    #[test]
    fn oversized_nmea_does_not_desync() {
        let mut dec = Decoder::new();

        let mut data = vec![b'$'];
        // Overflow the buffer, with '$' bytes past the overflow point.
        data.extend(core::iter::repeat_n(b'A', MAX_FRAME_SIZE));
        data.extend_from_slice(b"$GPGGA,junk*00");
        data.extend_from_slice(b"\r\n");
        data.extend_from_slice(GGA);

        let frames = collect_frames(&mut dec, &data);
        assert_eq!(frames.len(), 1, "expected only the trailing GGA, got {frames:?}");
        assert_eq!(frames[0], GGA_RAW);
    }

    #[test]
    fn nmea_bad_checksum_rejected() {
        let mut dec = Decoder::new();
        // Same sentence as GGA but with the checksum corrupted (4F -> 4E).
        let bad = b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,47.0,M,,*4E\r\n";

        let results = feed_all(&mut dec, bad);
        assert!(
            results.contains(&FeedResult::Error(DecodeError::NmeaChecksum)),
            "corrupt sentence should be rejected"
        );
        assert!(
            !results.contains(&FeedResult::FrameReady),
            "corrupt sentence must not be delivered"
        );
    }

    #[test]
    fn nmea_malformed_checksum_rejected() {
        let mut dec = Decoder::new();
        // '*' present but only one hex digit follows.
        let results = feed_all(&mut dec, b"$GPGGA,test*6\r\n");
        assert!(results.contains(&FeedResult::Error(DecodeError::NmeaChecksum)));
        assert!(!results.contains(&FeedResult::FrameReady));

        // '*' present but the digits are not hex.
        let mut dec = Decoder::new();
        let results = feed_all(&mut dec, b"$GPGGA,test*ZZ\r\n");
        assert!(results.contains(&FeedResult::Error(DecodeError::NmeaChecksum)));
        assert!(!results.contains(&FeedResult::FrameReady));
    }

    /// The `*hh` field is optional per the driver spec — a sentence without one
    /// is accepted rather than dropped.
    #[test]
    fn nmea_missing_checksum_accepted() {
        let mut dec = Decoder::new();
        let sentence = b"$GPZDA,no,checksum,here\r\n";
        feed_until_frame(&mut dec, sentence).expect("should produce a frame");
        match dec.take_frame().expect("frame ready") {
            Frame::Nmea(f) => assert_eq!(f.raw, b"$GPZDA,no,checksum,here"),
            _ => panic!("expected NMEA"),
        }
    }

    /// One corrupt sentence must not cost us the sentences either side of it.
    #[test]
    fn mixed_stream_survives_corruption() {
        let mut dec = Decoder::new();
        let mut data = Vec::new();
        data.extend_from_slice(GGA);
        // Valid RMC body, deliberately wrong checksum (6A -> 6B).
        data.extend_from_slice(
            b"$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6B\r\n",
        );
        data.extend_from_slice(GGA);

        let frames = collect_frames(&mut dec, &data);
        assert_eq!(frames.len(), 2, "corrupt sentence dropped, neighbours kept");
        assert_eq!(frames[0], GGA_RAW);
        assert_eq!(frames[1], GGA_RAW);
    }

    #[test]
    fn take_frame_before_ready_returns_none() {
        let dec = Decoder::new();
        assert!(dec.take_frame().is_none());

        // Also mid-frame, with a partial sentence buffered.
        let mut dec = Decoder::new();
        dec.feed(b'$');
        dec.feed(b'G');
        assert!(dec.take_frame().is_none());
    }

    #[test]
    fn nmea_without_cr() {
        let mut dec = Decoder::new();
        // Some receivers send just \n without \r
        let sentence = b"$GPGGA,test*6C\n";
        feed_until_frame(&mut dec, sentence).expect("should produce frame");
        match dec.take_frame().expect("frame ready") {
            Frame::Nmea(f) => {
                assert_eq!(f.raw, b"$GPGGA,test*6C");
            }
            _ => panic!("expected NMEA"),
        }
    }
}
