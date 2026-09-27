//! Byte-stream reassembly into 0x00-delimited (COBS) frames.
//!
//! Dependency-free so it can be tested on the host even though the rest of this
//! crate only builds for the target:
//!
//!   rustc --edition 2024 --test crates/elle-system/src/frame_buf.rs -o /tmp/frame_buf && /tmp/frame_buf

/// A frame larger than the caller's output buffer. It has been consumed and dropped.
#[derive(Debug, PartialEq, Eq)]
pub(crate) struct FrameTooLarge;

/// Accumulates bytes from a stream and hands them out one delimited frame at a time.
///
/// Every complete frame already in the buffer is found, not only frames that
/// end in the latest read: one read can deliver several frames, and the ones
/// after the first must come out on the next call rather than wait for (and get
/// glued onto) whatever arrives next.
pub(crate) struct FrameBuf<const N: usize> {
    buf: [u8; N],
    used: usize,
}

impl<const N: usize> FrameBuf<N> {
    #[must_use]
    pub(crate) const fn new() -> Self {
        Self {
            buf: [0; N],
            used: 0,
        }
    }

    /// Free space to read new bytes into; follow with [`commit`](Self::commit).
    pub(crate) fn spare(&mut self) -> &mut [u8] {
        &mut self.buf[self.used..]
    }

    /// Record `n` bytes written into [`spare`](Self::spare).
    pub(crate) fn commit(&mut self, n: usize) {
        self.used = (self.used + n).min(N);
    }

    /// Full with no delimiter anywhere: the stream is out of sync or a frame is
    /// longer than the buffer. [`clear`](Self::clear) to resynchronise.
    #[must_use]
    pub(crate) fn is_stuck(&self) -> bool {
        self.used == N && !self.buf.contains(&0)
    }

    pub(crate) fn clear(&mut self) {
        self.used = 0;
    }

    /// Take the oldest complete frame (delimiter excluded) into `out`.
    ///
    /// `None` when no complete frame is buffered. `Some(Ok(len))` copies the
    /// frame into `out[..len]` (may be 0 for back-to-back delimiters).
    /// `Some(Err(FrameTooLarge))` when it would not fit; it is dropped either way.
    pub(crate) fn pop_frame(&mut self, out: &mut [u8]) -> Option<Result<usize, FrameTooLarge>> {
        let pos = self.buf[..self.used].iter().position(|&b| b == 0)?;
        let result = if pos <= out.len() {
            out[..pos].copy_from_slice(&self.buf[..pos]);
            Ok(pos)
        } else {
            Err(FrameTooLarge)
        };
        self.buf.copy_within(pos + 1..self.used, 0);
        self.used -= pos + 1;
        Some(result)
    }
}

impl<const N: usize> Default for FrameBuf<N> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn feed<const N: usize>(fb: &mut FrameBuf<N>, bytes: &[u8]) {
        let spare = fb.spare();
        spare[..bytes.len()].copy_from_slice(bytes);
        fb.commit(bytes.len());
    }

    #[test]
    fn two_frames_in_one_read_both_come_out() {
        let mut fb = FrameBuf::<64>::new();
        let mut out = [0u8; 16];
        feed(&mut fb, &[1, 2, 3, 0, 4, 5, 0]);
        assert_eq!(fb.pop_frame(&mut out), Some(Ok(3)));
        assert_eq!(&out[..3], &[1, 2, 3]);
        // The second frame is available with no further input.
        assert_eq!(fb.pop_frame(&mut out), Some(Ok(2)));
        assert_eq!(&out[..2], &[4, 5]);
        assert_eq!(fb.pop_frame(&mut out), None);
    }

    #[test]
    fn stranded_frame_is_not_merged_with_the_next() {
        let mut fb = FrameBuf::<64>::new();
        let mut out = [0u8; 16];
        feed(&mut fb, &[1, 0, 2, 0]);
        assert_eq!(fb.pop_frame(&mut out), Some(Ok(1)));
        feed(&mut fb, &[3, 0]);
        assert_eq!(fb.pop_frame(&mut out), Some(Ok(1)));
        assert_eq!(out[0], 2);
        assert_eq!(fb.pop_frame(&mut out), Some(Ok(1)));
        assert_eq!(out[0], 3);
    }

    #[test]
    fn partial_frame_waits_for_its_delimiter() {
        let mut fb = FrameBuf::<64>::new();
        let mut out = [0u8; 16];
        feed(&mut fb, &[7, 8]);
        assert_eq!(fb.pop_frame(&mut out), None);
        feed(&mut fb, &[9, 0]);
        assert_eq!(fb.pop_frame(&mut out), Some(Ok(3)));
        assert_eq!(&out[..3], &[7, 8, 9]);
    }

    #[test]
    fn oversized_frame_is_dropped_and_the_next_survives() {
        let mut fb = FrameBuf::<64>::new();
        let mut out = [0u8; 2];
        feed(&mut fb, &[1, 2, 3, 4, 0, 5, 0]);
        assert_eq!(fb.pop_frame(&mut out), Some(Err(FrameTooLarge)));
        assert_eq!(fb.pop_frame(&mut out), Some(Ok(1)));
        assert_eq!(out[0], 5);
    }

    #[test]
    fn full_buffer_without_delimiter_is_stuck() {
        let mut fb = FrameBuf::<4>::new();
        feed(&mut fb, &[1, 2, 3, 4]);
        assert!(fb.is_stuck());
        fb.clear();
        assert!(!fb.is_stuck());
        assert_eq!(fb.spare().len(), 4);
    }
}
