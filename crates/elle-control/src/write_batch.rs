//! Sector-aligned batching for the SD card ULog writer.
//!
//! The FAT stack only turns a write into one multi-block card command when the
//! file position is on a sector boundary and the length is a whole number of
//! sectors; anything else goes through a one-sector cache, one read and one
//! single-block write per sector (see `elle_hardware::sd_writer`). ULog chunks
//! arrive in arbitrary lengths, so the writer collects them here, writes only
//! the whole-sector prefix, and keeps the tail (less than a sector) for the
//! next write. The file position then stays sector-aligned for the whole
//! recording; the tail goes out once, when the file closes.
//!
//! Pure bookkeeping over a borrowed buffer, so it is host-tested
//! (`tests/write_batch.rs`).

/// Bytes collected for the next write, over a borrowed buffer.
pub struct WriteBatch<'a> {
    buf: &'a mut [u8],
    len: usize,
    sector: usize,
}

impl<'a> WriteBatch<'a> {
    /// An empty batch over `buf`, for a device with `sector`-byte sectors.
    ///
    /// # Panics
    /// If `sector` is zero or `buf` is not a whole, non-zero number of sectors.
    #[must_use]
    pub fn new(buf: &'a mut [u8], sector: usize) -> Self {
        assert!(sector > 0 && !buf.is_empty() && buf.len().is_multiple_of(sector));
        Self {
            buf,
            len: 0,
            sector,
        }
    }

    /// Bytes held.
    #[must_use]
    pub const fn len(&self) -> usize {
        self.len
    }

    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Bytes that still fit.
    #[must_use]
    pub const fn free(&self) -> usize {
        self.buf.len() - self.len
    }

    /// Append as much of `data` as fits; returns how many bytes were taken.
    pub fn push(&mut self, data: &[u8]) -> usize {
        let n = data.len().min(self.free());
        self.buf[self.len..self.len + n].copy_from_slice(&data[..n]);
        self.len += n;
        n
    }

    /// The longest prefix that is a whole number of sectors (possibly empty).
    #[must_use]
    pub fn whole_sectors(&self) -> &[u8] {
        &self.buf[..self.len - self.len % self.sector]
    }

    /// Everything held, tail included.
    #[must_use]
    pub fn pending(&self) -> &[u8] {
        &self.buf[..self.len]
    }

    /// Drop the first `n` bytes (written, or given up on) and move the rest to
    /// the front. `n` is clamped to what is held.
    pub fn consume(&mut self, n: usize) {
        let n = n.min(self.len);
        self.buf.copy_within(n..self.len, 0);
        self.len -= n;
    }

    /// Drop everything held.
    pub fn clear(&mut self) {
        self.len = 0;
    }
}
