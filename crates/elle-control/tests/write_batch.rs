//! Host tests for the SD writer's sector-aligned batch.
//!
//!   cargo test -p elle-control --target x86_64-unknown-linux-gnu --test write_batch

use elle_config::profile::{SD_SECTOR_SIZE, SD_WRITE_BATCH_SIZE, ULOG_WRITE_CHUNK_SIZE};
use elle_control::write_batch::WriteBatch;

#[test]
fn only_whole_sectors_are_offered() {
    let mut buf = [0u8; 32];
    let mut batch = WriteBatch::new(&mut buf, 8);
    assert!(batch.is_empty());
    assert!(batch.whole_sectors().is_empty());

    assert_eq!(batch.push(&[1; 7]), 7);
    assert!(
        batch.whole_sectors().is_empty(),
        "7 of 8 bytes: nothing yet"
    );

    assert_eq!(batch.push(&[2; 12]), 12);
    assert_eq!(batch.len(), 19);
    assert_eq!(batch.whole_sectors().len(), 16);
    assert_eq!(batch.pending().len(), 19);
}

#[test]
fn consume_keeps_the_tail_in_order() {
    let mut buf = [0u8; 32];
    let mut batch = WriteBatch::new(&mut buf, 8);
    let data: [u8; 19] = core::array::from_fn(|i| i as u8);
    batch.push(&data);

    let n = batch.whole_sectors().len();
    batch.consume(n);
    assert_eq!(batch.pending(), &data[16..]);
    assert_eq!(batch.free(), 32 - 3);

    // The tail completes into the next sector with what follows.
    batch.push(&[100, 101, 102, 103, 104]);
    assert_eq!(
        batch.whole_sectors(),
        &[16, 17, 18, 100, 101, 102, 103, 104]
    );
}

#[test]
fn push_takes_only_what_fits() {
    let mut buf = [0u8; 16];
    let mut batch = WriteBatch::new(&mut buf, 8);
    assert_eq!(batch.push(&[7; 10]), 10);
    assert_eq!(batch.push(&[8; 10]), 6);
    assert_eq!(batch.free(), 0);
    assert_eq!(batch.push(&[9; 1]), 0);
    assert_eq!(batch.whole_sectors().len(), 16);
}

#[test]
fn consume_is_clamped_and_clear_empties() {
    let mut buf = [0u8; 16];
    let mut batch = WriteBatch::new(&mut buf, 8);
    batch.push(&[1; 5]);
    batch.consume(100);
    assert!(batch.is_empty());
    batch.push(&[1; 5]);
    batch.clear();
    assert!(batch.is_empty());
    assert_eq!(batch.free(), 16);
}

/// The writer's loop on the firmware's sizes: chunks of arbitrary length in,
/// only whole sectors out, every byte once and in order, and always room for
/// one more chunk after a write (the writer checks `free()` before it takes a
/// chunk off the queue).
#[test]
fn stream_of_odd_chunks_comes_out_sector_aligned() {
    let mut buf = [0u8; SD_WRITE_BATCH_SIZE];
    let mut batch = WriteBatch::new(&mut buf, SD_SECTOR_SIZE);
    let mut written: Vec<u8> = Vec::new();
    let mut sent: Vec<u8> = Vec::new();
    let mut next = 0u8;

    // The logger's pattern: full chunks, then a partial one ending each flush.
    let lengths = [512, 512, 512, 37, 512, 1, 511, 512, 512, 300, 512, 129];
    for round in 0..200 {
        let len = lengths[round % lengths.len()];
        assert!(len <= ULOG_WRITE_CHUNK_SIZE);
        let chunk: Vec<u8> = (0..len)
            .map(|_| {
                next = next.wrapping_add(1);
                next
            })
            .collect();
        sent.extend_from_slice(&chunk);

        assert!(batch.free() >= ULOG_WRITE_CHUNK_SIZE);
        assert_eq!(batch.push(&chunk), len);

        // Write when no further chunk is guaranteed to fit, or every third
        // round (the queue ran empty).
        if batch.free() < ULOG_WRITE_CHUNK_SIZE || round % 3 == 0 {
            let out = batch.whole_sectors();
            assert!(out.len().is_multiple_of(SD_SECTOR_SIZE));
            assert!(written.len().is_multiple_of(SD_SECTOR_SIZE));
            written.extend_from_slice(out);
            let n = out.len();
            batch.consume(n);
            assert!(batch.len() < SD_SECTOR_SIZE);
        }
    }

    // File close: the tail goes out last.
    written.extend_from_slice(batch.pending());
    assert_eq!(written, sent);
}
