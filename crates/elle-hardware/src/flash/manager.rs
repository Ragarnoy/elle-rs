use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m::peripheral::NVIC;
use defmt::*;
use elle_config::profile::{FlashRequest, FlashResponse, ULOG_CHUNK_SIZE, ULOG_WRITE_CHUNK_SIZE};
use embassy_rp::flash::{Async, Flash};
use embassy_rp::interrupt;
use embassy_rp::peripherals::FLASH;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};
use embedded_storage_async::nor_flash::NorFlash as AsyncNorFlash;
use sequential_storage::cache::NoCache;
use sequential_storage::map::{MapConfig, MapStorage};
use sequential_storage::queue::{QueueConfig, QueueStorage};

use super::constants::{PROFILE_FLASH_END, PROFILE_FLASH_START, ULOG_FLASH_START};

/// Mask SIO_IRQ_FIFO on Core0 before flash operations.
///
/// With `executor-interrupt` enabled, embassy registers a `SIO_IRQ_FIFO` handler
/// on Core0 that processes both `PEND_IRQ_TOKEN` and `PAUSE_TOKEN`. During flash
/// operations, `in_ram()` calls `pause_core1()` which polls the FIFO for Core1's
/// acknowledgment. If the interrupt handler fires first, it consumes the token
/// and enters the pause loop *on Core0*, deadlocking both cores (HardFault).
///
/// Masking the interrupt on Core0's NVIC before flash ops prevents this race.
/// Core1's handler is unaffected (NVIC is per-core).
fn mask_sio_fifo() {
    NVIC::mask(interrupt::SIO_IRQ_FIFO);
}

/// Unmask SIO_IRQ_FIFO on Core0 after flash operations complete.
///
/// # Safety
/// Only call after a corresponding `mask_sio_fifo()`.
unsafe fn unmask_sio_fifo() {
    unsafe { NVIC::unmask(interrupt::SIO_IRQ_FIFO) };
}

/// Fire-and-forget ULog write request (buffered channel, non-blocking send).
/// Uses smaller chunk size (512B) to avoid 4KB stack allocations in the write path.
pub struct ULogWriteRequest {
    pub data: [u8; ULOG_WRITE_CHUNK_SIZE],
    pub len: usize,
}

/// Buffered channel for fire-and-forget ULog writes (8 slots × 512B = 4KB)
pub static ULOG_WRITE_CHANNEL: Channel<CriticalSectionRawMutex, ULogWriteRequest, 8> =
    Channel::new();

/// Inter-core communication signals for flash operations (non-ULog-write ops)
pub static FLASH_REQUEST_SIGNAL: Signal<CriticalSectionRawMutex, FlashRequest> = Signal::new();
pub static FLASH_RESPONSE_SIGNAL: Signal<CriticalSectionRawMutex, FlashResponse> = Signal::new();

/// ULog storage usage counters (readable from any context for `ulog info`)
pub static ULOG_BYTES_USED: AtomicU32 = AtomicU32::new(0);
pub static ULOG_ITEMS_STORED: AtomicU32 = AtomicU32::new(0);

type FlashDevice<'a> = Flash<'a, FLASH, Async, { elle_config::profile::FLASH_SIZE }>;

pub struct SequentialFlashManager<'a> {
    /// Flash device wrapped in Option for take/put ownership transfer.
    /// `pop()` requires `MultiwriteNorFlash` which isn't impl'd for `&mut Flash`,
    /// so we move flash into QueueStorage and back via destroy().
    flash: Option<FlashDevice<'a>>,
}

impl<'a> SequentialFlashManager<'a> {
    #[must_use]
    pub const fn new(flash: FlashDevice<'a>) -> Self {
        Self {
            flash: Some(flash),
        }
    }

    /// Take the flash device out. Panics if already taken.
    const fn take_flash(&mut self) -> FlashDevice<'a> {
        self.flash.take().expect("flash already taken")
    }

    /// Put the flash device back.
    const fn put_flash(&mut self, flash: FlashDevice<'a>) {
        self.flash = Some(flash);
    }

    /// Main flash manager task running on Core 0.
    ///
    /// Handles PID profiles, mag calibration, and ULog extraction from flash.
    /// ULog **writes** are handled by `sd_writer_task` via `ULOG_WRITE_CHANNEL`.
    pub async fn run(&mut self) {
        info!("Core0: Flash manager started (profiles + ULog extraction only)");

        loop {
            let request = FLASH_REQUEST_SIGNAL.wait().await;
            match request {
                    FlashRequest::PeekULog => {
                        let response = self.peek_ulog_internal().await;
                        FLASH_RESPONSE_SIGNAL.signal(response);
                    }

                    FlashRequest::PopULog => {
                        let response = self.pop_ulog_internal().await;
                        FLASH_RESPONSE_SIGNAL.signal(response);
                    }

                    FlashRequest::EraseULog => {
                        info!("Flash: erasing ULog region");
                        let response = self.erase_ulog_internal().await;
                        FLASH_RESPONSE_SIGNAL.signal(response);
                    }

                    FlashRequest::SavePidProfile { data } => {
                        let response = self.save_pid_profile_internal(&data).await;
                        FLASH_RESPONSE_SIGNAL.signal(response);
                    }

                    FlashRequest::LoadPidProfile => {
                        let response = self.load_pid_profile_internal().await;
                        FLASH_RESPONSE_SIGNAL.signal(response);
                    }

                    FlashRequest::SaveMagCal { data } => {
                        let response = self.save_mag_cal_internal(&data).await;
                        FLASH_RESPONSE_SIGNAL.signal(response);
                    }

                    FlashRequest::LoadMagCal => {
                        let response = self.load_mag_cal_internal().await;
                        FLASH_RESPONSE_SIGNAL.signal(response);
                    }
                }

            // Small yield to ensure other tasks can run
            Timer::after(Duration::from_millis(1)).await;
        }
    }

    /// Peek at the oldest ULog entry without removing it
    async fn peek_ulog_internal(&mut self) -> FlashResponse {
        let flash = self.take_flash();
        let config = QueueConfig::new(ULOG_FLASH_START..super::constants::ULOG_FLASH_END_EXCL);
        let mut queue = QueueStorage::new(flash, config, NoCache::new());

        mask_sio_fifo();
        let mut buf = [0u8; ULOG_CHUNK_SIZE];
        let result = queue.peek(&mut buf).await;
        unsafe { unmask_sio_fifo() };

        // Copy data before destroying queue (result borrows ulog_buffer)
        let response = match result {
            Ok(Some(data)) => {
                let len = data.len();
                let mut resp_data = [0u8; ULOG_CHUNK_SIZE];
                resp_data[..len].copy_from_slice(data);
                FlashResponse::ULogData {
                    data: resp_data,
                    len,
                }
            }
            Ok(None) => FlashResponse::ULogEmpty,
            Err(e) => {
                warn!("Flash: ULog peek failed: {:?}", Debug2Format(&e));
                FlashResponse::ULogEmpty
            }
        };

        let (flash, _cache) = queue.destroy();
        self.put_flash(flash);

        response
    }

    /// Pop the oldest ULog entry from the queue
    async fn pop_ulog_internal(&mut self) -> FlashResponse {
        let flash = self.take_flash();
        let config = QueueConfig::new(ULOG_FLASH_START..super::constants::ULOG_FLASH_END_EXCL);
        let mut queue = QueueStorage::new(flash, config, NoCache::new());

        mask_sio_fifo();
        let mut buf = [0u8; ULOG_CHUNK_SIZE];
        let result = queue.pop(&mut buf).await;
        unsafe { unmask_sio_fifo() };

        let response = match result {
            Ok(Some(data)) => {
                ULOG_BYTES_USED.fetch_sub(data.len() as u32, Ordering::Relaxed);
                ULOG_ITEMS_STORED.fetch_sub(1, Ordering::Relaxed);
                FlashResponse::ULogPopSuccess
            }
            Ok(None) => FlashResponse::ULogEmpty,
            Err(e) => {
                warn!("Flash: ULog pop failed: {:?}", Debug2Format(&e));
                FlashResponse::ULogEmpty
            }
        };

        let (flash, _cache) = queue.destroy();
        self.put_flash(flash);

        response
    }

    /// Save a value to flash map storage by key.
    async fn save_to_map(&mut self, key: u8, value: &[u8]) -> bool {
        let flash = self.take_flash();
        let config = MapConfig::new(PROFILE_FLASH_START..PROFILE_FLASH_END);
        let mut map: MapStorage<u8, _, _> = MapStorage::new(flash, config, NoCache::new());

        let mut data_buffer = [0u8; 128];
        mask_sio_fifo();
        let result = map.store_item(&mut data_buffer, &key, &value).await;
        unsafe { unmask_sio_fifo() };

        let (flash, _cache) = map.destroy();
        self.put_flash(flash);

        result.is_ok()
    }

    /// Load a value from flash map storage by key, returning up to `N` bytes.
    async fn load_from_map<const N: usize>(&mut self, key: u8) -> Option<[u8; N]> {
        let flash = self.take_flash();
        let config = MapConfig::new(PROFILE_FLASH_START..PROFILE_FLASH_END);
        let mut map: MapStorage<u8, _, _> = MapStorage::new(flash, config, NoCache::new());

        let mut data_buffer = [0u8; 128];
        mask_sio_fifo();
        let result: Result<Option<&[u8]>, _> =
            map.fetch_item(&mut data_buffer, &key).await;
        unsafe { unmask_sio_fifo() };

        let loaded = match result {
            Ok(Some(slice)) if slice.len() == N => {
                let mut out = [0u8; N];
                out.copy_from_slice(slice);
                Some(out)
            }
            Ok(Some(slice)) => {
                warn!("Flash: key {} wrong size ({}), expected {}", key, slice.len(), N);
                None
            }
            Ok(None) => None,
            Err(e) => {
                warn!("Flash: key {} load failed: {:?}", key, Debug2Format(&e));
                None
            }
        };

        let (flash, _cache) = map.destroy();
        self.put_flash(flash);

        loaded
    }

    /// Save PID profile to flash map storage
    async fn save_pid_profile_internal(&mut self, data: &[u8; 32]) -> FlashResponse {
        if self.save_to_map(1, data.as_slice()).await {
            info!("Flash: PID profile saved");
            FlashResponse::PidProfileSaved
        } else {
            warn!("Flash: PID profile save failed");
            FlashResponse::PidProfileSaveFailed
        }
    }

    /// Load PID profile from flash map storage
    async fn load_pid_profile_internal(&mut self) -> FlashResponse {
        match self.load_from_map::<32>(1).await {
            Some(data) => {
                info!("Flash: PID profile loaded");
                FlashResponse::PidProfileLoaded { data }
            }
            None => {
                info!("Flash: No PID profile stored");
                FlashResponse::PidProfileEmpty
            }
        }
    }

    /// Save mag calibration offsets to flash map storage (key=2, 12 bytes)
    async fn save_mag_cal_internal(&mut self, data: &[u8; 12]) -> FlashResponse {
        if self.save_to_map(2, data.as_slice()).await {
            info!("Flash: Mag cal saved");
            FlashResponse::MagCalSaved
        } else {
            warn!("Flash: Mag cal save failed");
            FlashResponse::MagCalSaveFailed
        }
    }

    /// Load mag calibration offsets from flash map storage (key=2, 12 bytes)
    async fn load_mag_cal_internal(&mut self) -> FlashResponse {
        match self.load_from_map::<12>(2).await {
            Some(data) => {
                info!("Flash: Mag cal loaded");
                FlashResponse::MagCalLoaded { data }
            }
            None => {
                info!("Flash: No mag cal stored");
                FlashResponse::MagCalEmpty
            }
        }
    }

    /// Erase the entire ULog flash region
    async fn erase_ulog_internal(&mut self) -> FlashResponse {
        let flash = self.flash.as_mut().expect("flash not available");
        let mut addr = ULOG_FLASH_START;
        let end = super::constants::ULOG_FLASH_END_EXCL;
        const ERASE_CHUNK: u32 = 64 * 1024; // 64KB per iteration

        while addr < end {
            let chunk_end = (addr + ERASE_CHUNK).min(end);
            mask_sio_fifo();
            match flash.erase(addr, chunk_end).await {
                Ok(_) => {
                    unsafe { unmask_sio_fifo() };
                    addr = chunk_end;
                }
                Err(e) => {
                    unsafe { unmask_sio_fifo() };
                    crate::elle_event!(
                        error,
                        crate::event::EVT_FLASH_ULOG_ERASE_FAILED,
                        "Flash: ULog erase failed at 0x{:X}: {:?}",
                        addr,
                        Debug2Format(&e)
                    );
                    return FlashResponse::ULogEraseFailed;
                }
            }
            Timer::after(Duration::from_millis(1)).await;
        }

        ULOG_BYTES_USED.store(0, Ordering::Relaxed);
        ULOG_ITEMS_STORED.store(0, Ordering::Relaxed);
        FlashResponse::ULogEraseSuccess
    }
}

/// Fire-and-forget ULog write — returns immediately, data is queued for async flash write.
/// Returns `false` if the channel is full (data dropped) or input is invalid.
pub fn request_write_ulog(data: &[u8]) -> bool {
    if data.is_empty() || data.len() > ULOG_WRITE_CHUNK_SIZE {
        warn!("Flash: invalid ULog data size: {}", data.len());
        return false;
    }

    let mut buffer = [0u8; ULOG_WRITE_CHUNK_SIZE];
    buffer[..data.len()].copy_from_slice(data);

    match ULOG_WRITE_CHANNEL.try_send(ULogWriteRequest {
        data: buffer,
        len: data.len(),
    }) {
        Ok(()) => true,
        Err(_) => {
            warn!("Flash: ULog write channel full, data dropped");
            false
        }
    }
}

/// Blocking ULog write — waits until data is queued for flash write.
/// Splits large payloads (e.g. ULog header) into ULOG_WRITE_CHUNK_SIZE chunks.
/// Used for ULog header writes during initialization.
pub async fn request_write_ulog_blocking(data: &[u8]) -> bool {
    if data.is_empty() {
        warn!("Flash: empty ULog data");
        return false;
    }

    for chunk in data.chunks(ULOG_WRITE_CHUNK_SIZE) {
        let mut req = ULogWriteRequest {
            data: [0u8; ULOG_WRITE_CHUNK_SIZE],
            len: chunk.len(),
        };
        req.data[..chunk.len()].copy_from_slice(chunk);

        let timeout = Timer::after(Duration::from_secs(10));
        match embassy_futures::select::select(ULOG_WRITE_CHANNEL.send(req), timeout).await {
            embassy_futures::select::Either::First(()) => {}
            embassy_futures::select::Either::Second(_) => {
                crate::elle_event!(
                    error,
                    crate::event::EVT_FLASH_ULOG_WRITE_TIMEOUT,
                    "Flash: timeout waiting for ULog write channel"
                );
                return false;
            }
        }
    }
    true
}
