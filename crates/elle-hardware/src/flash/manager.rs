use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m::peripheral::NVIC;
use defmt::*;
use elle_config::profile::{
    FlashRequest, FlashResponse, ProfileEntry, ULOG_CHUNK_SIZE, ULOG_WRITE_CHUNK_SIZE,
};
use embassy_rp::flash::Flash;
use embassy_rp::interrupt;
use embassy_rp::mode::Async;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};
use embedded_storage_async::nor_flash::NorFlash as AsyncNorFlash;
use sequential_storage::cache::key_pointers::ArrayKeyPointers;
use sequential_storage::cache::page_pointers::ArrayPagePointers;
use sequential_storage::cache::page_states::{ArrayPageStates, CalculatedPageStates};
use sequential_storage::cache::{Cache, Uncached};
use sequential_storage::map::{MapConfig, MapStorage};
use sequential_storage::queue::{QueueConfig, QueueStorage};

use super::constants::{
    ERASE_SIZE, PROFILE_FLASH_END, PROFILE_FLASH_START, ULOG_FLASH_SIZE, ULOG_FLASH_START,
};

/// Mask SIO_IRQ_FIFO on Core0 before flash operations.
///
/// With `executor-interrupt` enabled, embassy registers a `SIO_IRQ_FIFO` handler
/// on Core0 that processes both `PEND_IRQ_TOKEN` and `PAUSE_TOKEN`. During flash
/// operations, `in_ram()` calls `pause_core1()` which polls the FIFO for Core1's
/// acknowledgment. If the interrupt handler fires first, it consumes the token
/// and enters the pause loop *on Core0*, deadlocking both cores (HardFault).
///
/// Masking the interrupt on Core0's NVIC before flash ops prevents this race.
/// Core1's handler is unaffected (NVIC is per-core). Full analysis:
/// `docs/embassy-rp-sio-irq-fifo-flash-bug.md`.
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
pub(crate) struct ULogWriteRequest {
    pub(crate) data: [u8; ULOG_WRITE_CHUNK_SIZE],
    pub(crate) len: usize,
}

/// Slots in [`ULOG_WRITE_CHANNEL`] (sized in `elle-config`).
pub(crate) const ULOG_WRITE_CHANNEL_DEPTH: usize = elle_config::profile::ULOG_WRITE_CHANNEL_DEPTH;

/// Buffered channel for fire-and-forget ULog writes (`ULOG_WRITE_CHANNEL_DEPTH` × 512 B)
pub(crate) static ULOG_WRITE_CHANNEL: Channel<
    CriticalSectionRawMutex,
    ULogWriteRequest,
    ULOG_WRITE_CHANNEL_DEPTH,
> = Channel::new();

/// Inter-core communication signals for flash operations (non-ULog-write ops)
pub static FLASH_REQUEST_SIGNAL: Signal<CriticalSectionRawMutex, FlashRequest> = Signal::new();
pub static FLASH_RESPONSE_SIGNAL: Signal<CriticalSectionRawMutex, FlashResponse> = Signal::new();

/// ULog storage usage counters (readable from any context for `ulog info`).
/// Nothing writes to the legacy flash ULog region any more (recordings go to
/// SD), so these are counted once when the flash manager starts and only go
/// down as `ulog extract` pops items; erase zeroes them.
pub static ULOG_BYTES_USED: AtomicU32 = AtomicU32::new(0);
pub static ULOG_ITEMS_STORED: AtomicU32 = AtomicU32::new(0);

/// Idle time before the one-off legacy ULog count runs (see `run`).
const ULOG_COUNT_IDLE: Duration = Duration::from_secs(5);

type FlashDevice<'a> = Flash<'a, Async, { elle_config::profile::FLASH_SIZE }>;

/// Number of erase pages in the PID/mag-cal profile map region (64KB / 4KB = 16)
const PROFILE_PAGE_COUNT: usize = ((PROFILE_FLASH_END - PROFILE_FLASH_START) as usize) / ERASE_SIZE;
/// Number of erase pages in the ULog queue region (~14MB / 4KB = 3584)
const ULOG_PAGE_COUNT: usize = ULOG_FLASH_SIZE / ERASE_SIZE;
/// Key slots in the map cache — one per `ProfileEntry` (keys 1–3), plus one spare
const MAP_KEY_SLOTS: usize = 4;

/// Persistent cache for the profile map region: full page states/pointers plus key
/// pointers (tiny at 16 pages), so repeated loads/saves skip the page scan.
type MapCache = Cache<
    ArrayPageStates<PROFILE_PAGE_COUNT>,
    ArrayPagePointers<PROFILE_PAGE_COUNT>,
    ArrayKeyPointers<u8, MAP_KEY_SLOTS>,
    u8,
>;
/// Persistent cache for the ULog queue region: `CalculatedPageStates` has fixed memory
/// use (array caches would cost ~KBs at 3584 pages), page/key pointers stay uncached.
type QueueCache = Cache<CalculatedPageStates, Uncached, Uncached>;

fn fresh_map_cache() -> MapCache {
    Cache::new(
        ArrayPageStates::new(),
        ArrayPagePointers::new(),
        ArrayKeyPointers::new(),
    )
}

fn fresh_queue_cache() -> QueueCache {
    Cache::new(
        CalculatedPageStates::new(ULOG_PAGE_COUNT),
        Uncached,
        Uncached,
    )
}

pub struct SequentialFlashManager<'a> {
    /// Flash device wrapped in Option for take/put ownership transfer.
    /// `pop()` requires `MultiwriteNorFlash` which isn't impl'd for `&mut Flash`,
    /// so we move flash into QueueStorage and back via destroy().
    flash: Option<FlashDevice<'a>>,
    /// Persistent map cache, moved into each MapStorage and recovered via destroy().
    /// Must be reset if the profile region is ever mutated outside sequential-storage
    /// (nothing does today; compare `queue_cache` and `erase_ulog_internal`).
    map_cache: Option<MapCache>,
    /// Persistent queue cache, same protocol as `map_cache` (see `erase_ulog_internal`).
    queue_cache: Option<QueueCache>,
}

impl<'a> SequentialFlashManager<'a> {
    #[must_use]
    pub fn new(flash: FlashDevice<'a>) -> Self {
        Self {
            flash: Some(flash),
            map_cache: Some(fresh_map_cache()),
            queue_cache: Some(fresh_queue_cache()),
        }
    }

    /// Take the flash device out. Panics if already taken.
    fn take_flash(&mut self) -> FlashDevice<'a> {
        self.flash.take().expect("flash already taken")
    }

    /// Put the flash device back.
    fn put_flash(&mut self, flash: FlashDevice<'a>) {
        self.flash = Some(flash);
    }

    /// Main flash manager task running on Core 0.
    ///
    /// Handles PID profiles, mag calibration, and ULog extraction from flash.
    /// ULog **writes** are handled by `sd_writer_task` via `ULOG_WRITE_CHANNEL`.
    pub async fn run(&mut self) {
        info!("Core0: Flash manager started (profiles + ULog extraction only)");

        // Count the legacy ULog queue once, but only after the boot-time loads
        // (PID profile, mag cal, level cal) are done: they wait on this task with
        // a 2 s timeout, and a long count ahead of them would drop the saved gains.
        let mut ulog_counted = false;
        loop {
            let request = if ulog_counted {
                FLASH_REQUEST_SIGNAL.wait().await
            } else {
                match embassy_futures::select::select(
                    FLASH_REQUEST_SIGNAL.wait(),
                    Timer::after(ULOG_COUNT_IDLE),
                )
                .await
                {
                    embassy_futures::select::Either::First(request) => request,
                    embassy_futures::select::Either::Second(()) => {
                        crate::watchdog::feed(Duration::from_millis(
                            elle_config::FLASH_OP_WATCHDOG_MS,
                        ));
                        self.count_ulog_internal().await;
                        ulog_counted = true;
                        continue;
                    }
                }
            };
            // The loop can't feed the watchdog while this runs (Core 1 paused,
            // Core 0 in blocking erase/program), so give it the whole budget.
            crate::watchdog::feed(Duration::from_millis(elle_config::FLASH_OP_WATCHDOG_MS));
            #[cfg(feature = "performance-monitoring")]
            let started = embassy_time::Instant::now();
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

                FlashRequest::ClearProfileEntry { entry } => {
                    let response = self.clear_profile_entry_internal(entry).await;
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

                FlashRequest::SaveLevelCal { data } => {
                    let response = self.save_level_cal_internal(&data).await;
                    FLASH_RESPONSE_SIGNAL.signal(response);
                }

                FlashRequest::LoadLevelCal => {
                    let response = self.load_level_cal_internal().await;
                    FLASH_RESPONSE_SIGNAL.signal(response);
                }
            }
            #[cfg(feature = "performance-monitoring")]
            crate::timing::FLASH_TIMING.record(started.elapsed().as_micros() as u32);

            // Small yield to ensure other tasks can run
            Timer::after(Duration::from_millis(1)).await;
        }
    }

    /// Peek at the oldest ULog entry without removing it
    async fn peek_ulog_internal(&mut self) -> FlashResponse {
        let flash = self.take_flash();
        let cache = self.queue_cache.take().expect("queue cache already taken");
        let config = QueueConfig::new(ULOG_FLASH_START..super::constants::ULOG_FLASH_END_EXCL);
        let mut queue = QueueStorage::new(flash, config, cache);

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

        let (flash, cache) = queue.destroy();
        self.put_flash(flash);
        self.queue_cache = Some(cache);

        response
    }

    /// Count what the legacy ULog queue holds, for `ulog info`.
    async fn count_ulog_internal(&mut self) {
        let flash = self.take_flash();
        let cache = self.queue_cache.take().expect("queue cache already taken");
        let config = QueueConfig::new(ULOG_FLASH_START..super::constants::ULOG_FLASH_END_EXCL);
        let mut queue = QueueStorage::new(flash, config, cache);

        let mut items: u32 = 0;
        let mut bytes: u32 = 0;
        mask_sio_fifo();
        let mut buf = [0u8; ULOG_CHUNK_SIZE];
        let result = match queue.iter().await {
            Ok(mut iter) => loop {
                match iter.next(&mut buf).await {
                    Ok(Some(entry)) => {
                        items += 1;
                        bytes += entry.len() as u32;
                    }
                    Ok(None) => break Ok(()),
                    Err(e) => break Err(e),
                }
            },
            Err(e) => Err(e),
        };
        unsafe { unmask_sio_fifo() };

        match result {
            Ok(()) => info!(
                "Flash: legacy ULog queue holds {} items, {} bytes",
                items, bytes
            ),
            Err(e) => warn!(
                "Flash: ULog queue count stopped after {} items: {:?}",
                items,
                Debug2Format(&e)
            ),
        }
        ULOG_ITEMS_STORED.store(items, Ordering::Relaxed);
        ULOG_BYTES_USED.store(bytes, Ordering::Relaxed);

        let (flash, cache) = queue.destroy();
        self.put_flash(flash);
        self.queue_cache = Some(cache);
    }

    /// Pop the oldest ULog entry from the queue
    async fn pop_ulog_internal(&mut self) -> FlashResponse {
        let flash = self.take_flash();
        let cache = self.queue_cache.take().expect("queue cache already taken");
        let config = QueueConfig::new(ULOG_FLASH_START..super::constants::ULOG_FLASH_END_EXCL);
        let mut queue = QueueStorage::new(flash, config, cache);

        mask_sio_fifo();
        let mut buf = [0u8; ULOG_CHUNK_SIZE];
        let result = queue.pop(&mut buf).await;
        unsafe { unmask_sio_fifo() };

        let response = match result {
            Ok(Some(data)) => {
                let len = data.len() as u32;
                // Saturating: an under-count must never wrap to ~4 billion.
                let _ = ULOG_BYTES_USED.fetch_update(Ordering::Relaxed, Ordering::Relaxed, |b| {
                    Some(b.saturating_sub(len))
                });
                let _ = ULOG_ITEMS_STORED.fetch_update(Ordering::Relaxed, Ordering::Relaxed, |n| {
                    Some(n.saturating_sub(1))
                });
                FlashResponse::ULogPopSuccess
            }
            Ok(None) => FlashResponse::ULogEmpty,
            Err(e) => {
                warn!("Flash: ULog pop failed: {:?}", Debug2Format(&e));
                FlashResponse::ULogEmpty
            }
        };

        let (flash, cache) = queue.destroy();
        self.put_flash(flash);
        self.queue_cache = Some(cache);

        response
    }

    /// Save a value to flash map storage by key.
    async fn save_to_map(&mut self, key: u8, value: &[u8]) -> bool {
        let flash = self.take_flash();
        let cache = self.map_cache.take().expect("map cache already taken");
        let config = MapConfig::new(PROFILE_FLASH_START..PROFILE_FLASH_END);
        let mut map: MapStorage<u8, _, _> = MapStorage::new(flash, config, cache);

        let mut data_buffer = [0u8; 128];
        mask_sio_fifo();
        let result = map.store_item(&mut data_buffer, &key, &value).await;
        unsafe { unmask_sio_fifo() };

        let (flash, cache) = map.destroy();
        self.put_flash(flash);
        self.map_cache = Some(cache);

        result.is_ok()
    }

    /// Remove a key from flash map storage. Other keys are untouched; a missing
    /// key is not an error. Needs `MultiwriteNorFlash`, which the RP flash
    /// driver implements. Slow in general (it scans every item), but this map
    /// holds three small entries.
    async fn remove_from_map(&mut self, key: u8) -> bool {
        let flash = self.take_flash();
        let cache = self.map_cache.take().expect("map cache already taken");
        let config = MapConfig::new(PROFILE_FLASH_START..PROFILE_FLASH_END);
        let mut map: MapStorage<u8, _, _> = MapStorage::new(flash, config, cache);

        let mut data_buffer = [0u8; 128];
        mask_sio_fifo();
        let result = map.remove_item(&mut data_buffer, &key).await;
        unsafe { unmask_sio_fifo() };

        if let Err(e) = &result {
            warn!("Flash: key {} remove failed: {:?}", key, Debug2Format(e));
        }

        let (flash, cache) = map.destroy();
        self.put_flash(flash);
        self.map_cache = Some(cache);

        result.is_ok()
    }

    /// Load a value from flash map storage by key, returning up to `N` bytes.
    async fn load_from_map<const N: usize>(&mut self, key: u8) -> Option<[u8; N]> {
        let flash = self.take_flash();
        let cache = self.map_cache.take().expect("map cache already taken");
        let config = MapConfig::new(PROFILE_FLASH_START..PROFILE_FLASH_END);
        let mut map: MapStorage<u8, _, _> = MapStorage::new(flash, config, cache);

        let mut data_buffer = [0u8; 128];
        mask_sio_fifo();
        let result: Result<Option<&[u8]>, _> = map.fetch_item(&mut data_buffer, &key).await;
        unsafe { unmask_sio_fifo() };

        let loaded = match result {
            Ok(Some(slice)) if slice.len() == N => {
                let mut out = [0u8; N];
                out.copy_from_slice(slice);
                Some(out)
            }
            Ok(Some(slice)) => {
                warn!(
                    "Flash: key {} wrong size ({}), expected {}",
                    key,
                    slice.len(),
                    N
                );
                None
            }
            Ok(None) => None,
            Err(e) => {
                warn!("Flash: key {} load failed: {:?}", key, Debug2Format(&e));
                None
            }
        };

        let (flash, cache) = map.destroy();
        self.put_flash(flash);
        self.map_cache = Some(cache);

        loaded
    }

    /// Save PID profile to flash map storage
    async fn save_pid_profile_internal(&mut self, data: &[u8; 32]) -> FlashResponse {
        if self
            .save_to_map(ProfileEntry::Pid.key(), data.as_slice())
            .await
        {
            info!("Flash: PID profile saved");
            FlashResponse::PidProfileSaved
        } else {
            warn!("Flash: PID profile save failed");
            FlashResponse::PidProfileSaveFailed
        }
    }

    /// Remove one profile entry (PID gains, mag cal or level cal)
    async fn clear_profile_entry_internal(&mut self, entry: ProfileEntry) -> FlashResponse {
        if self.remove_from_map(entry.key()).await {
            info!("Flash: {:?} cleared", Debug2Format(&entry));
            FlashResponse::ProfileEntryCleared
        } else {
            FlashResponse::ProfileEntryClearFailed
        }
    }

    /// Load PID profile from flash map storage
    async fn load_pid_profile_internal(&mut self) -> FlashResponse {
        match self.load_from_map::<32>(ProfileEntry::Pid.key()).await {
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
        if self
            .save_to_map(ProfileEntry::MagCal.key(), data.as_slice())
            .await
        {
            info!("Flash: Mag cal saved");
            FlashResponse::MagCalSaved
        } else {
            warn!("Flash: Mag cal save failed");
            FlashResponse::MagCalSaveFailed
        }
    }

    /// Load mag calibration offsets from flash map storage (key=2, 12 bytes)
    async fn load_mag_cal_internal(&mut self) -> FlashResponse {
        match self.load_from_map::<12>(ProfileEntry::MagCal.key()).await {
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

    /// Save the level-cal mount quaternion to flash map storage (key=3, 16 bytes)
    async fn save_level_cal_internal(&mut self, data: &[u8; 16]) -> FlashResponse {
        if self
            .save_to_map(ProfileEntry::LevelCal.key(), data.as_slice())
            .await
        {
            info!("Flash: Level cal saved");
            FlashResponse::LevelCalSaved
        } else {
            warn!("Flash: Level cal save failed");
            FlashResponse::LevelCalSaveFailed
        }
    }

    /// Load the level-cal mount quaternion from flash map storage (key=3, 16 bytes)
    async fn load_level_cal_internal(&mut self) -> FlashResponse {
        match self.load_from_map::<16>(ProfileEntry::LevelCal.key()).await {
            Some(data) => {
                info!("Flash: Level cal loaded");
                FlashResponse::LevelCalLoaded { data }
            }
            None => {
                info!("Flash: No level cal stored");
                FlashResponse::LevelCalEmpty
            }
        }
    }

    /// Erase the entire ULog flash region
    async fn erase_ulog_internal(&mut self) -> FlashResponse {
        // Direct erase bypasses sequential-storage, so the persistent cache
        // is stale afterwards (even on partial/failed erase) — reset it.
        self.queue_cache = Some(fresh_queue_cache());
        let flash = self.flash.as_mut().expect("flash not available");
        let mut addr = ULOG_FLASH_START;
        let end = super::constants::ULOG_FLASH_END_EXCL;
        const ERASE_CHUNK: u32 = 64 * 1024; // 64KB per iteration
        const _: () = core::assert!((ERASE_CHUNK as usize).is_multiple_of(ERASE_SIZE));

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
pub(crate) fn request_write_ulog(data: &[u8]) -> bool {
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
pub(crate) async fn request_write_ulog_blocking(data: &[u8]) -> bool {
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
