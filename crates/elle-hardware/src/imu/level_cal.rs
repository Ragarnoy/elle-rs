//! Level calibration plumbing: cross-core signals, status, and flash persistence.
//!
//! The math lives in `elle_control::level_cal`; the Core1 driver collects samples
//! and applies the mount rotation. This module is what Core0 calls, identically
//! from both airframes and both flight and RPC modes:
//!
//! - [`load_from_flash`] once at boot,
//! - [`start`] / [`clear`] on a command or gesture,
//! - [`poll_result`] every control-loop iteration.

use core::cell::Cell;

use elle_config::profile::{FlashRequest, FlashResponse, ProfileEntry};
use elle_control::level_cal::{
    LevelCalFail, mount_from_bytes, mount_to_bytes, mount_to_display_deg,
};
use embassy_futures::select::{Either, select};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};
use nalgebra::UnitQuaternion;

use crate::event;
use crate::flash::{FLASH_REQUEST_SIGNAL, FLASH_RESPONSE_SIGNAL};

/// Core0 → Core1: mount rotation to apply (loaded at boot, or identity on clear).
pub(crate) static LEVEL_CALIBRATION_SIGNAL: Signal<CriticalSectionRawMutex, UnitQuaternion<f32>> =
    Signal::new();

/// Core0 → Core1: start collecting a new calibration.
pub(crate) static LEVEL_CAL_START_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Core1 → Core0: the finished calibration, already applied on Core1 when `Ok`.
pub(crate) static LEVEL_CAL_RESULT_SIGNAL: Signal<
    CriticalSectionRawMutex,
    Result<UnitQuaternion<f32>, LevelCalFail>,
> = Signal::new();

/// Level calibration state as reported to the host.
#[derive(Clone, Copy, Default, defmt::Format)]
pub struct LevelCalStatus {
    /// Mounting offset, in the attitude telemetry's sign convention
    /// (what the uncorrected attitude reads with the airframe level).
    pub roll_deg: f32,
    pub pitch_deg: f32,
    pub calibrated: bool,
    pub collecting: bool,
}

/// Current status, read by the RPC `GetLevelCal` handler.
pub(crate) static STATUS: Mutex<CriticalSectionRawMutex, Cell<LevelCalStatus>> =
    Mutex::new(Cell::new(LevelCalStatus {
        roll_deg: 0.0,
        pitch_deg: 0.0,
        calibrated: false,
        collecting: false,
    }));

const LOAD_TIMEOUT: Duration = Duration::from_secs(2);
const SAVE_TIMEOUT: Duration = Duration::from_secs(5);

#[must_use]
pub fn status() -> LevelCalStatus {
    STATUS.lock(Cell::get)
}

#[must_use]
pub fn is_collecting() -> bool {
    status().collecting
}

fn set_mount(mount: Option<&UnitQuaternion<f32>>) {
    let (roll_deg, pitch_deg) = mount.map_or((0.0, 0.0), mount_to_display_deg);
    STATUS.lock(|c| {
        c.set(LevelCalStatus {
            roll_deg,
            pitch_deg,
            calibrated: mount.is_some(),
            collecting: false,
        });
    });
}

async fn flash_round_trip(request: FlashRequest, timeout: Duration) -> Option<FlashResponse> {
    FLASH_REQUEST_SIGNAL.signal(request);
    match select(FLASH_RESPONSE_SIGNAL.wait(), Timer::after(timeout)).await {
        Either::First(response) => Some(response),
        Either::Second(()) => None,
    }
}

/// Boot: load the stored mount from flash and hand it to Core1.
pub async fn load_from_flash() {
    match flash_round_trip(FlashRequest::LoadLevelCal, LOAD_TIMEOUT).await {
        Some(FlashResponse::LevelCalLoaded { data }) => match mount_from_bytes(&data) {
            Some(mount) => {
                LEVEL_CALIBRATION_SIGNAL.signal(mount);
                set_mount(Some(&mount));
                let s = status();
                crate::elle_event!(
                    info,
                    event::EVT_LEVEL_CAL_LOADED,
                    "Level cal loaded: roll {} pitch {} deg",
                    s.roll_deg,
                    s.pitch_deg
                );
            }
            // Identity (a cleared cal) or invalid data: run uncorrected.
            None => crate::elle_event!(
                info,
                event::EVT_LEVEL_CAL_LOAD_EMPTY,
                "No level cal in flash"
            ),
        },
        Some(FlashResponse::LevelCalEmpty) => crate::elle_event!(
            info,
            event::EVT_LEVEL_CAL_LOAD_EMPTY,
            "No level cal in flash"
        ),
        Some(_) => defmt::warn!("Level cal: unexpected flash response on load"),
        None => defmt::warn!("Level cal: flash load timed out, running uncorrected"),
    }
}

/// Start collecting. The aircraft must be still at its reference attitude.
pub fn start() {
    LEVEL_CAL_START_SIGNAL.signal(());
    STATUS.lock(|c| {
        let mut s = c.get();
        s.collecting = true;
        c.set(s);
    });
    crate::elle_event!(
        info,
        event::EVT_LEVEL_CAL_STARTED,
        "Level calibration started: hold still at reference attitude"
    );
}

/// Drop the calibration: Core1 runs uncorrected and the flash entry is removed.
pub async fn clear() {
    LEVEL_CALIBRATION_SIGNAL.signal(UnitQuaternion::identity());
    set_mount(None);
    let entry = ProfileEntry::LevelCal;
    let _ = flash_round_trip(FlashRequest::ClearProfileEntry { entry }, SAVE_TIMEOUT).await;
    crate::elle_event!(
        info,
        event::EVT_LEVEL_CAL_CLEARED,
        "Level calibration cleared"
    );
}

/// Handle a finished calibration from Core1, if there is one: update status and
/// persist it. Call every control-loop iteration; returns immediately when idle.
/// A save blocks the caller for up to `SAVE_TIMEOUT`, which only happens on the
/// ground right after a calibration.
pub async fn poll_result() {
    let Some(result) = LEVEL_CAL_RESULT_SIGNAL.try_take() else {
        return;
    };
    match result {
        Ok(mount) => {
            set_mount(Some(&mount));
            let s = status();
            crate::elle_event!(
                info,
                event::EVT_LEVEL_CAL_COMPLETE,
                "Level calibration complete: roll {} pitch {} deg",
                s.roll_deg,
                s.pitch_deg
            );
            let data = mount_to_bytes(&mount);
            match flash_round_trip(FlashRequest::SaveLevelCal { data }, SAVE_TIMEOUT).await {
                Some(FlashResponse::LevelCalSaved) => {
                    crate::elle_event!(info, event::EVT_LEVEL_CAL_SAVED, "Level cal saved");
                }
                _ => crate::elle_event!(
                    warn,
                    event::EVT_LEVEL_CAL_SAVE_FAILED,
                    "Level cal flash save failed (applied until reboot)"
                ),
            }
        }
        Err(fail) => {
            // Core1 keeps the previous mount on failure; only the flag changes.
            STATUS.lock(|c| {
                let mut s = c.get();
                s.collecting = false;
                c.set(s);
            });
            match fail {
                LevelCalFail::Moving => crate::elle_event!(
                    warn,
                    event::EVT_LEVEL_CAL_FAILED_MOVING,
                    "Level calibration failed: aircraft moved"
                ),
                LevelCalFail::Tilted => crate::elle_event!(
                    warn,
                    event::EVT_LEVEL_CAL_FAILED_TILTED,
                    "Level calibration failed: tilt beyond limit"
                ),
            }
        }
    }
}
