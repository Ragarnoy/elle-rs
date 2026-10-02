#![no_std]

//! Navigation for the elle flight controller, hardware-independent.
//!
//! Timestamped GNSS and baro samples go in; a [`NavState`] with per-field
//! validity and a bounded lateral [`Guidance`] demand come out. Nothing here
//! touches hardware or Embassy, so logs can be replayed and failures simulated
//! on the host (`crates/elle-nav/tests/`).
//!
//! The firmware runs it in **observation mode**: the demand for a loiter
//! around home is computed and logged every update, and never reaches the
//! attitude controller. See `docs/NAVIGATION_PLAN.md`.

pub mod estimate;
pub mod geo;
pub mod l1;

pub use estimate::{
    BaroSample, Estimator, EstimatorConfig, GnssSample, GnssVelocity, Home, NavState,
};
pub use geo::{GeoPoint, HomeNed, LocalFrame, Ne};
pub use l1::{Guidance, L1Config, Path, Phase};

/// Status bits, as logged in ULog `nav.status`.
pub mod status {
    /// A home exists (the local frame's origin).
    pub const HOME_SET: u16 = 1 << 0;
    /// Home is locked (taken before arming, held for the flight).
    pub const HOME_LOCKED: u16 = 1 << 1;
    pub const POS_VALID: u16 = 1 << 2;
    pub const VEL_VALID: u16 = 1 << 3;
    /// Baro height above home is valid.
    pub const ALT_VALID: u16 = 1 << 4;
    /// No usable fix for longer than expected: position is coasting on the
    /// last one (`NAV_COAST_AFTER_MS` until it drops at `NAV_FIX_TIMEOUT_MS`).
    /// Logs from 0.2.0 and earlier set this bit whenever velocity was valid.
    pub const COASTING: u16 = 1 << 5;
    pub const GUIDANCE_VALID: u16 = 1 << 6;
    pub const BANK_LIMITED: u16 = 1 << 7;
    /// Loiter capture (heading for the circle) rather than on it.
    pub const LOITER_CAPTURE: u16 = 1 << 8;
    /// Position and velocity valid but ground speed below the guidance minimum.
    pub const TOO_SLOW: u16 = 1 << 9;
    /// GNSS height above home is valid.
    pub const GNSS_ALT_VALID: u16 = 1 << 10;
}

/// One navigator update.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NavOutput {
    pub state: NavState,
    /// Lateral demand for the reference path, when position and velocity allow.
    pub guidance: Option<Guidance>,
    /// The demand must not be used after this time, µs since boot: the fix it
    /// came from expires then.
    pub valid_until_us: u64,
    /// Distance to home, m (meaningful when `status::POS_VALID`).
    pub home_dist_m: f32,
    /// Bearing to home, degrees clockwise from north, in (-180, 180].
    pub home_bearing_deg: f32,
    /// [`status`] bits.
    pub status: u16,
}

/// Estimator plus lateral guidance on a reference path around home.
#[derive(Clone, Copy, Debug)]
pub struct Navigator {
    est: Estimator,
    l1: L1Config,
    fix_timeout_us: u64,
    armed: bool,
}

impl Navigator {
    #[must_use]
    pub const fn new(est: EstimatorConfig, l1: L1Config) -> Self {
        Self {
            fix_timeout_us: est.fix_timeout_us,
            est: Estimator::new(est),
            l1,
            armed: false,
        }
    }

    /// The firmware configuration (`NAV_*` in elle-config).
    #[must_use]
    pub const fn from_config() -> Self {
        Self::new(EstimatorConfig::from_config(), L1Config::from_config())
    }

    #[must_use]
    pub const fn home(&self) -> Option<&Home> {
        self.est.home()
    }

    /// Track arming: home locks on the arming edge and is released on disarm.
    pub fn set_armed(&mut self, armed: bool) {
        if armed && !self.armed {
            self.est.lock_home();
        } else if !armed && self.armed {
            self.est.unlock_home();
        }
        self.armed = armed;
    }

    pub fn on_gnss(&mut self, s: GnssSample) {
        self.est.on_gnss(s, self.armed);
    }

    pub fn on_baro(&mut self, b: BaroSample) {
        self.est.on_baro(b, self.armed);
    }

    /// The observation reference path: a loiter around home (`NAV_LOITER_*`).
    #[must_use]
    pub const fn reference_path() -> Path {
        Path::Loiter {
            center: Ne::ZERO,
            radius_m: elle_config::NAV_LOITER_RADIUS_M,
            clockwise: elle_config::NAV_LOITER_CLOCKWISE,
        }
    }

    /// State and guidance for `path` at `now_us`.
    #[must_use]
    pub fn update(&self, now_us: u64, path: &Path) -> NavOutput {
        let state = self.est.state(now_us);
        let mut status = 0;
        let mut set = |bit: u16, on: bool| {
            if on {
                status |= bit;
            }
        };
        let home = self.est.home();
        set(status::HOME_SET, home.is_some());
        set(status::HOME_LOCKED, home.is_some_and(|h| h.locked));
        set(status::POS_VALID, state.pos_valid);
        set(status::VEL_VALID, state.vel_valid);
        set(status::ALT_VALID, state.alt_valid);
        set(status::COASTING, state.coasting);
        set(status::GNSS_ALT_VALID, state.gnss_alt_rel_m.is_some());

        let guidance = (state.pos_valid && state.vel_valid)
            .then(|| l1::guide(&self.l1, path, state.pos, state.vel))
            .flatten();
        set(
            status::TOO_SLOW,
            state.pos_valid && state.vel_valid && guidance.is_none(),
        );
        if let Some(g) = &guidance {
            set(status::GUIDANCE_VALID, true);
            set(status::BANK_LIMITED, g.bank_limited);
            set(status::LOITER_CAPTURE, g.phase == Phase::LoiterCapture);
        }

        NavOutput {
            state,
            guidance,
            valid_until_us: if guidance.is_some() {
                (now_us + self.fix_timeout_us).saturating_sub(state.fix_age_us)
            } else {
                0
            },
            home_dist_m: state.pos.norm(),
            home_bearing_deg: (-state.pos).bearing_rad().to_degrees(),
            status,
        }
    }
}
