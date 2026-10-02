//! Estimator gating, home capture and staleness through the `Navigator`.

use elle_nav::{BaroSample, GeoPoint, GnssSample, GnssVelocity, Navigator, Ne, status};

const HOME: GeoPoint = GeoPoint::new(488_566_000, 23_522_000);
const S: u64 = 1_000_000;

fn fix(t_us: u64, pos: GeoPoint, vel: Option<Ne>) -> GnssSample {
    GnssSample {
        t_us,
        fix_ok: true,
        num_satellites: 12,
        pos,
        alt_msl_m: 100.0,
        h_acc_m: Some(2.0),
        v_acc_m: Some(3.0),
        vel: vel.map(|ne| GnssVelocity { ne, s_acc_ms: 0.3 }),
    }
}

fn baro(t_us: u64, alt_m: f32) -> BaroSample {
    BaroSample {
        t_us,
        alt_m,
        climb_ms: 0.0,
    }
}

/// ~111 m north of HOME.
const NORTH: GeoPoint = GeoPoint::new(488_576_000, 23_522_000);

fn nav() -> (Navigator, elle_nav::Path) {
    (Navigator::from_config(), Navigator::reference_path())
}

#[test]
fn nothing_valid_without_home() {
    let (n, p) = nav();
    let out = n.update(S, &p);
    assert_eq!(out.status, 0);
    assert!(out.guidance.is_none());
}

#[test]
fn home_follows_on_the_ground_and_locks_at_arming() {
    let (mut n, p) = nav();
    n.on_baro(baro(S, 50.0));
    n.on_gnss(fix(S, HOME, Some(Ne::ZERO)));
    assert_eq!(n.home().unwrap().pos, HOME);
    assert_eq!(n.home().unwrap().baro_alt_m, Some(50.0));
    n.set_armed(true);
    n.on_gnss(fix(2 * S, NORTH, Some(Ne::new(15.0, 0.0))));
    let h = n.home().unwrap();
    assert!(h.locked);
    assert_eq!(h.pos, HOME, "home must not move in the air");

    n.on_baro(baro(2 * S, 80.0));
    let out = n.update(2 * S, &p);
    assert!(out.status & status::HOME_LOCKED != 0);
    assert!((out.state.pos.n - 111.3).abs() < 0.5, "{:?}", out.state.pos);
    assert!(
        (out.home_bearing_deg.abs() - 180.0).abs() < 0.1,
        "home is due south"
    );
    assert!((out.state.alt_rel_m - 30.0).abs() < 1e-3);
    assert!((out.state.gnss_alt_rel_m.unwrap()).abs() < 1e-3);

    // Disarmed again (landed elsewhere), home follows the aircraft.
    n.set_armed(false);
    n.on_gnss(fix(3 * S, NORTH, Some(Ne::ZERO)));
    assert_eq!(n.home().unwrap().pos, NORTH);
}

#[test]
fn poor_fix_never_becomes_home() {
    let (mut n, _) = nav();
    let mut f = fix(S, HOME, Some(Ne::ZERO));
    f.h_acc_m = Some(elle_config::NAV_HOME_MAX_H_ACC_M + 1.0);
    n.on_gnss(f);
    assert!(n.home().is_none());
    let mut f = fix(S, HOME, Some(Ne::ZERO));
    f.num_satellites = elle_config::NAV_HOME_MIN_SATS - 1;
    n.on_gnss(f);
    assert!(n.home().is_none());
}

#[test]
fn gga_fallback_gives_position_without_velocity_or_guidance() {
    let (mut n, p) = nav();
    n.on_gnss(fix(S, HOME, Some(Ne::ZERO)));
    n.set_armed(true);
    // GGA carries no accuracy estimate: not usable for position either,
    // so the last PVT fix ages out.
    let late = S + u64::from(elle_config::NAV_FIX_TIMEOUT_MS) * 1000 + 1;
    let mut gga = fix(late, NORTH, None);
    gga.h_acc_m = None;
    n.on_gnss(gga);
    let out = n.update(late, &p);
    assert!(
        out.status & status::POS_VALID == 0,
        "stale PVT fix older than the timeout"
    );
    assert!(out.guidance.is_none());

    // A PVT fix without a usable speed accuracy: position yes, velocity no.
    let mut f = fix(3 * S, NORTH, Some(Ne::new(15.0, 0.0)));
    f.vel.as_mut().unwrap().s_acc_ms = 5.0;
    n.on_gnss(f);
    let out = n.update(3 * S, &p);
    assert!(out.status & status::POS_VALID != 0);
    assert!(out.status & status::VEL_VALID == 0);
    assert!(out.guidance.is_none());
}

#[test]
fn position_extrapolates_briefly_then_expires() {
    let (mut n, p) = nav();
    n.on_gnss(fix(S, HOME, Some(Ne::ZERO)));
    n.set_armed(true);
    n.on_gnss(fix(2 * S, NORTH, Some(Ne::new(0.0, 20.0))));

    let at = |ms: u64| n.update(2 * S + ms * 1000, &p);
    let o = at(100);
    assert!((o.state.pos.e - 2.0).abs() < 0.01, "{:?}", o.state.pos);
    assert!(!o.state.coasting, "100 ms after a fix is not coasting");
    assert!(o.guidance.is_some());
    assert_eq!(
        o.valid_until_us,
        2 * S + u64::from(elle_config::NAV_FIX_TIMEOUT_MS) * 1000
    );
    // Capped at NAV_EXTRAPOLATE_MAX_MS.
    let o = at(900);
    let cap = elle_config::NAV_EXTRAPOLATE_MAX_MS as f32 * 1e-3 * 20.0;
    assert!((o.state.pos.e - cap).abs() < 0.01, "{:?}", o.state.pos);
    // Gone after NAV_FIX_TIMEOUT_MS.
    let o = at(u64::from(elle_config::NAV_FIX_TIMEOUT_MS) + 1);
    assert!(o.status & status::POS_VALID == 0 && o.guidance.is_none());
}

#[test]
fn slow_on_the_ground_is_reported_not_guided() {
    let (mut n, p) = nav();
    n.on_gnss(fix(S, HOME, Some(Ne::new(0.5, 0.0))));
    let out = n.update(S, &p);
    assert!(out.status & status::TOO_SLOW != 0);
    assert!(out.guidance.is_none());
}

#[test]
fn stale_baro_drops_altitude() {
    let (mut n, p) = nav();
    n.on_baro(baro(S, 50.0));
    n.on_gnss(fix(S, HOME, Some(Ne::ZERO)));
    let late = S + u64::from(elle_config::NAV_BARO_TIMEOUT_MS) * 1000 + 1;
    assert!(n.update(S, &p).status & status::ALT_VALID != 0);
    assert!(n.update(late, &p).status & status::ALT_VALID == 0);
}

/// A live 5 Hz stream, sampled at 25 Hz between fixes, never reports coasting:
/// the fix age is always nonzero there, which is not a dropout.
#[test]
fn live_stream_is_not_coasting() {
    let (mut n, p) = nav();
    n.on_gnss(fix(S, HOME, Some(Ne::ZERO)));
    n.set_armed(true);
    for k in 0..25u64 {
        let t_fix = 2 * S + k * 200_000;
        n.on_gnss(fix(t_fix, NORTH, Some(Ne::new(0.0, 20.0))));
        for j in 0..5u64 {
            let o = n.update(t_fix + j * 40_000 + 1, &p);
            assert!(o.status & status::POS_VALID != 0);
            assert!(o.status & status::COASTING == 0, "fix {k}, tick {j}");
        }
    }
}

/// After a dropout the bit sets past NAV_COAST_AFTER_MS and stays until the
/// position drops at NAV_FIX_TIMEOUT_MS (TEST_PLAN 6.9 row 3).
#[test]
fn dropout_coasts_until_the_fix_times_out() {
    let (mut n, p) = nav();
    n.on_gnss(fix(S, HOME, Some(Ne::ZERO)));
    n.set_armed(true);
    n.on_gnss(fix(2 * S, NORTH, Some(Ne::new(0.0, 20.0))));
    let coast = u64::from(elle_config::NAV_COAST_AFTER_MS);
    let timeout = u64::from(elle_config::NAV_FIX_TIMEOUT_MS);
    let at = |ms: u64| n.update(2 * S + ms * 1000, &p);

    assert!(at(coast).status & status::COASTING == 0);
    let o = at(coast + 1);
    assert!(o.status & status::COASTING != 0 && o.status & status::POS_VALID != 0);
    let o = at(timeout);
    assert!(o.status & status::COASTING != 0 && o.status & status::POS_VALID != 0);
    let o = at(timeout + 1);
    assert!(o.status & status::POS_VALID == 0 && o.status & status::COASTING == 0);
}
