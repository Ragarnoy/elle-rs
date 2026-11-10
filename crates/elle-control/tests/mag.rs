//! Host tests for magnetometer health and calibration (`elle_control::mag`).
//!
//!   cargo test -p elle-control --target x86_64-unknown-linux-gnu --test mag

use elle_config::{
    MAG_CAL_SAMPLES, MAG_COUNTS_PER_GAUSS, MAG_DROP_S, MAG_READ_HZ, MAG_RESTORE_S, MAG_STALE_S,
};
use elle_control::mag::{CalFail, CalStep, Calibration, Despike, DropReason, MagChange, MagHealth};

/// The eagle's hard-iron offset, counts.
const OFFSET: [f32; 3] = [240_000.0, -130_000.0, 40_000.0];
/// The earth's field here, gauss.
const EARTH_G: f32 = 0.47;

fn n(secs: f32) -> usize {
    (secs * MAG_READ_HZ).ceil() as usize
}

/// Direction `k` of a smooth spiral from the top of the sphere to the bottom
/// in `total` steps, ten turns: what a slow calibration rotation looks like
/// (neighbours ~0.2 rad apart). Wraps after `total`.
fn direction(k: usize, total: usize) -> [f32; 3] {
    let k = k % total;
    let z = 1.0 - 2.0 * (k as f32 + 0.5) / total as f32;
    let r = (1.0 - z * z).sqrt();
    let phi = core::f32::consts::TAU * 10.0 * k as f32 / total as f32;
    [r * phi.cos(), r * phi.sin(), z]
}

/// A raw reading: `OFFSET` plus a field of `gauss` along `dir`.
fn reading(dir: [f32; 3], gauss: f32) -> [f32; 3] {
    core::array::from_fn(|i| OFFSET[i] + dir[i] * gauss * MAG_COUNTS_PER_GAUSS)
}

fn calibrate(readings: impl Iterator<Item = [f32; 3]>) -> CalStep {
    let mut cal = Calibration::new();
    let mut d = Despike::new();
    let mut last = CalStep::Collecting(0);
    for r in readings {
        last = cal.step(&d.push(r).0);
        if !matches!(last, CalStep::Collecting(_)) {
            break;
        }
    }
    last
}

#[test]
fn despike_drops_a_single_spike_and_flags_it() {
    let mut d = Despike::new();
    let base = [1000.0, 2000.0, 3000.0];
    d.push(base);
    d.push(base);
    let spike = [1000.0, 7000.0, -3000.0];
    let (out, flagged) = d.push(spike);
    assert_eq!(out, base);
    assert!(flagged);
    let (out, flagged) = d.push(base);
    assert_eq!(out, base);
    assert!(!flagged);
}

#[test]
fn despike_passes_a_real_step_one_reading_late() {
    let mut d = Despike::new();
    let a = [0.0; 3];
    let b = [1500.0, -1500.0, 1500.0];
    d.push(a);
    d.push(a);
    assert_eq!(d.push(b).0, a);
    assert_eq!(d.push(b).0, b);
}

#[test]
fn calibration_finds_the_offset_of_a_full_rotation() {
    let total = MAG_CAL_SAMPLES as usize;
    let step = calibrate((0..total).map(|k| reading(direction(k, total), EARTH_G)));
    let CalStep::Done(o) = step else {
        panic!("{step:?}")
    };
    for i in 0..3 {
        // The min/max box of the spiral misses the extremes by a little.
        assert!((o[i] - OFFSET[i]).abs() < 300.0, "{o:?}");
    }
}

#[test]
fn spikes_do_not_move_the_offset() {
    let total = MAG_CAL_SAMPLES as usize;
    let step = calibrate((0..total).map(|k| {
        let mut r = reading(direction(k, total), EARTH_G);
        // 3 % of reads, the eagle's signature.
        if k % 33 == 7 {
            r[1] += 5000.0;
            r[2] -= 6000.0;
        }
        r
    }));
    let CalStep::Done(o) = step else {
        panic!("{step:?}")
    };
    for i in 0..3 {
        assert!((o[i] - OFFSET[i]).abs() < 300.0, "{o:?}");
    }
}

#[test]
fn too_little_rotation_is_rejected() {
    let total = MAG_CAL_SAMPLES as usize;
    // Only the top cap: z barely moves.
    let step = calibrate((0..total).map(|k| {
        let d = direction(k % 20, total);
        reading(d, EARTH_G)
    }));
    assert!(
        matches!(step, CalStep::Failed(CalFail::Rotation { .. })),
        "{step:?}"
    );
}

#[test]
fn a_field_much_stronger_than_the_earths_is_rejected() {
    // LOG_0005 on the dart: "successful" calibrations with a 4.5–7.6 G radius.
    let total = MAG_CAL_SAMPLES as usize;
    let step = calibrate((0..total).map(|k| reading(direction(k, total), 4.5)));
    let CalStep::Failed(CalFail::Field { radius_g }) = step else {
        panic!("{step:?}")
    };
    assert!((radius_g - 4.5).abs() < 0.5, "{radius_g}");
}

#[test]
fn the_earths_field_is_fused_after_the_drop_window_silently() {
    let mut h = MagHealth::new();
    let mut changes = Vec::new();
    for k in 0..n(MAG_DROP_S) {
        let r = reading(direction(k, 50), EARTH_G);
        let field = core::array::from_fn(|i| r[i] - OFFSET[i]);
        changes.extend(h.update(&r, &field));
    }
    assert!(h.fused());
    assert!(changes.is_empty(), "{changes:?}");
}

#[test]
fn a_3_gauss_residual_is_dropped_and_reported_once() {
    // The eagle after its calibration: ~3 G left over.
    let mut h = MagHealth::new();
    let mut changes = Vec::new();
    for k in 0..n(MAG_DROP_S) * 4 {
        let r = reading(direction(k, 50), 3.0);
        let field = core::array::from_fn(|i| r[i] - OFFSET[i]);
        changes.extend(h.update(&r, &field));
    }
    assert!(!h.fused());
    assert_eq!(changes.len(), 1, "{changes:?}");
    assert!(matches!(
        changes[0],
        MagChange::Dropped(DropReason::Implausible { gauss }) if (gauss - 3.0).abs() < 0.01
    ));
}

#[test]
fn a_dropped_field_comes_back_only_after_the_restore_window() {
    let mut h = MagHealth::new();
    let mut k = 0;
    let mut feed = |h: &mut MagHealth, gauss: f32, count: usize| {
        let mut out = Vec::new();
        for _ in 0..count {
            let r = reading(direction(k, 50), gauss);
            k += 1;
            let field = core::array::from_fn(|i| r[i] - OFFSET[i]);
            out.extend(h.update(&r, &field));
        }
        out
    };
    feed(&mut h, EARTH_G, n(MAG_DROP_S));
    assert!(h.fused());
    let dropped = feed(&mut h, 2.0, n(MAG_DROP_S));
    assert!(
        matches!(dropped[..], [MagChange::Dropped(_)]),
        "{dropped:?}"
    );
    // Good again, but not for long enough.
    let early = feed(&mut h, EARTH_G, n(MAG_RESTORE_S) - 1);
    assert!(early.is_empty() && !h.fused(), "{early:?}");
    let restored = feed(&mut h, EARTH_G, 1);
    assert_eq!(restored, vec![MagChange::Restored]);
    assert!(h.fused());
}

#[test]
fn a_frozen_reading_is_dropped_as_stale() {
    let mut h = MagHealth::new();
    let r = reading(direction(3, 50), EARTH_G);
    let field = core::array::from_fn(|i| r[i] - OFFSET[i]);
    let mut changes = Vec::new();
    for _ in 0..n(MAG_STALE_S) + n(MAG_DROP_S) {
        changes.extend(h.update(&r, &field));
    }
    assert!(!h.fused());
    assert_eq!(changes, vec![MagChange::Dropped(DropReason::Stale)]);
}

/// The repeat count saturates at `u16::MAX` (~1.8 h at 10 Hz); a reading
/// frozen longer than that must stay stale, not overflow or read as healthy.
#[test]
fn a_reading_frozen_past_the_repeat_counter_stays_stale() {
    let mut h = MagHealth::new();
    let r = reading(direction(3, 50), EARTH_G);
    let field = core::array::from_fn(|i| r[i] - OFFSET[i]);
    let mut changes = Vec::new();
    for _ in 0..usize::from(u16::MAX) + n(MAG_RESTORE_S) + 10 {
        changes.extend(h.update(&r, &field));
    }
    assert!(!h.fused());
    assert_eq!(changes, vec![MagChange::Dropped(DropReason::Stale)]);
}

#[test]
fn new_offsets_after_a_drop_report_the_restore() {
    let mut h = MagHealth::new();
    for k in 0..n(MAG_DROP_S) {
        let r = reading(direction(k, 50), 3.0);
        let field = core::array::from_fn(|i| r[i] - OFFSET[i]);
        h.update(&r, &field);
    }
    assert!(!h.fused());
    // A calibration fixes it: judged afresh, accepted after the short window.
    h.reset();
    let mut changes = Vec::new();
    for k in 0..n(MAG_DROP_S) {
        let r = reading(direction(k, 50), EARTH_G);
        let field = core::array::from_fn(|i| r[i] - OFFSET[i]);
        changes.extend(h.update(&r, &field));
    }
    assert!(h.fused());
    assert_eq!(changes, vec![MagChange::Restored]);
}
