//! Local frame against known geodesy.

use elle_nav::{GeoPoint, LocalFrame, Ne};

fn close(a: f32, b: f32, tol: f32) -> bool {
    (a - b).abs() <= tol
}

#[test]
fn origin_maps_to_zero() {
    let home = GeoPoint::from_deg(48.8566, 2.3522);
    let ne = LocalFrame::new(home).to_ne(home);
    assert!(ne.norm() < 1e-3, "{ne:?}");
}

#[test]
fn one_arc_minute_of_latitude() {
    // 1' of latitude is 1852.2 m at 45° on WGS84 (meridional radius of curvature).
    let home = GeoPoint::from_deg(45.0, 5.0);
    let ne = LocalFrame::new(home).to_ne(GeoPoint::from_deg(45.0 + 1.0 / 60.0, 5.0));
    assert!(close(ne.n, 1852.2, 0.5), "{ne:?}");
    assert!(ne.e.abs() < 0.01, "{ne:?}");
}

#[test]
fn longitude_shrinks_with_cos_latitude() {
    // 0.01° of longitude at 60° N: prime-vertical radius × cos φ = 558.0 m.
    let home = GeoPoint::from_deg(60.0, 10.0);
    let ne = LocalFrame::new(home).to_ne(GeoPoint::from_deg(60.0, 10.01));
    assert!(close(ne.e, 558.0, 0.05), "{ne:?}");
    // A parallel is not a straight line in the tangent plane: a point on the
    // same latitude 558 m east sits ~4 cm north of it.
    assert!(ne.n > 0.0 && ne.n < 0.06, "{ne:?}");
}

#[test]
fn receiver_resolution_survives() {
    // One unit of 1e-7 degree is ~1.1 cm of latitude; it must not be lost.
    let home = GeoPoint::new(488_566_000, 23_522_000);
    let f = LocalFrame::new(home);
    let one = f.to_ne(GeoPoint::new(488_566_001, 23_522_000));
    assert!(close(one.n, 0.0111, 0.001), "{one:?}");
}

#[test]
fn across_the_antimeridian() {
    let home = GeoPoint::from_deg(-17.0, 179.9995);
    let ne = LocalFrame::new(home).to_ne(GeoPoint::from_deg(-17.0, -179.9995));
    // 0.001° east at 17° S is ~106.5 m, not a trip round the world.
    assert!(close(ne.e, 106.5, 0.5), "{ne:?}");
}

#[test]
fn bearings_are_compass_bearings() {
    assert!(close(
        Ne::new(0.0, 1.0).bearing_rad().to_degrees(),
        90.0,
        1e-4
    ));
    assert!(close(
        Ne::new(-1.0, 0.0).bearing_rad().to_degrees().abs(),
        180.0,
        1e-4
    ));
    // `right` turns clockwise: north -> east.
    assert_eq!(Ne::new(1.0, 0.0).right(), Ne::new(0.0, 1.0));
}
