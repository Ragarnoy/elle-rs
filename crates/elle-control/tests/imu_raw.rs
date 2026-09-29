//! Host tests for the raw IMU capture (`elle_control::imu_raw`).
//!
//!   cargo test -p elle-control --target x86_64-unknown-linux-gnu --test imu_raw

use elle_control::attitude::{Attitude, AttitudePipeline};
use elle_control::imu_raw::{
    ACCEL_SCALE, BATCH_SAMPLES, CTX_INTERVAL, GYRO_SCALE, Record, Recorder, SAMPLE_BYTES, decode,
    encode, pack, unpack,
};
use nalgebra::{Quaternion, UnitQuaternion, Vector3};

const RANGE: i32 = 1 << 19;

#[test]
fn every_20_bit_value_survives_encode_decode() {
    for scale in [GYRO_SCALE, ACCEL_SCALE] {
        for raw in -RANGE..RANGE {
            let f = decode(raw, scale);
            assert_eq!(encode(f, scale), raw, "scale {scale}, raw {raw}");
            assert_eq!(decode(encode(f, scale), scale).to_bits(), f.to_bits());
        }
    }
}

#[test]
fn scales_match_the_datasheet() {
    // ±4000 dps and ±32 g over ±2^19 counts.
    assert!((GYRO_SCALE - 4000f32.to_radians() / 524_288.0).abs() < 1e-12);
    assert!((ACCEL_SCALE - 32.0 * 9.806_65 / 524_288.0).abs() < 1e-9);
}

#[test]
fn pack_round_trips_the_extremes() {
    let mut buf = [0u8; SAMPLE_BYTES];
    let gyro = [RANGE - 1, -RANGE, 0];
    let accel = [-1, 1, 123_456];
    pack(&mut buf, gyro, accel);
    assert_eq!(unpack(&buf), (gyro, accel));
}

/// A deterministic IMU stream as the driver produces it (integers × scale).
fn sample(i: u32) -> ((f32, f32, f32), (f32, f32, f32)) {
    let t = i as f32 * 1e-3;
    let g = |x: f32| decode(encode(x, GYRO_SCALE), GYRO_SCALE);
    let a = |x: f32| decode(encode(x, ACCEL_SCALE), ACCEL_SCALE);
    (
        (g(0.4 * (2.1 * t).sin()), g(0.2 * (1.3 * t).cos()), g(0.05)),
        (a(1.2 * (0.7 * t).sin()), a(0.6 * (1.9 * t).cos()), a(9.81)),
    )
}

fn v3((x, y, z): (f32, f32, f32)) -> Vector3<f32> {
    Vector3::new(x, y, z)
}

/// Fly the firmware path for `n` samples: fuse, then record. Bias arrives at
/// sample 700, a new mount at 1500, the mag appears at 300 and drops at 2600.
fn fly(n: u32) -> (Vec<Attitude>, Vec<Record>) {
    let mut p = AttitudePipeline::new();
    let mut rec = Recorder::new();
    let mut out = Vec::new();
    let mut records = Vec::new();
    for i in 0..n {
        if i == 700 {
            p.gyro_bias = Vector3::new(0.002, -0.001, 0.0005);
        }
        if i == 1500 {
            p.mount = UnitQuaternion::from_euler_angles(0.02, -0.01, 0.0);
        }
        let mag = (300..2600)
            .contains(&i)
            .then(|| p.mag_to_airframe(Vector3::new(0.2, 0.05, -0.4)));
        let (g, a) = sample(i);
        let q = p.quat();
        out.push(p.fuse(p.debias(v3(g)), v3(a), mag.as_ref()).unwrap());
        rec.sample(
            u64::from(i) * 1000,
            g,
            a,
            25.0,
            mag.as_ref(),
            &q,
            &p.gyro_bias,
            &p.mount,
            |r| records.push(r),
        );
    }
    (out, records)
}

#[test]
fn records_arrive_in_batches_with_context() {
    let (_, records) = fly(2500);
    let batches: Vec<_> = records
        .iter()
        .filter_map(|r| match r {
            Record::Batch(b) => Some(*b),
            _ => None,
        })
        .collect();
    assert_eq!(batches.len(), 250);
    assert!(
        batches
            .iter()
            .enumerate()
            .all(|(k, b)| b.first_index == (k * BATCH_SAMPLES) as u32
                && usize::from(b.count) == BATCH_SAMPLES)
    );

    let ctx: Vec<u32> = records
        .iter()
        .filter_map(|r| match r {
            Record::Ctx(c) => Some(c.index),
            _ => None,
        })
        .collect();
    // Periodic, plus the bias (700) and mount (1500) changes.
    assert_eq!(ctx, vec![0, 700, CTX_INTERVAL, 1500, 2 * CTX_INTERVAL]);

    let mags: Vec<(u32, bool)> = records
        .iter()
        .filter_map(|r| match r {
            Record::Mag(m) => Some((m.index, m.mag.is_some())),
            _ => None,
        })
        .collect();
    // The new mount at 1500 changes the vector as fed, so it is recorded again.
    assert_eq!(mags, vec![(0, false), (300, true), (1500, true)]);

    let Some(Record::Ctx(last)) = records.iter().rev().find(|r| matches!(r, Record::Ctx(_))) else {
        unreachable!()
    };
    assert_eq!(last.roundtrip_errors, 0);
}

#[test]
fn a_float_the_driver_cannot_produce_is_counted() {
    let mut rec = Recorder::new();
    let mut errors = 0;
    rec.sample(
        0,
        (0.123_456_7, 0.0, 0.0),
        (0.0, 0.0, 9.81),
        25.0,
        None,
        &UnitQuaternion::identity(),
        &Vector3::zeros(),
        &UnitQuaternion::identity(),
        |r| {
            if let Record::Ctx(c) = r {
                errors = c.roundtrip_errors;
            }
        },
    );
    assert_eq!(errors, 1);
}

/// Replay from the records alone; attitudes from sample `start` on.
///
/// A context is applied at its own sample; the mag in force is the latest
/// record at or before the sample, which after a gap may be one whose own
/// sample was lost.
fn replay(records: &[Record], start: u32) -> Vec<(u32, Attitude)> {
    let mut samples = Vec::new();
    let mut ctxs = Vec::new();
    let mut mags = Vec::new();
    for r in records {
        match r {
            Record::Batch(b) => {
                for k in 0..usize::from(b.count) {
                    let (g, a) = unpack(&b.data[k * SAMPLE_BYTES..]);
                    samples.push((b.first_index + k as u32, g, a));
                }
            }
            Record::Ctx(c) => ctxs.push(*c),
            Record::Mag(m) => mags.push(*m),
        }
    }
    let q = |v: [f32; 4]| UnitQuaternion::new_unchecked(Quaternion::new(v[0], v[1], v[2], v[3]));
    let mut p = AttitudePipeline::new();
    let mut out = Vec::new();
    for (index, g, a) in samples {
        if let Some(c) = ctxs.iter().find(|c| c.index == index) {
            p.set_quat(q(c.quat));
            p.gyro_bias = Vector3::from(c.gyro_bias);
            p.mount = q(c.mount);
        }
        let mag = mags
            .iter()
            .rev()
            .find(|m| m.index <= index)
            .and_then(|m| m.mag.map(Vector3::from));
        let gyro = Vector3::from(g.map(|r| decode(r, GYRO_SCALE)));
        let accel = Vector3::from(a.map(|r| decode(r, ACCEL_SCALE)));
        let att = p.fuse(p.debias(gyro), accel, mag.as_ref()).unwrap();
        if index >= start {
            out.push((index, att));
        }
    }
    out
}

#[test]
fn replay_from_records_is_bit_identical() {
    let (flown, records) = fly(3000);
    for (index, att) in replay(&records, 0) {
        assert_eq!(att, flown[index as usize], "sample {index}");
    }
}

#[test]
fn replay_resumes_exactly_after_a_gap() {
    let (flown, mut records) = fly(3000);
    // Lose the batches between samples 1020 and 1990: the replay picks up at
    // the next context (2000) and is exact again from there.
    records.retain(|r| !matches!(r, Record::Batch(b) if (1020..1990).contains(&b.first_index)));
    // Angles are exact from the context on. The rate filter's history is not
    // in the context (it re-converges within tens of ms), so rates are not.
    let resumed = replay(&records, 2000);
    assert_eq!(resumed.len(), 1000);
    for (index, att) in resumed {
        let f = flown[index as usize];
        assert_eq!(
            (att.pitch, att.roll, att.yaw),
            (f.pitch, f.roll, f.yaw),
            "sample {index}"
        );
    }
}
