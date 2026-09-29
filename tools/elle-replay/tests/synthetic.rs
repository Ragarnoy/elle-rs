//! End to end on a synthetic log: fly the firmware path (fuse, then record),
//! write it with the firmware's own ULog encoder, read it back with the replay
//! tool's reader, replay it, and check the faithfulness verdict.
//!
//!   cargo test -p elle-replay --target x86_64-unknown-linux-gnu

use elle_control::attitude::AttitudePipeline;
use elle_control::imu_raw::{ACCEL_SCALE, GYRO_SCALE, Record, Recorder, decode, encode};
use elle_replay::ulog::ULog;
use elle_ulog::{AttitudeMessage, ImuRawCtxMessage, ImuRawMagMessage, ImuRawMessage, ULogWriter};
use embassy_time::Instant;
use nalgebra::{UnitQuaternion, Vector3};

const SAMPLES: u32 = 6000;
/// Batches starting in here are lost (a full queue, say).
const GAP: std::ops::Range<u32> = 2210..3090;
/// The control loop logs the newest attitude every 5 ms, ~2 ms after it was sampled.
const LOG_EVERY: u32 = 5;
const LOG_DELAY_US: u64 = 2_000;

/// A deterministic IMU stream as the driver produces it (integers × scale).
fn imu(i: u32) -> ((f32, f32, f32), (f32, f32, f32)) {
    let t = i as f32 * 1e-3;
    let g = |x: f32| decode(encode(x, GYRO_SCALE), GYRO_SCALE);
    let a = |x: f32| decode(encode(x, ACCEL_SCALE), ACCEL_SCALE);
    (
        (
            g(0.5 * (1.7 * t).sin()),
            g(0.3 * (0.9 * t).cos()),
            g(0.2 * (0.4 * t).sin()),
        ),
        (
            a(1.5 * (0.6 * t).sin()),
            a(0.8 * (1.1 * t).cos()),
            a(9.81 + 0.3 * (3.0 * t).sin()),
        ),
    )
}

struct Flown {
    ulog: Vec<u8>,
    /// Indices of the samples whose attitude was logged.
    logged: Vec<u32>,
}

fn fly(corrupt_one: bool) -> Flown {
    let mut w = ULogWriter::new();
    let mut out = Vec::new();
    let mut flush = |w: &mut ULogWriter| {
        out.extend_from_slice(w.buffer());
        w.clear_buffer();
    };
    w.initialize(Instant::from_micros(0)).unwrap();
    w.write_definitions("ELLE-RS", "test", "0", 0).unwrap();
    let att_id = w.add_subscription(AttitudeMessage::NAME).unwrap();
    let raw_id = w.add_subscription(ImuRawMessage::NAME).unwrap();
    let mag_id = w.add_subscription(ImuRawMagMessage::NAME).unwrap();
    let ctx_id = w.add_subscription(ImuRawCtxMessage::NAME).unwrap();
    flush(&mut w);

    let mut p = AttitudePipeline::new();
    let mut rec = Recorder::new();
    let mut logged = Vec::new();
    for i in 0..SAMPLES {
        if i == 700 {
            p.gyro_bias = Vector3::new(0.002, -0.001, 0.0005);
        }
        if i == 1500 {
            p.mount = UnitQuaternion::from_euler_angles(0.02, -0.01, 0.0);
        }
        // The mag drops out during the gap and comes back after it.
        let mag =
            (!(2500..3500).contains(&i)).then(|| p.mag_to_airframe(Vector3::new(0.2, 0.05, -0.4)));
        let (g, a) = imu(i);
        let q = p.quat();
        let att = p.fuse(
            p.debias(Vector3::new(g.0, g.1, g.2)),
            Vector3::new(a.0, a.1, a.2),
            mag.as_ref(),
        );
        let t_us = 100_000 + u64::from(i) * 1000;
        let mut records = Vec::new();
        rec.sample(
            t_us,
            g,
            a,
            30.0,
            mag.as_ref(),
            &q,
            &p.gyro_bias,
            &p.mount,
            |r| {
                records.push(r);
            },
        );
        for r in records {
            match r {
                Record::Batch(b) if !GAP.contains(&b.first_index) => w
                    .write_imu_raw(
                        raw_id,
                        &ImuRawMessage {
                            timestamp: b.t_us,
                            first_index: b.first_index,
                            count: b.count,
                            temp_centi_c: b.temp_centi_c,
                            data: b.data,
                        },
                    )
                    .unwrap(),
                Record::Batch(_) => {}
                Record::Mag(m) => w
                    .write_imu_raw_mag(
                        mag_id,
                        &ImuRawMagMessage {
                            timestamp: m.t_us,
                            index: m.index,
                            valid: m.mag.is_some().into(),
                            mag: m.mag.unwrap_or([0.0; 3]),
                        },
                    )
                    .unwrap(),
                Record::Ctx(c) => w
                    .write_imu_raw_ctx(
                        ctx_id,
                        &ImuRawCtxMessage {
                            timestamp: c.t_us,
                            index: c.index,
                            quat: c.quat,
                            gyro_bias: c.gyro_bias,
                            mount: c.mount,
                            roundtrip_errors: c.roundtrip_errors,
                        },
                    )
                    .unwrap(),
            }
        }
        if i % LOG_EVERY == 0 {
            let roll = if corrupt_one && i == 5000 {
                att.roll + 1e-6
            } else {
                att.roll
            };
            w.write_attitude(
                att_id,
                &AttitudeMessage::new(
                    Instant::from_micros(t_us + LOG_DELAY_US),
                    att.pitch,
                    roll,
                    att.yaw,
                    att.pitch_rate,
                    att.roll_rate,
                    att.yaw_rate,
                ),
            )
            .unwrap();
            logged.push(i);
        }
        flush(&mut w);
    }
    Flown { ulog: out, logged }
}

#[test]
fn replay_reproduces_every_synced_attitude() {
    let flown = fly(false);
    let log = ULog::parse(&flown.ulog).unwrap();
    let replay = elle_replay::replay(&log).unwrap();
    let c = &replay.coverage;

    let lost = (GAP.start.div_ceil(10) * 10..GAP.end.div_ceil(10) * 10).len() as u64;
    assert_eq!(c.samples, u64::from(SAMPLES) - lost);
    assert_eq!((c.gaps, c.lost), (1, lost));
    assert_eq!(c.roundtrip_errors, 0);

    // Synced from the first context (sample 0) to the gap, and again from the
    // first context after it (sample 4000, the next periodic one).
    let resume = 4000;
    let synced = |i: &u32| *i < GAP.start.div_ceil(10) * 10 || *i >= resume;
    assert_eq!(c.synced, (0..SAMPLES).filter(synced).count() as u64);

    let f = elle_replay::faithfulness(&log, &replay);
    let expect_exact = flown.logged.iter().filter(|i| synced(i)).count() as u64;
    assert_eq!(f.mismatched, 0, "{f:?}");
    assert_eq!(f.exact, expect_exact, "{f:?}");
    assert!(f.ok());
}

#[test]
fn a_changed_attitude_is_caught() {
    let log = ULog::parse(&fly(true).ulog).unwrap();
    let replay = elle_replay::replay(&log).unwrap();
    let f = elle_replay::faithfulness(&log, &replay);
    assert_eq!(f.mismatched, 1, "{f:?}");
    assert!(!f.ok());
}

#[test]
fn csv_has_a_row_per_synced_sample() {
    let log = ULog::parse(&fly(false).ulog).unwrap();
    let replay = elle_replay::replay(&log).unwrap();
    let mut buf = Vec::new();
    elle_replay::write_csv(&replay, &mut buf).unwrap();
    let rows = String::from_utf8(buf).unwrap().lines().count() as u64;
    assert_eq!(rows, 1 + replay.coverage.synced);
}
