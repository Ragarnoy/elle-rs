//! Flight simulator: physically consistent IMU, mag and GNSS data for a known
//! flight, written as the ULog an `imu-raw-log` build would record.
//!
//! The aircraft is a point with an attitude, flying at constant airspeed along
//! its nose (no angle of attack or sideslip) in a constant wind: coordinated
//! turns, pull-ups and straight flight. Its motion is integrated in NED /
//! forward-right-down (FRD) at 1 kHz in `f64`; the sensors read exactly the
//! body rates and specific force of that motion, plus noise, vibration and a
//! residual gyro bias, quantised to the ICM-42686's 20-bit integers.
//!
//! The filter frame differs from FRD. From the firmware's verified conventions
//! (pitch positive nose up, roll positive right wing down after the pipeline
//! negates it, +1 g on z when level) the attitude filter's body axes are
//! forward = −x, right = +y, up = +z, and with the mag its earth frame is
//! north-west-up. [`FRD_TO_FILTER`] and [`NED_TO_FILTER`] map between them; a
//! test checks the mapped truth reads back as the pitch and bank flown.

use std::f64::consts::PI;

use anyhow::Result;
use elle_control::attitude::AttitudePipeline;
use elle_control::imu_raw::{ACCEL_SCALE, GYRO_SCALE, Record, Recorder, decode, encode};
use elle_ulog::{
    AttitudeMessage, GnssMessage, ImuRawCtxMessage, ImuRawMagMessage, ImuRawMessage, ULogWriter,
};
use embassy_time::Instant;
use nalgebra::{Matrix3, Rotation3, UnitQuaternion, Vector3};

/// Standard gravity, m/s².
pub const G: f64 = 9.806_65;
const DT: f64 = 1e-3;

/// FRD body axes → the filter's body axes (forward −x, right +y, up +z).
pub const FRD_TO_FILTER: Matrix3<f64> = Matrix3::new(-1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0);
/// NED → the filter's earth axes (north, west, up).
pub const NED_TO_FILTER: Matrix3<f64> = Matrix3::new(1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0);

/// One piece of the flight.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Segment {
    /// Wings level, constant heading.
    Straight { secs: f64 },
    /// Coordinated turn at `bank_deg` (positive right), rolled into and out of
    /// at [`ROLL_RATE_DPS`] within `secs`.
    Turn { bank_deg: f64, secs: f64 },
    /// Pitch up to `pitch_deg` and back to level over `secs` (a smooth pitch
    /// rate pulse: the load factor rises in the first half).
    PullUp { pitch_deg: f64, secs: f64 },
}

/// Roll rate into and out of turns, °/s.
pub const ROLL_RATE_DPS: f64 = 45.0;

/// Everything about the simulated aircraft and sensors.
#[derive(Clone, Debug, PartialEq)]
pub struct Config {
    pub airspeed_ms: f64,
    /// Wind, m/s, NED (the direction it blows towards).
    pub wind_ned: Vector3<f64>,
    pub start_heading_deg: f64,
    /// Gyro white noise, rad/s (1σ per sample).
    pub gyro_noise: f64,
    /// Accel white noise, m/s² (1σ per sample).
    pub accel_noise: f64,
    /// Motor vibration on the accel: amplitude m/s² at `vibration_hz`, each axis.
    pub vibration: f64,
    pub vibration_hz: f64,
    /// Gyro bias left after the boot estimate, rad/s (sensor frame, all axes).
    pub gyro_bias_residual: f64,
    /// GNSS solutions arrive this long after the instant they describe, s.
    pub gnss_latency_s: f64,
    /// GNSS velocity noise, m/s (1σ).
    pub gnss_vel_noise: f64,
    /// Earth field, NED, arbitrary units (magnetic north, dip down).
    pub mag_ned: Vector3<f64>,
    pub seed: u64,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            airspeed_ms: 15.0,
            wind_ned: Vector3::zeros(),
            start_heading_deg: 0.0,
            gyro_noise: 0.003,
            accel_noise: 0.05,
            vibration: 0.0,
            vibration_hz: 150.0,
            gyro_bias_residual: 0.0,
            gnss_latency_s: 0.08,
            gnss_vel_noise: 0.05,
            mag_ned: Vector3::new(0.21, 0.0, 0.43),
            seed: 1,
        }
    }
}

/// The attitude flown at one IMU sample, in the firmware's convention.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Truth {
    pub index: u32,
    pub t_us: u64,
    /// Radians: pitch positive nose up, roll positive right wing down, yaw as
    /// the filter reports it (from magnetic north in its own sense).
    pub pitch: f64,
    pub roll: f64,
    pub yaw: f64,
}

/// A simulated flight: the ULog bytes and the truth per sample.
pub struct Flight {
    pub ulog: Vec<u8>,
    pub truth: Vec<Truth>,
}

/// Small deterministic generator (xorshift64*), Gaussian by Box–Muller.
struct Rng(u64);

impl Rng {
    fn uniform(&mut self) -> f64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        ((self.0.wrapping_mul(0x2545_f491_4f6c_dd1d) >> 11) as f64 + 0.5) / (1u64 << 53) as f64
    }

    fn gauss(&mut self) -> f64 {
        (-2.0 * self.uniform().ln()).sqrt() * (2.0 * PI * self.uniform()).cos()
    }
}

/// Airframe state: Euler angles (ZYX: heading, pitch, bank) and position.
#[derive(Clone, Copy)]
struct State {
    heading: f64,
    pitch: f64,
    bank: f64,
    pos_ned: Vector3<f64>,
}

impl State {
    fn rotation(&self) -> Rotation3<f64> {
        Rotation3::from_euler_angles(self.bank, self.pitch, self.heading)
    }
}

/// Euler rates → FRD body rates.
fn body_rates(s: &State, bank_rate: f64, pitch_rate: f64, heading_rate: f64) -> Vector3<f64> {
    let (sp, cp) = s.pitch.sin_cos();
    let (sb, cb) = s.bank.sin_cos();
    Vector3::new(
        bank_rate - heading_rate * sp,
        pitch_rate * cb + heading_rate * cp * sb,
        -pitch_rate * sb + heading_rate * cp * cb,
    )
}

/// The filter-frame quaternion for an airframe attitude.
fn filter_quat(r: &Rotation3<f64>) -> UnitQuaternion<f64> {
    let m = NED_TO_FILTER * r.matrix() * FRD_TO_FILTER.transpose();
    UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(m))
}

/// Firmware-convention angles of a filter-frame quaternion: (pitch, roll, yaw)
/// with roll negated, as `AttitudePipeline::fuse` reports them.
#[must_use]
pub fn firmware_angles(q: &UnitQuaternion<f64>) -> (f64, f64, f64) {
    let (roll, pitch, yaw) = q.euler_angles();
    (pitch, -roll, yaw)
}

/// Collects ULog records into bytes.
struct Log {
    w: ULogWriter,
    out: Vec<u8>,
    att: u16,
    raw: u16,
    mag: u16,
    ctx: u16,
    gnss: u16,
}

impl Log {
    fn new() -> Result<Self> {
        let mut w = ULogWriter::new();
        let e = |e| anyhow::anyhow!("ULog writer: {e:?}");
        w.initialize(Instant::from_micros(0)).map_err(e)?;
        w.write_definitions("ELLE-SIM", "sim", "0", 0).map_err(e)?;
        let mut sub = |n| w.add_subscription(n).map_err(e);
        let (att, raw, mag, ctx, gnss) = (
            sub(AttitudeMessage::NAME)?,
            sub(ImuRawMessage::NAME)?,
            sub(ImuRawMagMessage::NAME)?,
            sub(ImuRawCtxMessage::NAME)?,
            sub(GnssMessage::NAME)?,
        );
        let mut log = Self {
            w,
            out: Vec::new(),
            att,
            raw,
            mag,
            ctx,
            gnss,
        };
        log.flush();
        Ok(log)
    }

    fn flush(&mut self) {
        self.out.extend_from_slice(self.w.buffer());
        self.w.clear_buffer();
    }

    fn record(&mut self, r: Record) {
        let _ = match r {
            Record::Batch(b) => self.w.write_imu_raw(
                self.raw,
                &ImuRawMessage {
                    timestamp: b.t_us,
                    first_index: b.first_index,
                    count: b.count,
                    temp_centi_c: b.temp_centi_c,
                    data: b.data,
                },
            ),
            Record::Mag(m) => self.w.write_imu_raw_mag(
                self.mag,
                &ImuRawMagMessage {
                    timestamp: m.t_us,
                    index: m.index,
                    valid: m.mag.is_some().into(),
                    mag: m.mag.unwrap_or([0.0; 3]),
                },
            ),
            Record::Ctx(c) => self.w.write_imu_raw_ctx(
                self.ctx,
                &ImuRawCtxMessage {
                    timestamp: c.t_us,
                    index: c.index,
                    quat: c.quat,
                    gyro_bias: c.gyro_bias,
                    mount: c.mount,
                    roundtrip_errors: c.roundtrip_errors,
                },
            ),
        };
        self.flush();
    }
}

/// Metres per 1e-7 degree of latitude (spherical, fine for a simulated field).
const M_PER_E7: f64 = 6_371_000.0 * PI / 180.0 * 1e-7;
const HOME_LAT_E7: i32 = 488_566_000;
const HOME_LON_E7: i32 = 23_522_000;

/// Fly `profile` and record it.
///
/// The firmware's pipeline runs on the simulated sensors exactly as on the
/// aircraft (its `attitude_data` goes into the log every 5 ms, 2 ms after the
/// sample), started from the true attitude with the true mount (identity) and
/// zero bias: the residual gyro bias is what the boot estimate would miss.
pub fn fly(cfg: &Config, profile: &[Segment]) -> Result<Flight> {
    let mut log = Log::new()?;
    let mut rng = Rng(cfg.seed.max(1));
    let mut s = State {
        heading: cfg.start_heading_deg.to_radians(),
        pitch: 0.0,
        bank: 0.0,
        pos_ned: Vector3::zeros(),
    };
    let mut pipeline = AttitudePipeline::new();
    let q0 = filter_quat(&s.rotation()).cast::<f32>();
    pipeline.set_quat(q0);
    let mut rec = Recorder::new();
    let mut truth = Vec::new();
    // GNSS: velocities waiting out their latency (report time, sample).
    let mut gnss_queue: Vec<(u64, Vector3<f64>, Vector3<f64>)> = Vec::new();
    let bias = Vector3::new(1.0, -1.0, 0.5) * cfg.gyro_bias_residual;
    let mut mag_fed: Option<Vector3<f32>> = None;
    let roll_rate = ROLL_RATE_DPS.to_radians();

    let mut i: u32 = 0;
    for seg in profile {
        let secs = match *seg {
            Segment::Straight { secs }
            | Segment::Turn { secs, .. }
            | Segment::PullUp { secs, .. } => secs,
        };
        let n = (secs / DT).round() as u32;
        for k in 0..n {
            let t_seg = f64::from(k) * DT;
            // Commanded rates for this segment.
            let (bank_rate, pitch_rate) = match *seg {
                Segment::Straight { .. } => ((-s.bank / DT).clamp(-roll_rate, roll_rate), 0.0),
                Segment::Turn { bank_deg, .. } => {
                    let roll_out_at = secs - bank_deg.to_radians().abs() / roll_rate;
                    let target = if t_seg < roll_out_at {
                        bank_deg.to_radians()
                    } else {
                        0.0
                    };
                    (((target - s.bank) / DT).clamp(-roll_rate, roll_rate), 0.0)
                }
                Segment::PullUp { pitch_deg, .. } => {
                    // Pitch follows (1 − cos) up and back down: rate is a sine.
                    let w = 2.0 * PI / secs;
                    (0.0, pitch_deg.to_radians() / 2.0 * w * (w * t_seg).sin())
                }
            };
            // Coordinated turn: heading rate from bank at the airspeed.
            let heading_rate = G * s.bank.tan() / cfg.airspeed_ms;
            let w_frd = body_rates(&s, bank_rate, pitch_rate, heading_rate);
            let r = s.rotation();

            // Specific force: kinematic acceleration minus gravity, in FRD.
            // Airspeed is constant along the nose and the wind is steady, so
            // the ground acceleration is ω × (V, 0, 0) rotated to NED.
            let v_body = Vector3::new(cfg.airspeed_ms, 0.0, 0.0);
            let a_ned = r * w_frd.cross(&v_body);
            let f_frd = r.inverse() * (a_ned - Vector3::new(0.0, 0.0, G));
            let v_ned = r * v_body + cfg.wind_ned;

            let t = f64::from(i) * DT;
            let t_us = 100_000 + u64::from(i) * 1000;

            // Sensors in the filter frame, with errors, quantised like the IMU.
            let vib = cfg.vibration * (2.0 * PI * cfg.vibration_hz * t).sin();
            let gyro = FRD_TO_FILTER * w_frd
                + bias
                + Vector3::from_fn(|_, _| cfg.gyro_noise * rng.gauss());
            let accel = FRD_TO_FILTER * f_frd
                + Vector3::from_fn(|_, _| cfg.accel_noise * rng.gauss() + vib);
            let q = |v: f64, sc: f32| decode(encode(v as f32, sc), sc);
            let g3 = (
                q(gyro.x, GYRO_SCALE),
                q(gyro.y, GYRO_SCALE),
                q(gyro.z, GYRO_SCALE),
            );
            let a3 = (
                q(accel.x, ACCEL_SCALE),
                q(accel.y, ACCEL_SCALE),
                q(accel.z, ACCEL_SCALE),
            );

            // Mag at 10 Hz, held between readings (as MAG_FIELD is).
            if i.is_multiple_of(100) {
                let m = FRD_TO_FILTER * (r.inverse() * cfg.mag_ned);
                mag_fed = Some(pipeline.mag_to_airframe(m.cast::<f32>()));
            }

            // The firmware path: fuse, then record.
            let q_before = pipeline.quat();
            let att = pipeline.fuse(
                pipeline.debias(Vector3::new(g3.0, g3.1, g3.2)),
                Vector3::new(a3.0, a3.1, a3.2),
                mag_fed.as_ref(),
            );
            let mut records = Vec::new();
            rec.sample(
                t_us,
                g3,
                a3,
                30.0,
                mag_fed.as_ref(),
                &q_before,
                &pipeline.gyro_bias,
                &pipeline.mount,
                |x| records.push(x),
            );
            records.into_iter().for_each(|x| log.record(x));
            if i.is_multiple_of(5) {
                let _ = log.w.write_attitude(
                    log.att,
                    &AttitudeMessage::new(
                        Instant::from_micros(t_us + 2000),
                        att.pitch,
                        att.roll,
                        att.yaw,
                        att.pitch_rate,
                        att.roll_rate,
                        att.yaw_rate,
                    ),
                );
                log.flush();
            }

            // GNSS: sample every 200 ms, report after the latency.
            if i.is_multiple_of(200) {
                let noisy = v_ned + Vector3::from_fn(|_, _| cfg.gnss_vel_noise * rng.gauss());
                let report_us = t_us + (cfg.gnss_latency_s * 1e6) as u64;
                gnss_queue.push((report_us, s.pos_ned, noisy));
            }
            while let Some(&(at, pos, v)) = gnss_queue.first()
                && at <= t_us
            {
                gnss_queue.remove(0);
                let gs = v.xy().norm();
                let _ = log.w.write_gnss(
                    log.gnss,
                    &GnssMessage::new(
                        Instant::from_micros(at),
                        HOME_LAT_E7 + (pos.x / M_PER_E7) as i32,
                        HOME_LON_E7
                            + (pos.y
                                / (M_PER_E7 * (f64::from(HOME_LAT_E7) * 1e-7).to_radians().cos()))
                                as i32,
                        (100.0 - pos.z) as f32,
                        1,
                        12,
                        99.9,
                        v.x as f32,
                        v.y as f32,
                        v.z as f32,
                        gs as f32,
                        v.y.atan2(v.x).to_degrees().rem_euclid(360.0) as f32,
                        1.5,
                        2.5,
                        0.2,
                        true,
                    ),
                );
                log.flush();
            }

            let (pitch, roll, yaw) = firmware_angles(&filter_quat(&r));
            truth.push(Truth {
                index: i,
                t_us,
                pitch,
                roll,
                yaw,
            });

            // Advance the airframe.
            s.bank += bank_rate * DT;
            s.pitch += pitch_rate * DT;
            s.heading += heading_rate * DT;
            s.pos_ned += v_ned * DT;
            i += 1;
        }
    }
    Ok(Flight {
        ulog: log.out,
        truth,
    })
}

/// A 3-minute test flight: straight legs and turns both ways at 15°, 30° and
/// 45°, a pull-up, and two 360° circles each way at 30°.
#[must_use]
pub fn standard_profile() -> Vec<Segment> {
    use Segment::{PullUp, Straight, Turn};
    // One 360° circle at 30° bank and 15 m/s takes 2πV/(g tan 30°) ≈ 16.6 s.
    let circle = 2.0 * PI * 15.0 / (G * 30f64.to_radians().tan());
    let mut p = vec![Straight { secs: 10.0 }];
    for bank in [15.0, 30.0, 45.0] {
        p.push(Turn {
            bank_deg: bank,
            secs: 15.0,
        });
        p.push(Straight { secs: 8.0 });
        p.push(Turn {
            bank_deg: -bank,
            secs: 15.0,
        });
        p.push(Straight { secs: 8.0 });
    }
    p.push(PullUp {
        pitch_deg: 20.0,
        secs: 4.0,
    });
    p.push(Straight { secs: 8.0 });
    p.push(Turn {
        bank_deg: 30.0,
        secs: 2.0 * circle + 1.4,
    });
    p.push(Straight { secs: 8.0 });
    p.push(Turn {
        bank_deg: -30.0,
        secs: 2.0 * circle + 1.4,
    });
    p.push(Straight { secs: 10.0 });
    p
}
