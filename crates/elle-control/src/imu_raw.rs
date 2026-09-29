//! Raw IMU capture for exact log replay (`imu-raw-log` builds).
//!
//! The ICM-42686 reports 20-bit integers that its driver scales to f32
//! (`integer as f32 * scale`). Logging the integers, not the floats, costs 3
//! bytes per axis and loses nothing: [`decode`] reproduces the driver's f32
//! bit for bit. Together with the AHRS quaternion, gyro bias and mount
//! ([`Ctx`]) and the mag vector as fed ([`Mag`]), a host replay of
//! [`crate::attitude::AttitudePipeline`] reproduces the firmware's attitude
//! angles exactly from any [`Ctx`] on. The rate low-pass history is not
//! captured: replayed rates converge to the firmware's within tens of ms.
//!
//! Every fused sample gets an index; a record says from which sample it
//! applies, so gaps (a full queue, a ULog dropout) are visible and the replay
//! can resynchronise at the next [`Ctx`].

use nalgebra::{UnitQuaternion, Vector3};

/// Full-scale integer magnitude of a 20-bit FIFO sample (2¹⁹).
const FULL_1SIDE_RANGE: f32 = (1 << 19) as f32;
/// Gyro scale, rad/s per LSB: the driver's own expression (icm426xx
/// `sample_from_packet4`, ICM-42686 at ±4000 dps), evaluated the same way.
pub const GYRO_SCALE: f32 =
    core::f32::consts::PI / 180.0 * elle_config::IMU_GYRO_FULL_SCALE_DPS / FULL_1SIDE_RANGE;
/// Accel scale, m/s² per LSB (ICM-42686 at ±32 g), as the driver computes it.
pub const ACCEL_SCALE: f32 = 9.806_65 * elle_config::IMU_ACCEL_FULL_SCALE_G / FULL_1SIDE_RANGE;

/// Samples per [`Batch`]: 100 batches a second at 1 kHz.
pub const BATCH_SAMPLES: usize = 10;
/// Bytes per sample: gyro x/y/z then accel x/y/z, 24-bit little-endian each.
pub const SAMPLE_BYTES: usize = 18;
/// A [`Ctx`] at least this often (samples), so a replay can start or resume
/// within a second of any point in a log.
pub const CTX_INTERVAL: u32 = 1000;

/// The integer the driver scaled into `value`.
#[must_use]
pub fn encode(value: f32, scale: f32) -> i32 {
    libm::roundf(value / scale) as i32
}

/// The driver's f32 for integer `raw`: `raw as f32 * scale`, exactly.
#[must_use]
pub fn decode(raw: i32, scale: f32) -> f32 {
    raw as f32 * scale
}

fn put24(out: &mut [u8], v: i32) {
    out[..3].copy_from_slice(&v.to_le_bytes()[..3]);
}

fn get24(b: &[u8]) -> i32 {
    // Sign-extend from bit 23.
    i32::from_le_bytes([b[0], b[1], b[2], 0]) << 8 >> 8
}

/// Pack one sample's raw integers (gyro, then accel).
pub fn pack(out: &mut [u8], gyro: [i32; 3], accel: [i32; 3]) {
    for (i, v) in gyro.iter().chain(accel.iter()).enumerate() {
        put24(&mut out[3 * i..], *v);
    }
}

/// Unpack one sample: (gyro, accel) raw integers.
#[must_use]
pub fn unpack(b: &[u8]) -> ([i32; 3], [i32; 3]) {
    let v = |i: usize| get24(&b[3 * i..]);
    ([v(0), v(1), v(2)], [v(3), v(4), v(5)])
}

/// Up to [`BATCH_SAMPLES`] consecutive samples.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Batch {
    /// Index of the first sample.
    pub first_index: u32,
    /// When the first sample was read, µs since boot.
    pub t_us: u64,
    /// Samples in `data` (short only when flushed early).
    pub count: u8,
    /// IMU temperature at the last sample, °C × 100.
    pub temp_centi_c: i16,
    pub data: [u8; BATCH_SAMPLES * SAMPLE_BYTES],
}

/// The mag vector fed to the AHRS changed (airframe frame, as fed; `None` =
/// 6-DOF from here on).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Mag {
    /// First sample it applies to.
    pub index: u32,
    pub t_us: u64,
    pub mag: Option<[f32; 3]>,
}

/// Pipeline state entering sample `index`, and the bias and mount it was
/// fused with. Written every [`CTX_INTERVAL`] samples and whenever the bias or
/// mount changes.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Ctx {
    pub index: u32,
    pub t_us: u64,
    /// AHRS quaternion before sample `index` (w, x, y, z).
    pub quat: [f32; 4],
    pub gyro_bias: [f32; 3],
    /// Level-cal mount (w, x, y, z).
    pub mount: [f32; 4],
    /// Samples whose floats did not survive encode → decode unchanged. Nonzero
    /// means the scale here disagrees with the driver's; the replay is not exact.
    pub roundtrip_errors: u32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Record {
    Batch(Batch),
    Mag(Mag),
    Ctx(Ctx),
}

/// Counts samples and turns them into [`Record`]s.
#[derive(Clone, Copy, Debug)]
pub struct Recorder {
    index: u32,
    batch: Batch,
    last_mag: Option<Option<[f32; 3]>>,
    last_bias: Option<[f32; 3]>,
    last_mount: Option<[f32; 4]>,
    roundtrip_errors: u32,
}

impl Default for Recorder {
    fn default() -> Self {
        Self::new()
    }
}

fn quat4(q: &UnitQuaternion<f32>) -> [f32; 4] {
    [q.w, q.i, q.j, q.k]
}

/// The raw integers for one vector, and whether they reproduce it exactly.
fn encode3(v: (f32, f32, f32), scale: f32) -> ([i32; 3], bool) {
    let raw = [v.0, v.1, v.2].map(|x| encode(x, scale));
    let exact = [v.0, v.1, v.2]
        .iter()
        .zip(raw)
        .all(|(x, r)| decode(r, scale).to_bits() == x.to_bits());
    (raw, exact)
}

impl Recorder {
    #[must_use]
    pub const fn new() -> Self {
        Self {
            index: 0,
            batch: Batch {
                first_index: 0,
                t_us: 0,
                count: 0,
                temp_centi_c: 0,
                data: [0; BATCH_SAMPLES * SAMPLE_BYTES],
            },
            last_mag: None,
            last_bias: None,
            last_mount: None,
            roundtrip_errors: 0,
        }
    }

    /// Record one sample, after it was fused.
    ///
    /// `gyro`/`accel`: the driver's floats for this sample (before bias and
    /// mount, zeros where the FIFO packet had none); `mag`: the vector fed to
    /// the AHRS; `quat_before`: the AHRS state before this sample; `bias` and
    /// `mount`: what this sample was fused with. Records go to `emit` in the
    /// order a replay needs them (mag and context before the batch holding the
    /// sample).
    #[allow(clippy::too_many_arguments)]
    pub fn sample(
        &mut self,
        t_us: u64,
        gyro: (f32, f32, f32),
        accel: (f32, f32, f32),
        temp_c: f32,
        mag: Option<&Vector3<f32>>,
        quat_before: &UnitQuaternion<f32>,
        bias: &Vector3<f32>,
        mount: &UnitQuaternion<f32>,
        mut emit: impl FnMut(Record),
    ) {
        let index = self.index;
        self.index = self.index.wrapping_add(1);

        let mag = mag.map(|m| [m.x, m.y, m.z]);
        if self.last_mag != Some(mag) {
            self.last_mag = Some(mag);
            emit(Record::Mag(Mag { index, t_us, mag }));
        }

        let (gyro, gyro_exact) = encode3(gyro, GYRO_SCALE);
        let (accel, accel_exact) = encode3(accel, ACCEL_SCALE);
        if !(gyro_exact && accel_exact) {
            self.roundtrip_errors = self.roundtrip_errors.saturating_add(1);
        }

        let bias = [bias.x, bias.y, bias.z];
        let mount = quat4(mount);
        if index.is_multiple_of(CTX_INTERVAL)
            || self.last_bias != Some(bias)
            || self.last_mount != Some(mount)
        {
            self.last_bias = Some(bias);
            self.last_mount = Some(mount);
            emit(Record::Ctx(Ctx {
                index,
                t_us,
                quat: quat4(quat_before),
                gyro_bias: bias,
                mount,
                roundtrip_errors: self.roundtrip_errors,
            }));
        }

        let n = usize::from(self.batch.count);
        if n == 0 {
            self.batch.first_index = index;
            self.batch.t_us = t_us;
        }
        pack(&mut self.batch.data[n * SAMPLE_BYTES..], gyro, accel);
        self.batch.count += 1;
        self.batch.temp_centi_c = (temp_c * 100.0) as i16;
        if usize::from(self.batch.count) == BATCH_SAMPLES {
            emit(Record::Batch(self.batch));
            self.batch.count = 0;
        }
    }
}
