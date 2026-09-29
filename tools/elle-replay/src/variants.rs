//! Alternative attitude filters, run on the same inputs as the firmware's.
//!
//! Each variant gets exactly what the firmware pipeline fused: the gyro with
//! the logged bias removed and the accel, both rotated by the logged mount,
//! and the mag vector as fed. It is seeded with the firmware's quaternion when
//! the replay (re)synchronises and then runs on its own, so its difference to
//! the exact replay is the effect of the filter alone.

use core::time::Duration;

use nalgebra::{UnitQuaternion, Vector3};
use uf_ahrs::{Ahrs, Madgwick, MadgwickParams, Mahony, MahonyParams, Vqf, VqfParams};

/// Standard gravity, m/s².
const G: f32 = 9.806_65;
/// IMU sample period.
const DT: Duration = Duration::from_micros(elle_config::AHRS_SAMPLE_PERIOD_US);
/// VQF's mag rate: new mag readings arrive at ~10 Hz.
const MAG_PERIOD: Duration = Duration::from_millis(100);

/// Which filter.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Kind {
    Madgwick { beta: f32 },
    Mahony,
    Vqf,
}

/// A filter and how its accelerometer input is gated.
#[derive(Clone, Debug, PartialEq)]
pub struct Spec {
    pub name: String,
    pub kind: Kind,
    /// Skip the accelerometer (gyro-only update, plus mag for VQF) while the
    /// low-passed accel magnitude is further than this from 1 g, in g.
    pub gate_g: Option<f32>,
}

/// Corner of the low-pass on the accel magnitude the gate looks at, Hz: slow
/// enough that motor vibration (100+ Hz) does not trip it.
pub const GATE_LPF_HZ: f32 = 5.0;
/// Default gate threshold, g.
pub const DEFAULT_GATE_G: f32 = 0.25;

/// The comparison set: the firmware's filter as a variant (must equal the exact
/// replay: a check on the plumbing), then each filter with and without gating.
#[must_use]
pub fn default_specs() -> Vec<Spec> {
    let fw = Kind::Madgwick {
        beta: elle_config::AHRS_BETA,
    };
    let spec = |name: &str, kind, gate_g| Spec {
        name: name.to_string(),
        kind,
        gate_g,
    };
    vec![
        spec("madgwick", fw, None),
        spec("madgwick-gated", fw, Some(DEFAULT_GATE_G)),
        spec("mahony", Kind::Mahony, None),
        spec("mahony-gated", Kind::Mahony, Some(DEFAULT_GATE_G)),
        spec("vqf", Kind::Vqf, None),
        spec("vqf-gated", Kind::Vqf, Some(DEFAULT_GATE_G)),
    ]
}

enum Filter {
    Madgwick(Madgwick),
    Mahony(Mahony),
    Vqf(Box<Vqf>),
}

impl Filter {
    fn new(kind: Kind) -> Self {
        match kind {
            Kind::Madgwick { beta } => Self::Madgwick(Madgwick::new(DT, MadgwickParams { beta })),
            Kind::Mahony => Self::Mahony(Mahony::new(DT, MahonyParams::default())),
            Kind::Vqf => Self::Vqf(Box::new(Vqf::new_with_sensor_rates(
                DT,
                DT,
                MAG_PERIOD,
                VqfParams::default(),
            ))),
        }
    }
}

/// One variant running alongside the replay.
pub struct Running {
    pub spec: Spec,
    filter: Filter,
    /// Low-passed accel magnitude, m/s² (`None` until the first sample).
    accel_norm_lp: Option<f32>,
    lp_alpha: f32,
    /// Samples fused without the accelerometer.
    pub gated: u64,
}

impl Running {
    #[must_use]
    pub fn new(spec: Spec) -> Self {
        let dt = DT.as_secs_f32();
        let rc = 1.0 / (2.0 * core::f32::consts::PI * GATE_LPF_HZ);
        Self {
            filter: Filter::new(spec.kind),
            spec,
            accel_norm_lp: None,
            lp_alpha: dt / (rc + dt),
            gated: 0,
        }
    }

    /// Restart from the firmware's state (at a (re)synchronisation).
    pub fn seed(&mut self, q: UnitQuaternion<f32>) {
        self.filter = Filter::new(self.spec.kind);
        match &mut self.filter {
            Filter::Madgwick(f) => f.set_orientation(q),
            Filter::Mahony(f) => f.set_orientation(q),
            Filter::Vqf(f) => f.set_orientation(q),
        }
        self.accel_norm_lp = None;
    }

    /// One sample in the airframe frame (gyro debiased and mounted, accel
    /// mounted, mag as fed; `mag_new` when it changed with this sample).
    /// Returns (pitch, roll, yaw) in the firmware's convention (roll negated).
    pub fn step(
        &mut self,
        gyro: Vector3<f32>,
        accel: Vector3<f32>,
        mag: Option<&Vector3<f32>>,
        mag_new: bool,
    ) -> [f32; 3] {
        let n = accel.norm();
        let lp = match self.accel_norm_lp {
            Some(prev) => prev + self.lp_alpha * (n - prev),
            None => n,
        };
        self.accel_norm_lp = Some(lp);
        let gate = self.spec.gate_g.is_some_and(|g| (lp - G).abs() > g * G);
        self.gated += u64::from(gate);

        let q = match &mut self.filter {
            Filter::Madgwick(f) => step_basic(f, gyro, accel, mag, gate),
            Filter::Mahony(f) => step_basic(f, gyro, accel, mag, gate),
            Filter::Vqf(f) => {
                f.gyroscope_update(gyro);
                if !gate {
                    f.accelerometer_update(accel);
                }
                if let Some(m) = mag
                    && mag_new
                {
                    f.magnetometer_update(*m);
                }
                f.orientation()
            }
        };
        let (roll, pitch, yaw) = q.euler_angles();
        [pitch, -roll, yaw]
    }
}

fn step_basic(
    f: &mut impl Ahrs,
    gyro: Vector3<f32>,
    accel: Vector3<f32>,
    mag: Option<&Vector3<f32>>,
    gate: bool,
) -> UnitQuaternion<f32> {
    match (gate, mag) {
        (true, _) => f.update_gyro(gyro),
        (false, Some(m)) => f.update(gyro, accel, *m),
        (false, None) => f.update_imu(gyro, accel),
    }
}
