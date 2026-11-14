//! Alternative attitude filters, run on the same inputs as the firmware's.
//!
//! Each variant gets exactly what the firmware pipeline fused: the gyro with
//! the logged bias removed and the accel, both rotated by the logged mount,
//! and the mag vector as fed. It is seeded with the firmware's quaternion when
//! the replay (re)synchronises and then runs on its own, so its difference to
//! the exact replay is the effect of the filter alone. Turn compensation and
//! the accel gate are the firmware's own (`elle_control::attitude`).

use core::time::Duration;

use elle_control::attitude::{AccelGate, Aid, TurnComp};
use nalgebra::{UnitQuaternion, Vector3};
use uf_ahrs::{Ahrs, Madgwick, MadgwickParams, Mahony, MahonyParams, Vqf, VqfParams};

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

/// How the accel is corrected for the aircraft's own acceleration.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Turn {
    /// Not at all (the firmware today).
    Off,
    /// `TurnComp`: ω × (GNSS ground speed along the nose). Wind-sensitive.
    Centripetal,
    /// The GNSS velocity change between fixes, rotated into the body with the
    /// variant's own attitude. Wind-proof, but 5 Hz and delayed.
    Earth,
}

/// A filter, its turn compensation and how its accelerometer input is gated.
#[derive(Clone, Debug, PartialEq)]
pub struct Spec {
    pub name: String,
    pub kind: Kind,
    pub turn: Turn,
    /// Skip the accelerometer (gyro-only update, plus mag for VQF) while the
    /// low-passed accel magnitude is further than this from 1 g, in g.
    pub gate_g: Option<f32>,
    /// With a gate on a Madgwick filter: scale β down smoothly instead of
    /// skipping the accel.
    pub smooth: bool,
    /// What-if mag axis alignment (proposal 0010): turn the fed mag's chip axes
    /// by this much more than the log did, degrees (a quarter turn; 0 = as fed).
    pub mag_axes_deg: f32,
}

/// Default gate threshold, g.
pub const DEFAULT_GATE_G: f32 = 0.25;

/// The comparison set. The first is the firmware's filter as a variant (must
/// equal the exact replay: a check on the plumbing).
#[must_use]
pub fn default_specs() -> Vec<Spec> {
    let fw = Kind::Madgwick {
        beta: elle_config::AHRS_BETA,
    };
    let g = Some(DEFAULT_GATE_G);
    let spec = |name: &str, kind, turn, gate_g| Spec {
        name: name.to_string(),
        kind,
        turn,
        gate_g,
        smooth: false,
        mag_axes_deg: 0.0,
    };
    let weighted = Spec {
        smooth: true,
        ..spec("madgwick-weighted", fw, Turn::Off, g)
    };
    vec![
        spec("madgwick", fw, Turn::Off, None),
        spec("madgwick-gated", fw, Turn::Off, g),
        weighted,
        spec("madgwick-cc", fw, Turn::Centripetal, None),
        spec("madgwick-cc-gated", fw, Turn::Centripetal, g),
        spec("madgwick-ce", fw, Turn::Earth, None),
        spec("mahony", Kind::Mahony, Turn::Off, None),
        spec("mahony-cc", Kind::Mahony, Turn::Centripetal, None),
        spec("vqf", Kind::Vqf, Turn::Off, None),
        spec("vqf-gated", Kind::Vqf, Turn::Off, g),
        spec("vqf-cc", Kind::Vqf, Turn::Centripetal, None),
        spec("vqf-ce", Kind::Vqf, Turn::Earth, None),
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

    fn orientation(&self) -> UnitQuaternion<f32> {
        match self {
            Self::Madgwick(f) => f.orientation(),
            Self::Mahony(f) => f.orientation(),
            Self::Vqf(f) => f.orientation(),
        }
    }
}

/// The firmware's filter with the fed mag's axes turned by `deg` more than the
/// log did (`--mag-axes`).
#[must_use]
pub fn mag_axes_spec(deg: f32) -> Spec {
    Spec {
        name: format!("madgwick-mag{deg:+}"),
        kind: Kind::Madgwick {
            beta: elle_config::AHRS_BETA,
        },
        turn: Turn::Off,
        gate_g: None,
        smooth: false,
        mag_axes_deg: deg,
    }
}

/// The fed mag (airframe frame, after `mount`) with the chip axes turned by
/// `deg` more: un-mount, swap, mount again.
#[must_use]
pub fn remap_fed_mag(fed: &Vector3<f32>, mount: &UnitQuaternion<f32>, deg: f32) -> Vector3<f32> {
    mount * elle_control::attitude::mag_axes(mount.inverse() * fed, deg)
}

/// `q` turned about the earth vertical so that `new_mag` lies where `old_mag`
/// did: the same tilt, the heading the remapped field implies. Seeding a
/// what-if with the firmware's quaternion unchanged would start it at the old
/// heading and measure its convergence, not the mapping.
#[must_use]
pub fn reseat_heading(
    q: UnitQuaternion<f32>,
    old_mag: &Vector3<f32>,
    new_mag: &Vector3<f32>,
) -> UnitQuaternion<f32> {
    let (o, n) = (q * old_mag, q * new_mag);
    let d = o.y.atan2(o.x) - n.y.atan2(n.x);
    UnitQuaternion::from_axis_angle(&Vector3::z_axis(), d) * q
}

/// One variant running alongside the replay.
pub struct Running {
    pub spec: Spec,
    filter: Filter,
    gate: AccelGate,
    turn: TurnComp,
    /// Fade for the earth-frame compensation (same ramp as `TurnComp`).
    earth_weight: TurnComp,
    /// Samples fused without the accelerometer.
    pub gated: u64,
}

impl Running {
    #[must_use]
    pub fn new(spec: Spec) -> Self {
        Self {
            filter: Filter::new(spec.kind),
            gate: AccelGate::new(spec.gate_g).smooth(spec.smooth),
            turn: TurnComp::new(),
            earth_weight: TurnComp::new(),
            spec,
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
        self.gate = AccelGate::new(self.spec.gate_g).smooth(self.spec.smooth);
        self.turn = TurnComp::new();
        self.earth_weight = TurnComp::new();
    }

    /// One sample in the filter frame (gyro debiased and mounted, accel
    /// mounted, mag as fed; `mag_new` when it changed with this sample).
    /// Returns (pitch, roll, yaw) in the firmware's convention (roll negated).
    pub fn step(
        &mut self,
        gyro: Vector3<f32>,
        accel: Vector3<f32>,
        mag: Option<&Vector3<f32>>,
        mag_new: bool,
        aid: &Aid,
    ) -> [f32; 3] {
        let accel = match self.spec.turn {
            Turn::Off => accel,
            Turn::Centripetal => {
                self.turn.update(aid.speed);
                self.turn.correct(&gyro, &accel)
            }
            Turn::Earth => {
                // Reuse TurnComp's fade as a 0..1 weight (speed is unused).
                self.earth_weight.update(aid.accel_earth.map(|_| 1.0));
                match aid.accel_earth {
                    Some(a) => {
                        let body = self.filter.orientation().inverse() * a;
                        accel - body * self.earth_weight.weight()
                    }
                    None => accel,
                }
            }
        };
        let w = self.gate.weight(&accel);
        let gate = w <= 0.0;
        self.gated += u64::from(gate);
        if self.spec.smooth
            && !gate
            && let (Filter::Madgwick(f), Kind::Madgwick { beta }) =
                (&mut self.filter, self.spec.kind)
        {
            f.set_params(MadgwickParams { beta: beta * w });
        }

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
