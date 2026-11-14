//! Proposal 0010: does the mag axis alignment help where the answer is known?
//! `cargo run --release -p elle-replay --example mag_axes_still -- [--mag-axes DEG] LOG.ulg...`
//!
//! At every still rest (gyro below 2°/s for 2 s by default, sampled at its end) the accel
//! gives the true tilt. For the firmware filter and the what-if with the fed mag
//! turned by DEG (default 90), prints the tilt error against the accel, and the
//! dip (angle between the field and the horizontal) the two mappings give: with
//! the right mapping the dip is the same at every rest, whatever the attitude.

use anyhow::{Context, Result};
use elle_control::attitude::mag_axes;
use elle_replay::{replay_with, ulog::ULog, variants};
use nalgebra::{UnitQuaternion, Vector3};

/// Still: gyro below this (°/s) for `STILL_S`; env `STILL_DPS` / `STILL_S`
/// override (hand-held poses rarely stay under 2°/s).
fn still_params() -> (f32, u64) {
    let env = |k: &str, d: f64| {
        std::env::var(k)
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(d)
    };
    (
        env("STILL_DPS", 2.0) as f32,
        (env("STILL_S", 2.0) * 1e6) as u64,
    )
}

fn up_error_deg(angles: [f32; 3], accel: &Vector3<f32>) -> f32 {
    let [pitch, roll, yaw] = angles;
    let q = UnitQuaternion::from_euler_angles(-roll, pitch, yaw);
    let up = q.inverse() * Vector3::z();
    up.angle(&accel.normalize()).to_degrees()
}

fn dip_deg(up: &Vector3<f32>, m: &Vector3<f32>) -> f32 {
    -(up.normalize().dot(&m.normalize())).asin().to_degrees()
}

/// Mean, standard deviation and largest value.
fn stats(v: &[f32]) -> (f32, f32, f32) {
    let n = v.len().max(1) as f32;
    let mean = v.iter().sum::<f32>() / n;
    let sd = (v.iter().map(|x| (x - mean).powi(2)).sum::<f32>() / n).sqrt();
    (mean, sd, v.iter().copied().fold(0f32, f32::max))
}

fn main() -> Result<()> {
    let (still_dps, still_us) = still_params();
    let mut args: Vec<String> = std::env::args().skip(1).collect();
    let mut deg = 90.0f32;
    if let Some(i) = args.iter().position(|a| a == "--mag-axes") {
        deg = args.get(i + 1).context("--mag-axes DEG")?.parse()?;
        args.drain(i..=i + 1);
    }
    println!(
        "log        rests  |B| G   tilt err vs accel: firmware mean/max | mag{deg:+} mean/max   dip firmware mean±sd | mag{deg:+} mean±sd"
    );
    for path in args {
        let data = std::fs::read(&path).with_context(|| path.clone())?;
        let log = ULog::parse(&data)?;
        let specs = [
            variants::default_specs().remove(0),
            variants::mag_axes_spec(deg),
        ];
        let rp = replay_with(&log, &specs)?;
        let (mut e_fw, mut e_new, mut d_fw, mut d_new, mut b) =
            (vec![], vec![], vec![], vec![], vec![]);
        let mut still_since: Option<u64> = None;
        let mut taken = false;
        for (k, (s, inp)) in rp.samples.iter().zip(&rp.inputs).enumerate() {
            if inp.gyro.norm().to_degrees() > still_dps {
                still_since = None;
                taken = false;
                continue;
            }
            let since = *still_since.get_or_insert(s.t_us);
            if taken || s.t_us < since + still_us {
                continue;
            }
            let (Some(fw), Some(new), Some(m)) =
                (rp.tracks[0].angles[k], rp.tracks[1].angles[k], inp.mag)
            else {
                continue;
            };
            taken = true;
            e_fw.push(up_error_deg(fw, &inp.accel));
            e_new.push(up_error_deg(new, &inp.accel));
            d_fw.push(dip_deg(&inp.accel, &m));
            let sensor_up = inp.mount.inverse() * inp.accel;
            let sensor_m = mag_axes(inp.mount.inverse() * m, deg);
            d_new.push(dip_deg(&sensor_up, &sensor_m));
            b.push(m.norm() / 16384.0);
            if std::env::var_os("REST_TRACE").is_some() {
                let tilt = inp.accel.angle(&Vector3::z()).to_degrees();
                println!(
                    "  t {:6.1} s  |B| {:.3} G  tilt {:5.1}  dip {:6.1} → {:6.1}  tilt err {:5.2} → {:5.2}  yaw {:7.1} → {:7.1}",
                    s.t_us as f64 / 1e6,
                    m.norm() / 16384.0,
                    tilt,
                    d_fw[d_fw.len() - 1],
                    d_new[d_new.len() - 1],
                    e_fw[e_fw.len() - 1],
                    e_new[e_new.len() - 1],
                    fw[2].to_degrees(),
                    new[2].to_degrees()
                );
            }
        }
        let name = std::path::Path::new(&path)
            .file_name()
            .map_or(path.clone(), |f| f.to_string_lossy().into_owned());
        let (ef, _, efx) = stats(&e_fw);
        let (en, _, enx) = stats(&e_new);
        let bmed = {
            let mut v = b.clone();
            v.sort_by(f32::total_cmp);
            v.get(v.len() / 2).copied().unwrap_or(f32::NAN)
        };
        let (df, dfs, _) = stats(&d_fw);
        let (dn, dns, _) = stats(&d_new);
        println!(
            "{name:<12} {:4}  {bmed:5.2}   {ef:5.2}/{efx:5.2}            | {en:5.2}/{enx:5.2}        {df:6.1}±{dfs:4.1}       | {dn:6.1}±{dns:4.1}",
            e_fw.len()
        );
    }
    Ok(())
}
