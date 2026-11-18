//! PID gains over the real `HostClient` path, against a fake flight controller
//! (postcard-rpc's `local_setup`): the f32 endpoint, the fixed-point fallback
//! for older firmware, and the firmware's refusal.
//!
//!   cargo test -p elle-rpc-host --target x86_64-unknown-linux-gnu --test pid

use std::sync::{Arc, Mutex};
use std::time::Duration;

use elle_rpc_host::pid::{PidGains, Sent, send_gains};
use elle_rpc_icd::*;
use postcard_rpc::header::VarKey;
use postcard_rpc::standard_icd::{ERROR_KEY, WireError};
use postcard_rpc::test_utils::{LocalFakeServer, local_setup};
use postcard_rpc::{Endpoint, Key, Topic};

/// The error path, as the firmware's dispatcher answers an unknown key.
struct ErrorTopic;
impl Topic for ErrorTopic {
    type Message = WireError;
    const PATH: &'static str = "error";
    const TOPIC_KEY: Key = ERROR_KEY;
}

/// What the fake does with the f32 endpoint.
#[derive(Clone, Copy)]
enum Firmware {
    /// Knows it; acks (or refuses with `PID_ERR_INVALID` when `refuse`).
    Current { refuse: bool },
    /// Predates it: answers `UnknownKey`, as postcard-rpc's dispatcher does.
    Old,
    /// Never answers the f32 request (a lost link).
    Silent,
}

#[derive(Default)]
struct Seen {
    f32_req: Option<SetPidGainsF32Req>,
    fixed_req: Option<SetPidGainsReq>,
}

fn key_is<E: Endpoint>(k: &VarKey) -> bool {
    *k == VarKey::Key8(E::REQ_KEY)
}

async fn fake(mut srv: LocalFakeServer, fw: Firmware, seen: Arc<Mutex<Seen>>) {
    while let Ok(frame) = srv.recv_from_client().await {
        let seq: u32 = frame.header.seq_no.into();
        let k = frame.header.key;
        if key_is::<SetPidGainsF32Endpoint>(&k) {
            seen.lock().unwrap().f32_req = postcard::from_bytes(&frame.body).ok();
            match fw {
                Firmware::Current { refuse } => {
                    let ack = if refuse {
                        AckResp::error(PID_ERR_INVALID)
                    } else {
                        AckResp::ok()
                    };
                    let _ = srv.reply::<SetPidGainsF32Endpoint>(seq, &ack).await;
                }
                Firmware::Old => {
                    let _ = srv.publish::<ErrorTopic>(seq, &WireError::UnknownKey).await;
                }
                Firmware::Silent => {}
            }
        } else if key_is::<SetPidGainsEndpoint>(&k) {
            seen.lock().unwrap().fixed_req = postcard::from_bytes(&frame.body).ok();
            let _ = srv.reply::<SetPidGainsEndpoint>(seq, &AckResp::ok()).await;
        }
    }
}

fn gains(scale: f32) -> PidGains {
    PidGains {
        pitch_kp: 0.35,
        pitch_ki: 0.02,
        pitch_kd: 0.02,
        roll_kp: 0.25,
        roll_ki: 0.012,
        roll_kd: 0.07,
        scale,
        i_limit: 0.5,
    }
}

async fn run(fw: Firmware, g: PidGains) -> (Result<Sent, String>, Arc<Mutex<Seen>>) {
    let (srv, host) = local_setup::<WireError>(64, "error");
    let seen = Arc::new(Mutex::new(Seen::default()));
    tokio::spawn(fake(srv, fw, seen.clone()));
    let r = send_gains(&host, &g, Duration::from_millis(300)).await;
    (r, seen)
}

#[tokio::test]
async fn operating_scale_goes_through_the_f32_endpoint() {
    let (r, seen) = run(Firmware::Current { refuse: false }, gains(5.0)).await;
    assert_eq!(r, Ok(Sent::F32));
    let seen = seen.lock().unwrap();
    assert_eq!(seen.f32_req.map(|q| q.scale), Some(5.0));
    assert!(seen.fixed_req.is_none());
}

#[tokio::test]
async fn older_firmware_falls_back_to_fixed_point() {
    let (r, seen) = run(Firmware::Old, gains(2.5)).await;
    assert_eq!(r, Ok(Sent::Fixed));
    assert_eq!(
        seen.lock().unwrap().fixed_req.map(|q| q.scale_x10000),
        Some(25_000)
    );
}

#[tokio::test]
async fn older_firmware_still_refuses_what_fixed_point_cannot_hold() {
    let (r, seen) = run(Firmware::Old, gains(5.0)).await;
    assert!(r.unwrap_err().starts_with("scale 5"));
    assert!(seen.lock().unwrap().fixed_req.is_none());
}

#[tokio::test]
async fn a_timeout_never_falls_back() {
    let (r, seen) = run(Firmware::Silent, gains(2.5)).await;
    assert_eq!(r, Err("timeout".into()));
    assert!(seen.lock().unwrap().fixed_req.is_none());
}

#[tokio::test]
async fn firmware_refusal_is_reported() {
    let (r, _) = run(Firmware::Current { refuse: true }, gains(2.5)).await;
    assert!(r.unwrap_err().contains("out of range"));
}
