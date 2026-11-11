//! The MCP server end to end, without hardware: a fake flight controller
//! answers on the real `HostClient` path (postcard-rpc's `local_setup`), and an
//! in-memory MCP client calls the tools as Claude Code would.
//!
//!   cargo test -p elle-rpc-host --target x86_64-unknown-linux-gnu --test mcp

use std::sync::{Arc, Mutex};
use std::time::Duration;

use elle_rpc_host::mcp::{ElleMcp, Options};
use elle_rpc_icd::*;
use postcard_rpc::Endpoint;
use postcard_rpc::header::VarKey;
use postcard_rpc::standard_icd::WireError;
use postcard_rpc::test_utils::{LocalFakeServer, local_setup};
use rmcp::ServiceExt;
use rmcp::model::CallToolRequestParams;
use rmcp::service::RunningService;
use serde_json::{Value, json};

/// What the fake flight controller holds, and what it saw.
#[derive(Default)]
struct Fc {
    armed: bool,
    throttle: u8,
    elevons: (i8, i8),
    pitch_cdeg: i16,
    pings: u32,
    disarms: u32,
    heading_cdeg: Option<i16>,
    /// The fake ESC's 192-byte settings, and how often it was written.
    esc_eeprom: Vec<u8>,
    esc_writes: u32,
}

fn esc_eeprom() -> Vec<u8> {
    let mut e = vec![0u8; ESC_EEPROM_SIZE];
    e[0] = 0x01;
    e[1] = 4; // layout version
    e[2] = 7;
    e[3] = 2;
    e[4] = 19;
    e
}

type Shared = Arc<Mutex<Fc>>;

fn key_is<E: Endpoint>(k: &VarKey) -> bool {
    *k == VarKey::Key8(E::REQ_KEY)
}

/// Answer requests like the firmware would, until the client goes away.
/// Events to publish come in on `events`.
async fn run_fc(
    mut srv: LocalFakeServer,
    fc: Shared,
    mut events: tokio::sync::mpsc::Receiver<(u8, u16)>,
) {
    let mut pub_seq = 0u32;
    loop {
        let frame = tokio::select! {
            f = srv.recv_from_client() => match f { Ok(f) => f, Err(_) => return },
            Some((level, code)) = events.recv() => {
                pub_seq += 1;
                let _ = srv.publish::<LogTopic>(pub_seq, &LogMsg { level, code }).await;
                continue;
            }
        };
        let seq: u32 = frame.header.seq_no.into();
        let k = frame.header.key;
        let ok = AckResp::ok();
        macro_rules! reply {
            ($e:ty, $v:expr) => {{
                let _ = srv.reply::<$e>(seq, &$v).await;
            }};
        }
        if key_is::<PingEndpoint>(&k) {
            fc.lock().unwrap().pings += 1;
            reply!(PingEndpoint, ());
        } else if key_is::<GetVersionEndpoint>(&k) {
            reply!(
                GetVersionEndpoint,
                VersionResp {
                    major: 0,
                    minor: 1,
                    patch: 0
                }
            );
        } else if key_is::<GetTimeEndpoint>(&k) {
            reply!(GetTimeEndpoint, 12_000_000u64);
        } else if key_is::<GetStatusEndpoint>(&k) {
            let armed = fc.lock().unwrap().armed;
            reply!(
                GetStatusEndpoint,
                StatusResp {
                    armed,
                    failsafe: false,
                    mode: ControlMode::Manual,
                    imu_calibrated: true,
                    imu_error_count: 0,
                    rc_age_ms: 0,
                    autotune_state: 0,
                }
            );
        } else if key_is::<GetAttitudeEndpoint>(&k) {
            let pitch_cdeg = fc.lock().unwrap().pitch_cdeg;
            reply!(
                GetAttitudeEndpoint,
                AttitudeResp {
                    pitch_cdeg,
                    roll_cdeg: -250,
                    yaw_cdeg: 9000,
                    pitch_rate_cdeg: 0,
                    roll_rate_cdeg: 0,
                    yaw_rate_cdeg: 0,
                }
            );
        } else if key_is::<GetBuildInfoEndpoint>(&k) {
            let mut platform = [0u8; 24];
            platform[..17].copy_from_slice(b"RP2350-XFly-Eagle");
            let mut git = [0u8; 24];
            git[..10].copy_from_slice(b"abc1234def");
            reply!(
                GetBuildInfoEndpoint,
                BuildInfoResp {
                    platform,
                    features: BUILD_FEATURE_RPC_RC | BUILD_FEATURE_GNSS,
                    turn_comp: 1,
                    accel_gate_g_x100: 0,
                    loop_hz: 200,
                    git,
                }
            );
        } else if key_is::<ArmEndpoint>(&k) {
            fc.lock().unwrap().armed = true;
            reply!(ArmEndpoint, ok);
        } else if key_is::<DisarmEndpoint>(&k) {
            {
                let mut f = fc.lock().unwrap();
                f.armed = false;
                f.throttle = 0;
                f.disarms += 1;
            }
            reply!(DisarmEndpoint, ok);
        } else if key_is::<EmergencyStopEndpoint>(&k) {
            {
                let mut f = fc.lock().unwrap();
                f.armed = false;
                f.throttle = 0;
            }
            reply!(EmergencyStopEndpoint, ok);
        } else if key_is::<SetThrottleEndpoint>(&k) {
            let r: SetThrottleReq = postcard::from_bytes(&frame.body).unwrap();
            fc.lock().unwrap().throttle = r.percent;
            reply!(SetThrottleEndpoint, ok);
        } else if key_is::<SetElevonsEndpoint>(&k) {
            let r: SetElevonsReq = postcard::from_bytes(&frame.body).unwrap();
            fc.lock().unwrap().elevons = (r.left, r.right);
            reply!(SetElevonsEndpoint, ok);
        } else if key_is::<EscModeEndpoint>(&k) {
            reply!(EscModeEndpoint, EscBootMode::EscConfig);
        } else if key_is::<EscConnectEndpoint>(&k) {
            let info = EscInfo {
                device_id: 0x1F06,
                pin_code: 0x06,
                flash_code: 0x1F,
                protocol_version: 1,
                layout: EscLayoutInfo {
                    address_shift: 0,
                    firmware_start: 0x1000,
                    filename_start: 0x7C00,
                    eeprom_start: 0x7C00,
                },
                max_firmware_size: 0x6C00,
            };
            reply!(EscConnectEndpoint, Ok::<_, EscError>(info));
        } else if key_is::<EscNameEndpoint>(&k) {
            let name = EscBlock::from_slice(b"AM32_TEST\0").unwrap();
            reply!(EscNameEndpoint, Ok::<_, EscError>(name));
        } else if key_is::<EscReadEepromEndpoint>(&k) {
            let e = {
                let mut f = fc.lock().unwrap();
                if f.esc_eeprom.is_empty() {
                    f.esc_eeprom = esc_eeprom();
                }
                f.esc_eeprom.clone()
            };
            reply!(
                EscReadEepromEndpoint,
                Ok::<_, EscError>(EscEeprom::from_slice(&e).unwrap())
            );
        } else if key_is::<EscReadEndpoint>(&k) {
            // The EEPROM at its base address (0x7C00 in the fake's layout),
            // 0xFF past the settings; anything else is a wrong request.
            let r: EscRawReadReq = postcard::from_bytes(&frame.body).unwrap();
            let mut e = {
                let mut f = fc.lock().unwrap();
                if f.esc_eeprom.is_empty() {
                    f.esc_eeprom = esc_eeprom();
                }
                f.esc_eeprom.clone()
            };
            e.resize(ESC_MAX_BLOCK, 0xFF);
            let len = usize::from(r.read.len);
            let resp = if r.read.address == 0x7C00 && (1..=ESC_MAX_BLOCK).contains(&len) {
                Ok(EscBlock::from_slice(&e[..len]).unwrap())
            } else {
                Err(EscError::BadAddress)
            };
            reply!(EscReadEndpoint, resp);
        } else if key_is::<EscWriteEepromEndpoint>(&k) {
            let r: EscWriteReq = postcard::from_bytes(&frame.body).unwrap();
            {
                let mut f = fc.lock().unwrap();
                f.esc_eeprom = r.data.to_vec();
                f.esc_writes += 1;
            }
            reply!(EscWriteEepromEndpoint, Ok::<(), EscError>(()));
        } else if key_is::<SetHeadingHoldEndpoint>(&k) {
            let r: SetHeadingHoldReq = postcard::from_bytes(&frame.body).unwrap();
            fc.lock().unwrap().heading_cdeg = r.enabled.then_some(r.heading_cdeg);
            reply!(SetHeadingHoldEndpoint, ok);
        }
        // Anything else goes unanswered: the tool times out, as with a
        // firmware that lacks the endpoint.
    }
}

struct Rig {
    fc: Shared,
    events: tokio::sync::mpsc::Sender<(u8, u16)>,
    client: RunningService<rmcp::RoleClient, ()>,
}

async fn rig(opts: Options) -> Rig {
    let (srv, host) = local_setup::<WireError>(64, "error");
    let fc: Shared = Arc::default();
    let (ev_tx, ev_rx) = tokio::sync::mpsc::channel(16);
    tokio::spawn(run_fc(srv, fc.clone(), ev_rx));
    let server = ElleMcp::with_client(opts, host).await.unwrap();
    let (a, b) = tokio::io::duplex(64 * 1024);
    tokio::spawn(async move {
        let s = server.serve(a).await.unwrap();
        let _ = s.waiting().await;
    });
    let client = ().serve(b).await.unwrap();
    Rig {
        fc,
        events: ev_tx,
        client,
    }
}

impl Rig {
    /// Call a tool; `Ok(text)` or `Err(text)` for a tool error.
    async fn call(&self, name: &'static str, args: Value) -> Result<String, String> {
        let mut p = CallToolRequestParams::new(name);
        if let Value::Object(m) = args {
            p = p.with_arguments(m);
        }
        let r = self.client.call_tool(p).await.expect("MCP call failed");
        let text = r
            .content
            .iter()
            .filter_map(|c| c.as_text().map(|t| t.text.clone()))
            .collect::<Vec<_>>()
            .join("\n");
        if r.is_error == Some(true) {
            Err(text)
        } else {
            Ok(text)
        }
    }

    async fn json(&self, name: &'static str, args: Value) -> Value {
        let t = self
            .call(name, args)
            .await
            .unwrap_or_else(|e| panic!("{name}: {e}"));
        serde_json::from_str(&t).unwrap_or_else(|e| panic!("{name}: {e}: {t}"))
    }
}

#[tokio::test]
async fn lists_the_tools() {
    let r = rig(Options::default()).await;
    let names: Vec<String> = r
        .client
        .list_all_tools()
        .await
        .unwrap()
        .into_iter()
        .map(|t| t.name.to_string())
        .collect();
    for t in [
        "connect",
        "disconnect",
        "link_status",
        "read",
        "sample",
        "wait_for",
        "events",
        "wait_event",
        "arm",
        "extend_armed",
        "set_throttle",
        "disarm",
        "emergency_stop",
        "set_elevons",
        "set_mode",
        "set_heading_hold",
        "mag_cal",
        "level_cal",
        "ulog",
        "autotune",
        "build_and_flash",
        "reset_target",
        "connect_defmt",
        "log",
        "wait_log",
        "esc_mode",
        "esc_connect",
        "esc_read",
        "esc_backup",
        "esc_compare",
        "esc_set",
        "esc_restore",
        "esc_run",
    ] {
        assert!(names.iter().any(|n| n == t), "missing tool {t}: {names:?}");
    }
}

#[tokio::test]
async fn reads_with_degrees_and_status() {
    let r = rig(Options::default()).await;
    let a = r.json("read", json!({"what": "attitude"})).await;
    assert_eq!(a["roll_deg"], json!(-2.5));
    assert_eq!(a["yaw_deg"], json!(90.0));
    let s = r.json("link_status", json!({})).await;
    assert_eq!(s["connected"], json!(true));
    assert_eq!(s["build"]["platform"], json!("RP2350-XFly-Eagle"));
    assert_eq!(
        s["build"]["features"],
        json!(["rpc-control", "rpc-rc", "gnss"])
    );
    assert_eq!(s["build"]["turn_comp"], json!("centripetal"));
    assert_eq!(s["build"]["accel_gate_g"], json!(null));
    assert_eq!(s["motors_allowed"], json!(false));
    assert_eq!(s["uptime_s"], json!(12.0));
}

#[tokio::test]
async fn sample_gives_statistics() {
    let r = rig(Options::default()).await;
    r.fc.lock().unwrap().pitch_cdeg = 150;
    let s = r
        .json(
            "sample",
            json!({"what": "attitude", "duration_s": 0.5, "rate_hz": 20, "fields": ["pitch_deg"]}),
        )
        .await;
    let p = &s["fields"]["pitch_deg"];
    assert!(p["n"].as_u64().unwrap() >= 5, "{s}");
    assert_eq!(
        (p["min"].as_f64(), p["max"].as_f64()),
        (Some(1.5), Some(1.5))
    );
    assert_eq!(s["fields"].as_object().unwrap().len(), 1);
}

#[tokio::test]
async fn wait_for_sees_the_operator_act() {
    let r = rig(Options::default()).await;
    let fc = r.fc.clone();
    tokio::spawn(async move {
        tokio::time::sleep(Duration::from_millis(300)).await;
        fc.lock().unwrap().pitch_cdeg = 2000; // "tilt the nose up 20°"
    });
    let w = r
        .json("wait_for", json!({"what": "attitude", "field": "pitch_deg", "op": "gt", "value": 15, "timeout_s": 3}))
        .await;
    assert_eq!(w["met"], json!(true));
    assert!(w["after_s"].as_f64().unwrap() >= 0.25, "{w}");

    let e = r
        .call("wait_for", json!({"what": "attitude", "field": "pitch_deg", "op": "lt", "value": 0, "timeout_s": 0.3}))
        .await
        .unwrap_err();
    assert!(e.contains("timed out"), "{e}");
    let e = r
        .call(
            "wait_for",
            json!({"what": "attitude", "field": "nope", "op": "lt", "value": 0, "timeout_s": 1}),
        )
        .await
        .unwrap_err();
    assert!(e.contains("no numeric field"), "{e}");
}

#[tokio::test]
async fn events_are_buffered_labelled_and_awaited() {
    let r = rig(Options::default()).await;
    r.events.send((3, 16)).await.unwrap();
    tokio::time::sleep(Duration::from_millis(100)).await;
    let ev = r.json("events", json!({})).await;
    let first = &ev["events"][0];
    assert_eq!(first["code"], json!(16));
    assert_eq!(first["label"], json!(elle_rpc_host::events::label(16)));

    // wait_event only sees events after the call by default.
    let tx = r.events.clone();
    tokio::spawn(async move {
        tokio::time::sleep(Duration::from_millis(200)).await;
        tx.send((3, 14)).await.unwrap();
    });
    let got = r
        .json("wait_event", json!({"codes": [14, 16], "timeout_s": 2}))
        .await;
    assert_eq!(got["code"], json!(14));
    let e = r
        .call("wait_event", json!({"codes": [99], "timeout_s": 0.2}))
        .await
        .unwrap_err();
    assert!(e.contains("no event"), "{e}");
}

#[tokio::test]
async fn engines_are_off_limits_without_the_flag() {
    let r = rig(Options::default()).await;
    let e = r
        .call("arm", json!({"props_off_confirmed": true}))
        .await
        .unwrap_err();
    assert!(e.contains("--dangerously-allow-motors"), "{e}");
    let e = r
        .call("set_throttle", json!({"percent": 10}))
        .await
        .unwrap_err();
    assert!(e.contains("--dangerously-allow-motors"), "{e}");
    assert!(!r.fc.lock().unwrap().armed);
    // Surfaces and stopping stay available.
    r.call("set_elevons", json!({"left": 50, "right": -50}))
        .await
        .unwrap();
    assert_eq!(r.fc.lock().unwrap().elevons, (50, -50));
    r.call("disarm", json!({})).await.unwrap();
    r.call("emergency_stop", json!({})).await.unwrap();
}

fn motors(max_armed_ms: u64) -> Options {
    Options {
        allow_motors: true,
        max_armed: Duration::from_millis(max_armed_ms),
        max_throttle_pct: 30,
        allow_esc_write: false,
    }
}

#[tokio::test]
async fn arming_needs_props_off_and_throttle_is_capped() {
    let r = rig(motors(60_000)).await;
    let e = r
        .call("set_throttle", json!({"percent": 10}))
        .await
        .unwrap_err();
    assert!(e.contains("arm through this server first"), "{e}");
    let e = r
        .call("arm", json!({"props_off_confirmed": false}))
        .await
        .unwrap_err();
    assert!(e.contains("props_off_confirmed"), "{e}");
    assert!(!r.fc.lock().unwrap().armed);

    r.call("arm", json!({"props_off_confirmed": true}))
        .await
        .unwrap();
    assert!(r.fc.lock().unwrap().armed);
    let e = r
        .call("set_throttle", json!({"percent": 31}))
        .await
        .unwrap_err();
    assert!(e.contains("30 % cap"), "{e}");
    r.call("set_throttle", json!({"percent": 20}))
        .await
        .unwrap();
    assert_eq!(r.fc.lock().unwrap().throttle, 20);
    // Keepalive while armed.
    tokio::time::sleep(Duration::from_millis(350)).await;
    assert!(r.fc.lock().unwrap().pings >= 2);

    r.call("disarm", json!({})).await.unwrap();
    let f = r.fc.lock().unwrap();
    assert!(!f.armed && f.throttle == 0);
}

#[tokio::test]
async fn the_armed_timer_disarms_by_itself() {
    let r = rig(motors(400)).await;
    r.call("arm", json!({"props_off_confirmed": true}))
        .await
        .unwrap();
    r.call("set_throttle", json!({"percent": 15}))
        .await
        .unwrap();
    tokio::time::sleep(Duration::from_millis(250)).await;
    r.call("extend_armed", json!({"seconds": 0.4}))
        .await
        .unwrap();
    tokio::time::sleep(Duration::from_millis(300)).await;
    assert!(r.fc.lock().unwrap().armed, "renewed: still armed at 550 ms");
    tokio::time::sleep(Duration::from_millis(400)).await;
    {
        let f = r.fc.lock().unwrap();
        assert!(!f.armed && f.throttle == 0, "timer should have disarmed");
    }
    let s = r.json("link_status", json!({})).await;
    assert_eq!(s["armed_by_server"], json!(false));
}

#[tokio::test]
async fn arming_again_replaces_the_timer() {
    let r = rig(motors(400)).await;
    r.call("arm", json!({"props_off_confirmed": true}))
        .await
        .unwrap();
    tokio::time::sleep(Duration::from_millis(250)).await;
    r.call("arm", json!({"props_off_confirmed": true}))
        .await
        .unwrap();
    // The first timer would have fired at 400 ms.
    tokio::time::sleep(Duration::from_millis(300)).await;
    assert!(
        r.fc.lock().unwrap().armed,
        "re-armed: still armed at 550 ms"
    );
    let s = r.json("link_status", json!({})).await;
    assert_eq!(s["armed_by_server"], json!(true));
    tokio::time::sleep(Duration::from_millis(300)).await;
    assert!(!r.fc.lock().unwrap().armed, "the second timer disarms");
}

#[tokio::test]
async fn heading_hold_wraps_to_plus_minus_180() {
    let r = rig(Options::default()).await;
    for (deg, cdeg) in [(350.0, -1000), (90.0, 9000), (-90.0, -9000), (720.5, 50)] {
        r.call(
            "set_heading_hold",
            json!({"enabled": true, "heading_deg": deg}),
        )
        .await
        .unwrap();
        assert_eq!(r.fc.lock().unwrap().heading_cdeg, Some(cdeg), "{deg}°");
    }
}

#[tokio::test]
async fn disconnecting_disarms() {
    let r = rig(motors(60_000)).await;
    r.call("arm", json!({"props_off_confirmed": true}))
        .await
        .unwrap();
    r.call("disconnect", json!({})).await.unwrap();
    let f = r.fc.lock().unwrap();
    assert!(!f.armed && f.disarms >= 1);
}

#[tokio::test]
async fn tools_say_when_not_connected() {
    let server = ElleMcp::new(Options::default());
    let (a, b) = tokio::io::duplex(64 * 1024);
    tokio::spawn(async move {
        let s = server.serve(a).await.unwrap();
        let _ = s.waiting().await;
    });
    let client = ().serve(b).await.unwrap();
    let r = client
        .call_tool(
            CallToolRequestParams::new("read")
                .with_arguments(json!({"what": "status"}).as_object().unwrap().clone()),
        )
        .await
        .unwrap();
    assert_eq!(r.is_error, Some(true));
    let text = r.content[0].as_text().unwrap().text.clone();
    assert!(text.contains("connect"), "{text}");
}

#[tokio::test]
async fn defmt_tools_explain_what_is_missing() {
    let r = rig(Options::default()).await;
    let e = r.call("log", json!({})).await.unwrap_err();
    assert!(e.contains("connect_defmt"), "{e}");
    let e = r
        .call("wait_log", json!({"contains": "x", "timeout_s": 0.1}))
        .await
        .unwrap_err();
    assert!(e.contains("connect_defmt"), "{e}");
    let e = r.call("connect_defmt", json!({})).await.unwrap_err();
    assert!(e.contains("no ELF"), "{e}");
    let s = r.json("link_status", json!({})).await;
    assert_eq!(s["defmt_log"], json!(null));
}

/// A fresh directory under the system temp dir, removed on drop (also when an
/// assertion fails), named so no two runs or tests share one.
struct TempDir(std::path::PathBuf);

impl TempDir {
    fn new(tag: &str) -> Self {
        let nanos = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map_or(0, |d| d.as_nanos());
        let p = std::env::temp_dir().join(format!("elle-{tag}-{}-{nanos}", std::process::id()));
        std::fs::create_dir_all(&p).unwrap();
        Self(p)
    }
}

impl Drop for TempDir {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

fn esc_write() -> Options {
    Options {
        allow_esc_write: true,
        ..Options::default()
    }
}

#[tokio::test]
async fn esc_reads_are_always_allowed() {
    let r = rig(Options::default()).await;
    let m = r.json("esc_mode", json!({"action": "get"})).await;
    assert_eq!(m["mode"], json!("EscConfig"));
    let c = r.json("esc_connect", json!({"engine": "left"})).await;
    assert_eq!(c["device_id"], json!("0x1F06"));
    assert_eq!(c["firmware_name"], json!("AM32_TEST"));
    let s = r.json("esc_read", json!({"engine": "left"})).await;
    assert_eq!(s["raw_hex"].as_str().unwrap().len(), 2 * ESC_EEPROM_SIZE);
    assert!(s["settings"].is_object(), "{s}");
}

#[tokio::test]
async fn esc_writes_need_the_flag() {
    let r = rig(Options::default()).await;
    let e = r
        .call(
            "esc_set",
            json!({"engine": "left", "fields": {"beep_volume": 5}}),
        )
        .await
        .unwrap_err();
    assert!(e.contains("--dangerously-allow-esc-write"), "{e}");
    let e = r
        .call(
            "esc_restore",
            json!({"engine": "left", "path": "/nonexistent"}),
        )
        .await
        .unwrap_err();
    assert!(e.contains("--dangerously-allow-esc-write"), "{e}");
    assert_eq!(r.fc.lock().unwrap().esc_writes, 0);
}

#[tokio::test]
async fn esc_set_needs_a_backup_then_writes_settings_only() {
    let r = rig(esc_write()).await;
    let e = r
        .call(
            "esc_set",
            json!({"engine": "left", "fields": {"beep_volume": 5}}),
        )
        .await
        .unwrap_err();
    assert!(e.contains("esc_backup"), "{e}");
    assert_eq!(r.fc.lock().unwrap().esc_writes, 0);

    let dir = TempDir::new("esc-test");
    let path = dir.0.join("left.bin");
    let b = r
        .json(
            "esc_backup",
            json!({"engine": "left", "path": path.display().to_string()}),
        )
        .await;
    assert_eq!(b["bytes"], json!(256));
    let before = std::fs::read(&path).unwrap();

    let s = r
        .json(
            "esc_set",
            json!({"engine": "left", "fields": {"beep_volume": 5}}),
        )
        .await;
    assert_eq!(s["changes"][0]["field"], json!("beep_volume"), "{s}");
    let after = r.fc.lock().unwrap().esc_eeprom.clone();
    assert_eq!(&after[..5], &before[..5], "boot and version bytes kept");
    assert_eq!(
        &after[48..],
        &before[48..ESC_EEPROM_SIZE],
        "melody and CAN kept"
    );

    let c = r
        .json(
            "esc_compare",
            json!({"engine": "left", "path": path.display().to_string()}),
        )
        .await;
    assert_eq!(c["identical"], json!(false));

    // Restoring the backup undoes the edit.
    r.json(
        "esc_restore",
        json!({"engine": "left", "path": path.display().to_string()}),
    )
    .await;
    assert_eq!(r.fc.lock().unwrap().esc_eeprom, before[..ESC_EEPROM_SIZE]);
}
