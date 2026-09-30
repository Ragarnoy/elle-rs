//! `elle mcp`: an MCP server (stdio) that holds the debug probe and lets an
//! agent interrogate and command the flight controller, so most of
//! `TEST_PLAN.md` can be run through it.
//!
//! Reading is always allowed. Moving surfaces (elevons, modes, calibrations,
//! ULog) is allowed. **Engines are not**: `arm` and `set_throttle` refuse
//! unless the server was started with `--dangerously-allow-motors`, need
//! `props_off_confirmed`, cap the throttle, and run an armed timer that
//! commands throttle 0 and disarms when it expires (60 s by default,
//! `extend_armed` renews it). While armed the server pings every 100 ms (the
//! firmware fails safe 300 ms after the host goes quiet in pure RPC builds),
//! and it disarms before disconnecting or exiting.

use std::collections::VecDeque;
use std::sync::Arc;
use std::time::{Duration, Instant};

use elle_rpc_icd::*;
use postcard_rpc::host_client::HostClient;
use postcard_rpc::standard_icd::WireError;
use rmcp::handler::server::router::tool::ToolRouter;
use rmcp::handler::server::wrapper::Parameters;
use rmcp::model::{Implementation, ServerCapabilities, ServerConfig};
use rmcp::{ServerHandler, tool, tool_handler, tool_router};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use tokio::sync::Mutex;
use tokio::task::JoinHandle;
use tokio::time::timeout;

use crate::link::Link;

/// How long one RPC request may take.
const REQ_TIMEOUT: Duration = Duration::from_secs(2);
/// Keepalive while armed: well inside the firmware's 300 ms host failsafe.
const KEEPALIVE: Duration = Duration::from_millis(100);
/// Events kept for `events` / `wait_event`.
const EVENT_LOG_LEN: usize = 4000;
/// How long a connection may take by default: the probe attach, the RTT scan
/// and the firmware's first RPC reply together take up to ~20 s, longer
/// right after a flash or reset while the firmware boots.
const CONNECT_LIMIT: Duration = Duration::from_secs(30);
/// Longest `timeout_s` a `connect` call may ask for.
const CONNECT_LIMIT_MAX: Duration = Duration::from_secs(120);

/// What the server may do; set on its command line, never by the agent.
#[derive(Clone, Debug)]
pub struct Options {
    /// `--dangerously-allow-motors`: `arm` and `set_throttle` work at all.
    pub allow_motors: bool,
    /// Longest the engines may stay armed without `extend_armed`.
    pub max_armed: Duration,
    /// Highest throttle `set_throttle` accepts, percent.
    pub max_throttle_pct: u8,
}

impl Default for Options {
    fn default() -> Self {
        Self {
            allow_motors: false,
            max_armed: Duration::from_secs(60),
            max_throttle_pct: 30,
        }
    }
}

/// One firmware event, as buffered.
#[derive(Clone, Debug, Serialize)]
struct Event {
    seq: u64,
    /// Milliseconds since the server started.
    t_ms: u64,
    level: u8,
    code: u16,
    label: &'static str,
}

#[derive(Default)]
struct EventLog {
    next_seq: u64,
    events: VecDeque<Event>,
}

struct Connected {
    client: HostClient<WireError>,
    /// The probe link (`None` when the client was handed in, as in tests).
    link: Option<Link>,
    events: JoinHandle<()>,
}

struct Inner {
    opts: Options,
    started: Instant,
    conn: Mutex<Option<Connected>>,
    events: std::sync::Mutex<EventLog>,
    /// The armed timer and keepalive, while the server has armed the engines.
    armed: Mutex<Option<ArmedGuard>>,
    /// The flight build's defmt log, when attached to one.
    defmt: Mutex<Option<crate::target::DefmtReader>>,
    /// The last build flashed (its ELF decodes the defmt log).
    last_build: Mutex<Option<(crate::target::Build, std::path::PathBuf)>>,
}

struct ArmedGuard {
    deadline: Arc<std::sync::Mutex<Instant>>,
    task: JoinHandle<()>,
}

/// The MCP server.
#[derive(Clone)]
pub struct ElleMcp {
    inner: Arc<Inner>,
    tool_router: ToolRouter<Self>,
}

// ---------------------------------------------------------------------------
// Tool parameters

/// What `read` (and `sample`, `wait_for`) fetch from the firmware.
#[derive(Clone, Copy, Debug, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum Source {
    /// Armed, failsafe, mode, IMU calibrated/errors, RC age, autotune state.
    Status,
    /// Pitch/roll/yaw and rates (centidegrees; `*_deg` fields added).
    Attitude,
    /// Magnetometer raw counts.
    Mag,
    /// Pressure, temperature, altitude, vario.
    Baro,
    /// GNSS fix, position, velocity, accuracy, link state.
    Gnss,
    /// Raw RC channels (rpc-rc builds).
    Rc,
    /// Per-engine eRPM, throttle, targets, telemetry.
    Engine,
    /// Controller output: setpoints, corrections, elevon pulses, heading hold.
    Controller,
    /// Control loop and IMU timing (performance-monitoring builds).
    Perf,
    /// ULog recording state.
    Ulog,
    /// Mag calibration offsets and progress.
    MagCal,
    /// Level calibration mount offsets.
    LevelCal,
    /// Firmware uptime, µs.
    Time,
    /// Firmware version.
    Version,
    /// What the firmware was built as: platform, features, turn compensation,
    /// loop rate, git describe.
    Build,
    /// The navigator's latest update: home, validity bits, position, bank demand.
    Nav,
    /// Core 1 (IMU task) load over the last ~1 s: mean/max busy µs, backlog.
    Core1,
}

#[derive(Deserialize, JsonSchema)]
pub struct ConnectReq {
    /// Give up after this long, seconds (default 30, max 120). Probe attach,
    /// RTT scan and the first RPC reply together take up to ~20 s.
    #[serde(default)]
    pub timeout_s: Option<f64>,
}

#[derive(Deserialize, JsonSchema)]
pub struct ReadReq {
    pub what: Source,
}

#[derive(Deserialize, JsonSchema)]
pub struct SampleReq {
    pub what: Source,
    /// How long to sample, seconds (max 600).
    pub duration_s: f64,
    /// Samples per second (max 20; the RTT link carries ~100 requests/s in total).
    #[serde(default = "default_rate")]
    pub rate_hz: f64,
    /// Only these fields (dotted paths, e.g. `pitch_deg`); all numeric fields if empty.
    #[serde(default)]
    pub fields: Vec<String>,
}

fn default_rate() -> f64 {
    5.0
}

/// A comparison for `wait_for`.
#[derive(Clone, Copy, Debug, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum Op {
    Lt,
    Le,
    Gt,
    Ge,
    Eq,
    Ne,
    /// |field| > value
    AbsGt,
    /// |field| < value
    AbsLt,
}

#[derive(Deserialize, JsonSchema)]
pub struct WaitForReq {
    pub what: Source,
    /// Dotted field path, e.g. `pitch_deg` or `armed` (booleans read as 0/1).
    pub field: String,
    pub op: Op,
    pub value: f64,
    /// Give up after this long, seconds (max 600).
    pub timeout_s: f64,
    /// Polls per second (max 20).
    #[serde(default = "default_rate")]
    pub rate_hz: f64,
}

#[derive(Deserialize, JsonSchema)]
pub struct EventsReq {
    /// Only events after this sequence number (0: all buffered).
    #[serde(default)]
    pub since_seq: u64,
    /// Only these codes (all if empty).
    #[serde(default)]
    pub codes: Vec<u16>,
    /// At most this many, newest last (default 100).
    #[serde(default)]
    pub limit: Option<usize>,
}

#[derive(Deserialize, JsonSchema)]
pub struct WaitEventReq {
    /// Any of these event codes.
    pub codes: Vec<u16>,
    /// Only events after this sequence number; omit to wait for a new one.
    #[serde(default)]
    pub since_seq: Option<u64>,
    /// Give up after this long, seconds (max 600).
    pub timeout_s: f64,
}

#[derive(Deserialize, JsonSchema)]
pub struct ArmReq {
    /// The operator confirmed the propellers are removed. Required.
    pub props_off_confirmed: bool,
}

#[derive(Deserialize, JsonSchema)]
pub struct ExtendArmedReq {
    /// Seconds from now until the server disarms (at most the server's limit).
    pub seconds: f64,
}

#[derive(Deserialize, JsonSchema)]
pub struct ThrottleReq {
    /// Percent, 0..=the server's cap (30 by default).
    pub percent: u8,
}

#[derive(Deserialize, JsonSchema)]
pub struct ElevonsReq {
    /// -100..=100
    pub left: i8,
    /// -100..=100
    pub right: i8,
}

#[derive(Clone, Copy, Debug, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum Mode {
    Manual,
    Stabilized,
    AltitudeHold,
}

#[derive(Deserialize, JsonSchema)]
pub struct ModeReq {
    pub mode: Mode,
}

#[derive(Deserialize, JsonSchema)]
pub struct HeadingHoldReq {
    pub enabled: bool,
    /// Target heading, degrees (used when enabling).
    #[serde(default)]
    pub heading_deg: f64,
}

#[derive(Clone, Copy, Debug, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum CalAction {
    Start,
    Clear,
    Get,
}

#[derive(Deserialize, JsonSchema)]
pub struct CalReq {
    pub action: CalAction,
}

#[derive(Clone, Copy, Debug, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum UlogAction {
    Start,
    Stop,
    Info,
}

#[derive(Deserialize, JsonSchema)]
pub struct UlogReq {
    pub action: UlogAction,
}

#[derive(Clone, Copy, Debug, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum AutotuneAction {
    Pitch,
    Roll,
    Abort,
    /// Save the current PID gains to flash (disarmed only).
    SavePid,
    /// Erase the saved PID gains from flash (disarmed only).
    ErasePid,
}

#[derive(Deserialize, JsonSchema)]
pub struct AutotuneReq {
    pub action: AutotuneAction,
    /// Relay amplitude, degrees (pitch/roll; default 5).
    #[serde(default)]
    pub relay_deg: Option<f64>,
    /// Cycles to measure (pitch/roll; default 6, max 14).
    #[serde(default)]
    pub cycles: Option<u8>,
}

#[derive(Clone, Copy, Debug, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum AirframeParam {
    Eagle,
    Dart,
}

/// Which firmware: the three build lines of CLAUDE.md (RPC ones keep `gnss`).
#[derive(Clone, Copy, Debug, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum Profile {
    /// CRSF/ELRS flight firmware (default features; defmt log, no RPC).
    Flight,
    /// RPC ground-test firmware (`rpc-control,gnss`): the host commands.
    Rpc,
    /// RPC monitoring with RC control (`rpc-control,rpc-rc,gnss`).
    RpcRc,
}

#[derive(Deserialize, JsonSchema)]
pub struct FlashReq {
    pub airframe: AirframeParam,
    pub profile: Profile,
    /// Extra features, e.g. `imu-raw-log`, `performance-monitoring`.
    #[serde(default)]
    pub extra_features: Vec<String>,
}

#[derive(Deserialize, JsonSchema)]
pub struct DefmtReq {
    /// ELF of the running build (default: the last one `build_and_flash` flashed).
    #[serde(default)]
    pub elf: Option<String>,
}

#[derive(Deserialize, JsonSchema)]
pub struct LogReq {
    /// Only lines after this sequence number (0: all buffered).
    #[serde(default)]
    pub since_seq: u64,
    /// Only lines containing this text.
    #[serde(default)]
    pub contains: Option<String>,
    /// At most this many, newest last (default 100).
    #[serde(default)]
    pub limit: Option<usize>,
}

#[derive(Deserialize, JsonSchema)]
pub struct WaitLogReq {
    /// Text the line must contain, e.g. `115200 baud`.
    pub contains: String,
    /// Only lines after this sequence number; omit to wait for a new one.
    #[serde(default)]
    pub since_seq: Option<u64>,
    /// Give up after this long, seconds (max 600).
    pub timeout_s: f64,
}

#[derive(Deserialize, JsonSchema)]
pub struct CopyLogsReq {
    /// File names on the card, e.g. `LOG_0061.ulg`. Empty: the newest `last`.
    #[serde(default)]
    pub names: Vec<String>,
    /// How many of the newest files when no names are given (default 1).
    #[serde(default)]
    pub last: Option<usize>,
}

#[derive(Deserialize, JsonSchema)]
pub struct AnalyseReq {
    pub command: crate::analysis::LogCommand,
    /// Log names in `logs/` (e.g. `LOG_0061.ulg`) or paths.
    pub files: Vec<String>,
    /// `window` only: start, seconds from the file's first record.
    #[serde(default)]
    pub t0_s: Option<f64>,
    /// `window` only: end.
    #[serde(default)]
    pub t1_s: Option<f64>,
}

#[derive(Deserialize, JsonSchema)]
pub struct ReplayReq {
    /// Log name in `logs/` or a path (an `imu-raw-log` build's log). With
    /// `simulate`, where to write the simulated flight.
    pub file: String,
    /// Also run the alternative filters and score them against the gyro reference.
    #[serde(default)]
    pub compare: bool,
    /// Count turns by turn rate, not bank (the vehicle test, 7.2).
    #[serde(default)]
    pub score_by_rate: bool,
    /// Simulate a flight into `file` first (host check of the harness).
    #[serde(default)]
    pub simulate: bool,
    /// Simulation: wind towards north / east, m/s; accel vibration, m/s²;
    /// residual gyro bias, °/s.
    #[serde(default)]
    pub wind_north: f64,
    #[serde(default)]
    pub wind_east: f64,
    #[serde(default)]
    pub vibration: f64,
    #[serde(default)]
    pub gyro_bias_dps: f64,
}

#[derive(Deserialize, JsonSchema)]
pub struct TestRecordReq {
    /// TEST_PLAN part, e.g. `7`.
    pub part: String,
    /// Row or section, e.g. `7.2`.
    pub row: String,
    pub outcome: crate::analysis::Outcome,
    /// What was measured or seen, and why this outcome.
    #[serde(default)]
    pub note: String,
    /// Evidence: ULog files, CSVs.
    #[serde(default)]
    pub logs: Vec<String>,
}

#[derive(Deserialize, JsonSchema)]
pub struct TestReportReq {
    /// `YYYY-MM-DD`; default the newest results file.
    #[serde(default)]
    pub date: Option<String>,
}

// ---------------------------------------------------------------------------

type ToolResult = Result<String, String>;

fn to_text(v: &Value) -> ToolResult {
    serde_json::to_string_pretty(v).map_err(|e| e.to_string())
}

fn ack(a: AckResp) -> ToolResult {
    if a.success {
        Ok("ok".to_string())
    } else {
        Err(format!(
            "refused by the firmware (error code {})",
            a.error_code
        ))
    }
}

/// Every numeric (or boolean) leaf of a JSON value, keyed by dotted path.
fn numeric_fields(v: &Value, prefix: &str, out: &mut Vec<(String, f64)>) {
    match v {
        Value::Number(n) => out.push((prefix.to_string(), n.as_f64().unwrap_or(f64::NAN))),
        Value::Bool(b) => out.push((prefix.to_string(), f64::from(u8::from(*b)))),
        Value::Object(m) => {
            for (k, x) in m {
                let p = if prefix.is_empty() {
                    k.clone()
                } else {
                    format!("{prefix}.{k}")
                };
                numeric_fields(x, &p, out);
            }
        }
        Value::Array(a) => {
            for (i, x) in a.iter().enumerate() {
                numeric_fields(x, &format!("{prefix}[{i}]"), out);
            }
        }
        _ => {}
    }
}

fn field(v: &Value, path: &str) -> Option<f64> {
    let mut all = Vec::new();
    numeric_fields(v, "", &mut all);
    all.into_iter().find(|(p, _)| p == path).map(|(_, x)| x)
}

fn compare(x: f64, op: Op, v: f64) -> bool {
    match op {
        Op::Lt => x < v,
        Op::Le => x <= v,
        Op::Gt => x > v,
        Op::Ge => x >= v,
        Op::Eq => x == v,
        Op::Ne => x != v,
        Op::AbsGt => x.abs() > v,
        Op::AbsLt => x.abs() < v,
    }
}

fn period(rate_hz: f64) -> Duration {
    Duration::from_secs_f64(1.0 / rate_hz.clamp(0.1, 20.0))
}

fn limit_secs(s: f64) -> Duration {
    Duration::from_secs_f64(s.clamp(0.0, 600.0))
}

async fn req<E>(client: &HostClient<WireError>, r: &E::Request) -> Result<E::Response, String>
where
    E: postcard_rpc::Endpoint,
    E::Request: Serialize,
    E::Response: serde::de::DeserializeOwned,
{
    match timeout(REQ_TIMEOUT, client.send_resp::<E>(r)).await {
        Ok(Ok(v)) => Ok(v),
        Ok(Err(e)) => Err(format!("RPC error: {e:?}")),
        Err(_) => Err("no answer within 2 s (link lost, or not an RPC build)".to_string()),
    }
}

/// Read one source as JSON.
async fn read_source(client: &HostClient<WireError>, what: Source) -> Result<Value, String> {
    let out = match what {
        Source::Status => jv(&req::<GetStatusEndpoint>(client, &()).await?),
        Source::Attitude => {
            let a = req::<GetAttitudeEndpoint>(client, &()).await?;
            let mut j = jv(&a);
            let deg = |c: i16| f64::from(c) / 100.0;
            for (k, c) in [
                ("pitch_deg", a.pitch_cdeg),
                ("roll_deg", a.roll_cdeg),
                ("yaw_deg", a.yaw_cdeg),
                ("pitch_rate_dps", a.pitch_rate_cdeg),
                ("roll_rate_dps", a.roll_rate_cdeg),
                ("yaw_rate_dps", a.yaw_rate_cdeg),
            ] {
                j[k] = json!(deg(c));
            }
            j
        }
        Source::Mag => jv(&req::<GetMagnetometerEndpoint>(client, &()).await?),
        Source::Baro => jv(&req::<GetBarometerEndpoint>(client, &()).await?),
        Source::Gnss => jv(&req::<GetGnssEndpoint>(client, &()).await?),
        Source::Rc => jv(&req::<GetRcChannelsEndpoint>(client, &()).await?),
        Source::Engine => jv(&req::<GetEngineEndpoint>(client, &()).await?),
        Source::Controller => jv(&req::<GetControllerOutputEndpoint>(client, &()).await?),
        Source::Perf => jv(&req::<GetPerformanceEndpoint>(client, &()).await?),
        Source::Ulog => jv(&req::<GetULogInfoEndpoint>(client, &()).await?),
        Source::MagCal => jv(&req::<GetMagCalEndpoint>(client, &()).await?),
        Source::LevelCal => jv(&req::<GetLevelCalEndpoint>(client, &()).await?),
        Source::Time => json!({ "uptime_us": req::<GetTimeEndpoint>(client, &()).await? }),
        Source::Version => jv(&req::<GetVersionEndpoint>(client, &()).await?),
        Source::Build => {
            let b = req::<GetBuildInfoEndpoint>(client, &()).await?;
            let text = |f: &[u8]| {
                String::from_utf8_lossy(f)
                    .trim_end_matches('\0')
                    .to_string()
            };
            // No bit for rpc-control: only an RPC build answers this at all.
            let features: Vec<&str> = std::iter::once("rpc-control")
                .chain(
                    [
                        (BUILD_FEATURE_RPC_RC, "rpc-rc"),
                        (BUILD_FEATURE_GNSS, "gnss"),
                        (BUILD_FEATURE_IMU_RAW_LOG, "imu-raw-log"),
                        (BUILD_FEATURE_PERF_MON, "performance-monitoring"),
                    ]
                    .into_iter()
                    .filter(|(bit, _)| b.features & bit != 0)
                    .map(|(_, n)| n),
                )
                .collect();
            let turn_comp = ["off", "centripetal", "gnss_accel"]
                .get(usize::from(b.turn_comp))
                .copied()
                .unwrap_or("unknown");
            json!({
                "platform": text(&b.platform),
                "git": text(&b.git),
                "features": features,
                "turn_comp": turn_comp,
                "accel_gate_g": (b.accel_gate_g_x100 != 0)
                    .then(|| f64::from(b.accel_gate_g_x100) / 100.0),
                "loop_hz": b.loop_hz,
            })
        }
        Source::Nav => jv(&req::<GetNavEndpoint>(client, &()).await?),
        Source::Core1 => jv(&req::<GetCore1LoadEndpoint>(client, &()).await?),
    };
    Ok(out)
}

fn jv<T: Serialize>(t: &T) -> Value {
    serde_json::to_value(t).unwrap_or(Value::Null)
}

impl ElleMcp {
    #[must_use]
    pub fn new(opts: Options) -> Self {
        Self {
            inner: Arc::new(Inner {
                opts,
                started: Instant::now(),
                conn: Mutex::new(None),
                events: std::sync::Mutex::new(EventLog::default()),
                armed: Mutex::new(None),
                defmt: Mutex::new(None),
                last_build: Mutex::new(None),
            }),
            tool_router: Self::tool_router(),
        }
    }

    /// A server already connected through `client` (tests: a fake device).
    pub async fn with_client(opts: Options, client: HostClient<WireError>) -> Result<Self, String> {
        let s = Self::new(opts);
        s.attach(client, None).await?;
        Ok(s)
    }

    async fn attach(
        &self,
        client: HostClient<WireError>,
        link: Option<Link>,
    ) -> Result<(), String> {
        let mut sub = client
            .subscribe_multi::<LogTopic>(64)
            .await
            .map_err(|e| format!("event subscription failed: {e:?}"))?;
        let inner = self.inner.clone();
        let events = tokio::spawn(async move {
            loop {
                match sub.recv().await {
                    Ok(msg) => inner.push_event(msg.level, msg.code),
                    // Lost some to a slow consumer: keep the rest.
                    Err(postcard_rpc::host_client::MultiSubRxError::Lagged(_)) => {}
                    Err(_) => return,
                }
            }
        });
        *self.inner.conn.lock().await = Some(Connected {
            client,
            link,
            events,
        });
        Ok(())
    }

    async fn client(&self) -> Result<HostClient<WireError>, String> {
        self.inner
            .conn
            .lock()
            .await
            .as_ref()
            .map(|c| c.client.clone())
            .ok_or_else(|| "not connected: call `connect` first".to_string())
    }

    /// Disarm (if this server armed) and release the probe. Called on exit.
    pub async fn shutdown(&self) {
        let _ = self.stop_engines().await;
        if let Some(mut d) = self.inner.defmt.lock().await.take() {
            let _ = tokio::task::spawn_blocking(move || d.stop()).await;
        }
        if let Some(c) = self.inner.conn.lock().await.take() {
            c.events.abort();
            c.client.close();
            if let Some(l) = c.link {
                let _ = tokio::task::spawn_blocking(move || l.close()).await;
            }
        }
    }

    /// After flashing: the RPC link for RPC builds, the defmt log otherwise.
    async fn reattach(&self, build: &crate::target::Build, elf: &std::path::Path) -> String {
        if build.is_rpc() {
            self.connect_retrying().await
        } else {
            self.start_defmt(elf).await
        }
    }

    /// Attach to the probe and wait for the firmware to answer a ping, all
    /// within `limit`; every stage is retried until then (the probe may still
    /// be enumerating, the firmware still booting). How long each stage took.
    async fn open_link(&self, limit: Duration) -> Result<Value, String> {
        let started = Instant::now();
        let link = tokio::task::spawn_blocking(move || Link::connect_within(Some(limit)))
            .await
            .map_err(|e| e.to_string())?
            .map_err(|e| format!("no RTT link within {} s: {e:#}", limit.as_secs()))?;
        let rtt_s = started.elapsed().as_secs_f64();
        // RTT is up before the RPC server runs: wait for its first reply.
        let client = link.client.clone();
        loop {
            let last = match req::<PingEndpoint>(&client, &()).await {
                Ok(()) => break,
                Err(e) => e,
            };
            if !link.alive() {
                return Err(format!(
                    "the RTT link dropped before the firmware answered: {last}"
                ));
            }
            if started.elapsed() > limit {
                let _ = tokio::task::spawn_blocking(move || link.close()).await;
                return Err(format!(
                    "RTT attached after {rtt_s:.1} s, but no RPC reply within {} s \
                     (a flight build? use connect_defmt): {last}",
                    limit.as_secs()
                ));
            }
            tokio::time::sleep(Duration::from_millis(200)).await;
        }
        self.attach(client, Some(link)).await?;
        Ok(json!({
            "rtt_attached_s": (rtt_s * 10.0).round() / 10.0,
            "first_reply_s": (started.elapsed().as_secs_f64() * 10.0).round() / 10.0,
        }))
    }

    /// Connect after a flash or reset; the result, or why not.
    async fn connect_retrying(&self) -> String {
        match self.open_link(CONNECT_LIMIT).await {
            Ok(t) => format!("rpc (link up after {} s)", t["first_reply_s"]),
            Err(e) => format!("not attached: {e}"),
        }
    }

    async fn start_defmt(&self, elf: &std::path::Path) -> String {
        let e = elf.to_path_buf();
        match tokio::task::spawn_blocking(move || {
            crate::target::DefmtReader::start(&e, CONNECT_LIMIT)
        })
        .await
        {
            Ok(Ok(r)) => {
                *self.inner.defmt.lock().await = Some(r);
                "defmt".to_string()
            }
            Ok(Err(e)) => format!("not attached: {e:#}"),
            Err(e) => format!("not attached: {e}"),
        }
    }

    /// Throttle 0 and disarm, cancelling the armed timer.
    async fn stop_engines(&self) -> Result<(), String> {
        if let Some(g) = self.inner.armed.lock().await.take() {
            g.task.abort();
        }
        let client = self.client().await?;
        let _ = req::<SetThrottleEndpoint>(&client, &SetThrottleReq { percent: 0 }).await;
        ack(req::<DisarmEndpoint>(&client, &()).await?).map(|_| ())
    }
}

impl Inner {
    fn push_event(&self, level: u8, code: u16) {
        let mut log = self
            .events
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        log.next_seq += 1;
        let e = Event {
            seq: log.next_seq,
            t_ms: self.started.elapsed().as_millis() as u64,
            level,
            code,
            label: crate::events::label(code),
        };
        log.events.push_back(e);
        while log.events.len() > EVENT_LOG_LEN {
            log.events.pop_front();
        }
    }

    fn events_after(&self, since: u64, codes: &[u16]) -> Vec<Event> {
        let log = self
            .events
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        log.events
            .iter()
            .filter(|e| e.seq > since && (codes.is_empty() || codes.contains(&e.code)))
            .cloned()
            .collect()
    }

    fn last_seq(&self) -> u64 {
        self.events
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .next_seq
    }
}

#[tool_router]
impl ElleMcp {
    #[tool(
        description = "Attach to the debug probe and the firmware's RPC link (RPC builds: \
        rpc-control or rpc-control,rpc-rc). Takes up to ~20 s (probe attach, RTT scan, first \
        reply); retries every stage until `timeout_s` (default 30). Returns how long it took. \
        No-op when already connected. The probe has one owner: the TUI cannot run at the same time."
    )]
    async fn connect(&self, Parameters(r): Parameters<ConnectReq>) -> ToolResult {
        if self.inner.conn.lock().await.is_some() {
            return self.link_status().await;
        }
        // A defmt session (flight build) holds the probe: release it first.
        if let Some(mut d) = self.inner.defmt.lock().await.take() {
            let _ = tokio::task::spawn_blocking(move || d.stop()).await;
        }
        let limit = r.timeout_s.map_or(CONNECT_LIMIT, |s| {
            Duration::from_secs_f64(s.clamp(1.0, CONNECT_LIMIT_MAX.as_secs_f64()))
        });
        let timing = self
            .open_link(limit)
            .await
            .map_err(|e| format!("could not connect: {e}"))?;
        let mut status: Value =
            serde_json::from_str(&self.link_status().await?).map_err(|e| e.to_string())?;
        status["connect_timing"] = timing;
        to_text(&status)
    }

    #[tool(
        description = "Disarm if this server armed, then release the probe (e.g. for \
        `cargo run` or the TUI)."
    )]
    async fn disconnect(&self) -> ToolResult {
        self.shutdown().await;
        Ok("disconnected".to_string())
    }

    #[tool(
        description = "Whether the link is up, the firmware version and uptime, events \
        buffered, whether this server has the engines armed, and what the server allows."
    )]
    async fn link_status(&self) -> ToolResult {
        let connected = self.inner.conn.lock().await.is_some();
        let defmt = self
            .inner
            .defmt
            .lock()
            .await
            .as_ref()
            .map(|d| json!({"elf": d.elf.display().to_string(), "alive": d.alive()}));
        let mut j = json!({
            "connected": connected,
            "defmt_log": defmt,
            "motors_allowed": self.inner.opts.allow_motors,
            "max_throttle_pct": self.inner.opts.max_throttle_pct,
            "max_armed_s": self.inner.opts.max_armed.as_secs_f64(),
            "armed_by_server": self.inner.armed.lock().await.is_some(),
            "last_event_seq": self.inner.last_seq(),
        });
        if connected {
            let client = self.client().await?;
            match read_source(&client, Source::Version).await {
                Ok(v) => j["firmware_version"] = v,
                Err(e) => j["error"] = json!(e),
            }
            // Older firmware has no build info: leave it out.
            if let Ok(b) = read_source(&client, Source::Build).await {
                j["build"] = b;
            }
            if let Ok(t) = read_source(&client, Source::Time).await {
                j["uptime_s"] = json!(t["uptime_us"].as_f64().unwrap_or(0.0) / 1e6);
            }
        }
        to_text(&j)
    }

    #[tool(
        description = "Read one thing from the firmware, as JSON. Units are in the field \
        names (cdeg = centidegrees; attitude also gets *_deg fields)."
    )]
    async fn read(&self, Parameters(r): Parameters<ReadReq>) -> ToolResult {
        let client = self.client().await?;
        to_text(&read_source(&client, r.what).await?)
    }

    #[tool(
        description = "Poll a source for a while and return, per numeric field: count, \
        min, max, mean, std, first, last. For checks like 'pitch drifts < 0.5° over 10 min'."
    )]
    async fn sample(&self, Parameters(r): Parameters<SampleReq>) -> ToolResult {
        let client = self.client().await?;
        let end = Instant::now() + limit_secs(r.duration_s);
        let mut ticker = tokio::time::interval(period(r.rate_hz));
        let mut series: std::collections::BTreeMap<String, Vec<f64>> = Default::default();
        let mut errors = 0u32;
        while Instant::now() < end {
            ticker.tick().await;
            match read_source(&client, r.what).await {
                Ok(v) => {
                    let mut f = Vec::new();
                    numeric_fields(&v, "", &mut f);
                    for (k, x) in f {
                        if r.fields.is_empty() || r.fields.contains(&k) {
                            series.entry(k).or_default().push(x);
                        }
                    }
                }
                Err(_) => errors += 1,
            }
        }
        let stats: serde_json::Map<String, Value> = series
            .into_iter()
            .map(|(k, xs)| {
                let n = xs.len() as f64;
                let mean = xs.iter().sum::<f64>() / n;
                let var = xs.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / n;
                let min = xs.iter().copied().fold(f64::INFINITY, f64::min);
                let max = xs.iter().copied().fold(f64::NEG_INFINITY, f64::max);
                (
                    k,
                    json!({"n": xs.len(), "min": min, "max": max, "mean": mean,
                           "std": var.sqrt(), "first": xs[0], "last": xs[xs.len() - 1]}),
                )
            })
            .collect();
        to_text(&json!({ "failed_reads": errors, "fields": stats }))
    }

    #[tool(
        description = "Poll a source until a field meets a condition (e.g. attitude \
        pitch_deg gt 15 while the operator tilts the nose up). Returns the value and how long \
        it took, or an error on timeout with the last value seen."
    )]
    async fn wait_for(&self, Parameters(r): Parameters<WaitForReq>) -> ToolResult {
        let client = self.client().await?;
        let started = Instant::now();
        let end = started + limit_secs(r.timeout_s);
        let mut ticker = tokio::time::interval(period(r.rate_hz));
        let mut last = None;
        while Instant::now() < end {
            ticker.tick().await;
            let Ok(v) = read_source(&client, r.what).await else {
                continue;
            };
            let Some(x) = field(&v, &r.field) else {
                return Err(format!(
                    "no numeric field `{}` in {:?}: {}",
                    r.field, r.what, v
                ));
            };
            last = Some(x);
            if compare(x, r.op, r.value) {
                return to_text(&json!({"met": true, "value": x,
                    "after_s": started.elapsed().as_secs_f64()}));
            }
        }
        Err(format!(
            "timed out after {:.1} s; last {} = {:?}",
            r.timeout_s, r.field, last
        ))
    }

    #[tool(
        description = "Firmware events buffered since connecting (numeric codes with labels, \
        docs/OPERATIONS.md#event-codes). Each has a sequence number for `since_seq`."
    )]
    async fn events(&self, Parameters(r): Parameters<EventsReq>) -> ToolResult {
        let mut ev = self.inner.events_after(r.since_seq, &r.codes);
        let limit = r.limit.unwrap_or(100);
        if ev.len() > limit {
            ev.drain(..ev.len() - limit);
        }
        to_text(&json!({ "last_seq": self.inner.last_seq(), "events": ev }))
    }

    #[tool(
        description = "Wait for any of the given event codes (e.g. 14 RC failsafe, 16 kill \
        engaged). By default only events after this call; pass since_seq to include earlier ones."
    )]
    async fn wait_event(&self, Parameters(r): Parameters<WaitEventReq>) -> ToolResult {
        self.client().await?;
        let since = r.since_seq.unwrap_or_else(|| self.inner.last_seq());
        let end = Instant::now() + limit_secs(r.timeout_s);
        loop {
            if let Some(e) = self.inner.events_after(since, &r.codes).into_iter().next() {
                return to_text(&json!(e));
            }
            if Instant::now() >= end {
                return Err(format!(
                    "no event {:?} within {:.1} s",
                    r.codes, r.timeout_s
                ));
            }
            tokio::time::sleep(Duration::from_millis(50)).await;
        }
    }

    // --- Build, flash, reset, defmt ----------------------------------------------

    #[tool(
        description = "Build firmware (cargo build --release) and flash it through the probe, \
        then reset and reattach: RPC profiles connect the RPC link, the flight profile starts \
        reading its defmt log (`log`, `wait_log`). Disarms and releases the probe first. Takes \
        a minute or two. Ask the operator before replacing the firmware."
    )]
    async fn build_and_flash(&self, Parameters(r): Parameters<FlashReq>) -> ToolResult {
        use crate::target::{Airframe, Build};
        let (no_default_features, mut features): (bool, Vec<String>) = match r.profile {
            Profile::Flight => (false, Vec::new()),
            Profile::Rpc => (true, vec!["rpc-control".into(), "gnss".into()]),
            Profile::RpcRc => (
                true,
                vec!["rpc-control".into(), "rpc-rc".into(), "gnss".into()],
            ),
        };
        features.extend(r.extra_features);
        let build = Build {
            airframe: match r.airframe {
                AirframeParam::Eagle => Airframe::Eagle,
                AirframeParam::Dart => Airframe::Dart,
            },
            no_default_features,
            features,
        };
        self.shutdown().await;
        let b = build.clone();
        let elf = tokio::task::spawn_blocking(move || b.run())
            .await
            .map_err(|e| e.to_string())?
            .map_err(|e| format!("{e:#}"))?;
        let e = elf.clone();
        tokio::task::spawn_blocking(move || crate::target::flash(&e))
            .await
            .map_err(|e| e.to_string())?
            .map_err(|e| format!("flash failed: {e:#}"))?;
        *self.inner.last_build.lock().await = Some((build.clone(), elf.clone()));
        let attached = self.reattach(&build, &elf).await;
        to_text(&json!({
            "flashed": elf.display().to_string(),
            "features": build.features,
            "no_default_features": build.no_default_features,
            "attached": attached,
        }))
    }

    #[tool(
        description = "Reset the flight controller (it reboots), then reattach as before \
        (RPC link or defmt log). Disarms first."
    )]
    async fn reset_target(&self) -> ToolResult {
        let was_defmt = self
            .inner
            .defmt
            .lock()
            .await
            .as_ref()
            .map(|d| d.elf.clone());
        let was_rpc = self.inner.conn.lock().await.is_some();
        self.shutdown().await;
        tokio::task::spawn_blocking(crate::target::reset)
            .await
            .map_err(|e| e.to_string())?
            .map_err(|e| format!("reset failed: {e:#}"))?;
        let attached = if let Some(elf) = was_defmt {
            self.start_defmt(&elf).await
        } else if was_rpc {
            self.connect_retrying().await
        } else {
            "not attached (was not before)".to_string()
        };
        to_text(&json!({ "reset": true, "attached": attached }))
    }

    #[tool(
        description = "Read a flight build's defmt log (flight builds have no RPC): attach \
        and decode with the given ELF, or the one last flashed. Then `log`, `wait_log`."
    )]
    async fn connect_defmt(&self, Parameters(r): Parameters<DefmtReq>) -> ToolResult {
        let elf = match r.elf {
            Some(p) => std::path::PathBuf::from(p),
            None => self
                .inner
                .last_build
                .lock()
                .await
                .as_ref()
                .map(|(_, e)| e.clone())
                .ok_or("no ELF: pass `elf`, or flash with build_and_flash first")?,
        };
        self.shutdown().await;
        Ok(self.start_defmt(&elf).await)
    }

    #[tool(
        description = "Decoded defmt lines buffered from a flight build (see connect_defmt), \
        with sequence numbers for since_seq."
    )]
    async fn log(&self, Parameters(r): Parameters<LogReq>) -> ToolResult {
        let d = self.inner.defmt.lock().await;
        let d = d
            .as_ref()
            .ok_or("no defmt log attached: connect_defmt or build_and_flash")?;
        let ring = d
            .lines
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner);
        let mut lines = ring.after(r.since_seq, r.contains.as_deref());
        let limit = r.limit.unwrap_or(100);
        if lines.len() > limit {
            lines.drain(..lines.len() - limit);
        }
        to_text(&json!({ "last_seq": ring.last_seq(), "alive": d.alive(), "lines": lines }))
    }

    #[tool(
        description = "Wait for a defmt line containing some text (e.g. after a reset: \
        `115200 baud`). By default only lines after this call."
    )]
    async fn wait_log(&self, Parameters(r): Parameters<WaitLogReq>) -> ToolResult {
        let lines = self
            .inner
            .defmt
            .lock()
            .await
            .as_ref()
            .map(|d| d.lines.clone())
            .ok_or("no defmt log attached: connect_defmt or build_and_flash")?;
        let since = r.since_seq.unwrap_or_else(|| {
            lines
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner)
                .last_seq()
        });
        let end = Instant::now() + limit_secs(r.timeout_s);
        loop {
            let hit = lines
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner)
                .after(since, Some(&r.contains))
                .into_iter()
                .next();
            if let Some(l) = hit {
                return to_text(&json!(l));
            }
            if Instant::now() >= end {
                return Err(format!(
                    "no line containing {:?} within {:.1} s",
                    r.contains, r.timeout_s
                ));
            }
            tokio::time::sleep(Duration::from_millis(100)).await;
        }
    }

    // --- Engines: only with --dangerously-allow-motors ---------------------------

    #[tool(
        description = "Arm the engines (props off only). Refused unless the server runs with \
        --dangerously-allow-motors and props_off_confirmed is true. Starts the armed timer: the \
        server commands throttle 0 and disarms when it expires (see link_status); \
        `extend_armed` renews it."
    )]
    async fn arm(&self, Parameters(r): Parameters<ArmReq>) -> ToolResult {
        if !self.inner.opts.allow_motors {
            return Err(
                "refused: engines are off limits; the server must be started with \
                --dangerously-allow-motors (a human decision, in .mcp.json)"
                    .to_string(),
            );
        }
        if !r.props_off_confirmed {
            return Err(
                "refused: confirm the propellers are removed (props_off_confirmed)".to_string(),
            );
        }
        // Re-arming replaces the timer: an old one left running would disarm early.
        if let Some(g) = self.inner.armed.lock().await.take() {
            g.task.abort();
        }
        let client = self.client().await?;
        ack(req::<ArmEndpoint>(&client, &()).await?)?;
        let deadline = Arc::new(std::sync::Mutex::new(
            Instant::now() + self.inner.opts.max_armed,
        ));
        let d = deadline.clone();
        let inner = self.inner.clone();
        let task = tokio::spawn(async move {
            let mut tick = tokio::time::interval(KEEPALIVE);
            loop {
                tick.tick().await;
                let due = *d.lock().unwrap_or_else(std::sync::PoisonError::into_inner);
                if Instant::now() >= due {
                    let _ =
                        req::<SetThrottleEndpoint>(&client, &SetThrottleReq { percent: 0 }).await;
                    let _ = req::<DisarmEndpoint>(&client, &()).await;
                    inner.armed.lock().await.take();
                    return;
                }
                let _ = req::<PingEndpoint>(&client, &()).await;
            }
        });
        *self.inner.armed.lock().await = Some(ArmedGuard { deadline, task });
        Ok(format!(
            "armed; the server disarms in {:.0} s unless extend_armed",
            self.inner.opts.max_armed.as_secs_f64()
        ))
    }

    #[tool(
        description = "Renew the armed timer: disarm this many seconds from now (at most the \
        server's limit)."
    )]
    async fn extend_armed(&self, Parameters(r): Parameters<ExtendArmedReq>) -> ToolResult {
        let g = self.inner.armed.lock().await;
        let Some(g) = g.as_ref() else {
            return Err("not armed by this server".to_string());
        };
        let secs = Duration::from_secs_f64(r.seconds.max(0.0)).min(self.inner.opts.max_armed);
        *g.deadline
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner) = Instant::now() + secs;
        Ok(format!("disarms in {:.0} s", secs.as_secs_f64()))
    }

    #[tool(
        description = "Set engine throttle, percent. Needs --dangerously-allow-motors and \
        an `arm` from this server; capped (see link_status)."
    )]
    async fn set_throttle(&self, Parameters(r): Parameters<ThrottleReq>) -> ToolResult {
        if !self.inner.opts.allow_motors {
            return Err("refused: engines are off limits (--dangerously-allow-motors)".to_string());
        }
        if r.percent > 0 && self.inner.armed.lock().await.is_none() {
            return Err("refused: arm through this server first".to_string());
        }
        if r.percent > self.inner.opts.max_throttle_pct {
            return Err(format!(
                "refused: above the {} % cap",
                self.inner.opts.max_throttle_pct
            ));
        }
        let client = self.client().await?;
        ack(req::<SetThrottleEndpoint>(&client, &SetThrottleReq { percent: r.percent }).await?)
    }

    #[tool(description = "Throttle 0 and disarm. Always allowed.")]
    async fn disarm(&self) -> ToolResult {
        self.stop_engines().await.map(|()| "disarmed".to_string())
    }

    #[tool(description = "Emergency stop: engines off immediately. Always allowed.")]
    async fn emergency_stop(&self) -> ToolResult {
        if let Some(g) = self.inner.armed.lock().await.take() {
            g.task.abort();
        }
        let client = self.client().await?;
        ack(req::<EmergencyStopEndpoint>(&client, &()).await?)
    }

    // --- Surfaces, modes, calibration, logging -------------------------------------

    #[tool(
        description = "Elevon positions -100..=100 (pure RPC builds; ignored with rpc-rc). \
        Works disarmed: surface checks."
    )]
    async fn set_elevons(&self, Parameters(r): Parameters<ElevonsReq>) -> ToolResult {
        let client = self.client().await?;
        ack(req::<SetElevonsEndpoint>(
            &client,
            &SetElevonsReq {
                left: r.left,
                right: r.right,
            },
        )
        .await?)
    }

    #[tool(description = "Control mode (pure RPC builds; with rpc-rc the CH6 switch decides).")]
    async fn set_mode(&self, Parameters(r): Parameters<ModeReq>) -> ToolResult {
        let mode = match r.mode {
            Mode::Manual => ControlMode::Manual,
            Mode::Stabilized => ControlMode::Stabilized,
            Mode::AltitudeHold => ControlMode::AltitudeHold,
        };
        let client = self.client().await?;
        ack(req::<SetControlModeEndpoint>(&client, &SetControlModeReq { mode }).await?)
    }

    #[tool(description = "Engage or release heading hold (Stabilized only).")]
    async fn set_heading_hold(&self, Parameters(r): Parameters<HeadingHoldReq>) -> ToolResult {
        let client = self.client().await?;
        // Centidegrees in an i16: ±180°, not 0..360° (327.67° is the i16 limit).
        let wrapped = (r.heading_deg + 180.0).rem_euclid(360.0) - 180.0;
        let heading_cdeg = (wrapped * 100.0).round() as i16;
        ack(req::<SetHeadingHoldEndpoint>(
            &client,
            &SetHeadingHoldReq {
                enabled: r.enabled,
                heading_cdeg,
            },
        )
        .await?)
    }

    #[tool(
        description = "Magnetometer calibration: start (rotate the aircraft through all \
        orientations ~30 s), clear (flash write, disarmed only), get."
    )]
    async fn mag_cal(&self, Parameters(r): Parameters<CalReq>) -> ToolResult {
        let client = self.client().await?;
        match r.action {
            CalAction::Start => ack(req::<StartMagCalEndpoint>(&client, &()).await?),
            CalAction::Clear => ack(req::<ClearMagCalEndpoint>(&client, &()).await?),
            CalAction::Get => to_text(&read_source(&client, Source::MagCal).await?),
        }
    }

    #[tool(
        description = "Level calibration: start (hold the aircraft still at its reference \
        attitude ~3 s), clear (flash write, disarmed only), get."
    )]
    async fn level_cal(&self, Parameters(r): Parameters<CalReq>) -> ToolResult {
        let client = self.client().await?;
        match r.action {
            CalAction::Start => ack(req::<StartLevelCalEndpoint>(&client, &()).await?),
            CalAction::Clear => ack(req::<ClearLevelCalEndpoint>(&client, &()).await?),
            CalAction::Get => to_text(&read_source(&client, Source::LevelCal).await?),
        }
    }

    #[tool(description = "ULog recording (RPC builds): start, stop, info.")]
    async fn ulog(&self, Parameters(r): Parameters<UlogReq>) -> ToolResult {
        let client = self.client().await?;
        match r.action {
            UlogAction::Start => ack(req::<StartULogEndpoint>(&client, &()).await?),
            UlogAction::Stop => ack(req::<StopULogEndpoint>(&client, &()).await?),
            UlogAction::Info => to_text(&read_source(&client, Source::Ulog).await?),
        }
    }

    #[tool(
        description = "Autotune: pitch or roll run (armed, Stabilized; the firmware \
        enforces its envelope), abort, or save/erase the PID gains in flash (disarmed)."
    )]
    async fn autotune(&self, Parameters(r): Parameters<AutotuneReq>) -> ToolResult {
        let client = self.client().await?;
        let start = |axis| StartAutotuneReq {
            axis,
            relay_deg_x10: (r.relay_deg.unwrap_or(5.0) * 10.0).clamp(0.0, 255.0) as u8,
            num_cycles: r.cycles.unwrap_or(6),
            rule: 0,
        };
        match r.action {
            AutotuneAction::Abort => ack(req::<AbortAutotuneEndpoint>(&client, &()).await?),
            AutotuneAction::Pitch => {
                ack(req::<StartAutotuneEndpoint>(&client, &start(AUTOTUNE_AXIS_PITCH)).await?)
            }
            AutotuneAction::Roll => {
                ack(req::<StartAutotuneEndpoint>(&client, &start(AUTOTUNE_AXIS_ROLL)).await?)
            }
            AutotuneAction::SavePid => {
                ack(req::<StartAutotuneEndpoint>(&client, &start(AUTOTUNE_AXIS_SAVE_PID)).await?)
            }
            AutotuneAction::ErasePid => {
                ack(req::<StartAutotuneEndpoint>(&client, &start(AUTOTUNE_AXIS_ERASE_PID)).await?)
            }
        }
    }
    // --- Logs, analysis, results -----------------------------------------------

    #[tool(
        description = "ULog files on the mounted SD card (/run/media/$USER/*/LOG_*.ulg), \
        and whether logs/ already has each. The card must be in the host's reader."
    )]
    async fn card_logs(&self) -> ToolResult {
        use crate::analysis::{card_logs, card_root, logs_dir};
        let files = card_logs(&card_root(), &logs_dir()).map_err(|e| format!("{e:#}"))?;
        to_text(&jv(&files))
    }

    #[tool(
        description = "Copy ULog files from the SD card into logs/ (named, or the newest \
        `last`). Never overwrites a different file of the same name."
    )]
    async fn copy_logs(&self, Parameters(r): Parameters<CopyLogsReq>) -> ToolResult {
        use crate::analysis::{card_logs, card_root, copy_logs, logs_dir};
        tokio::task::spawn_blocking(move || {
            let files = card_logs(&card_root(), &logs_dir())?;
            copy_logs(&files, &logs_dir(), &r.names, r.last)
        })
        .await
        .map_err(|e| e.to_string())?
        .map_err(|e| format!("{e:#}"))
        .and_then(|v| to_text(&v))
    }

    #[tool(
        description = "Run the flight-log analyser (.claude/skills/flight-logs/elle_log.py) on \
        logs: list, summary, timing, esc, sensors, stages, nav, or window (one file, t0_s..t1_s). \
        Returns its text report. See the flight-logs skill for how to read it."
    )]
    async fn analyse_log(&self, Parameters(r): Parameters<AnalyseReq>) -> ToolResult {
        use crate::analysis::{LogCommand, elle_log, resolve_log};
        tokio::task::spawn_blocking(move || {
            let files = r
                .files
                .iter()
                .map(|f| resolve_log(f))
                .collect::<anyhow::Result<Vec<_>>>()?;
            let window = match r.command {
                LogCommand::Window => r.t0_s.zip(r.t1_s),
                _ => None,
            };
            elle_log(r.command, &files, window)
        })
        .await
        .map_err(|e| e.to_string())?
        .map_err(|e| format!("{e:#}"))
    }

    #[tool(
        description = "Replay a raw IMU log (imu-raw-log build) through the firmware's attitude \
        pipeline (elle-replay): coverage, whether the replay reproduces the firmware exactly \
        (`problems` empty), and with `compare` every filter's roll/pitch error in turns against \
        the gyro reference. `simulate` writes and analyses a simulated flight instead. The first \
        call builds elle-replay (a minute)."
    )]
    async fn replay(&self, Parameters(r): Parameters<ReplayReq>) -> ToolResult {
        use crate::analysis::{ReplayOpts, logs_dir, replay, resolve_log};
        tokio::task::spawn_blocking(move || {
            let opts = ReplayOpts {
                compare: r.compare,
                score_by_rate: r.score_by_rate,
                simulate: r.simulate.then_some([
                    r.wind_north,
                    r.wind_east,
                    r.vibration,
                    r.gyro_bias_dps,
                ]),
            };
            let file = if r.simulate {
                // A new file: a bare name goes in logs/.
                let p = std::path::PathBuf::from(&r.file);
                if p.components().count() == 1 {
                    std::fs::create_dir_all(logs_dir())?;
                    logs_dir().join(p)
                } else {
                    p
                }
            } else {
                resolve_log(&r.file)?
            };
            replay(&file, &opts)
        })
        .await
        .map_err(|e| e.to_string())?
        .map_err(|e| format!("{e:#}"))
        .and_then(|v| to_text(&v))
    }

    #[tool(
        description = "Record a TEST_PLAN result (pass, fail, skip, inconclusive) with a note and \
        evidence, in logs/test-runs/<date>.jsonl. The connected firmware's build info is \
        attached. Record what was measured, not only the verdict."
    )]
    async fn test_record(&self, Parameters(r): Parameters<TestRecordReq>) -> ToolResult {
        use crate::analysis::{TestResult, logs_dir, record};
        let build = match self.client().await {
            Ok(c) => read_source(&c, Source::Build).await.ok(),
            Err(_) => None,
        };
        let result = TestResult {
            at: chrono::Local::now().to_rfc3339_opts(chrono::SecondsFormat::Secs, false),
            part: r.part,
            row: r.row,
            outcome: r.outcome,
            note: r.note,
            logs: r.logs,
            build,
        };
        let path = record(&logs_dir(), &result).map_err(|e| format!("{e:#}"))?;
        to_text(&json!({ "recorded": path.display().to_string(), "result": result }))
    }

    #[tool(
        description = "The recorded TEST_PLAN results of one day (default: the newest): the \
        latest outcome per row, and counts."
    )]
    async fn test_report(&self, Parameters(r): Parameters<TestReportReq>) -> ToolResult {
        use crate::analysis::{logs_dir, report};
        report(&logs_dir(), r.date.as_deref())
            .map_err(|e| format!("{e:#}"))
            .and_then(|v| to_text(&v))
    }
}

#[tool_handler(router = self.tool_router)]
impl ServerHandler for ElleMcp {
    fn get_info(&self) -> ServerConfig {
        ServerConfig::new(ServerCapabilities::builder().enable_tools().build())
            .with_server_info(Implementation::new("elle", env!("CARGO_PKG_VERSION")))
            .with_instructions(
                "Elle flight controller over the debug probe. Call `connect` first (RPC builds). \
             Use `read`, `sample`, `wait_for`, `events` and `wait_event` to run TEST_PLAN.md rows: \
             ask the operator for physical actions (tilt, switches, TX off), then verify. \
             Engines are off limits unless link_status says motors_allowed; never spin engines \
             without the operator confirming props are off.",
            )
    }
}

/// Run the server on stdio until the client disconnects; disarm and release
/// the probe on the way out.
pub async fn serve(opts: Options) -> anyhow::Result<()> {
    use rmcp::ServiceExt;
    let server = ElleMcp::new(opts);
    let service = server.clone().serve(rmcp::transport::stdio()).await?;
    tokio::select! {
        r = service.waiting() => { r?; }
        _ = tokio::signal::ctrl_c() => {}
    }
    server.shutdown().await;
    Ok(())
}
