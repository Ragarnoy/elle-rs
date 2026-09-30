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
/// How long `connect` waits for the firmware's RTT control block.
const CONNECT_LIMIT: Duration = Duration::from_secs(10);

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
        if let Some(c) = self.inner.conn.lock().await.take() {
            c.events.abort();
            c.client.close();
            if let Some(l) = c.link {
                let _ = tokio::task::spawn_blocking(move || l.close()).await;
            }
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
        rpc-control or rpc-control,rpc-rc). Waits up to 10 s for the firmware's RTT block. \
        No-op when already connected. The probe has one owner: the TUI cannot run at the same time."
    )]
    async fn connect(&self) -> ToolResult {
        if self.inner.conn.lock().await.is_some() {
            return self.link_status().await;
        }
        let link = tokio::task::spawn_blocking(|| Link::connect_within(Some(CONNECT_LIMIT)))
            .await
            .map_err(|e| e.to_string())?
            .map_err(|e| format!("could not connect: {e:#}"))?;
        let client = link.client.clone();
        self.attach(client, Some(link)).await?;
        self.link_status().await
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
        let mut j = json!({
            "connected": connected,
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
        let heading_cdeg = (r.heading_deg.rem_euclid(360.0) * 100.0) as i16;
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
