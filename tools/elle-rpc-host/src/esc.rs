//! AM32 ESC configuration through the flight controller (proposal 0002).
//!
//! The firmware only moves bytes between the RPC link and each ESC's
//! bootloader; everything that knows what the bytes mean is here, on the host:
//! the settings decoding (`esc_protocol::am32_config`) and the host policy
//! (`esc_protocol::policy`: edits touch settings bytes 5..48 only, a restore
//! merges 5..192 and refuses another EEPROM layout, the boot and version bytes
//! always come from the ESC). The TUI, `elle direct esc …` and `elle mcp` all
//! go through these functions.
//!
//! The ESC operations only work in an ESC-config boot. [`enter`] and [`exit`]
//! reboot the flight controller; the caller drops its link and reconnects
//! (see [`reconnect`]).

use std::time::{Duration, Instant};

use anyhow::{Context, Result, anyhow, bail};
use elle_rpc_icd::*;
use esc_protocol::am32_config::{Am32Config, Am32ConfigData, Setting, apply_setting};
use esc_protocol::policy;
use postcard_rpc::host_client::HostClient;
use postcard_rpc::standard_icd::WireError;
use serde_json::{Value, json};
use tokio::time::timeout;

use crate::link::Link;

/// One ESC operation: the firmware gives up after `ESC_OP_TIMEOUT_MS` and
/// answers `NoReply`; the host waits a little longer for that answer.
pub const OP_TIMEOUT: Duration = Duration::from_millis(elle_config::ESC_HOST_TIMEOUT_MS);
/// `exit` runs up to three operations per ESC (Run, and for an ESC this boot
/// never connected, Connect and Run again) on both ESCs before it answers.
pub const EXIT_TIMEOUT: Duration =
    Duration::from_millis(6 * elle_config::ESC_HOST_TIMEOUT_MS + 1_000);
/// Plain requests (mode, enter).
const SHORT_TIMEOUT: Duration = Duration::from_secs(2);
/// How long a reboot plus probe attach plus first reply may take.
pub const REATTACH_LIMIT: Duration = Duration::from_secs(30);

// The settings size the firmware's write verify uses is the host policy's.
const _: () = assert!(policy::EEPROM_SIZE == elle_config::ESC_EEPROM_SIZE);

/// The 192-byte settings structure.
pub type Block = [u8; policy::EEPROM_SIZE];

async fn call<E>(
    client: &HostClient<WireError>,
    req: &E::Request,
    limit: Duration,
) -> Result<E::Response>
where
    E: postcard_rpc::Endpoint,
    E::Request: serde::Serialize,
    E::Response: serde::de::DeserializeOwned,
{
    match timeout(limit, client.send_resp::<E>(req)).await {
        Ok(Ok(v)) => Ok(v),
        Ok(Err(e)) => Err(anyhow!("RPC error: {e:?}")),
        Err(_) => Err(anyhow!(
            "no answer within {:.0} s (link lost, or firmware without ESC configuration)",
            limit.as_secs_f64()
        )),
    }
}

fn esc<T>(r: Result<T, EscError>) -> Result<T> {
    r.map_err(|e| anyhow!("ESC: {e}"))
}

/// The engine name, as typed on the command line and shown.
#[must_use]
pub const fn engine_name(e: EscEngine) -> &'static str {
    match e {
        EscEngine::Left => "left",
        EscEngine::Right => "right",
    }
}

/// Parse `left` / `right` (also `l` / `r`).
pub fn parse_engine(s: &str) -> Result<EscEngine> {
    match s.to_ascii_lowercase().as_str() {
        "left" | "l" => Ok(EscEngine::Left),
        "right" | "r" => Ok(EscEngine::Right),
        _ => bail!("engine must be left or right, not {s:?}"),
    }
}

/// This boot's mode. Firmware without the endpoint (older, or a flight build)
/// never answers, which comes back as an `Err` after a short wait.
pub async fn mode(client: &HostClient<WireError>) -> Result<EscBootMode> {
    call::<EscModeEndpoint>(client, &(), SHORT_TIMEOUT).await
}

fn mode_ack(a: AckResp, what: &str) -> Result<()> {
    if a.success {
        return Ok(());
    }
    match a.error_code {
        ESC_MODE_REFUSED_ARMED => bail!("{what} refused: disarm and set throttle 0 first"),
        ESC_MODE_BUSY => bail!("{what} refused: the flight controller is busy, try again"),
        c => bail!("{what} refused (error code {c})"),
    }
}

/// Ask for the ESC-config reboot. On `Ok` the flight controller reboots about
/// 200 ms later; reconnect with [`reconnect`] and check [`mode`].
pub async fn enter(client: &HostClient<WireError>) -> Result<()> {
    mode_ack(
        call::<EscEnterEndpoint>(client, &(), SHORT_TIMEOUT).await?,
        "ESC config",
    )
}

/// Start each ESC's app (Run), then reboot normally. Reconnect afterwards.
pub async fn exit(client: &HostClient<WireError>) -> Result<()> {
    mode_ack(
        call::<EscExitEndpoint>(client, &(), EXIT_TIMEOUT).await?,
        "ESC config exit",
    )
}

/// How long after an accepted [`enter`] / [`exit`] the old firmware may still
/// be running: the reboot comes `ESC_REBOOT_DELAY_MS` after the answer, or up
/// to `ESC_REBOOT_SD_WAIT_MS` later while the ULog file closes (a file still
/// open then cancels the switch). Attaching earlier could catch the old
/// firmware's RTT block on its way down.
pub const REBOOT_WAIT: Duration = Duration::from_millis(
    elle_config::ESC_REBOOT_DELAY_MS + elle_config::ESC_REBOOT_SD_WAIT_MS + 800,
);

/// After [`enter`] / [`exit`]: drop `old`, wait for the reboot, attach again
/// and wait for the firmware's first reply. Blocking; run it off the runtime
/// (`spawn_blocking`) or before one exists.
pub fn reattach_blocking(old: Link) -> Result<Link> {
    old.close();
    std::thread::sleep(REBOOT_WAIT);
    Link::connect_within(Some(REATTACH_LIMIT))
}

/// Wait for the firmware on a fresh link to answer, then read its mode.
pub async fn wait_ready(client: &HostClient<WireError>) -> Result<EscBootMode> {
    let started = Instant::now();
    loop {
        if call::<PingEndpoint>(client, &(), Duration::from_millis(500))
            .await
            .is_ok()
        {
            return mode(client).await;
        }
        if started.elapsed() > REATTACH_LIMIT {
            bail!("the flight controller did not answer after the reboot");
        }
        tokio::time::sleep(Duration::from_millis(200)).await;
    }
}

/// [`reattach_blocking`] then [`wait_ready`], from async code.
pub async fn reconnect(old: Link) -> Result<(Link, EscBootMode)> {
    let link = tokio::task::spawn_blocking(move || reattach_blocking(old))
        .await
        .context("reattach task")??;
    let mode = wait_ready(&link.client).await?;
    Ok((link, mode))
}

/// What a connected ESC reported, plus its firmware file name.
#[derive(Debug, Clone)]
pub struct Connected {
    pub info: EscInfo,
    pub name: String,
}

/// Bootloader handshake, then the firmware file name.
pub async fn connect(client: &HostClient<WireError>, engine: EscEngine) -> Result<Connected> {
    let info = esc(call::<EscConnectEndpoint>(client, &engine, OP_TIMEOUT).await?)?;
    let name = name(client, engine).await.unwrap_or_default();
    Ok(Connected { info, name })
}

/// The 32-byte firmware file name, up to its first NUL or erased byte.
pub async fn name(client: &HostClient<WireError>, engine: EscEngine) -> Result<String> {
    let raw = esc(call::<EscNameEndpoint>(client, &engine, OP_TIMEOUT).await?)?;
    Ok(text(&raw))
}

fn text(raw: &[u8]) -> String {
    let end = raw
        .iter()
        .position(|&b| b == 0x00 || b == 0xFF)
        .unwrap_or(raw.len());
    String::from_utf8_lossy(&raw[..end]).into_owned()
}

/// Read the 192-byte settings structure.
pub async fn read(client: &HostClient<WireError>, engine: EscEngine) -> Result<Block> {
    let data = esc(call::<EscReadEepromEndpoint>(client, &engine, OP_TIMEOUT).await?)?;
    data.as_slice()
        .try_into()
        .map_err(|_| anyhow!("ESC answered {} settings bytes, not 192", data.len()))
}

/// Read 256 bytes at the EEPROM base: the backup format of the esc-config-rs
/// CLI, so backups taken either way compare byte for byte.
pub async fn read_backup(
    client: &HostClient<WireError>,
    engine: EscEngine,
    info: &EscInfo,
) -> Result<Vec<u8>> {
    let req = EscRawReadReq {
        engine,
        read: EscReadReq {
            address: info.layout.eeprom_start,
            len: policy::BACKUP_SIZE as u16,
        },
    };
    let data = esc(call::<EscReadEndpoint>(client, &req, OP_TIMEOUT).await?)?;
    if data.len() != policy::BACKUP_SIZE {
        bail!(
            "ESC answered {} bytes, not {}",
            data.len(),
            policy::BACKUP_SIZE
        );
    }
    Ok(data.to_vec())
}

/// Write the whole settings structure. The firmware reads it back and
/// compares (byte 2 excepted) before it answers.
pub async fn write(client: &HostClient<WireError>, engine: EscEngine, block: &Block) -> Result<()> {
    let data = EscEeprom::from_slice(block).map_err(|_| anyhow!("192 bytes fit an Eeprom"))?;
    let req = EscWriteReq { engine, data };
    esc(call::<EscWriteEepromEndpoint>(client, &req, OP_TIMEOUT).await?)
}

/// Leave the bootloader and start the ESC's app. The app resets back into the
/// bootloader by itself after ~2 s without DShot (this boot sends none), so
/// this is only useful right before an FC reset.
pub async fn run(client: &HostClient<WireError>, engine: EscEngine) -> Result<()> {
    esc(call::<EscRunEndpoint>(client, &engine, OP_TIMEOUT).await?)
}

/// The settings as an `Am32Config` (the 256-byte view, 0xFF past 192).
#[must_use]
pub fn config(block: &Block) -> Am32Config {
    let mut raw = [0xFFu8; policy::BACKUP_SIZE];
    raw[..policy::EEPROM_SIZE].copy_from_slice(block);
    Am32Config::from_bytes(raw)
}

/// The decoded settings as JSON (field names of `Am32ConfigData`, the TOML
/// export format of the esc-config-rs CLI).
#[must_use]
pub fn decoded(block: &Block) -> Value {
    serde_json::to_value(Am32ConfigData::from(&config(block))).unwrap_or(Value::Null)
}

/// One field that differs.
#[derive(Debug, Clone, PartialEq, serde::Serialize)]
pub struct Change {
    pub field: String,
    pub before: Value,
    pub after: Value,
}

/// Field-by-field differences between two settings blocks (decoded), plus
/// the raw bytes that differ where no decoded field shows them: always for
/// the melody and CAN bytes (48..192), and for the settings bytes when no
/// decoded field changed.
#[must_use]
pub fn diff(before: &[u8], after: &[u8]) -> Vec<Change> {
    let pad = |b: &[u8]| {
        let mut block = [0xFFu8; policy::EEPROM_SIZE];
        let n = b.len().min(policy::EEPROM_SIZE);
        block[..n].copy_from_slice(&b[..n]);
        block
    };
    let (a, b) = (pad(before), pad(after));
    let (ja, jb) = (decoded(&a), decoded(&b));
    let mut out = Vec::new();
    if let (Value::Object(ma), Value::Object(mb)) = (&ja, &jb) {
        for (k, va) in ma {
            let vb = mb.get(k).cloned().unwrap_or(Value::Null);
            if *va != vb {
                out.push(Change {
                    field: k.clone(),
                    before: va.clone(),
                    after: vb,
                });
            }
        }
    }
    let from = if out.is_empty() {
        0
    } else {
        policy::SETTINGS.end
    };
    let bytes: Vec<usize> = (from..policy::EEPROM_SIZE)
        .filter(|&i| a[i] != b[i])
        .collect();
    if !bytes.is_empty() {
        let show = |x: &Block| {
            json!(
                bytes
                    .iter()
                    .map(|&i| format!("{i}:{:02X}", x[i]))
                    .collect::<Vec<_>>()
            )
        };
        out.push(Change {
            field: "raw_bytes".to_string(),
            before: show(&a),
            after: show(&b),
        });
    }
    out
}

/// A setting by its CLI name or its decoded (`esc_read`) name; two decoded
/// names carry a unit or kind suffix the CLI names lack.
fn setting(name: &str) -> Option<Setting> {
    Setting::parse(name).or_else(|| {
        ["_khz", "_mode"]
            .iter()
            .find_map(|suffix| name.strip_suffix(suffix))
            .and_then(Setting::parse)
    })
}

/// Apply `name = value` edits to `current` under the host policy: only
/// settings bytes 5..48 change. Errors name the first bad field or value.
pub fn edit(current: &Block, edits: &[(String, String)]) -> Result<Block> {
    if edits.is_empty() {
        bail!("no settings given (e.g. beep_volume=5)");
    }
    let mut cfg = config(current);
    for (name, value) in edits {
        let setting = setting(name).ok_or_else(|| {
            anyhow!(
                "unknown setting {name:?}; one of: {}",
                Setting::all_names().join(", ")
            )
        })?;
        apply_setting(&mut cfg, setting, value).map_err(|e| anyhow!("{name} = {value}: {e}"))?;
    }
    let mut edited: Block = [0; policy::EEPROM_SIZE];
    edited.copy_from_slice(&cfg.as_bytes()[..policy::EEPROM_SIZE]);
    let mut out = *current;
    policy::merge_settings(&mut out, &edited);
    Ok(out)
}

/// Parse `name=value` pairs.
pub fn parse_edits(args: &[String]) -> Result<Vec<(String, String)>> {
    args.iter()
        .map(|a| {
            a.split_once('=')
                .map(|(k, v)| (k.trim().to_string(), v.trim().to_string()))
                .ok_or_else(|| anyhow!("expected name=value, not {a:?}"))
        })
        .collect()
}

/// Read, edit, write (verified by the firmware), read back; the changes.
pub async fn set(
    client: &HostClient<WireError>,
    engine: EscEngine,
    edits: &[(String, String)],
) -> Result<Vec<Change>> {
    let current = read(client, engine).await?;
    let new = edit(&current, edits)?;
    if new == current {
        return Ok(Vec::new());
    }
    write(client, engine, &new).await?;
    let back = read(client, engine).await?;
    Ok(diff(&current, &back))
}

/// Restore a backup (256 or 192 bytes) under the host policy; the changes.
pub async fn restore(
    client: &HostClient<WireError>,
    engine: EscEngine,
    backup: &[u8],
) -> Result<Vec<Change>> {
    let current = read(client, engine).await?;
    let mut new = current;
    policy::merge_restore(&mut new, backup).map_err(|e| anyhow!("{e}"))?;
    if new == current {
        return Ok(Vec::new());
    }
    write(client, engine, &new).await?;
    let back = read(client, engine).await?;
    Ok(diff(&current, &back))
}

/// `EscInfo` and file name as JSON.
#[must_use]
pub fn info_json(c: &Connected) -> Value {
    let i = &c.info;
    json!({
        "device_id": format!("0x{:04X}", i.device_id),
        "pin_code": format!("0x{:02X}", i.pin_code),
        "flash_code": format!("0x{:02X}", i.flash_code),
        "protocol_version": i.protocol_version,
        "eeprom_start": format!("0x{:04X}", i.layout.eeprom_start),
        "firmware_start": format!("0x{:04X}", i.layout.firmware_start),
        "max_firmware_size": i.max_firmware_size,
        "firmware_name": c.name,
    })
}

/// Write a backup file, never over an existing one (a later backup to the
/// same path would destroy the restore source).
pub fn save_backup(path: &std::path::Path, data: &[u8]) -> Result<()> {
    use std::io::Write;
    if let Some(dir) = path.parent().filter(|d| !d.as_os_str().is_empty()) {
        std::fs::create_dir_all(dir)?;
    }
    let mut f = std::fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(path)
        .with_context(|| {
            format!(
                "{} (an existing backup is never overwritten)",
                path.display()
            )
        })?;
    f.write_all(data)?;
    Ok(())
}

/// Where backups go by default: `logs/esc/`, next to the flight logs.
#[must_use]
pub fn backup_dir() -> std::path::PathBuf {
    crate::analysis::logs_dir().join("esc")
}

/// A default backup file name: `<engine>-<device id>-<local time>.bin`.
#[must_use]
pub fn backup_name(engine: EscEngine, info: &EscInfo) -> String {
    format!(
        "{}-{:04X}-{}.bin",
        engine_name(engine),
        info.device_id,
        chrono::Local::now().format("%Y%m%d-%H%M%S")
    )
}

#[cfg(test)]
mod tests {
    use super::*;

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

    fn block() -> Block {
        let mut b = [0u8; policy::EEPROM_SIZE];
        b[0] = 0x01;
        b[1] = Am32Config::CURRENT_EEPROM_VERSION;
        b[2] = 7; // bootloader version
        b[3] = 2;
        b[4] = 19;
        b
    }

    #[test]
    fn edits_touch_settings_bytes_only() {
        let mut cur = block();
        cur[100] = 0xAB; // melody
        let new = edit(&cur, &[("beep_volume".into(), "5".into())]).unwrap();
        assert_eq!(&new[..5], &cur[..5]);
        assert_eq!(&new[48..], &cur[48..]);
        let d = diff(&cur, &new);
        assert_eq!(d.len(), 1, "{d:?}");
        assert_eq!(d[0].field, "beep_volume");
    }

    #[test]
    fn bad_names_and_values_are_refused() {
        let cur = block();
        let e = edit(&cur, &[("nope".into(), "1".into())]).unwrap_err();
        assert!(e.to_string().contains("unknown setting"), "{e}");
        assert!(edit(&cur, &[]).is_err());
        assert!(parse_edits(&["beep_volume".into()]).is_err());
        assert_eq!(
            parse_edits(&["beep_volume = 5".into()]).unwrap(),
            vec![("beep_volume".to_string(), "5".to_string())]
        );
    }

    #[test]
    fn raw_differences_show_when_nothing_decoded_changed() {
        let a = block();
        let mut b = a;
        b[150] = 0x42;
        let d = diff(&a, &b);
        assert!(
            d.iter()
                .any(|c| c.field == "raw_bytes" || c.field.contains("melody")),
            "{d:?}"
        );
    }

    #[test]
    fn melody_changes_show_next_to_a_setting_change() {
        let a = block();
        let mut b = edit(&a, &[("beep_volume".into(), "5".into())]).unwrap();
        b[100] = 0x42;
        let d = diff(&a, &b);
        assert!(d.iter().any(|c| c.field == "beep_volume"), "{d:?}");
        assert!(d.iter().any(|c| c.field == "raw_bytes"), "{d:?}");
    }

    #[test]
    fn decoded_names_are_setting_names() {
        // `esc_set` takes the names `esc_read` shows.
        let Value::Object(m) = decoded(&block()) else {
            panic!("decoded settings are an object");
        };
        let meta = ["firmware_version", "bootloader_version", "eeprom_version"];
        let bad: Vec<&String> = m
            .keys()
            .filter(|k| !meta.contains(&k.as_str()) && setting(k).is_none())
            .collect();
        assert!(bad.is_empty(), "not settable by name: {bad:?}");
    }

    #[test]
    fn decoded_values_round_trip_through_edit() {
        // Feeding a decoded value back through `edit` must not change the
        // byte: `esc_set` takes the values `esc_read` shows.
        let mut cfg = Am32Config::with_defaults();
        cfg.as_bytes_mut()[0] = 0x01;
        cfg.as_bytes_mut()[1] = Am32Config::CURRENT_EEPROM_VERSION;
        let mut block: Block = [0; policy::EEPROM_SIZE];
        block.copy_from_slice(&cfg.as_bytes()[..policy::EEPROM_SIZE]);
        let Value::Object(m) = decoded(&block) else {
            panic!("decoded settings are an object");
        };
        let meta = ["firmware_version", "bootloader_version", "eeprom_version"];
        let (mut changed, mut rejected, mut accepted) = (Vec::new(), Vec::new(), 0);
        for (k, v) in &m {
            if meta.contains(&k.as_str()) {
                continue;
            }
            let value = match v {
                Value::String(s) => s.clone(),
                other => other.to_string(),
            };
            match edit(&block, &[(k.clone(), value.clone())]) {
                Ok(new) if new != block => changed.push(format!("{k}={value}")),
                Ok(_) => accepted += 1,
                Err(_) => rejected.push(format!("{k}={value}")),
            }
        }
        assert!(
            changed.is_empty(),
            "values that change the byte: {changed:?}"
        );
        // Enum fields show their variant name, which `apply_setting` does not
        // take (it wants the CLI's syntax): refused, never written wrong.
        assert!(rejected.len() <= 4, "rejected: {rejected:?}");
        assert!(
            accepted >= 30,
            "accepted only {accepted}; rejected {rejected:?}"
        );
    }

    #[test]
    fn backups_are_never_overwritten() {
        let dir = TempDir::new("esc-bak");
        let path = dir.0.join("x.bin");
        save_backup(&path, &[1, 2, 3]).unwrap();
        assert!(save_backup(&path, &[4]).is_err());
        assert_eq!(std::fs::read(&path).unwrap(), vec![1, 2, 3]);
    }

    #[test]
    fn engine_names_round_trip() {
        for e in [EscEngine::Left, EscEngine::Right] {
            assert_eq!(parse_engine(engine_name(e)).unwrap(), e);
        }
        assert!(parse_engine("middle").is_err());
    }
}
