//! RPC handlers for ESC configuration (proposal 0002).
//!
//! The mode switch goes to the control loop like every other command
//! (`RpcCommand::SwitchBootMode`): the loop owns the arming state, the engine
//! targets and the watchdog, and makes the final decision. The ESC operations
//! run on `esc_config_task` (`elle_hardware::esc_config`), one whole operation
//! per request; these handlers are `async` and hold the RPC server until the
//! answer comes back (≤ `ESC_OP_TIMEOUT_MS`).

use core::sync::atomic::Ordering;

use elle_config::{ESC_HOST_STAMP_MS, ESC_OP_TIMEOUT_MS};
use elle_hardware::boot_mode::{self, BootMode};
use elle_hardware::esc_config::{self, Engine, EscOp, EscReply, EscRequest};
use elle_rpc_icd::{
    AckResp, ESC_MODE_BUSY, ESC_MODE_REFUSED_ARMED, EscAckResp, EscBlockResp, EscBootMode,
    EscConnectResp, EscEepromResp, EscEngine, EscError, EscRawReadReq, EscWriteReq,
};
use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Instant, Ticker};
use postcard_rpc::header::VarHeader;

use crate::rpc_app::RpcContext;
use crate::rpc_handlers::RpcCommand;

const fn engine(e: EscEngine) -> Engine {
    match e {
        EscEngine::Left => Engine::Left,
        EscEngine::Right => Engine::Right,
    }
}

/// Stamp the host link as alive. The host is waiting for our answer, so the RPC
/// failsafe must not age it out while the server is busy with an operation.
fn stamp_host() {
    elle_system::rpc::HOST_LAST_RX_MS.store(Instant::now().as_millis() as u32, Ordering::Relaxed);
}

/// Run one ESC operation and wait for its reply. `None` in a normal boot, on a
/// timeout, or if the task is busy.
async fn run(engine: EscEngine, op: EscOp) -> Option<EscReply> {
    if !boot_mode::is_esc_config() {
        return None;
    }
    let req = EscRequest {
        engine: self::engine(engine),
        op,
    };
    let answer = esc_config::request(req, Duration::from_millis(ESC_OP_TIMEOUT_MS));
    let mut stamp = Ticker::every(Duration::from_millis(ESC_HOST_STAMP_MS));
    let mut answer = core::pin::pin!(answer);
    loop {
        match select(answer.as_mut(), stamp.next()).await {
            Either::First(reply) => {
                stamp_host();
                return reply;
            }
            Either::Second(()) => stamp_host(),
        }
    }
}

/// The error for "no reply": not an ESC-config boot, or no answer in time.
fn no_reply() -> EscError {
    if boot_mode::is_esc_config() {
        EscError::NoReply
    } else {
        EscError::NotConnected
    }
}

pub(crate) fn handle_esc_mode(_ctx: &mut RpcContext, _hdr: VarHeader, _req: ()) -> EscBootMode {
    match boot_mode::current() {
        BootMode::Normal => EscBootMode::Normal,
        BootMode::EscConfig => EscBootMode::EscConfig,
    }
}

pub(crate) fn handle_esc_enter(ctx: &mut RpcContext, _hdr: VarHeader, _req: ()) -> AckResp {
    if boot_mode::is_esc_config() {
        return AckResp::ok();
    }
    // The loop checks again (engine targets too) before it reboots; refusing
    // here as well gives the host an answer it can show.
    if crate::flight_state::FLIGHT_STATE.read_cached().armed {
        elle_hardware::elle_event!(
            warn,
            elle_hardware::event::EVT_ESC_CONFIG_REFUSED,
            "ESC config refused: armed"
        );
        return AckResp::error(ESC_MODE_REFUSED_ARMED);
    }
    match ctx
        .cmd_sender
        .try_send(RpcCommand::SwitchBootMode(BootMode::EscConfig))
    {
        Ok(()) => AckResp::ok(),
        Err(_) => AckResp::error(ESC_MODE_BUSY),
    }
}

/// Leave ESC-config mode: start each ESC's app (Run; connecting it first if
/// this boot never did), then reboot normally.
pub(crate) async fn handle_esc_exit(ctx: &mut RpcContext, _hdr: VarHeader, _req: ()) -> AckResp {
    if !boot_mode::is_esc_config() {
        return AckResp::ok();
    }
    for e in [EscEngine::Left, EscEngine::Right] {
        // An ESC not connected this boot may still wait in its bootloader:
        // connect it first, so it gets Run too. A failed connect (no such
        // engine, or an ESC already in its app) is fine.
        if let Some(EscReply::Ack(Err(EscError::NotConnected))) = run(e, EscOp::Run).await
            && let Some(EscReply::Connect(Ok(_))) = run(e, EscOp::Connect).await
        {
            let _ = run(e, EscOp::Run).await;
        }
    }
    match ctx
        .cmd_sender
        .try_send(RpcCommand::SwitchBootMode(BootMode::Normal))
    {
        Ok(()) => AckResp::ok(),
        Err(_) => AckResp::error(ESC_MODE_BUSY),
    }
}

pub(crate) async fn handle_esc_connect(
    _ctx: &mut RpcContext,
    _hdr: VarHeader,
    req: EscEngine,
) -> EscConnectResp {
    match run(req, EscOp::Connect).await {
        Some(EscReply::Connect(r)) => r,
        _ => Err(no_reply()),
    }
}

pub(crate) async fn handle_esc_read_eeprom(
    _ctx: &mut RpcContext,
    _hdr: VarHeader,
    req: EscEngine,
) -> EscEepromResp {
    match run(req, EscOp::ReadEeprom).await {
        Some(EscReply::Eeprom(r)) => r,
        _ => Err(no_reply()),
    }
}

pub(crate) async fn handle_esc_write_eeprom(
    _ctx: &mut RpcContext,
    _hdr: VarHeader,
    req: EscWriteReq,
) -> EscAckResp {
    match run(req.engine, EscOp::WriteEeprom(req.data)).await {
        Some(EscReply::Ack(r)) => r,
        _ => Err(no_reply()),
    }
}

pub(crate) async fn handle_esc_name(
    _ctx: &mut RpcContext,
    _hdr: VarHeader,
    req: EscEngine,
) -> EscBlockResp {
    match run(req, EscOp::ReadName).await {
        Some(EscReply::Block(r)) => r,
        _ => Err(no_reply()),
    }
}

pub(crate) async fn handle_esc_read(
    _ctx: &mut RpcContext,
    _hdr: VarHeader,
    req: EscRawReadReq,
) -> EscBlockResp {
    match run(req.engine, EscOp::Read(req.read)).await {
        Some(EscReply::Block(r)) => r,
        _ => Err(no_reply()),
    }
}

pub(crate) async fn handle_esc_run(
    _ctx: &mut RpcContext,
    _hdr: VarHeader,
    req: EscEngine,
) -> EscAckResp {
    match run(req, EscOp::Run).await {
        Some(EscReply::Ack(r)) => r,
        _ => Err(no_reply()),
    }
}
