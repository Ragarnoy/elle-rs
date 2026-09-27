use elle_control::PidConfig;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;

/// Commands sent from RPC handlers to flight controller
#[derive(Debug, Clone, Copy)]
// Under `rpc-rc` the RC link is the pilot source, so the loop ignores the
// payloads of SetThrottle/SetElevons/SetMode.
#[cfg_attr(feature = "rpc-rc", allow(dead_code))]
pub(crate) enum RpcCommand {
    SetThrottle(u8),
    SetElevons {
        left: i8,
        right: i8,
    },
    SetMode(elle_rpc_icd::ControlMode),
    Arm,
    Disarm,
    EmergencyStop,
    SetPidGains {
        config: PidConfig,
    },
    StartULog,
    StopULog,
    ReadULogChunk,
    PopAndPeekULog,
    EraseULog,
    StartAutotune {
        axis: u8,
        relay_deg_x10: u8,
        num_cycles: u8,
        rule: u8,
    },
    AbortAutotune,
    StartMagCal,
    ClearMagCal,
    StartLevelCal,
    ClearLevelCal,
    SetHeadingHold {
        enabled: bool,
        target_cdeg: i16,
    },
}

/// Channel for RPC commands to flight controller
pub(crate) static RPC_CMD_CHANNEL: Channel<CriticalSectionRawMutex, RpcCommand, 16> =
    Channel::new();
