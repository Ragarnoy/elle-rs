use elle_control::PidConfig;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;

/// Commands sent from RPC handlers to flight controller
#[derive(Debug, Clone, Copy)]
#[allow(dead_code)] // Some variant fields unused when `rc` feature ignores RPC flight commands
pub enum RpcCommand {
    SetThrottle(u8),
    SetElevons {
        left: i8,
        right: i8,
    },
    SetMode(super::ControlMode),
    Arm,
    Disarm,
    EmergencyStop,
    SetPidGains { config: PidConfig },
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
    #[allow(dead_code)] // pre-wired for savepid TUI command
    SavePidProfile {
        data: [u8; 32],
    },
    ClearPidProfile,
    StartMagCal,
    ClearMagCal,
    SetHeadingHold {
        enabled: bool,
        target_cdeg: i16,
    },
}

/// Channel for RPC commands to flight controller
pub static RPC_CMD_CHANNEL: Channel<CriticalSectionRawMutex, RpcCommand, 16> = Channel::new();
