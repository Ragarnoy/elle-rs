use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;

/// Commands sent from RPC handlers to flight controller
#[derive(Debug, Clone, Copy)]
pub enum RpcCommand {
    SetThrottle(u8),
    SetElevons { left: i8, right: i8 },
    SetMode(super::ControlMode),
    Arm,
    Disarm,
    EmergencyStop,
    AdjustTrim { left: i8, right: i8 },
    SaveCalibration,
    ClearCalibration,
}

/// Channel for RPC commands to flight controller
pub static RPC_CMD_CHANNEL: Channel<CriticalSectionRawMutex, RpcCommand, 16> = Channel::new();
