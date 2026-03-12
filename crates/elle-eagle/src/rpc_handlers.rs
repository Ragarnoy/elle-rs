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
    SetPidGains {
        pitch_kp: f32,
        pitch_ki: f32,
        pitch_kd: f32,
        roll_kp: f32,
        roll_ki: f32,
        roll_kd: f32,
        scale: f32,
        i_limit: f32,
    },
    SetAttitudeSetpoint {
        pitch_deg: f32,
        roll_deg: f32,
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
    #[allow(dead_code)] // pre-wired for savepid TUI command
    SavePidProfile {
        data: [u8; 32],
    },
    StartMagCal,
    ClearMagCal,
}

/// Channel for RPC commands to flight controller
pub static RPC_CMD_CHANNEL: Channel<CriticalSectionRawMutex, RpcCommand, 16> = Channel::new();
