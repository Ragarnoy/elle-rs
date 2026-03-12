use elle_hardware::SignalCache;
use elle_rpc_icd::ControlMode;

#[derive(Debug, Clone, Copy)]
pub struct FlightState {
    pub armed: bool,
    pub failsafe: bool,
    pub mode: ControlMode,
}

pub static FLIGHT_STATE: SignalCache<FlightState> = SignalCache::new(FlightState {
    armed: false,
    failsafe: false,
    mode: ControlMode::Manual,
});

/// Controller output snapshot published from the main loop for RPC observability
#[derive(Debug, Clone, Copy, Default)]
pub struct ControllerOutput {
    pub pitch_correction: f32,
    pub roll_correction: f32,
    pub pitch_setpoint_deg: f32,
    pub roll_setpoint_deg: f32,
    pub elevon_left_us: u32,
    pub elevon_right_us: u32,
    pub engine_left_dshot: u16,
    pub engine_right_dshot: u16,
}

pub static CONTROLLER_OUTPUT: SignalCache<ControllerOutput> = SignalCache::new(ControllerOutput {
    pitch_correction: 0.0,
    roll_correction: 0.0,
    pitch_setpoint_deg: 0.0,
    roll_setpoint_deg: 0.0,
    elevon_left_us: 0,
    elevon_right_us: 0,
    engine_left_dshot: 0,
    engine_right_dshot: 0,
});
