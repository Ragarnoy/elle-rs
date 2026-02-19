use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use elle_rpc_icd::ControlMode;

#[derive(Debug, Clone, Copy)]
pub struct FlightState {
    pub armed: bool,
    pub failsafe: bool,
    pub mode: ControlMode,
}

pub static FLIGHT_STATE: Signal<CriticalSectionRawMutex, FlightState> = Signal::new();

/// Controller output snapshot published from the main loop for RPC observability
#[derive(Debug, Clone, Copy, Default)]
pub struct ControllerOutput {
    pub pitch_correction: f32,
    pub roll_correction: f32,
    pub pitch_setpoint_deg: f32,
    pub roll_setpoint_deg: f32,
    pub elevon_left_us: u32,
    pub elevon_right_us: u32,
    pub engine_left_us: u32,
    pub engine_right_us: u32,
}

pub static CONTROLLER_OUTPUT: Signal<CriticalSectionRawMutex, ControllerOutput> = Signal::new();
