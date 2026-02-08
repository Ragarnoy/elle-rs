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
