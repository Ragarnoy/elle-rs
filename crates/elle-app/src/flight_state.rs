use elle_hardware::SignalCache;
use elle_rpc_icd::ControlMode;

#[derive(Debug, Clone, Copy)]
pub(crate) struct FlightState {
    pub(crate) armed: bool,
    pub(crate) failsafe: bool,
    pub(crate) mode: ControlMode,
    pub(crate) rc_age_ms: u16,
    /// Autotune state: 0=off, 1=pitch, 2=roll, 3=done, 4=error
    pub(crate) autotune_state: u8,
}

pub(crate) static FLIGHT_STATE: SignalCache<FlightState> = SignalCache::new(FlightState {
    armed: false,
    failsafe: false,
    mode: ControlMode::Manual,
    rc_age_ms: 0,
    autotune_state: 0,
});

/// Controller output snapshot published from the main loop for RPC observability
#[derive(Debug, Clone, Copy, Default)]
pub(crate) struct ControllerOutput {
    pub(crate) pitch_correction: f32,
    pub(crate) roll_correction: f32,
    pub(crate) pitch_setpoint_deg: f32,
    pub(crate) roll_setpoint_deg: f32,
    pub(crate) elevon_left_us: u32,
    pub(crate) elevon_right_us: u32,
    pub(crate) engine_left_dshot: u16,
    pub(crate) engine_right_dshot: u16,
    pub(crate) heading_hold_active: bool,
    pub(crate) heading_target_deg: f32,
    pub(crate) heading_error_deg: f32,
}

pub(crate) static CONTROLLER_OUTPUT: SignalCache<ControllerOutput> =
    SignalCache::new(ControllerOutput {
        pitch_correction: 0.0,
        roll_correction: 0.0,
        pitch_setpoint_deg: 0.0,
        roll_setpoint_deg: 0.0,
        elevon_left_us: 0,
        elevon_right_us: 0,
        engine_left_dshot: 0,
        engine_right_dshot: 0,
        heading_hold_active: false,
        heading_target_deg: 0.0,
        heading_error_deg: 0.0,
    });
