//! postcard-RPC dispatch using define_dispatch! macro
//!
//! Replaces the hand-rolled dispatch_rpc_request() with macro-generated dispatch.

use defmt::info;
use elle_rpc_icd::*;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Sender;
use postcard_rpc::header::VarHeader;

use crate::rpc_handlers::RpcCommand;

/// Context passed to all RPC handlers
pub struct RpcContext {
    pub cmd_sender: Sender<'static, CriticalSectionRawMutex, RpcCommand, 16>,
}

// ---------------------------------------------------------------------------
// Handlers
// ---------------------------------------------------------------------------

fn handle_set_throttle(
    ctx: &mut RpcContext,
    _hdr: VarHeader,
    req: SetThrottleReq,
) -> AckResp {
    let _ = ctx.cmd_sender.try_send(RpcCommand::SetThrottle(req.percent));
    AckResp::ok()
}

fn handle_set_elevons(
    ctx: &mut RpcContext,
    _hdr: VarHeader,
    req: SetElevonsReq,
) -> AckResp {
    let _ = ctx.cmd_sender.try_send(RpcCommand::SetElevons {
        left: req.left,
        right: req.right,
    });
    AckResp::ok()
}

fn handle_set_control_mode(
    ctx: &mut RpcContext,
    _hdr: VarHeader,
    req: SetControlModeReq,
) -> AckResp {
    let _ = ctx.cmd_sender.try_send(RpcCommand::SetMode(req.mode));
    AckResp::ok()
}

fn handle_arm(ctx: &mut RpcContext, _hdr: VarHeader, _req: ()) -> AckResp {
    let _ = ctx.cmd_sender.try_send(RpcCommand::Arm);
    AckResp::ok()
}

fn handle_disarm(ctx: &mut RpcContext, _hdr: VarHeader, _req: ()) -> AckResp {
    let _ = ctx.cmd_sender.try_send(RpcCommand::Disarm);
    AckResp::ok()
}

fn handle_emergency_stop(ctx: &mut RpcContext, _hdr: VarHeader, _req: ()) -> AckResp {
    let _ = ctx.cmd_sender.try_send(RpcCommand::EmergencyStop);
    AckResp::ok()
}

fn handle_adjust_trim(
    ctx: &mut RpcContext,
    _hdr: VarHeader,
    req: AdjustTrimReq,
) -> AckResp {
    let _ = ctx.cmd_sender.try_send(RpcCommand::AdjustTrim {
        left: req.left,
        right: req.right,
    });
    AckResp::ok()
}

fn handle_save_calibration(ctx: &mut RpcContext, _hdr: VarHeader, _req: ()) -> AckResp {
    let _ = ctx.cmd_sender.try_send(RpcCommand::SaveCalibration);
    AckResp::ok()
}

fn handle_clear_calibration(ctx: &mut RpcContext, _hdr: VarHeader, _req: ()) -> AckResp {
    let _ = ctx.cmd_sender.try_send(RpcCommand::ClearCalibration);
    AckResp::ok()
}

fn handle_get_status(_ctx: &mut RpcContext, _hdr: VarHeader, _req: ()) -> StatusResp {
    let imu_status = elle_hardware::imu::IMU_STATUS.try_read();
    let state = crate::flight_state::FLIGHT_STATE.try_take();
    if let Some(s) = state {
        crate::flight_state::FLIGHT_STATE.signal(s); // put back
    }
    StatusResp {
        armed: state.map(|s| s.armed).unwrap_or(false),
        failsafe: state.map(|s| s.failsafe).unwrap_or(false),
        mode: state.map(|s| s.mode).unwrap_or(ControlMode::Manual),
        imu_calibrated: imu_status.as_ref().map(|s| s.calibrated).unwrap_or(false),
        imu_error_count: imu_status.as_ref().map(|s| s.error_count).unwrap_or(0),
    }
}

fn handle_get_attitude(_ctx: &mut RpcContext, _hdr: VarHeader, _req: ()) -> AttitudeResp {
    if let Some(att) = elle_hardware::imu::ATTITUDE_SIGNAL.try_take() {
        let resp = AttitudeResp {
            pitch_cdeg: (att.pitch * 5729.578) as i16,
            roll_cdeg: (att.roll * 5729.578) as i16,
            yaw_cdeg: (att.yaw * 5729.578) as i16,
            pitch_rate_cdeg: (att.pitch_rate * 5729.578) as i16,
            roll_rate_cdeg: (att.roll_rate * 5729.578) as i16,
            yaw_rate_cdeg: (att.yaw_rate * 5729.578) as i16,
        };
        // Put attitude back for main loop
        elle_hardware::imu::ATTITUDE_SIGNAL.signal(att);
        resp
    } else {
        AttitudeResp {
            pitch_cdeg: 0,
            roll_cdeg: 0,
            yaw_cdeg: 0,
            pitch_rate_cdeg: 0,
            roll_rate_cdeg: 0,
            yaw_rate_cdeg: 0,
        }
    }
}

fn handle_get_performance(_ctx: &mut RpcContext, _hdr: VarHeader, _req: ()) -> PerformanceResp {
    #[cfg(feature = "performance-monitoring")]
    {
        let pm = unsafe { &*core::ptr::addr_of!(elle_system::PERFORMANCE_MONITOR) };
        PerformanceResp {
            control_loop_avg_us: pm.control_loop.avg_us,
            control_loop_max_us: pm.control_loop.max_us,
            imu_avg_us: pm.imu_update.avg_us,
            imu_max_us: pm.imu_update.max_us,
        }
    }
    #[cfg(not(feature = "performance-monitoring"))]
    {
        PerformanceResp {
            control_loop_avg_us: 0,
            control_loop_max_us: 0,
            imu_avg_us: 0,
            imu_max_us: 0,
        }
    }
}

fn handle_reset_performance(_ctx: &mut RpcContext, _hdr: VarHeader, _req: ()) -> AckResp {
    info!("RPC: Reset performance counters");
    #[cfg(feature = "performance-monitoring")]
    unsafe {
        (*core::ptr::addr_of_mut!(elle_system::PERFORMANCE_MONITOR)).reset_all();
    }
    AckResp::ok()
}

fn handle_ping(_ctx: &mut RpcContext, _hdr: VarHeader, _req: ()) {
    // Echo - nothing to do for () -> ()
}

fn handle_get_version(_ctx: &mut RpcContext, _hdr: VarHeader, _req: ()) -> VersionResp {
    VersionResp {
        major: 0,
        minor: 1,
        patch: 0,
    }
}

fn handle_get_magnetometer(_ctx: &mut RpcContext, _hdr: VarHeader, _req: ()) -> MagnetometerResp {
    if let Some(mag) = crate::mag_signal::MAG_SIGNAL.try_take() {
        crate::mag_signal::MAG_SIGNAL.signal(mag);
        mag
    } else {
        MagnetometerResp { x: 0, y: 0, z: 0 }
    }
}

fn handle_get_gnss(_ctx: &mut RpcContext, _hdr: VarHeader, _req: ()) -> GnssResp {
    #[cfg(feature = "gnss")]
    {
        if let Some(gnss) = crate::gnss_signal::GNSS_SIGNAL.try_take() {
            crate::gnss_signal::GNSS_SIGNAL.signal(gnss);
            return gnss;
        }
    }
    GnssResp {
        latitude: 0.0,
        longitude: 0.0,
        altitude_m: 0.0,
        fix_quality: 0,
        num_satellites: 0,
        hdop: 99.9,
    }
}

// ---------------------------------------------------------------------------
// Dispatch table
// ---------------------------------------------------------------------------

#[allow(unused_imports)]
use elle_system::rpc::{ElleWireSpawn, RttTx, elle_spawn};

postcard_rpc::define_dispatch! {
    app: ElleApp;

    spawn_fn: elle_spawn;
    tx_impl: RttTx;
    spawn_impl: ElleWireSpawn;
    context: RpcContext;

    endpoints: {
        list: ENDPOINT_LIST;

        | EndpointTy                | kind      | handler                   |
        | ----------                | ----      | -------                   |
        | SetThrottleEndpoint       | blocking  | handle_set_throttle       |
        | SetElevonsEndpoint        | blocking  | handle_set_elevons        |
        | SetControlModeEndpoint    | blocking  | handle_set_control_mode   |
        | ArmEndpoint               | blocking  | handle_arm                |
        | DisarmEndpoint            | blocking  | handle_disarm             |
        | EmergencyStopEndpoint     | blocking  | handle_emergency_stop     |
        | AdjustTrimEndpoint        | blocking  | handle_adjust_trim        |
        | SaveCalibrationEndpoint   | blocking  | handle_save_calibration   |
        | ClearCalibrationEndpoint  | blocking  | handle_clear_calibration  |
        | GetStatusEndpoint         | blocking  | handle_get_status         |
        | GetAttitudeEndpoint       | blocking  | handle_get_attitude       |
        | GetPerformanceEndpoint    | blocking  | handle_get_performance    |
        | ResetPerformanceEndpoint  | blocking  | handle_reset_performance  |
        | PingEndpoint              | blocking  | handle_ping               |
        | GetVersionEndpoint        | blocking  | handle_get_version        |
        | GetMagnetometerEndpoint   | blocking  | handle_get_magnetometer   |
        | GetGnssEndpoint           | blocking  | handle_get_gnss           |
    };
    topics_in: {
        list: TOPICS_IN_LIST;

        | TopicTy           | kind      | handler               |
        | ----------        | ----      | -------               |
    };
    topics_out: {
        list: TOPICS_OUT_LIST;
    };
}
