//! postcard-RPC dispatch using define_dispatch! macro
//!
//! Replaces the hand-rolled dispatch_rpc_request() with macro-generated dispatch.

use core::cell::Cell;
use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU16, Ordering};
use defmt::info;
use elle_config::profile::ULOG_CHUNK_SIZE;
use elle_rpc_icd::*;
use embassy_rp::aon_timer::AonTimer;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Sender;
use embassy_sync::signal::Signal;
use postcard_rpc::header::VarHeader;

use crate::rpc_handlers::RpcCommand;

/// Radians to centidegrees: (180/PI) * 100
const RAD_TO_CDEG: f32 = 5729.578;

// ULog recording enable flag (checked by main loop before logging)
pub static ULOG_ENABLED: AtomicBool = AtomicBool::new(false);

// ULog transfer state machine
#[repr(u8)]
pub enum ULogState {
    Idle = 0,
    Reading = 1,
    Ready = 2,
    Empty = 3,
}

/// ULog transfer state (atomic for cross-context access)
pub static ULOG_STATE: AtomicU8 = AtomicU8::new(ULogState::Idle as u8);

/// Full queue item buffer (up to ULOG_CHUNK_SIZE bytes) from flash peek
pub static ULOG_ITEM_SIGNAL: Signal<CriticalSectionRawMutex, ([u8; ULOG_CHUNK_SIZE], usize)> =
    Signal::new();

/// Fragment tracking: offset within current item
pub static ULOG_OFFSET: AtomicU16 = AtomicU16::new(0);
/// Total length of current item
pub static ULOG_ITEM_LEN: AtomicU16 = AtomicU16::new(0);

// Mag calibration observability statics
/// Mag cal offsets (readable by GetMagCal handler)
pub static MAG_CAL_OFFSET: Mutex<CriticalSectionRawMutex, Cell<(f32, f32, f32)>> =
    Mutex::new(Cell::new((0.0, 0.0, 0.0)));
/// 0=uncalibrated, 1=collecting, 2=calibrated
pub static MAG_CAL_STATUS: AtomicU8 = AtomicU8::new(0);
/// Number of calibration samples collected so far
pub static MAG_CAL_SAMPLES: AtomicU16 = AtomicU16::new(0);

/// Context passed to all RPC handlers
pub struct RpcContext {
    pub cmd_sender: Sender<'static, CriticalSectionRawMutex, RpcCommand, 16>,
    pub aon_timer: &'static AonTimer<'static>,
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Send an RpcCommand via the context channel, returning AckResp.
fn send_cmd(ctx: &mut RpcContext, cmd: RpcCommand) -> AckResp {
    match ctx.cmd_sender.try_send(cmd) {
        Ok(()) => AckResp::ok(),
        Err(_) => AckResp::error(1),
    }
}

// ---------------------------------------------------------------------------
// Handlers
// ---------------------------------------------------------------------------

fn handle_set_throttle(ctx: &mut RpcContext, _hdr: VarHeader, req: SetThrottleReq) -> AckResp {
    send_cmd(ctx, RpcCommand::SetThrottle(req.percent))
}

fn handle_set_elevons(ctx: &mut RpcContext, _hdr: VarHeader, req: SetElevonsReq) -> AckResp {
    send_cmd(ctx, RpcCommand::SetElevons {
        left: req.left,
        right: req.right,
    })
}

fn handle_set_control_mode(
    ctx: &mut RpcContext,
    _hdr: VarHeader,
    req: SetControlModeReq,
) -> AckResp {
    send_cmd(ctx, RpcCommand::SetMode(req.mode))
}

fn handle_arm(ctx: &mut RpcContext, _hdr: VarHeader, _req: ()) -> AckResp {
    send_cmd(ctx, RpcCommand::Arm)
}

fn handle_disarm(ctx: &mut RpcContext, _hdr: VarHeader, _req: ()) -> AckResp {
    send_cmd(ctx, RpcCommand::Disarm)
}

fn handle_emergency_stop(ctx: &mut RpcContext, _hdr: VarHeader, _req: ()) -> AckResp {
    send_cmd(ctx, RpcCommand::EmergencyStop)
}

fn handle_get_status(_ctx: &mut RpcContext, _hdr: VarHeader, _req: ()) -> StatusResp {
    let imu_status = elle_hardware::imu::IMU_STATUS.try_read();
    let state = crate::flight_state::FLIGHT_STATE.read_cached();
    StatusResp {
        armed: state.armed,
        failsafe: state.failsafe,
        mode: state.mode,
        imu_calibrated: imu_status.as_ref().map(|s| s.calibrated).unwrap_or(false),
        imu_error_count: imu_status.as_ref().map(|s| s.error_count).unwrap_or(0),
        rc_age_ms: state.rc_age_ms,
    }
}

fn handle_get_attitude(_ctx: &mut RpcContext, _hdr: VarHeader, _req: ()) -> AttitudeResp {
    let att = elle_hardware::imu::ATTITUDE.read_cached();
    AttitudeResp {
        pitch_cdeg: (att.pitch * RAD_TO_CDEG) as i16,
        roll_cdeg: (att.roll * RAD_TO_CDEG) as i16,
        yaw_cdeg: (att.yaw * RAD_TO_CDEG) as i16,
        pitch_rate_cdeg: (att.pitch_rate * RAD_TO_CDEG) as i16,
        roll_rate_cdeg: (att.roll_rate * RAD_TO_CDEG) as i16,
        yaw_rate_cdeg: (att.yaw_rate * RAD_TO_CDEG) as i16,
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
    let mag = elle_hardware::imu::MAG.read_cached();
    MagnetometerResp {
        x: mag.x,
        y: mag.y,
        z: mag.z,
    }
}

fn handle_get_rc_channels(_ctx: &mut RpcContext, _hdr: VarHeader, _req: ()) -> RcChannelsResp {
    if let Some(channels) = crate::rc_signal::RC_SIGNAL.try_take() {
        crate::rc_signal::RC_SIGNAL.signal(channels); // put back
        RcChannelsResp { channels }
    } else {
        RcChannelsResp { channels: [0; 16] }
    }
}

fn handle_get_barometer(_ctx: &mut RpcContext, _hdr: VarHeader, _req: ()) -> BarometerResp {
    let baro = elle_hardware::imu::BARO.read_cached();
    BarometerResp {
        pressure_hpa: baro.pressure_hpa,
        temperature_c: baro.temperature_c,
        altitude_m: baro.altitude_m,
    }
}

fn handle_get_engine(_ctx: &mut RpcContext, _hdr: VarHeader, _req: ()) -> EngineResp {
    let eng = elle_hardware::dshot::ENGINE_CACHE.lock(|c| c.get());
    EngineResp {
        left: EngineUnit {
            erpm: eng.left.erpm,
            throttle: eng.left.throttle,
            valid: eng.left.valid,
            target_erpm: eng.left.target_erpm,
            temperature: eng.left.temperature,
            voltage_mv: eng.left.voltage_mv,
            current_ma: eng.left.current_ma,
        },
        right: EngineUnit {
            erpm: eng.right.erpm,
            throttle: eng.right.throttle,
            valid: eng.right.valid,
            target_erpm: eng.right.target_erpm,
            temperature: eng.right.temperature,
            voltage_mv: eng.right.voltage_mv,
            current_ma: eng.right.current_ma,
        },
    }
}

fn handle_start_ulog(ctx: &mut RpcContext, _hdr: VarHeader, _req: ()) -> AckResp {
    send_cmd(ctx, RpcCommand::StartULog)
}

fn handle_stop_ulog(ctx: &mut RpcContext, _hdr: VarHeader, _req: ()) -> AckResp {
    send_cmd(ctx, RpcCommand::StopULog)
}

fn handle_read_ulog_chunk(ctx: &mut RpcContext, _hdr: VarHeader, _req: ()) -> ULogReadResp {
    let state = ULOG_STATE.load(Ordering::Acquire);
    match state {
        s if s == ULogState::Ready as u8 => {
            let offset = ULOG_OFFSET.load(Ordering::Acquire) as usize;
            let total = ULOG_ITEM_LEN.load(Ordering::Acquire) as usize;

            if let Some((item_data, _)) = ULOG_ITEM_SIGNAL.try_take() {
                let chunk_len = (total - offset).min(512);
                let mut data = heapless::Vec::new();
                let _ = data.extend_from_slice(&item_data[offset..offset + chunk_len]);
                let new_offset = offset + chunk_len;

                if new_offset >= total {
                    // Item fully sent — pop it and prefetch next
                    ULOG_STATE.store(ULogState::Reading as u8, Ordering::Release);
                    ULOG_OFFSET.store(0, Ordering::Release);
                    let _ = ctx.cmd_sender.try_send(RpcCommand::PopAndPeekULog);
                } else {
                    // More fragments of this item remain
                    ULOG_OFFSET.store(new_offset as u16, Ordering::Release);
                    ULOG_ITEM_SIGNAL.signal((item_data, total));
                }

                ULogReadResp {
                    data,
                    has_more: true,
                    pending: false,
                }
            } else {
                // Signal was taken between state check and try_take
                ULogReadResp {
                    data: heapless::Vec::new(),
                    has_more: true,
                    pending: true,
                }
            }
        }
        s if s == ULogState::Empty as u8 => {
            ULOG_STATE.store(ULogState::Idle as u8, Ordering::Release);
            ULogReadResp {
                data: heapless::Vec::new(),
                has_more: false,
                pending: false,
            }
        }
        s if s == ULogState::Idle as u8 => {
            // Start first read
            ULOG_STATE.store(ULogState::Reading as u8, Ordering::Release);
            let _ = ctx.cmd_sender.try_send(RpcCommand::ReadULogChunk);
            ULogReadResp {
                data: heapless::Vec::new(),
                has_more: true,
                pending: true,
            }
        }
        _ => {
            // READING state — not ready yet
            ULogReadResp {
                data: heapless::Vec::new(),
                has_more: true,
                pending: true,
            }
        }
    }
}

fn handle_erase_ulog(ctx: &mut RpcContext, _hdr: VarHeader, _req: ()) -> AckResp {
    info!("RPC: ULog erase requested");
    send_cmd(ctx, RpcCommand::EraseULog)
}

fn handle_get_ulog_info(_ctx: &mut RpcContext, _hdr: VarHeader, _req: ()) -> ULogInfoResp {
    use elle_hardware::flash_constants::ULOG_FLASH_SIZE;
    use elle_hardware::sequential_flash_manager::{ULOG_BYTES_USED, ULOG_ITEMS_STORED};

    ULogInfoResp {
        recording: ULOG_ENABLED.load(Ordering::Relaxed),
        region_total: ULOG_FLASH_SIZE as u32,
        bytes_used: ULOG_BYTES_USED.load(Ordering::Relaxed),
        items_stored: ULOG_ITEMS_STORED.load(Ordering::Relaxed),
    }
}

fn handle_get_time(ctx: &mut RpcContext, _hdr: VarHeader, _req: ()) -> u64 {
    ctx.aon_timer.now()
}

fn handle_get_controller_output(
    _ctx: &mut RpcContext,
    _hdr: VarHeader,
    _req: (),
) -> ControllerOutputResp {
    let out = crate::flight_state::CONTROLLER_OUTPUT.read_cached();
    ControllerOutputResp {
        pitch_correction_cp: (out.pitch_correction * 10000.0) as i16,
        roll_correction_cp: (out.roll_correction * 10000.0) as i16,
        pitch_setpoint_cdeg: (out.pitch_setpoint_deg * 100.0) as i16,
        roll_setpoint_cdeg: (out.roll_setpoint_deg * 100.0) as i16,
        elevon_left_us: out.elevon_left_us as u16,
        elevon_right_us: out.elevon_right_us as u16,
        engine_left_dshot: out.engine_left_dshot,
        engine_right_dshot: out.engine_right_dshot,
    }
}

fn handle_set_pid_gains(
    ctx: &mut RpcContext,
    _hdr: VarHeader,
    req: SetPidGainsReq,
) -> AckResp {
    send_cmd(ctx, RpcCommand::SetPidGains {
        pitch_kp: req.pitch_kp_x1000 as f32 / 1000.0,
        pitch_ki: req.pitch_ki_x1000 as f32 / 1000.0,
        pitch_kd: req.pitch_kd_x1000 as f32 / 1000.0,
        roll_kp: req.roll_kp_x1000 as f32 / 1000.0,
        roll_ki: req.roll_ki_x1000 as f32 / 1000.0,
        roll_kd: req.roll_kd_x1000 as f32 / 1000.0,
        scale: req.scale_x10000 as f32 / 10000.0,
        i_limit: req.i_limit_x10 as f32 / 10.0,
    })
}

fn handle_set_attitude_setpoint(
    ctx: &mut RpcContext,
    _hdr: VarHeader,
    req: SetAttitudeSetpointReq,
) -> AckResp {
    send_cmd(ctx, RpcCommand::SetAttitudeSetpoint {
        pitch_deg: req.pitch_cdeg as f32 / 100.0,
        roll_deg: req.roll_cdeg as f32 / 100.0,
    })
}

fn handle_start_autotune(
    ctx: &mut RpcContext,
    _hdr: VarHeader,
    req: StartAutotuneReq,
) -> AckResp {
    send_cmd(ctx, RpcCommand::StartAutotune {
        axis: req.axis,
        relay_deg_x10: req.relay_deg_x10,
        num_cycles: req.num_cycles,
        rule: req.rule,
    })
}

fn handle_abort_autotune(ctx: &mut RpcContext, _hdr: VarHeader, _req: ()) -> AckResp {
    send_cmd(ctx, RpcCommand::AbortAutotune)
}

fn handle_start_mag_cal(ctx: &mut RpcContext, _hdr: VarHeader, _req: ()) -> AckResp {
    send_cmd(ctx, RpcCommand::StartMagCal)
}

fn handle_clear_mag_cal(ctx: &mut RpcContext, _hdr: VarHeader, _req: ()) -> AckResp {
    send_cmd(ctx, RpcCommand::ClearMagCal)
}

fn handle_get_mag_cal(_ctx: &mut RpcContext, _hdr: VarHeader, _req: ()) -> MagCalResp {
    let (ox, oy, oz) = MAG_CAL_OFFSET.lock(|c| c.get());
    let status = MAG_CAL_STATUS.load(Ordering::Relaxed);
    let samples = MAG_CAL_SAMPLES.load(Ordering::Relaxed);
    MagCalResp {
        offset_x: ox,
        offset_y: oy,
        offset_z: oz,
        calibrated: status == 2,
        collecting: status == 1,
        samples,
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
        | GetStatusEndpoint         | blocking  | handle_get_status         |
        | GetAttitudeEndpoint       | blocking  | handle_get_attitude       |
        | GetPerformanceEndpoint    | blocking  | handle_get_performance    |
        | ResetPerformanceEndpoint  | blocking  | handle_reset_performance  |
        | PingEndpoint              | blocking  | handle_ping               |
        | GetVersionEndpoint        | blocking  | handle_get_version        |
        | GetMagnetometerEndpoint   | blocking  | handle_get_magnetometer   |
        | GetBarometerEndpoint      | blocking  | handle_get_barometer      |
        | GetEngineEndpoint         | blocking  | handle_get_engine         |
        | GetGnssEndpoint           | blocking  | handle_get_gnss           |
        | GetRcChannelsEndpoint     | blocking  | handle_get_rc_channels    |
        | StartULogEndpoint         | blocking  | handle_start_ulog         |
        | StopULogEndpoint          | blocking  | handle_stop_ulog          |
        | ReadULogChunkEndpoint     | blocking  | handle_read_ulog_chunk    |
        | EraseULogEndpoint         | blocking  | handle_erase_ulog         |
        | GetULogInfoEndpoint       | blocking  | handle_get_ulog_info      |
        | GetTimeEndpoint           | blocking  | handle_get_time           |
        | GetControllerOutputEndpoint | blocking | handle_get_controller_output |
        | SetPidGainsEndpoint       | blocking  | handle_set_pid_gains      |
        | SetAttitudeSetpointEndpoint | blocking | handle_set_attitude_setpoint |
        | StartAutotuneEndpoint     | blocking  | handle_start_autotune       |
        | AbortAutotuneEndpoint     | blocking  | handle_abort_autotune       |
        | StartMagCalEndpoint       | blocking  | handle_start_mag_cal        |
        | ClearMagCalEndpoint       | blocking  | handle_clear_mag_cal        |
        | GetMagCalEndpoint         | blocking  | handle_get_mag_cal          |
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
