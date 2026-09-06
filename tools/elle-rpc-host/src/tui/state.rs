//! TUI application state

use std::collections::VecDeque;
use std::time::Instant;

use chrono::{DateTime, Local};
use elle_rpc_icd::*;

const LOG_HISTORY: usize = 256;
const ATTITUDE_HISTORY: usize = 60;
const COMMAND_HISTORY: usize = 64;

/// A log line as displayed: the device event plus host-side arrival time and a
/// repeat count for collapsed duplicates.
pub struct LogEntry {
    pub level: u8,
    pub code: u16,
    pub at: DateTime<Local>,
    pub count: u32,
}

pub struct AppState {
    // Device info (polled)
    pub status: Option<StatusResp>,
    pub attitude: Option<AttitudeResp>,
    pub performance: Option<PerformanceResp>,
    pub version: Option<VersionResp>,

    // Magnetometer
    pub magnetometer: Option<MagnetometerResp>,

    // Barometer
    pub barometer: Option<BarometerResp>,

    // GNSS
    pub gnss: Option<GnssResp>,

    // RC channels
    pub rc_channels: Option<RcChannelsResp>,

    // Controller output
    pub controller_output: Option<ControllerOutputResp>,

    // Engine telemetry
    pub engine: Option<EngineResp>,

    // Device uptime (polled)
    pub device_time_ms: Option<u64>,

    // ULog recording state (polled from device)
    pub ulog_recording: bool,

    // Attitude history for sparklines
    pub attitude_history: VecDeque<(i16, i16)>, // (pitch, roll) in centidegrees

    // PID correction history for sparklines
    pub pid_history: VecDeque<(i16, i16)>, // (pitch_correction_cp, roll_correction_cp)

    // Logs
    pub logs: VecDeque<LogEntry>,

    // Connection state
    pub connected: bool,
    pub last_poll: Option<Instant>,

    // UI state
    pub command_input: String,
    pub command_history: VecDeque<String>,
    pub history_index: Option<usize>,
    pub status_message: Option<(String, Instant)>,

    // Background task (e.g. ulog extract)
    pub background_task: Option<tokio::task::JoinHandle<String>>,
}

impl AppState {
    pub fn new() -> Self {
        Self {
            status: None,
            attitude: None,
            performance: None,
            version: None,
            magnetometer: None,
            barometer: None,
            gnss: None,
            rc_channels: None,
            controller_output: None,
            engine: None,
            device_time_ms: None,
            ulog_recording: false,
            attitude_history: VecDeque::with_capacity(ATTITUDE_HISTORY),
            pid_history: VecDeque::with_capacity(ATTITUDE_HISTORY),
            logs: VecDeque::with_capacity(LOG_HISTORY),
            connected: false,
            last_poll: None,
            command_input: String::new(),
            command_history: VecDeque::with_capacity(COMMAND_HISTORY),
            history_index: None,
            status_message: None,
            background_task: None,
        }
    }

    pub fn push_attitude(&mut self, att: AttitudeResp) {
        // Skip all-zero responses — firmware returns zeros before first real sample.
        if att.pitch_cdeg == 0 && att.roll_cdeg == 0 && att.yaw_cdeg == 0 && self.attitude.is_some()
        {
            self.mark_poll_success();
            return;
        }

        self.attitude_history
            .push_back((att.pitch_cdeg, att.roll_cdeg));
        if self.attitude_history.len() > ATTITUDE_HISTORY {
            self.attitude_history.pop_front();
        }

        self.attitude = Some(att);
        self.mark_poll_success();
    }

    pub fn push_controller_output(&mut self, c: ControllerOutputResp) {
        self.pid_history
            .push_back((c.pitch_correction_cp, c.roll_correction_cp));
        if self.pid_history.len() > ATTITUDE_HISTORY {
            self.pid_history.pop_front();
        }
        self.controller_output = Some(c);
    }

    /// Append a log message, collapsing consecutive repeats of the same event into
    /// a single entry with a repeat count. Periodic events (CRSF TX heartbeat, GNSS
    /// updates) otherwise fill the whole panel with identical lines.
    pub fn push_log(&mut self, msg: LogMsg) {
        if let Some(last) = self.logs.back_mut()
            && last.level == msg.level
            && last.code == msg.code
        {
            last.count += 1;
            last.at = Local::now();
            return;
        }

        self.logs.push_back(LogEntry {
            level: msg.level,
            code: msg.code,
            at: Local::now(),
            count: 1,
        });
        if self.logs.len() > LOG_HISTORY {
            self.logs.pop_front();
        }
    }

    pub fn mark_poll_success(&mut self) {
        self.last_poll = Some(Instant::now());
        self.connected = true;
    }

    pub fn set_status_message(&mut self, msg: String) {
        self.status_message = Some((msg, Instant::now()));
    }

    pub fn push_command_history(&mut self, cmd: String) {
        if !cmd.is_empty() {
            // Don't duplicate consecutive commands
            if self.command_history.front() != Some(&cmd) {
                self.command_history.push_front(cmd);
                if self.command_history.len() > COMMAND_HISTORY {
                    self.command_history.pop_back();
                }
            }
        }
        self.history_index = None;
    }
}
