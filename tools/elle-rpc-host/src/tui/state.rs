//! TUI application state

use std::collections::VecDeque;
use std::time::Instant;

use elle_rpc_icd::*;

const LOG_HISTORY: usize = 256;
const ATTITUDE_HISTORY: usize = 60;
const COMMAND_HISTORY: usize = 64;

pub struct AppState {
    // Device info (polled)
    pub status: Option<StatusResp>,
    pub attitude: Option<AttitudeResp>,
    pub performance: Option<PerformanceResp>,
    pub version: Option<VersionResp>,

    // Magnetometer
    pub magnetometer: Option<MagnetometerResp>,

    // GNSS
    pub gnss: Option<GnssResp>,

    // RC channels
    pub rc_channels: Option<RcChannelsResp>,

    // Attitude history for sparklines
    pub attitude_history: VecDeque<(i16, i16)>, // (pitch, roll) in centidegrees

    // Logs
    pub logs: VecDeque<LogMsg>,

    // Connection state
    pub connected: bool,
    pub last_poll: Option<Instant>,

    // UI state
    pub command_input: String,
    pub command_history: VecDeque<String>,
    pub history_index: Option<usize>,
    pub status_message: Option<(String, Instant)>,
}

impl AppState {
    pub fn new() -> Self {
        Self {
            status: None,
            attitude: None,
            performance: None,
            version: None,
            magnetometer: None,
            gnss: None,
            rc_channels: None,
            attitude_history: VecDeque::with_capacity(ATTITUDE_HISTORY),
            logs: VecDeque::with_capacity(LOG_HISTORY),
            connected: false,
            last_poll: None,
            command_input: String::new(),
            command_history: VecDeque::with_capacity(COMMAND_HISTORY),
            history_index: None,
            status_message: None,
        }
    }

    pub fn push_attitude(&mut self, att: AttitudeResp) {
        self.attitude_history
            .push_back((att.pitch_cdeg, att.roll_cdeg));
        if self.attitude_history.len() > ATTITUDE_HISTORY {
            self.attitude_history.pop_front();
        }

        self.attitude = Some(att);
        self.last_poll = Some(Instant::now());
        self.connected = true;
    }

    pub fn push_log(&mut self, msg: LogMsg) {
        self.logs.push_back(msg);
        if self.logs.len() > LOG_HISTORY {
            self.logs.pop_front();
        }
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
