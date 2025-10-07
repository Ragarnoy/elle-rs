//! RTT-based remote control interface for debugging

use crate::system::ControlMode;
use defmt::Format;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};

#[cfg(feature = "rtt-control")]
use crate::control::commands::{AttitudeMode, NormalizedCommands, PilotCommands};
#[cfg(feature = "rtt-control")]
use embassy_sync::channel::Receiver;
#[cfg(feature = "rtt-control")]
use embassy_time::Instant;

#[cfg(feature = "rtt-control")]
use core::fmt::Write;
#[cfg(feature = "rtt-control")]
use defmt::info;
#[cfg(feature = "rtt-control")]
use embassy_sync::channel::Sender;

#[cfg(feature = "rtt-control")]
use rtt_target::ChannelMode::NoBlockSkip;
#[cfg(feature = "rtt-control")]
use rtt_target::{DownChannel, UpChannel, rtt_init, set_defmt_channel};

#[cfg(feature = "rtt-control")]
use rtt_target as _;

/// Command types that can be sent over RTT
#[derive(Debug, Clone, Copy, Format)]
pub enum DebugCommand {
    // Direct control commands
    SetThrottle(u16), // 0-2047 SBUS value
    SetElevons {
        left: u16,
        right: u16,
    },
    SetControlMode(ControlMode),

    // Safety commands
    Arm,
    Disarm,
    EmergencyStop,

    // Trim adjustments
    AdjustTrim {
        left: i8,
        right: i8,
    },

    // Calibration
    SaveCalibration,
    ClearCalibration,

    // Status requests
    GetStatus,
    GetAttitude,

    // Performance monitoring
    #[cfg(feature = "performance-monitoring")]
    GetPerformance,
    #[cfg(feature = "performance-monitoring")]
    ResetPerformance,
}

/// Channel for passing commands to flight controller
pub static COMMAND_CHANNEL: Channel<CriticalSectionRawMutex, DebugCommand, 16> = Channel::new();

/// RTT Commander - manages RTT commands and converts them to PilotCommands
#[cfg(feature = "rtt-control")]
pub struct RttCommander {
    commands: NormalizedCommands,
    command_rx: Receiver<'static, CriticalSectionRawMutex, DebugCommand, 16>,
}

#[cfg(feature = "rtt-control")]
impl RttCommander {
    pub fn new(command_rx: Receiver<'static, CriticalSectionRawMutex, DebugCommand, 16>) -> Self {
        Self {
            commands: NormalizedCommands::neutral(),
            command_rx,
        }
    }

    pub async fn read_commands<F>(&mut self, mut debug_callback: F) -> Option<PilotCommands>
    where
        F: FnMut(DebugCommand),
    {
        // Process all pending RTT commands
        while let Ok(cmd) = self.command_rx.try_receive() {
            if self.is_flight_command(cmd) {
                self.apply_debug_command(cmd);
                self.commands.timestamp = Instant::now();
            } else {
                // Route non-flight commands to callback
                debug_callback(cmd);
            }
        }

        // Only return if fresh (2 second timeout)
        if self.commands.timestamp.elapsed() < Duration::from_secs(2) {
            Some(PilotCommands::Normalized(self.commands))
        } else {
            None
        }
    }

    fn is_flight_command(&self, cmd: DebugCommand) -> bool {
        matches!(
            cmd,
            DebugCommand::SetThrottle(_)
                | DebugCommand::SetElevons { .. }
                | DebugCommand::SetControlMode(_)
                | DebugCommand::EmergencyStop
                | DebugCommand::Disarm
        )
    }

    fn apply_debug_command(&mut self, cmd: DebugCommand) {
        use defmt::info;

        match cmd {
            DebugCommand::SetThrottle(raw_value) => {
                self.commands.throttle = (raw_value as f32 / 2047.0).clamp(0.0, 1.0);
                info!("RTT: Throttle -> {}", self.commands.throttle);
            }

            DebugCommand::SetElevons { left, right } => {
                // Convert raw elevon positions (0-2047) to normalized pitch/roll
                // Inverse of mixing: left+right = pitch, left-right = roll
                let left_norm = (left as f32 / 2047.0 - 0.5) * 2.0;
                let right_norm = (right as f32 / 2047.0 - 0.5) * 2.0;

                self.commands.pitch = ((left_norm + right_norm) / 2.0).clamp(-1.0, 1.0);
                self.commands.roll = ((left_norm - right_norm) / 2.0).clamp(-1.0, 1.0);
                info!(
                    "RTT: Elevons -> P:{} R:{}",
                    self.commands.pitch, self.commands.roll
                );
            }

            DebugCommand::SetControlMode(mode) => {
                self.commands.attitude_mode = match mode {
                    ControlMode::Manual => AttitudeMode::Manual,
                    ControlMode::Mixed => AttitudeMode::Mixed,
                    ControlMode::Autopilot => AttitudeMode::Autopilot,
                };
                info!("RTT: Mode -> {:?}", self.commands.attitude_mode);
            }

            DebugCommand::EmergencyStop => {
                self.commands = NormalizedCommands::neutral();
                info!("RTT: EMERGENCY STOP");
            }

            DebugCommand::Disarm => {
                self.commands.throttle = 0.0;
                info!("RTT: Disarm");
            }

            // Non-flight commands handled in main
            DebugCommand::Arm
            | DebugCommand::AdjustTrim { .. }
            | DebugCommand::SaveCalibration
            | DebugCommand::ClearCalibration
            | DebugCommand::GetStatus
            | DebugCommand::GetAttitude => {}

            #[cfg(feature = "performance-monitoring")]
            DebugCommand::GetPerformance | DebugCommand::ResetPerformance => {}
        }
    }
}

/// RTT control manager
#[cfg(feature = "rtt-control")]
pub struct RttControl {
    down_channel: DownChannel,
    up_channel: UpChannel,
    command_sender: Sender<'static, CriticalSectionRawMutex, DebugCommand, 16>,
}

#[cfg(feature = "rtt-control")]
impl RttControl {
    pub fn init() -> Self {
        // Initialize RTT with separate channels for defmt and commands
        let channels = rtt_init! {
            up: {
                0: {
                    size: 1024,
                    mode: NoBlockSkip,
                    name: "Defmt Logs"
                }
                1: {
                    size: 512,
                    mode: NoBlockSkip,
                    name: "Commands"
                }
            }
            down: {
                0: {
                    size: 256,
                    mode: NoBlockSkip,
                    name: "Commands"
                }
            }
        };

        // Set channel 0 as the defmt output channel
        set_defmt_channel(channels.up.0);

        Self {
            down_channel: channels.down.0,
            up_channel: channels.up.1, // Use channel 1 for command responses
            command_sender: COMMAND_CHANNEL.sender(),
        }
    }

    /// Parse incoming command string
    fn parse_command(&mut self, input: &str) -> Option<DebugCommand> {
        let mut parts = input.split_whitespace();
        let cmd = parts.next()?;

        match cmd {
            "throttle" | "t" => parts
                .next()
                .and_then(|v| v.parse::<u16>().ok())
                .map(|val| DebugCommand::SetThrottle(val.min(2047))),

            "elevon" | "e" => {
                let left = parts.next().and_then(|v| v.parse().ok())?;
                let right = parts.next().and_then(|v| v.parse().ok())?;
                Some(DebugCommand::SetElevons { left, right })
            }

            "mode" | "m" => parts.next().and_then(|m| match m {
                "manual" => Some(DebugCommand::SetControlMode(ControlMode::Manual)),
                "mixed" => Some(DebugCommand::SetControlMode(ControlMode::Mixed)),
                "auto" => Some(DebugCommand::SetControlMode(ControlMode::Autopilot)),
                _ => None,
            }),

            "arm" => Some(DebugCommand::Arm),
            "disarm" => Some(DebugCommand::Disarm),
            "stop" | "emergency" => Some(DebugCommand::EmergencyStop),

            "trim" => {
                let left = parts.next().and_then(|v| v.parse().ok())?;
                let right = parts.next().and_then(|v| v.parse().ok())?;
                Some(DebugCommand::AdjustTrim { left, right })
            }

            "save_cal" => Some(DebugCommand::SaveCalibration),
            "clear_cal" => Some(DebugCommand::ClearCalibration),

            "status" | "s" => Some(DebugCommand::GetStatus),
            "attitude" | "a" => Some(DebugCommand::GetAttitude),

            #[cfg(feature = "performance-monitoring")]
            "perf" | "performance" => Some(DebugCommand::GetPerformance),
            #[cfg(feature = "performance-monitoring")]
            "reset_perf" => Some(DebugCommand::ResetPerformance),

            "help" | "?" => {
                self.print_help();
                None
            }

            _ => None,
        }
    }

    fn print_help(&mut self) {
        let _ = writeln!(self.up_channel, "RTT Debug Commands:");
        let _ = writeln!(self.up_channel, "  throttle/t <0-2047>     - Set throttle");
        let _ = writeln!(
            self.up_channel,
            "  elevon/e <l> <r>        - Set elevon positions"
        );
        let _ = writeln!(
            self.up_channel,
            "  mode/m <manual|mixed|auto> - Set control mode"
        );
        let _ = writeln!(self.up_channel, "  arm                     - Arm motors");
        let _ = writeln!(self.up_channel, "  disarm                  - Disarm motors");
        let _ = writeln!(
            self.up_channel,
            "  stop/emergency          - Emergency stop"
        );
        let _ = writeln!(
            self.up_channel,
            "  trim <left> <right>     - Adjust trim (-100 to 100)"
        );
        let _ = writeln!(
            self.up_channel,
            "  save_cal                - Save IMU calibration"
        );
        let _ = writeln!(
            self.up_channel,
            "  clear_cal               - Clear calibration"
        );
        let _ = writeln!(
            self.up_channel,
            "  status/s                - Get system status"
        );
        let _ = writeln!(
            self.up_channel,
            "  attitude/a              - Get attitude data"
        );

        #[cfg(feature = "performance-monitoring")]
        {
            let _ = writeln!(
                self.up_channel,
                "  perf/performance        - Get performance stats"
            );
            let _ = writeln!(
                self.up_channel,
                "  reset_perf              - Reset performance counters"
            );
        }

        let _ = writeln!(
            self.up_channel,
            "  help/?                  - Show this help"
        );
    }

    pub fn send_status(&mut self, msg: &str) {
        // Send status to the Commands tab (channel 1)
        let _ = writeln!(self.up_channel, "{}", msg);
    }

    /// Main RTT command processing task
    pub async fn run(&mut self) {
        info!("RTT Control interface started");

        let _ = writeln!(
            self.up_channel,
            "RTT Control ready. Type 'help' for commands."
        );
        self.print_help();

        let mut buffer = [0u8; 64];

        loop {
            // Check for incoming commands
            let count = self.down_channel.read(&mut buffer);
            if count > 0
                && let Ok(cmd_str) = core::str::from_utf8(&buffer[..count])
            {
                let cmd_str = cmd_str.trim_end_matches('\0').trim();
                if !cmd_str.is_empty() {
                    self.parse_and_send_command(cmd_str);
                }
            }

            // Small delay to prevent busy waiting
            Timer::after(Duration::from_millis(10)).await;
        }
    }

    /// Helper method to send a command (for testing/external use)
    pub fn send_command(
        &mut self,
        command: DebugCommand,
    ) -> Result<(), embassy_sync::channel::TrySendError<DebugCommand>> {
        self.command_sender.try_send(command)
    }

    /// Parse a command string and send it (for external tools)
    pub fn parse_and_send_command(&mut self, cmd_str: &str) -> bool {
        if let Some(command) = self.parse_command(cmd_str) {
            match self.command_sender.try_send(command) {
                Ok(()) => {
                    self.send_status("OK");
                    true
                }
                Err(_) => {
                    self.send_status("ERROR: Command queue full");
                    false
                }
            }
        } else if !cmd_str.starts_with("help") {
            self.send_status("ERROR: Invalid command");
            false
        } else {
            true
        }
    }
}

/// Stub implementation when RTT control is disabled
#[cfg(not(feature = "rtt-control"))]
pub struct RttControl;

#[cfg(not(feature = "rtt-control"))]
impl RttControl {
    pub fn init() -> Self {
        Self
    }

    pub async fn run(&mut self) {
        // No-op when RTT control is disabled
        loop {
            Timer::after(Duration::from_secs(60)).await;
        }
    }

    pub fn send_status(&mut self, _msg: &str) {
        // No-op when RTT control is disabled
    }

    pub fn send_command(
        &mut self,
        _command: DebugCommand,
    ) -> Result<(), embassy_sync::channel::TrySendError<DebugCommand>> {
        // Always return error when RTT control is disabled
        Err(embassy_sync::channel::TrySendError::Full(_command))
    }

    pub fn parse_and_send_command(&mut self, _cmd_str: &str) -> bool {
        false
    }
}
