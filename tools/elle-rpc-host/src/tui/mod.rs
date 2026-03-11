//! TUI monitoring dashboard
//!
//! Streams live telemetry and logs from the flight controller with interactive commands.

mod commands;
mod state;
mod ui;

use std::io;
use std::time::Duration;

use anyhow::Result;
use crossterm::ExecutableCommand;
use crossterm::event::{Event, KeyCode, KeyEventKind, KeyModifiers};
use crossterm::terminal::{
    EnterAlternateScreen, LeaveAlternateScreen, disable_raw_mode, enable_raw_mode,
};
use elle_rpc_icd::*;
use futures::StreamExt;
use postcard_rpc::header::VarSeqKind;
use postcard_rpc::host_client::HostClient;
use postcard_rpc::standard_icd::WireError;
use ratatui::Terminal;
use ratatui::backend::CrosstermBackend;
use tokio::sync::mpsc;

use crate::probe;
use crate::wire::{ProbeRttRx, ProbeRttTx, TokSpawn};

use self::commands::CommandResult;
use self::state::AppState;

pub async fn run() -> Result<()> {
    // Connect to probe and set up RTT
    let (session, rtt) = probe::connect()?;

    let (out_tx, out_rx) = mpsc::channel(64);
    let (inc_tx, inc_rx) = mpsc::channel(64);

    let app_rx = ProbeRttRx { inc: inc_rx };
    let app_tx = ProbeRttTx { out: out_tx };

    // Spawn RTT worker thread with shutdown flag
    let shutdown = probe::shutdown_flag();
    let shutdown_clone = shutdown.clone();
    let worker_handle =
        std::thread::spawn(move || probe::rtt_worker(session, rtt, inc_tx, out_rx, shutdown_clone));

    // Create HostClient
    let client = HostClient::<WireError>::new_with_wire(
        app_tx,
        app_rx,
        TokSpawn,
        VarSeqKind::Seq2,
        "error",
        64,
    );

    // Subscribe to topics
    let mut log_sub = client.subscribe_multi::<LogTopic>(64).await.unwrap();

    // Initialize terminal
    enable_raw_mode()?;
    io::stdout().execute(EnterAlternateScreen)?;

    // Install panic hook to restore terminal
    let original_hook = std::panic::take_hook();
    std::panic::set_hook(Box::new(move |info| {
        let _ = disable_raw_mode();
        let _ = io::stdout().execute(LeaveAlternateScreen);
        original_hook(info);
    }));

    let backend = CrosstermBackend::new(io::stdout());
    let mut terminal = Terminal::new(backend)?;

    let mut state = AppState::new();
    state.connected = true;

    // Fetch initial version
    let client_ref = &client;
    if let Ok(Ok(v)) = tokio::time::timeout(
        Duration::from_secs(2),
        client_ref.send_resp::<GetVersionEndpoint>(&()),
    )
    .await
    {
        state.version = Some(v);
    }

    // Fetch initial status
    if let Ok(Ok(s)) = tokio::time::timeout(
        Duration::from_secs(2),
        client_ref.send_resp::<GetStatusEndpoint>(&()),
    )
    .await
    {
        state.status = Some(s);
    }

    let mut tick_interval = tokio::time::interval(Duration::from_millis(100)); // 10Hz UI refresh
    let mut attitude_interval = tokio::time::interval(Duration::from_millis(100)); // 10Hz attitude poll
    let mut status_interval = tokio::time::interval(Duration::from_secs(2)); // Status poll
    let mut mag_interval = tokio::time::interval(Duration::from_millis(200)); // 5Hz mag poll
    let mut baro_interval = tokio::time::interval(Duration::from_secs(1)); // 1Hz baro poll
    let mut gnss_interval = tokio::time::interval(Duration::from_secs(1)); // 1Hz GNSS poll
    let mut rc_interval = tokio::time::interval(Duration::from_millis(50)); // 20Hz RC poll
    let mut ctrl_interval = tokio::time::interval(Duration::from_millis(100)); // 10Hz controller poll
    let mut engine_interval = tokio::time::interval(Duration::from_millis(200)); // 5Hz engine poll
    let mut event_stream = crossterm::event::EventStream::new();

    let result = loop {
        tokio::select! {
            // Log subscription
            Ok(msg) = log_sub.recv() => {
                state.push_log(msg);
            }

            // Keyboard input
            Some(Ok(evt)) = event_stream.next() => {
                if let Event::Key(key) = evt {
                    if key.kind != KeyEventKind::Press {
                        continue;
                    }
                    match (key.code, key.modifiers) {
                        (KeyCode::Char('c'), KeyModifiers::CONTROL) => break Ok(()),
                        (KeyCode::Char('d'), KeyModifiers::CONTROL) => break Ok(()),
                        (KeyCode::Enter, _) => {
                            let input = state.command_input.clone();
                            state.command_input.clear();
                            if !input.trim().is_empty() {
                                state.push_command_history(input.clone());
                                match commands::execute(&input, &client, &mut state).await {
                                    CommandResult::Ok(msg) => {
                                        if !msg.is_empty() {
                                            state.set_status_message(msg);
                                        }
                                    }
                                    CommandResult::Err(msg) => {
                                        state.set_status_message(format!("ERROR: {msg}"));
                                    }
                                    CommandResult::Quit => break Ok(()),
                                    CommandResult::Unknown(cmd) => {
                                        state.set_status_message(
                                            format!("Unknown command: {cmd} (? for help)")
                                        );
                                    }
                                    CommandResult::Background(msg, handle) => {
                                        state.set_status_message(msg);
                                        state.background_task = Some(handle);
                                    }
                                }
                            }
                        }
                        (KeyCode::Char(c), _) => {
                            state.command_input.push(c);
                            state.history_index = None;
                        }
                        (KeyCode::Backspace, _) => {
                            state.command_input.pop();
                        }
                        (KeyCode::Esc, _) => {
                            state.command_input.clear();
                            state.history_index = None;
                        }
                        (KeyCode::Up, _) => {
                            if !state.command_history.is_empty() {
                                let idx = match state.history_index {
                                    Some(i) => (i + 1).min(state.command_history.len() - 1),
                                    None => 0,
                                };
                                state.history_index = Some(idx);
                                state.command_input = state.command_history[idx].clone();
                            }
                        }
                        (KeyCode::Down, _) => {
                            if let Some(idx) = state.history_index {
                                if idx == 0 {
                                    state.history_index = None;
                                    state.command_input.clear();
                                } else {
                                    state.history_index = Some(idx - 1);
                                    state.command_input =
                                        state.command_history[idx - 1].clone();
                                }
                            }
                        }
                        _ => {}
                    }
                }
            }

            // UI refresh tick
            _ = tick_interval.tick() => {
                // Check for stale connection
                if let Some(last) = state.last_poll {
                    state.connected = last.elapsed() < Duration::from_secs(5);
                }

                // Poll background task for completion
                if let Some(handle) = &mut state.background_task
                    && handle.is_finished()
                    && let Some(handle) = state.background_task.take()
                {
                    match handle.await {
                        Ok(msg) => state.set_status_message(msg),
                        Err(e) => state.set_status_message(format!("ERROR: Task failed: {e}")),
                    }
                }
            }

            // Periodic attitude poll (paused during background tasks to avoid RTT congestion)
            _ = attitude_interval.tick(), if state.background_task.is_none() => {
                if let Ok(Ok(a)) = tokio::time::timeout(
                    Duration::from_millis(200),
                    client.send_resp::<GetAttitudeEndpoint>(&()),
                ).await {
                    state.push_attitude(a);
                }
            }

            // Periodic status + ULog info poll
            _ = status_interval.tick(), if state.background_task.is_none() => {
                if let Ok(Ok(s)) = tokio::time::timeout(
                    Duration::from_secs(1),
                    client.send_resp::<GetStatusEndpoint>(&()),
                ).await {
                    state.status = Some(s);
                    state.last_poll = Some(std::time::Instant::now());
                    state.connected = true;
                }
                if let Ok(Ok(info)) = tokio::time::timeout(
                    Duration::from_secs(1),
                    client.send_resp::<GetULogInfoEndpoint>(&()),
                ).await {
                    state.ulog_recording = info.recording;
                }
            }

            // Periodic magnetometer poll
            _ = mag_interval.tick(), if state.background_task.is_none() => {
                if let Ok(Ok(m)) = tokio::time::timeout(
                    Duration::from_millis(500),
                    client.send_resp::<GetMagnetometerEndpoint>(&()),
                ).await {
                    state.magnetometer = Some(m);
                    state.connected = true;
                }
            }

            // Periodic barometer poll
            _ = baro_interval.tick(), if state.background_task.is_none() => {
                if let Ok(Ok(b)) = tokio::time::timeout(
                    Duration::from_secs(1),
                    client.send_resp::<GetBarometerEndpoint>(&()),
                ).await {
                    state.barometer = Some(b);
                    state.connected = true;
                }
            }

            // Periodic GNSS poll
            _ = gnss_interval.tick(), if state.background_task.is_none() => {
                if let Ok(Ok(g)) = tokio::time::timeout(
                    Duration::from_secs(1),
                    client.send_resp::<GetGnssEndpoint>(&()),
                ).await {
                    state.gnss = Some(g);
                    state.connected = true;
                }
            }

            // Periodic RC channels poll
            _ = rc_interval.tick(), if state.background_task.is_none() => {
                if let Ok(Ok(rc)) = tokio::time::timeout(
                    Duration::from_millis(100),
                    client.send_resp::<GetRcChannelsEndpoint>(&()),
                ).await {
                    state.rc_channels = Some(rc);
                    state.connected = true;
                }
            }

            // Periodic controller output poll
            _ = ctrl_interval.tick(), if state.background_task.is_none() => {
                if let Ok(Ok(c)) = tokio::time::timeout(
                    Duration::from_millis(200),
                    client.send_resp::<GetControllerOutputEndpoint>(&()),
                ).await {
                    state.push_controller_output(c);
                    state.connected = true;
                }
            }

            // Periodic engine telemetry poll
            _ = engine_interval.tick(), if state.background_task.is_none() => {
                if let Ok(Ok(e)) = tokio::time::timeout(
                    Duration::from_millis(200),
                    client.send_resp::<GetEngineEndpoint>(&()),
                ).await {
                    state.engine = Some(e);
                    state.connected = true;
                }
            }
        }

        // Render
        terminal.draw(|f| ui::draw(f, &state))?;
    };

    // Restore terminal
    disable_raw_mode()?;
    io::stdout().execute(LeaveAlternateScreen)?;

    // Signal the RTT worker to stop and wait for it to release the probe
    shutdown.store(true, std::sync::atomic::Ordering::Relaxed);
    let _ = worker_handle.join();

    result
}
