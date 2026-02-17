//! TUI rendering with ratatui

use ratatui::Frame;
use ratatui::layout::{Constraint, Direction, Layout, Rect};
use ratatui::style::{Color, Modifier, Style};
use ratatui::text::{Line, Span};
use ratatui::widgets::{Block, Borders, Gauge, Paragraph, Sparkline, Wrap};

use super::state::AppState;

pub fn draw(f: &mut Frame, state: &AppState) {
    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Length(3), // header
            Constraint::Min(10),   // main content
            Constraint::Length(5), // command area
        ])
        .split(f.area());

    draw_header(f, chunks[0], state);
    draw_main(f, chunks[1], state);
    draw_command(f, chunks[2], state);
}

fn draw_header(f: &mut Frame, area: Rect, state: &AppState) {
    let armed = state.status.map(|s| s.armed).unwrap_or(false);

    let failsafe = state.status.map(|s| s.failsafe).unwrap_or(false);

    let mode = state
        .status
        .map(|s| format!("{:?}", s.mode))
        .unwrap_or_else(|| "---".into());

    let version = state
        .version
        .map(|v| format!("v{}.{}.{}", v.major, v.minor, v.patch))
        .unwrap_or_else(|| "---".into());

    let connected_style = if state.connected {
        Style::default().fg(Color::Green)
    } else {
        Style::default().fg(Color::Red)
    };

    let connected_text = if state.connected {
        "CONNECTED"
    } else {
        "DISCONNECTED"
    };

    let armed_style = if armed {
        Style::default().fg(Color::Red).add_modifier(Modifier::BOLD)
    } else {
        Style::default().fg(Color::Green)
    };

    let armed_text = if armed { "ARMED" } else { "DISARMED" };

    let failsafe_span = if failsafe {
        Span::styled(
            " | FAILSAFE",
            Style::default().fg(Color::Red).add_modifier(Modifier::BOLD),
        )
    } else {
        Span::raw("")
    };

    let header = Paragraph::new(Line::from(vec![
        Span::styled(
            " elle monitor ",
            Style::default()
                .fg(Color::Cyan)
                .add_modifier(Modifier::BOLD),
        ),
        Span::styled(format!(" | {version} "), Style::default().fg(Color::Gray)),
        Span::raw(" | "),
        Span::styled(armed_text, armed_style),
        Span::styled(
            format!(" | Mode: {mode}"),
            Style::default().fg(Color::White),
        ),
        failsafe_span,
        Span::raw(" | "),
        Span::styled(connected_text, connected_style),
    ]))
    .block(Block::default().borders(Borders::ALL));

    f.render_widget(header, area);
}

fn draw_main(f: &mut Frame, area: Rect, state: &AppState) {
    let chunks = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([
            Constraint::Percentage(40),
            Constraint::Percentage(25),
            Constraint::Percentage(35),
        ])
        .split(area);

    draw_telemetry(f, chunks[0], state);
    draw_rc_channels(f, chunks[1], state);
    draw_logs(f, chunks[2], state);
}

fn draw_telemetry(f: &mut Frame, area: Rect, state: &AppState) {
    let block = Block::default().title(" Telemetry ").borders(Borders::ALL);
    let inner = block.inner(area);
    f.render_widget(block, area);

    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Length(13), // attitude data + mag + heading + baro + gnss
            Constraint::Length(4),  // pitch sparkline
            Constraint::Length(4),  // roll sparkline
            Constraint::Min(0),     // remaining space
        ])
        .split(inner);

    // Attitude data (from polled GetAttitude endpoint)
    let (pitch, roll, yaw) = if let Some(a) = state.attitude {
        (
            a.pitch_cdeg as f32 / 100.0,
            a.roll_cdeg as f32 / 100.0,
            format!("{:.2}", a.yaw_cdeg as f32 / 100.0),
        )
    } else {
        (0.0, 0.0, "---".into())
    };

    let perf_line = state
        .performance
        .map(|p| {
            format!(
                "  Loop: {}us avg / {}us max",
                p.control_loop_avg_us, p.control_loop_max_us,
            )
        })
        .unwrap_or_else(|| "  Loop: ---".into());

    let imu_line = state
        .status
        .map(|s| {
            format!(
                "  IMU: {} | Errors: {}",
                if s.imu_calibrated { "Cal" } else { "NO CAL" },
                s.imu_error_count,
            )
        })
        .unwrap_or_else(|| "  IMU: ---".into());

    let (mag_line, heading_line) = if let Some(m) = state.magnetometer {
        let heading = (m.y as f64).atan2(m.x as f64).to_degrees();
        let heading = if heading < 0.0 {
            heading + 360.0
        } else {
            heading
        };
        (
            format!("  Mag: X={:>7} Y={:>7} Z={:>7}", m.x, m.y, m.z),
            format!("  Heading: {:>6.1}°", heading),
        )
    } else {
        ("  Mag: ---".into(), "  Heading: ---".into())
    };

    let (baro_line1, baro_line2) = if let Some(b) = state.barometer {
        (
            format!(
                "  Baro: {:.1} hPa | {:.1}\u{00B0}C",
                b.pressure_hpa, b.temperature_c
            ),
            format!("  Alt:  {:.1}m (baro)", b.altitude_m),
        )
    } else {
        ("  Baro: ---".into(), "  Alt:  --- (baro)".into())
    };

    let (gnss_pos_line, gnss_fix_line, gnss_alt_line) = if let Some(g) = state.gnss {
        let fix_str = match g.fix_quality {
            0 => "No fix",
            1 => "GPS",
            2 => "DGPS",
            _ => "Other",
        };
        (
            format!("  GPS: {:.6}, {:.6}", g.latitude, g.longitude),
            format!(
                "  Fix: {} | Sats: {} | HDOP: {:.1}",
                fix_str, g.num_satellites, g.hdop
            ),
            format!("  Alt: {:.1}m MSL", g.altitude_m),
        )
    } else {
        (
            "  GPS: ---".into(),
            "  Fix: ---".into(),
            "  Alt: ---".into(),
        )
    };

    let attitude_text = vec![
        Line::from(format!("  Pitch: {:>7.2}°", pitch)),
        Line::from(format!("  Roll:  {:>7.2}°", roll)),
        Line::from(format!("  Yaw:   {:>7}°", yaw)),
        Line::from(""),
        Line::from(mag_line),
        Line::from(heading_line),
        Line::from(baro_line1),
        Line::from(baro_line2),
        Line::from(gnss_pos_line),
        Line::from(gnss_fix_line),
        Line::from(gnss_alt_line),
        Line::from(perf_line),
        Line::from(imu_line),
    ];

    f.render_widget(Paragraph::new(attitude_text), chunks[0]);

    // Pitch sparkline
    let pitch_data: Vec<u64> = state
        .attitude_history
        .iter()
        .map(|(p, _)| (*p as i32 + 18000) as u64) // shift to positive range
        .collect();

    if !pitch_data.is_empty() {
        let pitch_spark = Sparkline::default()
            .block(Block::default().title(" Pitch ").borders(Borders::TOP))
            .data(&pitch_data)
            .style(Style::default().fg(Color::Yellow));
        f.render_widget(pitch_spark, chunks[1]);
    }

    // Roll sparkline
    let roll_data: Vec<u64> = state
        .attitude_history
        .iter()
        .map(|(_, r)| (*r as i32 + 18000) as u64)
        .collect();

    if !roll_data.is_empty() {
        let roll_spark = Sparkline::default()
            .block(Block::default().title(" Roll ").borders(Borders::TOP))
            .data(&roll_data)
            .style(Style::default().fg(Color::Magenta));
        f.render_widget(roll_spark, chunks[2]);
    }
}

fn draw_rc_channels(f: &mut Frame, area: Rect, state: &AppState) {
    let block = Block::default()
        .title(" RC Channels ")
        .borders(Borders::ALL);
    let inner = block.inner(area);
    f.render_widget(block, area);

    let channels = match &state.rc_channels {
        Some(rc) => &rc.channels,
        None => {
            let empty = Paragraph::new(Line::from(Span::styled(
                "  No RC data",
                Style::default().fg(Color::Gray),
            )));
            f.render_widget(empty, inner);
            return;
        }
    };

    let labels: [(&str, Color); 8] = [
        ("CH1 Roll ", Color::Yellow),
        ("CH2 Pitch", Color::Yellow),
        ("CH3 Throt", Color::Green),
        ("CH4 Yaw  ", Color::Cyan),
        ("CH5 Aux1 ", Color::Gray),
        ("CH6 Aux2 ", Color::Gray),
        ("CH7 Aux3 ", Color::Gray),
        ("CH8 Aux4 ", Color::Gray),
    ];

    // Each gauge needs 2 rows (1 for label+gauge, 1 spacing)
    let constraints: Vec<Constraint> = labels
        .iter()
        .map(|_| Constraint::Length(2))
        .chain(core::iter::once(Constraint::Min(0)))
        .collect();

    let rows = Layout::default()
        .direction(Direction::Vertical)
        .constraints(constraints)
        .split(inner);

    for (i, (label, color)) in labels.iter().enumerate() {
        let value = channels[i];
        let ratio = (value as f64 / 2047.0).clamp(0.0, 1.0);
        let gauge = Gauge::default()
            .block(Block::default())
            .gauge_style(Style::default().fg(*color))
            .ratio(ratio)
            .label(format!("{label} {value:>4}"));
        f.render_widget(gauge, rows[i]);
    }
}

fn draw_logs(f: &mut Frame, area: Rect, state: &AppState) {
    let block = Block::default().title(" Logs ").borders(Borders::ALL);
    let inner = block.inner(area);
    f.render_widget(block, area);

    let log_lines: Vec<Line> = state
        .logs
        .iter()
        .rev()
        .take(inner.height as usize)
        .collect::<Vec<_>>()
        .into_iter()
        .rev()
        .map(|log| {
            let (label, color) = match log.level {
                0 => ("TRACE", Color::Gray),
                1 => ("DEBUG", Color::Blue),
                2 => ("INFO ", Color::Green),
                3 => ("WARN ", Color::Yellow),
                4 => ("ERROR", Color::Red),
                _ => ("?????", Color::White),
            };
            Line::from(vec![
                Span::styled(format!("[{label}]"), Style::default().fg(color)),
                Span::raw(format!(" {}", log_code_text(log.code))),
            ])
        })
        .collect();

    if log_lines.is_empty() {
        let empty = Paragraph::new(Line::from(Span::styled(
            "  No logs received",
            Style::default().fg(Color::Gray),
        )));
        f.render_widget(empty, inner);
    } else {
        let log_widget = Paragraph::new(log_lines).wrap(Wrap { trim: false });
        f.render_widget(log_widget, inner);
    }
}

fn log_code_text(code: u16) -> &'static str {
    match code {
        // GNSS (1–9)
        1 => "GNSS: first GGA received",
        2 => "GNSS: periodic update",
        3 => "GNSS: UART error",
        // Safety (10–19)
        10 => "Motors ARMED",
        11 => "Motors DISARMED",
        12 => "EMERGENCY STOP",
        // CRSF telemetry TX (20–29)
        20 => "CRSF TX: telemetry started",
        21 => "CRSF TX: first second OK",
        22 => "CRSF TX: UART write error",
        23 => "CRSF TX: running (periodic)",
        // ULog (30–39)
        30 => "ULog: recording started",
        31 => "ULog: init failed",
        32 => "ULog: not compiled in",
        33 => "ULog: recording stopped",
        34 => "ULog: flash erased",
        // IMU / sensors (40–49)
        40 => "IMU: init failed",
        41 => "IMU: FIFO overflow",
        42 => "IMU: read errors",
        43 => "Magnetometer: init failed",
        44 => "Barometer: init failed",
        // CRSF receiver (50–59)
        50 => "CRSF RX: first frame received",
        51 => "CRSF RX: UART error",
        // Flash storage (60–69)
        60 => "Flash: ULog write failed",
        61 => "Flash: ULog erase failed",
        62 => "Flash: ULog write timeout",
        // Supervisor (70–79)
        70 => "Supervisor: Core1 unhealthy",
        71 => "Supervisor: Core1 restored",
        // Flight state (80–89)
        80 => "Attitude data stale",
        81 => "ULog: RC switch ON",
        82 => "ULog: RC switch OFF",
        _ => "unknown",
    }
}

fn draw_command(f: &mut Frame, area: Rect, state: &AppState) {
    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([Constraint::Length(3), Constraint::Length(2)])
        .split(area);

    // Command input
    let input_style = Style::default().fg(Color::Cyan);
    let input = Paragraph::new(Line::from(vec![
        Span::styled("> ", input_style),
        Span::raw(&state.command_input),
    ]))
    .block(Block::default().borders(Borders::ALL).title(" Command "));
    f.render_widget(input, chunks[0]);

    // Set cursor position
    let cursor_x = chunks[0].x + 3 + state.command_input.len() as u16;
    let cursor_y = chunks[0].y + 1;
    f.set_cursor_position((cursor_x, cursor_y));

    // Status/help line — keep message visible while background task is running
    let has_bg_task = state.background_task.is_some();
    let help_text = if let Some((msg, when)) = &state.status_message {
        if has_bg_task || when.elapsed().as_secs() < 10 {
            msg.clone()
        } else {
            default_help()
        }
    } else {
        default_help()
    };

    let help_color = if has_bg_task {
        Color::Yellow
    } else if help_text != default_help() {
        Color::Cyan
    } else {
        Color::Gray
    };

    let help = Paragraph::new(Line::from(Span::styled(
        format!(" {help_text}"),
        Style::default().fg(help_color),
    )));
    f.render_widget(help, chunks[1]);
}

fn default_help() -> String {
    "arm disarm throttle elevon mode estop status perf cal quit | ? for help".into()
}
