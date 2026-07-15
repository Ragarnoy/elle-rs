//! TUI rendering with ratatui

use ratatui::Frame;
use ratatui::layout::{Constraint, Direction, Layout, Rect};
use ratatui::style::{Color, Modifier, Style};
use ratatui::text::{Line, Span};
use ratatui::widgets::canvas::{Canvas, Line as CanvasLine};
use ratatui::widgets::{Block, Borders, Gauge, Paragraph, Wrap};

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

    let autotune_span = match state.status.map(|s| s.autotune_state).unwrap_or(0) {
        1 => Span::styled(
            " | AT PITCH",
            Style::default()
                .fg(Color::Yellow)
                .add_modifier(Modifier::BOLD),
        ),
        2 => Span::styled(
            " | AT ROLL",
            Style::default()
                .fg(Color::Yellow)
                .add_modifier(Modifier::BOLD),
        ),
        3 => Span::styled(
            " | AT DONE",
            Style::default()
                .fg(Color::Green)
                .add_modifier(Modifier::BOLD),
        ),
        4 => Span::styled(
            " | AT ERR",
            Style::default().fg(Color::Red).add_modifier(Modifier::BOLD),
        ),
        _ => Span::raw(""),
    };

    // Blink REC at ~1Hz using sub-second parity
    let rec_span = if state.ulog_recording {
        let blink_on = (std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_millis()
            / 500)
            .is_multiple_of(2);
        if blink_on {
            Span::styled(
                " REC",
                Style::default().fg(Color::Red).add_modifier(Modifier::BOLD),
            )
        } else {
            Span::styled(" REC", Style::default().fg(Color::DarkGray))
        }
    } else {
        Span::styled(" REC", Style::default().fg(Color::DarkGray))
    };

    // Device wall clock — AON timer is seeded with compile-time Unix epoch
    let time_str = state
        .device_time_ms
        .map(|ms| {
            let total_secs = (ms / 1000) as i64;
            let h = ((total_secs % 86400) / 3600) as u32;
            let m = ((total_secs % 3600) / 60) as u32;
            let s = (total_secs % 60) as u32;
            format!("{h:02}:{m:02}:{s:02}")
        })
        .unwrap_or_else(|| "--:--:--".into());

    // Render the block first, then split its inner area
    let block = Block::default().borders(Borders::ALL);
    let inner = block.inner(area);
    f.render_widget(block, area);

    let header_cols = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([
            Constraint::Min(0),
            Constraint::Length(time_str.len() as u16 + 1),
        ])
        .split(inner);

    let left = Paragraph::new(Line::from(vec![
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
        autotune_span,
        Span::raw(" |"),
        rec_span,
        Span::raw(" | "),
        Span::styled(connected_text, connected_style),
    ]));

    let right = Paragraph::new(Line::from(Span::styled(
        format!("{time_str} "),
        Style::default().fg(Color::Gray),
    )))
    .alignment(ratatui::layout::Alignment::Right);

    f.render_widget(left, header_cols[0]);
    f.render_widget(right, header_cols[1]);
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
            Constraint::Length(14), // attitude data + mag + heading + baro + gnss + rc age
            Constraint::Length(4),  // controller output + engine + EDT
            Constraint::Min(8),     // artificial horizon canvas
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

    let rc_age_spans: Vec<Span> = if let Some(s) = state.status {
        let age = s.rc_age_ms;
        let color = if age < 100 {
            Color::Green
        } else if age < 200 {
            Color::Yellow
        } else {
            Color::Red
        };
        vec![
            Span::raw("  RC age: "),
            Span::styled(format!("{}ms", age), Style::default().fg(color)),
        ]
    } else {
        vec![Span::styled(
            "  RC age: ---",
            Style::default().fg(Color::Gray),
        )]
    };

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
            format!(
                "  Alt:  {:.1}m | Vario: {:+.1} m/s",
                b.altitude_m, b.vario_ms
            ),
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
        Line::from(rc_age_spans),
    ];

    f.render_widget(Paragraph::new(attitude_text), chunks[0]);

    // Controller output
    let ctrl_lines = if let Some(c) = state.controller_output {
        let err_str = if let Some(a) = state.attitude {
            let pitch_err = (c.pitch_setpoint_cdeg - a.pitch_cdeg) as f32 / 100.0;
            let roll_err = (c.roll_setpoint_cdeg - a.roll_cdeg) as f32 / 100.0;
            format!(
                "  Err: P={:+.1}\u{00B0} R={:+.1}\u{00B0}",
                pitch_err, roll_err
            )
        } else {
            String::new()
        };

        const POLE_PAIRS: u32 = 7; // 14-pole motor
        let (engine_line, edt_line) = if let Some(e) = state.engine {
            let l_rpm_str = if e.left.valid {
                let rpm = e.left.erpm / POLE_PAIRS;
                format!("{rpm} RPM")
            } else {
                "STALE".into()
            };
            let r_rpm_str = if e.right.valid {
                let rpm = e.right.erpm / POLE_PAIRS;
                format!("{rpm} RPM")
            } else {
                "STALE".into()
            };
            let l_target = if e.left.target_erpm > 0 {
                format!("\u{2192}{}", e.left.target_erpm / POLE_PAIRS)
            } else {
                String::new()
            };
            let r_target = if e.right.target_erpm > 0 {
                format!("\u{2192}{}", e.right.target_erpm / POLE_PAIRS)
            } else {
                String::new()
            };
            let edt_line = if e.left.voltage_mv > 0 || e.right.voltage_mv > 0 {
                Some(format!(
                    "  EDT: {:.1}V {:.1}A {}°C | {:.1}V {:.1}A {}°C",
                    e.left.voltage_mv as f32 / 1000.0,
                    e.left.current_ma as f32 / 1000.0,
                    e.left.temperature,
                    e.right.voltage_mv as f32 / 1000.0,
                    e.right.current_ma as f32 / 1000.0,
                    e.right.temperature,
                ))
            } else {
                None
            };
            let eng = format!(
                "  Eng L: {l_rpm_str}{l_target} (cmd:{}) | R: {r_rpm_str}{r_target} (cmd:{})",
                e.left.throttle, e.right.throttle,
            );
            (eng, edt_line)
        } else {
            ("  Eng: ---".into(), None)
        };

        let mut lines = vec![
            Line::from(format!(
                "  PID: P={:+.3} R={:+.3}{err_str}",
                c.pitch_correction_cp as f32 / 10000.0,
                c.roll_correction_cp as f32 / 10000.0,
            )),
            Line::from(format!(
                "  Elevon L={} R={}  Eng L={} R={}",
                c.elevon_left_us, c.elevon_right_us, c.engine_left_dshot, c.engine_right_dshot,
            )),
            Line::from(engine_line),
        ];
        if let Some(edt) = edt_line {
            lines.push(Line::from(edt));
        }
        if c.heading_hold_active {
            lines.push(Line::from(Span::styled(
                format!(
                    "  Heading hold: {:.1}\u{00B0} (err {:+.1}\u{00B0})",
                    c.heading_target_cdeg as f32 / 100.0,
                    c.heading_error_cdeg as f32 / 100.0,
                ),
                Style::default()
                    .fg(Color::Blue)
                    .add_modifier(Modifier::BOLD),
            )));
        }
        lines
    } else {
        vec![Line::from("  Controller: ---")]
    };
    f.render_widget(Paragraph::new(ctrl_lines), chunks[1]);

    // Artificial horizon with PID correction arrows
    draw_horizon(f, chunks[2], state);
}

fn draw_horizon(f: &mut Frame, area: Rect, state: &AppState) {
    let pitch_deg = state
        .attitude
        .map(|a| a.pitch_cdeg as f64 / 100.0)
        .unwrap_or(0.0);
    let roll_deg = state
        .attitude
        .map(|a| a.roll_cdeg as f64 / 100.0)
        .unwrap_or(0.0);

    let (pitch_cp, roll_cp) = state
        .controller_output
        .map(|c| (c.pitch_correction_cp as i32, c.roll_correction_cp as i32))
        .unwrap_or((0, 0));

    let roll_rad = -roll_deg.to_radians();
    let cos_r = roll_rad.cos();
    let sin_r = roll_rad.sin();
    let pitch_shift = -pitch_deg;

    let pid_color = |mag: i32| -> Color {
        let abs = mag.unsigned_abs();
        if abs >= 3000 {
            Color::Red
        } else if abs >= 1000 {
            Color::Yellow
        } else {
            Color::Green
        }
    };

    let canvas = Canvas::default()
        .block(Block::default().title(" Horizon ").borders(Borders::TOP))
        .x_bounds([-100.0, 100.0])
        .y_bounds([-45.0, 45.0])
        .paint(move |ctx| {
            // Horizon line (full width, rotated by roll, shifted by pitch)
            let half_w = 100.0;
            ctx.draw(&CanvasLine {
                x1: -half_w * cos_r,
                y1: pitch_shift - half_w * sin_r,
                x2: half_w * cos_r,
                y2: pitch_shift + half_w * sin_r,
                color: Color::White,
            });

            // Pitch ladder at ±10°, ±20°, ±30°
            let ladder_half = 25.0;
            for &angle in &[-30.0, -20.0, -10.0, 10.0, 20.0, 30.0_f64] {
                let y_off = pitch_shift + (-angle);
                ctx.draw(&CanvasLine {
                    x1: -ladder_half * cos_r
                        + (y_off * sin_r).copysign(-1.0) * ladder_half / half_w,
                    y1: y_off - ladder_half * sin_r,
                    x2: ladder_half * cos_r + (y_off * sin_r).copysign(1.0) * ladder_half / half_w,
                    y2: ladder_half.mul_add(sin_r, y_off),
                    color: Color::DarkGray,
                });
                // Simplified ladder: short horizontal segments at the pitch offset
                let lx = ladder_half + 2.0;
                ctx.print(
                    lx * cos_r,
                    y_off + lx * sin_r,
                    ratatui::text::Line::from(Span::styled(
                        format!("{:+.0}", angle),
                        Style::default().fg(Color::DarkGray),
                    )),
                );
            }

            // Center reference (fixed aircraft symbol)
            ctx.draw(&CanvasLine {
                x1: -18.0,
                y1: 0.0,
                x2: -5.0,
                y2: 0.0,
                color: Color::Cyan,
            });
            ctx.draw(&CanvasLine {
                x1: 5.0,
                y1: 0.0,
                x2: 18.0,
                y2: 0.0,
                color: Color::Cyan,
            });
            ctx.draw(&CanvasLine {
                x1: 0.0,
                y1: -2.0,
                x2: 0.0,
                y2: 2.0,
                color: Color::Cyan,
            });

            // PID pitch arrow — fixed position on right edge, direction indicates sign
            if pitch_cp.unsigned_abs() >= 10 {
                let arrow = if pitch_cp > 0 { "^" } else { "v" };
                ctx.print(
                    88.0,
                    0.0,
                    ratatui::text::Line::from(Span::styled(
                        arrow,
                        Style::default()
                            .fg(pid_color(pitch_cp))
                            .add_modifier(Modifier::BOLD),
                    )),
                );
            }

            // PID roll arrow — fixed position on bottom, direction indicates sign
            if roll_cp.unsigned_abs() >= 10 {
                let arrow = if roll_cp > 0 { ">>>" } else { "<<<" };
                ctx.print(
                    -8.0,
                    -38.0,
                    ratatui::text::Line::from(Span::styled(
                        arrow,
                        Style::default()
                            .fg(pid_color(roll_cp))
                            .add_modifier(Modifier::BOLD),
                    )),
                );
            }

            // Debug: show raw correction values on horizon
            ctx.print(
                -95.0,
                -40.0,
                ratatui::text::Line::from(Span::styled(
                    format!("P:{pitch_cp} R:{roll_cp}"),
                    Style::default().fg(Color::DarkGray),
                )),
            );
        });

    f.render_widget(canvas, area);
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

const fn log_code_text(code: u16) -> &'static str {
    match code {
        // GNSS (1–9)
        1 => "GNSS: first GGA received",
        2 => "GNSS: periodic update",
        3 => "GNSS: UART error",
        // Safety (10–19)
        10 => "Motors ARMED",
        11 => "Motors DISARMED",
        12 => "EMERGENCY STOP",
        13 => "RC: signal warning",
        14 => "RC: SIGNAL LOST",
        15 => "RC: signal restored",
        16 => "Kill switch ENGAGED",
        17 => "Kill switch released",
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
        // Autotune (90–99)
        90 => "Autotune: started",
        91 => "Autotune: complete",
        92 => "Autotune: aborted (RC)",
        93 => "Autotune: safety abort",
        // PID profile persistence (100–109)
        100 => "PID: saved to flash",
        101 => "PID: save failed",
        102 => "PID: loaded from flash",
        103 => "PID: no saved profile",
        // Mag calibration (110–119)
        110 => "Mag cal: started",
        111 => "Mag cal: complete",
        112 => "Mag cal: failed",
        113 => "Mag cal: saved to flash",
        114 => "Mag cal: cleared",
        115 => "Mag cal: loaded from flash",
        116 => "Mag cal: no saved cal",
        // Tap detection (120–129)
        120 => "Double-tap: armed",
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

    // Status/help line — show status messages with timeout, otherwise empty
    let has_bg_task = state.background_task.is_some();
    let help_text = if let Some((msg, when)) = &state.status_message {
        if has_bg_task || when.elapsed().as_secs() < 10 {
            msg.clone()
        } else {
            String::new()
        }
    } else {
        String::new()
    };

    let help_color = if has_bg_task {
        Color::Yellow
    } else {
        Color::Cyan
    };

    let help = Paragraph::new(Line::from(Span::styled(
        format!(" {help_text}"),
        Style::default().fg(help_color),
    )));
    f.render_widget(help, chunks[1]);
}
