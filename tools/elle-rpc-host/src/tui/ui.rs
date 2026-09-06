//! TUI rendering with ratatui

use ratatui::Frame;
use ratatui::layout::{Constraint, Direction, Layout, Rect};
use ratatui::style::{Color, Modifier, Style};
use ratatui::text::{Line, Span};
use ratatui::widgets::{Block, Borders, Gauge, Paragraph, Wrap};

use elle_rpc_icd::EngineUnit;

use super::horizon::Horizon;
use super::state::AppState;

// Semantic colours. A full theme module is a larger job; these at least keep the
// health-coded values consistent, and give one place to change them.
const MUTED: Color = Color::DarkGray;
const LABEL: Color = Color::Gray;
const OK: Color = Color::Green;
const WARN: Color = Color::Yellow;
const CRIT: Color = Color::Red;

/// A "no data yet" line, dimmed so empty fields stop shouting.
fn muted_line(text: &str) -> Line<'static> {
    Line::from(Span::styled(text.to_string(), Style::default().fg(MUTED)))
}

/// Label + value, with the value carrying the health colour.
fn kv(label: &str, value: String, color: Color) -> Line<'static> {
    Line::from(vec![
        Span::styled(label.to_string(), Style::default().fg(LABEL)),
        Span::styled(value, Style::default().fg(color)),
    ])
}

/// Green below `warn`, yellow below `crit`, red at or above `crit`.
fn scale(value: f64, warn: f64, crit: f64) -> Color {
    if value >= crit {
        CRIT
    } else if value >= warn {
        WARN
    } else {
        OK
    }
}

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
            Constraint::Length(5),  // controller output + engine + EDT + heading hold
            Constraint::Min(8),     // artificial horizon
        ])
        .split(inner);

    // Attitude data (from polled GetAttitude endpoint)
    let attitude_lines = if let Some(a) = state.attitude {
        let pitch = f64::from(a.pitch_cdeg) / 100.0;
        let roll = f64::from(a.roll_cdeg) / 100.0;
        let yaw = f64::from(a.yaw_cdeg) / 100.0;
        vec![
            kv(
                "  Pitch: ",
                format!("{pitch:>7.2}\u{00B0}"),
                scale(pitch.abs(), 20.0, 45.0),
            ),
            kv(
                "  Roll:  ",
                format!("{roll:>7.2}\u{00B0}"),
                scale(roll.abs(), 20.0, 45.0),
            ),
            kv("  Yaw:   ", format!("{yaw:>7.2}\u{00B0}"), Color::White),
        ]
    } else {
        vec![
            muted_line("  Pitch:     ---"),
            muted_line("  Roll:      ---"),
            muted_line("  Yaw:       ---"),
        ]
    };

    let perf_line = state.performance.map_or_else(
        || muted_line("  Loop: ---"),
        |p| {
            kv(
                "  Loop: ",
                format!(
                    "{}us avg / {}us max",
                    p.control_loop_avg_us, p.control_loop_max_us
                ),
                // 77Hz control loop => 13ms budget per iteration.
                scale(f64::from(p.control_loop_max_us), 10_000.0, 13_000.0),
            )
        },
    );

    let imu_line = state.status.map_or_else(
        || muted_line("  IMU: ---"),
        |s| {
            Line::from(vec![
                Span::styled("  IMU: ", Style::default().fg(LABEL)),
                if s.imu_calibrated {
                    Span::styled("Cal", Style::default().fg(OK))
                } else {
                    Span::styled("NO CAL", Style::default().fg(CRIT))
                },
                Span::styled(" | Errors: ", Style::default().fg(LABEL)),
                Span::styled(
                    s.imu_error_count.to_string(),
                    Style::default().fg(if s.imu_error_count == 0 { OK } else { WARN }),
                ),
            ])
        },
    );

    let rc_age_line = state.status.map_or_else(
        || muted_line("  RC age: ---"),
        |s| {
            kv(
                "  RC age: ",
                format!("{}ms", s.rc_age_ms),
                // Matches the firmware's RC_WARNING_MS / RC_TIMEOUT_MS staging.
                scale(f64::from(s.rc_age_ms), 200.0, 300.0),
            )
        },
    );

    let (mag_line, heading_line) = state.magnetometer.map_or_else(
        || (muted_line("  Mag: ---"), muted_line("  Heading: ---")),
        |m| {
            let heading = f64::from(m.y).atan2(f64::from(m.x)).to_degrees();
            let heading = if heading < 0.0 {
                heading + 360.0
            } else {
                heading
            };
            (
                kv(
                    "  Mag: ",
                    format!("X={:>7} Y={:>7} Z={:>7}", m.x, m.y, m.z),
                    Color::White,
                ),
                kv(
                    "  Heading: ",
                    format!("{heading:>6.1}\u{00B0}"),
                    Color::White,
                ),
            )
        },
    );

    let (baro_line1, baro_line2) = state.barometer.map_or_else(
        || (muted_line("  Baro: ---"), muted_line("  Alt:  ---")),
        |b| {
            (
                Line::from(vec![
                    Span::styled("  Baro: ", Style::default().fg(LABEL)),
                    Span::styled(
                        format!("{:.1} hPa", b.pressure_hpa),
                        Style::default().fg(Color::White),
                    ),
                    Span::styled(" | ", Style::default().fg(LABEL)),
                    Span::styled(
                        format!("{:.1}\u{00B0}C", b.temperature_c),
                        Style::default().fg(scale(f64::from(b.temperature_c), 50.0, 70.0)),
                    ),
                ]),
                kv(
                    "  Alt:  ",
                    format!("{:.1}m | Vario: {:+.1} m/s", b.altitude_m, b.vario_ms),
                    Color::White,
                ),
            )
        },
    );

    let (gnss_pos_line, gnss_fix_line, gnss_alt_line) = state.gnss.map_or_else(
        || {
            (
                muted_line("  GPS: ---"),
                muted_line("  Fix: ---"),
                muted_line("  Alt: ---"),
            )
        },
        |g| {
            let (fix_str, fix_color) = match g.fix_quality {
                0 => ("No fix", CRIT),
                1 => ("GPS", OK),
                2 => ("DGPS", OK),
                _ => ("Other", WARN),
            };
            (
                kv(
                    "  GPS: ",
                    format!("{:.6}, {:.6}", g.latitude, g.longitude),
                    if g.fix_quality == 0 {
                        MUTED
                    } else {
                        Color::White
                    },
                ),
                Line::from(vec![
                    Span::styled("  Fix: ", Style::default().fg(LABEL)),
                    Span::styled(fix_str, Style::default().fg(fix_color)),
                    Span::styled(" | Sats: ", Style::default().fg(LABEL)),
                    Span::styled(
                        g.num_satellites.to_string(),
                        // A 3D fix needs 4; 6+ is comfortable.
                        Style::default().fg(if g.num_satellites >= 6 {
                            OK
                        } else if g.num_satellites >= 4 {
                            WARN
                        } else {
                            CRIT
                        }),
                    ),
                    Span::styled(" | HDOP: ", Style::default().fg(LABEL)),
                    Span::styled(
                        format!("{:.1}", g.hdop),
                        Style::default().fg(scale(f64::from(g.hdop), 2.0, 5.0)),
                    ),
                ]),
                kv(
                    "  Alt: ",
                    format!("{:.1}m MSL", g.altitude_m),
                    if g.fix_quality == 0 {
                        MUTED
                    } else {
                        Color::White
                    },
                ),
            )
        },
    );

    let mut attitude_text = attitude_lines;
    attitude_text.extend([
        Line::from(""),
        mag_line,
        heading_line,
        baro_line1,
        baro_line2,
        gnss_pos_line,
        gnss_fix_line,
        gnss_alt_line,
        perf_line,
        imu_line,
        rc_age_line,
    ]);

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
            let rpm_span = |u: &EngineUnit| {
                if u.valid {
                    Span::styled(
                        format!("{} RPM", u.erpm / POLE_PAIRS),
                        Style::default().fg(Color::White),
                    )
                } else {
                    Span::styled("STALE", Style::default().fg(CRIT))
                }
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
            let edt_spans = |u: &EngineUnit| {
                vec![
                    Span::styled(
                        format!(
                            "{:.1}V {:.1}A ",
                            u.voltage_mv as f32 / 1000.0,
                            u.current_ma as f32 / 1000.0
                        ),
                        Style::default().fg(Color::White),
                    ),
                    Span::styled(
                        format!("{}\u{00B0}C", u.temperature),
                        // ESC/motor temperature — the prop swap was about heat.
                        Style::default().fg(scale(f64::from(u.temperature), 70.0, 90.0)),
                    ),
                ]
            };
            let edt_line = if e.left.voltage_mv > 0 || e.right.voltage_mv > 0 {
                let mut spans = vec![Span::styled("  EDT: ", Style::default().fg(LABEL))];
                spans.extend(edt_spans(&e.left));
                spans.push(Span::styled(" | ", Style::default().fg(LABEL)));
                spans.extend(edt_spans(&e.right));
                Some(Line::from(spans))
            } else {
                None
            };
            let eng = Line::from(vec![
                Span::styled("  Eng L: ", Style::default().fg(LABEL)),
                rpm_span(&e.left),
                Span::styled(l_target, Style::default().fg(Color::Cyan)),
                Span::styled(
                    format!(" (cmd:{})", e.left.throttle),
                    Style::default().fg(MUTED),
                ),
                Span::styled(" | R: ", Style::default().fg(LABEL)),
                rpm_span(&e.right),
                Span::styled(r_target, Style::default().fg(Color::Cyan)),
                Span::styled(
                    format!(" (cmd:{})", e.right.throttle),
                    Style::default().fg(MUTED),
                ),
            ]);
            (eng, edt_line)
        } else {
            (muted_line("  Eng: ---"), None)
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
            engine_line,
        ];
        if let Some(edt) = edt_line {
            lines.push(edt);
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
        vec![muted_line("  Controller: ---")]
    };
    f.render_widget(Paragraph::new(ctrl_lines), chunks[1]);

    // Artificial horizon with PID correction arrows
    draw_horizon(f, chunks[2], state);
}

fn draw_horizon(f: &mut Frame, area: Rect, state: &AppState) {
    let block = Block::default().title(" Horizon ").borders(Borders::ALL);
    let inner = block.inner(area);
    f.render_widget(block, area);

    let (pitch_cp, roll_cp) = state.controller_output.map_or((0, 0), |c| {
        (
            i32::from(c.pitch_correction_cp),
            i32::from(c.roll_correction_cp),
        )
    });

    f.render_widget(
        Horizon {
            pitch_deg: state
                .attitude
                .map_or(0.0, |a| f64::from(a.pitch_cdeg) / 100.0),
            roll_deg: state
                .attitude
                .map_or(0.0, |a| f64::from(a.roll_cdeg) / 100.0),
            pitch_cp,
            roll_cp,
        },
        inner,
    );
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
            let empty = Paragraph::new(muted_line("  No RC data"));
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
                0 => ("TRACE", MUTED),
                1 => ("DEBUG", Color::Blue),
                2 => ("INFO ", OK),
                3 => ("WARN ", WARN),
                4 => ("ERROR", CRIT),
                _ => ("?????", Color::White),
            };
            // Repeats are collapsed in state::push_log; show how many.
            let repeat = if log.count > 1 {
                Span::styled(format!(" \u{00D7}{}", log.count), Style::default().fg(WARN))
            } else {
                Span::raw("")
            };
            Line::from(vec![
                Span::styled(
                    log.at.format("%H:%M:%S ").to_string(),
                    Style::default().fg(MUTED),
                ),
                Span::styled(format!("[{label}] "), Style::default().fg(color)),
                Span::styled(
                    log_code_text(log.code),
                    Style::default().fg(if log.level >= 3 { color } else { Color::White }),
                ),
                repeat,
            ])
        })
        .collect();

    if log_lines.is_empty() {
        let empty = Paragraph::new(muted_line("  No logs received"));
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
