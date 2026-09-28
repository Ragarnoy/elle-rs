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
            Constraint::Length(16), // attitude data + mag + heading + baro + gnss + rc age
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
                // 12 ms control loop period (elle-config CONTROL_LOOP_PERIOD_MS).
                scale(f64::from(p.control_loop_max_us), 9_000.0, 12_000.0),
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

    let (gnss_pos_line, gnss_fix_line, gnss_alt_line, gnss_vel_line, gnss_src_line) =
        state.gnss.map_or_else(
            || {
                (
                    muted_line("  GPS: ---"),
                    muted_line("  Fix: ---"),
                    muted_line("  Alt: ---"),
                    muted_line("  Spd: ---"),
                    muted_line("  Src: ---"),
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
                        // Satellites in view, when the firmware reports it: `Sats`
                        // alone counts only those used in the fix, which stays at
                        // zero for the whole of acquisition.
                        Span::styled(
                            if g.sats_in_view > 0 {
                                format!("/{}", g.sats_in_view)
                            } else {
                                String::new()
                            },
                            Style::default().fg(MUTED),
                        ),
                        Span::styled(" | HDOP: ", Style::default().fg(LABEL)),
                        // NAV-PVT carries no DOP, so on the primary path `hdop`
                        // keeps its "unavailable" seed. Printing that as 99.9 in red
                        // reads as a failing receiver for an entire healthy flight.
                        if g.pvt_active {
                            Span::styled("---", Style::default().fg(MUTED))
                        } else {
                            Span::styled(
                                format!("{:.1}", g.hdop),
                                Style::default().fg(scale(f64::from(g.hdop), 2.0, 5.0)),
                            )
                        },
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
                    Line::from(vec![
                        Span::styled("  Spd: ", Style::default().fg(LABEL)),
                        Span::styled(
                            format!("{:.1} m/s", g.ground_speed_ms),
                            Style::default().fg(if g.fix_quality == 0 {
                                MUTED
                            } else {
                                Color::White
                            }),
                        ),
                        Span::styled(" | Trk: ", Style::default().fg(LABEL)),
                        Span::styled(
                            format!("{:.0}°", g.heading_motion_deg),
                            Style::default().fg(if g.fix_quality == 0 {
                                MUTED
                            } else {
                                Color::White
                            }),
                        ),
                        Span::styled(" | hAcc: ", Style::default().fg(LABEL)),
                        Span::styled(
                            // hAcc is 0 until the first NAV-PVT; show that as
                            // unknown rather than as a perfect fix.
                            if g.h_acc_m > 0.0 {
                                format!("{:.1}m", g.h_acc_m)
                            } else {
                                "---".to_string()
                            },
                            Style::default().fg(if g.h_acc_m > 0.0 {
                                scale(f64::from(g.h_acc_m), 3.0, 10.0)
                            } else {
                                MUTED
                            }),
                        ),
                    ]),
                    Line::from(vec![
                        Span::styled("  Src: ", Style::default().fg(LABEL)),
                        Span::styled(
                            if g.pvt_active { "NAV-PVT" } else { "NMEA" },
                            Style::default().fg(if g.pvt_active { OK } else { WARN }),
                        ),
                        Span::styled(" | ", Style::default().fg(LABEL)),
                        Span::styled(
                            if g.link_baud == 0 {
                                "---".to_string()
                            } else {
                                format!("{} baud", g.link_baud)
                            },
                            // 9600 means the baud switch did not take.
                            Style::default().fg(if g.link_baud >= 115_200 { OK } else { WARN }),
                        ),
                        Span::styled(" | ", Style::default().fg(LABEL)),
                        Span::styled(
                            if g.nav_rate_ms == 0 {
                                "---".to_string()
                            } else {
                                format!("{:.0} Hz", 1000.0 / f64::from(g.nav_rate_ms))
                            },
                            Style::default().fg(Color::White),
                        ),
                        Span::styled(" | ", Style::default().fg(LABEL)),
                        Span::styled(
                            gnss_cfg_text(g.cfg_mask),
                            Style::default().fg(if g.cfg_mask == elle_rpc_icd::GNSS_CFG_MASK_ALL {
                                OK
                            } else {
                                WARN
                            }),
                        ),
                    ]),
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
        gnss_vel_line,
        gnss_src_line,
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

/// Summarise the GNSS configuration mask, naming the first key that is not in
/// force.
///
/// The module NAKs per key, so a partial mask says exactly which setting did
/// not take — far more use than "config failed". A mask carrying
/// `GNSS_CFG_ABANDONED` is reported differently: the firmware stopped sending
/// after the module went quiet, so the clear bits are keys never tried, and
/// calling them rejected would send the reader after the wrong fault.
fn gnss_cfg_text(mask: u16) -> String {
    use elle_rpc_icd::{GNSS_CFG_ABANDONED, GNSS_CFG_KEY_NAMES, GNSS_CFG_MASK_ALL};

    let abandoned = mask & GNSS_CFG_ABANDONED != 0;
    let applied = mask & GNSS_CFG_MASK_ALL;

    if !abandoned && applied == GNSS_CFG_MASK_ALL {
        return "cfg ok".to_string();
    }
    let missing: Vec<&str> = GNSS_CFG_KEY_NAMES
        .iter()
        .enumerate()
        .filter(|(i, _)| applied & (1 << i) == 0)
        .map(|(_, name)| *name)
        .collect();
    if abandoned {
        return match missing.first() {
            // Configuration stopped at the first key we never heard back on.
            Some(first) => format!("cfg abandoned at {first} (module silent)"),
            None => "cfg abandoned (module silent)".to_string(),
        };
    }
    match missing.len() {
        0 => "cfg ok".to_string(),
        1 => format!("cfg: {} failed", missing[0]),
        n => format!("cfg: {} failed +{}", missing[0], n - 1),
    }
}

const fn log_code_text(code: u16) -> &'static str {
    match code {
        // GNSS (1–9)
        1 => "GNSS: first fix",
        2 => "GNSS: periodic update",
        3 => "GNSS: UART error",
        4 => "GNSS: config rejected (NAK)",
        5 => "GNSS: NAV-PVT acquired",
        6 => "GNSS: PVT stale, NMEA fallback",
        7 => "GNSS: 115200 baud, 5Hz",
        8 => "GNSS: baud switch failed, 9600",
        140 => "GNSS: config key unanswered",
        141 => "GNSS: config only partly applied",
        9 => "GNSS: NO DATA from module",
        // Safety (10–19)
        10 => "Motors ARMED",
        11 => "Motors DISARMED",
        12 => "EMERGENCY STOP",
        13 => "RC: signal warning",
        14 => "RC: SIGNAL LOST",
        15 => "RC: signal restored",
        16 => "Kill switch ENGAGED",
        17 => "Kill switch released",
        18 => "Arm refused: throttle not at zero",
        // CRSF telemetry TX (20–29)
        20 => "CRSF TX: telemetry started",
        21 => "CRSF TX: first second OK",
        22 => "CRSF TX: UART write error",
        23 => "CRSF TX: running (periodic)",
        // ULog (30–39)
        30 => "ULog: recording started",
        31 => "ULog: init failed",
        33 => "ULog: recording stopped",
        34 => "ULog: flash erased",
        // IMU / sensors (40–49)
        40 => "IMU: init failed",
        41 => "IMU: FIFO overflow",
        42 => "IMU: read errors",
        43 => "Magnetometer: init failed",
        44 => "Barometer: init failed",
        45 => "IMU: caught up queued samples",
        46 => "IMU: gyro bias measured",
        47 => "IMU: gyro bias failed (keep still at boot)",
        48 => "I2C0 error: mag and baro disabled (AHRS 6-DOF)",
        // CRSF receiver (50–59)
        50 => "CRSF RX: first frame received",
        51 => "CRSF RX: UART error",
        // Flash storage (60–69)
        61 => "Flash: ULog erase failed",
        62 => "Flash: ULog write timeout",
        63 => "Flash: command refused while armed (disarm first)",
        // Supervisor (70–79)
        70 => "Supervisor: Core1 unhealthy",
        71 => "Supervisor: Core1 restored",
        // Flight state (80–89)
        80 => "Attitude data stale",
        81 => "ULog: recording started (auto, SD)",
        // Autotune (90–99)
        90 => "Autotune: started",
        91 => "Autotune: complete",
        92 => "Autotune: aborted (RC)",
        93 => "Autotune: safety abort",
        94 => "Autotune: result rejected (gains restored)",
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
        120 => "IMU: double-tap detected (calibration gesture)",
        // Heading hold (130–139)
        130 => "Heading hold: engaged",
        131 => "Heading hold: disengaged",
        132 => "Heading hold: target set",
        // Level calibration (150–159)
        150 => "Level cal: started",
        151 => "Level cal: complete",
        152 => "Level cal: failed (moved, or refused while armed)",
        153 => "Level cal: failed (tilt beyond limit)",
        154 => "Level cal: saved to flash",
        155 => "Level cal: flash save failed",
        156 => "Level cal: cleared",
        157 => "Level cal: loaded from flash",
        158 => "Level cal: no saved cal",
        // ESC link (160–169)
        160 => "ESC left: stopped replying (power loss or restart?)",
        161 => "ESC right: stopped replying (power loss or restart?)",
        162 => "ESC left: spin direction + EDT re-sent (reappeared, or no EDT)",
        163 => "ESC right: spin direction + EDT re-sent (reappeared, or no EDT)",
        164 => "ESC left: still no EDT after retries, spin direction unconfirmed",
        165 => "ESC right: still no EDT after retries, spin direction unconfirmed",
        _ => "unknown",
    }
}

fn draw_command(f: &mut Frame, area: Rect, state: &AppState) {
    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Length(3), // input box
            Constraint::Length(1), // transient status message
            Constraint::Length(1), // permanent help bar
        ])
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

    // Transient status message, cleared after 10s unless a background task is running
    let has_bg_task = state.background_task.is_some();
    let status_text = state
        .status_message
        .as_ref()
        .filter(|(_, when)| has_bg_task || when.elapsed().as_secs() < 10)
        .map_or_else(String::new, |(msg, _)| msg.clone());

    let status_color = if has_bg_task { WARN } else { Color::Cyan };
    f.render_widget(
        Paragraph::new(Line::from(Span::styled(
            format!(" {status_text}"),
            Style::default().fg(status_color),
        ))),
        chunks[1],
    );

    draw_help_bar(f, chunks[2]);
}

/// Always-on key and command reference along the bottom edge. Commands come from
/// the command table so this cannot drift from what the parser accepts.
fn draw_help_bar(f: &mut Frame, area: Rect) {
    const KEYS: &str = " Enter run \u{00B7} Tab complete \u{00B7} \u{2191}\u{2193} history \u{00B7} Esc clear \u{00B7} ^C quit";

    let mut spans = vec![Span::styled(KEYS, Style::default().fg(LABEL))];

    // Fill the remaining width with as many command names as fit, so narrow
    // terminals keep the keys rather than truncating mid-word.
    let commands = super::commands::command_names();
    let mut remaining = (area.width as usize).saturating_sub(KEYS.chars().count() + 4);
    let mut listed = String::new();
    for name in commands {
        if name.len() + 2 > remaining {
            break;
        }
        listed.push_str(name);
        listed.push(' ');
        remaining -= name.len() + 1;
    }
    if !listed.is_empty() {
        spans.push(Span::styled("  \u{2502}  ", Style::default().fg(MUTED)));
        spans.push(Span::styled(listed, Style::default().fg(MUTED)));
    }

    f.render_widget(Paragraph::new(Line::from(spans)), area);
}

#[cfg(test)]
mod tests {
    use super::*;
    use ratatui::Terminal;
    use ratatui::backend::TestBackend;

    /// Render the command area at `width` and return its rows.
    fn command_rows(width: u16) -> Vec<String> {
        // Height matches the constraint draw() gives this area.
        let mut term = Terminal::new(TestBackend::new(width, 5)).unwrap();
        let state = AppState::new();
        term.draw(|f| draw_command(f, f.area(), &state)).unwrap();
        let buf = term.backend().buffer().clone();
        (0..buf.area.height)
            .map(|row| {
                (0..buf.area.width)
                    .map(|col| {
                        buf.cell((col, row))
                            .unwrap()
                            .symbol()
                            .chars()
                            .next()
                            .unwrap_or(' ')
                    })
                    .collect()
            })
            .collect()
    }

    #[test]
    fn help_bar_occupies_the_last_row() {
        let rows = command_rows(173);
        let last = rows.last().expect("command area must render");
        assert!(last.contains("Enter run"), "help bar missing: {last}");
        assert!(last.contains("autotune"), "command list missing: {last}");
    }

    /// A narrow terminal drops command names rather than truncating the key hints.
    #[test]
    fn help_bar_degrades_to_keys_only() {
        let last = command_rows(60).last().cloned().unwrap();
        assert!(last.contains("^C quit"), "keys were truncated: {last}");
        assert!(
            !last.contains('\u{2502}'),
            "command list should be dropped at this width: {last}"
        );
    }
}
