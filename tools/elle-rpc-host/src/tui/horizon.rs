//! Artificial horizon (attitude director indicator).
//!
//! Drawn as a plain `Widget` rather than a `Canvas` because braille canvas cells
//! can only be set, not filled — and a horizon without a sky/ground fill reads as
//! a stray line on black. Writing cell backgrounds directly gives a real ADI, and
//! half-block characters (`▀`) double the vertical resolution so the horizon edge
//! lands on the right half-row instead of snapping to whole rows.

use ratatui::buffer::Buffer;
use ratatui::layout::Rect;
use ratatui::style::{Color, Modifier, Style};
use ratatui::widgets::Widget;

/// Sign relating the firmware's roll to on-screen rotation.
///
/// `+1.0` follows the aviation convention: positive roll is right-wing-down, so
/// the ground rotates towards the right of the instrument and the horizon's right
/// end rises. If the display looks mirrored when you bank the airframe, the
/// firmware's roll sign is the opposite of this convention and only this constant
/// needs to flip — the geometry below stays correct either way.
const ROLL_SIGN: f64 = 1.0;

/// Vertical field of view, degrees from top to bottom of the instrument.
const VERTICAL_FOV_DEG: f64 = 60.0;

/// A terminal cell is roughly twice as tall as it is wide. Without this, a 45°
/// bank does not draw at 45° — the previous canvas version was out by ~1.4x.
const CELL_ASPECT: f64 = 2.0;

/// Half-length of a pitch ladder rung, in degrees.
///
/// Rungs are every 10°: at this vertical scale a 5° minor would sit under two
/// rows of its neighbour and the ladder turns to noise as soon as you bank.
const RUNG_HALF_DEG: f64 = 11.0;
const RUNG_STEP_DEG: f64 = 10.0;

const SKY: Color = Color::Rgb(56, 116, 180);
const GROUND: Color = Color::Rgb(122, 79, 44);
const HORIZON_LINE: Color = Color::Rgb(240, 240, 240);
const LADDER: Color = Color::Rgb(226, 226, 226);
/// Below-horizon rungs are dimmer, the terminal-friendly stand-in for the dashed
/// rungs a real PFD draws below the horizon.
const LADDER_DIM: Color = Color::Rgb(150, 150, 150);
const AIRCRAFT: Color = Color::Rgb(255, 214, 0);

pub struct Horizon {
    pub pitch_deg: f64,
    pub roll_deg: f64,
    /// PID pitch correction, centi-percent — drives the trend arrow.
    pub pitch_cp: i32,
    /// PID roll correction, centi-percent.
    pub roll_cp: i32,
}

/// Colour for a PID correction magnitude (centi-percent).
fn pid_color(mag: i32) -> Color {
    match mag.unsigned_abs() {
        3000.. => Color::Red,
        1000..3000 => Color::Yellow,
        _ => Color::Green,
    }
}

impl Widget for Horizon {
    #[allow(clippy::cast_possible_truncation, clippy::cast_precision_loss)]
    fn render(self, area: Rect, buf: &mut Buffer) {
        if area.width < 8 || area.height < 4 {
            return;
        }

        let w = f64::from(area.width);
        let h = f64::from(area.height);
        let deg_per_row = VERTICAL_FOV_DEG / h;
        let deg_per_col = deg_per_row / CELL_ASPECT;

        // Instrument centre, in cell coordinates relative to `area`.
        let cx = w / 2.0;
        let cy = h / 2.0;

        let roll_rad = (ROLL_SIGN * self.roll_deg).to_radians();
        let (sin_r, cos_r) = roll_rad.sin_cos();
        let pitch = self.pitch_deg;

        // Degrees below the horizon at a point offset (dx, dy) degrees from centre,
        // with dy measured downwards. Negative is sky.
        let below = |dx: f64, dy: f64| (dy - pitch).mul_add(cos_r, dx * sin_r);

        // Cell centre -> degrees from instrument centre.
        let dx_of = |col: u16| (f64::from(col) + 0.5 - cx) * deg_per_col;
        let dy_of = |row: u16, half: f64| (f64::from(row) + half - cy) * deg_per_row;

        // --- sky / ground fill, at half-row resolution -----------------------
        for row in 0..area.height {
            for col in 0..area.width {
                let dx = dx_of(col);
                let top = below(dx, dy_of(row, 0.25));
                let bottom = below(dx, dy_of(row, 0.75));

                let Some(cell) = buf.cell_mut((area.x + col, area.y + row)) else {
                    continue;
                };
                let color_of = |d: f64| if d > 0.0 { GROUND } else { SKY };

                if top.is_sign_positive() == bottom.is_sign_positive() {
                    cell.set_char(' ').set_bg(color_of(top));
                } else {
                    // The horizon crosses this cell: upper half-block in white gives
                    // a crisp line without costing a whole row of height.
                    cell.set_char('\u{2580}')
                        .set_fg(HORIZON_LINE)
                        .set_bg(color_of(bottom));
                }
            }
        }

        // --- horizon line and pitch ladder ------------------------------------
        // A rung for pitch angle `theta` is the locus where `below == -theta`.
        // With n = (sin_r, cos_r) the "down" normal and u = (cos_r, -sin_r) along
        // the horizon, that is the segment b + t*u for b = (pitch*cos_r - theta)*n.
        let rung_ends = |theta: f64, half: f64| {
            let base = pitch.mul_add(cos_r, -theta);
            let (bx, by) = (base * sin_r, base * cos_r);
            let at = move |t: f64| {
                (
                    cx + t.mul_add(cos_r, bx) / deg_per_col,
                    cy + (-t).mul_add(sin_r, by) / deg_per_row,
                )
            };
            (at(-half), at(half), at)
        };

        // The fill edge already separates sky from ground, but it lands on a
        // half-row boundary and vanishes when it coincides with a row edge. Draw
        // the horizon explicitly so it is always present.
        let span = w.mul_add(deg_per_col, h * deg_per_row);
        let (a, b, _) = rung_ends(0.0, span);
        draw_line(buf, area, a, b, HORIZON_LINE);

        let mut theta: f64 = -30.0;
        while theta <= 30.0 {
            if theta.abs() > f64::EPSILON {
                let color = if theta > 0.0 { LADDER } else { LADDER_DIM };
                let (a, b, at) = rung_ends(theta, RUNG_HALF_DEG);
                draw_line(buf, area, a, b, color);

                // Labels are written horizontally from the rung ends: stepping the
                // characters along the rotated axis scatters them across rows.
                let label = format!("{:.0}", theta.abs());
                // Two cells of clearance keeps the labels clear of both the rung
                // and the fixed aircraft symbol at the centre.
                let pad = 2.0 * deg_per_col;
                let left = at(-RUNG_HALF_DEG - pad);
                let right = at(RUNG_HALF_DEG + pad);
                put_text(
                    buf,
                    area,
                    left.0 - label.chars().count() as f64,
                    left.1,
                    &label,
                    color,
                );
                put_text(buf, area, right.0, right.1, &label, color);
            }
            theta += RUNG_STEP_DEG;
        }

        // --- fixed aircraft symbol -------------------------------------------
        let mid_row = area.y + area.height / 2;
        let mid_col = area.x + area.width / 2;
        let wing = (area.width / 10).clamp(3, 5);
        for offset in 2..=2 + wing {
            for col in [mid_col.saturating_sub(offset), mid_col + offset] {
                if let Some(cell) = buf.cell_mut((col, mid_row)) {
                    cell.set_char('\u{2501}')
                        .set_fg(AIRCRAFT)
                        .set_style(Style::default().add_modifier(Modifier::BOLD));
                }
            }
        }
        if let Some(cell) = buf.cell_mut((mid_col, mid_row)) {
            cell.set_char('\u{25C6}').set_fg(AIRCRAFT);
        }

        // --- PID correction trend arrows --------------------------------------
        if self.pitch_cp.unsigned_abs() >= 10 {
            let arrow = if self.pitch_cp > 0 {
                '\u{25B2}'
            } else {
                '\u{25BC}'
            };
            if let Some(cell) = buf.cell_mut((area.x + area.width - 2, mid_row)) {
                cell.set_char(arrow).set_fg(pid_color(self.pitch_cp));
            }
        }
        if self.roll_cp.unsigned_abs() >= 10 {
            let arrow = if self.roll_cp > 0 {
                '\u{25B6}'
            } else {
                '\u{25C0}'
            };
            let row = area.y + area.height - 1;
            for i in 0..3 {
                if let Some(cell) = buf.cell_mut((mid_col.saturating_sub(1) + i, row)) {
                    cell.set_char(arrow).set_fg(pid_color(self.roll_cp));
                }
            }
        }
    }
}

/// Draw a line between two points given in fractional cell coordinates relative to
/// `area`, preserving whatever background the sky/ground fill put there. The glyph
/// follows the slope so a banked ladder reads as sloped rather than as stair-steps.
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
fn draw_line(buf: &mut Buffer, area: Rect, from: (f64, f64), to: (f64, f64), color: Color) {
    let (dx, dy) = (to.0 - from.0, to.1 - from.1);
    let ch = if dy.abs() < dx.abs() * 0.35 {
        '\u{2500}'
    } else if dy.abs() > dx.abs() * 2.5 {
        '\u{2502}'
    } else if dy > 0.0 {
        '\\'
    } else {
        '/'
    };

    let steps = dx.abs().max(dy.abs()).ceil().max(1.0);
    let mut i = 0.0;
    while i <= steps {
        let t = i / steps;
        put_cell(
            buf,
            area,
            dx.mul_add(t, from.0),
            dy.mul_add(t, from.1),
            ch,
            color,
        );
        i += 1.0;
    }
}

/// Write horizontal text starting at a fractional cell position.
fn put_text(buf: &mut Buffer, area: Rect, x: f64, y: f64, text: &str, color: Color) {
    for (i, ch) in text.chars().enumerate() {
        #[allow(clippy::cast_precision_loss)]
        put_cell(buf, area, x + i as f64, y, ch, color);
    }
}

/// Write one character at a fractional cell position, keeping the existing
/// background so overlays sit on top of the sky/ground fill.
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
fn put_cell(buf: &mut Buffer, area: Rect, x: f64, y: f64, ch: char, color: Color) {
    // Round rather than floor: line samples are spaced exactly one cell apart on the
    // dominant axis, and flooring turns the inevitable float error into dropped cells
    // (a dashed horizon with the fill showing through the gaps).
    let (col, row) = (x.round(), y.round());
    if col < 0.0 || row < 0.0 || col >= f64::from(area.width) || row >= f64::from(area.height) {
        return;
    }
    if let Some(cell) = buf.cell_mut((area.x + col as u16, area.y + row as u16)) {
        cell.set_char(ch).set_fg(color);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Render to an ASCII map: '.' sky, '#' ground, overlay chars as-is.
    fn render_ascii(pitch: f64, roll: f64, w: u16, h: u16) -> String {
        let area = Rect::new(0, 0, w, h);
        let mut buf = Buffer::empty(area);
        Horizon {
            pitch_deg: pitch,
            roll_deg: roll,
            pitch_cp: 0,
            roll_cp: 0,
        }
        .render(area, &mut buf);
        let mut out = String::new();
        for row in 0..h {
            for col in 0..w {
                let cell = buf.cell((col, row)).unwrap();
                let ch = cell.symbol().chars().next().unwrap_or(' ');
                out.push(if ch == ' ' {
                    if cell.bg == GROUND { '#' } else { '.' }
                } else if ch == '\u{2580}' {
                    '='
                } else {
                    ch
                });
            }
            out.push('\n');
        }
        out
    }

    /// Eyeball the instrument: `cargo test ... --bin elle dump -- --nocapture --ignored`
    #[test]
    #[ignore = "visual inspection helper, not an assertion"]
    fn dump() {
        for (p, r) in [(0.0, 0.0), (0.0, 30.0), (10.0, 0.0), (-15.0, -20.0)] {
            println!("--- pitch {p} roll {r} ---\n{}", render_ascii(p, r, 66, 20));
        }
    }

    /// Rows as char vectors — the overlay glyphs are multi-byte, so byte slicing
    /// the rendered rows is not safe.
    fn rows(pitch: f64, roll: f64) -> Vec<Vec<char>> {
        render_ascii(pitch, roll, 66, 20)
            .lines()
            .map(|l| l.chars().collect())
            .collect()
    }

    #[test]
    fn level_flight_puts_sky_above_and_ground_below() {
        let rows = rows(0.0, 0.0);
        assert_eq!(rows[2][0], '.', "top of instrument should be sky");
        assert_eq!(rows[17][0], '#', "bottom of instrument should be ground");
    }

    /// Nose up moves the horizon *down* the instrument, as the world does.
    #[test]
    fn pitch_up_lowers_the_horizon() {
        let ground_row = |pitch: f64| {
            rows(pitch, 0.0)
                .iter()
                .position(|r| r[0] == '#')
                .expect("ground must be visible")
        };
        assert!(ground_row(10.0) > ground_row(0.0));
        assert!(ground_row(-10.0) < ground_row(0.0));
    }

    /// Documents `ROLL_SIGN`: positive roll is right-wing-down, so the ground
    /// swings to the right of the instrument. If the real airframe shows the
    /// mirror image, that constant is what flips.
    #[test]
    fn positive_roll_puts_ground_to_the_right() {
        let rows = rows(0.0, 45.0);
        let mid = &rows[10];
        let ground_right = mid[40..].iter().filter(|c| **c == '#').count();
        let ground_left = mid[..26].iter().filter(|c| **c == '#').count();
        assert!(
            ground_right > ground_left,
            "expected ground on the right at +45 roll, got: {}",
            mid.iter().collect::<String>()
        );
    }

    /// The horizon is drawn by sampling a line one cell at a time; flooring the
    /// sample positions used to drop cells and leave the fill showing through.
    #[test]
    fn horizon_line_has_no_gaps() {
        let rows = rows(0.0, 0.0);
        let line = rows
            .iter()
            .find(|r| r[0] == '\u{2500}')
            .expect("horizon line must be drawn");
        // Everything outside the aircraft symbol must be line, not fill.
        let gaps = line[..25].iter().filter(|c| **c != '\u{2500}').count();
        assert_eq!(
            gaps,
            0,
            "horizon line has gaps: {}",
            line.iter().collect::<String>()
        );
    }
}
