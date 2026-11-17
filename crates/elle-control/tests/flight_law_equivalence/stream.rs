//! The recorded input stream: one operation per line, the calls `elle-app`
//! makes on a `FlightController`, with times in µs from the stream's start.
//!
//! ```text
//! tick <t> <rx_t> raw <16 channels> [att <att_t> <pitch> <roll> <yaw> <pitch_rate> <roll_rate> <yaw_rate>]
//! tick <t> <rx_t> norm <throttle> <pitch> <roll> <yaw> <M|S|A> [att …]
//! wait <t>               clock moves on, no update (kill switch held)
//! rc <t>                 note_rc_packet(t)
//! failsafe               check_failsafe()
//! supervise <0|1>        supervisor_check(), Core 1 heartbeat sent first if 1
//! enable_supervisor | arm | disarm | safe | clear_override | unheading
//! explicit <0|1> | override <pitch_deg> <roll_deg> | heading <rad>
//! gains <kp_p> <ki_p> <kd_p> <kp_r> <ki_r> <kd_r> <i_limit> <scale>
//! ```
//!
//! `tick` and `wait` move the clock to `t`; every other operation runs at the
//! current time. Floats are written in Rust's shortest round-trip form, so a
//! stream reads back bit for bit. `#` starts a comment.

use std::fmt::Write as _;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AttIn {
    pub t_us: u64,
    pub pitch: f32,
    pub roll: f32,
    pub yaw: f32,
    pub pitch_rate: f32,
    pub roll_rate: f32,
    pub yaw_rate: f32,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Cmd {
    Raw([u16; 16]),
    Norm {
        throttle: f32,
        pitch: f32,
        roll: f32,
        yaw: f32,
        mode: char,
    },
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Op {
    Tick {
        t_us: u64,
        rx_us: u64,
        cmd: Cmd,
        att: Option<AttIn>,
    },
    Wait(u64),
    Rc(u64),
    Failsafe,
    Supervise(bool),
    EnableSupervisor,
    Arm,
    Disarm,
    Safe,
    ClearOverride,
    Unheading,
    Explicit(bool),
    Override(f32, f32),
    Heading(f32),
    Gains([f32; 8]),
}

pub fn write(ops: &[Op]) -> String {
    let mut s = String::new();
    for op in ops {
        match op {
            Op::Tick {
                t_us,
                rx_us,
                cmd,
                att,
            } => {
                write!(s, "tick {t_us} {rx_us} ").unwrap();
                match cmd {
                    Cmd::Raw(ch) => {
                        s.push_str("raw");
                        for c in ch {
                            write!(s, " {c}").unwrap();
                        }
                    }
                    Cmd::Norm {
                        throttle,
                        pitch,
                        roll,
                        yaw,
                        mode,
                    } => write!(s, "norm {throttle:?} {pitch:?} {roll:?} {yaw:?} {mode}").unwrap(),
                }
                if let Some(a) = att {
                    write!(
                        s,
                        " att {} {:?} {:?} {:?} {:?} {:?} {:?}",
                        a.t_us, a.pitch, a.roll, a.yaw, a.pitch_rate, a.roll_rate, a.yaw_rate
                    )
                    .unwrap();
                }
            }
            Op::Wait(t) => write!(s, "wait {t}").unwrap(),
            Op::Rc(t) => write!(s, "rc {t}").unwrap(),
            Op::Failsafe => s.push_str("failsafe"),
            Op::Supervise(hb) => write!(s, "supervise {}", u8::from(*hb)).unwrap(),
            Op::EnableSupervisor => s.push_str("enable_supervisor"),
            Op::Arm => s.push_str("arm"),
            Op::Disarm => s.push_str("disarm"),
            Op::Safe => s.push_str("safe"),
            Op::ClearOverride => s.push_str("clear_override"),
            Op::Unheading => s.push_str("unheading"),
            Op::Explicit(on) => write!(s, "explicit {}", u8::from(*on)).unwrap(),
            Op::Override(p, r) => write!(s, "override {p:?} {r:?}").unwrap(),
            Op::Heading(rad) => write!(s, "heading {rad:?}").unwrap(),
            Op::Gains(g) => {
                s.push_str("gains");
                for v in g {
                    write!(s, " {v:?}").unwrap();
                }
            }
        }
        s.push('\n');
    }
    s
}

pub fn parse(text: &str) -> Vec<Op> {
    let mut ops = Vec::new();
    for (n, line) in text.lines().enumerate() {
        let line = line.split('#').next().unwrap().trim();
        if line.is_empty() {
            continue;
        }
        let mut w = line.split_whitespace();
        let bad = || -> ! { panic!("stream line {}: {line}", n + 1) };
        let mut next = || w.next().unwrap_or_else(|| bad());
        let op = match next() {
            "tick" => {
                let t_us = next().parse().unwrap();
                let rx_us = next().parse().unwrap();
                let cmd = match next() {
                    "raw" => {
                        let mut ch = [0u16; 16];
                        for c in &mut ch {
                            *c = next().parse().unwrap();
                        }
                        Cmd::Raw(ch)
                    }
                    "norm" => Cmd::Norm {
                        throttle: next().parse().unwrap(),
                        pitch: next().parse().unwrap(),
                        roll: next().parse().unwrap(),
                        yaw: next().parse().unwrap(),
                        mode: next().chars().next().unwrap(),
                    },
                    _ => bad(),
                };
                let att = match w.next() {
                    None => None,
                    Some("att") => {
                        let t_us = w.next().unwrap_or_else(|| bad()).parse().unwrap();
                        let mut f =
                            || -> f32 { w.next().unwrap_or_else(|| bad()).parse().unwrap() };
                        Some(AttIn {
                            t_us,
                            pitch: f(),
                            roll: f(),
                            yaw: f(),
                            pitch_rate: f(),
                            roll_rate: f(),
                            yaw_rate: f(),
                        })
                    }
                    Some(_) => bad(),
                };
                Op::Tick {
                    t_us,
                    rx_us,
                    cmd,
                    att,
                }
            }
            "wait" => Op::Wait(next().parse().unwrap()),
            "rc" => Op::Rc(next().parse().unwrap()),
            "failsafe" => Op::Failsafe,
            "supervise" => Op::Supervise(next() == "1"),
            "enable_supervisor" => Op::EnableSupervisor,
            "arm" => Op::Arm,
            "disarm" => Op::Disarm,
            "safe" => Op::Safe,
            "clear_override" => Op::ClearOverride,
            "unheading" => Op::Unheading,
            "explicit" => Op::Explicit(next() == "1"),
            "override" => Op::Override(next().parse().unwrap(), next().parse().unwrap()),
            "heading" => Op::Heading(next().parse().unwrap()),
            "gains" => {
                let mut g = [0f32; 8];
                for v in &mut g {
                    *v = next().parse().unwrap();
                }
                Op::Gains(g)
            }
            _ => bad(),
        };
        ops.push(op);
    }
    ops
}
