//! The scripted stream: a deterministic session written to reach every branch of
//! the control law and the controller around it. `scripted.stream` is this,
//! written out (`write_scripted_stream`); a test keeps the two in step.
//!
//! The session follows the firmware's loop: each tick is an update, the RC
//! packet note, the failsafe check and (once enabled) the supervisor check.

use crate::stream::{AttIn, Cmd, Op};
// libm, not std: std's trig goes to the system libm, whose last bit can differ
// between machines, and the committed stream must regenerate identically on CI.
use libm::{cosf, sinf};

const TICK_US: u64 = 5_000;
/// CRSF stick and switch positions.
const LOW: u16 = 172;
const MID: u16 = 992;
const HIGH: u16 = 1811;

#[derive(Clone, Copy, PartialEq, Eq)]
enum Mode {
    Manual,
    Stabilized,
    AltitudeHold,
}

struct Session {
    ops: Vec<Op>,
    t: u64,
    last_rx: u64,
    /// RC packets arriving.
    link: bool,
    /// Supervisor checks run (after `enable_supervisor`).
    supervising: bool,
    /// Core 1 heartbeat arriving.
    heartbeat: bool,
    /// Simulated attitude (rad), driven by the session.
    pitch: f32,
    roll: f32,
    yaw: f32,
    rng: u32,
}

impl Session {
    fn new() -> Self {
        Self {
            ops: Vec::new(),
            t: 0,
            last_rx: 0,
            link: true,
            supervising: false,
            heartbeat: true,
            pitch: 0.0,
            roll: 0.0,
            yaw: 0.0,
            rng: 0x1234_5678,
        }
    }

    fn noise(&mut self) -> f32 {
        self.rng = self.rng.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        (self.rng >> 8) as f32 / (1u32 << 24) as f32 - 0.5
    }

    /// The simulated aircraft's attitude sample, read 300 µs before the tick.
    fn attitude(&mut self) -> AttIn {
        let n = [self.noise(), self.noise(), self.noise()];
        AttIn {
            t_us: self.t - 300,
            pitch: self.pitch + 0.01 * n[0],
            roll: self.roll + 0.01 * n[1],
            yaw: self.yaw,
            pitch_rate: 0.5 * n[0],
            roll_rate: 0.5 * n[1],
            yaw_rate: 0.3 * n[2],
        }
    }

    fn push(&mut self, op: Op) {
        self.ops.push(op);
    }

    /// One control tick: update, RC note, failsafe, supervisor.
    fn tick(&mut self, cmd: Cmd, with_attitude: bool) {
        self.t += TICK_US;
        if self.link {
            self.last_rx = self.t - 1_000;
        }
        let att = with_attitude.then(|| self.attitude());
        self.push(Op::Tick {
            t_us: self.t,
            rx_us: self.last_rx,
            cmd,
            att,
        });
        self.after_tick();
    }

    /// A tick with the kill switch held: no update.
    fn kill_tick(&mut self) {
        self.t += TICK_US;
        if self.link {
            self.last_rx = self.t - 1_000;
        }
        self.push(Op::Wait(self.t));
        self.after_tick();
    }

    fn after_tick(&mut self) {
        if self.link {
            self.push(Op::Rc(self.last_rx));
        }
        self.push(Op::Failsafe);
        if self.supervising {
            self.push(Op::Supervise(self.heartbeat));
        }
    }

    fn raw(roll: u16, pitch: u16, throttle: u16, yaw: u16, mode: Mode) -> Cmd {
        let mut ch = [LOW; 16];
        ch[0] = roll;
        ch[1] = pitch;
        ch[2] = throttle;
        ch[3] = yaw;
        ch[5] = match mode {
            Mode::Manual => LOW,
            Mode::Stabilized => MID,
            Mode::AltitudeHold => HIGH,
        };
        Cmd::Raw(ch)
    }

    fn norm(throttle: f32, pitch: f32, roll: f32, yaw: f32, mode: Mode) -> Cmd {
        Cmd::Norm {
            throttle,
            pitch,
            roll,
            yaw,
            mode: match mode {
                Mode::Manual => 'M',
                Mode::Stabilized => 'S',
                Mode::AltitudeHold => 'A',
            },
        }
    }

    /// `n` raw ticks with sticks sweeping around `throttle`.
    fn fly_raw(&mut self, n: usize, throttle: u16, mode: Mode, amp: f32) {
        for i in 0..n {
            let ph = i as f32 * 0.15;
            let stick = |k: f32| (992.0 + amp * 800.0 * sinf(ph * k)) as u16;
            self.pitch = 0.2 * sinf(ph * 0.7);
            self.roll = 0.3 * cosf(ph * 0.5);
            self.yaw = (self.yaw + 0.01).rem_euclid(core::f32::consts::TAU) - 0.0;
            let cmd = Self::raw(stick(1.0), stick(1.3), throttle, stick(0.9), mode);
            self.tick(cmd, true);
        }
    }

    fn fly_norm(&mut self, n: usize, throttle: f32, mode: Mode) {
        for i in 0..n {
            let ph = i as f32 * 0.2;
            self.pitch = 0.15 * sinf(ph * 0.6);
            self.roll = 0.25 * sinf(ph * 0.4);
            let cmd = Self::norm(
                throttle,
                0.5 * sinf(ph),
                0.6 * cosf(ph * 1.1),
                0.4 * sinf(ph),
                mode,
            );
            self.tick(cmd, true);
        }
    }
}

pub fn scripted() -> Vec<Op> {
    use Mode::*;
    let mut s = Session::new();

    // Boot: Manual, throttle low, no attitude yet, then attitude arrives.
    for _ in 0..5 {
        s.tick(Session::raw(MID, MID, LOW, MID, Manual), false);
    }
    s.fly_raw(10, LOW, Manual, 0.2);
    s.push(Op::EnableSupervisor);
    s.supervising = true;

    // Throttle low without the high first: no arm. Then the gesture.
    s.fly_raw(5, LOW, Manual, 0.0);
    s.fly_raw(10, 1500, Manual, 0.1);
    s.fly_raw(5, LOW, Manual, 0.0);

    // Manual flight, armed: fast path, differential thrust.
    s.fly_raw(40, 1300, Manual, 0.6);

    // Stabilized (leaves Manual: PID reset), sticks moving.
    s.fly_raw(100, 1300, Stabilized, 0.5);

    // Hold full nose-up stick against a nose-down attitude: the mixer saturates,
    // the integral must freeze that way and unwind after.
    for _ in 0..40 {
        s.pitch = -0.6;
        s.roll = 0.5;
        s.tick(Session::raw(HIGH, HIGH, 1300, MID, Stabilized), true);
    }
    s.fly_raw(30, 1300, Stabilized, 0.1);

    // Low throttle in Stabilized: integrators reset, damper off.
    s.fly_raw(15, LOW, Stabilized, 0.3);

    // Attitude stops: held (event on the 2nd tick), then expired (event once).
    s.fly_raw(5, 1300, Stabilized, 0.3);
    for _ in 0..8 {
        s.tick(Session::raw(1200, 900, 1300, MID, Stabilized), false);
    }
    s.fly_raw(10, 1300, Stabilized, 0.3);
    // A single missing sample: held one tick, no event.
    s.tick(Session::raw(1200, 900, 1300, MID, Stabilized), false);
    s.fly_raw(5, 1300, Stabilized, 0.3);

    // Heading hold across the ±π wrap, with gaps in the attitude.
    s.yaw = 3.1;
    s.push(Op::Heading(-3.1));
    for i in 0..60 {
        s.yaw = 3.1 + 0.005 * i as f32;
        if s.yaw > core::f32::consts::PI {
            s.yaw -= core::f32::consts::TAU;
        }
        let att = i % 13 != 7;
        s.tick(Session::raw(MID, MID, 1300, MID, Stabilized), att);
    }
    // Heading hold with the attitude gone long enough to expire: stick roll.
    for _ in 0..6 {
        s.tick(Session::raw(1400, MID, 1300, MID, Stabilized), false);
    }
    s.fly_raw(5, 1300, Stabilized, 0.2);

    // AltitudeHold (no reset between attitude modes), heading still engaged.
    s.fly_raw(40, 1300, AltitudeHold, 0.5);

    // Autotune relay: override bypasses the filter, in both attitude modes.
    s.push(Op::Override(10.0, 0.0));
    s.fly_raw(20, 1300, AltitudeHold, 0.2);
    s.push(Op::Override(-10.0, 0.0));
    s.fly_raw(20, 1300, Stabilized, 0.2);
    s.push(Op::Override(0.0, 12.5));
    s.fly_raw(10, 1300, Stabilized, 0.2);
    s.push(Op::ClearOverride);
    s.fly_raw(20, 1300, Stabilized, 0.2);

    // Gains change in flight (integrals reset, version bumps).
    s.push(Op::Gains([0.4, 0.15, 0.03, 0.35, 0.12, 0.025, 0.3, 5.0]));
    s.fly_raw(30, 1300, Stabilized, 0.4);

    // Back to Manual with heading hold on: PID reset, heading disengaged.
    s.push(Op::Heading(0.5));
    s.fly_raw(5, 1300, Stabilized, 0.2);
    s.fly_raw(20, 1300, Manual, 0.4);
    s.fly_raw(20, 1300, Stabilized, 0.4);

    // RC link lost in Stabilized: warning, lost (disarm, PID reset), the
    // failsafe hold on every tick, a mode change while held, then restored
    // without re-arming.
    s.link = false;
    for _ in 0..70 {
        s.tick(Session::raw(1500, 1500, 1300, MID, Stabilized), true);
    }
    for _ in 0..10 {
        s.tick(Session::raw(1500, 1500, LOW, MID, Manual), true);
    }
    s.link = true;
    s.fly_raw(10, LOW, Manual, 0.1);
    s.fly_raw(10, 1500, Manual, 0.1);
    s.fly_raw(5, LOW, Manual, 0.0);
    s.fly_raw(30, 1300, Stabilized, 0.3);

    // A short dropout: warning only, then restored.
    s.link = false;
    for _ in 0..45 {
        s.tick(Session::raw(1500, 1500, 1300, MID, Stabilized), true);
    }
    s.link = true;
    s.fly_raw(10, 1300, Stabilized, 0.3);

    // Core 1 stops: unhealthy after the timeout, attitude control off and kept
    // off by every tick; heartbeat back, control back.
    s.heartbeat = false;
    s.fly_raw(430, 1300, Stabilized, 0.3);
    s.heartbeat = true;
    s.fly_raw(30, 1300, Stabilized, 0.3);

    // Kill switch: no updates, then disarm and centre (as elle-app does).
    for _ in 0..20 {
        s.kill_tick();
    }
    s.push(Op::Disarm);
    s.push(Op::Safe);
    s.fly_raw(20, LOW, Stabilized, 0.2);

    // RPC mode: explicit arming only, normalized commands, Manual on the
    // normalized path, arm and disarm by command.
    s.push(Op::Explicit(true));
    s.fly_norm(10, 0.0, Manual);
    s.fly_norm(10, 0.6, Manual);
    s.push(Op::Arm);
    s.fly_norm(40, 0.6, Manual);
    s.fly_norm(60, 0.6, Stabilized);
    s.fly_norm(40, 0.6, AltitudeHold);
    s.fly_norm(10, 0.02, Stabilized);
    s.push(Op::Heading(1.0));
    s.fly_norm(30, 0.6, Stabilized);
    s.push(Op::Unheading);
    s.fly_norm(10, 0.6, Stabilized);
    // Throttle just under the integrator-reset threshold (0.05), and just over.
    s.fly_norm(10, 0.045, Stabilized);
    s.fly_norm(10, 0.055, Stabilized);
    s.fly_norm(20, 0.6, Stabilized);
    // Normalized Manual with heading hold on: disengages it (the RC fast path
    // does not).
    s.push(Op::Heading(-0.7));
    s.fly_norm(10, 0.6, Stabilized);
    s.fly_norm(5, 0.6, Manual);
    s.fly_norm(20, 0.6, Stabilized);
    // Host link lost and back, re-armed by command at cruise throttle: the
    // loss must have cleared the integrators (no low-throttle tick resets them).
    s.link = false;
    for _ in 0..70 {
        s.tick(Session::norm(0.6, 0.3, -0.2, 0.0, Stabilized), true);
    }
    s.link = true;
    s.fly_norm(5, 0.6, Stabilized);
    s.push(Op::Arm);
    s.fly_norm(30, 0.6, Stabilized);
    s.push(Op::Disarm);
    s.fly_norm(20, 0.6, Stabilized);
    s.push(Op::Explicit(false));
    s.fly_norm(10, 0.0, Stabilized);
    s.fly_norm(10, 0.5, Stabilized);
    s.fly_norm(10, 0.0, Stabilized);
    s.fly_norm(20, 0.5, Stabilized);

    s.ops
}
