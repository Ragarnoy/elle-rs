#!/usr/bin/env python3
"""Proposal 0007: check the equivalence oracle can fail.

Applies one mutation at a time to the new flight law or the controller around
it, runs the oracle, and reports whether a stream caught it. Every source file
is restored afterwards. Run from the workspace root:

    python3 crates/elle-control/tests/flight_law_equivalence/mutate.py [--dart]

A mutation the oracle misses is a gap in the streams, unless it is equivalent
(it can't change anything observable); the known one is marked.
"""

import subprocess
import sys

LAW = "crates/elle-control/src/flight_law.rs"
SYS = "crates/elle-system/src/system.rs"

# (file, name, original text, replacement, equivalent)
MUTATIONS = [
    (LAW, "no PID reset leaving Manual",
     "            if self.current_control_mode == ControlMode::Manual {\n"
     "                self.attitude_controller.reset();",
     "            if self.current_control_mode == ControlMode::Manual {", False),
    (LAW, "ignore core1_healthy", "            && inputs.core1_healthy;", "            && true;", False),
    (LAW, "signal_loss keeps the PID",
     "        self.arming.signal_loss();\n        self.attitude_controller.reset();",
     "        self.arming.signal_loss();", False),
    (LAW, "normalized Manual keeps heading hold",
     "                if self.heading_hold_active {\n"
     "                    self.disengage_heading_hold();\n"
     "                }", "", False),
    (LAW, "disarm keeps heading hold",
     "        self.arming.disarm();\n        self.disengage_heading_hold();",
     "        self.arming.disarm();", False),
    (LAW, "held_ticks not cleared",
     "            self.held_ticks = 0;\n            self.attitude_dropped = false;",
     "            self.attitude_dropped = false;", False),
    (LAW, "override goes through the filter",
     "            if self.setpoint_override.is_some() {\n                // Autotune",
     "            if false {\n                // Autotune", False),
    (LAW, "damper never reset",
     "                        yaw_damp = if self.attitude_controller.enabled && !low_throttle {\n"
     "                            self.yaw_damper.update(att.yaw_rate)\n"
     "                        } else {\n"
     "                            self.yaw_damper.reset();\n"
     "                            0.0\n"
     "                        };",
     "                        yaw_damp = self.yaw_damper.update(att.yaw_rate);", False),
    (LAW, "disable_attitude does nothing",
     "    pub fn disable_attitude(&mut self) {\n        self.attitude_controller.enabled = false;",
     "    pub fn disable_attitude(&mut self) {", False),
    (LAW, "fast path does not store saturation",
     "        let elevon_outputs = mix_elevons_direct_lut(channels);\n"
     "        self.last_saturation = elevon_outputs.saturation;",
     "        let elevon_outputs = mix_elevons_direct_lut(channels);", True),
    (LAW, "failsafe hold leaves the PID on",
     "        self.attitude_controller.enabled = false;\n        self.yaw_damper.reset();\n"
     "        self.last_output = ControllerOutputSnapshot {\n            pitch_correction: 0.0,",
     "        self.yaw_damper.reset();\n"
     "        self.last_output = ControllerOutputSnapshot {\n            pitch_correction: 0.0,", False),
    (LAW, "low-throttle threshold 0.05 -> 0.04", "norm.throttle < 0.05;", "norm.throttle < 0.04;", False),
    (LAW, "stale event not latched", "            self.attitude_dropped = true;\n", "", False),
    (LAW, "cached attitude ignored",
     "match attitude.or(fresh_cached) {\n                    Some(att) => {\n                        // Compute",
     "match attitude {\n                    Some(att) => {\n                        // Compute", False),
    (LAW, "attitude never cached",
     "        if let Some(att) = attitude {\n            self.last_attitude = Some(*att);\n        }", "", False),
    (LAW, "explicit arming ignored", "        if !self.explicit_arming_only {", "        if true {", False),
    (LAW, "engines run in the failsafe hold",
     "            engine_left_dshot: 0,\n            engine_right_dshot: 0,\n            att_age_us: u32::MAX,",
     "            att_age_us: u32::MAX,", False),
    (LAW, "Safe write reports the mixed pulses",
     "            ElevonCommand::Safe => (\n                ELEVON_LEFT_CENTER_US,",
     "            ElevonCommand::Safe => (\n                ELEVON_LEFT_CENTER_US + 1,", False),
    (LAW, "write drops dt_us", "            dt_us,\n            ..self.snapshot", "            ..self.snapshot", False),
    (SYS, "restore does not clear failsafe",
     "                self.law.signal_restored();\n", "", False),
    (SYS, "loss does not centre the elevons",
     "                self.law.signal_loss();\n                self.apply_failsafe();",
     "                self.law.signal_loss();", False),
    (SYS, "Core 1 verdict not passed", "                core1_healthy: self.core1_health.is_healthy,",
     "                core1_healthy: true,", False),
    (SYS, "events emitted after the write",
     "        self.last_output = out.write(&mut self.pwm, dt_us);\n",
     "", False),
    (SYS, "measured dt not passed", "        self.last_output = out.write(&mut self.pwm, dt_us);",
     "        self.last_output = out.write(&mut self.pwm, 0);", False),
    (SYS, "held event dropped",
     "        if let Some(age_ms) = out.events.attitude_held_ms {",
     "        if let Some(age_ms) = None::<u64> {", False),
]


def main():
    features = ["--features", "platform-dart"] if "--dart" in sys.argv else []
    originals = {f: open(f).read() for f in {m[0] for m in MUTATIONS}}
    missed = []
    try:
        for path, name, old, new, equivalent in MUTATIONS:
            text = originals[path]
            assert text.count(old) == 1, f"{name}: original text not found once in {path}"
            mutated = text.replace(old, new)
            if name == "events emitted after the write":
                # Move the write above the events instead of deleting it.
                mutated = mutated.replace(
                    "        if let Some((from, to)) = out.events.mode_change {",
                    "        self.last_output = out.write(&mut self.pwm, dt_us);\n"
                    "        if let Some((from, to)) = out.events.mode_change {")
            open(path, "w").write(mutated)
            r = subprocess.run(
                ["cargo", "test", "-p", "elle-control", "--target", "x86_64-unknown-linux-gnu",
                 *features, "--test", "flight_law_equivalence", "legacy_and_current"],
                capture_output=True, text=True)
            out = r.stdout + r.stderr
            open(path, "w").write(text)
            if "error[" in out:
                verdict = "BUILD ERROR"
            elif r.returncode != 0:
                verdict = "caught"
            else:
                verdict = "equivalent" if equivalent else "MISSED"
                if not equivalent:
                    missed.append(name)
            print(f"{verdict:12} {name}")
    finally:
        for path, text in originals.items():
            open(path, "w").write(text)
    if missed:
        sys.exit(f"missed: {missed}")


if __name__ == "__main__":
    main()
