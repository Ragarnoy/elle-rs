#!/usr/bin/env python3
"""Read Elle ULog flight logs (LOG_NNNN.ulg from the SD card).

Subcommands (all take one or more .ulg paths unless noted):

  list      one line per file: duration, which build features it has, armed time
  summary   armed intervals, modes, events with their labels
  timing    control-loop busy time by mode/engine state, late ticks, Core 1 load
  esc       ESC link health and engine telemetry sanity
  sensors   mag / baro / GNSS / attitude rates and ranges
  stages    flight-loop time per stage (loop_stages) by armed/mode, DShot executor share
  window    events and key numbers between two times: window FILE T0 T1

Needs pyulog and numpy (see SKILL.md for the venv). Older logs lack some
messages (controller, esc_health, core1_load); every section says so instead of
failing.
"""

import argparse
import re
import sys
import warnings
from collections import Counter
from pathlib import Path

import numpy as np

warnings.filterwarnings("ignore")  # pyulog warns on every dropout record
from pyulog import ULog  # noqa: E402

REPO = Path(__file__).resolve().parents[3]
UI_RS = REPO / "tools/elle-rpc-host/src/tui/ui.rs"
MODES = {0: "Manual", 1: "Stabilized", 2: "AltitudeHold"}
LATE_TICK_MS = 24  # two 12 ms control periods: what support::resync_after_stall warns on


def event_labels():
    """Event code -> label, read from the host TUI so it never drifts."""
    try:
        text = UI_RS.read_text()
    except OSError:
        return {}
    return {int(c): t for c, t in re.findall(r'^\s*(\d+) => "([^"]*)"', text, re.M)}


class Log:
    def __init__(self, path):
        self.path = Path(path)
        self.ulog = ULog(str(path))
        self.m = {d.name: d.data for d in self.ulog.data_list}
        self.t0 = min(d.data["timestamp"][0] for d in self.ulog.data_list)

    def t(self, ts):
        """µs-since-boot timestamps -> seconds since the first record in the file."""
        return (np.asarray(ts) - self.t0) / 1e6

    def has(self, name):
        return name in self.m

    @property
    def name(self):
        return self.path.name

    @property
    def duration(self):
        return max(self.t(d["timestamp"][-1]) for d in self.m.values())

    def status_context(self):
        """Per system_status sample (8.3 Hz): armed, mode, engines running."""
        s = self.m["system_status"]
        ts = s["timestamp"]
        armed = s["armed"] == 1
        c = self.m["commands"]
        mode = np.rint(np.interp(ts, c["timestamp"], c["attitude_mode"])).astype(int)
        e = self.m["engine_data"]
        running = np.interp(ts, e["timestamp"], e["left_target_erpm"]) > 0
        return ts, armed, mode, running

    def tick_timestamps(self):
        """One timestamp per control tick (controller, or commands in older logs)."""
        return self.m["controller" if self.has("controller") else "commands"]["timestamp"]


def pct(a, q):
    return int(np.percentile(a, q)) if len(a) else 0


def changes_per_s(log, name, fields):
    d = log.m[name]
    tt = log.t(d["timestamp"])
    if len(tt) < 2:
        return 0.0
    chg = np.zeros(len(tt) - 1, bool)
    for f in fields:
        chg |= np.diff(d[f]) != 0
    return chg.sum() / (tt[-1] - tt[0])


def armed_intervals(log):
    s = log.m["system_status"]
    ts = log.t(s["timestamp"])
    arm = s["armed"].astype(int)
    out, start = [], (ts[0] if arm[0] else None)
    for i in np.nonzero(np.diff(arm))[0]:
        if arm[i + 1]:
            start = ts[i + 1]
        else:
            out.append((start, ts[i + 1]))
            start = None
    if start is not None:
        out.append((start, ts[-1]))
    return out


# ---------------------------------------------------------------------------


def cmd_list(logs, _args):
    for log in logs:
        feats = [n for n in ("controller", "esc_health", "core1_load", "gyro_raw") if log.has(n)]
        mag = changes_per_s(log, "magnetometer_data", ["mag_x", "mag_y", "mag_z"]) if log.has("magnetometer_data") else 0
        armed = sum(b - a for a, b in armed_intervals(log)) if log.has("system_status") else 0
        ev = len(log.m["log_event"]["code"]) if log.has("log_event") else 0
        print(f"{log.name}  {log.duration:6.0f}s  armed {armed:5.0f}s  mag {mag:4.1f}/s  events {ev:3d}  [{', '.join(feats)}]")


def cmd_summary(logs, _args):
    labels = event_labels()
    for log in logs:
        print(f"== {log.name}  {log.duration:.0f}s  (t=0 is the first record, not boot: t0 = {log.t0 / 1e6:.2f}s after boot)")
        print("  armed:", [(round(float(a), 1), round(float(b), 1)) for a, b in armed_intervals(log)] or "never")
        c = log.m["commands"]
        modes = Counter(int(x) for x in c["attitude_mode"])
        print("  mode share:", {MODES.get(k, k): f"{v / len(c['attitude_mode']):.0%}" for k, v in sorted(modes.items())})
        if log.has("log_event"):
            ev = log.m["log_event"]
            codes = [int(x) for x in ev["code"]]
            print("  event counts:", {f"{k} {labels.get(k, '?')}": v for k, v in sorted(Counter(codes).items())})
            print("  timeline (periodic 2/7/23 omitted):")
            for t, k in zip(log.t(ev["timestamp"]), codes):
                if k not in (2, 7, 23):
                    print(f"    {t:8.2f}s  {k:3d}  {labels.get(k, '?')}")
        if log.ulog.dropouts:
            print(f"  ULog dropouts: {len(log.ulog.dropouts)}, {sum(d.duration for d in log.ulog.dropouts)} ms total")


def cmd_timing(logs, _args):
    for log in logs:
        print(f"== {log.name}")
        ts = log.tick_timestamps()
        dt = np.diff(ts) / 1000
        late = np.nonzero(dt > LATE_TICK_MS)[0]
        tt = log.t(ts)
        print(f"  ticks {len(ts)}, median period {np.median(dt):.2f} ms, late (> {LATE_TICK_MS} ms): {len(late)}"
              + (f", max {dt.max():.0f} ms" if len(late) else ""))
        if len(late):
            buckets = Counter(int(tt[i] // 10) * 10 for i in late)
            print("    late ticks per 10 s bucket:", dict(sorted(buckets.items())))
        _, armed, mode, running = log.status_context()
        lt = log.m["system_status"]["loop_time_us"]
        print("  loop busy time (loop_time_us: wall time from tick start to the ULog write), p50 / p90 / p99:")
        groups = [("disarmed", ~armed)]
        for m in sorted(set(mode[armed])):
            groups += [(f"armed {MODES.get(m, m)}, stopped", armed & (mode == m) & ~running),
                       (f"armed {MODES.get(m, m)}, running", armed & (mode == m) & running)]
        for lab, mk in groups:
            if mk.sum() > 5:
                print(f"    {lab:30s} n={mk.sum():5d}  {pct(lt[mk], 50):4d} / {pct(lt[mk], 90):4d} / {pct(lt[mk], 99):4d} µs")
        if log.has("core1_load"):
            c = log.m["core1_load"]
            w = slice(1, None)  # the first window covers boot
            print("  Core 1 IMU wake-ups (1 ms deadline), per ~1 s window:")
            print(f"    busy avg median {pct(c['busy_avg_us'][w], 50)} µs; busy max p50 / p99 / max "
                  f"{pct(c['busy_max_us'][w], 50)} / {pct(c['busy_max_us'][w], 99)} / {int(c['busy_max_us'][w].max())} µs")
            print(f"    mag read max p50 {pct(c['mag_max_us'][w], 50)} µs, baro read max p50 {pct(c['baro_max_us'][w], 50)} µs"
                  f" (async reads; include time the IMU task held the core)")
            drains = np.nonzero(c["max_drain"] > 1)[0]
            print(f"    windows with FIFO backlog (max_drain > 1): {[(round(float(log.t(c['timestamp'][i])), 1), int(c['max_drain'][i])) for i in drains]}")
        else:
            print("  (no core1_load: older build)")


def cmd_esc(logs, _args):
    for log in logs:
        print(f"== {log.name}")
        if log.has("esc_health"):
            h = log.m["esc_health"]
            th = log.t(h["timestamp"])
            for side in ("left", "right"):
                r = h[f"{side}_replies"].astype(np.int64)
                rate = np.diff(r) / np.diff(th) if len(r) > 1 else np.array([0])
                edt = f", EDT frames {int(h[f'{side}_edt_frames'][-1])}" if f"{side}_edt_frames" in h else ""
                print(f"  {side:5s}: replies {int(r[-1])} (per s median {pct(rate, 50)}, min {int(rate.min())}), "
                      f"timeouts {int(h[f'{side}_timeouts'][-1])}, corrupt {int(h[f'{side}_bad_frames'][-1])}, "
                      f"re-configured {int(h[f'{side}_reconfigs'][-1])}{edt}")
        else:
            print("  (no esc_health: older build)")
        e = log.m["engine_data"]
        _, armed, _, _ = log.status_context()
        arm_e = np.interp(e["timestamp"], log.m["system_status"]["timestamp"], armed.astype(float)) > 0.5
        print(f"  target > 0 while disarmed: {int(((e['left_target_erpm'] > 0) & ~arm_e).sum())} samples (should be 0)")
        for side in ("left", "right"):
            v = e[f"{side}_voltage_mv"]
            print(f"  {side:5s}: EDT voltage {'none (EDT never arrived)' if v.max() == 0 else f'{v[v > 0].min()}-{v.max()} mV'}, "
                  f"max eRPM {int(e[f'{side}_erpm'].max())}, max DShot {int(e[f'{side}_throttle'].max())}")


def cmd_sensors(logs, _args):
    for log in logs:
        print(f"== {log.name}  ({log.duration:.0f}s)")
        if log.has("magnetometer_data"):
            n = len(log.m["magnetometer_data"]["mag_x"]) / log.duration
            print(f"  mag:  logged {n:.1f}/s, value changes {changes_per_s(log, 'magnetometer_data', ['mag_x', 'mag_y', 'mag_z']):.1f}/s"
                  " (changes ≈ 1/s means the sensor itself runs at ~1 Hz)")
        if log.has("barometer_data"):
            b = log.m["barometer_data"]
            print(f"  baro: logged {len(b['pressure_hpa']) / log.duration:.1f}/s, {b['pressure_hpa'].min():.2f}-{b['pressure_hpa'].max():.2f} hPa")
        if log.has("gnss_data"):
            g = log.m["gnss_data"]
            print(f"  gnss: max sats {int(g['num_satellites'].max())}, max fix {int(g['fix_quality'].max())}, "
                  f"best h_acc {g['h_acc_m'][g['h_acc_m'] > 0].min() if (g['h_acc_m'] > 0).any() else '-'} m")
        a = log.m["attitude_data"]
        print(f"  attitude: pitch {np.degrees(a['pitch']).min():.1f}..{np.degrees(a['pitch']).max():.1f}°, "
              f"roll {np.degrees(a['roll']).min():.1f}..{np.degrees(a['roll']).max():.1f}°, "
              f"yaw {np.degrees(a['yaw']).min():.1f}..{np.degrees(a['yaw']).max():.1f}°")


STAGE_NAMES = ["intake", "update", "outputs", "switches", "autotune", "log", "tail"]


def cmd_stages(logs, _args):
    for log in logs:
        print(f"== {log.name}")
        if not log.has("loop_stages"):
            print("  (no loop_stages: older build)")
            continue
        st = log.m["loop_stages"]
        ts = st["timestamp"]
        s = log.m["system_status"]
        armed = np.interp(ts, s["timestamp"], s["armed"].astype(float)) > 0.5
        c = log.m["commands"]
        mode = np.rint(np.interp(ts, c["timestamp"], c["attitude_mode"])).astype(int)
        e = log.m["engine_data"]
        running = np.interp(ts, e["timestamp"], e["left_target_erpm"]) > 0
        avg = np.stack([st[f"avg_us[{i}]"] for i in range(7)], axis=1).astype(float)
        mx = np.stack([st[f"max_us[{i}]"] for i in range(7)], axis=1).astype(float)
        # DShot executor share of the window's wall time.
        win_us = np.diff(ts, prepend=ts[0]).astype(float)
        share = np.where(win_us > 0, st["dshot_busy_us"] / np.maximum(win_us, 1), 0)
        groups = [("disarmed", ~armed)]
        for m in sorted(set(mode[armed])):
            groups += [(f"armed {MODES.get(m, m)}, stopped", armed & (mode == m) & ~running),
                       (f"armed {MODES.get(m, m)}, running", armed & (mode == m) & running)]
        print("  mean µs per tick (median over windows); max = p90 of per-window max")
        print("  " + " " * 30 + "".join(f"{n:>9s}" for n in STAGE_NAMES) + "    total   dshot%")
        for lab, mk in groups:
            mk = mk.copy()
            mk[0] = False  # the first window covers boot
            if mk.sum() < 3:
                continue
            med = np.median(avg[mk], axis=0)
            p90 = np.percentile(mx[mk], 90, axis=0)
            print(f"  {lab:28s}  " + "".join(f"{v:9.0f}" for v in med) + f"  {med.sum():7.0f}  {100 * np.median(share[mk]):6.1f}")
            print(f"  {'   max':28s}  " + "".join(f"{v:9.0f}" for v in p90))


def cmd_window(logs, args):
    labels = event_labels()
    log = logs[0]
    a, b = args.t0, args.t1
    print(f"== {log.name}  {a}-{b}s")
    if log.has("log_event"):
        ev = log.m["log_event"]
        for t, k in zip(log.t(ev["timestamp"]), ev["code"]):
            if a <= t <= b:
                print(f"  {t:8.2f}s  {int(k):3d}  {labels.get(int(k), '?')}")
    ts = log.tick_timestamps()
    tt = log.t(ts)
    m = (tt[1:] >= a) & (tt[1:] <= b)
    dt = np.diff(ts)[m] / 1000
    print(f"  ticks {m.sum()}, late {int((dt > LATE_TICK_MS).sum())}" + (f", max {dt.max():.0f} ms" if len(dt) else ""))
    s = log.m["system_status"]
    sm = (log.t(s["timestamp"]) >= a) & (log.t(s["timestamp"]) <= b)
    if sm.any():
        lt = s["loop_time_us"][sm]
        print(f"  loop_time_us p50 / p90 / max {pct(lt, 50)} / {pct(lt, 90)} / {int(lt.max())}, rc_age max {int(s['rc_age_ms'][sm].max())} ms, "
              f"armed {bool(s['armed'][sm].max())}")
    c = log.m["commands"]
    cm = (log.t(c["timestamp"]) >= a) & (log.t(c["timestamp"]) <= b)
    if cm.any():
        print(f"  modes {[MODES.get(int(x), int(x)) for x in np.unique(c['attitude_mode'][cm])]}, "
              f"throttle {c['throttle'][cm].min():.2f}-{c['throttle'][cm].max():.2f}")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)
    for name in ("list", "summary", "timing", "esc", "sensors", "stages"):
        sp = sub.add_parser(name)
        sp.add_argument("files", nargs="+")
    w = sub.add_parser("window")
    w.add_argument("files", nargs=1)
    w.add_argument("t0", type=float)
    w.add_argument("t1", type=float)
    args = p.parse_args()

    logs = []
    for f in args.files:
        try:
            logs.append(Log(f))
        except Exception as e:  # truncated or empty files are common after a power cut
            print(f"{Path(f).name}: unreadable ({e})", file=sys.stderr)
    {"list": cmd_list, "summary": cmd_summary, "timing": cmd_timing, "esc": cmd_esc,
     "sensors": cmd_sensors, "stages": cmd_stages, "window": cmd_window}[args.cmd](logs, args)


if __name__ == "__main__":
    main()
