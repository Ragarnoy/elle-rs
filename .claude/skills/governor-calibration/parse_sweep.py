#!/usr/bin/env python3
"""Turn `rpm_range` sweep logs into a GOVERNOR_FF_TABLE and its ceiling constants.

Usage: parse_sweep.py [--poles 14] [--step 100] LOG [LOG ...]

One log per engine (dart: one; eagle: left then right). Per engine the sweep's
`T=<dshot>: avg=<rpm>` lines are read; steps the sweep marked unstable are
absent from those lines and so skipped.

Ceiling: each engine's peak is its highest-RPM step. Above the peak RPM falls as
throttle rises, the region the governor must never wind into, so the table stops
at the *lowest* engine peak (twin: the engines must stay symmetric). A curve
still rising at the last step has no peak, and the ceiling is that last step.

Table rows are every `--step` DShot from 48 up to the ceiling, plus the ceiling
itself, eRPM averaged over engines. The last row is pinned to MAX_ERPM (slowest
engine's RPM at the ceiling, rounded down to 100), so the feedforward's top entry
and the stick's full-scale target agree. With twin engines the *average* can pass
MAX_ERPM below the ceiling (the faster engine pulls it up); those rows are dropped
so the pinned row stays the top, and the ceiling row is always kept because
GOVERNOR_DSHOT_MAX is asserted against it.
"""

import argparse
import re
import sys

STEP_RE = re.compile(r"T=(\d+): avg=(\d+)\s+min=(\d+)\s+max=(\d+)")
VOLT_RE = re.compile(r"Last voltage: (\d+)\.(\d+)V")
TEMP_RE = re.compile(r"Last temperature: (\d+)C")


def load(path):
    steps, volt, temp = {}, None, None
    with open(path, errors="replace") as f:
        for line in f:
            if m := STEP_RE.search(line):
                steps[int(m[1])] = int(m[2])
            elif m := VOLT_RE.search(line):
                volt = f"{m[1]}.{m[2]}V"
            elif m := TEMP_RE.search(line):
                temp = f"{m[1]}C"
    if not steps:
        sys.exit(f"{path}: no `T=..: avg=..` lines; did the sweep run?")
    return steps, volt, temp


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--poles", type=int, default=14)
    ap.add_argument("--step", type=int, default=100)
    ap.add_argument("logs", nargs="+")
    a = ap.parse_args()
    pairs = a.poles // 2

    engines = [load(p) for p in a.logs]
    for path, (steps, volt, temp) in zip(a.logs, engines):
        peak = max(steps, key=steps.get)
        last = max(steps)
        where = "still rising at end" if peak == last else f"falls above DShot {peak}"
        print(f"# {path}: {len(steps)} stable steps, peak {steps[peak]} RPM @ DShot {peak} ({where}); "
              f"pack {volt or '?'}, ESC {temp or '?'}")

    ceiling = min(max(s, key=s.get) for s, _, _ in engines)
    common = sorted(set.intersection(*(set(s) for s, _, _ in engines)))
    grid = range(48, ceiling + 1, a.step)
    wanted = [d for d in grid if d in common]
    for d in sorted(set(grid) - set(common)):
        print(f"# note: DShot {d} unstable on some engine; no table row there", file=sys.stderr)
    if not wanted or wanted[0] != 48:
        print("# WARNING: no stable DShot 48 step on every engine; table starts higher", file=sys.stderr)
    if ceiling not in common:
        sys.exit(f"ceiling DShot {ceiling} is not a stable step on every engine")
    if wanted[-1] != ceiling:
        wanted.append(ceiling)

    max_rpm = min(s[ceiling] for s, _, _ in engines) // 100 * 100
    max_erpm = max_rpm * pairs

    rows, prev = [], 0
    for d in wanted:
        if d == ceiling:
            continue
        rpms = [s[d] for s, _, _ in engines]
        erpm = sum(rpms) * pairs // len(rpms)
        if erpm <= prev:
            print(f"# dropped DShot {d}: eRPM {erpm} not above previous {prev} "
                  "(interpolation needs strictly rising eRPM)", file=sys.stderr)
            continue
        # Twin engines: the *average* can pass MAX_ERPM (the slower engine's reach)
        # below the ceiling. The pinned last row must stay the top, so those rows go
        # — not the ceiling row, which GOVERNOR_DSHOT_MAX is asserted against.
        if erpm >= max_erpm:
            print(f"# dropped DShot {d}: averaged eRPM {erpm} >= MAX_ERPM {max_erpm} "
                  "(slower engine can't reach it)", file=sys.stderr)
            continue
        rows.append((erpm, d, rpms))
        prev = erpm
    rows.append((max_erpm, ceiling, [s[ceiling] for s, _, _ in engines]))

    print(f"\nconst GOVERNOR_FF_TABLE: [(u32, u16); {len(rows)}] = [")
    for erpm, d, rpms in rows:
        what = f"avg({','.join(map(str, rpms))}) RPM × {pairs}" if len(rpms) > 1 else f"{rpms[0]:,} RPM"
        if d == ceiling:
            what = f"ceiling (MAX_ERPM); {what}"
        print(f"    ({erpm:_}, {d}),".ljust(22) + f" // {what}")
    print("];\n")
    print(f"MAX_RPM            = {max_rpm:_}")
    print(f"MAX_ERPM           = {max_erpm:_}   (MAX_RPM × {pairs})")
    print(f"GOVERNOR_DSHOT_MAX = {ceiling:_}")


if __name__ == "__main__":
    main()
