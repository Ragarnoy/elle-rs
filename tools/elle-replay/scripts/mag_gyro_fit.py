"""Which signed axis permutation maps the magnetometer chip onto the IMU? (proposal 0010)

For a constant earth field, each mag sample must equal the previous one rotated by
the gyro in between (dm/dt = -w x m, body frame). For each of the 48 signed
permutations P, with the hard-iron offset u fitted jointly, the residual of
  P m_j - u = R (P m_i - u)
over every pair of mag samples between which the aircraft turned more than 3 deg
says how well P explains the data. Uses the raw chip-frame `magnetometer_data` and
the gyro rates in `attitude_data` (any build), undoing the board rotation to get
the IMU's sensor frame. Best run on a mag-cal tumble (events 110 -> 111).

  logs/.venv/bin/python tools/elle-replay/scripts/mag_gyro_fit.py LOG.ulg T0 T1 [--board-yaw 180]

A map and its negation (a mirror) always tie; the datasheet settles that (rev A: the
proper rotation).

T0/T1: seconds from the first attitude sample. --board-yaw: BOARD_YAW_DEG of the
airframe the log is from (eagle 180, dart 0). The raw mag stays in the chip frame on
every build, so the answer is the chip-to-IMU map (rev A: (-my, +mx, +mz),
MAG_AXES_YAW_DEG 90), whatever the firmware does with it.
"""
import argparse
import itertools

import numpy as np
from pyulog import ULog

ap = argparse.ArgumentParser()
ap.add_argument("log")
ap.add_argument("t0", type=float)
ap.add_argument("t1", type=float)
ap.add_argument("--board-yaw", type=float, default=180.0)
a = ap.parse_args()

u = ULog(a.log, ["magnetometer_data", "attitude_data"])
m = u.get_dataset("magnetometer_data").data
mt = m["timestamp"].astype(float)
raw = np.stack([m["mag_x"], m["mag_y"], m["mag_z"]], 1).astype(float)
att = u.get_dataset("attitude_data").data
t = att["timestamp"].astype(float)
# attitude_data rates: pitch_rate = gy, roll_rate = -gx, yaw_rate = gz (airframe).
g_air = np.stack([-att["roll_rate"], att["pitch_rate"], att["yaw_rate"]], 1)
c, s = np.cos(np.radians(a.board_yaw)), np.sin(np.radians(a.board_yaw))
board = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
g = g_air @ board  # rows: board^T g_air = sensor frame
T0 = t[0]


def rot(w, dt):
    th = np.linalg.norm(w) * dt
    if th < 1e-12:
        return np.eye(3)
    k = w / np.linalg.norm(w)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K


def pairs(shift_us):
    out = []
    for i in range(len(mt) - 1):
        lo_t, hi_t = mt[i] + shift_us, mt[i + 1] + shift_us
        if lo_t < T0 + a.t0 * 1e6 or hi_t > T0 + a.t1 * 1e6 or hi_t - lo_t > 150e3:
            continue
        lo, hi = np.searchsorted(t, lo_t), np.searchsorted(t, hi_t)
        R = np.eye(3)
        for k in range(lo, hi):
            dt = (t[k + 1] - t[k]) / 1e6 if k + 1 < len(t) else 1e-3
            R = rot(-g[k], dt) @ R
        ang = np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))
        if ang > 3:
            out.append((i, R))
    return out


def name(perm, sg, det):
    s = ", ".join(("+" if sg[i] > 0 else "-") + "m" + "xyz"[perm[i]] for i in range(3))
    return f"({s})" + (" mirror" if det < 0 else "")


best = None
for shift in (-40e3, -20e3, 0, 20e3, 40e3):
    pr = pairs(shift)
    rows = []
    for perm in itertools.permutations(range(3)):
        for sg in itertools.product([1, -1], repeat=3):
            P = np.zeros((3, 3))
            for r in range(3):
                P[r, perm[r]] = sg[r]
            A = np.vstack([R - np.eye(3) for _, R in pr])
            b = np.concatenate([R @ (P @ raw[i]) - P @ raw[i + 1] for i, R in pr])
            off = np.linalg.lstsq(A, b, rcond=None)[0]
            res = np.sqrt(np.mean((A @ off - b) ** 2)) / 16384
            rows.append((res, name(perm, sg, np.linalg.det(P)), P.T @ off))
    rows.sort(key=lambda r: r[0])
    ident = next(r for r in rows if r[1] == "(+mx, +my, +mz)")
    print(f"shift {shift / 1e3:+4.0f} ms, {len(pr)} pairs: best {rows[0][1]} {rows[0][0]:.4f} G"
          f" | 2nd {rows[1][1]} {rows[1][0]:.4f} | 3rd {rows[2][1]} {rows[2][0]:.4f}"
          f" | identity {ident[0]:.4f}")
    if best is None or rows[0][0] < best[0][0]:
        best = rows
print("fitted offset (chip frame, counts) for the best map:", best[0][2].round(0))
