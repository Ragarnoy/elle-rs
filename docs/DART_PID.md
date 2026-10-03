# Dart attitude PID

> **Status (27 Sep 2026).** The "Next (fit)" gains below are now the firmware defaults
> on **both** airframes (`elle-config/src/lib.rs`). Since this note: `GOVERNOR_FF_TABLE`
> was re-swept on the 3-blade (26 Sep), the mixer-aware anti-windup, the 90°/s setpoint
> limit and the 30 Hz gyro-rate low-pass are live, and ULog logs the P/I/D terms and
> gains directly (`controller`, `pid_gains`), so the least-squares fit below is no longer
> needed to see what ran. The rest is kept as the record of how the gains were derived.

Engineering note from ULogs `LOG_0020`–`LOG_0035` (13 Sep 2026).
All times are **seconds from the start of the log**, not the boot-clock
timestamps stored in the file (those run ~4 s ahead; on absolute time the
0026 window lands in a throttle-0 segment and every number below changes).

Kp and Kd below are measured at rest (armed, throttle 0, so I ≡ 0) by least
squares on `corr = 5·(Kp·e − Kd·ω)`: Kp to ±0.002, Kd to ±0.005, with the two
columns near-orthogonal (|corr(Kp,Kd)| < 0.1). Ki is the compiled default that
rode along. None of the pre-0032 builds are in git — the at-rest fit is the
only record of what flew.

**Prop:** the 2-blade broke mid-session and a 3-blade went on before `LOG_0019`.
Every log used here (0020–0035) is on the 3-blade, so the comparisons below are
on one airframe. Do **not** fold in `LOG_0014`–`LOG_0018`: at the same pack
voltage those reach 87 k eRPM at DShot 1000–1200 and 141–146 k at full stick,
against 72–79 k and ~103 k for everything from 0020 on — a ~30% different plant
at the top end. `GOVERNOR_FF_TABLE` as committed describes the *broken* prop
(see the governor note in `elle-config`). *Since re-swept on the 3-blade — see the
status note above.*

Flash PID is currently ignored (`IGNORE_PID_FLASH`). *(The `clearpid` sector
erase this note warned about is gone: it now removes only the PID entry.)*

---

## Next hop (done — now the defaults)

The 0.35 / 0.20 step was an interpolation between “too soft” and “too hot”.
Replayed on the holds, it does **not** fit: at the 0026 sag it produces only
60% of the elevator that still failed to hold, and on the 0032 bank it
leaves a predicted 12° of residual wing-drop. The set below is the invert
of those two holds (derivation in [Fit to the logs](#fit-to-the-logs)).

| Term | Eagle | 0020 | 0026 | 0032–35 | Was proposing | **Next (fit)** |
|---|---|---|---|---|---|---|
| Pitch Kp | 1.00 | 0.50 | 0.50 | 0.25 | 0.35 | **0.45** |
| Pitch Ki | 0.10 | ~0.05 | ~0.05 | 0.025 | 0.020 | **0.020** |
| Pitch Kd | 0.25 | ~0.10 | ~0.11 | 0.0625 | 0.12 | **0.16** |
| Roll Kp | 0.50 | 0.40 | 0.125 | 0.125 | 0.20 | **0.25** |
| Roll Ki | 0.03 | ~0.024 | 0.0075 | 0.0075 | 0.012 | **0.012** |
| Roll Kd | 0.12 | ~0.10 | 0.03 | 0.03 | 0.06 | **0.07** |
| `PID_SCALE` | 5 | 5 | 5 | 5 | 5 | leave |
| `PID_I_LIMIT` | 0.5 | 0.5 | 0.5 | 0.5 | 0.5 | leave |

Leave `IGNORE_PID_FLASH = true`. Short Stabilized hops. Hold ~10–15° on the
stick through the throw.

Pitch 0.50 is the 0026 P that held (and then punched). We are not going
back to it: I is 40% of 0026’s (0.020 vs 0.05) and D is above 0026’s
(0.16 vs ~0.11). Roll 0.40 is the 0020 blow-up — 0.25 is halfway from
today to that wall.

---

## What actually ran

| Log | Pitch Kp | Roll Kp | Flight? | Notes |
|---|---|---|---|---|
| 0020 | 0.49 | **0.40** | hop | Roll too hot (corr up to 9, \|roll\| > 40° on 12% of samples) |
| 0024 | 0.50 | 0.125 | hop | Roll cut; pitch still 0.50 |
| 0026 | 0.50 | 0.125 | 3 hops | Pitch sag then I-punch. Flash empty |
| 0027–0031 | — | — | no | Never a throttle-up hop (0031 = mag cal / handling) |
| **0032** | **0.25** | 0.125 | 1 stab + Manual | First 0.25 hop |
| **0033** | **0.25** | 0.125 | 4 hops | Longest air time, +16.7 m baro |
| **0035** | **0.25** | 0.125 | 10 hops | Cleanest hold is the last one |
| 0049 | 0.45 | 0.25 (Kd 0.07) | 21 s Stabilized | 2026-10-03. Quiet: roll-rate 3–40 Hz RMS 22–51 dps, > 100 dps 0–4% of the time |
| 0050 | 0.45 | 0.25 (Kd 0.07) | 40 s Stabilized | ~4 Hz roll limit cycle, worse with throttle: 51 / 92 / 120 dps by throttle band (0.25 / 0.5 / 0.75+), > 100 dps 16% at high throttle |
| 0052 | 0.45 | 0.25 (Kd **0.04**) | 123 s Stabilized | Level cal applied (pitch 9.2°). Same limit cycle: 54 / 82 / 101 dps, > 100 dps 7 / 12 / 15%. Pitch-up blocked 43% of the time, 12° pitch sag, roll-left blocked 36% |

0050 → 0052: cutting roll Kd 0.07 → 0.04 did not remove the 3.9 Hz roll
oscillation. In an episode the summed roll command passes ±1 and the elevons
go stop to stop every ~125 ms; while they sit at the stops a smaller gain
changes nothing, which may be why the cut had so little effect. The logs at roll Kp 0.125 / Kd 0.03 (0032–0035) stayed at 34–53 dps
with > 100 dps under 4% of the time; the oscillation appeared with Kp 0.25.
No GNSS fix in any of these logs, so throttle is the only speed proxy.

0032 / 0033 / 0035 confirm the compiled 0.25 / 0.125 defaults flew.
`IGNORE_PID_FLASH` did its job.

---

## Fit to the logs

### What we can prove

The firmware PID on a logged tick is

```
u = 5 · (Kp · e + Ki · I + Kd · (−ω))
İ = e ,   I ∈ [−0.5, +0.5] ,   I ← 0 if disarmed or throttle < 0.05
e = (filtered setpoint) − AHRS
```

Replaying that forward on the hold windows, with the gains above, gives
per-window RMSE against the logged correction:

| Window | pitch | roll |
|---|---|---|
| 0026 hold 58–65 s | **0.244** | 0.055 |
| 0032 hold 76–80 s | **0.024** | 0.065 |
| 0035-j 621–628 s | 0.092 | 0.059 |
| 0035-a 75–84 s | 0.086 | 0.090 |

Only 0032 is a tight reproduction. The 0026 residual is *not* a wrong-gain
guess — refitting Kp/Ki/Kd freely on that window still leaves 0.159, so it is
real unmodelled scatter (attitude/command skew while the airframe is moving
fast). **The tick-by-tick loop is verified on 0032 only.** The invert below
uses window *means* of `u_d` and `e`, and those do reproduce on all four
windows — but 0026, which drives the whole pitch recommendation, rests on the
mean, not on a matched trace.

A 2nd-order plant `θ̈ = aθ̇ + bθ + c u + …` fitted on the same windows
has **R² ≈ 0** (gyro/AHRS acceleration is noise). A 1st-order
`θ̇ = A θ + B u + …` gets R² > 0.95, but `B = ∂θ̇/∂u` is the same on
every log because `u` is already a function of `θ` (closed-loop
identity, not the airframe). Those fits are **not** used for gains.

What is used: on a quasi-steady hold, the logged `u` *is* the elevator
the plant required at that attitude/throttle. Call it `u_d`. If that
disturbance stays put (thrust moment, a parked bank), the P+I loop
balances it at

```
e_ss = (u_d − 5 · Ki · I_lim) / (5 · Kp)     (I saturated, D ≈ 0)
```

Inverting the **flew** gains on the measured `u_d` recovers the measured
error where I was actually sat:

| Hold | `u_d` | `e` measured | Invert (I-sat) | Invert (P-only) |
|---|---|---|---|---|
| 0026 pitch 58–65 s | +0.340 | +6.6° | +4.9° | +7.8° |
| 0032 roll 76–80 s | −0.239 | −20.4° | **−20.2°** | −21.9° |
| 0035-j roll | −0.112 | −8.3° | **−8.5°** | −10.3° |
| 0035-j pitch | +0.086 | +5.4° | +1.1° | +3.9° |

Roll is a P loop (I adds 0.019 at the limit — nothing). The invert is
exact. 0026 pitch sits between P-only and I-sat (I was at 0.5, D and the
setpoint filter eat a bit). 0035-j pitch logged `u` is closer to P-only
— do not trust the I-sat column there.

### The 0.35 / 0.20 step fails this invert

Same `u_d`, new gains, predicted `e_ss`:

| Hold | Measured `e` | 0.35 / 0.020 / 0.20 | 0.45 / 0.020 / 0.25 | 0.50 / 0.020 |
|---|---|---|---|---|
| 0026 pitch (`u_d`=0.34) | +6.6° | **+9.5°** (worse) | +7.4° | +6.6° |
| 0032 pitch (`u_d`=0.22) | +10.4° | +5.7° | +4.4° | +4.0° |
| 0032 roll (`u_d`=−0.24) | −20.4° | **−12.0°** | **−9.6°** | — |
| 0035-j pitch (`u_d`=0.09) | +5.4° | +1.2° | +0.9° | +0.8° |
| 0035-j roll (`u_d`=−0.11) | −8.3° | −4.7° | −3.8° | — |

0.35 is sized for 0035-j (small `u_d`) and **loses** the 0026 hold — the
hop we used to diagnose sag-then-punch. 0.20 roll still leaves a 12°
parked bank on the 0032 walk; that is not “reacts in time”.

Same-state replay (new PID on the logged attitude, not a plant sim)
says the same thing: on 0026, 0.35 / 0.020 commands **0.21 vs 0.34**
logged (0.60×). On 0032 roll, 0.20 commands 0.37 vs 0.24 (1.57×) — more
bite, not enough to kill a 20° walk.

### Why 0.45 / 0.25 / more D

**Pitch Kp 0.45** is the lowest P that gets 0026 `e_ss` back near the
measured 6.6° (7.4°) without returning I to 0.05. **Ki 0.020** is
deliberate: 0026’s punch was I sat at 0.5 contributing **0.125** of
elevator; 0.020 contributes **0.050**.

Net up-elevator at leap *onset* (e = 10°, I sat, ω = 15 °/s — the 0026
break starts at low rate):

```
u_net = 5·Kp·e + 5·Ki·0.5 − 5·Kd·ω
```

| Gains | P | I | D | **u_net** |
|---|---|---|---|---|
| 0026 (punched) | 0.436 | 0.125 | 0.144 | **0.417** |
| 0032–35 (soft) | 0.218 | 0.062 | 0.082 | 0.199 |
| 0.35 / 0.020 / 0.12 | 0.305 | 0.050 | 0.157 | 0.198 |
| **0.45 / 0.020 / 0.16** | 0.393 | 0.050 | 0.209 | **0.233** |
| 0.50 / 0.020 / 0.16 | 0.436 | 0.050 | 0.209 | 0.277 |

0.45 / 0.16 has 56% of 0026’s punch drive and 45% more D than 0026 at the
same rate (0.209 vs 0.144). Kd 0.12 is barely above 0026’s measured ~0.11,
and 0026’s D did not stop the leap.

**Roll Kp 0.25** is the invert of `u_d = −0.24` to ~10° residual
(from 20°). 0.20 → 12°, 0.30 → 8°, 0.40 → 6° and that is the 0020
oscillation. Kd 0.07 is ~2× today, under 0020’s ~0.10.

### Mix budget at the fitted gains

P-only `busy = max(|p±r|)` on Stabilized, throttle > 0.2:

| Log | Today p50 / p90 | 0.35/0.20 sat | **0.45/0.25 sat** |
|---|---|---|---|
| 0032 (diagnostic) | 0.30 / 0.51 | 3% | **8%** |
| 0026 | 0.49 / 1.17 | 10% | 16% |
| 0033 / 0035 | 0.45–0.48 / 0.95 | 19% | **34–36%** |

0033/0035 sat is full-stick chaos, not the hold. The 0032 hold stays
under the clamp. If the next hop’s *hold* (not the arrival) shows mix
busy p90 > 1, stop raising P and add a cruise pitch offset.

### What this invert is not

`u_d` is assumed constant in attitude. If the missing elevator is
mostly aero stiffness, a smaller error needs less `u` and the real
`e_ss` is better than the table. If it is a thrust bias (the 0026
level-wing sag), the table is the right one and 0.35 loses.

---

## How the surfaces are shared

Elevon mix is 1:1, then clamp:

```
left  = clamp(pitch − roll + 0.1·yaw, ±1)
right = clamp(pitch + roll − 0.1·yaw, ±1)
```

A mean pulse near 1500 µs can hide one surface already at a stop. Judge
authority by `busy = max(|pitch−roll|, |pitch+roll|)`, not by the average
elevon.

On 0032’s hold, mean elevon was only 92 µs off center while the busier
surface was doing all the work (busy p50 = 0.47). Still not saturated — both
loops were late, not out of throw.

Replaying 0033 / 0035 with 0.45 / 0.25 (P-terms only) takes mix saturation
from ~9% of samples to ~35% — that is the chaotic full-stick part of those
logs. The 0032 diagnostic hold stays at 8%. Going back to pitch 0.50
*and* roll 0.40 would spend the rest on the two known-hot corners.

`ATTITUDE_MAX_AUTHORITY` (0.8) is defined and **not applied**. Each axis can
ask for ±1 and the mix clips per surface. Leave it unused until a hop with
the new gains shows one axis starving the other.

---

## Pitch, with roll in the picture

The 0.25 cut **did** fly. The I *term* halved with Ki — its p90 contribution
to `u` is 0.062 against 0026’s 0.125. But the integrator itself is pinned at
the ±0.5 clamp just as often: |I| p90 = 0.500 on 0026, 0032, 0033 **and**
0035. Nothing about the winding changed, only the gain on it — and Ki 0.020
will saturate the same way, for a constant 0.050. Two pitch problems remain,
and they are not the same event.

### 1. Steady sag — real, and not caused by roll

Holds with throttle up and a steady stick:

| Hop | Kp | Bank | Pitch err | Pitch corr | Roll corr | Mix busy | Elev sat |
|---|---|---|---|---|---|---|---|
| 0026 58–65 s | 0.50 | **3.5°** | **+6.6°** | +0.34 | 0.00 | 0.37 | 0% |
| 0032 76–80 s | 0.25 | **20°** | **+10.4°** | +0.22 | −0.24 | 0.47 | 0% |
| 0035 621–628 s | 0.25 | **8°** | **+5.4°** | +0.09 | −0.11 | 0.19 | 0% |

0026 sagged **6.6° with wings level**. That is a pitch-loop / plant problem
(thrust pitching moment + not enough elevator at the error), not a roll
steal. 0035’s “clean” hop still sagged 5° with mix almost idle (busy 0.19) —
today’s P-term is 0.09 at 5° error; the 0026 hold needed ~0.34 of pitch
command (P + I) to do the same job.

Stratified: on 0026, pitch |err| p50 is **9.3° at |bank| < 8°** (n = 1785).
On 0035, **12.3° at |bank| < 8°**. Bank does not create the sag.

0032 is the coupled case: 10° of pitch sag *and* 20° of uncommanded bank.
Roll is using as much correction as pitch (−0.24 vs +0.22). Raising only
pitch would spend the extra throw on a wing that is still walking out.

### 2. The leap — 0026 is I; later hops are “late, then it breaks”

Pitch rises of ≥15° in 0.40 s, Stabilized, throttle up:

- **0026 t = 64.9 s (the original punch):** +1° → +16°, **bank +2.3°**, mix
  busy 0.64. Wings level, surfaces not on the stops. I was hard against the
  +0.5 clamp, worth 0.125 of elevator. This is the integrator winding while
  the nose sat low, then unloading.
- **0032 t = 79.9 s:** −5° → +11°, **bank +21°**, mix busy 0.50. Both axes
  late; mix still has room. Then full stick and it comes apart.
- **0033 / 0035:** most leaps are full-stick (setpoint 25°) porpoise, often
  after roll is already lost. Period ~0.5–1 s, amplitude ±20–30°.

Do not raise Ki. 0026 already showed what I does at pitch Kp 0.50. More D
is to catch the 30–80 °/s break that still ends every hop. More P (to 0.45,
not 0.50) is so a 6–10° error produces elevator *now*, instead of waiting
for I.

At 10° error, P-term = `5 · Kp · 0.175`:

| Kp | P-term |
|---|---|
| 0.25 (today) | 0.22 |
| 0.35 (old rec) | 0.31 |
| **0.45 (next)** | **0.39** |
| 0.50 (0026) | 0.44 + I → punch |

---

## Roll

Leaving roll at 0.125 was wrong. The “won’t react in time” report is in the
log as an uncommanded bank that the loop does not pick up.

On 0032, roll stick was **centered** (wants 0°) from 70–80 s. Bank walked
0° → **+21° over 7 s**. Correction sat at −0.09 to −0.30. At 20° error,
P = `5 · 0.125 · 0.35 ≈ 0.22`. That is the late wing.

0035’s clean hop held **8° of uncommanded bank** the whole time (corr ≈
−0.11), then went fully inverted (|roll| reaches 180°) when pitch leaped.
Six other 0035 windows sat at 12–38° of uncommanded bank for 1.4–3.2 s with
corr still around −0.2.

0020’s roll Kp **0.40** is the other wall (max corr 9). Next is **0.25**
(2× today, still under that wall), with D 0.07 and I barely moved
(0.012). Same shape as the pitch step.

---

## Why the two axes have to move together

1. Mix is 1:1. A roll loop that sits at −0.24 forever is a permanent 0.24
   bias on one elevon. Pitch then looks “soft” on that side.
2. A 20° bank the loop will not pick up is not a pitch-tracking problem,
   but it *is* a pitch-flying problem (the wing is not flying the heading
   you think, and the next leap dumps roll).
3. Raising only pitch Kp on a 0032-shaped hop spends throw on the nose
   while the wing is still walking. Raising only roll leaves the 0026 sag
   (which happens at 3° bank) untouched.

Replay of fitted P-terms on the 0032 hold (12° pitch err + 21° roll err):
busy goes 0.50 → ~0.93. Still under the clamp. The next hop has room.

---

## What this is not

- **Not established either way: phugoid.** The “0.3–0.4 s” figure was the
  detector window (≥15° rise in 0.40 s), not the event. Onset to peak is
  **2.7 s** on both named leaps (0026 t=64.5: +2° → +26°; 0032 t=79.5:
  −5° → +30°). That is not evidence of a phugoid, but it does not exclude
  one — it is a single divergence, not a sustained oscillation.
- **Not flash PID.** Every hop of the day logged empty or skipped flash.
  0.50 then 0.25 are compiled defaults.
- **Not “cut P again.”** 0.25 already leaves 5–12° of level-wing sag with
  unused mix.
- **Not “back to 0.50 / 0.40.”** Those are the two known-hot corners
  (pitch punch, roll blow-up).
- **Not mix starvation** on the diagnostic holds. 0026, 0032, and 0035-j
  holds are at 0% elevon-stop. Saturation shows up *after* the leap or on
  full-stick 0033 chaos.

---

## How to read the next log

After the hop, sitting armed with throttle 0:

- pitch Kp must measure **≈ 0.45** (slope `corr / error_rad / 5`)
- roll Kp must measure **≈ 0.25**

If you see 0.25 / 0.125, the new binary did not fly.

In the hop itself:

- A 7 s walk to +20° bank with stick-center means roll is still too soft.
- A 5–8 s sit 6° below a steady 10–16° setpoint with mix busy < 0.5, then a
  0.3 s leap through the target, means pitch P/D is still short (and I must
  stay down).
- Mix busy p90 > 1.0 *during the hold* (not during the crash) means we are
  out of throw and should not raise P further — add trim / a cruise pitch
  offset instead.

---

## Later (not this hop)

- **Cruise pitch offset.** Stick-center = 0° AHRS. The wing wants ~10–15°
  to climb at these throttles. A +8° Stabilized trim would stop the loop
  fighting the deck angle and the thrust moment. That is a setpoint change,
  not a substitute for the gain step.
- **`ATTITUDE_MAX_AUTHORITY`** actually applied, or a mix that prioritises
  roll when both axes ask for > 0.7. Only after a hop proves one axis is
  starving the other.
- ~~**`clearpid`** rewritten as a map-key delete, not a 64 KB sector erase.~~
  Done. `IGNORE_PID_FLASH` can go once a dart bench run confirms it (TEST_PLAN 6.3).
