# Spline Controller Jitter — Diagnosis & Two Fixes

**Date:** 2026-07-22
**Scope:** Teensy 4.1 firmware (`ExoCode/src/Controller.cpp`, `ExoCode/src/Side.cpp`).
**Status:** ~~Both fixes committed on branch `add_remote_control`~~ — **updated 2026-08-12: that
branch no longer exists.** Both fixes live on **`fix_spline_jitter`**: `7f95828` = the spline gain
scheduler, `226854b` = the `Side.cpp` int→float `percent_gait` change. Still **pending bench
validation** (nothing was run on the motors during this investigation).
**Round 2 / follow-ups:** `Spline-Jitter-Round-2-SD-Logging-Regression.md`, then
`Spline-Run-Analysis-And-RT-Stream-Fix.md` (2026-08-12), which re-derives this ground independently
and extends it — see its F3 (the D-term noise gain this document's root cause #1 addresses) and F7
(the "end nodes must be zero" invariant flagged under root cause #3 below).

---

## The complaint

With PID enabled and the *same* nominal gains entered for both controllers (p = 6, d = 0.03, i = 0),
the **Spline** controller made the leg visibly shake, while **PJMC** was smooth. "Identical settings,
very different behavior."

## How this was investigated

Evidence-first, using the on-device SD logs rather than guessing. The current (07-16) logs live in
`Test results/Motor logs/{0001,0002}/` at ~100 Hz with `Gait_phase`, `Desired_Torque_Nm`,
`Commanded_Torque_Nm`, and `Torque_Nm` columns. **0001 = Spline, 0002 = PJMC** (identified by the
desired-torque value set matching each controller's node/formula output). Note the older
`Test results/{PJMC,Spline}.csv` are stale 07-03 GUI exports and were *not* used for conclusions.

Four contributing causes were found. Two were real, controller-specific jitter sources and were fixed.
One turned out to be a non-bug. One is a system-wide loop-rate issue the user had already improved.

---

## Root cause #1 — the gains were never actually identical (FIXED)

PJMC has a **near-zero-torque gain scheduler** (`ProportionalJointMoment::calc_motor_cmd`) that Spline
did not. When the commanded torque is within a small band of zero **and** the measured torque is close
to it, PJMC swaps its nominal gains for a much gentler set read from `PJMC.csv`
(`GS_Flag=1, kp_zero=3, ki_zero=0, kd_zero=0.001`).

Replaying that predicate against the PJMC log showed the scheduler is active **82.8% / 87.0% (L/R)** of
samples. So PJMC was effectively running **p = 3, d = 0.001** almost the whole time — not the p = 6,
d = 0.03 in its CSV. Spline had no scheduler, so it ran the full nominal gains 100% of the time:
**2× the P gain and 30× the D gain.** On a noisy torque signal, that high D term is exactly what turns
sensor noise into audible/visible buzz.

### The fix
Added the same near-zero gain scheduler to `Spline::calc_motor_cmd` (`Controller.cpp`), with the same
0.5 Nm setpoint band and 3.5 Nm error band as PJMC.

- **Near-zero gains are HARD CODED** at `3 / 0 / 0.001` (matching `zeroTorque.csv` and PJMC's
  `kp_zero/ki_zero/kd_zero`) and are flagged in the code as **not** GUI- or SD-settable.
- **Nominal gains are untouched** — still read from the controller parameters, so the GUI can change
  them.
- A `TODO` notes the intended future improvement: source the near-zero gains from the ZeroTorque
  controller's parameters instead of hard-coding, so the transparency gains live in exactly one place.

Replaying the new predicate against the Spline log: engages **69.7% / 88.2% (L/R)** — parity with PJMC,
so the D gain drops 30× in exactly the near-zero regime where the shaking was visible.

---

## Root cause #2 — `percent_gait` was integer-quantized (FIXED)

`Side::_calc_percent_gait()` (and `_calc_percent_stance` / `_calc_percent_swing`) computed the gait
percentage in float, **assigned it into an `int`** (truncating to whole percent), then returned it
through the `float` signature. The float-ness was decorative; the fraction was destroyed.

This is an upstream OpenExo bug present since the **first commit** (248fa9f, 2023-04-13) — every commit
since only touched whitespace/comments/the Leg→Side rename. The dead `percent_gait_x10` debug comments
are the fossil of a pre-git fixed-point ×10 representation (0–1000 = 0.0–100.0%), where `int` *was*
correct. When someone dropped the ×10 scaling they left the `int` behind.

### Why it hit Spline and not PJMC
Spline's output is a direct function of `percent_gait` — quantize the input, you quantize the output.
PJMC's setpoint comes from the analog toe FSR (`scaled_fsr * stance_max`), which is continuous and has
no grid to snap to. Same PID, but one controller is fed a smooth signal and the other a staircase.

### The mechanism (from real stride timing)
Median stride ≈ 1228 ms (from `Ground_strike_log.txt`), so 1% of gait = ~12.3 ms. The control loop
runs faster than that, so `percent_gait` **holds constant for several cycles, then ticks +1 and the
setpoint jumps**. On the steep part of the current spline pulse, one 1% tick = up to **1.69 Nm**. The
PID command is `cmd = setpoint + p_gain*(setpoint − measured)`, so:

    1.69 Nm setpoint step  ×  (1 + p_gain 6)  =  ~11.8 Nm command discontinuity, ~81 Hz

Crucially, the P term punishes **tracking error**. During the frozen cycles the loop settles (error → 0);
then the setpoint teleports and manufactures the *maximum* possible error right when the plant had
caught up. That self-inflicted sawtooth is what read as "PID jitter" — it scales with p_gain but has
nothing to do with the human, which is why re-tuning never fixed it. (The D term is
derivative-on-measurement, not on error, so these steps do *not* also spike D — a lucky escape.)

### The fix
Changed the three functions to `float` (locals, literals, `min(..., 100.0f)`), removed the stale
`percent_gait_x10` comments. Resolution is now bounded by `millis()` (1 ms ≈ 0.08% of a stride) instead
of the 1% grid.

Simulated at real stride timing (current nodes, p = 6):

| Metric (active pulse) | int (before) | float (after) |
|---|---|---|
| Max command jump @500 Hz | 11.8 Nm | 1.9 Nm |
| Max command jump @280 Hz | 11.8 Nm | 3.5 Nm |
| Distinct setpoint values across pulse | 25 | 160 |
| Cycles with a frozen setpoint | 84% | 0% |

The benefit is **capped by loop rate** — the controller only reads `percent_gait` once per cycle, so a
faster loop (see #4) makes this fix worth more. The two compound.

This also strictly smooths the other percent-gait controllers (ZhangCollins, FranksCollinsHip) with no
behavior inversion. Storage was already `float` (`SideData.h`), the SD logger already `%.2f`, and the
`-1` sentinel survives (all consumers use `<`/`>=`, no integer `== -1`).

---

## Root cause #3 — the `-1` gait dropout (INVESTIGATED, NOT A BUG — no change made)

Initially flagged because `percent_gait == -1` for ~24.5% of the Spline log. That was a **bad
denominator** — it counted the pre-trial FSR-calibration phase. Narrowing the window:

- during `trial_on`: 6.6% / 7.6%
- **during the window where Spline actually drove torque: 0.00%, both legs.**

The `-1` is a **boot transient only**. `SideData.cpp` inits `expected_step_duration = -1`, and
`_update_expected_duration()` has **no path that re-zeroes it**, so it can only appear once per power
cycle, before the gait clock is established — by which point the ankle is still on the default
zeroTorque controller. The single `-1 → valid` transition in the log had a **0.00 Nm** setpoint step,
because `-1` falls through `_spline_interpolate`'s `pg <= x[0] → y[0]` clamp and y[0] = 0 (safe zero).

Nothing to fix. Two side findings from the same look, though, both worth knowing:

- **Undocumented invariant:** the end clamps are safe *only* because current layouts have y[0] = y[4] =
  0. A layout with nonzero end nodes gives constant torque before the first stride, and — the dangerous
  one — **standing still pegs `percent_gait` at 100 forever, so the exo holds y[4] torque
  indefinitely.** Spline end nodes must be zero. (Consider a code comment / assert.)
- **The gait clock finishes early a lot:** during the spline-active window `percent_gait` sits
  saturated at 100 for **21.4% / 32.0% (L/R)** of samples — the real stride outruns
  `expected_step_duration` about a third of the time, so the profile completes and dead-times until the
  next strike. Not a jitter source, but more evidence the open-loop gait clock tracks this hardware
  poorly vs FSR-proportional PJMC.

---

## Root cause #4 — control loop ran ~280 Hz, not 500 Hz (out of scope here)

In these 07-16 logs the loop ran at ~280 Hz (`ran/s` median 280, min 51; `maxLoop` up to 287 ms), well
below `LOOP_FREQ_HZ = 500`. `_pid` gates its D and I terms on `dt <= 2200 us`, so at that rate the D
term was discarded most cycles and flickered on intermittently. `maxSD ≪ maxLoop` points the stall at
`exo.run()` (control / CAN / UART), **not** the SD logger.

**The user reports #4 has since been improved (after these logs were captured).** It was not
re-measured in this session and the thread was closed here. If spline jitter persists after
bench-testing #1 + #2, re-profile the loop rate rather than re-touching gains — a faster loop also
raises the ceiling on the #2 fix.

---

## Files changed

| File | Change |
|---|---|
| `ExoCode/src/Controller.cpp` | Added near-zero gain scheduler to `Spline::calc_motor_cmd` (hard-coded 3/0/0.001, nominal gains still from params). Commit `7f95828`. |
| `ExoCode/src/Side.cpp` | `_calc_percent_gait` / `_calc_percent_stance` / `_calc_percent_swing`: `int` → `float`; `min(...,100)` → `min(...,100.0f)`; removed stale `percent_gait_x10` comments; added an explanatory block comment. |

## Verification done (no hardware)

- Both changed code blocks were transcribed into standalone harnesses and compiled clean under
  `g++ -std=c++14 -Wall -Wextra`, emulating Arduino's `min()` **macro** exactly.
- Gain-scheduler branch table checked at both band edges and the `use_pid=0` passthrough.
- `percent_gait` float version checked for fractional resolution, saturation clamp at 100, `-1`
  sentinel preservation, `percent_stance` zeroing on `toe_stance=0`, and full-stride monotonicity /
  no-NaN.
- New scheduler predicate replayed against the real Spline log to confirm engagement parity with PJMC.

**Not verified:** no Teensy build, nothing run on the motors. Both fixes remain pending bench
validation.
