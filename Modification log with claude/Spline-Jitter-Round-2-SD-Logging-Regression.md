# Spline Jitter, Round 2 — SD-Logging Regression, Torque-Filter Mismatch, Cubic Overshoot

**Date:** 2026-07-23
**Scope:** `ExoCode/src/Controller.cpp`, `SDCard/config.ini`, `SDCard/ankleControllers/spline.csv`.
**Status:** **RESOLVED on-device.** User reports the spline no longer feels jittery after flashing the
firmware change *and* setting `Node5_x = 20.5` together. Changes are in the working tree, **not
committed**.
**Read alongside:** `Modification log with claude/Spline-Jitter-Diagnosis.md` (round 1 — the gain
scheduler and the `percent_gait` int→float fix; this log continues directly from it) and
`Modification log with claude/SD-Card-Logging-and-End-Trial-Reset.md` (the logger this one indicts).

---

## The complaint

Round 1's two fixes were flashed, and the spline **still** felt much jitterier than PJMC. Same
premise as before: identical nominal PID gains, visibly different behavior.

## Round 1's fixes were verified working — the premise finally held

Before looking for anything new, both round-1 fixes were confirmed live on-device from SD log 0009:

- **Gain scheduler at parity.** Replaying the firmware predicate against the log: scheduler engaged
  **76.7% / 77.6%** of spline samples vs **76.6%** for PJMC (right leg). Round 1's core finding — that
  PJMC was silently running its low gains most of the time and spline wasn't — is fixed.
- **`percent_gait` is continuous.** `Gait_phase` logs as `0.50, 4.67, 7.83, 10.17…`, not integers.

So the gains really were identical this time, and the jitter was still there. That is new information,
not a failed fix.

## The reframe — it was never primarily a spline bug

The decisive data point came from the user, not the analysis: they re-flashed the **old** branch
(`backup_branch_with_UW_edits`, 6a47f63, no SD logging) and reported PJMC there had essentially zero
jitter.

`ProportionalJointMoment::calc_motor_cmd` is **byte-identical** between the two branches, and
`SDCard/ankleControllers/PJMC.csv` is identical too (`0,0,1,1,6,0,0.03,1,1,3,0,0.001`). Same code,
same gains, same GUI streaming path — so any difference is environmental.

Comparing GUI logs only (same measurement chain both sides, ~65 Hz), first 25 s of each PJMC segment
to control for in-trial drift, matched peak torque:

| PJMC, quiet/swing HF residual | L | R |
|---|---|---|
| OLD branch (no SD logging) | 0.903 | 0.719 |
| NEW branch, **logging on** | 1.326 | 0.984 |
| NEW branch, **logging off** (`sdLogEnabled = 0`) | **0.991** | **0.696** |

**SD logging was costing real control quality.** Turning it off recovered ~79% of the gap on the left
and all of it on the right. In the pulse phase, logger-off beat the old branch outright (1.41–1.71 vs
2.69 on L).

Mechanism (consistent with the evidence, but **not proven** — with logging off there is no
`debug_log.txt`, so `ran/s` can't be measured): with the logger on the loop ran at **~262 Hz, not 500**,
and `_Controller::_pid` gates its derivative term on `dt <= expected*(1+tol)` = 2200 µs. At 262 Hz,
dt ≈ 3800 µs, so **the D term — the damping term — is silently dead**, and `d_gain` does nothing for
*any* controller. There was also a ~12 ms stall in 96 of 126 one-second buckets, though `maxSD`
(3–6 ms) ≪ `maxLoop` (12 ms), so the SD write itself is not obviously the whole cost.

---

## The fixes

### A. `sdLogEnabled = 0` (`SDCard/config.ini`)

Read at boot — **no reflash needed**. This is the change that recovered PJMC to old-branch parity.
Treat SD logging as something that trades control quality for observability, not as free.

### B. Torque-measurement filter: match PJMC (`Controller.cpp`, in `Spline::calc_motor_cmd`)

```cpp
// was: utils::ewma(_joint_data->torque_reading, _controller_data->filtered_torque_reading, 0.5f)
_controller_data->filtered_torque_reading =
    utils::ewma(_joint_data->torque_reading, _controller_data->filtered_torque_reading, 1.0f);
```

The spline hard-coded an EWMA alpha of `0.5`; PJMC reads its `torque_alpha` param, which is **1** in
`PJMC.csv` — i.e. PJMC runs the **raw** reading with no filtering at all. In **late swing (gait
60–95%) both controllers command setpoint == 0 with identical scheduled gains**, so the code paths
should be identical; the alpha was the only remaining difference. Measured in that window (SD log
0009, raw `Torque_Nm` — same sensor chain for both): `Filtered != raw` in **94.8% / 83.3%** of spline
samples vs **0.0%** for PJMC, and spline showed **32–47% higher measured-torque RMS**, consistently in
both spline segments and both legs.

A low-pass *inside* the feedback loop adds phase lag, erodes phase margin, and lets the loop ring
rather than damp. Side benefit: with alpha = 1 the spline now reports raw torque like PJMC, so the
GUI's "Measured Torque" column is finally comparable between the two controllers.

**Still hard-coded.** TODO: promote to a real parameter like PJMC's `torque_alpha_idx` (needs a 17th
column in `spline.csv`).

### C. Kill the cubic overshoot: `Node5_x` 26 → 20.5 (`spline.csv`)

```
0,0,5,-8,10,-12,20,0,20.5,0,0,1,1,6,0,0.03
```

`Spline::_spline_interpolate` is a **natural cubic**. With nodes `(0,0)(5,-8)(10,-12)(20,0)(26,0)`, the
long flat `(20,0)→(26,0)` run after a steep rise forces the curve to bulge **positive** — verified by
replicating the interpolator exactly: **+0.887 Nm at 22.5% gait, every step** (logged `pos_lobe`
+0.69…+0.81; PJMC is exactly 0.00). Two problems, the second worse than the first:

1. It is **wrong-direction (dorsiflexion) torque at toe-off**.
2. At 0.887 Nm it **exceeds the 0.5 Nm gain-scheduler band**, so it kicks the controller out of
   transparency gains onto **p = 6 while the foot is unloaded**.

Shrinking the final span collapses the bulge; the `pg >= x[4] → y[4]` clamp holds flat zero after.
Candidates evaluated against the real interpolator:

| layout | max positive | peak | assist lobe |
|---|---|---|---|
| `…20,0,26,0` (old) | **+0.887** @22.6% | −12.020 @9.7% | 0.3–19.4% |
| `…20,0,20.5,0` (**chosen**) | **+0.010** | −12.061 @9.6% | 0.3–18.9% |
| `…18,-1.5,20,0` | +0.000 | −12.062 @9.6% | 0.3–19.2% |

`20.5` was chosen because it changes exactly one number and leaves peak magnitude, peak timing, and
lobe width essentially untouched.

---

## Attribution: deliberately not isolated

B and C were applied **together**, so this log does **not** claim how much each contributed. The user
made the call that fixing it mattered more than attributing it, which is a reasonable call — but it
means if the jitter returns, you cannot assume either change is load-bearing on its own. The
single-variable A/B (toggle `Node5_x` 26 ↔ 20.5 live via the GUI on one firmware build) is still
available if that question ever becomes worth answering.

## What was ruled out (measured, not assumed)

- **Loop rate as a spline-vs-PJMC differentiator** — identical per segment (262 / 261 / 242 Hz).
- **Torque dose** — per-step impulse 1.87 vs 1.83 Nm·s, peak −11.9 vs −12.0, width 233 vs 224 ms
  (right leg). On the **left**, PJMC was actually the *stronger* controller (−15.3 vs −12.1). An
  earlier read of "spline delivers ~2× the impulse" came from a single unrepresentative window and was
  wrong; the all-steps numbers killed it.
- **Whole-segment HF content** of command/measured torque — 1.70/1.87 vs 1.84, i.e. equal. The
  difference only appears when the data is **binned by gait phase**.
- **Missed ground strikes** — right toe FSR dropped strikes in Spline #1 (`prev_step` 2745/2943 ms,
  `Expected_step_ms` up to **2349** vs ~1250 nominal, stretching the gait clock ~2×). Real, and an
  aggravator for a clock-driven controller — but Spline #2 was clean (0% miss) and still felt jittery,
  so not the mechanism. See `Modification log with claude/Heel-FSR-Disable.md` and the toe-FSR
  asymmetry notes.

## Known remaining issues (NOT fixed here)

1. **Spline has no stance gate.** It commands torque with the foot airborne in **13.0% / 8.1%** of
   pulse samples (right leg); PJMC is 0.0% *by construction* — its feed-forward lives inside
   `if (_side_data->toe_stance)`. This is a code change and was left alone.
2. **The D term is dead below ~458 Hz.** `_pid` gates on `dt <= 2200 µs`. Any future work that slows
   the loop silently disables damping for every controller. Either protect the loop budget or relax
   the gate.
3. **SD logging is expensive.** Re-enabling it reintroduces the regression in section A. If logs are
   needed, budget for it (bigger ring buffer, cheaper flush) rather than assuming it is free.

## Method notes — traps that cost time here

- **GUI "Measured Torque" is `filtered_torque_reading`** (`uart_commands.h:411`), which is
  alpha-filtered for Spline but raw for PJMC. Before fix B, **spline-vs-PJMC comparisons in GUI logs
  were apples-to-oranges** and made spline look artificially smooth. Only same-controller comparisons
  were valid. Fix B incidentally removes this trap.
- **Jitter drifts upward within a trial**, crossing controller boundaries without stepping (one trial
  went 0.66 → 2.69 over 126 s). Never attribute to a controller without controlling for time-in-trial.
- **`Velocity_rad_s` in the SD log is stale** — it changes in only 3.5% of PJMC samples. A spectrum
  computed from it is an artifact of when the value happened to update. Do not use it.
- **A second-difference metric is blind to the interesting band.** `d2` weights ~26 Hz heavily and
  ~6 Hz barely, which is why whole-segment metrics showed parity while the controllers behaved
  differently. Bin by gait phase.
- **Log rate ceiling.** `sdLogDecimation = 5` on a ~262 Hz loop is ~52 Hz effective, **Nyquist 26 Hz**.
  Nothing above 26 Hz is observable. If a future question needs that band, drop decimation to 2
  (~131 Hz) — it is read at boot, no reflash.

## GUI parameter editing — how it actually works

Recorded because this log's author got it wrong mid-session and asserted the opposite:

- BLE `'f'` (`ble_names::update_param`, 4 fields) → `ble_handlers::update_param` →
  `UART_command_names::update_controller_param` (**singular**) → Teensy writes
  `j_data->controller.parameters[request.param_index] = request.value`. **Individual parameters can be
  set to arbitrary float values from the GUI.**
- Do **not** confuse this with `update_controller_params` (**plural**), a legacy path whose third field
  is a CSV *row selector* (`line_to_read = header_size + set_num`). That one cannot set values.
- `Node5_x` = **param index 8**, bounds `0.0–100.0`, `integer_only = false` — so `20.5` validates.
- **Gotcha:** `update_status` calls `exo_data->set_default_parameters()` on `trial_on`, and
  `update_controller_param` calls it whenever `controller_id` changes. **Both reload every parameter
  from the SD card**, wiping GUI-set values. Set params *after* the trial is running, and persist
  anything permanent to the CSV.
- **Guard:** `_spline_interpolate` returns `0.0f` if any `x[i] <= x[i-1]`. A typo making `Node5_x`
  ≤ `Node4_x` silently produces **zero torque for the whole trial**, not an error.

## Files changed

| File | Change | Persisted? |
|---|---|---|
| `ExoCode/src/Controller.cpp` | EWMA alpha `0.5f` → `1.0f` in `Spline::calc_motor_cmd`, with rationale comment | working tree, **uncommitted** — needs flash |
| `SDCard/config.ini` | `sdLogEnabled = 1` → `0` | working tree; **must also be on the physical card** |
| `SDCard/ankleControllers/spline.csv` | `Node5_x` `26` → `20.5` | working tree; **must also be on the physical card** |

**Persistence warning:** the working configuration currently depends on two SD-card files. The
`Node5_x` value used during the successful trial was set at runtime via the GUI; unless the physical
card's `spline.csv` also says `20.5`, the next `trial_on` reloads `26` and the overshoot returns.
Same for `sdLogEnabled`.

## How to validate on-device

1. Confirm the physical card matches the repo for both `config.ini` and `spline.csv`.
2. Start a trial, run spline, and confirm `Desired Torque` never exceeds ~`+0.01` (it was `+0.87`
   before — this is the quickest visual confirmation the node fix is live).
3. Baselines to compare against, quiet/swing HF residual, PJMC: old branch **0.903 / 0.719**, new
   branch logger-off **0.991 / 0.696**.

## If it bugs out — where to look

- **Jitter returns after re-enabling SD logging:** expected — that is section A. Check `ran/s` in
  `debug_log.txt`; below ~458 Hz the D term is gated off.
- **Overshoot is back (`Desired Torque` max ≈ +0.87):** the card's `spline.csv` still has
  `Node5_x = 26`, or a `trial_on` / controller-change reloaded defaults over a GUI-set value.
- **Spline outputs zero torque entirely:** node x-values are no longer strictly increasing —
  `_spline_interpolate` returns 0 with no error.
- **Spline feels smooth in swing but kicks at toe-off:** that is the unfixed stance gate (remaining
  issue #1), not the overshoot.

## Data provenance

Numbers above come from SD log `Test results/Motor logs/0009/` and GUI logs
`python_gui/saved_data/trial_20260723_{135550,143759,150211,150335}.csv`. **Log 0009 was untracked and
was deleted by a branch checkout mid-session**; it was later restored from the physical card. The GUI
logs are the durable record.
