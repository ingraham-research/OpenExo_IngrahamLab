# SplineAlt — a shape-parameterised sibling of the Spline controller

**Date:** 2026-08-27
**Scope:** Teensy 4.1 firmware (`ExoCode/src/Controller.{h,cpp}`, `ControllerData.{h,cpp}`,
`Joint.{h,cpp}`, `ParseIni.h`, `ParamsFromSD.h`) and the SD card
(`SDCard/ankleControllers/splineAlt.csv`, `SDCard/config.ini`). Ankle only.
**Status:** Implemented on branch `add_new_spline_parameters`, **uncommitted at time of writing**.
**Host-verified only — never compiled for Teensy, never flashed, never run on hardware.** The node
builder and the PCHIP interpolator were extracted from `Controller.cpp` verbatim, compiled with
`g++ -Wall -Wextra`, and checked numerically against `scipy.interpolate.PchipInterpolator`;
`calc_motor_cmd()` was reviewed but not executed.
**Related:** `Spline-Jitter-Diagnosis.md`, `Spline-Jitter-Round-2-SD-Logging-Regression.md`,
`Jitter-Round-3-Both-Branches-PJMC-PID.md`, `BLE-Handshake-Controller-List-Loss.md`,
`Remote-Control-UDP.md`.

---

## Why

The `Spline` controller is described by twelve explicit `(percent gait, torque)` node pairs. Retuning
it means editing 24 numbers, which is awkward from the GUI's Update Controller box and worse from a
program driving the UDP remote. The goal was to be able to say "peak plantarflexion at 52 % instead
of 49.7 %" or "run everything at 70 % magnitude" as a single parameter write.

The first plan was to add shape parameters *to* `Spline`. That was abandoned because the spline CSV
is already exactly at the BLE handshake's column ceiling (see
[The column ceiling](#the-column-ceiling-why-shape-params-were-not-added-to-spline) below). A separate
controller with its own, much smaller parameter list sidesteps the limit entirely.

## What it is

`SplineAlt` (ankle controller id **13**, name **`splineAlt`**) produces its curve from **17
parameters** instead of 30. Its command path — the ±15 Nm feed-forward clamp, the `torque_alpha = 1.0`
filter, the near-zero gain scheduler, the uncalibrated-sensor guard, the PID — is a **deliberate,
line-for-line copy of `Spline::calc_motor_cmd()`**, so the two controllers should feel identical when
given the same curve.

```
 0  PlantarNm    peak plantarflexion magnitude, Nm (applied NEGATIVE)
 1  DorsiNm      peak dorsiflexion magnitude, Nm (applied POSITIVE)
 2  PlantarPk    percent gait at which plantar torque first reaches peak
 3  DorsiPk      percent gait at which dorsi torque first reaches peak
 4  PlantRise    duration 0 -> peak
 5  PlantDwel    duration held AT peak (0 = no plateau)
 6  PlantFall    duration peak -> 0
 7  DorsiRise / 8 DorsiDwel / 9 DorsiFall
10  TorqScale    0-100 %, applied to both lobes
11  1=sim%gt  12 1=%gait  13 PID Flag  14 P Gain  15 I Gain  16 D Gain
```

Torque magnitudes are entered **positive**; the sign is applied internally (plantarflexion negative)
to match the convention in `SDCard/ankleControllers/spline.csv`.

## Node construction

Each lobe contributes up to four nodes:

```
(peak - rise, 0)   (peak, amplitude)   (peak + dwell, amplitude)   (peak + dwell + fall, 0)
```

Nodes from both lobes are then reduced **modulo 100**, sorted by x, and **extended periodically by two
nodes on each side**. Two rather than one, so every real node gets a genuine *interior* tangent
computed from its true periodic neighbours instead of the endpoint approximation from
`_pchip_edge_tangent`.

Three construction rules, each closing a silent-failure trap:

| Rule | Why |
|---|---|
| Magnitude exactly 0 → the lobe emits **no nodes at all** | How you disable a direction. Its timing parameters then cannot collide with the other lobe's. |
| Dwell 0 → **collapse to 3 nodes** | Zero dwell is the normal default. Emitting the duplicate peak node would trip the monotonicity guard and zero the *entire* profile. |
| Lobes emitted **sorted by peak time** | Dorsiflexion peaking before plantarflexion is expressible rather than fatal. |

### The deliberate asymmetry — dwell 0 is fine, rise/fall 0 is not

The PCHIP guard rejects `x[i] <= x[i-1]`, so **equal** x values are as fatal as out-of-order ones, and
the failure is all-or-nothing: the whole profile goes to zero, not just the offending segment.

Zero dwell is repaired (collapse to 3 nodes). Zero rise or zero fall is **not** repaired — it trips the
guard and the controller commands zero torque. This is intentional, and the reason is worth keeping:

> The obvious "fix" would be to drop the zero-valued boundary node when rise is 0. That leaves the
> **peak** node as the array endpoint, and the interpolator holds `y[0]` for everything below `x[0]`.
> A zero rise time would therefore apply **full peak torque from 0 % gait onward**. Silence is the
> safe failure here; full torque is not.

This rationale is duplicated in the comment block above `SplineAlt::_build_nodes` so it survives
independently of this document.

### Wrap-around

The periodic extension is what lets a lobe straddle the end of the gait cycle. The **current ankle
profile needs this**: after the 2026-08-26 edit to `spline.csv`, the dorsiflexion lobe peaks at
~94.7 % and decays through 0 % into the next stride (`spline.csv` has `y = 2` at both x = 0 and
x = 100).

A side effect worth knowing: `Spline` clamps at its end nodes, which leaves a **slope discontinuity at
the 100 %→0 % stride boundary**. `SplineAlt` is C¹ continuous across the seam. Measured on the default
profile: 99.5 % → +2.575, 99.9 % → +2.275, 0.0 % → +2.201, 0.5 % → +1.832 Nm.

## The copied-bugs problem

`SplineAlt` contains a **second copy** of the Fritsch–Carlson PCHIP math and of the entire command
path. This was a deliberate choice — sharing the code would have meant editing `Spline`, which is the
controller currently being tuned on this branch, and the jitter history (`Spline-Jitter-*.md`) makes
any behavioural change there expensive to re-validate.

The cost is that **the two will silently diverge**. Nothing in the build warns about it. Both files
carry banner comments saying so: the top of the `SplineAlt` block in `Controller.cpp`, and the class
comment on `Spline` in `Controller.h`.

Known warts copied on purpose, all inherited from `Spline`:

- Hard-coded `KP_ZERO = 3 / KI_ZERO = 0 / KD_ZERO = 0.001` that cannot be reached from the GUI or the
  SD card (mirrors `zeroTorque.csv` and PJMC's `kp_zero`/`ki_zero`/`kd_zero`).
- `torque_alpha` hard-coded to `1.0f` rather than being a real parameter.
- Both carry their original TODOs.

**If you fix anything in `Spline::calc_motor_cmd()`, fix it here too, and vice versa.**

## Verification

The three new functions (`_build_nodes`, `_pchip_interpolate`, `_pchip_edge_tangent`) were extracted
from `Controller.cpp` **by brace-matching on the real source text**, wrapped in a minimal shim with
the index constants pulled from `ControllerData.h`, compiled with `g++ -std=c++14 -Wall -Wextra -O2`,
and run. This tests the code that will be flashed, not a hand-written copy of it.

**Compiles clean.** Agreement with `scipy.interpolate.PchipInterpolator`:

| | max deviation |
|---|---|
| `SplineAlt` C++ (float32), all cases | **3.9e-06** |
| Same math in float64 Python | 1.4e-14 |
| **Existing `Spline::_pchip_interpolate` on the 12-node profile** | **5.3e-15** |

That last row is an independent result worth recording: **the existing Spline PCHIP is scipy-exact.**
The interpolator has never been the problem in the jitter investigations.

Behaviours confirmed against the compiled source:

| Case | Result |
|---|---|
| Default CSV (dorsi lobe wraps past 100) | 11 nodes, −15.000 / +5.000 |
| `TorqScale = 50` | 11 nodes, −7.500 / +2.500 |
| `TorqScale = 0` | 0 nodes → commands zero (usable transparency mode, see below) |
| `DorsiNm = 0` | 8 nodes, dorsi lobe absent |
| Both lobes with dwell | 12 nodes (the maximum) |
| Plantar lobe wraps past 100 | 8 nodes, correct |
| Dorsi peak *before* plantar peak | 10 nodes, correct |
| True node collision (plantar ends 70, dorsi starts 70) | 0 nodes → commands zero |
| `PlantRise = 0` | 0 nodes → commands zero |
| `PlantDwel = 0` | 7 nodes — does **not** zero |

`TorqScale = 0` is a free transparency mode: every node collapses to zero, so the near-zero gain
scheduler holds the tuned `3 / 0 / 0.001` gains for the whole stride.

## Default CSV

`SDCard/ankleControllers/splineAlt.csv` was fitted to the **updated** (2026-08-26) `spline.csv` by
differential evolution over the six timing parameters, with the magnitudes pinned to the real profile's
−15 / +5:

```
PlantarNm 15   PlantarPk 49.7   PlantRise 25.3   PlantDwel 0.5   PlantFall 16.2
DorsiNm    5   DorsiPk   94.6   DorsiRise 10.1   DorsiDwel 0     DorsiFall 10
TorqScale 100  sim 0  %gait 1  PID 1  P 3  I 0  D 0.01
```

**RMS 0.44 Nm, max 1.9 Nm.** The dorsi lobe matches to ~0.07 Nm and the plantar peak and fall to
~0.2 Nm. The error concentrates at **30–35 % gait**, where the real profile holds near zero and then
drops steeply — see the limitation below.

## Known limitations

1. **A lobe cannot hold flat and then drop steeply.** With 4 nodes it is rise → plateau → fall; there
   is no way to shape the *curvature* of the rise. The real profile's knee at ~30 % (near 0 until 30,
   then a fast descent to −15) is the single worst-fitting region, at 1.9 Nm. Adding a shoulder node
   per lobe would fix it and is the obvious next lever if the feel is wrong.
2. **Interleaved lobes do not silent-zero.** Only an *exact* x collision does. Lobes overlapping in
   time but with distinct node x's (plantar 30–70, dorsi 35–75) build a valid but strange curve. Not
   a bug — just not caught.
3. **Magnitudes above 15 Nm are silently clamped.** The bounds table allows ±50 (deliberately loose so
   the hard-coded table need not be reflashed), but `SplineAlt` keeps `Spline`'s ±15 Nm feed-forward
   clamp. The separate joint-level limit is `MAX_JOINT_TORQUE_NM = 25.0f` in `Config.h`, enforced in
   `Motor.cpp` at the motor shaft (`25 / gearing`) with a rate-limited Serial report.
4. **Ankle only.** Not wired into hip, knee, elbow or arm.

---

## The column ceiling — why shape params were *not* added to Spline

Recorded here because it is not obvious from the code and it constrains any future parameter addition.

`ListCtrlParams.h` sets `MAX_COLUMNS = 34`, but **`PREFIX_COLS = 4`** (line 74 — the in-code comments
saying "3" are stale). Those four cells hold joint name, joint ID, controller name and controller
index, leaving **30 data columns**. `spline.csv` uses all 30 exactly. **There is no spare column.**

Overflow is silent: at the 31st field, `colIndex` reaches 34 and `readAndParseFifthRow` simply
`break`s (`ListCtrlParams.cpp:568`). No error, no log. The parameter would work on the exo and be
invisible to the GUI dropdown and un-addressable by name from the UDP remote.

Two more limits in the same area:

- **`MAX_STRING_LENGTH = 10`**, and the cell copy is `strncpy(dst, src, maxLen - 1)`, so **names
  truncate at 9 characters.** The UDP remote addresses controllers and parameters by exactly that
  truncated string. This is why the controller is `splineAlt` (9 chars) and not `spline_alt` (10,
  which would arrive as `spline_al`), and why every name in `splineAlt.csv` is ≤ 9 characters. It is
  also the mechanism behind the phantom joint in `BLE-Handshake-Controller-List-Loss.md`
  ("Dorsi Scaling [Nm]" → "Dorsi Sca").
- **The controller name comes from the CSV *filename***, via `retrieveJointAndController()` — not from
  the `ParseIni.h` key. `splineAlt.csv` is what makes the name `splineAlt`.

Cost of raising `MAX_COLUMNS`, if it is ever needed: about **3.9 KB of Teensy RAM per column**
(`stringArray` in DMAMEM plus `txBuffer_bulkStr`, which is a plain ~70 KB global in RAM1). Adding a
`static_assert` tying `num_parameter + PREFIX_COLS <= MAX_COLUMNS` would turn the silent truncation
into a compile error and is recommended.

### The Nano buffer landmine

`GetBulkChar.h:10` declares `const size_t MAX_MESSAGE_SIZE = 25000` under the comment *"Must be large
enough to hold the full controller payload sent by Teensy"* — but this is a **different constant with
the same name** as the Teensy's `MAX_MESSAGE_SIZE` in `ListCtrlParams.h` (70,501 at 34 columns).
Different files, mutually exclusive `#if` branches, never compared. **They do not match and never
have.**

Do not "fix" this by making the Nano follow the sender. The Nano 33 BLE (nRF52840) has 256 KB of RAM
and already holds two 25 KB copies of the payload — `rxBuffer_bulkStr` and `sanitized_payload`
(`ExoBLE.cpp:67`) — roughly 20 % of RAM, alongside the BLE stack. Two copies of the Teensy constant
would be ~141 KB and will not fit. If anything the Nano buffer could shrink: the real payload is
~3.5 KB.

### Handshake payload budget

Measured for the current ankle-only bilateral config: **~3,503 B over 40 rows → ~184 BLE
notifications at 19 B each → ~3.7 s**. Per-controller cost (both sides, names + values rows):

```
spline 722 B | spv2 584 | pjmc_plus 460 | PJMC 320 | zhangCollins 294
trec 288 | chirp 250 | step 242 | constantTorque 200 | zeroTorque 138
```

`splineAlt` adds ~560 B / ~30 notifications, and `ankle_controllers::Count` 13→14 raises
`MAX_SNAPSHOTS` 188→192 (+1.4 KB `stringArray`). Everything stays far inside the limits: **44 of 192
rows, 21 of 34 columns.**

**Dropping unused controllers works by deleting the CSV from the SD card.** `csvExists` is checked
against the compile-time map in `ParamsFromSD.h`, not the card, so a deleted file still enters the
loop, fails its `SD.open`, and is simply never added to the payload. Two caveats:

- `MAX_SNAPSHOTS` is derived from the enum `Count`s, so **Teensy RAM does not shrink** — only payload
  and handshake time.
- `set_controller_params` guards on `if (param_file)`. If a controller whose CSV was deleted is still
  *selectable* and someone selects it, the load is skipped silently and `parameters[]` **retains the
  previous controller's values**, reinterpreted under the new controller's indices. Delete only
  controllers that can never be selected, and remove them from the `ParseIni.h` name map too.

Given the ~20 % handshake row-loss rate documented in `BLE-Handshake-Controller-List-Loss.md`, cutting
the payload is worthwhile independently of this change.

---

## Files changed

| File | Change |
|---|---|
| `ExoCode/src/Controller.cpp` | `SplineAlt` constructor, `_build_nodes`, `calc_motor_cmd`, variable-`n` `_pchip_interpolate`, `_pchip_edge_tangent`, and the copied-bugs banner |
| `ExoCode/src/Controller.h` | `class SplineAlt`; sister-controller warning added to `Spline`'s class comment (**the only change to `Spline`**) |
| `ExoCode/src/ControllerData.h` | `namespace spline_alt`, 17 indices |
| `ExoCode/src/ControllerData.cpp` | `spline_alt_bounds[]`, `bounds_for_spline_alt`, two ankle switch cases |
| `ExoCode/src/Joint.h` / `Joint.cpp` | `_spline_alt` member, ctor init, ankle switch case |
| `ExoCode/src/ParseIni.h` | `spline_alt = 13`; `{"splineAlt", ...}` name map |
| `ExoCode/src/ParamsFromSD.h` | `ankleControllers/splineAlt.csv` |
| `SDCard/ankleControllers/splineAlt.csv` | **new** |
| `SDCard/config.ini` | controller list comment (also fixed the pre-existing `splin` typo) |

`Spline`'s executable code is **byte-identical** to before; only its class comment changed.

## Next steps

1. Compile for Teensy and flash — neither has been done.
2. Bench-validate before any walking trial. Nothing has been run on the motors.
3. If the curve feels wrong at 30–35 %, add a shoulder node per lobe (limitation 1). Columns are free:
   21 of 34 used.
4. Drop unused controllers from `ParseIni.h` and the SD card (planned separately).
