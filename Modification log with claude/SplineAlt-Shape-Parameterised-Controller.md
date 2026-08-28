# SplineAlt — a shape-parameterised sibling of the Spline controller

**Date:** 2026-08-27
**Scope:** Teensy 4.1 firmware (`ExoCode/src/Controller.{h,cpp}`, `ControllerData.{h,cpp}`,
`Joint.{h,cpp}`, `ParseIni.h`, `ParamsFromSD.h`) and the SD card
(`SDCard/ankleControllers/splineAlt.csv`, `SDCard/config.ini`). Ankle only.
**Also covers:** unwiring the **TREC** and **SPV2** controllers from the ankle — see
[Removing TREC and SPV2](#removing-trec-and-spv2-from-the-ankle) at the end.
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
−15 / +5, then **re-solved over the integer grid** (see
[Node precision](#node-precision-cleanup) below):

```
PlantarNm 15   PlantarPk 50   PlantRise 26   PlantDwel 0   PlantFall 17
DorsiNm    5   DorsiPk   95   DorsiRise 11   DorsiDwel 0   DorsiFall  9
TorqScale 100  sim 0  %gait 1  PID 1  P 3  I 0  D 0.01
```

**RMS 0.45 Nm, max 2.0 Nm** (the non-integer fit was rms 0.4416 — the integer constraint costs
0.013 Nm, i.e. nothing). The dorsi lobe matches closely; the error concentrates at **30–35 % gait**,
where the real profile holds near zero and then drops steeply — see the limitation below.

Run through the **compiled firmware code**, these parameters build 10 nodes
`(4,0) (24,0) (50,−15) (67,0) (84,0) (95,5)` plus the four wrap copies, giving +2.09 Nm at 0 % gait,
−15.00 at 50 %, +5.00 at 95 %.

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

---

## Removing TREC and SPV2 from the ankle

**Date:** 2026-08-27, same branch, same uncommitted change set.

ZJ asked to drop `chirp`, `step`, `trec` and `spv2` as unused. After checking what each one is, the
scope was narrowed to **TREC and SPV2 only**:

| | Joints | What it is | Outcome |
|---|---|---|---|
| **TREC** | ankle only | Terrain Responsive Exoskeleton Controller, from Cuddeback's NAU thesis | **removed** |
| **SPV2** | ankle only | Header says *"STILL UNDER DEVELOPMENT"* | **removed** |
| **Chirp** | hip, knee, ankle, elbow | Sine sweep, *"used for hardware performance validation"* | **kept** |
| **Step** | hip, knee, ankle, elbow | Step response, *"used for hardware performance validation"* | **kept** |

Chirp and Step were kept because they are actuator-characterisation tools, and a chirp sweep is
precisely how you would measure the ankle's frequency response — the open question behind
`Jitter-Round-3-Both-Branches-PJMC-PID.md` and the mechanical-resonance finding that followed it.

### What was done — "unwire", not "delete"

The `TREC` and `SPV2` **classes still exist** in `Controller.h` / `Controller.cpp`, and their
`controller_defs::trec` / `controller_defs::spv2` namespaces and their `ControllerData` state fields
(`setpoint2use_spv2`, `wasStance_spv2`, …) remain, because the class bodies reference them. They are
simply **no longer instantiated or reachable**:

- `ParseIni.h` — enum entries and `{"TREC", …}` / `{"SPV2", …}` name-map entries removed
- `ParamsFromSD.h` — ankle CSV-path entries removed
- `ControllerData.cpp` — the two ankle switch cases in each of `get_parameter_length_for` and
  `get_parameter_bounds_for`, plus `trec_bounds[]`, `spv2_bounds[]`, `bounds_for_trec`,
  `bounds_for_spv2` (those four live in the **anonymous namespace** at line 8, so leaving them with
  no call site would produce unused-function/variable warnings)
- `Joint.h` / `Joint.cpp` — `_trec` and `_spv2` members, their ctor-init entries, and their switch cases
- `SDCard/ankleControllers/trec.csv` and `spv2.csv` deleted

Verified afterwards: no dangling `ankle_controllers::trec` / `::spv2` / `_trec` / `_spv2` references
anywhere, and `ControllerData.cpp` braces balance.

### Enum IDs were deliberately NOT renumbered

`trec = 6` and `spv2 = 10` are left as **gaps**, with a comment in `ParseIni.h` saying so. These
values are the controller IDs on the wire: they ship in the BLE handshake, and the Python UDP remote
can address a controller by number (`Python_GUI/examples/remote_console.py` documents spline as
id 12). Closing the gaps would silently repoint any saved script at a different controller.
`Count` is unaffected — it still follows `spline_alt = 13`, so it stays 14.

### Handshake result

```
before this session       ~3508 B   185 notifications   3.69 s
after splineAlt added     ~4068 B   214 notifications   4.28 s
after trec+spv2 removed   ~3267 B   172 notifications   3.44 s
                          --------------------------------------
net vs session start       -241 B   -13 notifications  -0.25 s
```

So the handshake ends up **smaller than before a whole new controller was added**. Current per-controller
costs (both sides, names + values):

```
spline 880 | splineAlt 478 | pjmc_plus 460 | PJMC 320 | zhangCollins 294
chirp 250 | step 242 | constantTorque 200 | zeroTorque 138
```

Note `spline.csv` now costs **880 B**, not the 722 B measured before 2026-08-26 — that edit
introduced long decimals (`4.94366`, `0.0451984`) which inflate its values row. It is now by far the
most expensive row in the handshake.

`MAX_SNAPSHOTS` does **not** shrink (it is derived from the enum `Count`s, and `Count` is unchanged),
so Teensy RAM is unaffected — only payload and handshake time.

---

## Node precision cleanup

**Date:** 2026-08-27, same change set.

The 2026-08-26 `spline.csv` edit stored values like `4.94366` and `0.0451984`, which made it by far
the most expensive handshake row (880 B). ZJ's point: that precision is *"WAY beyond the bandwidth of
the pulley system."*

**That argument is correct for x but does not transfer to y.** Timing precision below 1 % gait
(~12 ms at a 1.2 s stride, ~80 Hz) is far beyond what the Bowden/pulley transmission can track. The
y digits are an *amplitude* resolution question instead, and both the torque sensor and the motor
resolve well below 1 Nm. Measured deviation from the pre-rounding curve:

| | max dev | rms | values row |
|---|---|---|---|
| current (pre-rounding) | – | – | 344 B |
| x 2dp, y 2dp | 0.0072 Nm | 0.0022 | 270 B |
| **x 1dp, y 2dp — CHOSEN** | **0.0332 Nm** | **0.0127** | **250 B** |
| x 1dp, y 1dp | 0.0714 Nm | 0.0256 | 236 B |
| x integer, y 2dp | 0.4209 Nm | 0.1255 | 210 B |
| x integer, y 1dp | 0.399 Nm | 0.131 | 196 B |
| x, y both integer | 0.848 Nm | 0.193 | 180 B |

**The decisive figure: dropping y from 2 decimals to integer saves only ~40 bytes (~2
notifications) but costs 25× the deviation** — 0.848 vs 0.033 Nm. For scale,
`spline-node-count-branches` treats 0.14–0.44 Nm profile changes as worth analysing, and at
`p_gain = 3` a feed-forward shift reaches the command at roughly 4×.

Final: **x to 1 decimal, y to 2 decimals.** The timing digits — the ones the bandwidth argument
actually covers — are cut hard, while the torque values keep enough resolution that the profile is
indistinguishable from the one that was tuned (0.033 Nm peak deviation, 0.2 % of the −15 Nm peak).
Peak stays exactly −15.00 / +5.00, and the rounded x values remain strictly increasing
(`0, 4.9, 29.8, 33.8, 39.7, 51.2, 56.7, 59.8, 66.1, 84.7, 94.7, 100`) so there is no silent-zero risk.

```
0,2, 4.9,0, 29.8,0.05, 33.8,-6.11, 39.7,-9.84, 51.2,-15,
56.7,-9.5, 59.8,-5.36, 66.1,0, 84.7,0.09, 94.7,5, 100,2
```

`splineAlt.csv` **was** rounded to all-integer timings, because its parameters are shape knobs rather
than a hand-tuned curve — and re-solving the fit over the integer grid costs only 0.013 Nm rms. Note
the fitted `PlantDwel = 0.5` rounds to **0, not 1**: 0 fits better (rms 0.486 vs 0.554) and is an
explicitly supported path (the lobe collapses to 3 nodes).

### Handshake, final

```
session start             3508 B   185 notifications   3.69 s
after splineAlt added     4068 B   214 notifications   4.28 s
after trec + spv2 removed 3267 B   172 notifications   3.44 s
after precision cleanup   3147 B   166 notifications   3.31 s
                          --------------------------------------
net vs session start      -361 B   -19 notifications  -0.38 s
```

`spline.csv` drops 880 → 786 B; `splineAlt.csv` is 452 B. Net: a new controller was added and the
handshake still shrank by 10 %.

### If TREC or SPV2 is ever wanted back

Restore the five wiring points and the CSV. The classes, namespaces and bounds *values* are all still
in git history at this commit's parent; the bounds arrays would need to be re-added to
`ControllerData.cpp`.

---

## Next steps

1. Compile for Teensy and flash — neither has been done.
2. Bench-validate before any walking trial. Nothing has been run on the motors.
3. If the curve feels wrong at 30–35 %, add a shoulder node per lobe (limitation 1). Columns are free:
   21 of 34 used.
4. ~~Consider shrinking `spline.csv`'s stored precision.~~ **Done 2026-08-27** — see
   [Node precision cleanup](#node-precision-cleanup).
