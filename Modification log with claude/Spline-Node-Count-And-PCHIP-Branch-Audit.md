# Spline branch audit: `adding_nodes_to_spline_new` vs `pchip_spline_12_nodes` vs `main_working_branch`

**Date:** 2026-08-19 · **Revised 2026-08-20** after `origin/pchip_spline_12_nodes` moved to
`f1489af`, which changes the single biggest finding — see §4.
**Scope:** read-only audit. No code was modified. Everything below was verified against
git objects (`git show <rev>:<path>`) and, where behaviour was in question, by porting the
firmware's interpolators to Python and running them on the real node values (and, for
PCHIP, diffing against `scipy.interpolate.PchipInterpolator`).

**What changed since the 2026-08-19 pass:**

| | 2026-08-19 | 2026-08-20 |
|---|---|---|
| `origin/pchip_spline_12_nodes` tip | `fc8f678` | **`f1489af`** ("Update Controller.cpp", +90/−1) |
| Does that branch use PCHIP? | No — declared, never defined or called | **Yes — defined, called, and mathematically exact** (§4) |
| `origin/adding_nodes_to_spline_new` | `7d9593a` | `7d9593a` — unchanged; your local copy is *still* 1 commit behind (§2) |
| New branch seen | — | `origin/test/pchip_spline_control` @ `8092330` — a strict *ancestor* of `pchip_spline_12_nodes`, i.e. its old tip. Nothing new in it; ignore it. |

---

## 0. TL;DR

| Question | Answer |
|---|---|
| Is `adding_nodes_to_spline_new` safe to merge? | **Yes — but merge `origin/adding_nodes_to_spline_new`, NOT your local branch.** Your local copy is 1 commit behind and contains a fatal index bug that the remote already fixed. |
| Can the spline support 12 nodes? | **Yes.** `pchip_spline_12_nodes` does the 12-node plumbing correctly and completely (indices, bounds table, `max_parameters`, `MAX_COLUMNS`). |
| Does `pchip_spline_12_nodes` actually use PCHIP? | **Yes, as of `f1489af` (2026-08-20).** This reverses the previous answer. It is defined in the `.cpp`, wired into `calc_motor_cmd`, stateless, fixed-array, and matches `scipy.PchipInterpolator` to **2.7e−14 Nm** over 300 random 12-node stress cases. It is also ~30 % *cheaper* than the natural cubic. §4. |
| Is the 12-node + PCHIP branch now mergeable? | **Yes.** The merge auto-resolves everything except `SDCard/ankleControllers/spline.csv`, and I confirmed the merged `calc_motor_cmd` keeps **all** of `main_working`'s spline work (gain scheduling, `ewma 1.0f`, torque-offset guard) *and* takes the 12-node PCHIP path. §6. |
| What will you feel on the hardware? | PCHIP does not overshoot, so peak assistance goes **−12.061 → −12.000 Nm** and the profile shifts by **0.14 Nm (best port) to 0.44 Nm (naive port)** out of 12. Real, small, and in the safe direction. §4 has the exact 12-node CSV to use. |
| Biggest thing both branches *still* get wrong | **`hip`/`arm1`/`arm2` `spline.csv` were never updated** — still 16 params on `f1489af`. Those three joints' spline controllers go silently dead (0 Nm). §3. |
| Cost of 12 nodes | **+8.1 KiB DTCM** (`txBuffer_bulkStr`) + 7.3 KiB DMAMEM, and **+8 BLE notifications** on the handshake you are currently debugging — ~20 % → ~21.7 % row-loss rate. §7a. |

Merge order I'd recommend: §6.

---

## 1. What the three branches actually are

All three fork from the same ancestor, `c475745` ("Merge pull request #2 from
ingraham-research/adding_logging_sdcard").

```
                    +-- main_working_branch  (d8d04e7)  5 nodes + all the 2026-07/08 safety work
c475745 ------------+-- origin/adding_nodes_to_spline_new (7d9593a)  8 nodes
                    +-- origin/test/pchip_spline_control  (8092330)  <- old tip, strict ancestor
                         `-- origin/pchip_spline_12_nodes (f1489af)  12 nodes + WORKING PCHIP
```

`origin/test/pchip_spline_control` is not a separate line of work — it is
`pchip_spline_12_nodes` as it stood at commit 9 of 14, before the 12-node plumbing and
before `f1489af`. `git log test/pchip_spline_control..pchip_spline_12_nodes` shows 7
commits ahead, `git log pchip_spline_12_nodes..test/pchip_spline_control` shows none.
Ignore it; merge `pchip_spline_12_nodes`.

Both feature branches are **tiny and surgical** — this is good news for merging:

```
base -> origin/adding_nodes_to_spline_new     3 files changed,   28 insertions(+),   16 deletions(-)
base -> origin/pchip_spline_12_nodes         10 files changed, 1198 insertions(+),   25 deletions(-)
```

Of the 12-node branch's 10 files, only **5 are in the firmware build** —
`Controller.cpp` (+111), `Controller.h` (+2), `ControllerData.cpp` (+26),
`ControllerData.h` (+30), `ListCtrlParams.h` (+1/−1) — plus the ankle CSV. The other 616
lines are the two `.md` guides and the two out-of-build reference files under
`Test results/`.

Neither branch touches the motor path, the BLE/UART path, the SD logger, or the GUI.
The huge diffstat you see from `git diff main_working_branch <branch>` is just the
2026-07/08 work on `main_working_branch` that the feature branches predate — a *merge*
does not revert any of it (verified, §6).

---

## 2. CRITICAL — your local `adding_nodes_to_spline_new` is broken; the remote is not

```
$ git log --oneline adding_nodes_to_spline_new..origin/adding_nodes_to_spline_new
7d9593a Fixed a bug where node8 indices are duplicates of node 7 and spline wont run
```

Local tip `72d8111` has, in `ExoCode/src/ControllerData.h`:

```cpp
const uint8_t node7_x_idx = 12;   // NEW
const uint8_t node7_y_idx = 13;   // NEW
const uint8_t node8_x_idx = 12;   // NEW   <-- should be 14
const uint8_t node8_y_idx = 13;   // NEW   <-- should be 15
```

### Why this is fatal, not cosmetic

`Spline::_spline_interpolate` opens with an all-or-nothing monotonicity guard:

```cpp
for (int i = 1; i < n; ++i)
{
    if (x[i] <= x[i - 1])
    {
        return 0.0f;
    }
}
```

With `node8_x_idx == node7_x_idx`, `x[7]` and `x[6]` read the *same* parameter, so
`x[7] <= x[6]` is **always** true regardless of what is on the SD card or what the GUI
sends. The function returns `0.0f` on every single call. `ff_setpoint` is 0, the GUI's
"Desired Torque" trace is 0, and the spline controller commands nothing, forever.

Confirmed numerically:

```
AS THE LOCAL BRANCH ACTUALLY BUILDS IT (node8=node7)   range=[0.00, 0.00]
as the CSV intends (indices fixed)                     range=[-16.13, 10.08]
```

**Action:** `git fetch` and fast-forward the local branch (it is a strict ancestor of the
remote, so this is a clean fast-forward — no local-only commits to preserve). Merge
`origin/adding_nodes_to_spline_new`. Your "confirmed to work" testing was almost
certainly against the fixed remote version; the local checkout is stale.

A secondary consequence of the same bug worth knowing about, because it explains what
the GUI would have shown: the GUI derives a parameter's firmware index from its *column
position* in the handshake names row (`ActiveTrialSettingsPage.py:544`,
`for param_idx, c in enumerate(range(4, len(data)))`). So on the buggy build, editing
"Node8_x" in the GUI writes `parameters[14]`, which nothing reads, while editing
"Node7_x" silently moves both node 7 and node 8.

---

## 3. HIGH — both branches leave `hip` / `arm1` / `arm2` spline.csv at 5 nodes

`controller_defs::spline` and the `Spline` class are **shared by ankle, hip, arm_1 and
arm_2**. Both branches updated only `SDCard/ankleControllers/spline.csv`. The other three
still declare 16 parameters:

```
$ git show origin/pchip_spline_12_nodes:SDCard/hipControllers/spline.csv | sed -n 2p
16,"parameter number, the number of parameters to read per line",...
```

`set_controller_params()` reads `param_num_in_file` values in file order and zero-fills
the rest up to `max_parameters`. So the hip loads its old 16 values into the *new* index
layout. On the 8-node firmware that gives:

```
x = [0, 25, 50, 75, 100, 1, 0, 0]        <- values 10..15 were sim_gait/use_pid/P/I/D
```

Not strictly increasing → guard trips → **hip spline outputs 0 Nm**. Verified:

```
hip, 8-node firmware, old 16-col CSV     range=[0.00, 0.00]
hip, 12-node firmware, old 16-col CSV    range=[0.00, 0.00]
```

Worse, it is silent *and* mislabelled: `sim_gait`, `use_percent_gait`, `use_pid`, `P`,
`I`, `D` all read 0 (PID off, stance-based x-axis), and the GUI will show the hip's
parameter #11 labelled "1=sim %gait" while the firmware treats index 10 as `node6_x`.

**Action:** all four `spline.csv` files must be regenerated together with the firmware
index layout, in the same commit. This is on top of the sign fix you already planned.

---

## 4. RESOLVED (2026-08-20) — `pchip_spline_12_nodes` now has a real, correct PCHIP

**This reverses the previous finding.** On 2026-08-20 12:32 PDT, Siena Villancio-Wolter
pushed `f1489af "Update Controller.cpp"` — 90 insertions, 1 deletion — which does the two
things the branch had never done:

```diff
@@ ExoCode/src/Controller.cpp:878  (inside Spline::calc_motor_cmd)
-    float torque_cmd = _spline_interpolate(x, y, percent_gait);
+    float torque_cmd = _pchip_interpolate(x, y, percent_gait);
```

...and defines `Spline::_pchip_interpolate` and `Spline::_pchip_edge_tangent` in the
`.cpp`, in the stateless fixed-array form the 500 Hz loop needs — no `std::vector`, no
heap, `float h[11] / secant[11] / m[12]` on the stack. It is a correct port of
`Test results/spline_pchip.hpp`, not a copy of it.

The prior audit's headline finding ("PCHIP is declared and never defined") is **no longer
true** and should be disregarded. `_spline_interpolate` is still defined but is now dead
code.

### The maths is exact — verified against scipy

I transcribed `f1489af`'s `_pchip_interpolate` and `_pchip_edge_tangent` line-for-line
into Python and diffed them against `scipy.interpolate.PchipInterpolator` on 20 001
sample points per case:

| Test case | max abs(firmware − scipy) |
|---|---|
| The branch's own shipped 12-node CSV | 5.3e−15 Nm |
| `main_working`'s pulse shape resampled to 12 nodes | 3.6e−15 Nm |
| Flat shoulder + steep drop (the ringing case) | 7.1e−15 Nm |
| Strictly monotone increasing | 1.4e−14 Nm |
| **300 random 12-node sets, x in [0,100], y in [−15,15]** | **2.7e−14 Nm** |

That is double-precision round-off. The implementation is Fritsch–Carlson with scipy's
non-centred end conditions, and it is right:

- interior harmonic-mean tangents with the `w1 = 2h[i]+h[i−1]`, `w2 = h[i]+2h[i−1]`
  weights — matches scipy exactly;
- the zero-tangent rule at sign changes and at flat segments (`m0 == 0 || m1 == 0 ||
  sign(m0) != sign(m1)` → `m[i] = 0`) — this is what kills the overshoot;
- `_pchip_edge_tangent` reproduces scipy's `_edge_case`, including the `3*m0` clamp;
- the end call is `_pchip_edge_tangent(h[n−2], h[n−3], secant[n−2], secant[n−3])`, which
  is scipy's `_edge_case(h[−1], h[−2], m[−1], m[−2])`. Correct, and easy to get wrong.

One deliberate-looking detail worth recording: the firmware uses `x >= 0.0f` for "sign",
so `sign(0)` is *positive*, whereas `np.sign(0)` is `0`. I worked the divergent case
(`secant[0] == 0`, i.e. a flat first segment — which is exactly what your CSV will have)
and it still returns 0 through the second branch. **No behavioural difference.**

### Guard and clamp behaviour are unchanged

`_pchip_interpolate` opens with the same strict-monotonicity guard and the same end
clamps as `_spline_interpolate`:

```cpp
for (int i = 1; i < n; ++i) { if (x[i] <= x[i-1]) return 0.0f; }
if (percent_gait <= x[0])    return y[0];
if (percent_gait >= x[n-1])  return y[n-1];
```

So it is a true drop-in: §9's silent-failure discussion still applies verbatim, and
behaviour outside the node window (hold `y[n−1]`) is identical to today.

### It is also *cheaper* than the natural cubic

Divide count per call at n = 12 — divides dominate on a Cortex-M7 FPU (~14 cycles for
`VDIV.F32`, vs ~3 for a multiply):

| Interpolator | divides/call | ≈ cycles | ≈ time @ 600 MHz | % of the 2 ms control period |
|---|---|---|---|---|
| Natural cubic (`_spline_interpolate`) | ~62 | ~870 | 1.45 µs | 0.07 % |
| **PCHIP (`_pchip_interpolate`)** | **~43** | **~600** | **1.00 µs** | **0.05 %** |

No timing concern. PCHIP replaces a tridiagonal solve with two straight passes.

### What PCHIP does to *your* profile

This is the number that should drive the decision. Baseline = what the exo delivers today:
`main_working_branch`'s 5 nodes `(0,0) (5,−8) (10,−12) (20,0) (20.5,0)` through the
natural cubic → peak **−12.061 Nm at 9.53 %gait**.

| How you port the CSV to 12 nodes | peak | max deviation from today | RMS dev | wrong-direction torque | as command (×7) |
|---|---|---|---|---|---|
| **B — resample today's curve at 12 points, PCHIP** | −12.000 | **0.142 Nm (1.2 %)** | 0.041 | 0.000 | 0.00 |
| C — same 12 x, natural cubic | −12.061 | 0.008 Nm | 0.002 | +0.010 | +0.07 |
| **D — naive port: the 5 nodes + 7 zeros out to 100 %, PCHIP** | −12.000 | **0.441 Nm (3.7 %)** | 0.255 | 0.000 | 0.00 |
| D — same nodes, natural cubic | −12.061 | 0.002 Nm | 0.001 | +0.023 | +0.16 |

Read that as: **switching to PCHIP changes your tuned profile by 0.14–0.44 Nm out of
12 Nm, depending on how you write the CSV.** PCHIP does not overshoot, so it will not
reach the natural cubic's −12.061 peak; it tops out at the node value, −12.000. That is a
0.5 % reduction in peak assistance and is the *correct* behaviour, but it is a real change
to a profile you tuned on hardware, and you should expect to feel for it.

**Option B is the one to use.** Sample your existing natural-cubic curve at 12 points
inside 0–20.5 %gait and use those as the nodes:

```
x: 0,   2,      4,      6,      8,       10,   12,      14,     16,     18,     20,  20.5
y: 0, -3.328, -6.510, -9.385, -11.477, -12.0, -10.446, -7.494, -4.120, -1.297,  0,   0
```

That reproduces today's delivered torque to **0.142 Nm max / 0.041 Nm RMS**, spends the
whole node budget where the assistance actually happens, and holds 0 from 20.5 → 100 %gait
exactly as the current firmware does. Option D also works and is safe (0.44 Nm), it just
tracks your tuning less closely and wastes 7 nodes on a region that is flat zero anyway.

### Why PCHIP still matters — it gates *shaping*, not merging

The above says PCHIP is roughly neutral for the profile you have today. Its value is in
what it lets you do next. Re-running the ringing analysis on your pulse shape, `p_gain = 6`:

The column that matters is **wrong-direction torque** — the maximum *positive* torque the
interpolator produces when every node is zero or negative. Because
`cmd = ff + p_gain * (ff − measured)`, a wiggle of Δ in the feed-forward shows up
**1 + p = 7× larger** in the actual command (same amplification as the `Side.cpp:332`
comment about int-quantised `percent_gait`).

| Node layout (all shaped like main_working's pulse) | wrong-dir, natural cubic | → as command (×7) | wrong-dir, PCHIP |
|---|---|---|---|
| 5 nodes — what ships today | 0.01 Nm | 0.07 Nm | 0.00 |
| 8 nodes, densified **along the same smooth curve** | 0.01 Nm | 0.07 Nm | 0.00 |
| 8 nodes, with a **flat zero shoulder** at the start | 0.51 Nm | **3.54 Nm** | 0.00 |
| 12 nodes, even 1.86 % spacing, same smooth curve | 0.00 Nm | 0.00 Nm | 0.00 |
| 12 nodes, **zero shoulders both ends** | 0.31 Nm | **2.19 Nm** | 0.00 |
| 12 nodes, **flat-top plateau** (sharp shoulders) | 0.64 Nm | **4.51 Nm** | 0.00 |

- **Adding nodes along a smooth curve costs nothing.** The natural cubic converges — 8 or
  12 nodes tracing your existing shape ring *less* than the current 5 do.
- **Ringing appears the moment you put a flat region next to a steep one** — a hold-at-zero
  shoulder, or a flat-topped plateau. C² continuity forces curvature across the junction,
  so the curve must undershoot on the far side. PCHIP's zero-tangent-at-flat rule is
  precisely the fix.
- That is *what extra nodes are for*. Nobody adds 7 nodes to redraw the same smooth arc;
  they add them to hold zero longer, square off the top, or put a knee in the ramp.

**Now that PCHIP is written and verified, this stops being a trade-off — you get the
shaping freedom for free by merging the branch as-is.**

### Resolution is not a concern at 12 nodes

`percent_gait` is a float bounded by `millis()`: 1 ms / ~1200 ms stride ≈ **0.083 %gait**.
12 nodes across a 20.5 %gait window is 1.86 %gait per segment ≈ **22 distinct setpoints
per segment**. No staircase, no re-run of the int-quantisation problem.

### Still on the branch, outside the build

- `Test results/spline_pchip.hpp` — the `std::vector` reference implementation `f1489af`
  was ported from. Now redundant; not compiled.
- `Test results/spline_pchip_visualize.py` — scipy cross-check.
- `Test results/Switching_spline_controller_to_pchip.md` (209 lines) and
  `Switching_spline_controller_to_12_nodes.md` (407 lines) — both accurate; the latter
  correctly identifies both hard limits the 12-node change crosses (§7).

---

## 5. What `main_working_branch` has that neither branch has

Merging *into* `main_working_branch` preserves all of this (I verified the merge result
trees, §6). Listing it because it is what you'd lose if you ever went the other way —
rebasing onto a feature branch, or cherry-picking the feature branch as the new base.

| Feature | Where | Present on feature branches? |
|---|---|---|
| `MAX_JOINT_TORQUE_NM 25.0f` final fault gate + non-finite rejection in `_CANMotor::send_data()` | `Config.h` | **No** |
| Spline gain scheduling near zero torque (KP_ZERO 3 / 0 / 0.001) | `Controller.cpp` | **No** |
| Spline torque filter `ewma(..., 1.0f)` (was 0.5f — the phase-lag fix) | `Controller.cpp` | **No** (still 0.5f) |
| Uncalibrated-torque-sensor guard (stops `torque_cmd + _pid()` returning 2× ff) | `Controller.cpp` | **No** |
| `_sd_ready()` mount-once (the AK60v3 controller-change freeze fix) | `ParamsFromSD.cpp` | **No** |
| `ERROR_MANAGER_ENABLED 0`, `END_TRIAL_CUTS_MOTOR_POWER 1` | `Config.h` | **No** |

Also worth knowing: `percent_gait` is already a `float` on **all** branches (the int
quantisation was fixed before `c475745`), so 12 tightly-spaced nodes will not hit a
resolution floor.

---

## 6. Merge mechanics — I re-simulated both merges against the new tip

Using `git merge-tree --write-tree` (read-only; nothing was written to the worktree):

```
main_working_branch <- origin/adding_nodes_to_spline_new   (7d9593a, unchanged)
  Auto-merging ExoCode/src/Controller.cpp                 OK
  CONFLICT (content): SDCard/ankleControllers/spline.csv   <- the only conflict

main_working_branch <- origin/pchip_spline_12_nodes        (f1489af, NEW)
  Auto-merging ExoCode/src/Controller.cpp                 OK
  CONFLICT (content): SDCard/ankleControllers/spline.csv   <- the only conflict
```

Both sides now edit the *same function* (`Spline::calc_motor_cmd`), so a clean textual
merge is not by itself proof of a sane result. I extracted the merged tree
(`e03f036`) and read the merged `calc_motor_cmd` in full. **It is correct.** The merge
produces, in one function:

| From `origin/pchip_spline_12_nodes` | From `main_working_branch` |
|---|---|
| `float x[12] {...}` / `float y[12] {...}` — all 12 node indices | `KP_ZERO 3.0f / KI_ZERO 0.0f / KD_ZERO 0.001f` gain scheduling, with the `ZERO_SETPOINT_BAND_NM 0.5f` / `ZERO_ERROR_BAND_NM 3.5f` bands |
| `float torque_cmd = _pchip_interpolate(x, y, percent_gait);` | `utils::ewma(..., 1.0f)` — the phase-lag fix (branch still has 0.5f) |
| | the `torque_offset_reading == 0` uncalibrated-sensor guard, and its full comment block |
| | the ±15 Nm `torque_cmd` clamp (present on both sides) |

Nothing from `main_working_branch` is lost, and nothing from the feature branch is
dropped. Git got this right because the two sides touched disjoint *line ranges* within
the function: the branch rewrote the node-array block at the top, `main_working` rewrote
the PID block at the bottom.

**Verify this yourself after merging** rather than trusting it — read
`Spline::calc_motor_cmd` and confirm you can see `_pchip_interpolate`, `KP_ZERO`, `1.0f`,
and `torque_offset_reading` all present. A merge that silently drops the gain scheduling
would reintroduce the swing-phase shaking you already diagnosed and fixed.

The single conflict is `spline.csv` — exactly the file you already know needs manual work.

### Recommended sequence

The PCHIP branch being complete changes the calculus: **the 8-node stop is now optional
rather than the obvious safe step.** Two viable routes:

**Route A — go straight to 12 + PCHIP (now my recommendation).** The thing that made this
a two-stage plan was that 12 nodes forced you to also write PCHIP yourself. That work is
done and verified. Going straight there skips an entire merge, an entire round of CSV
rewrites across four files, and an entire bench-validation cycle.

1. **Land the Nano/GUI handshake fix first.** §7a — 12 nodes adds ~8 BLE notifications to
   the handshake, pushing the row-loss rate from ~20 % to ~21.7 %. Merging spline changes
   into an unfixed handshake means every failure is ambiguous between the two.
2. Merge `origin/pchip_spline_12_nodes` into `main_working_branch`.
   - Resolve `SDCard/ankleControllers/spline.csv` by taking **`main_working_branch`'s side
     wholesale** and re-expressing it as 12 nodes. Do not hand-merge cell by cell — the
     incoming side is wrong on sign, timing axis, *and* the PID block (§11). Use the
     Option B node values in §4; keep `use_pid=1, P 6, I 0, D 0.03`.
   - Update `hip` / `arm1` / `arm2` `spline.csv` to 30 params **in the same commit** (§3),
     or those three joints go silently dead.
   - Confirm the merged `calc_motor_cmd` per the table above.
3. Bench-validate. Expect a ~0.5 % lower peak (−12.000 vs −12.061 Nm) and a profile within
   0.14 Nm of today's. If it feels different beyond that, the CSV is wrong, not the maths.

**Route B — 8 nodes first, as originally planned.** Still perfectly reasonable if you want
the smallest possible step, or if you want the node-count change validated on hardware
before the interpolator change lands on top of it.

1. **`git fetch`** and fast-forward `adding_nodes_to_spline_new` to `7d9593a`. Do not merge
   the local tip — it is still 1 commit behind and still contains the fatal index bug (§2).
2. Land the handshake fix.
3. Merge `origin/adding_nodes_to_spline_new`. 8 nodes crosses **no** other firmware limit
   (§7) — no `max_parameters` bump, no `MAX_COLUMNS` bump, no RAM growth. Resolve
   `spline.csv` as above (22 params), all four files together. Extend `spline_bounds[]`
   (§8).
4. Then merge `origin/pchip_spline_12_nodes` on top, redoing the CSV work at 30 params.

Route B costs you one extra merge and one extra full CSV rewrite across four files, and
its only real benefit is isolating "did more nodes break something?" from "did PCHIP
change the feel?". Given that §4 already answers the second question numerically, I would
take Route A.

---

## 7. Limits crossed by node count (why 8 is the free stop and 12 is not)

Parameter count = `2N + 6`.

| Limit | Value | Cap on N | Crossed by 8 nodes? | By 12? |
|---|---|---|---|---|
| `max_parameters` (`= spv2::num_parameter = 22`) | 22 | **N ≤ 8** | No — exactly fits (22) | **Yes** — needs 30 |
| `MAX_COLUMNS` 30, minus `PREFIX_COLS` 4 | 26 data cols | N ≤ 10 | No | **Yes** — needs 34 |

`origin/pchip_spline_12_nodes` handles both: `max_parameters = spline::num_parameter`
(30) and `MAX_COLUMNS = 34`. Both are correct. Note `readAndParseValuesRow` uses a
*3*-column prefix, not 4, so 34 covers the names row (4+30) and the values row (3+30).

**Costs of going to 12** — quantified precisely in **§7a**. In summary:

- `ControllerData::parameters[]` grows 22 → 30 floats **per joint per side** (every
  controller, not just spline). ~32 bytes × ~12 instances ≈ 400 bytes. Irrelevant.
- `MAX_COLUMNS` 30 → 34: **+8.08 KiB DTCM** and +7.34 KiB DMAMEM. The DTCM half is the
  one to check against the linker output.
- **Handshake payload:** +153 B on the spline rows, **+8 BLE notifications**, taking the
  known row-loss rate from ~20 % to ~21.7 %. This is why §6 says land the handshake fix
  first.

---

## 7a. The real cost of 12 nodes — RAM and handshake payload

Only the 12-node change pays this. 8 nodes costs nothing here (§7).

### Static RAM

`MAX_COLUMNS` goes 30 → 34, which scales two large static arrays. The relevant constant is
`MAX_SNAPSHOTS = 4 × Σ(controller Counts)`. From `ParseIni.h` those enums start at `= 1`,
so `Count` is last-value + 1: hip 10, knee 7, ankle 13, elbow 7, arm1 5, arm2 5 → **Σ = 47,
MAX_SNAPSHOTS = 188**.

| Array | Placement | `MAX_COLUMNS` 30 | 34 | Δ |
|---|---|---|---|---|
| `txBuffer_bulkStr[MAX_MESSAGE_SIZE]` | **DTCM (RAM1)** — no `DMAMEM` | 62 229 B | 70 501 B | **+8 272 B (+8.08 KiB)** |
| `stringArray[SNAP][COLS][LEN]` | `DMAMEM` → OCRAM (RAM2) | 56 400 B | 63 920 B | +7 520 B (+7.34 KiB) |
| | | 118 629 B | 134 421 B | +15 792 B |

The one that matters is `txBuffer_bulkStr`: it is **not** `DMAMEM`, so it lands in DTCM,
which on a Teensy 4.1 is the tight 512 KiB region shared with the stack and every other
global. It is already 61 KiB and would become 69 KiB.

**I could not verify this fits** — that needs a compile, which was out of scope. Check the
linker output for RAM1 headroom before flashing. If it is tight, marking
`txBuffer_bulkStr` as `DMAMEM` would move 69 KiB off DTCM, but that changes access latency
on the BLE tx path and should be measured, not assumed.

### BLE handshake payload

You are on `fix_nano_GUI_handshaking` right now, so this interaction matters. I rebuilt
the handshake string from the actual SD-card CSVs, reproducing the firmware's
`create_csv_message()` rules (4 prefix cells on the names row, 3 on the values row, cells
truncated to `MAX_STRING_LENGTH − 1 = 9` chars):

| | bytes |
|---|---|
| Full ankle controller set, one side, today | 1 599 B |
| ...with the spline row at 12 nodes | 1 752 B |
| **spline rows alone: 5-node → 12-node** | **201 B → 354 B (+153 B)** |

At the proven 19-byte loss quantum (one BLE notification — see the
`ble-handshake-row-loss` finding):

- one ankle side: **85 → 93 notifications (+8)**
- the spline row alone accounts for **+9** notifications
- if spline is enabled on hip + arm1 + arm2 as well: **+36** total

Calibrating against the observed ~20 % whole-handshake failure rate gives a
per-notification drop probability of ~0.262 %. At 93 notifications instead of 85:

> **P(clean handshake) 80.0 % → 78.3 %, i.e. failure rate ~20 % → ~21.7 %.**

Not catastrophic, but it moves the wrong way, and it is a 9.6 % payload increase on the
exact mechanism you are currently debugging. **Land the handshake fix first** — otherwise
every subsequent handshake failure is ambiguous between "the old bug" and "the payload got
bigger."

### Column budget is at exactly zero headroom

The 12-node names row is 30 names + `PREFIX_COLS` 4 = **34 columns = `MAX_COLUMNS` exactly.**
I traced `readAndParseFifthRow`: the parse is bounded (`if (colIndex >= maxCols) break;`),
`totalColsWritten` comes out to exactly 34, and nothing overflows. The values row uses
`kValuePrefixCols = 3`, so 33 of 34. **Correct, but with zero slack** — the next parameter
anyone adds to the spline controller silently truncates the last column of the names row
rather than erroring. Worth a comment at `ListCtrlParams.h:38` when you merge.

---

## 8. MEDIUM — `spline_bounds[]` not extended on `adding_nodes_to_spline_new`

`ControllerData.cpp` declares:

```cpp
const ParameterBoundConfig spline_bounds[controller_defs::spline::num_parameter] = { ...16 entries... };
```

On that branch `num_parameter` is now 22, so this is a 22-element array with 16
initialisers. It compiles (aggregate init; the last 6 zero-fill) and is **inert today**,
because every entry has `enabled = false` and `read_parameter_bound()` early-returns on
`!bound.enabled`. So no current misbehaviour.

But the mapping is now wrong and armed. If anyone ever flips `enabled` to `true`:
- index 10 (`node6_x`) would be clamped to `[0, 1]` **integer-only** (the old `sim_gait` row)
- index 11 (`node6_y`) → `[0, 1]` integer-only
- index 12 (`node7_x`) → `[0, 1]` integer-only
- indices 13–15 (`node7_y`, `node8_x`, `node8_y`) → `[0, 10000]`, i.e. no negative torque
- indices 16–21 (`sim_gait` … `d_gain`) → zero-filled → `[0, 0]`, i.e. **PID and its gains
  become unsettable from the GUI**

`origin/pchip_spline_12_nodes` does this correctly (all 30 rows, right order). Cheap to
fix on the 8-node merge; recommended.

---

## 9. MEDIUM — the monotonicity guard is a silent-failure design, and 12 nodes makes it worse

`_spline_interpolate` returns `0.0f` if *any* `x[i] <= x[i-1]`. **`_pchip_interpolate`
copies this guard verbatim**, so switching interpolators does not fix it — if anything it
is now the single most likely way to get a silent 0 Nm on the merged firmware, since it is
the only remaining silent-failure path in the spline. There is no error, no log line, no
GUI indication — assistance just stops. Two ways this bites harder with more nodes:

- **N nodes = N−1 orderings to keep valid.** 5 nodes → 4; 12 nodes → 11.
- **GUI edits are one parameter at a time** (`ble_names::update_param` carries a single
  joint/controller/index/value). So moving a node past its neighbour necessarily passes
  through an invalid intermediate state, during which torque silently drops to zero and
  then silently comes back. With 5 widely-spaced nodes that is rare; with 12 nodes spaced
  5 % apart it is easy to hit.
- **There is no "use only the first K nodes" mechanism.** With 12-node firmware you must
  supply 12 strictly-increasing x values even if you only want 6 meaningful ones. The
  usual dodge (park spares at the end) requires them to still be strictly increasing.

Not a blocker, but if you go to 12 nodes it is worth either (a) surfacing the guard trip
(a rate-limited log line or a status bit), or (b) making the guard degrade to "use the
first K strictly-increasing nodes" instead of returning 0.

---

## 10. LOW — file placement

`origin/pchip_spline_12_nodes` puts a firmware header (`spline_pchip.hpp`), a Python
tool, and two how-to docs under **`Test results/`**. Per repo convention these belong in
`Modification log with claude/` (write-ups) or `Useful guides by us/` (how-tos), and
`ADDING_SPLINE_NODES.md` already lives in the latter and covers the same ground for 5→7.
Worth relocating on merge so the three spline guides sit together.

---

## 11. The ankle CSVs — `main_working_branch` is the reference, and it differs by more than sign

**Resolved (confirmed by ZJ, 2026-08-19): `SDCard/ankleControllers/spline.csv` on
`main_working_branch` is correct, and is the only correct one. Every other branch's spline
CSV is to be rewritten to match it after the merge.** Recording the actual deltas here so
that rewrite is mechanical rather than from memory.

| Branch | `use_percent_gait` | Node x range | Shape | PID |
|---|---|---|---|---|
| **`main_working_branch` — REFERENCE** | 1 | **0 → 20.5** | peak **−12 Nm at 10 %** gait, zero after 20 % | **on**, P 6 / I 0 / D 0.03 |
| `adding_nodes` (8) | 1 | 0 → 100 | −15 at 48 %, then **+10 at 80 %** (swing) | off, 0/0/0 |
| `pchip_12` (12) | 1 | 0 → 100 | −12 at 50 %, zero elsewhere | off, 0/0/0 |

So the branch CSVs differ from the reference in **four** ways, not one:

1. **Sign** — the known issue.
2. **Timing axis** — the branches spread nodes over the full 0–100 %gait cycle; the
   reference confines the whole profile to 0–20.5 %. Any ported node set must be
   re-scaled into that window, not just sign-flipped. With 12 nodes that means ~1.86 %gait
   spacing (fine for resolution — see §4).
3. **PID block** — the reference runs closed loop (`use_pid = 1`, P 6, D 0.03); both
   branches ship `0,0,0,0`. Carrying a branch CSV forward verbatim silently drops the exo
   to open-loop feed-forward.
4. **Swing-phase content** — the 8-node set's `+10 Nm at 80 %gait` is a dorsiflexion pulse
   during swing, which the reference does not have at all. Given the swing-phase
   Bowden/chain rattle in `Jitter-Round-3-Both-Branches-PJMC-PID.md`, that is not
   something to inherit by accident.

Note also that the reference's last two nodes are `(20, 0)` and `(20.5, 0)` — a 0.5 %gait
segment. When extending to 8 or 12 nodes, keep an eye on that trailing pair: it is the
tightest knot spacing in the profile and every added node has to stay strictly to the left
of it (§9).

For reference, `Using_spline_controller.md`'s literature shape (rise 25 %, peak 48 %, zero
by 63 %) is a *different* profile from the one in use — worth knowing when reading that
guide, since its example CSV line no longer matches what ships.

---

## 12. What I could not verify

- **Compilation.** No Teensy toolchain was invoked. Every claim above is source-level.
  The merge-result trees are consistent, but a `Sketch uses …` compile is still needed —
  **particularly for the +8.08 KiB of DTCM in §7a**, which is the one finding here that
  could turn out to be a hard blocker rather than a cost.
- **Bench behaviour.** No hardware was touched and no motors were moved. In particular the
  0.14–0.44 Nm profile shift in §4 is a computed difference between two interpolants, not
  a measured difference in delivered torque — the transmission, the Bowden slack, and the
  PID loop all sit between `torque_cmd` and the ankle.
- **Float vs double.** My scipy cross-check ran in double precision; the firmware is
  `float`. The 2.7e−14 Nm agreement proves the *algorithm* matches, not that single
  precision holds to that tolerance. Expect ~1e−6 Nm from float rounding — irrelevant next
  to a 12 Nm command, and far below the 0.083 %gait input quantisation, but stated so the
  number is not over-read.
- **Which SD card is physically in the exo.** The §2 zero-torque finding is
  CSV-independent (the two indices alias each other regardless of file contents), but the
  §3 hip/arm finding assumes the SD card matches the repo's `SDCard/` tree.
- **Whether `f1489af` was ever run on hardware.** It landed 2026-08-20 12:32 PDT with the
  message "Update Controller.cpp" and no accompanying test notes. The maths is right; that
  is not the same as saying someone has walked in it.
