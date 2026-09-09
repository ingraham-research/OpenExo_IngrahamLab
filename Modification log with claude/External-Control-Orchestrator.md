# External control code — an optimization backend driving the exo through the GUI

**Date:** 2026-09-08
**Scope:** New host-side package `Python_GUI/external_control/`. **No firmware changes, no GUI changes.**
**Status:** Written and host-tested against a stubbed link (62 checks). **Never run against a real GUI
or a real exo.**
**Design spec:** `Modification log with claude/specs/2026-09-08-experiment-orchestrator-design.md`
**Related:** `Remote-Control-UDP.md`, `SplineAlt-Shape-Parameterised-Controller.md`,
`2026-07-14-control-loop-and-transparency-backlog.md`

---

## What it is

The OpenExo counterpart of `main_control_code.py` on the hip exo. The difference is where the control
loop lives: there, that file owns the motors and runs splines at 200 Hz; here the Teensy owns all of
that, so this code is **supervisory** — it owns an optimization backend, computes machine actions, and
applies them by writing controller parameters through the GUI's UDP remote service, confirming each
write against the exo's own acknowledgement.

```
Python_GUI/external_control/
  main_external_control.py              mode menu, session setup, the supervisory loop
  Utilities/OpenExoLink_utilities.py    the UDP link: name->id resolution, confirmed writes, retries
  Utilities/ActionMap_utilities.py      machine action -> the splineAlt parameter writes that implement it
  PSEUDOCODE.md                         the whole design as pseudocode, for hand-typing
  README.md                             how to run it, what to set first
```

Max Shepherd's `GameTheory_1D_Backend` and `UDPReceiver` are **imported from the hip code base by full
path**, never copied, so the algorithm has one source of truth. Full path rather than `sys.path`
because both code bases have a folder called `Utilities` and putting both on the path makes
`import Utilities.x` ambiguous.

## The governing rule

**The orchestrator does no signal processing.** It never derives an observation from raw exo telemetry.
Exo-derived inputs would have to be computed on the Teensy and sent up; external instruments (treadmill,
metabolic cart) connect directly to this code on their own sockets, not through the GUI.

For the game-theory protocol this means **the exo is write-only** — the human action is treadmill speed
and nothing comes back from the exo except acknowledgements. That is what keeps firmware and GUI work
off the critical path.

## The machine action

`splineAlt`'s **`TorqScale`** (param index 10), a literal mirror of the hip's `new_torque_percentage`.
It is the right knob for two non-obvious reasons:

- It scales node amplitudes at construction (`Controller.cpp:1239-1246`), so it lands on `torque_cmd`
  **before the PID and before the ±15 Nm clamp**. Scaling *after* the PID would be silently cancelled —
  the loop servos measured torque back to the unscaled target — and only when `use_pid=1`, which is our
  configuration.
- `TorqScale = 0` is a real transparency mode (`Controller.cpp:1420`): every node collapses to zero and
  the gain scheduler holds the tuned zero-torque gains all stride. So the action range `[0, 100]` maps on
  with no dead zone, and `m = 0` is a safe action rather than an edge case.

## The five traps the code exists to handle

Everything non-obvious in `OpenExoLink` traces to one of these:

1. **Silence is the only failure signal.** A dropped BLE write produces no negative ack, and the GUI
   returns `ok` to `set_param` even with no exo connected — `ok` only means it queued a write.
2. **Acks carry no request id**, only `joint_id`/`controller_id`/`param_index`. So names are resolved to
   numbers on our side; without them a reply cannot be matched to its write.
3. **A fast ack gets swallowed.** `ExoRemote._command` drains stream frames while waiting for its own
   `ok`, so an ack arriving in that window lands in `last_ack()` and is never re-delivered by `stream()`.
   `_wait_for_ack` therefore checks `last_ack()` *first*, then falls back to the stream.
4. **Writing a parameter to a non-active controller switches the joint to it and loads its SD defaults.**
   `splineAlt.csv` ships `TorqScale = 95`, so `engage_controller_safely()` must write `TorqScale = 0`
   first or the leg begins assisting at 95 % the instant it is touched.
5. **`bilateral=True` acks twice and only the last is kept.** Every write is sent unilaterally instead —
   identical on the wire (`QtExoDeviceManager.py:744-750` expands bilateral into two BLE writes anyway),
   but unambiguous to match.

## The mode menu

Kept as close to the hip's as the hardware allows.

```
2  game theory backend      human action = treadmill speed (UDP 5004), machine action = torque percentage
1  everything else
     OP        run the profile shape already on the SD card
     plantar   plantarflexion only, you pick the peak timing (dorsi lobe disabled)
     dorsi     dorsiflexion only, you pick the peak timing (plantar lobe disabled)
     udp       hold at zero torque, then follow peak timings arriving on UDP 5003
     <enter>   zero torque, ignore UDP
```

Two deliberate departures: **game theory has only one option** (cadence would need stride timing, which
only exists on the Teensy), and **`flexion`/`extension` became `plantar`/`dorsi`**, the ankle's two lobes
and the names the CSV uses. A lobe is disabled by writing its magnitude to 0, which makes `splineAlt`
emit no nodes for it at all.

**UDP mode supports either lobe**, unlike the hip where UDP only ever drove flexion. On the hip a timing
meant rebuilding the whole spline via a builder hardcoded per lobe; here it is one parameter index
(`PlantarPk` = 2 or `DorsiPk` = 3), chosen once at startup.

## Two bugs found by the tests, both from BLE cost

A UDP update is a local variable on the hip but a BLE round trip here, which needed rate limiting — and
the first attempt at that was wrong twice:

- **The rate limiter dropped values instead of deferring them.** `get_latest_timing_value()` wipes its
  buffer on read, so a value arriving inside the 0.5 s window was gone for good. Now the newest pending
  value is held and applied at the next allowed slot.
- **A disable command could be coalesced away** by a later positive timing while it sat pending.
  Non-positive values now bypass the rate limit entirely — it is a safety command, not a state update.

## How to modify

- **Change the machine action surface:** `ActionMap_utilities.py` is the only file that knows `splineAlt`
  (param indices, "magnitude 0 disables a lobe", the `TorqScale`-first ordering). Falling back to PJMC is
  one file; `main` does not move.
- **Change the wire behaviour:** `OpenExoLink_utilities.py`. Note `_wait_for_ack` relies on dict
  *identity* (`ack is not self._ack_snapshot`), not equality — that is deliberate, not a typo for `!=`.
- **Safety limits:** `max_torque_scaling` in `main` is our own ceiling, applied before every write on top
  of the firmware's bounds check. `body_mass_standard` is defined in `main` **only** and passed into the
  action map with no default, so the prompt text and the scaling arithmetic cannot diverge.

## Status and what is unverified

- Host-tested only: 30 unit checks (resolver, confirmed writes, retry-on-silence, stale-ack rejection)
  and 32 menu/loop checks driving `main()` through every branch against a stubbed link. **Nothing has
  talked to a real GUI or exo.**
- The loop rate is 20 Hz and nothing in it needs to be fast; the exo runs its own loop at ~100x that.
- Watchdog, honestly stated: `pump()` only notices a dead link once BLE gives up, roughly
  9.6 s after the exo goes quiet. The real detector is a write that comes back silent.
- On a `link_down` break the exo **cannot** be parked — `park_to_transparency()` says so plainly and
  tells the operator to end the trial from the GUI. This code never starts or ends a trial.

## Audit status (2026-09-08)

- `main_external_control.py` — **audited by the user, done.**
- `Utilities/ActionMap_utilities.py` — **audit in progress.**
- `Utilities/OpenExoLink_utilities.py` — **audit in progress.**

Changes made during the audit so far: logging split into `log_m_action_enabled` /
`log_param_write_enabled` / `log_h_action_enabled`; the human-action buffer switched back to
`np.zeros` + `_h_action_current_stride_count` to match the hip line-for-line; mode-specific loop state
moved into each mode's own init block; the dead `apply_timing()` removed; `body_mass_standard`
de-duplicated. The user also renamed `ExoLink`→`OpenExoLink` and `exo_control_code_path`→
`hippo_control_code_path`.

---

# Also done in this session

## Documentation moved out of `docs/`

`docs/superpowers/{specs,plans}/` → `Modification log with claude/{specs,plans}/`, and the top-level
`docs/` tree was deleted. The user found a second top-level docs root misleading. Moved with `git mv` so
renames are tracked, and every stale path reference was rewritten — 18 markdown files plus two firmware
comments (`Controller.cpp:349`, `SdLogger.h:63`).

## `SplineAlt` status corrected

Its mod-log entry still said "host-verified only — never compiled for Teensy, never flashed, never run on
hardware". **It has since been flashed and works well** (user, 2026-09-08). The original line is kept in
the document for the record. There is therefore no validation gate ahead of using `splineAlt`, and the
PJMC fallback in the design spec is not needed.

## Branch audit — `disconnection_troubleshooting` vs `main_working_branch`

Asked whether the commits ahead are experimental-only. **They are not; keep them.** The delta is
`8f79871` (feature) + `12d0531` (docs), **515 insertions and zero deletions**. Verified:

- The firmware write is in `ExoBLE::setup()` — boot, not per-connection — and parks the value for the GUI
  to *read* rather than notifying, so it adds nothing to the handshake burst that already loses controller
  rows on ~20 % of connections.
- It reuses `ErrorChar` rather than adding a characteristic, avoiding the Windows GATT-cache staleness
  problem. The GATT table is unchanged.
- The GUI read sits before `start_notify`, wrapped in try/except with a 3 s timeout and explicitly
  non-fatal, so the GUI still works against un-flashed exos.
- `SystemReset.h` already existed on `main_working_branch` (`a3fad3e`); this branch only appended two
  `inline` functions. Nothing else in the tree defines `CPU_RESTART` or `NVIC_SystemReset`.
- **Both targets compile clean** — `arduino:mbed_nano:nano33ble` (37 % flash, 48 % RAM) and
  `teensy:avr:teensy41` — so the flagged CMSIS header-ordering hazard is genuinely resolved. `nrf.h`
  does resolve in the mbed_nano core, so the `RESETREAS` path is real and not silently falling back to
  the `RST:UNAVAILABLE` sentinel.

Two things to remember rather than worry about: `RESETREAS` is write-1-to-clear and is consumed once per
boot, and the reason lives in `ErrorChar` only until the first runtime error overwrites it (the GUI
already prints a clear message for that case).

## Max plantar torque raised to 25 Nm — IMPLEMENTED 2026-09-09

Was deferred; the user took the decision on 2026-09-09. **Three constants changed, compiled for both
boards, never flashed and never run on motors.**

| Where | Was | Now |
|---|---|---|
| `Config.h:44` `MAX_JOINT_TORQUE_NM` | `25.0f` | `30.0f` |
| `Controller.cpp` `Spline::calc_motor_cmd()` feed-forward clamp | `±15.0f` | `±25.0f` |
| `Controller.cpp` `SplineAlt::calc_motor_cmd()` feed-forward clamp | `±15.0f` | `±25.0f` |

**The user's rationale, which is the thing to preserve:** the feed-forward alone consistently
*under*-delivers measured torque, and the PID is what brings the measured value up to the prescribed
profile. So the feed-forward must be allowed to ask for the full profile value, and the ceiling must
sit above it — clipping at the peak is exactly what you do not want. The user also explicitly set
aside the ×1.165 packing question: it has not shown up in the measured torque, and measured torque is
what was actually delivered.

**Why 30 and not more.** The gate was never the parameter bounds — `PlantarNm` has always been
bounds-checked at ±50 Nm (`ControllerData.cpp:145`), so 20 or 25 was accepted and then silently
truncated by the ±15 feed-forward clamp. `PlantarNm = 25` produced exactly the same profile as 15,
with no warning. That is now fixed.

**PID authority is the quantity being spent.** The feed-forward clamp bounds only the profile; the PID
correction is added on top of it and has no limit of its own, so `MAX_JOINT_TORQUE_NM` is what
truncates the sum. The gap between the two is what the PID has to work with:

| Feed-forward | Ceiling | PID authority | Tracking error that clips, at `p_gain` 3 |
|---|---|---|---|
| 15 (old) | 25 (old) | 10 Nm | 3.3 Nm |
| 15 (unchanged profile) | **30 (new)** | **15 Nm** | **5.0 Nm** |
| **25 (new peak)** | **30 (new)** | **5 Nm** | **1.7 Nm** |

Note the middle row: **at the profiles currently on the SD card (`PlantarNm 15`) this change is
strictly safer than before**, because the ceiling moved up and the feed-forward did not. Authority only
tightens if `PlantarNm` is actually raised toward 25.

**Verified for this change (2026-09-09):**

- `MAX_JOINT_TORQUE_NM` has exactly one definition (`Config.h:44`) and exactly one enforcement site
  (`Motor.cpp:259`, the final gate in `_CANMotor::send_data()`). No second copy anywhere.
- The ±15 clamp existed in exactly the two spline controllers and nowhere else. No other controller
  has a feed-forward clamp of its own.
- The `_I_MAX` current saturation (`Motor.cpp:327`) does **not** engage: 30 Nm ÷ 4.5 gearing ÷ 1.11 Kt
  = 6.0 A against a 10.3 A full scale (58 %). `_I_MAX` binds at 51.4 Nm, so `MAX_JOINT_TORQUE_NM`
  remains the thing that actually stops a runaway.
- The 12-bit `t_ff` quantisation is 0.025 Nm at the joint, unchanged by this edit.
- The int16 ×100 real-time stream does not wrap: 30 Nm → 3000, against a ±32767 field.
- The Python GUI has no hard-coded torque ceiling of its own — nothing to keep in sync.
- Only the ankle is enabled in `SDCard/config.ini` (`ankle = AK60v3`, gear 4.5; hip/knee/elbow/arm all
  `0`), so the global define currently reaches the ankles only.
- Compiles clean, exit 0, both `teensy:avr:teensy41` and `arduino:mbed_nano:nano33ble`.

**Three things to watch on the bench:**

1. **The define is global.** It is the only absolute torque ceiling in the system and it applies to
   every joint and every controller. Raising it also raised the worst case for PJMC, chirp, step and
   for any PID runaway — from 25 Nm to 30 (from 26.2 to 34.9 with the ×1.165 packing). Nothing else
   stands behind it except `_I_MAX` at 51.4 Nm.
2. **The D term is the most likely thing to hit the new clamp,** not the profile. `d_gain` 0.01 at
   500 Hz multiplies a sample-to-sample torque jump by 5, so once `PlantarNm` is at 25 a **1.0 Nm
   single-sample jump** on the torque reading is enough to clip on its own (it took 2.0 Nm before).
   Spline runs the **raw, unfiltered** torque reading (`torque_alpha` hard-set to 1.0), and the
   gain scheduler does *not* protect the peak — it only engages near zero setpoint. Swing phase is
   still covered by `KD_ZERO` 0.001.
3. **Clipping is visible and cheap.** `Motor.cpp:278-296` prints a rate-limited (1/s) `TORQUE CLAMP:`
   line giving the joint-Nm asked for, and clamps. It does not fault or cut out. Watch for that line
   over USB when first running a 25 Nm profile — it is the direct evidence of whether 5 Nm of PID
   authority is enough.
