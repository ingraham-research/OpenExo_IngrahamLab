# Experiment Orchestrator — a backend-owning "main code" driving the GUI over UDP

**Date:** 2026-09-08
**Scope:** New host-side Python package. **No firmware changes. No GUI changes.**
**First target:** `GameTheory_1D_Backend`, human action = self-paced-treadmill speed,
machine action = torque percentage.
**Status:** Implemented — `Python_GUI/external_control/` (see its README and PSEUDOCODE.md).
Not yet run against real hardware.
**Related:** `Modification log with claude/Remote-Control-UDP.md`,
`Modification log with claude/SplineAlt-Shape-Parameterised-Controller.md`,
`Modification log with claude/2026-07-14-control-loop-and-transparency-backlog.md`,
`Modification log with claude/plans/2026-07-18-udp-remote-control.md`.

---

## Goal

Stand up an external program that owns an optimization backend, computes machine actions, and
applies them to the exo by writing controller parameters through the GUI's UDP remote service —
the OpenExo analogue of `main_control_code.py` in
`E:/Research/Exoskeletons/Max Shepherd's exo/Exo control codes/`.

## The governing architectural decision

**The orchestrator does no signal processing.** It writes parameters and consumes observations
handed to it. It never derives an observation from raw exo telemetry.

Consequences:

- Any exo-derived observation must be computed on the Teensy and sent up
  (Teensy to Nano to GUI to UDP), arriving at low frequency as a discrete value.
- External instruments (self-paced treadmill, metabolic cart) are **not** exo data and connect
  directly to the orchestrator on their own sockets. No GUI hop.
- **For the first target, the exo is write-only.** Human action is treadmill speed over the
  existing MATLAB UDP feed. Nothing needs to come back from the exo except acks.

This is what removes firmware and GUI work from the critical path.

## What this is *not*

`main_control_code.py` runs a 200 Hz loop that owns the motors: it builds torque splines in
Python, calls `TorqueSplineController.update()`, and writes CAN frames. **None of that ports.**
The Teensy owns the control loop. The orchestrator is supervisory: it changes one number,
infrequently, and confirms the change landed.

Three things follow, and each was a wrong turn in an earlier draft of this design:

| Rejected | Why |
|---|---|
| A Python-side stride detector off `toe_stance` | Only existed to reconstruct `left_controller.next_duration` as the human action. It buys nothing: the treadmill already sends **one speed packet per step**, so the stride signal arrives from outside the exo fully formed, and counting successful reads counts strides exactly. The hip exo's `HS_count` gate before each fetch was redundant with that. If stride *duration* is ever wanted, the Teensy computes and sends it. |
| A background RX thread / dual sockets for a "100 Hz consumer" | `rt` is opt-in per subscriber. Subscribe to `ack` + `status` only and traffic is a handful of datagrams per action. Even subscribed to `rt` (~13 floats of JSON at ~100 Hz, about 30 KB/s, `Config.h:166`) a full 2 s block would not overflow a default socket buffer. Throughput was never the problem. |
| Applying the new setting on a stride boundary from the host | If mid-stride application ever proves to matter, the fix is a pending-parameter latch in firmware applied at the next heel strike — not host-side timing games. Out of scope. |

**What is actually a hazard, and is rate-independent:** `ExoRemote._command` drains stream frames
while waiting for its own `ok` reply, so an ack arriving inside that window lands in `_last_ack`
and is *never re-delivered* by `stream()`. A `stream("ack")` loop started after `set_param` can
hang for its full timeout on a write the exo already acknowledged. This is documented at length in
`Python_GUI/remote/client.py`'s `last_ack()` docstring. The fix is the snapshot-before /
compare-after pattern that docstring prescribes — not concurrency.

**Therefore: single socket, single thread, synchronous.**

## The machine action — SplineAlt `TorqScale`

Max's game-theory SPT branch applies the machine action as a global scale on a fixed profile:

```python
new_torque_profile = [(t, val * new_torque_percentage * body_mass_scaling_factor / 100)
                      for t, val in Optimized_torque_profile]
```

`SplineAlt`'s `TorqScale` is a literal mirror of this, and is the right knob:

- **Parameter index 10**, units percent, `SDCard/ankleControllers/splineAlt.csv` default 95.
- Scales node amplitudes at construction (`ExoCode/src/Controller.cpp:1239-1246`), i.e. it lands on
  `torque_cmd` **before the PID and before the plus/minus 15 Nm clamp**. This is exactly the
  injection point the backlog doc identifies as correct, and it dodges the failure it warns about:
  scaling after the PID makes the loop servo *measured* torque back up to the unscaled target and
  cancel the scale entirely, silently, whenever `use_pid=1` — which is our configuration.
- **`TorqScale = 0` is a usable transparency mode** (`Controller.cpp:1420`): every node collapses to
  zero and the gain scheduler holds the tuned zero-torque gains for the whole stride. So the
  game-theory action range `[0, 100]` maps onto the parameter with no dead zone and no
  discontinuity at the bottom, and `m = 0` is a safe, meaningful action rather than an edge case.
- One parameter, so one write per action.

**Alternatives considered.** PJMC `max stance torque (Nm)` is proven on hardware and would work
with a percent-to-Nm conversion, but it is a different controller shape, not a mirror of the hip
experiment. `Spline` has no global scale — scaling it means 12 writes to the y-nodes, non-atomic,
with the leg walking through the intermediate profiles. The generic `torque_scale` parameter in the
backlog is the eventual right answer for *all* controllers but is a firmware project and is **not**
a prerequisite here.

**Hardware status.** `SplineAlt` has been flashed and run on hardware and works well (confirmed by
the user 2026-09-08). Its mod-log entry still carried the original "host-verified only, never
flashed" status and was stale; that line has been corrected. There is therefore **no validation gate**
ahead of using it, and the PJMC fallback is not needed.

## Session setup vs. experiment writes

Split the writes into two classes. This keeps the experiment loop to a single parameter.

**Session setup (once, operator-confirmed):** the profile shape and the *mass-scaled* reference
magnitudes. Max applies `body_mass_scaling_factor = participant_mass / 75` to the torque values;
here that is folded into `PlantarNm` / `DorsiNm` and written once at session start.

**Experiment loop (many, backend-driven):** `TorqScale` only.

## Module layout

```
Python_GUI/orchestrator/          # separate process; imports remote/client.py only
  main_experiment.py   # mode select, session setup, supervisory loop
  exo_link.py          # set_param_confirmed(): send, match ack, retry, clamp
  action_map.py        # backend action -> ordered set_param writes
  adapters.py          # BackendAdapter protocol + GameTheory adapter
  session_log.py       # op-log mirroring gametheory_op_log format
```

Max's backends are imported unmodified. `HiLO_backend.py:9-11` already describes this exact
deployment ("OpenExo: a standalone main_control_code calls this backend and communicates to the GUI
via UDP connection"), so no backend edits are expected.

### `exo_link.set_param_confirmed`

The only non-trivial piece. Per write:

1. Snapshot `last_ack()`.
2. `set_param(...)` — raises `RemoteError` on a name/bounds rejection by the GUI.
3. Compare `last_ack()` against the snapshot; if unchanged, fall back to `stream("ack", timeout=...)`.
4. Match on `(joint_id, controller_id, param_index)`. Acks carry no request id.
5. Retry N times. **Silence is the only failure signal** — a dropped BLE write produces no negative
   ack. Abort the whole action set rather than leaving a half-applied profile.

**Send two unilateral writes, never `bilateral=True`.** They are identical on the wire —
`Python_GUI/services/QtExoDeviceManager.py:744-750` expands `bilateral` into two sequential BLE
writes GUI-side — but `_last_ack` holds only the most recent ack, so a bilateral write can have its
first ack overwritten before it is read. Two unilateral writes give unambiguous 1:1 matching.
Ankle joint ids: **left = 68, right = 36** (`ParseIni.h:125-137`). Do **not** use
`Python_GUI/utils/config.py: JointConfig.ID_TO_NUM` — all 8 of its entries have left/right inverted
and it would command the wrong leg. The handshake matrix is the source of truth.

### Adapter contract

```
submit_observation(value, t)   # h action / cost / preference
poll() -> action | None        # None = nothing new
finished -> bool
shutdown()
```

`GameTheory_1D_Backend` maps near 1:1: `read_human_action()` in; poll `output_ready`, read
`m_action_next`, set `output_fetched = True`; `finished`; `stop_and_finalize()`.

## The loop

Event-driven and mostly asleep, clocked by the backend's own cadence (`human_lag_time`), not by a
telemetry rate.

```
wait for status: connected + non-empty matrix
session setup writes (mass-scaled magnitudes, profile shape), operator-confirmed
loop:
    sample SPT speed from its UDP feed - one packet per step, so N reads == N strides
    once N strides are collected: backend.submit_observation(mean_speed, t)
    if backend.poll() -> m:
        m = clamp(m, 0, m_max)
        exo_link.set_param_confirmed(Ankle(L), splineAlt, TorqScale, m)
        exo_link.set_param_confirmed(Ankle(R), splineAlt, TorqScale, m)
        log
    if backend.finished: break
```

## Safety

- **No trial start/stop, motor enable, or calibration over UDP** — deliberate, per
  `Remote-Control-UDP.md`. The operator drives the GUI. The orchestrator gates on
  `status: connected` plus a non-empty matrix and never starts anything.
- **Host-side clamp on `m` before the write**, on top of the firmware's bounds check. Start with
  `m_max` well below 100.
- **Sizing `m_max`:** at `TorqScale = 100` the CSV's `PlantarNm = 15` requests 15 Nm, against
  SplineAlt's plus/minus 15 Nm feed-forward clamp. Separately, commanded torque is understood to
  come out about 1.165x the requested value because `t_ff` is packed against a plus/minus 10.3 A
  field while the firmware treats it as plus/minus 12.0 N·m — so a 15 Nm request is roughly 17.5 Nm
  at the joint. **Re-verify this factor on the bench before choosing `m_max`**; it is carried here
  from prior investigation, not re-derived.
- **Writing a parameter to a non-active controller switches the joint to it and loads that
  controller's defaults.** The first setup write therefore establishes `splineAlt` *and* its
  defaults; subsequent writes refine. Order the setup writes so no intermediate state is unsafe,
  and confirm each before sending the next.
- Watchdog: on `status: disconnected` / `device_error`, stop issuing actions and surface it loudly.
  Note that the Nano radio-silence failure mode presents as a disconnect only after the roughly
  9.6 s BLE supervision timeout, so detection is not immediate.

## Phasing

| Phase | Deliverable | Gate |
|---|---|---|
| 0 | `exo_link` + `session_log`; `ping` / `get_matrix` / subscribe only. No writes. | none |
| 1 | `set_param_confirmed` with ack matching and retry; scripted `TorqScale` sweep | First writes. Leg off the body, motors on a stand |
| 2 | Session setup writes + `action_map`; measure ack latency and settling time | first walking test |
| 3 | GameTheory adapter end-to-end against the existing SPT UDP feed | full protocol |
| 4 | HiLO adapter — needs `HiLO_backend._thread_loop` finished (it has TODOs and never publishes a candidate) plus COSMED TCP | later |
| 5 | Bayesian A/B — different loop shape (query, hold A, hold B, ask); needs the preference UI ported | independent |

## Reference — facts this design rests on

- Remote service commands `set_param` / `subscribe` / `unsubscribe` / `get_matrix` / `ping`;
  streams `rt` / `ack` / `matrix` / `status` (`Python_GUI/remote/service.py`).
- `setParamRequested` to `_apply_param_update`, the same path as the GUI Apply button
  (`Python_GUI/MainWindow.py:169-196`, `:972`).
- RT channels for `bilateral_ankle` (`ExoCode/src/uart_commands.h:426-454`): 0-3 desired/measured
  torque L/R, 4-7 toe FSR + toe_stance L/R, 8-9 commanded torque L/R, 10 status, 11 exo clock,
  12 battery. **No `percent_gait`** — the hip config has it, the ankle does not.
- RT payload is 13 floats against a **15-float hard ceiling** (I2C 32-byte transfer buffer),
  `ExoCode/src/RealTimeI2C.h:33,43-45`. Two spare slots exist, but `rt` is a fixed-rate array and
  is the wrong shape for a per-stride event.
- RT rate about 100 Hz (`ExoCode/src/Config.h:166`).
- Channel 11 (exo clock) is int16 x100 and **wraps at 327.67 s**. Use the host clock as the session
  timebase; treat ch 11 as a dropout diagnostic only.
