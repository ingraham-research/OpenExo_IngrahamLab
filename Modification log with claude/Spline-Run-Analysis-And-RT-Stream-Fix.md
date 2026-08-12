# Running the Spline controller from the laptop GUI — end-to-end analysis, branch comparison, and RT-stream fix

**Date:** 2026-08-12
**Branch:** `fix_spline_jitter` @ `9dfac1d`
**Compared against:** `backup_branch_with_UW_edits` (merge base `192b207`, 2026-08-10)
**Scope:** read-only analysis of the whole path from the GUI's "Update Controller → spline" click to
the CAN frame, plus one targeted fix (the Teensy → Nano → GUI real-time stream).

**Method:** source read end to end (firmware + Python GUI), numeric evaluation of the shipped spline
profile, git archaeology on the RT payload length, and host-side compile + execution of the edited
transport logic. No hardware was involved; every hardware-behaviour claim below is marked as such.

---

# PART 1 — Analysis

## 1.0 What "select spline in the GUI" actually does

1. Update Controller → Apply sends BLE `'f'` plus four 8-byte doubles
   (`joint_id, controller_id, param_index, value`) — `QtExoDeviceManager.py:843`.
2. The Nano validates and forwards over UART — `ble_commands.h:442`.
3. The Teensy's `update_controller_param` (`uart_commands.h:676`): **if `controller_id` differs from
   the current one it switches the controller and re-reads that joint's defaults off the SD card**,
   then applies the one edited parameter.
4. On the next control cycle `AnkleJoint::run_joint` (`Joint.cpp:1105`) dispatches to
   `Spline::calc_motor_cmd` (`Controller.cpp:837`).

**There is no separate "select controller" step.** Changing any single spline parameter is what
switches the leg to spline — live, mid-stride, no ramp-in, no confirmation.

Active profile, `SDCard/ankleControllers/spline.csv`:

| nodes (x = %gait, y = Nm) | sim | %gait | PID | P | I | D |
|---|---|---|---|---|---|---|
| (0,0) (5,−8) (10,−12) (20,0) (20.5,0) | 0 | 1 | 1 | 6 | 0 | 0.03 |

Numeric evaluation of the exact `_spline_interpolate` code: minimum **−12.06 Nm at 9.5 % gait**
(essentially no knot overshoot — the tight 20 → 20.5 spacing is harmless because both y are 0),
zero for all %gait > 20.5, peak slope **1.71 Nm per %gait ⇒ ~155 Nm/s at a 1.1 s stride**.

## 1.1 Will it work?

### BLOCKER (now fixed — see Part 3): the real-time stream was dead

`RealTimeI2C.cpp:28` sized the receive guard from `rt_data::len`, which commit `a546918` changed
from `11` to `MAX_LEN = 16`:

```
byte_buffer_len = 16 * sizeof(float)/sizeof(short) + 2 = 34
```

The Teensy transmits `_packed_len(BILATERAL_ANKLE_RT_LEN = 13) = 28` bytes. The Nano ISR did:

```cpp
static void on_receive(int byte_len) {
    if (byte_len != byte_buffer_len) return;   // 28 != 34 -> every packet dropped
```

Before `a546918` these matched (11 → 24 both sides). Net effect: **blank plots, an empty CSV, dead
battery and status readouts — while the exo is enabled and the spline applies torque.**

Corroborating evidence: the newest trial CSVs in `Python_GUI/Saved_Data/` are dated **2026-08-08**
and carry the *old* 11-channel header (`… ,Status,Exoskeleton time (seconds)`), whereas `a546918`
landed **2026-08-11 21:33**. This firmware had never been run on hardware.

Latent behind it: `BleMessage::data` is `float[10]` (`BleMessage.h`) but `ComsMCU.cpp:189` filled
`i < rt_data::len = 16` — a **24-byte out-of-bounds write at ~100 Hz** on the Nano, plus the same
overrun read in `package_raw_data` / `copy()`. `expecting = 16` also made `ExoBLE::send_message`'s
VLA (171 bytes) too small for a worst-case frame.

### Also broken (now fixed): only 11 of 13 channel names were advertised

`PlottingTitles.cpp` hardcoded `num_columns = 11` while `getColumnHeader` for `bilateral_ankle`
defines 13. The GUI's `_param_names` therefore stopped at `"Status"`, so:

- `"Exoskeleton time (seconds)"` was not found → `ActiveTrialPage._exo_time_idx = None` → **the plot
  x-axis silently fell back to wall-clock BLE arrival time**, which the code's own comment says
  makes a steady 100 Hz trace look jerky. A direct confound for "spline jitter".
- The CSV lost the exo-clock column (`_csv_channel_indices`, `MainWindow.py:349`).
- Battery and Status still resolved via index fallback, so those two survived.

### Preconditions that must hold even with a working stream

| # | Requirement | Where it can silently fail |
|---|---|---|
| 1 | Handshake must deliver the `Ankle(L/R),…,spline,12,…` row | ~3.2 kB payload, 169 × 19-byte chunks over ~3.4 s (`ExoBLE.cpp:34`). Known ~11 % row loss. Lose that row and **spline is simply absent from the dropdown**. Detection exists, but the `declared_rows` check is off by one (firmware counts 40, GUI parses 42), so a *single* lost row escapes it. |
| 2 | Torque calibration must have run | `_calibration` starts at 0 (`TorqueSensor.cpp:18`). The GUI gates Start Trial behind Calibrate Torque + 3 s (`ScanPage.py:407`), so normally OK. |
| 3 | FSR calibration **and refinement** must finish | Ground contact stays `false` until `_calibration_refinement_max > 0` (`FSR.cpp:205`). No strikes ⇒ no percent_gait ⇒ no torque. |
| 4 | ≥ 3 consecutive good strikes | `_num_steps_avg = 3` (`Side.h:181`); `expected_step_duration` stays −1 until the array fills, and `percent_gait = -1` ⇒ spline returns `y[0] = 0`. Fail-safe, but "nothing happens for the first few steps" is expected, not a bug. |
| 5 | **Bilateral checkbox must be ticked** | Unticked ⇒ only one ankle switches to spline; the other stays on zeroTorque. Nothing warns you. |

## 1.2 Failure modes, ranked

### F1 — ~2-second control-loop freeze on every controller change  🔴

`set_controller_params()` (`ParamsFromSD.cpp:512-640`) runs inside the 500 Hz loop. `spline.csv`
ends with a trailing `\r\n` (verified with `od`), so after reading the 16 parameters
`while(param_file.available())` runs one more iteration:

- `parseInt()` hits EOF → Arduino `Stream` 1000 ms timeout → 0
- `readStringUntil('\n')` hits EOF → another 1000 ms timeout

≈ **2 s per joint**, ≈ 4 s for both ankles at trial start (`update_status(trial_on)` →
`set_default_parameters()`, `uart_commands.h:208`). During the stall `Exo::run()` never executes, so
**no CAN frames go out and the AK60v3 holds its last torque command**.

This is very likely the residual cause of the "motor hangs for a bit and outputs a fixed non-zero
torque" symptom that `a6413c9` only partially addressed — that commit removed the `SD.begin()`
remount; these Stream timeouts remain. *(Inference from the Arduino `Stream` timeout contract;
not measured on hardware.)*

### F2 — The output clamp will fire on essentially every step  🔴

`cmd = ff + Kp·(ff − measured)`, so the feed-forward is effectively multiplied by **(1 + Kp) = 7**
during any tracking error. With FF slewing at ~155 Nm/s and a series-elastic ankle that cannot
follow instantly, ~30 ms of lag is ~4.6 Nm of error → ~28 Nm commanded → clipped by
`MAX_JOINT_TORQUE_NM = 25` (`Motor.cpp:257`). The clamp reports via a bare `Serial.print`
(deliberately, because `logger` is gated to Release) — **invisible on a battery-powered trial**.

Also `AK60v3::_I_MAX = 10.3` vs the motor's ±12.0 field scale (`Motor.cpp:806-830`): every command
lands **1.165× larger** than intended, so the real ceiling is ~26.2 Nm and a "12 Nm" spline peak is
really ~14 Nm.

### F3 — D-term noise dominates inside the assist window  🟠

`AI_CNT_TO_V = 3.3/4096`, `TRQ_V_TO_NM = 53.70` ⇒ **1 ADC count = 0.0433 Nm**. The spline runs the
torque reading with `ewma(..., 1.0f)` — **raw, unfiltered** (`Controller.cpp:889`). `_pid`
differentiates on measurement over `dt = 2 ms`:

```
1 LSB of noise -> de/dt = 21.6 Nm/s -> D = 0.03 * 21.6 = 0.65 Nm
```

±3 counts ⇒ **±2 Nm of command dither at up to 250 Hz**, but only while `|torque_cmd| > 0.5 Nm`,
i.e. exactly during the assist pulse. Outside it, gain scheduling drops to `kd = 0.001` and the
noise gain falls ~30×. That asymmetry matches "quiet in swing, buzzy under load".

Compounding: when the superloop runs long (`dt > 2.2 ms`), `time_good` goes false and **D is dropped
to exactly 0 for that cycle** (`Controller.cpp:249`) — a several-Nm discontinuity keyed to loop
timing rather than to anything physical.

### F4 — Gain-schedule switching is discontinuous and has no hysteresis  🟠

`Controller.cpp:917-928`: hard switch between (6, 0, 0.03) and (3, 0, 0.001) at `|cmd| ≤ 0.5 Nm`
**and** `|error| ≤ 3.5 Nm`. At the boundary the P contribution jumps by `3 × error` — up to
**10.5 Nm instantaneously**. The setpoint crosses ±0.5 Nm twice per step and the error band can
chatter at loop rate on sensor noise. The near-zero gains are hard-coded and cannot be changed from
the GUI.

### F5 — Spline lacks PJMC's uncalibrated-sensor guard  🟠

PJMC checks `if (_joint_data->torque_offset_reading == 0) cmd = filtered_setpoint;`
(`Controller.cpp:699`). **Spline does not.** It calls `_pid` unconditionally, and `_pid`
early-returns `cmd` when the sensor is uncalibrated (`Controller.cpp:210`). Since Spline does
`cmd = torque_cmd + _pid(...)`, the result is **exactly double the intended torque** — a −12 Nm
profile becomes −24 Nm. Reachable if the torque sensor is disconnected/reads ~0 V, or if
calibration collects zero samples.

### F6 — Parameter edits during a trial can be lost, or wedge the command parser  🟠

`updateTorqueValues` sends 5 separate **write-without-response** packets
(`QtExoDeviceManager.py:843`). The codebase's own comment at `send_end_trial_sequence` documents
that write-without-response **hangs on WinRT under real-time-notification congestion**.
`BleParser` has **no timeout** on a partial message (`BleParser.cpp:55-83`): if fewer than four
doubles arrive, `_waiting_for_data` stays true and the **next ~8 single-byte commands
(`E`, `G`, `w`, `x`, `H`, `L`, `Z`) are consumed as payload instead of executed**. Worst case: you
press END TRIAL and nothing happens.

### F7 — Node-order edits transiently kill assist; end-node torque is permanent  🟡

`_spline_interpolate` returns `0.0f` if x is not strictly increasing (`Controller.cpp:954`) —
fail-safe, but moving a node right requires editing in the correct order or assist silently
vanishes mid-trial with no message. Conversely `percent_gait <= x[0]` returns `y[0]` and
`>= x[4]` returns `y[4]`, so **a nonzero node1_y or node5_y becomes a constant torque applied
through all of swing and while standing still.**

**All spline parameter bounds are `enabled = false`** (`ControllerData.cpp:104-122`), unlike PJMC's,
so `validate_request` skips range checking entirely and the GUI spinbox allows ±100000. Only the
internal ±15 Nm clamp and the 25 Nm output clamp stand between a typo and a full-scale command.

### F8 — Torque-variance error latches and floods UART  🟡

`torque_failure_count` is incremented and **never reset** (`error_types.h:123`; only other mention
is the declaration at `JointData.h:80`). With `torque_std_dev_multiple = 10` over a 100-sample
window, the first −12 Nm pulse after a quiet window trips it. From then on the check returns true
**every control cycle, forever**, calling `ErrorReporter::report()` — ~1000 UART messages/second
across two ankles. Two aggravators:

- `Joint.cpp:1127` — `for (i=0; i < errorQueueSize(); i++) pop()` drains only half the queue each
  pass, so `std::queue<ErrorCodes>` **grows without bound** on the heap.
- Every check calls `online_std_dev`, which takes the queue **by value and copies it again**
  (`Utilities.cpp:344`) — ~4000 deque copies/s across both ankles plus `check_response`. Real,
  unmodelled control-loop jitter.

All error handlers have their `motor.enabled = false` **commented out**, so an error takes no
protective action.

### F9 — `-fpermissive` hides a const-map bug  🟡

`ParamsFromSD.cpp:495` does `controller_parameter_filenames::ankle[controller_id]` on a
`const std::map`. Verified with a minimal repro: GCC rejects this *except* under `-fpermissive`,
which Teensyduino enables. It therefore builds, discards const, and **inserts an empty filename for
an unknown controller id** → `SD.open("")` fails → **the controller switches but keeps the previous
controller's parameter array**, reinterpreted as spline nodes. The error code returned by
`set_controller_params` is discarded by every caller.

### F10 — Latent hang on a malformed SD file  🟡

`while(!param_file.findUntil('\n','\n')) { ; }` (`ParamsFromSD.cpp:531`) spins forever at EOF. If
`spline.csv` is ever edited to fewer than 6 lines the Teensy hangs with the motors holding their
last command. There is no watchdog.

## 1.3 Safety hazards

| Severity | Hazard |
|---|---|
| High | **Standing weight-shift triggers a full assist pulse.** With `heelFsrPresent = 0`, a ground strike is just the toe FSR's rising edge from swing (`Side.cpp:281`). Rocking on the spot re-arms `percent_gait = 0` and fires the whole −12 Nm plantarflexion ramp over ~230 ms while the wearer is stationary. |
| High | **~2 s bilateral freeze on controller change while walking** (F1). Both ankles hold their last CAN command. |
| High | **No ramp-in / no arming step.** Any single parameter Apply switches the live controller instantly, at whatever gait phase you are in. |
| High | **Asymmetric assist if "Bilateral mode" is unchecked** — one leg at −12 Nm, the other transparent, with no warning. |
| Medium | **Direction convention unverified from the GUI.** `ankleFlipMotorDir = right`, `ankleFlipTorqueDir = left`. If either is wrong the PID becomes positive feedback. `CalibrManager` exists for exactly this check but has **no CSV entry in `controller_parameter_filenames::ankle`**, so it never appears in the handshake and **cannot be selected from the GUI**. |
| Medium | **`torque_output_threshold = 60 Nm`, and the handler does nothing anyway.** No torque-based cutout. The only real limit is the 25 Nm (≈26.2 Nm actual) frame clamp. |
| Medium | **Bad step-duration estimates scale the pulse.** `expected_step_duration` accepts anything within 0.25×–1.75× of the stored window (`Side.cpp:433`). At 300 ms the profile compresses to ~60 ms and the FF slew reaches ~570 Nm/s. |
| Medium | **UDP remote control is ON by default** (`utils/config.py:143`, `127.0.0.1:9750`). Any local process can change torque parameters with no auth. |
| Low / unknown | **`END_TRIAL_CUTS_MOTOR_POWER = 1` is documented as never bench-checked** (`Config.h:46-56`). If dropping that pin shorts the phases rather than cutting drive, End Trial mid-stride brakes both ankles. |
| Low | `status_defs::messages::error` is **never set anywhere** — the exo cannot report an error state, and `set_status` would latch it permanently if it ever did (`ExoData.cpp:152`). |

## 1.4 What the GUI log will not show

1. **Before the fix: nothing at all** — the I²C guard mismatch meant zero rows. A blank CSV looks
   like "no data yet", not "telemetry is broken".
2. **5× decimation with no anti-aliasing.** The loop is 500 Hz; RT is emitted every 9000 µs ≈ 111 Hz
   (`Config.h:105`). Everything above ~50 Hz — where the D-term dither, the gain-schedule chatter
   and the clamp events live — **aliases or vanishes**. A 2–4 ms torque spike has roughly a 20–40 %
   chance of being sampled at all.
3. **Instantaneous snapshots, not min/max.** `motor.t_ff` is read once per RT tick; a single-cycle
   26 Nm clamp event is very likely never logged.
4. **Clamp and non-finite events are invisible.** `Motor.cpp:284` uses a bare rate-limited
   `Serial.print`; there is no clamp counter in the RT stream.
5. **Device error notifications go nowhere.** `qt_dev.deviceErrorReceived` is connected **only**
   inside the `if self.remote.is_bound():` block (`MainWindow.py:184`). No UI element, no CSV
   column, no popup.
6. **Nothing streams outside `trial_on / fsr_calibration / fsr_refinement`** (`Exo.cpp:108`). Torque
   during **torque calibration** and during the **entire End-Trial sequence** is unlogged — exactly
   the window where the large-torque event was seen.
7. **No firmware timestamp in the CSV** (because of the 11-vs-13 names bug) — so BLE transport
   jitter cannot be separated from control jitter. *(Fixed in Part 3.)*
8. **The plot x-axis was wall-clock**, which makes bursty BLE delivery look like signal jitter.
   *(Fixed in Part 3.)*
9. **Fixed-point range.** All RT channels are `(short)(value * 100)` (`Utilities.cpp:252`) — 0.01
   resolution, wraps past ±327.67. Fine for torque; the exo clock wraps every 655 s (handled
   GUI-side).
10. **"Desired Torque" ≠ what the motor got.** Channels 0/2 are the pre-PID feed-forward; the
    post-PID/post-clamp value is only on channels 8/9.
11. **`_mark_index = 1`** (`ComsMCU.h`) — a firmware mark increment overwrites **channel 1 =
    Measured Torque (L)** for one packet. Dormant (the Qt GUI's Mark button never sends `'N'`) but a
    live landmine; now commented in the header.
12. **Silent controller/parameter mismatch.** If the SD read fails the controller switches but the
    old parameter array survives (F9), while the GUI updates its value cache **from the ack alone**
    (`MainWindow.py:570`) — so the GUI can display values the firmware never loaded.
13. **No sequence number on the RT packets.** Teensy→Nano loss is a silent overwrite of the latest
    value; the GUI's drop detection is timing-based only.

---

# PART 2 — Comparison with `backup_branch_with_UW_edits`

Every finding above, checked against the UW branch. "Present" means the same defect exists there.

## 2.1 Streaming pipeline

| # | Finding | UW branch | Notes |
|---|---|---|---|
| B1 | RT I²C length-guard mismatch | **ABSENT** | `rt_data::len = BILATERAL_HIP_ANKLE_RT_LEN = 11` and `BILATERAL_ANKLE_RT_LEN = 11` → both sides compute 24 bytes. Self-consistent, so the stream works. The bug is **unique to `fix_spline_jitter`**, introduced by `a546918`. |
| B2 | `BleMessage::data[10]` overrun | **Present, milder** | `_max_size = 10` with `expecting = 11` → 1 float past the end (lands on the private `_size`), not 6. |
| B3 | `num_columns = 11` vs channel count | **ABSENT** | UW genuinely has 11 channels, so 11 labels is correct. |
| B4 | `float_values = new float(len)` single-object heap alloc | **PRESENT** (worse) | UW still has `static float* float_values = new float(len);` — a 4-byte allocation written with 11 floats on every packet. Fixed on our branch. |
| B5 | `_pack` writes `len + 2` as a float count | Present | Same off-by-two; benign on UW because the reader over-read into a same-sized buffer. |
| B6 | `MAX_PARSER_CHARACTERS = 8` undersized VLA | Present | Same value; smaller `expecting` (11) makes an overflow less likely but not impossible. |
| B7 | No RT sequence number | Present | Same design. |

## 2.2 Spline controller

| # | Finding | UW branch | Notes |
|---|---|---|---|
| B8 | Spline exists for the ankle (`id = 12`) | Present | Same enum, same `_spline_interpolate`. |
| B9 | **Shipped ankle profile** | **Very different** | UW: nodes (0,0) (25,0) (48,12) (63,0) (100,0), **PID Flag = 0**, all gains 0 → **open loop, +12 Nm peak at 48 % gait** (a conventional push-off profile). Ours: −12 Nm at 10 % gait with PID on and P = 6. |
| B10 | F2 clamp saturation / (1+Kp) amplification | **ABSENT on UW** | PID is off there, so no amplification. Also no `MAX_JOINT_TORQUE_NM` clamp exists on UW at all — see B18. |
| B11 | F3 D-term noise | **ABSENT on UW** | PID off; and the torque reading is filtered (`ewma(..., 0.5f)`) rather than raw. |
| B12 | F4 gain-schedule discontinuity | **ABSENT on UW** | Gain scheduling was added on our branch. |
| B13 | F5 missing uncalibrated-sensor guard (torque doubling) | **PRESENT** | Identical code; `Spline` calls `_pid` unconditionally. |
| B14 | F7 spline bounds all `enabled = false` | **PRESENT** | Identical table. |
| B15 | percent_gait 1 % quantization staircase | **PRESENT** (worse) | UW computes into `int percent_gait` before widening to float — the staircase our `226854b` fixed. |
| B16 | F10 `findUntil` EOF spin | Present | Identical. |
| B17 | F9 const-map `operator[]` | Present | Identical. |

## 2.3 Motor / safety

| # | Finding | UW branch | Notes |
|---|---|---|---|
| B18 | `MAX_JOINT_TORQUE_NM` output clamp | **ABSENT on UW** | Zero occurrences. **No absolute torque ceiling and no NaN rejection at all.** Ours is strictly safer here. |
| B19 | AK60v3 guard on `enable()` / `zero()` | **ABSENT on UW** | `enable(bool)` has no top guard and `check_response` calls `enable(true)` unguarded — this is the malformed-frame path documented as having destroyed the right ankle on 2026-07-23. **UW is exposed to it.** |
| B20 | Continuous zero-torque frames while disabled | **ABSENT on UW** | UW uses `else if (_prev_motor_enabled)` — a one-shot zero frame, so the AK60v3 "hold last command" freeze is present. |
| B21 | Unconditional CAN drain (`read_data` every cycle) | **ABSENT on UW** | UW gates on `if (_motor_data->enabled)`, so the FIFO backlog / positional-routing starvation and the frozen current-variance signal are both present. |
| B22 | End Trial deferred reset | **ABSENT on UW** | UW's `get_system_reset` reboots inline after `delay(10)` → the AK60v3 holds its last command through the reboot. |
| B23 | F8 torque-variance latch + queue half-drain | **PRESENT** | Identical in both. |
| B24 | Error handlers neutered (`motor.enabled = false` commented out) | Present | Identical. |
| B25 | `status_defs::messages::error` never set | Present | Identical. |

## 2.4 SD / parameters

| # | Finding | UW branch | Notes |
|---|---|---|---|
| B26 | F1 ~2 s Stream-timeout stall in `set_controller_params` | **PRESENT** | Identical loop structure. |
| B27 | `SD.begin()` remount on every controller change | **PRESENT** (worse) | 6 call sites on UW; ours caches via `_sd_ready()`. So UW's controller-change stall is the ~2 s **plus** a card remount. |
| B28 | `set_default_parameters()` on trial start wipes GUI edits | Present | Identical. |

## 2.5 GUI

| # | Finding | UW branch | Notes |
|---|---|---|---|
| B29 | Handshake row-loss detection | **ABSENT on UW** | No `n,<rows>` header in `ExoBLE.cpp`, no `_emit_matrix_completeness` in `RtBridge.py`. A controller silently missing from the dropdown is undetectable there. |
| B30 | CSV channel selection | **Worse on UW** | Hardcoded `values[:10]`; ours selects by name. UW therefore never logged channels ≥ 10 at all. |
| B31 | Plot pages | **Worse on UW** | Only two 4-channel blocks (0-3, 4-7) — channels 8+ unreachable. Ours adds a third page for Commanded Torque. |
| B32 | Plot x-axis from exo clock | **ABSENT on UW** | Wall-clock only, so the "jerky trace" artifact is unconditional there. |
| B33 | F6 BleParser wedge (no partial-message timeout) | **PRESENT** | Identical parser. |
| B34 | UDP remote control | **ABSENT on UW** | `Python_GUI/remote/` does not exist. One less attack surface. |
| B35 | Param-update ack / rejection plumbing | **ABSENT on UW** | No `_csv_channel_indices`, no ack handling — parameter updates are fire-and-forget with no feedback. |
| B36 | 500 Hz `Serial.print` in `_pid` and `handle_msg` | **PRESENT** | Both present verbatim on UW. |

## 2.6 Summary of the comparison

- **The RT-stream break, and only that, is unique to `fix_spline_jitter`.** It was introduced three
  commits ago and has never been run on hardware.
- **On motor safety, `fix_spline_jitter` is substantially ahead of UW**: it has the output clamp,
  NaN rejection, the AK60v3 enable/zero guards, continuous zero frames, unconditional CAN drain,
  and the deferred End-Trial reset — UW has none of these. Reverting to UW to "get streaming back"
  would trade a telemetry bug for several actuation hazards, including the one that damaged an
  ankle.
- **The spline itself is far more aggressive on our branch** (closed-loop, P = 6, −12 Nm at 10 %
  gait) than on UW (open-loop, +12 Nm at 48 % gait). Most of the jitter/clamp findings (F2, F3, F4)
  follow from that configuration change, not from a code defect.
- **Shared, unfixed on both:** the torque-doubling guard gap (F5/B13), disabled spline bounds
  (F7/B14), the ~2 s SD stall (F1/B26), the torque-variance latch and error-queue leak (F8/B23),
  the BleParser wedge (F6/B33), and the `findUntil` EOF spin (F10/B16).

---

# PART 3 — Fix: making Teensy → Nano → GUI streaming work

## 3.1 Root causes addressed

| ID | Root cause | Fix |
|---|---|---|
| R1 | I²C receive guard sized from buffer **capacity** (34 B) while the Teensy sends the **payload** (28 B) | ISR now accepts a *range* of well-formed lengths and records the byte count |
| R2 | Ambiguous constant `rt_data::len` used to mean both capacity and length | Renamed to `rt_data::capacity`; the payload length now travels **with the packet** |
| R3 | `_pack` wrote `len + 2` into the float-count field; `poll` read it as a float count | Field is now an unambiguous float count, and `poll` cross-checks it against the received byte count |
| R4 | `ComsMCU::update_gui` wrote `rt_data::len` (16) floats into `BleMessage::data[10]` | Length comes from `poll()`; `BleMessage::_max_size` raised to 16; the write is clamped to `BleMessage::k_max_data` |
| R5 | `MAX_PARSER_CHARACTERS = 8` undersized `send_message`'s VLA vs `BleParser::_maxChars = 12` | Raised to 12 |
| R6 | `create_plotting_titles` hardcoded 11 labels for a 13-channel layout | New `getColumnCount(config)` derived from `rt_data::*_RT_LEN` |
| R7 | Nothing stopped a future payload from exceeding the 32-byte Wire buffer | `rt_data::MAX_WIRE_PAYLOAD = 15` and a clamp in `real_time_i2c::msg()` |
| R8 | Dead UART fallback compared `msg.len` against capacity | Bound-check instead of equality (and the per-TU `static` trap is now documented) |

## 3.2 Files changed

| File | Change |
|---|---|
| `ExoCode/src/RealTimeI2C.h` | `len` → `capacity`; added `MAX_WIRE_PAYLOAD`; `poll()` gained an out-length parameter; capacity-vs-payload doc block |
| `ExoCode/src/RealTimeI2C.cpp` | `byte_buffer_len` → `byte_buffer_capacity` + `byte_buffer_min`; new `new_byte_count`; range-based ISR guard; payload clamp in `msg()`; `_pack` writes the float count; `poll()` derives and reports the length |
| `ExoCode/src/ComsMCU.h` | includes `RealTimeI2C.h`; new `_rt_len` member; comment on `_mark_index` |
| `ExoCode/src/ComsMCU.cpp` | stores the config-derived length; uses the packet length from `poll()`; clamps before filling `BleMessage` |
| `ExoCode/src/BleMessage.h` | `_max_size` 10 → 16; public `k_max_data` |
| `ExoCode/src/ExoBLE.h` | `MAX_PARSER_CHARACTERS` 8 → 12 |
| `ExoCode/src/PlottingTitles.h` | new `getColumnCount()`; includes `RealTimeI2C.h` |
| `ExoCode/src/PlottingTitles.cpp` | uses `getColumnCount()` instead of the literal 11 |
| `ExoCode/src/uart_commands.h` | bound-check in `update_real_time_data`; **500 Hz `Serial.print` commented out** |
| `ExoCode/src/Controller.cpp` | **500 Hz `Serial.print` commented out** in `_pid` |
| `ExoCode/src/ble_commands.h` | comment clarifying that the RT length in the command table is inert |

**No Python GUI files were changed** — the GUI already selects channels by name, so restoring the
13 labels is enough.

## 3.3 Serial prints removed

Two prints ran at control-loop rate and are now commented out, with an explanation left in place:

1. `Controller.cpp` `_Controller::_pid` — "Torque sensor not calibrated…", once per joint per cycle
   (~1000 lines/s at 500 Hz with two ankles).
2. `uart_commands.h` `UART_command_utils::handle_msg` — `msg.command`, once per inbound UART
   message; becomes ~1000/s under the latched torque-variance error (F8).

Both are kept commented rather than deleted, with a note that the underlying conditions are worth
knowing about but **reading serial off this hardware is impractical** — the exo runs untethered on
battery during a trial, and Teensy's `usb_serial_write` blocks for up to ~120 ms when the port is
enumerated but undrained, which is a control-loop stall on the very path that is misbehaving. If
either state needs to be visible, it should go on a real-time channel or in the status word.

The rate-limited `TORQUE CLAMP:` print in `Motor.cpp` was **kept** — it is a safety event, capped at
one line per second, and is the only record of a clamp.

## 3.4 Verification performed

No hardware. Two host-side harnesses were used.

**(a) Transport logic compiled and executed against the real header.** The edited
`_packed_len`/`_pack`/`on_receive`/`poll` were compiled with `g++ -std=gnu++17 -fpermissive`
including the actual `RealTimeI2C.h`, with stubs only for `Wire`/`utils`/interrupts:

```
byte_buffer_capacity = 34, byte_buffer_min = 4, rt_data::capacity = 16, MAX_WIRE_PAYLOAD = 15
  PASS  tx length is 28 bytes
  PASS  header float count is 13
  PASS  ISR accepts it
  PASS  poll returns true / reports 13 floats
  PASS  values round-trip (max err 0.0100 = int16 x100 quantisation)
  PASS  ISR rejects short / oversized / odd-length packets
  PASS  legacy header (len+2) still yields 13
  PASS  over-long payload clamped
ALL C++ CHECKS PASSED
```

The legacy-header case matters: the two boards are flashed **separately**, so a Nano running the new
code must still decode a Teensy running the old code. It does, because the length is derived from
the byte count and only cross-checked against the header.

**(b) Full pipeline simulated in Python** (Teensy pack → I²C guard → poll → `update_gui` bounds →
`package_raw_data` → `RtBridge` parse → GUI name resolution), transcribed from the sources. All 27
checks pass, including that the *old* code would have dropped the same packet (28 B sent vs 34 B
demanded) and would have written 16 floats into a 10-float array.

Key measured numbers with the fix in place:

| Stage | Value |
|---|---|
| I²C packet, bilateral_ankle | **28 bytes** (2 preamble + 13 × 2), fits the 32-byte Wire buffer |
| Length recovered by `poll()` | **13** |
| `BleMessage` fill | 13 of 16 floats — **in bounds** |
| BLE frame, typical | **69 bytes** of a 208-byte buffer |
| Channel labels advertised | **13** (was 11) |
| CSV columns | epoch, mark + **12 channels** (was 10) — now includes Commanded Torque L/R **and** the exo clock |
| Plot x-axis | resolves `"Exoskeleton time (seconds)"` at index 11 → **exo clock, not wall clock** |
| Plotting-titles row | 234 B of a 400 B buffer (166 B headroom) |
| Handshake payload | 3203 → ~3252 B; 169 → 172 chunks (+3, ~60 ms) |

## 3.5 Re-analysis of the streaming pipeline after the fix

Stage by stage, what can still go wrong:

| Stage | Status | Residual risk |
|---|---|---|
| Teensy samples `get_real_time_data` every 9 ms | OK | Only while status ∈ {trial_on, fsr_cal, fsr_refinement, error}. Torque-calibration and post-End-Trial windows remain unlogged. |
| `real_time_i2c::msg` packs 13 int16 | OK | Values outside ±327.67 wrap. Torque and FSR are safe; the exo clock wraps every 655 s and is handled GUI-side. |
| I²C transfer (28 B) | OK | No ACK, no sequence number. A dropped packet is a silent overwrite of the latest value. Unchanged by this fix. |
| Nano ISR | OK | Now length-tolerant, and rejects malformed frames. |
| `poll()` | OK | Length is derived from the byte count, cross-checked, and clamped. |
| `update_gui` → `BleMessage` | OK | Double-clamped (`k_max_data`, `capacity`). |
| `package_raw_data` → BLE notify | OK | 69 B typical, 174 B worst case, 208 B buffer. **Depends on a negotiated MTU > ~72 B** — true in practice (the 11-channel frame was ~60 B and worked), but if the MTU ever stayed at the 23-byte default the GUI could not reassemble, because `RtBridge` only enters its RT branch on a chunk containing `'c'`. Pre-existing, not introduced here. |
| `RtBridge.feed_bytes` | OK | Parses 13, pads to 16. |
| CSV / plots / readouts | OK | All resolved by name. |

**Not fixed by this change** (unchanged, still open): F1 SD stall, F2 clamp saturation, F3 D-term
noise, F4 gain-schedule discontinuity, F5 torque doubling, F6 BleParser wedge, F7 disabled bounds,
F8 error latch/leak, F9 const map, F10 EOF spin, and every item in §1.3 except the logging ones.

## 3.6 What to check on first hardware run

1. Flash **both** boards. The fix spans Teensy and Nano.
2. Confirm the GUI plots and CSV populate. Expect **12 data columns**, ending with
   `Commanded Torque (L)`, `Commanded Torque (R)`, `Status`, `Exoskeleton time (seconds)`.
3. Confirm the plot x-axis advances smoothly (exo clock) rather than in bursts.
4. With the joint clamped and unworn, run a single spline step and watch for `TORQUE CLAMP:` on
   serial — F2 predicts it will fire.
5. Time a controller change; F1 predicts a ~2 s freeze.
