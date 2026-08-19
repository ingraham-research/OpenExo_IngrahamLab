# Running the Spline controller from the laptop GUI — end-to-end analysis, branch comparison, and RT-stream fix

**Date:** 2026-08-12
**Branch:** `fix_spline_jitter` @ `9dfac1d`
**Compared against:** `backup_branch_with_UW_edits` (merge base `192b207`, 2026-08-10)
**Scope:** read-only analysis of the whole path from the GUI's "Update Controller → spline" click to
the CAN frame, plus one targeted fix (the Teensy → Nano → GUI real-time stream).

**Method:** source read end to end (firmware + Python GUI), numeric evaluation of the shipped spline
profile, git archaeology on the RT payload length, and host-side compile + execution of the edited
transport logic. No hardware was involved; every hardware-behaviour claim below is marked as such.

**Deliberately derived independently.** This analysis was produced without reading the other
documents in this folder, at the user's instruction, so it would not inherit their conclusions.
Cross-references were added afterwards (2026-08-12). Prior work covering overlapping ground:
`Spline-Jitter-Diagnosis.md` (round 1 — the gain scheduler and the `percent_gait` int→float fix,
both of which F3 and F7 rediscover from the other direction), `Spline-Jitter-Round-2-SD-Logging-
Regression.md`, and `End-Trial-Diagnosis-Correction.md`. See `README.md` for the full index.

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

### F1 — Synchronous SD I/O inside the 500 Hz loop on every controller change  🟡

> **CORRECTION (2026-08-12, same day).** An earlier revision of this document claimed this stall was
> **~2 s per joint**, caused by `while(param_file.available())` running a second iteration that hit
> Arduino `Stream`'s 1000 ms EOF timeouts in `parseInt()` and `readStringUntil()`. **That was wrong.**
> There is a `break;` at the end of the loop body (`ParamsFromSD.cpp:648`, comment "We don't need to
> read the rest of the file"), verified by brace-depth analysis, so the loop runs **exactly once**
> and EOF is never reached. No `Stream` timeout occurs anywhere on this path. The retracted figure
> should not be used.

What is actually true: `set_controller_params()` (`ParamsFromSD.cpp:512-651`) performs blocking SD
card I/O **synchronously inside the 500 Hz control loop**:

- `SD.open(filename)` — a FAT directory lookup and file open
- five `findUntil('\n','\n')` line skips plus ~16 `parseFloat()` calls, all byte-at-a-time through
  the `Stream` interface over a ~600-byte file
- one `readStringUntil('\n')`, which builds an Arduino `String` (heap allocation)
- `param_file.close()`

Order **milliseconds**, i.e. a few control cycles — not seconds. **Unmeasured; needs a scope or a
`micros()` bracket to quantify.** During it `Exo::run()` does not execute, so no CAN frames go out
and the AK60v3 holds its last torque command for that interval.

Triggered by: every controller change from the GUI (`update_controller_param` →
`set_default_parameters(id)`), and once per used joint at trial start (`update_status(trial_on)` →
`set_default_parameters()`, `uart_commands.h:208`).

Given the corrected magnitude, `a6413c9` (which removed the per-call `SD.begin()` remount) was
probably the substantive fix for the "motor hangs for a bit" symptom, and what remains here is a
much smaller residual. Note the same function still contains the genuine hang risk in F10.

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

### F5 — Spline lacks PJMC's uncalibrated-sensor guard  🟠  → **FIXED, see Part 5**

PJMC checks `if (_joint_data->torque_offset_reading == 0) cmd = filtered_setpoint;`
(`Controller.cpp:699`). **Spline does not.** It calls `_pid` unconditionally, and `_pid`
early-returns `cmd` when the sensor is uncalibrated (`Controller.cpp:210`). Since Spline does
`cmd = torque_cmd + _pid(...)`, the result is **exactly double the intended torque** — a −12 Nm
profile becomes −24 Nm. Reachable if the torque sensor is disconnected/reads ~0 V, or if
calibration collects zero samples.

### F6 — `BleParser` has no partial-message timeout, and re-queues a stale message  🟠

Two separate defects in `BleParser::handle_raw_data` (`BleParser.cpp:14-91`).

**(a) A partial multi-byte command wedges the parser.** `updateTorqueValues` sends 5 separate
**write-without-response** packets: `'f'`, then four 8-byte doubles (`QtExoDeviceManager.py:843`).
The codebase's own comment at `send_end_trial_sequence` documents that write-without-response
**hangs on WinRT under real-time-notification congestion**; an exception in the coroutine aborts
the rest of the sequence the same way. There is **no timeout** on a partial message, so if only
three doubles land, `_waiting_for_data` stays true with `_bytes_collected = 24` of an expected 32.

Every subsequent BLE write is then appended as *payload*, not interpreted as a command. Single-byte
commands are 1 byte each, so the **next eight** (`E`, `G`, `w`, `x`, `H`, `L`, `N`, `Z`) are
swallowed. Concretely: `send_end_trial_sequence` writes `'Z'`, `'G'`, `'w'` — all three are eaten,
**END TRIAL does nothing**, and the shutdown dialog sits until it reports TIMEOUT. Each dialog
Retry sends three more bytes; the wedge clears on its own after ~8 bytes, so the third or fourth
Retry gets through. That "it worked on the fourth try" signature is the fingerprint.

When the 32nd byte finally arrives the parser declares the message complete and decodes four
doubles: the first three (joint id, controller id, param index) are **real**, and the fourth — the
*value* — is eight ASCII command bytes reinterpreted as a double. Most such patterns overflow to
`inf` as a float and are rejected by `is_finite_float`, but not all: a `'$'` (0x24) in the top byte
yields a denormal that converts to **0.0f**, which passes validation. Because spline parameter
bounds are disabled (F7), that writes a silent **0** into whatever spline parameter was in flight.

An 8-byte write while wedged instead trips `_bytes_collected >= expecting * 8` and resets the
parser, so a following parameter update clears the wedge at the cost of garbling itself.

**(b) A non-completing write re-executes the previous command.** `return_msg` is `static` and is
only written when a message completes; `is_complete` is set true on the first completed message and
**never cleared**. Every call that does not complete a message returns that stale object, and
`ble_rx::on_rx_recieved` pushes it whenever `msg->is_complete` is true (`ExoBLE.cpp:466-476`), with
`ble_queue::push` taking a copy. So the `'f'` byte and doubles 1-3 of *every* parameter update
re-queue and re-execute the **previous** completed command four times (eight in bilateral mode).

Masked today only because the reachable handlers are idempotent (`start`, `stop`, `motors_on/off`,
`cal_fsr`, and a repeat of the identical previous `update_param`). It stops being masked the moment
a non-idempotent command becomes the last completed one — `mark` (`data->mark++`) is already one.

Related, same file: `ble_queue` is a **LIFO stack**, not a queue (`push` writes `queue[++m_size]`,
`pop` returns `queue[m_size--]`), and `ComsMCU::handle_ble` pops one per loop iteration — so a burst
of commands executes in reverse order.

### F7 — Node-order edits transiently kill assist; end-node torque is permanent  🟡

`_spline_interpolate` returns `0.0f` if x is not strictly increasing (`Controller.cpp:954`) —
fail-safe, but moving a node right requires editing in the correct order or assist silently
vanishes mid-trial with no message. Conversely `percent_gait <= x[0]` returns `y[0]` and
`>= x[4]` returns `y[4]`, so **a nonzero node1_y or node5_y becomes a constant torque applied
through all of swing and while standing still.**

**All spline parameter bounds are `enabled = false`** (`ControllerData.cpp:104-122`), unlike PJMC's,
so `validate_request` skips range checking entirely and the GUI spinbox allows ±100000. Only the
internal ±15 Nm clamp and the 25 Nm output clamp stand between a typo and a full-scale command.

### F8 — The error framework cannot detect anything, but pays for the attempt every cycle  🟡

> **CORRECTION (2026-08-12).** Two earlier revisions of this section were wrong, and the second was
> badly wrong. It claimed `TorqueVarianceError` **latches** and therefore floods the UART with
> ~1000 blocking messages/second, costing **~23 % of the control loop**. That was prompted for
> re-examination by an excellent objection: *if the Teensy→Nano link were really that saturated,
> the BLE stream should show obvious drops — and it doesn't.* Correct. **The latch never happens,
> because the check is mathematically incapable of firing.** The ~23 % figure, the ~1000 msg/s
> figure, and the claim that this feeds F3's D-term dropouts are all **retracted**. An earlier
> revision also claimed the error queue leaks; that was retracted separately and is confirmed below.

**Why `TorqueVarianceError` can never fire.** It tests whether the newest torque sample lies more
than `torque_std_dev_multiple = 10` **sample** standard deviations from the mean of a window that
**contains that same sample**. For a value inside its own sample of size *n*, using the sample sd
(`utils::online_std_dev` divides `M2` by `n-1`, `Utilities.cpp:372`), the maximum possible
studentized deviate is:

```
max |x_i - mean| / s  =  (n - 1) / sqrt(n)
n = torque_data_window_max_size = 100   ->   99 / 10  =  9.9
```

9.9 < 10 for **any input whatsoever**. Verified numerically against the exact transcribed algorithm:

| input | max &#124;z&#124; observed | fires? |
|---|---|---|
| quiet standing, ±2 ADC counts | 3.60 | no |
| walking transparency, ±3 Nm | 3.20 | no |
| quiet → the −12 Nm spline pulse (my claimed trigger) | 9.88 | **no** |
| 101 zeros then a +1000 Nm spike | 9.90 | no |
| pathological: 99 equal values + 1 huge, repeated | 9.90 | no |

Identical constants and divisor on `backup_branch_with_UW_edits`, so this has never fired on either
branch.

**What that means for the rest of the framework.** Five of the eight checks are hardcoded
`return false`. `MotorTimeoutError` needs `motor.timeout_count >= 40`, but the only code that would
increment it (`_CANMotor::_handle_read_failure`) is commented out. `TorqueVarianceError` is
impossible as shown. That leaves **`TorqueOutOfBoundsError` (|torque| > 60 Nm) as the only reachable
check**, and only on a railed or faulted torque sensor — never in normal walking.

So in normal operation: **zero errors fire, zero UART messages are sent, and there is no flood.**

**What it does still cost.** Every control cycle, per joint, the framework does 8 `std::map::at()`
lookups, 8 virtual `check()` calls, and — inside `TorqueVarianceError::check` — **two full copies of
a 100-element `std::queue<float>`** (`online_std_dev` takes its argument by value *and* copies it
again internally, `Utilities.cpp:344,346`) plus a 100-iteration Welford pass with a float divide per
iteration. Across two ankles that is **~4000 heap-allocating deque copies per second** inside a hard
real-time loop, to reach a conclusion that can never change. Order **~1 % of the 2000 µs cycle —
estimated, not measured.** Real waste and a heap-fragmentation risk, but not a jitter driver.

Note `_CANMotor::check_response` calls the same `online_std_dev` on a 25-element queue every cycle
per motor. That is **live logic** (it re-enables a motor whose measured current has gone static) and
is *not* covered by the Part 4 disable, so roughly half the deque churn remains.

**Confirmed (not retracted):** the error queue does **not** leak. The half-draining loop
`for (i = 0; i < errorQueueSize(); i++) pop()` leaves `floor(S/2)` entries, so with `k` errors per
cycle the steady state `R = floor((R+k)/2)` converges to `R ≈ k`. Measured on the host with a forced
`k = 8`: converges to 7, peaks at 15. Still a defect (bursts are reported with a lag), not a leak.

**Confirmed:** all handlers have `motor.enabled = false` commented out, so a detected error takes no
protective action, and `ComsMCU::handle_errors` only forwards on a *change* of code — one BLE
notification per trial at most, to a signal nothing consumes.

**The latent hazard is real even though the current cost is not.** If someone "fixes" the detector
by lowering `torque_std_dev_multiple` to something reachable without fixing the rest, everything the
retracted revision described happens for real: `torque_failure_count` is never reset so the error
latches permanently, `Joint.cpp` reports once per joint per cycle, and each report ends in
`UARTHandler::UART_msg` → `MY_SERIAL.flush()`, which **spins until the UART shift register drains**
(~234 µs per 6-byte SLIP frame at 256000 baud 8N1) → ~469 µs per 2000 µs cycle across two ankles,
and ~1000 msg/s into a coms MCU whose `ComsMCU::update_UART` consumes at most **one message per
1000 µs**. That is what the Part 4 switch and its checklist exist to prevent.

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
| High | **Commanded torque is probably being silently clamped on every step** (F2). `cmd = ff + Kp·(ff − measured)` with `Kp = 6` multiplies any tracking error by 7, and the shipped profile slews at ~155 Nm/s. The 25 Nm clamp truncates the result and reports only via a bare `Serial.print` nobody sees. This is a safety item *and* a data-validity item: the profile you think you are commanding would not be the one the ankle receives. **Untested prediction — bench-check it.** |
| ~~High~~ → Low | ~~**~2 s bilateral freeze on controller change while walking** (F1).~~ **Downgraded with the F1 correction:** the stall is milliseconds, not seconds. The AK60v3 does hold its last command for that window, but a few control cycles is not a fall hazard. |
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
| B23 | F8 error framework inert but costly | **PRESENT** | Identical constants (`torque_std_dev_multiple = 10`, `torque_data_window_max_size = 100`) and the same `M2/(n-1)` divisor, so `TorqueVarianceError` has never been able to fire on either branch. *(Row revised with F8 — the "latches and floods the UART" description applied to neither branch.)* |
| B24 | Error handlers neutered (`motor.enabled = false` commented out) | Present | Identical. |
| B25 | `status_defs::messages::error` never set | Present | Identical. |

## 2.4 SD / parameters

| # | Finding | UW branch | Notes |
|---|---|---|---|
| B26 | F1 synchronous SD I/O in the control loop | **PRESENT** | Identical loop structure, including the `break` that bounds it. *(Row revised with F1 — the retracted ~2 s figure applied to neither branch.)* |
| B27 | `SD.begin()` remount on every controller change | **PRESENT** (worse) | 6 call sites on UW; ours caches via `_sd_ready()`. UW's controller-change stall is therefore the SD read **plus** a full card remount — so on UW this is the dominant cost, and it is what `a6413c9` removed on our branch. |
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

**Not fixed by this change** (unchanged, still open): F1 SD I/O in the loop, F2 clamp saturation,
F3 D-term noise, F4 gain-schedule discontinuity, F5 torque doubling, F6 BleParser wedge/stale
re-push, F7 disabled bounds, F9 const map, F10 EOF spin, and every item in §1.3 except the logging
ones. **F8 is addressed separately in Part 4.**

## 3.6 What to check on first hardware run

1. Flash **both** boards. The fix spans Teensy and Nano.
2. Confirm the GUI plots and CSV populate. Expect **12 data columns**, ending with
   `Commanded Torque (L)`, `Commanded Torque (R)`, `Status`, `Exoskeleton time (seconds)`.
3. Confirm the plot x-axis advances smoothly (exo clock) rather than in bursts.
4. With the joint clamped and unworn, run a single spline step and watch for `TORQUE CLAMP:` on
   serial — F2 predicts it will fire.
5. Bracket `set_controller_params()` with `micros()` and log the delta once, to put a real number on
   F1 (expected: milliseconds).
6. Instrument the actual loop period (`delta_t` in `Exo::run`) and count cycles over 2200 us. This
   is now the **open question**, not a confirmation: F8 turned out to be inert, so whatever is
   causing F3's D-term dropouts has not been found yet. Candidates still on the table are F1's SD
   I/O, the surviving `check_response` deque copies, and plain superloop variance.

---

# PART 4 — Disabling the error manager / error reporter (F8)

## 4.1 Decision

F8 is disabled rather than repaired, behind a single switch:
**`ERROR_MANAGER_ENABLED` in `Config.h` (currently `0`)**.

The rationale, written out in full at the define itself:

- **It protects nothing.** Every handler in `error_types.h` has its `motor.enabled = false` line
  commented out, so a detected error takes no action.
- **It cannot detect anything either.** Five of the eight checks are hardcoded `return false`;
  `MotorTimeoutError` is unreachable; `TorqueVarianceError` is mathematically incapable of firing
  (10 σ threshold against a 9.9 σ ceiling — see the corrected F8). Only `TorqueOutOfBoundsError`
  can fire, and only on a railed/faulted torque sensor.
- **Nothing receives the output.** The chain ends at `deviceErrorReceived`, which `MainWindow` only
  connects inside the `if remote.is_bound():` block. With no UDP subscriber attached the signal is
  dropped. It is not in the UI, not in the CSV, not even in the GUI log file.
- **It still costs every cycle.** ~4000 heap-allocating deque copies/second plus 100-iteration
  Welford passes, to reach a conclusion that can never change. Estimated ~1 % of the loop — small,
  but pure waste, and heap churn inside a hard real-time loop.
- **It is a loaded gun.** Lowering the σ threshold without fixing the rest turns it into ~1000
  blocking UART messages/second and ~23 % of the control loop (see the end of F8). The switch plus
  §4.5's checklist is what stops that from being a one-line mistake.

The real torque ceiling — `MAX_JOINT_TORQUE_NM` enforced in `_CANMotor::send_data()`, together with
its non-finite rejection — is **untouched** and remains the actual protection. Disabling this
framework removes no safety behaviour, because it had none.

## 4.2 Implementation

Purely additive: **82 insertions, 0 deletions.** No existing line was modified or removed; the
original logic is intact inside `#else` branches, so re-enabling is a one-character change.

| File | Change |
|---|---|
| `Config.h` | new `#define ERROR_MANAGER_ENABLED 0` with the full rationale and a re-enable checklist |
| `ErrorManager.h` | `run()` early-returns `false` when disabled, compiling out all eight checks; includes `Config.h`; fail-closed `#ifndef` fallback |
| `ErrorReporter.h` | `report()` early-returns when disabled (second gate, in case a new caller appears); same include + fallback |

Short-circuiting `ErrorManager::run()` is safe because every field the checks touch —
`smoothed_motor_torque`, `torque_error`, `torque_data_window`, `torque_failure_count` — is written
**and read only inside the check that owns it**. Verified by grepping each one across the whole
firmware: no external consumer. `motor.timeout_count` is only ever set to `0` elsewhere
(`Motor.cpp:205,228`) and read in `enable()` and the SD logger, so nothing depended on
`MotorTimeoutError` clearing it.

## 4.3 Verification

The **real edited headers** were copied verbatim into a stub environment and compiled + executed on
the host with `g++ -std=gnu++17 -fpermissive -Wall -Wextra`, driven for 5000 iterations (10 s at
500 Hz) exactly the way `run_joint()` drives them.

Disabled (`ERROR_MANAGER_ENABLED 0`) — compiles with **no diagnostics**:

```
  PASS  run() reports no error
  PASS  no checks executed (JointData untouched)
  PASS  torque_failure_count never incremented
  PASS  error queue stayed empty
  PASS  no ErrorReporter::report() calls
  PASS  no blocking UART sends
  PASS  direct report() call is also a no-op
  PASS  popError() unreachable while disabled
```

Re-enabled (`ERROR_MANAGER_ENABLED 1`) — also compiles with no diagnostics, proving the switch is
usable and the `#else` branch was not broken:

```
  PASS  checks DID execute when enabled
  PASS  reports happened when enabled / UART sends happened when enabled
  INFO  8 errors pushed per cycle over 5000 cycles
  INFO  peak queue length: 15,  final queue length: 7
  INFO  reports: 39993  (8.00 per cycle)
  PASS  queue is BOUNDED, not a leak
```

That last result is also the empirical confirmation of the F8 correction: with `k = 8` errors per
cycle the queue converges to 7 and peaks at 15 (= `R + k`), exactly as `R = floor((R+k)/2)`
predicts. It does **not** leak.

The Part 3 real-time-stream harnesses were re-run afterwards and still pass, confirming no
cross-contamination between the two changes.

## 4.4 End-to-end re-analysis — every path this touches

| Path | Effect | Verdict |
|---|---|---|
| Teensy boot | `error_map` still constructs its 8 objects at static init (unchanged, now unused — a few dozen bytes) | no change |
| `run_joint()` x6 joints | `error` is always `false`; the `if (error)` body is dead code | no change in behaviour; `popError()` on an empty queue (which would be UB) is now unreachable, i.e. **safer** |
| Teensy 500 Hz loop | ~4 of the ~8 deque copies/cycle removed, plus two 100-iteration Welford passes and 16 map lookups + 16 virtual calls | **small improvement**, estimated ~1 % of the cycle. The other ~4 copies/cycle are `_CANMotor::check_response`, which is live logic and stays. **No** measurable jitter change should be expected — the retracted ~23 % figure was wrong. |
| Teensy -> Nano UART | **no change** | The framework was never sending anything, because no check can fire. The earlier claim that error spam was saturating the Nano's intake (which caps at one message per 1000 us) is **retracted** — that is a hazard of *re-enabling* naively, not a description of today. |
| Nano `handle_errors()` | `_data->error_code` stays `NO_ERROR`, so the `!=` guard never fires and no BLE error notification is sent. `ExoBLE::setup()`'s one-time `send_error(0,0)` at boot is unchanged | no change |
| GUI `_on_error` / `deviceErrorReceived` | never fires | **no observable change** — it was already connected only inside the remote-bound block and had no UI, CSV or log consumer |
| SD logger | logs `motor.timeout_count`, which is `0` either way; logs none of the error-check fields | no change |
| Real-time stream (Part 3) | separate code path entirely | unaffected; harnesses re-run and pass |
| Torque clamp / NaN rejection | untouched | still active |

**Conclusion: no functional regression.** The change removes work that could never produce an output
at all, let alone one anybody consumed. The only behavioural delta is a small reduction in per-cycle
computation and heap churn; **do not expect a visible improvement**, and if loop-timing jitter does
change noticeably after this, that means something else was going on and is worth chasing.

## 4.5 Before re-enabling

Do not set `ERROR_MANAGER_ENABLED` back to `1` until all of these are true:

1. Handlers take a real action (or the check is removed).
2. `torque_failure_count` is reset, so `TorqueVarianceError` stops latching.
3. `utils::online_std_dev` stops taking its queue by value and copying it again internally.
4. Reporting is non-blocking or rate-limited — `UARTHandler::UART_msg` ends in a spinning
   `MY_SERIAL.flush()`, and the Nano can only consume ~1000 messages/second.
5. The half-draining loop in `Joint.cpp` (`for (i = 0; i < errorQueueSize(); i++) pop()`) is fixed.
6. `deviceErrorReceived` is wired to something a human actually sees.

A genuine torque cutout would be better written as a direct check with a direct action than routed
through this framework.

---

# PART 5 — F5 fix: uncalibrated-torque-sensor guard on the Spline controller

## 5.1 The defect

`_Controller::_pid` bails out early when the torque sensor is uncalibrated, and it returns **its
setpoint argument**, not a PID contribution (`Controller.cpp:210`):

```cpp
if (_joint_data->torque_offset_reading == 0) {
    return cmd;                       // <- returns the SETPOINT
}
```

PJMC and PJMC_PLUS wrap the call to account for that (`Controller.cpp:713`, `:3036`). **Spline did
not**, so `cmd = torque_cmd + _pid(torque_cmd, ...)` evaluated to `torque_cmd + torque_cmd` —
**exactly double the intended feed-forward**, silently. The GUI's "Desired Torque" channel would
still read the single value, because that is `ff_setpoint`, so nothing on screen would show it.

`torque_offset_reading` is `TorqueSensor::_calibration`, which starts at 0 and is written only when
the timed calibration completes. The GUI workflow makes this hard to reach — Start Trial is gated
behind Calibrate Torque plus a 3 s settle in `ScanPage.on_calibrate_torque` — so the realistic route
in is **a disconnected or dead sensor reading ~0 V**, or a calibration window that collected no
samples. Low probability, cheap guard.

## 5.2 The change

One `if`/`else` around the existing expression in `Spline::calc_motor_cmd`, matching PJMC's shape.
Functional diff is 5 lines replaced by the same 5 lines wrapped:

```cpp
if (_joint_data->torque_offset_reading == 0)
{
    cmd = torque_cmd;                 // open-loop fallback, same as PJMC
}
else
{
    cmd = torque_cmd + _pid(torque_cmd, _controller_data->filtered_torque_reading,
                            kp_use, ki_use, kd_use);
}
```

Structural checks: brace delta identical before and after (+2, pre-existing, from braces inside
comments), preprocessor directives balanced 30/30.

## 5.3 Verification

Modelled `_pid` and the Spline PID branch exactly, driven over a full stride of the shipped profile:

| case | result |
|---|---|
| **calibrated** sensor (`offset_reading = 1.19`), full stride | max &#124;before − after&#124; = **0.0000000000** — behaviour bit-identical |
| **uncalibrated**, at the profile peak (setpoint −12.0 Nm) | was **−24.0 Nm**, now **−12.0 Nm** |
| **uncalibrated**, at 5 % gait (setpoint −7.85 Nm) | was −15.71 Nm, now −7.85 Nm |
| `use_pid = 0` branch | untouched |

## 5.4 Still unguarded elsewhere — NOT fixed here

The same `setpoint + _pid(setpoint, ...)` pattern exists in six other controllers that have **no**
guard: `ZhangCollins` (`:785`), `FranksCollinsHip` (`:1153`), `ConstantTorque` (`:1292`),
`ElbowMinMax` (`:1579`), `Step` (`:1929`) and `SPV2` (`:2847`, `:2852`). `ZeroTorque` (`:338`) has
it too but is harmless there — its setpoint is 0, so `0 + 0 = 0`.

Guarded today: `ProportionalJointMoment`, `PJMC_PLUS`, and now `Spline`.

Changing `_pid()` to return `0` instead of `cmd` would fix all of them at once and is arguably the
right fix, but it alters behaviour for every controller in the codebase and was deliberately left
out of scope. Flagged for a separate decision.
