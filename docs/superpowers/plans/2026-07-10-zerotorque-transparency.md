# ZeroTorque Transparency Implementation Plan

> **For agentic workers:** Steps use checkbox (`- [ ]`) syntax for tracking. This is Teensy
> firmware; there is no unit-test harness for the controller path, so verification is
> compile/static review plus a user-run bench test (the user runs all motor tests). Per the
> user's standing rule, the assistant does NOT commit — the user reviews and commits.

**Goal:** Make the ankle "zero torque" a stable transparency controller (motor overcomes its own
friction) by running the schedule-free `ZeroTorque` PID at PJMC-0's low-gain regime, and make it
the controller that runs at trial start.

**Architecture:** Reuse `ZeroTorque(use_pid=1)` — which already computes `cmd = -kp*tau - kd*dtau/dt`
with no gain schedule — and ship the low-regime gains `(kp=3, ki=0, kd=0.001)` that feel good in
PJMC-0. Fix the plotting field it fails to write, and set it as the ankle default controller.

**Tech Stack:** Teensy 4.x C++ (Arduino), SD-card CSV/INI config, Python GUI over BLE/UART.

## Global Constraints

- Ankle actuator/sensor only. Gains are specific to AK60v3 + this torque sensor + 4.5:1 external
  gearing. Do NOT touch other joints' `zeroTorque.csv`.
- Keep ZeroTorque's four GUI-exposed params (`use_pid, p_gain, i_gain, d_gain`) as-is. No new
  parameter, no bounds-table changes this session.
- No torque filter added this session (deferred; would change the approved feel).
- Assistant does not run motor tests and does not commit.
- The `use_pid` toggle semantics must be preserved: `use_pid=0` = freewheel ("motor do nothing"),
  `use_pid=1` = transparent.

---

### Task 1: Write the ZeroTorque plotting field so the torque trace is live

**Files:**
- Modify: `ExoCode/src/Controller.cpp` — `ZeroTorque::calc_motor_cmd()` (currently ~lines 313-336)

**Interfaces:**
- Consumes: `_joint_data->torque_reading` (float, signed, side-corrected), `_controller_data`
  (`ControllerData*`).
- Produces: `_controller_data->filtered_torque_reading` is set every cycle so
  `uart_commands.h` (GUI stream) and `SdLogger.cpp` show measured torque.

- [ ] **Step 1: Read the current function**

Confirm current body of `ZeroTorque::calc_motor_cmd()`. It sets `ff_setpoint` and `desired_torque`
to `cmd_ff` (0) but never writes `filtered_torque_reading`.

- [ ] **Step 2: Add the plotting write**

In `ZeroTorque::calc_motor_cmd()`, after the PID block and before `return cmd;`, add:

```cpp
    //Report the measured torque for GUI streaming and SD logging.
    //No filtering here: mirrors PJMC-0's low-gain regime (raw torque), which is the
    //behavior being reproduced. (See docs/superpowers/specs/2026-07-10-zerotorque-transparency-design.md)
    _controller_data->filtered_torque_reading = _joint_data->torque_reading;
```

Leave `_controller_data->desired_torque = cmd_ff;` (0) unchanged — desired torque is 0 for zero
torque, which is correct.

- [ ] **Step 3: Static verification**

Confirm the field exists: `_controller_data->filtered_torque_reading` is declared in
`ControllerData.h` (it is, ~line 287) and written the same way by other controllers
(e.g. PJMC at `Controller.cpp` ~line 610). Confirm no other change to the return value.

- [ ] **Step 4: (User) compile**

User builds the firmware (Arduino/PlatformIO) for the Teensy target and confirms it compiles with
no new warnings in `Controller.cpp`. Assistant cannot build the Teensy target here.

- [ ] **Step 5: Commit (USER)**

Assistant does not commit. User reviews the diff and commits if satisfied.

---

### Task 2: Ship the transparent default gains for the ankle ZeroTorque

**Files:**
- Modify: `SDCard/ankleControllers/zeroTorque.csv` — line 6 (values row)

**Interfaces:**
- Consumes: nothing. Read at boot / on controller selection by `ParamsFromSD` into
  `controller.parameters[0..3]` = `use_pid, p_gain, i_gain, d_gain`.
- Produces: default parameter set `use_pid=1, p_gain=3, i_gain=0, d_gain=0.001` for
  ankle ZeroTorque.

- [ ] **Step 1: Confirm current contents**

Current file:
```
5,"header Size, the first N rows will be ignored, except for this first cells in the first two rows",,
4,"parameter number, the number of parameters to read per line",,
,Parameter list for the ZeroTorque controller,,
,Parameter order: ,,
use_pid,p_gain,i_gain,d_gain
0,0,0,0
```

- [ ] **Step 2: Change the values row (line 6)**

Replace `0,0,0,0` with `1,3,0,0.001`. Leave lines 1-5 unchanged (still 4 params, names row intact).

- [ ] **Step 3: Static verification**

Confirm: header row 1 still says 5 header rows; row 2 still says 4 params; line 5 names unchanged;
line 6 now `1,3,0,0.001`. The value order matches `controller_defs::zero_torque`
(`use_pid_idx=0, p_gain_idx=1, i_gain_idx=2, d_gain_idx=3`).

- [ ] **Step 4: Commit (USER)**

Assistant does not commit. This is an SD-card file; user copies it to the physical SD card before
testing. No reflash needed for CSV changes.

---

### Task 3: Make ZeroTorque the ankle controller that runs at trial start

**Files:**
- Modify: `SDCard/config.ini` — `ankleDefaultController`
- Modify (optional, comment only): `ExoCode/src/ble_commands.h` — `ble_handlers::start`

**Interfaces:**
- Consumes: parsed by `ParseIni` via the `ankle_controllers` key map
  (`{"zeroTorque", ...zero_torque}`).
- Produces: ankle boots into `zeroTorque`; `start_trial` runs it.

- [ ] **Step 1: Change the ankle default controller**

In `SDCard/config.ini`, change:
```
ankleDefaultController = PJMC
```
to:
```
ankleDefaultController = zeroTorque
```
Leave all other joints and fields unchanged. `zeroTorque` is a valid key in the `ankle_controllers`
map in `ParseIni.h`.

- [ ] **Step 2: Static verification — handshake still lists all controllers**

Confirm the GUI will still see every ankle controller: the `ListCtrlParams` gate includes a joint
when its default controller id `> 1`. `zero_torque = 2 > 1`, so the ankle is still included and PJMC
remains selectable in the GUI. No change needed there.

- [ ] **Step 3: (Optional) Fix the misleading start-handler comment**

In `ExoCode/src/ble_commands.h`, `ble_handlers::start`, the comment claims it "set the controller to
zero torque" but the code only enables the motor. Either remove that clause or reword to:
```cpp
        //Start the trial (enable motors and begin streaming data). Controller selection is
        //governed by the config.ini default / the last GUI selection; this handler does not
        //change the controller.
```
No behavioral code change. Skip if you prefer to leave it.

- [ ] **Step 4: Commit (USER)**

Assistant does not commit. `config.ini` is an SD-card file (no reflash); the `ble_commands.h`
comment change, if made, requires a reflash but is comment-only.

---

### Task 4: Bench validation (USER-run) and final default lock-in

**Files:**
- Possibly modify: `SDCard/ankleControllers/zeroTorque.csv` (final tuned values)

- [ ] **Step 1: Deploy**

User updates the SD card with the new `config.ini` and `ankleControllers/zeroTorque.csv`, and
flashes firmware containing the Task 1 change.

- [ ] **Step 2: Sanity check controller selection**

Connect GUI, start a trial. Confirm ankle is running `zeroTorque` with `use_pid=1`, and the Measured
Torque trace is live (not frozen) — verifies Task 1.

- [ ] **Step 3: Transparency + stability sweep**

With the exo on a bench/leg and the torque sensor calibrated, backdrive the ankle by hand through
full range, deliberately pushing measured torque past ~3.5 Nm. Success: smooth, low residual torque,
no sustained oscillation at any torque level (this is the kp=3-above-3.5-Nm regime the logs could not
cover).

- [ ] **Step 4: Live-tune if needed**

- If sluggish (too much residual impedance): raise `p_gain` in small steps (3 -> 4 -> 5) from the GUI
  *Update Controller Settings* page (Ankle -> zeroTorque -> p_gain).
- If any chatter: lower `p_gain` and/or `d_gain`.
- Toggle `use_pid` 1 <-> 0 to confirm transparent vs freewheel both behave.

- [ ] **Step 5: Lock in the final default**

Write the best validated `(use_pid, p_gain, i_gain, d_gain)` back into
`SDCard/ankleControllers/zeroTorque.csv` line 6 as the shipped default. User commits.

---

## Self-Review

**Spec coverage:**
- Control law (ZeroTorque use_pid=1, no schedule) — no code change needed; documented in Task 1/2. ✓
- Default gains `1,3,0,0.001` — Task 2. ✓
- No filter — Global Constraints + Task 1 comment. ✓
- `use_pid` toggle via GUI — Global Constraints; verified in Task 4 Step 4. ✓
- `filtered_torque_reading` write — Task 1. ✓
- `ankleDefaultController = zeroTorque` — Task 3. ✓
- Misleading start comment — Task 3 Step 3 (optional). ✓
- Safety (uncalibrated -> freewheel; use_pid=0 freewheel) — existing behavior, noted; nothing to
  implement. ✓
- Validation bench procedure — Task 4. ✓
- Out of scope (filter, bounds, other joints) — Global Constraints. ✓

**Placeholder scan:** No TBD/TODO; each code/config step shows exact content. ✓

**Type consistency:** `filtered_torque_reading` (float) matches `ControllerData.h`; param indices
match `controller_defs::zero_torque`. ✓
