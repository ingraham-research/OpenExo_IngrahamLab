# Stable Transparent Zero-Torque (ZeroTorque) — Design

**Date:** 2026-07-10
**Branch:** `fix_zerotorque_pid`
**Status:** Approved design, pending implementation

## Problem

The user wants "zero torque" on the ankle to mean **transparency** — the motor actively
overcomes its own friction/inertia so the wearer feels minimal impedance — not "motor
does nothing." Two defects block this today:

1. **Wrong controller runs at trial start.** `start_trial` (`ble_commands.h` `ble_handlers::start`)
   only enables the motor; it never selects a controller. The ankle therefore stays on the
   `config.ini` default `ankleDefaultController = PJMC`. With `PJMC.csv` setpoints at 0, this is
   a torque-nulling PID at zero reference whose gain schedule chatters ±5 Nm. Confirmed against
   the GUI logs in `Python_GUI/saved_data`: at every trial start Desired Torque is flat 0 while
   Measured Torque oscillates at >=28 Hz (aliased) with the foot unloaded — a motor-driven limit
   cycle, gated by the 3.5 Nm gain-schedule threshold.

2. **The actual ZeroTorque controller freewheels.** Shipped `zeroTorque.csv` is `0,0,0,0`
   (`use_pid=0`), so `ZeroTorque::calc_motor_cmd()` returns exactly 0. With the motor MIT-mode
   gains all zero (`MotorData`: `kp=kd=p_des=v_des=0`), that commands zero current. The wearer
   feels the full drivetrain impedance (Coulomb + viscous friction, cogging, and rotor inertia
   reflected through ~27:1 ankle gearing). That is the opposite of transparency.

## Key insight

PJMC-with-setpoint-0 reduces every cycle to a torque-nulling PID:

```
cmd = -kp * tau_measured - kd * d(tau_measured)/dt      (i_gain = 0)
```

Its **gain schedule** (`GS_Flag=1`) selects gains by |measured torque|:

| Measured torque | kp | kd | Behavior           |
|-----------------|----|----|--------------------|
| \|tau\| <= 3.5 Nm | 3  | 0.001 | gentle, quiet — feels good |
| \|tau\| >  3.5 Nm | 6  | 0.03  | 2x kp, 30x kd on raw derivative — chatters |

The transparency the user likes **is** the low-gain regime. The chatter is only the high-gain
regime, entered when a disturbance pushes torque past 3.5 Nm. This is corroborated by the logs
(quiet below 3.5, violent above) and by the user's subjective report.

`ZeroTorque(use_pid=1)` computes the identical law via the same `_pid()` function **and has no
gain schedule**. So running ZeroTorque with the low-regime gains reproduces the good feel and is
structurally incapable of entering the chattering regime — there is no threshold to cross.

## Design

Reproduce PJMC-0's low-gain regime in the schedule-free `ZeroTorque` controller.

- **Control law:** unchanged. `ZeroTorque(use_pid=1)` already yields `cmd = -kp*tau - kd*dtau/dt`.
- **Gains (shipped default):** `use_pid=1, p_gain=3, i_gain=0, d_gain=0.001`.
- **No torque filter.** PJMC-0's good regime runs on the raw torque signal with `kd=0.001`
  (~0.02 Nm command jitter per ADC count). Adding an EWMA filter would change the feel the user
  already approved. A defensive internal filter is explicitly deferred (see Out of Scope).
- **Toggle:** the existing `use_pid` parameter is the transparent/do-nothing switch —
  `use_pid=1` = transparent, `use_pid=0` = freewheel. Editable live from the Python GUI
  *Update Controller Settings* page (Ankle -> zeroTorque -> use_pid -> 0/1). The firmware accepts
  ZeroTorque parameter writes today (its bounds are `enabled=false`, so `validate_request` skips
  range checks and accepts). No new parameter, no bounds changes.

### Changes

1. **`SDCard/ankleControllers/zeroTorque.csv`** — line 6 values `0,0,0,0` -> `1,3,0,0.001`.
   Ankle only; other joints' `zeroTorque.csv` remain `0,0,0,0` (freewheel), since these gains are
   specific to the AK60v3 + this torque sensor + 4.5:1 external gearing.

2. **`ExoCode/src/Controller.cpp`, `ZeroTorque::calc_motor_cmd()`** — write
   `_controller_data->filtered_torque_reading = _joint_data->torque_reading;` so the streamed
   torque plot (`uart_commands.h`) and SD log (`SdLogger.cpp`) show measured torque. It currently
   never writes this field, so the trace would otherwise freeze at its last value. `desired_torque`
   stays 0 (correct for zero torque).

3. **`SDCard/config.ini`** — `ankleDefaultController = PJMC` -> `zeroTorque`, so a trial boots into
   transparency. PJMC and all other controllers remain selectable from the GUI (the handshake still
   lists every controller whose joint default is not "disabled").

4. **(Optional, low risk) `ExoCode/src/ble_commands.h`** — the `start` handler comment claims it
   "sets the controller to zero torque" but does not. Either delete the misleading clause or leave
   controller selection to the config default (item 3). No behavioral code change proposed here;
   comment fix only.

### Safety (existing behavior, relied upon)

- **Uncalibrated torque sensor -> freewheel.** `_pid()` returns its `cmd` argument (0 for
  ZeroTorque) when `_joint_data->torque_offset_reading == 0`, so an uncalibrated/failed sensor
  cannot drive the closed loop.
- **`use_pid=0` remains freewheel**, unchanged and available as the explicit "motor do nothing"
  mode.

## Validation (user-run bench test)

Stability of `kp=3` **above** 3.5 Nm is the one regime the logs cannot show (the schedule always
switched away from kp=3 there). Lower gain is more stable than the kp=6 that chattered, so this is
expected to be fine, but must be confirmed on hardware. Assistant cannot run motor tests.

Procedure:
1. Flash firmware with changes; keep `SDCard` files updated on the card.
2. Connect GUI, start a trial. Confirm ankle boots into `zeroTorque`, `use_pid=1`.
3. With the exo on a bench/leg, backdrive the ankle by hand through its full range, including
   deliberately pushing past ~3.5 Nm measured torque.
4. Watch the Measured Torque plot and feel for chatter. Success = smooth, low residual torque,
   no sustained oscillation at any torque level.
5. If sluggish (too much residual impedance), raise `p_gain` live in small steps (e.g. 3 -> 4 -> 5)
   and note where transparency is best without chatter.
6. If any chatter appears, lower `p_gain`/`d_gain` live.
7. Bake the best validated `(use_pid, p_gain, i_gain, d_gain)` set into
   `SDCard/ankleControllers/zeroTorque.csv` as the final shipped default this session.

## Out of scope (this session)

- Defensive internal EWMA torque filter (revisit only if bench shows noise issues).
- Enabling the ZeroTorque bounds table (`ControllerData.cpp` `zero_torque_bounds`) — parameters
  stay GUI-exposed and unbounded as they are now.
- Transparency defaults for non-ankle joints.
- Reworking `start_trial` to force-select a controller each trial (config default is sufficient;
  user's GUI selection persists intentionally).

## Risks

- `kp=3` above 3.5 Nm unverified (mitigated: lower gain than the chattering kp=6; bench check).
- Changing the ankle default controller alters boot behavior — intended, and PJMC stays available.
- `d_gain=0.001` on raw torque relies on the low value keeping derivative noise small, as in
  PJMC-0's low regime; bench test confirms.
