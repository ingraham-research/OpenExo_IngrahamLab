# End-Trial motor lock-up: the previous root cause is WRONG. Full re-derivation.

**Date:** 2026-08-10
**Status:** The previous diagnosis is **disproven**. A replacement root cause is **NOT established** —
this document says so plainly rather than substituting another chain. Two defensive fixes were
implemented that make the failure class impossible regardless of which mechanism was responsible.
**Code changed:** `Motor.cpp` (`send_data`, `read_data`), `uart_commands.h` (`get_system_reset`),
`Config.h` (new flag). **Not compiled, not flashed, not tested.**
**Supersedes:** `superseded/End-Trial-Malformed-Enable-Frame-Right-Ankle-Damage.md` (root cause
section) and Part 1 of `superseded/Branch-Comparison-End-Trial-Regression-And-Must-Keep-Edits.md`.
Both were moved into `superseded/` on 2026-08-12; **Part 2 of the branch-comparison doc is still
current** (the must-keep vs optional edit tiers) — see `README.md`.

---

## 1. What was previously claimed

Both earlier documents concluded: at End Trial, `'w'` (motors_off) can set `motor.enabled = 0` while
the exo status is still an active trial; `_CANMotor::check_response()` then sees a collapsed
measured-current variance, re-enables the motor, and calls `enable(true)`, which transmits
`FF FF FF FF FF FF FF FC` on the AK60v3's *command* CAN id — decoded by the motor as
kp=500 / kd=5 / p_des=+12.5 rad / i_ff=+10.3 A, a ~51 Nm max-gain position slam.

## 2. Why that is impossible

`check_response()` (`Motor.cpp:333`) opens with:

```cpp
if (_data->user_paused || !active_trial || _data->estop || _error) { return; }
```

and the Teensy's handler for the message `'w'` sends is:

```cpp
inline static void update_motor_enable_disable(UARTHandler*, ExoData* exo_data, UART_msg_t msg)
{
    exo_data->for_each_joint([](JointData* j, float* a){ if (j->is_used) j->motor.enabled = (bool)a[0]; },
                             msg.data);
    exo_data->user_paused = !(bool)msg.data[0];      // <-- SAME handler, SAME message
}
```

with `UART_command_enums::motor_enable_disable::ENABLE_DISABLE = 0`, so `msg.data[0] == 0`
(disable) ⇒ `user_paused = true`.

**`enabled = 0` and `user_paused = true` are set atomically, in one handler, from one UART message.
There is no window to race.** `'w'` cannot arm `check_response()`. Neither can anything else in the
End Trial sequence:

| command | sets `enabled = 0` | what closes `check_response()` | atomic? |
|---|---|---|---|
| `'w'` motors_off | yes | `user_paused = true` | yes — same handler |
| `'G'` stop | yes | `trial_off` **and** `user_paused = true` | yes — and status is sent first |
| `'Z'` reset (`get_system_reset`) | yes | `trial_off` | yes — same handler |

This is identical on `backup_branch_with_UW_edits`. Both branches are protected, by the same
upstream interlock. **The `'w'` vs `'G'` race that both documents were built on does not exist.**

### Exhaustive enumeration of every `enabled = false` writer on the Teensy

| location | also closes `check_response()`? |
|---|---|
| `ExoCode.ino:455-497` | sets `true` at boot only |
| `uart_commands.h` `update_motor_enable_disable` | **yes** — `user_paused` |
| `uart_commands.h` `get_system_reset` | **yes** — `trial_off` |
| `Exo.cpp:86` (estop branch) | **yes** — `estop`; also dead code (`data->estop = 0` is hardcoded above it) |
| `error_types.h:37` `TestError::handle` | dead — its `check()` returns `false` unconditionally |
| `Joint.cpp:708/932/1169/1406/1530/1648` — the **`disabled` controller** case in `set_controller()` | **NO** |

So exactly one live path can arm the trigger: the joint being set to the `disabled` controller
(`ankle_controllers::disabled = 1`). And `run_joint()` calls `set_controller(controller.controller)`
**every control cycle** with no change-guard, so while that id is selected the trigger would re-arm
and fire at 500 Hz continuously — a violent, immediate, permanent fault.

That is **inconsistent with the observed behaviour**: the exo was reported to work perfectly
throughout the trial, with the event beginning only on the End Trial click. So this is not what
happened either.

**Conclusion: the malformed enable frame was not sent, and the root cause is unknown.**

## 3. Also retracted: the branch-difference explanation

`Branch-Comparison...md` Part 1 argued that the raw branch's 5000 ms Nano reset delay
(`ComsMCU::_reset_delay_ms`) gave ~2500 control cycles in which a normal frame would overwrite the
malformed one, whereas our ~6 ms reset guaranteed silence.

With no malformed frame, that argument is moot. Re-tracing the nominal End Trial on both branches:

| cycle | both branches |
|---|---|
| N | normal frame; UART: `update_status(trial_off)` |
| N+1 | frame computed under `trial_off`, still `enabled`; UART: `update_motor_enable_disable(0)` |
| N+2 | `enabled==0`, `_prev==true` → **one-shot zero frame**; UART: reset (ours) / nothing (raw) |
| N+3… | silence, then reboot (ours ~6 ms, raw ~5 s) |

**Both branches end with the zero frame as the last frame on the bus.** The reset-timing difference
is real but does not, on its own, produce a held torque. That part of the analysis is withdrawn.

The measured CAN-starvation asymmetry in that document (right motor's variance window collapsed
69 % of the time vs 8 % left, and the decode-change A/B showing 8.1 % vs 8.3 % / 69.2 % vs 69.4 %)
**stands** — it was measured, not inferred. It is just no longer load-bearing for this bug.

## 4. What is still true and still worth fixing

These are real defects found during the investigation, independent of the unknown root cause:

1. **`send_data()` treated "disabled" as "go silent."** On a hold-last-command motor that means
   *freeze whatever was last on the bus*, not *stop*. **Fixed** — see §5.
2. **`read_data()` was gated on `enabled`**, which froze `_motor_data->i` — the exact signal
   `check_response()` measures the variance of. The "motor stopped responding" check had its input
   disabled by the very flag it reacts to, so its re-enable was structurally guaranteed to fire
   eventually. **Fixed** — see §5.
3. **`check_response()`'s variance re-enable is still wrong in principle.** It silently turns a
   deliberately-disabled motor back on. The `is_AK60v3` guard (`f9a478f`) stops the destructive
   frame but not the re-enable. Deleting this check outright is the right long-term answer.
4. **`update_controller_params()` (`uart_commands.h:185`) writes the incoming controller id with
   ZERO validation:** `j_data->controller.controller = msg.data[CONTROLLER_ID];`. Any byte that
   arrives becomes the controller, including `disabled = 1`.
5. **The GUI can substitute a positional index for a real controller id.**
   `ActiveTrialSettingsPage._on_apply()` line 453: `controller_id = controller_local_idx` when
   `row[3]` cannot be parsed; line 484: `payload = [is_bilateral, 1, 0, 0, value]` as a catch-all
   fallback. Combined with 4, a truncated handshake can select a controller the user did not pick.
   On 2026-07-23 16:33 the log recorded *"Controller list looks incomplete: 12 of 42 rows lost in
   transit; 1 malformed row(s)"* — so the precondition was present that day. **This is a hypothesis
   with a plausible precondition, not a demonstrated chain**: no parameter-apply was logged in that
   session, and it does not explain "perfect during the trial."
6. **CAN frames are routed by queue position, not by CAN id** (`CAN::read()` is a single destructive
   pop), and `timeout_count++` is commented out (`Motor.cpp:514`) so starvation is undetectable.
7. **`constrain()` is a macro and passes NaN through**, and `(unsigned int)NaN` saturates to 0 on
   Cortex-M7 — which `send_data()` encodes as **−I_MAX, i.e. full negative torque**. Negative is
   plantarflexion, and plantarflexion on the right leg is what was reported. No NaN source was ever
   identified, so this remains a hazard rather than a finding.

## 5. Fixes implemented (2026-08-10, unflashed)

### A. `send_data()` transmits zero continuously while disabled — `Motor.cpp`
`else if (_prev_motor_enabled)` → `else`. The zero frame (kp=0, kd=0, i_ff=0 — a true free-spin
command) now goes out **every cycle** the motor is disabled, instead of once on the falling edge.

This makes the held command always zero. Whatever frame a bad cycle emits — the malformed enable
frame, a NaN-saturated command, a glitch — is overwritten ~2 ms later, and the motor is *actively
commanded to zero* rather than latching. It also fixes the reproduced "ankle freezes on controller
change / after End Trial" bug at the root rather than by reset timing.

`_prev_motor_enabled` is still maintained (it is read by `enable()` at `Motor.cpp:459` and `:777`).

### B. `read_data()` reads every cycle — `Motor.cpp`
Removed the `if (_motor_data->enabled)` gate. **Required by A**: now that we transmit while
disabled, the motor replies while disabled, and unread replies accumulate in the single shared
FlexCAN RX queue — which, given positional routing, permanently offsets the left/right alternation.
Draining every cycle prevents that, and simultaneously un-freezes the variance signal (defect 2).

### C. End Trial can also drop the motor-enable pin — `uart_commands.h`, `Config.h`
`get_system_reset()` sets `j_data->motor.is_on = false` alongside `enabled = 0`, so
`_Motor::on_off()` drives `logic_micro_pins::enable_*_pin` low on the next control cycle — inside
the `reset_pending` deferral and after zero frames have gone out.

**Gated behind `END_TRIAL_CUTS_MOTOR_POWER`.**

> **UPDATED 2026-08-12 — this flag is now `1` (ON), deliberately.** This section previously said it
> "defaults to 0 (OFF)", which had drifted out of step with `Config.h` and was the wrong direction
> for a doc to be wrong in. Decision recorded by the user: **End Trial is not pressed mid-stride** —
> it is an end-of-session action — and if it ever is pressed mid-stride that is a hazard whether or
> not this toggle is set, so gating on it buys nothing. The flag stays ON for the belt-and-braces
> benefit of physically de-powering the driver after the zero frames have gone out.

It is still not established whether that pin cuts driver power (joint free-spins — safe) or asserts
a driver disable that shorts the phases (velocity-dependent brake on both ankles). It is also still
lightly-exercised code: the only other writer of `is_on` is the estop branch, and estop is hardcoded
off. Neither changes the decision above, but both are worth knowing if this path ever misbehaves.

**Optional bench check, if you ever want the answer:** power the exo, back-drive an ankle by hand,
drop the pin, feel whether it goes free or stiff.

Scoped to the reset path only. It must **not** be copied into `motors_off` (`'w'` = the Pause
button) — nothing sets `is_on` back to true except the first-run init block in `ExoCode.ino`, so the
motors would stay dead until a power cycle.

## 6. What would actually settle the root cause

The chain is unknown, so instrument rather than theorise:

1. **Log every `enable()` transmission** with its caller, the exo status, `user_paused`, `enabled`,
   and the active controller id. If the malformed frame is ever really sent, this catches it and
   names the path.
2. **Log every controller-id write** in `update_controller_params()` — this is the one unvalidated
   input that can reach `set_controller(disabled)`.
3. **Re-enable SD logging for the end-trial window specifically** (`sdLogEnabled = 1`, high
   decimation) so the last ~200 control cycles before a reset are on disk. Every log we have stops
   before the event: the GUI CSV closes ~40 ms early, and SD logging was off on 2026-07-23.
4. **Re-enable `timeout_count++`** so a starved motor is visible at all.

Until one of those produces evidence, do not treat any mechanism in this file as established.

## 7. Sessions and which firmware they ran (for future reference)

Determined from the GUI logs — useful because the two branches are easy to confuse:

| session | branch | evidence |
|---|---|---|
| 2026-07-23 (all) | **ours** (`fix_spline_jitter`) | `"...reset commands (reliable)"`, `[shutdown-debug] parsed step=N`, ShutdownDialog |
| 2026-08-06 | **raw** (`backup_branch_with_UW_edits`) | `G`/`w`/`Z` as three separate `_run_write` calls, `disconnect()` ~200 ms after End Trial |
| 2026-08-08 | **raw** | same signature |

The controller list arrived truncated in **every** session on both branches — 13 / 20 / 19 / 21
entries across four connections on 2026-08-08 alone. Only our branch warns about it.
