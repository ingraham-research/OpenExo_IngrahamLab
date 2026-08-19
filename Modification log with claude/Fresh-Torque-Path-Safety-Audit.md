# Fresh torque-path safety audit — can faulty torque reach the ankle, and what caused the lock-up?

**Date:** 2026-08-11
**Method:** clean re-derivation from source. Every CAN transmit site enumerated from
`grep can->send`, every caller traced, every field in the command frame checked. Prior conclusions
deliberately not assumed. No code changed by this audit.
**Verdict on goal 1: the ankle is currently protected, but by three accidents rather than by design.**
**Verdict on goal 2: one quantitative, branch-specific mechanism reaches the damage level using
only ordinary numbers. It is the best candidate found so far, and it is not proven.**

> ### ⚠ PARTLY OVERTAKEN BY `3c08c77`, COMMITTED 38 MINUTES AFTER THIS AUDIT WAS WRITTEN
> Noted 2026-08-12. Two things below are no longer current:
>
> 1. **"Protected by three accidents rather than by design" — no longer true for `enable()`/`zero()`.**
>    `3c08c77` put an `is_AK60v3` guard at the **top of `_CANMotor::enable(bool)` and
>    `_CANMotor::zero()` themselves**, so every caller is covered by construction. In particular the
>    §"three accidents" claim that `Side::disable_motors` (`Side.cpp:62`) has **NO GUARD** and is
>    "saved only because the function has zero callers" is **now false** — it is guarded at the
>    callee. The audit's point that guarding at each call site was fragile is what motivated that
>    change; treat this document as the argument, not the current state.
> 2. **The 51.4 Nm ceiling below is superseded by 54.0 Nm.** That figure assumed
>    `_I_MAX × Kt × gearing`. The AK60-6 unpacks the 12-bit torque field against ±12.0, not against
>    our `_I_MAX` of 10.3, so the real held value is `12.0 × 4.5 = 54.0 Nm`. See
>    `Motor-Current-Decode-Investigation.md` and the comment at `Motor.cpp::enable()`.
>
> Everything else here — the four-transmit-site map, the frame decode, the field table — still
> stands and is still the clearest description of *why* those two frames are dangerous.

---

# Goal 1 — Complete map of every path to the motor

There are exactly **four** CAN transmit sites in the firmware (`Motor.cpp:372, 420, 566, 606`).
`CAN::send()` is the only wrapper, `Can0.write()` the only primitive.

| # | site | function | contents | clamped? |
|---|---|---|---|---|
| 1 | `Motor.cpp:372` | `send_data()` normal | controller command | **YES** — `MAX_JOINT_TORQUE_NM` gate |
| 2 | `Motor.cpp:420` | `send_data()` zero frame | constant zeros | N/A — provably zero |
| 3 | `Motor.cpp:566` | **`enable()`** | `FF FF FF FF FF FF FF FC`/`FD` | **NO — bypasses `send_data()`** |
| 4 | `Motor.cpp:606` | **`zero()`** | `FF FF FF FF FF FF FF FE` | **NO — bypasses `send_data()`** |

## Why sites 3 and 4 are dangerous on an AK60v3

Both build the frame with

```cpp
msg.id = ((uint32_t) 8 << 8) | (uint32_t)_motor_data->id;   // identical to send_data()
```

— **the same CAN id used for torque commands.** The motor therefore unpacks those bytes with the
normal field layout (`kp[11:0], kd[11:0], p[15:0], v[11:0], i[11:0]`):

| field | bits from `FF FF FF FF FF FF FF FC` | decoded |
|---|---|---|
| kp | 4095 | **500 (max)** |
| kd | 4095 | **5 (max)** |
| p_des | 65535 | **+12.5 rad (716°)** |
| v_des | 4095 | **+48 rad/s** |
| i_ff | 4092 | **+10.28 A (≈max)** |

A max-gain position command to an unreachable target: the error never closes, so the motor
saturates and **holds**. `0xFD` and `0xFE` decode to the same thing within one LSB of current.
Neither applies `send_data()`'s `direction_modifier`, so on the flipped side it drives the wrong way.

**Ceiling, confirmed from source:** `_I_MAX = 10.3 A` and `Kt = 0.185 × 6 = 1.11 Nm/A`
(`Motor.cpp:750-755`), external gearing 4.5 (`config.ini`) ⇒ **51.4 Nm at the joint.**

## Why they are nonetheless unreachable today — three accidents

**Site 3, `enable()`** — all callers traced:

| caller | AK60v3 reachable? |
|---|---|
| `Joint.cpp:1141` (`AnkleJoint::run_joint`) | No — wrapped in `if (!is_AK60v3)` |
| `Joint.cpp:678, 904, …` (hip/knee/elbow/arm) | Same guard; those joints are `is_used == false` here anyway |
| `Motor.cpp:468` (`check_response`) | No **on this branch** — the `is_AK60v3` guard added 2026-07-23. **Still reachable on `backup_branch_with_UW_edits`, which has no guard.** |
| **`Side.cpp:64-69` (`Side::disable_motors`)** | **NO GUARD.** Calls `enable(true)` on all six motors, `overide = true` so it transmits unconditionally. **Saved only because the function has zero callers** — declared in `Side.h:77`, defined, never invoked. Identical on both branches. |

**Site 4, `zero()`** — one caller: `Joint.cpp:115`, `if (_joint_data->motor.do_zero) _motor->zero();`
inside `_Joint::check_calibration()`, which runs **every control cycle**. `do_zero` is **never set
true anywhere in the main firmware** (only in the `systemCheck/SPI` test sketch). And
`MotorData.h:48` declares `bool do_zero;` with **no initializer** — the constructor never touches it.
It is false only because `ExoData` is a function-local `static` (`ExoCode.ino:120`) and objects with
static storage duration are zero-initialized before construction. Change that to a heap allocation
(as the Nano branch does at line 850: `new ExoData(...)`) and `do_zero` becomes indeterminate — a
per-boot coin flip on whether the ankle gets slammed every control cycle.

**Inside `send_data()`** — the clamp bounds `torque`, but the frame also carries `kp`, `kd`,
`p_des`, `v_des`. If `kp` were nonzero the motor would run a position loop producing torque
**entirely outside the clamp**. All four are `= 0` by default member initializer (`MotorData.h`) and
are explicitly re-zeroed per joint in `ExoCode.ino:204-395`. **No code anywhere writes a nonzero
value** (verified by grep: the only other references are `logger::println` calls in `ExoData.cpp`).
So the position channel is inert — by disuse, not by enforcement.

## Answer to goal 1

**No, I cannot give you an unconditional confirmation.** What I can say precisely:

- **Every torque-producing command that actually flows today is clamped at 25 Nm at the joint**, and
  the clamp binds well before the hardware ceiling (25 Nm ⇒ 5.0 A vs `I_MAX` 10.3 A).
- **Two unclamped 51 Nm paths exist in the binary.** Both are unreachable, but each is held shut by
  a single accident: a function nobody calls, and a flag nobody sets that is zeroed only by storage
  class. Neither is defended by a check.

### To make it unconditional (recommended, not implemented)

1. **Guard `enable()` and `zero()` at the top**, not at every call site:
   `if (_motor_data->motor_type == AK60v3) return;` — the AK60v3 auto-enables and has no origin-set
   in this protocol, so both are meaningless to it. This closes sites 3 and 4 permanently and makes
   `Side::disable_motors()` harmless.
2. **Delete `Side::disable_motors()`** or reimplement it as `enabled = 0` (its name already lies —
   `enable(true)` is the *override* flag, not "enable").
3. **Initialize `do_zero`, `enabled`, `is_on`, `timeout_count`** in the `MotorData` constructor.
4. **Clamp `kp`/`kd` to 0 in `send_data()`** for torque-mode motors, so the position channel cannot
   be opened by a future edit.
5. `_MaxonMotor::send_data()` is not covered by the clamp (not used on this exo — ankles are AK60v3).

---

# Goal 2 — What could have caused the lock-up

## The mechanism that reaches 51 Nm with ordinary numbers

This needs no exotic frame, no race, and no corrupted message.

**`spline.csv` differs between the branches in a way I under-weighted before:**

| field | raw | ours |
|---|---|---|
| PID Flag | **0** | **1** |
| P Gain | 0 | **6** |
| D Gain | 0 | **0.03** |
| nodes (x,y) | `0,0 / 25,0 / 48,12 / 63,0 / 100,0` | `0,0 / 5,-8 / 10,-12 / 20,0 / 20.5,0` |

On raw, `Spline::calc_motor_cmd()` takes the `else` branch: `cmd = torque_cmd`. **Open loop, bounded
by the node values — it cannot saturate.**

On ours: `cmd = torque_cmd + 6·(torque_cmd − filtered_torque_reading) + 0.03·de_dt`.

Solve for the saturation point with the peak node `torque_cmd = −12 Nm`:

```
-12 + 6·(-12 - m) = -51.4   →   m = -5.43 Nm
```

**A measured torque of −5.4 Nm while the spline commands −12 Nm saturates the motor at full
current — 51.4 Nm at the joint.** That is a tracking error of 6.6 Nm: ordinary actuator lag, at the
exact phase of every stride where the joint is most loaded. Recorded measured torques on this exo
routinely reach ±15–20 Nm.

The D term makes it worse: `de_dt = -(measurement - _prev_input)/dt_s`, and our branch also changed
the spline's torque filter from `ewma alpha 0.5` to **`1.0` (no filtering)**, so a single-sample
reading jump of Δ contributes `0.03 · Δ/0.002 = 15·Δ`. A 3.5 Nm sample-to-sample jump alone saturates.

Note `time_good` guards only *large* `dt`; `dt_us > 0` passes, so an unusually **small** dt inflates
`de_dt` without limit. Nothing bounds the D term.

## Why saturation becomes "held", and why at End Trial

A saturated command is transient while the loop runs — the error closes and it backs off. It becomes
**held** when the frame stream stops, because the AK60v3 latches its last command. Before the
2026-08-10 change, `send_data()` went **silent** once `enabled` dropped to 0 (one zero frame on the
falling edge, then nothing), and End Trial then reboots the Teensy.

So the composite requires two ingredients, and our branch supplied both:

1. **a saturated frame** — spline PID on, P=6 stacked on a 12 Nm feed-forward, unfiltered;
2. **the stream stopping right after it** — End Trial disable → silence → reboot.

Raw supplies neither readily: its spline is open loop, and its PJMC (`max stance torque = 0`) must
develop ≈8.5 Nm of *measured* torque with **no feed-forward added** to reach the same ceiling.

**Honest caveat:** raw's PJMC does run P=6 with `torque alpha = 1` (its CSV is byte-identical on both
branches), so raw is **not structurally immune** — it is further from the boundary, not behind a
wall. This hypothesis explains a difference in *likelihood*, not in *possibility*. It is consistent
with "many instances on ours, none observed on raw," but it does not prove that difference.

**What it does not explain:** why the exo felt normal for the whole trial and failed only on the
click. If the spline saturated every stride you would expect to feel it. Two readings are possible —
that saturation did occur during walking and was masked by the assist feeling strong, or that the
saturating sample occurred only in the last cycles before the reboot. I cannot distinguish them from
the available logs.

## Why no log shows it

`_motor_data->t_ff` records the value *after* `constrain()` — the saturated value, not the request.
The GUI plots `controller.desired_torque`, which for the spline is `ff_setpoint` (the node value,
≤12 Nm), **not** `cmd`. So a saturated PID output was invisible on both the plot and the CSV. The
new clamp's `Serial` warning is the first thing that would ever have reported it.

## Both current fixes attack this directly

- **Continuous zero frames while disabled** removes ingredient 2: whatever the last frame was, it is
  overwritten ~2 ms later and the motor is actively commanded to zero.
- **The 25 Nm clamp** removes ingredient 1: the command physically cannot exceed 25 Nm at the joint,
  saturated or not, and it now prints when it trips.

## Ranked candidates

| # | candidate | status |
|---|---|---|
| 1 | **Spline PID saturation** (above) | Quantitative, branch-specific, reaches 51.4 Nm with ordinary values. **Best candidate. Unproven.** |
| 2 | `check_response()` → `enable()` on raw | Real 51 Nm path, **but blocked at End Trial** on both branches: `update_motor_enable_disable` sets `user_paused = true` in the same handler that clears `enabled` (`ENABLE_DISABLE = 0`), and `check_response` early-returns on `user_paused`. Only `set_controller(disabled)` leaves it open, and that would fire at 500 Hz continuously. |
| 3 | `Side::disable_motors()` / `zero()` | 51 Nm paths held shut by dead code and an unset flag. Not reachable in the shipped binary. |
| 4 | NaN reaching `send_data()` | Was real — `constrain()` is a macro, NaN passes, `(unsigned int)NaN` → 0 → `-I_MAX`. **Now closed by the clamp.** No source was ever identified. |

## What would settle it

The clamp's `Serial` line is now the instrument: **if the spline hypothesis is right, a bench walk
with PID on will print `TORQUE CLAMP` lines.** That is a decisive, safe, no-hardware-risk test — run
it before anything else. Silence over a few minutes of walking falsifies candidate 1.

Second: log `cmd` (post-PID) alongside `desired_torque`, since nothing currently records the value
that actually drives the motor.
