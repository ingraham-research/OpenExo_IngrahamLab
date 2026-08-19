# `fix_spline_jitter` vs `backup_branch_with_UW_edits`: did our edits cause the End-Trial torque lock-up, and what must we keep?

> # ⛔ PART 1 RETRACTED 2026-08-10 (same day, later) — PART 2 STANDS
>
> **Part 1's answer ("yes, our edits caused it") is withdrawn.** It inherited the arming chain from
> `End-Trial-Malformed-Enable-Frame-Right-Ankle-Damage.md`, which is disproven: `'w'` sets
> `user_paused = true` atomically with `enabled = 0`, and `check_response()` early-returns on
> `user_paused`, so the malformed frame cannot be generated at End Trial **on either branch**.
>
> With no malformed frame, the overwrite-window argument is moot. Re-tracing the nominal End Trial,
> **both branches end with the one-shot zero frame as the last frame on the bus.** The 5000 ms →
> ~6 ms reset-relay change is real, but on its own it does not produce a held torque.
>
> **Still valid from Part 1:** the measured CAN-starvation asymmetry (right motor's variance window
> collapsed 69 % of the time vs 8 % left) and the decode A/B that falsified the CAN-decode
> hypothesis (8.1 % vs 8.3 % left, 69.2 % vs 69.4 % right). Those were measured.
>
> **The true root cause is unknown.** See `End-Trial-Diagnosis-Correction.md`.
>
> **Part 2 (the must-keep / optional edit tiers) is unaffected and remains the reference** — with
> one change: **Tier X is downgraded.** The end-trial reset acceleration is no longer known to be
> dangerous. It is still worth reviewing, but it is not the regression Part 1 claimed.

**Date:** 2026-08-10
**Method:** static analysis of both branches (`git diff backup_branch_with_UW_edits...fix_spline_jitter`,
merge base `6a47f63`) plus a quantitative re-analysis of the trial-0009 SD motor logs.
**Code changes made:** none. Nothing was compiled, flashed, or run on hardware. The raw branch was
not touched.
**Read alongside:** `End-Trial-Malformed-Enable-Frame-Right-Ankle-Damage.md` (the mechanism; this
document **corrects its final conclusion**), `Motor-Freeze-Controller-Change-And-End-Trial.md`,
`SD-Card-Logging-and-End-Trial-Reset.md`.

---

# Part 1 — Did our edits cause it?

## Short answer

**Yes. It is not bad luck, and the raw branch is not merely "luckier".**

The *destructive frame* is a pre-existing bug that exists identically on both branches. But the frame
only causes damage if **nothing overwrites it before the CPU reboots**, and that is where the two
branches genuinely differ:

| | `backup_branch_with_UW_edits` | `fix_spline_jitter` |
|---|---|---|
| Malformed frame can be generated | **yes** (same code) | **yes** (same code, until `f9a478f`) |
| What happens on the **next** control cycle | a normal torque frame is sent → **frame overwritten in ~2 ms** | **nothing is sent** → frame stands |
| Time from that moment to Teensy reboot | **~5 seconds** | **~6 ms** |
| Net effect | a 2 ms click | **a held ~51 Nm position slam that survives the reboot** |

The raw branch has a *second, independent layer of protection* that we removed. The earlier write-up
found the first layer (`'G'`-first ordering) and correctly called it probabilistic — but it missed
this second layer, which is structural. That is why the company's testing on the raw branch found
"zero issues" even though the same latent bug is sitting in their firmware.

**The single decisive edit is `ComsMCU::_maybe_system_reset()`: we replaced a 5000 ms delay before
relaying the reset with a state machine that relays it in ~2 Nano loop iterations.**

---

## 1.1 The part that is genuinely shared (pre-existing, not ours)

All of this is byte-for-byte identical on both branches and was verified again here:

- `_CANMotor::check_response()` (`Motor.cpp:~350`) re-enables a motor whose measured-current variance
  collapses, and calls `enable(true)` with **no `is_AK60v3` guard** — unlike `Joint.cpp::run_joint()`,
  which has one.
- `_CANMotor::enable()` transmits `FF FF FF FF FF FF FF FC` on `((8<<8)|id)` — the *same* CAN ID
  `send_data()` uses for torque. The AK60v3 does not consume the MIT "enter motor mode" magic word,
  so it unpacks those bytes as `kp=500, kd=5, p_des=+12.5 rad, v_des=+48 rad/s, i_ff=+10.3 A`.
  A full-current, max-gain position slam to an unreachable target ⇒ **saturated hold, ≈51 Nm at the
  joint** (`I_MAX 10.3 A × Kt 1.11 × gearing 4.5`). It also skips `send_data()`'s
  `direction_modifier`, so it drives uncompensated.
- `read_data()` is gated on `_motor_data->enabled`, so disabling the motor **freezes**
  `_motor_data->i` — which is the very input the variance check reads. Arming is therefore
  structurally guaranteed once the motor is disabled.
- `ble_handlers::motors_off` (`'w'`) disables the motors and **never touches the exo status**, so it
  can create "motor disabled while the trial is still active" — the one state in which
  `check_response()` can fire.
- `CAN::read()` is a single destructive pop routed by queue position, not by CAN ID, so one motor
  routinely eats the other's frames.

Config is also identical: `ankle = AK60v3`, `ankleGearRatio = 4.5`, `ankleFlipMotorDir = right`.

## 1.2 A hypothesis I tested and **falsified**: the CAN decode rewrite

We rewrote the AK60v3 feedback decode (`b68feec`): the raw branch reinterprets a signed int16
current as unsigned and unpacks it through a 12-bit MIT scale (producing values from −10.3 A to
**+319 A**); ours decodes it correctly as `int16 × 0.01 A`.

It was very plausible that this made the low-variance trigger far easier to satisfy. **It does not.**

I recovered the raw int16 counts from the trial-0009 logs (`Current_A / 0.01 / direction_modifier`),
re-applied the raw branch's decode, and swept the firmware's actual trigger
(`stdev(window) < 0.01`, `_current_queue_size = 25`, `online_std_dev` returns **std dev**, not
variance) over 6522 samples per leg:

| | LEFT (new) | LEFT (old) | RIGHT (new) | RIGHT (old) |
|---|---|---|---|---|
| trigger armed, 5-row window (= 25 control cycles by time) | 8.1 % | 8.3 % | **69.2 %** | **69.4 %** |
| trigger armed, 25-row window | 1.0 % | 1.1 % | 32.5 % | 32.5 % |
| window fully frozen (identical samples) | 7.9 % | — | 69.1 % | — |

The armed fraction tracks the *frozen* fraction almost exactly, on both decodes. The trigger is
armed by **CAN starvation**, which is decode-independent — a frozen value has zero variance no
matter how you scale it. The decode change is **not** implicated. (It is still a must-keep for other
reasons; see Part 2.)

This also settles a sub-question: the *arming* precondition is equally present on both branches, and
it is heavily biased toward the **right** leg (69 % vs 8 %), independent of anything we changed.

## 1.3 The part that is ours: the overwrite window

`_CANMotor::send_data()` has three behaviors, and the third one is the whole story:

```cpp
if (_motor_data->enabled)      { can->send(msg); _prev_motor_enabled = true; }   // normal frame
else if (_prev_motor_enabled)  { ...send ONE zero frame...; _prev_motor_enabled = false; }
// else: SEND NOTHING  <-- CAN goes silent for this motor
```

And `Exo::run()` runs **`run_side()` first, then consumes exactly one UART message per control
cycle**. So a control cycle is: `send_data → read_data → check_response → consume one UART message`.

### Trace: `fix_spline_jitter`, with `'G'` lost

Nano loop order is `handle_ble()` (pops **one** BLE message) → `local_sample()` (runs
`_maybe_system_reset()`) → `update_UART()`. With `'Z'` sent first and `'G'` dropped, the Teensy's
UART queue ends up `[motor_disable (from 'w'), get_system_reset (from 'Z')]` — adjacent.

| cycle | `send_data` | `check_response` | UART consumed | CAN bus |
|---|---|---|---|---|
| N | normal frame (still enabled) | — | `motor_disable` → `enabled=0`, status still `trial_on` | normal |
| N+1 | `enabled==0`, `_prev==true` → **one-shot zero frame**, `_prev=false` | armed (69 % on right) → **fires** → `enabled=true` + **malformed frame** | `get_system_reset` → `trial_off`, `enabled=0`, `reset_pending=1` | **malformed** |
| N+2 | `enabled==0`, `_prev==false` → **nothing** | early-return (`trial_off`) | — | silent |
| N+3 | nothing | — | — | silent |
| N+4 | nothing | — | `reset_ticks==3` → `exo_system_reset()` | **reboot** |
| after | motors disabled, fresh `_prev=false` → nothing | — | — | silent |

**The malformed frame is the last command the AK60v3 ever receives.** It holds ≈51 Nm until a power
cycle or until a reconnect starts sending frames again — exactly the reported "large torque held for
several seconds, then released abruptly."

Note the bitter detail: the deferred-reset fix was added *specifically* so the zero frame could get
out before the reboot. It did get out — **at cycle N+1, one line before the malformed frame**. The
`_prev_motor_enabled` one-shot was already spent, so the second disable produces no second zero
frame. **The deferral therefore buys three cycles of silence that protect the malformed frame rather
than clearing it.**

### Trace: `backup_branch_with_UW_edits`, same `'G'` loss

`_maybe_system_reset()` there is:

```cpp
if (!_reset_pending) return;
if ((millis() - _reset_start_ms) < _reset_delay_ms) return;   // _reset_delay_ms = 5000
...forward reset over UART...; delay(10); exo_system_reset();
```

| cycle | `send_data` | `check_response` | CAN bus |
|---|---|---|---|
| N | normal frame | — | normal |
| N+1 | one-shot zero, `_prev=false` | **fires** → `enabled=true` + malformed frame | **malformed** |
| N+2 | `enabled==true` (set by `check_response`) → **normal torque frame** | — | **OVERWRITTEN, ~2 ms** |
| … | ~2500 more normal cycles | — | normal |
| +5 s | — | — | reset forwarded → inline `delay(10)` → reboot |

The exo does misbehave here (the motors silently re-enable after "motor off", and it keeps running
for 5 s), and the inline reboot then leaves the AK60v3 holding its **last normal command** — which
under ZeroTorque/idle is ~0 Nm. **That is the mild "ankle freeze" we originally set out to fix.**
It is not destructive.

### The two edits that produce the difference

1. **`ComsMCU::_maybe_system_reset()` — 5000 ms delay → ~2 Nano loop iterations** (`ComsMCU.cpp/.h`,
   part of the end-trial shutdown-progress work). This is the decisive one. It collapses the gap
   between "malformed frame emitted" and "CPU reboots" from ~5 s (≈2500 overwriting control cycles)
   to ~6 ms (**zero** overwriting cycles).
2. **The deferred Teensy reset** (`uart_commands.h::get_system_reset` + `ExoCode.ino`,
   `RESET_ZERO_TICKS = 3`). On its own this would be neutral-to-good, but combined with (1) it
   guarantees the silence window instead of shortening it.

The `'Z'`→`'G'`→`'w'` reorder is a contributing factor but **not** the root: with `'G'` lost, both
orderings put `'w'` immediately before the reset. What changed is how fast the reset lands.

## 1.4 The reliability inversion (why `'G'` gets lost in the first place)

Our own `send_end_trial_sequence()` docstring records that fire-and-forget writes were being dropped
under trial congestion — that is *why* `'Z'` was promoted to write-with-response. So we have direct
empirical evidence, from this project, that BLE writes drop at exactly this moment.

- **Before:** `'G'`, `'w'`, `'Z'` all fire-and-forget. The one that dropped was usually `'Z'` →
  symptom: *"the exo didn't reboot."* Annoying, harmless.
- **After:** `'Z'` is ACK'd and retried; `'G'` and `'w'` are still fire-and-forget. Now the droppable
  command is `'G'` — **the only one that carries `update_status(trial_off)`**, i.e. the only one that
  closes the `check_response()` window.

We made the *reboot* reliable and left the *safety interlock* unreliable. Losing `'G'` while `'Z'`
is guaranteed to arrive is precisely the destructive combination.

A second, BLE-independent route to the same state exists: `'G'` sends `update_status` and
`update_motor_enable_disable` as two separate UART messages 100 µs apart. Losing the first on the
Teensy UART link arms it identically.

## 1.5 Why the right ankle, and why it was intermittent

Two independent selectors, both agreeing, both pre-existing:

- **Arming:** the right motor's variance window is already collapsed **69 %** of the time versus
  **8 %** on the left (measured above). At End Trial the right fires on the very first cycle roughly
  7 times out of 10; the left usually needs ~25 cycles (~50 ms) and loses the race to the reboot.
- **Direction:** `enable()` applies no `direction_modifier`. With `ankleFlipMotorDir = right`, the
  same electrical direction is plantarflexion on the right — matching the reported direction, and
  matching it being the *same* direction every occurrence.

Intermittency is fully explained: it needs `'G'` (or its UART `update_status`) to be lost **and** the
variance window to be collapsed. Neither is rare, but their conjunction is not every trial.

## 1.6 Correction to the previous write-up

`End-Trial-Malformed-Enable-Frame-Right-Ankle-Damage.md` concluded: *"The backup branch is not safe,
only luckier."* That is **wrong on the decisive point**, and it matters because it would push us
toward carrying our end-trial changes forward as-is.

The backup branch has a real, structural second layer: `check_response()` sets
`_motor_data->enabled = true` **before** calling `enable(true)`, so the very next `send_data()` emits
a normal frame and erases the slam. The 5-second reset delay guarantees that next cycle happens. The
prior document analysed only whether the trigger could *fire*, not what happened to the frame
afterwards.

Everything else in that document — the frame decode, the arming mechanism, the CAN starvation
measurements, the leg selection, the "invisible in every log" section, the ruled-out hypotheses —
holds up under re-examination.

## 1.7 What this means practically

- The company's "tested with zero issues" on the raw branch is consistent and believable. The latent
  bug is in their firmware too, but its damaging expression is suppressed.
- **The raw branch is still capable of the same destruction** if anything ever shortens that
  5-second window or stops the control loop right after the frame. It lacks the `is_AK60v3` guard
  (`f9a478f`), which is the actual fix. Do not treat it as safe — treat it as *currently
  suppressed*.
- Our branch is **not** safe to run again until either the guard is validated on hardware or the
  end-trial timing is reverted. See Part 2, Tier X.

---

# Part 2 — What must be kept, what is optional

Tiers are by *necessity for a correct, safe, functional exo on our hardware*, not by effort.
"Validated" means run on the exo; "unvalidated" means written and reasoned but never flashed.

## Tier 0 — Required for the exo to be correct on our hardware

Without these the exo runs, but it runs on wrong numbers.

**0.1 — AK60v3 CAN feedback decode** — `Motor.cpp::read_data()` (`b68feec`)
The raw branch decodes the AK60v3's CubeMars status frame with the MIT unsigned-with-offset layout.
Measured current lands anywhere in **−10.3 … +319 A**; position and velocity are equally wrong
(velocity is electrical ERPM and needs `/(14 × 6)`). Every PID that closes on measured torque, every
plotted current/velocity, and every log column is garbage without this.
*Confirmed above not to affect the enable-frame bug in either direction — safe to carry on its own.*

**0.2 — Heel-FSR presence gating** — `HeelFsrConfig.{h,cpp}`, `Side.cpp`, `uart_commands.h`,
`config.ini [Sensors] heelFsrPresent` — **this is your item (a)**
The raw branch (commit `6a47f63`) only hardcoded `heel_contact_state = false` inside
`_check_ground_strike()`. Everything else still runs against the floating pin: `_heel_fsr.read()`
every control cycle, `get_contact_thresholds()`, `set_contact_thresholds()`, `reset_calibration()`,
and — worst — `calibrate()` / `refine_calibration()` from the `'F'`/refinement UART handlers, each of
which stamps `fsr_calibration` / `fsr_refinement` into the exo status. **On the raw branch a
recalibration can leave the exo parked in FSR refinement forever, driven entirely by noise on an
unconnected pin.** Our version gates all of it behind one cached SD-card flag, no reflash to change.

**0.3 — Float `percent_gait` / `percent_stance` / `percent_swing`** — `Side.cpp` (`226854b`)
Raw branch computes into an `int`, truncating to whole percent before returning a float. At a ~1.2 s
stride that freezes the setpoint for ~6 control cycles and then steps it. On the ankle spline the
step reached 1.69 Nm, and since `cmd = setpoint + p_gain·(setpoint − measured)`, at `p_gain = 6` the
commanded torque jumped ~11.8 Nm — a self-inflicted ~81 Hz sawtooth. Any `%gait`-driven controller
(Spline, ZhangCollins, FranksCollinsHip) needs this.

## Tier 1 — Safety

**1.1 — `is_AK60v3` guard in `check_response()`** — `Motor.cpp` (`f9a478f`) — **unvalidated**
The actual fix for the destructive frame. Present only on `fix_spline_jitter`. It stops the frame
being transmitted; it does **not** fix the spurious re-enable itself (`_motor_data->enabled` is still
set back to `true`, so an AK60v3 can still silently come back alive at End Trial).

**1.2 — Still open, none implemented.** Listed so they are not lost:
- Delete or redesign the variance re-enable. Its input cannot update while the motor is disabled, so
  it always eventually fires. This is the real defect.
- Make `motors_off` (`'w'`) also set `trial_off`, or have `update_status(trial_off)` disable motors
  in the same handler, so "trial active + motor disabled" cannot exist.
- Route CAN frames by **ID**, not queue position, and re-enable `timeout_count++` (`Motor.cpp:514`),
  which is currently commented out — `Timeout_ct` and `Error` are structurally 0 in every log, so the
  firmware is blind to a motor whose feedback is frozen 69 % of the time.
- Clamp the final motor command and reject non-finite values in `send_data()`. `constrain()` is a
  macro and passes NaN through; `(unsigned int)NaN` saturates to 0 on Cortex-M7, which the motor
  decodes as **−I_MAX**. Not implicated here, same hazard class.

## Tier X — Must **not** be carried over as-is

**X.1 — The end-trial reset acceleration.** `ComsMCU::_maybe_system_reset()` state machine
(`ComsMCU.cpp/.h`) + deferred Teensy reset (`uart_commands.h`, `ExoCode.ino`) + `'Z'`-first
`send_end_trial_sequence()` (`QtExoDeviceManager.py`). **This is the regression from Part 1.**

Carrying it requires *at least one* of:
- **1.1 flashed and validated** (removes the destructive frame at the source), **and/or**
- making `'G'` reliable — it is the only command that closes the window, and it is currently the one
  most likely to drop, **and/or**
- fixing 1.2's `motors_off`-sets-`trial_off` item, which removes the arming state entirely, **and/or**
- keeping a deliberate settle window before the reboot in which `send_data()` is *guaranteed* to emit
  a zero frame — note the current `RESET_ZERO_TICKS = 3` does **not** do this, because the
  `_prev_motor_enabled` one-shot has already been spent by then.

My recommendation, when we get back to code: do **1.2's `motors_off` change plus 1.1** first — those
remove the failure mode rather than re-tuning a race — then re-enable the shutdown handshake.

**X.2 — Note for whoever ships the raw branch.** It is suppressed, not fixed. If anyone shortens
`_reset_delay_ms`, adds an inline reset, or stalls the control loop right after End Trial, the same
51 Nm slam returns.

## Tier 2 — Behavior and tuning (keep, but low risk to defer)

- **SD-card files only, no reflash:** `config.ini ankleDefaultController = zeroTorque`;
  `zeroTorque.csv` → `use_pid=1, 3, 0, 0.001` (transparency); `spline.csv` nodes + PID on. Pure
  tuning, trivially reversible.
- **Spline gain scheduling near zero torque + `ewma alpha 0.5 → 1.0`** — `Controller.cpp` (`7f95828`,
  `8954667`). Needed for the spline to be usable; the near-zero gains are still hard-coded (TODO in
  the source to read them from ZeroTorque's params).
- **`ZeroTorque` reports `filtered_torque_reading`** — `Controller.cpp`. Without it the GUI/CSV show
  no measured torque under the (now default) ZeroTorque controller.
- **`ParamsFromSD::_sd_ready()`** — mounts the SD card once instead of re-mounting on every
  controller change. Removes a multi-ms control-loop stall during which the AK60v3 holds its last
  walking torque (felt as a catch at the switch), and removes a latent FAT-corruption risk when
  `SdLogger` has files open.
- **BLE handshake truncation detection** — `ExoBLE.cpp`, `RtBridge.py`, `MainWindow.py`
  (`ac1c88e`, `53079b5`). Diagnostic in form, safety-adjacent in effect: a short controller list
  means the GUI can write parameters against the wrong controller index. It was flagged in *every*
  session on 2026-07-23 (up to 12 of 42 rows lost). The underlying loss is **not** fixed — only
  surfaced.

## Tier 3 — Optional (great to have, not core)

- **Teensy SD logging** — `SdLogger.{h,cpp}`, `SdRingBuffer.h`, `ExoCode.ino` hooks,
  `config.ini [Logging]`. **Currently disabled** (`sdLogEnabled = 0`) because it dropped the control
  loop from 500 Hz to 240–300 Hz with `maxLoop` stalls up to 12 ms. Worth noting it was the tool that
  exposed the end-trial problems in the first place — but it needs the loop-timing work before it can
  be left on.
- **UDP remote control** — `Python_GUI/remote/`, `examples/`, `tests/test_remote_*`, `utils/config.py`.
  Self-contained; GUI only.
- **End-trial shutdown progress UI** — `ShutdownDialog.py`, `shutdown_progress` codes,
  `send_shutdown_progress`. QoL, but wired to the Tier X state machine — do not lift it across
  without reading X.1.
- **Status streaming on RT Channel 8** — `uart_commands.h` hijack, `PlottingTitles.h`
  (`Channel 8` → `Status`, `Stance Phase` → `In Stance`), `ActiveTrialPage.update_exo_status()`.
  Diagnostic.
- **GUI plotting fixes** — `RtBridge.py` int16 ×100 handling and exo-time wrap,
  `ActiveTrialPage._x_for_sample()` (`6322e17`). Cosmetic; fixes the ~300 s plot glitch.
- **`last_step_duration`** — `SideData.{h,cpp}`. Logging support only.
- **Docs and test logs** — `Modification log with claude/`, `docs/superpowers/`,
  `Test results/Motor logs/0009/`. Zero runtime effect; the 0009 logs are the evidence base for the
  measurements in Part 1 — keep them.

---

## Verification status — read before acting

- Part 1's cycle-by-cycle traces are **static code analysis**. They were not observed on hardware and
  cannot be until the exo is instrumented.
- The 69 % / 8 % arming figures and the decode A/B are **measured**, from
  `Test results/Motor logs/0009/Motor_{L,R}_log.txt` (6522 samples/leg). The SD log is decimated 5×,
  so the 5-row window is a time-equivalent proxy for the firmware's 25-control-cycle window; the
  25-row column is given alongside so the conclusion does not rest on that choice. Both windows give
  the same answer.
- Nothing in Tier 0/1 has been compiled or flashed. `f9a478f` in particular is **untested**.

**To confirm the mechanism on rebuilt hardware:** instrument `enable()` to log every transmission
with its caller and the current exo status, then force the ordering race (drop `'G'` deliberately, or
add a long BLE link). Expect an `enable()` call from `check_response()` with the status still
`trial_on`, on the starved motor, followed by ≥3 control cycles with no CAN transmission.
