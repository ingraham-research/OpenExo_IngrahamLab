# End Trial sends a malformed "enable" CAN frame that slams the ankle (right ankle destroyed)

**Date:** 2026-07-23
**Scope:** Analysis, plus **one code change**: the `is_AK60v3` guard in `_CANMotor::check_response()`
(fix 1 below). **Not compiled, not flashed, not tested** — hardware is damaged and out of service.
**Status:** Root cause identified by code inspection + log/telemetry evidence. **Not yet confirmed
on hardware** (cannot be, until the exo is rebuilt). Confidence is high because the mechanism
predicted the damaged side (right) before that was known, and the prediction held.
**Read alongside:**
`Motor-Freeze-Controller-Change-And-End-Trial.md` (the deferred-reset fix this interacts with),
`SD-Card-Logging-and-End-Trial-Reset.md` (end-trial handshake; its "right motor CAN" note is
partly superseded here), `BLE-Handshake-Controller-List-Loss.md`.

---

## Symptom

Pressing **End Trial** occasionally causes the ankle to command a large, sustained torque that is
**held for several seconds** and then released abruptly. On 2026-07-23 this was severe enough to
**physically break the exo (right ankle)**. Reported to have happened once or twice before, on
earlier firmware.

Observed properties, all of which the root cause must explain:

- The exo behaves **perfectly during the trial**. The event begins only on the End Trial click.
- The torque direction felt like **plantarflexion**, on **one leg only**, and was **the same
  direction every occurrence**.
- It happened under the **ZeroTorque** controller — which commands ~0 Nm by design.
- **Nothing appears in any log**: not the SD motor logs, not the GUI CSV.
- It is **intermittent** and appears correlated with BLE link conditions / distance to the laptop.

---

## Root cause

At End Trial, the firmware can transmit a **malformed CAN frame** that the AK60v3 decodes as a
maximum-gain position command. It bypasses the controller entirely, which is why the active
controller (ZeroTorque) is irrelevant and why nothing is logged.

### 1. The frame

`_CANMotor::enable(bool overide)` (`ExoCode/src/Motor.cpp:402-460`) builds the MIT-protocol
"enter motor mode" special frame and — for an AK60v3 — sends it with
`msg.id = ((uint32_t)8 << 8) | id`, **the same CAN ID `send_data()` uses for torque commands**:

```cpp
msg.buf[0..6] = 0xFF;
msg.buf[7]    = (enabled && !_error && !estop) ? 0xFC : 0xFD;
```

The AK60v3 does not consume this as a magic word — `Joint.cpp:1138` says so explicitly:

```cpp
if (!is_AK60v3) {
    // The AK60v3 enables automatically and does not expect an enable command.
    _motor->enable();
}
```

So the motor unpacks those 8 bytes with the same MIT field layout `send_data()` packs:

| field | bits from `FF FF FF FF FF FF FF FC` | decoded value |
|---|---|---|
| `kp_int`  | `(0xFF<<4)｜(0xFF>>4)` = 4095 | **kp = 500** (`_KP_MAX`) |
| `kd_int`  | `(0xF<<8)｜0xFF` = 4095       | **kd = 5** (`_KD_MAX`) |
| `p_int`   | `(0xFF<<8)｜0xFF` = 65535     | **p_des = +12.5 rad** (`_P_MAX`) |
| `v_int`   | `(0xFF<<4)｜(0xFF>>4)` = 4095 | **v_des = +48 rad/s** (`_V_MAX`) |
| `i_int`   | `(0xF<<8)｜0xFC` = 4092       | **i_ff = +10.3 A** (`_I_MAX`) |

That is a command to drive the joint to **+12.5 rad (716°) with maximum position gain and full
current**. The target is unreachable, so the position error never closes and the motor saturates
**continuously**. It is a *position hold*, not a torque pulse — it does not decay. It persists until
a new frame arrives or power is cut, which is the observed "held for seconds, then released."

The disable frame (`...0xFD`) decodes just as badly (`i_int` = 4093).

Torque ceiling at the joint: `I_MAX 10.3 A x Kt 1.11 Nm/A x gearing 4.5` ≈ **51 Nm**.

### 2. The only path that can send it

`enable()` is **never** called for an AK60v3 from the control loop — `run_joint()` guards it (above).
There is exactly one other caller, and it has **no such guard**:

```cpp
// _CANMotor::check_response(), Motor.cpp:350
if (pop_vals.second < _variance_threshold && !_motor_data->enabled)
{
    _motor_data->enabled = true;
    enable(true);              // <-- no is_AK60v3 check
}
```

This branch requires `!enabled` **while `active_trial` is still true** (`check_response` returns
early otherwise, `Motor.cpp:335`). During a trial `enabled` is always 1, so it can never fire.
**The only moment it can fire is when the motor is disabled while the status is still active.**

### 3. Why End Trial creates exactly that state

`send_end_trial_sequence()` (`Python_GUI/services/QtExoDeviceManager.py:595`) sends `'Z'` (reliable,
write-with-response), then `'G'` and `'w'` (both `response=False`, best-effort).

- `'G'` → `ble_handlers::stop` sends `update_status(trial_off)` **then**
  `update_motor_enable_disable(0)`. Status changes first → `check_response` is closed → safe.
- `'Z'` → `get_system_reset` sets `enabled=0` **and** `trial_off` in the same handler → atomic → safe.
- **`'w'` → `ble_handlers::motors_off` sends the disable and *nothing else*. It never touches status.**

The Teensy consumes **one UART message per control cycle** (`Exo.cpp:102-103`). So if `'w'` reaches
it before `'G'`'s status change — because `'G'` was delayed/lost, or because the Nano's
`_maybe_system_reset()` state machine takes ≥2 loop iterations to relay `'Z'` — then `enabled` goes
to 0 with the trial still active. **Armed.**

This is the intermittent part, and it is why the failure tracks BLE conditions. Note the mechanism
is *timing*, not payload corruption: BLE CRCs every packet, so a weak link causes **loss and
reordering, never garbled bytes**.

### 4. Why the re-enable is guaranteed once armed

`read_data()` only updates `_motor_data->i` **while the motor is enabled** (`Motor.cpp:153`).
Disable it and the current reading **freezes**. The variance queue then fills with one repeated
value, variance → 0, and the re-enable condition is satisfied. This is not a coincidence — it is
structural. The check is designed to detect "motor stopped responding" but its input is gated on
the very flag it reacts to.

### 5. Why the right leg (two independent reasons that agree)

**(a) Direction.** `send_data()` compensates for mirrored motor mounting:

```cpp
int direction_modifier = _motor_data->flip_direction ? -1 : 1;
```

**`enable()` applies no direction modifier at all.** So the malformed frame drives the same
*electrical* direction on both motors, uncompensated. With `ankleFlipMotorDir = right`:

| leg | `flip_direction` | raw +10.3 A produces |
|---|---|---|
| left | 0 | + torque direction (dorsiflexion) |
| **right** | **1** | **− torque direction (plantarflexion)** |

Controllers command negative torque for push-off assist (`spline.csv` nodes are `-8`, `-12`), so
negative = plantarflexion. **Plantarflexion ⇒ right leg.** Because the frame is a constant, the
direction is fixed — it drives the same way on the same leg every time, matching the report that
the direction was identical on separate occurrences.

**(b) Which motor wins the re-enable race.** See the CAN section below: the right motor's feedback
is frozen far more often, so its variance is *already* collapsed and it re-enables on the first
control cycle. A healthy motor needs ~25 cycles (~50 ms) to flush its queue and normally loses the
race to the status flip.

Both reasons independently select the right ankle. **The right ankle is what was damaged.**

---

## The CAN starvation mechanism (measured)

### Why the read pattern starves a motor

There is **one CAN peripheral and one shared RX queue**. `CAN::read()` (`CAN.h:60`) is a single
**destructive pop**:

```cpp
CAN_message_t read() { CAN_message_t msg; Can0.read(msg); return msg; }
```

`read_data()` pops one frame and checks the ID **after** popping; on mismatch it simply returns and
**the frame is gone** — not requeued, not routed to its owner:

```cpp
CAN_message_t msg = can->read();
if (msg.len == 0 || !msg.flags.extended) return;
if ((msg.id & 0xFF) == uint8_t(_motor_data->id)) { ...decode... }
// no match -> silently discarded
```

`Exo::run()` calls `left_side.run_side()` then `right_side.run_side()` (`Exo.cpp:90-91`), and each
side's `transaction()` does `send_data(); read_data();` with **no settle delay** — so each motor
pops one frame per cycle, always in the order left-then-right.

This is a **positional demultiplexer**: it assumes "the 1st pop of the cycle is left's frame, the
2nd is right's." Identity is checked only to validate that assumption, never to route. Two
consequences follow:

**Consequence A — parity errors latch permanently.** Production and consumption are both exactly
2 frames per cycle, so a phase offset is a *conserved quantity*. One missing or extra frame flips
the alternation, and from then on left pops right's frames and right pops left's — both discarded,
forever. Nothing drains, flushes, or resyncs. There is no recovery path, and with
`timeout_count++` commented out (`Motor.cpp:514`) nothing detects it either.

**Consequence B — when the queue is short, left always wins.** If only **one** frame is available
when the cycle runs, left pops it (whether or not it is left's) and right finds the queue empty and
gets nothing. Left reads at the earliest possible instant after its own command; right reads only
after left's entire send/read/check_response plus right's send. So whenever the queue is
under-filled — which is often, given the loop was running at **306–436 Hz against a 500 Hz target
with `maxLoop=334 ms` stalls** (`debug_log.txt`) — **left is systematically served and right is
systematically starved.**

Note the reads are also simply *too early*: `transaction()` reads immediately after sending, while
`enable()` elsewhere in the same file does `delayMicroseconds(500)` before reading. Reading before
the reply can plausibly have arrived is what keeps knocking the phase off in the first place.

### Measured, trial 0009 (`Test results/Motor logs/0009/`, 6522 samples per leg)

The `±319 A` garbage described in `SD-Card-Logging-and-End-Trial-Reset.md` is **gone** — a later
CAN-read fix resolved it. That part of the older note is superseded:

```
|I| > 300 A -> 0 on both legs     |P| > 12.4 -> 0     |V| > 47 -> 0
```

But **starvation remains, and is severely asymmetric**. Feedback fields hold their previous value
when no matching frame arrives, so repeated samples measure "got nothing":

| metric | LEFT | RIGHT |
|---|---|---|
| consecutive-identical samples | 40.0% | **85.2%** |
| longest frozen run | 0.56 s | **1.98 s** |
| distinct current values (of 6522) | 1136 | **439** |
| **fraction of trial with the full 25-cycle variance window frozen** | **14.1%** | **81.4%** |

(SD log is decimated 5x from the control loop, so ~5 logged samples ≈ the 25-control-cycle window
`check_response` uses.)

**The right motor's re-enable condition is already satisfied 81% of the time.** So whenever the
End Trial ordering race occurs, the right motor re-enables on the first control cycle roughly 4
times out of 5, while the left is armed only 14% of the time and usually loses the race.

`Timeout_ct` and `Error` are **0 across all 6522 samples on both legs** — the log's own health
columns are structurally zero because detection is commented out. The firmware is blind to a motor
whose feedback is frozen 81% of the time.

### Why this doesn't happen on a Linux/SocketCAN setup

Worth recording because it isolates the actual design flaw. On a Raspberry Pi with
SocketCAN + `python-can`, frames are demultiplexed **by identity, in the kernel**, before user code
sees them:

- each socket receives its **own copy** of every frame — one consumer's read cannot consume or
  destroy another's data (the Teensy path is destructive and shared);
- per-socket **hardware/kernel filters** (`can_filters=[{"can_id": X, "can_mask": ...}]`) mean a
  motor's reader never even sees the other motor's frames — no mismatch, nothing to discard;
- the kernel RX queue is deep and per-socket, and readers typically **drain in a loop** rather than
  popping exactly one frame per pass;
- `can.Notifier` dispatches on a background thread, so a late main loop doesn't cause misses.

So the RPi design has no shared-position assumption to violate. The Teensy code's flaw is not "CAN
is unreliable" — it is **routing by queue position instead of by CAN ID**.

### Fix directions (none implemented)

All three replace **routing by queue position** with **routing by CAN ID**, which is what SocketCAN
does for us on the RPi. Listed smallest-change first.

**(a) Stash non-matching frames instead of discarding them.** The most local change — `read_data()`
keeps doing its own reading, but stops destroying other motors' data. Loop until the matching ID
turns up or the queue drains, and park anything else in its owner's slot:

```cpp
namespace {
    struct CanSlot { CAN_message_t msg; bool fresh = false; };
    CanSlot g_slots[k_max_motor_ids];        // indexed by (msg.id & 0xFF)
}

void _CANMotor::read_data()
{
    const uint8_t my_id = (uint8_t)_motor_data->id;

    // Take a frame parked for us by the other motor's read, if there is one.
    if (g_slots[my_id].fresh) { g_slots[my_id].fresh = false; decode(g_slots[my_id].msg); return; }

    CAN* can = can->getInstance();
    for (;;)
    {
        CAN_message_t msg = can->read();
        if (msg.len == 0) { _handle_read_failure(); return; }   // genuinely nothing for anyone
        const uint8_t id = msg.id & 0xFF;
        if (id == my_id) { decode(msg); return; }
        if (id < k_max_motor_ids) { g_slots[id].msg = msg; g_slots[id].fresh = true; }  // park, don't drop
    }
}
```

Note this must **not** be gated on `_motor_data->enabled` the way the current code is — that gating
is what freezes `_motor_data->i` and arms the spurious re-enable (section 4 above).

**(b) One routing drain per cycle** (preferred). Call a single `can_drain_and_route()` at the top of
`Exo::run()`, before either `run_side()`: pop until the queue is empty, write each frame into
`g_slots[id]`, and let `read_data()` only ever consume its own slot. This is strictly better than
(a) because draining to empty each cycle means **no backlog can accumulate, so a phase offset can
never form in the first place** — it removes the failure mode rather than tolerating it. It also
means each motor reads the *newest* frame rather than the oldest queued one, which is what you want
for state feedback and fixes the "reading a pipelined stale frame" problem at the same time.

**(c) FlexCAN_T4 per-mailbox filtering.** Give each motor its own mailbox via `setMBFilter()` so the
hardware demultiplexes and the shared queue never exists. Cleanest conceptually, largest change, and
needs care with the AK60v3's extended IDs.

Whichever is chosen, **re-enable `timeout_count++`** (`Motor.cpp:514`). With (a) or (b) a motor's
slot being empty is an unambiguous per-motor starvation signal — currently `Timeout_ct` is
structurally 0 and the firmware cannot see a motor that has been silent for two seconds.

---

## Why it is invisible in every log

- The frame never passes through `calc_motor_cmd()` or `t_ff`, so **nothing records it** — it is
  emitted directly by `enable()`.
- `trial_off` stops RT streaming (`Exo.cpp:110`), so the GUI CSV ends ~9 ms *before* the press.
- SD logging was disabled on 2026-07-23 (`sdLogEnabled = 0`) to fix loop-timing jitter.
- `Timeout_ct`/`Error` are hardcoded to 0 by the commented-out detection.

Separately, `trial_off` disables **every watchdog before it disables the motor**: the error manager
stops (`Joint.cpp:1118`), `check_response` returns early (`Motor.cpp:335`), RT streaming stops, and
both logs close — while the motor is still enabled and commanded. That is an architectural issue in
its own right.

### The "measured torque jumped to 100+" observation was an artifact

The GUI plot showed measured torque spiking past 100 Nm right after the click, while the CSV showed
nothing. Both are fed by the same signal (`rtDataUpdated` → `_on_rt_update`), but the handler
**always plots** (`MainWindow.py:244`) and **only logs if the CSV is open** (`MainWindow.py:252`).
`_on_end_trial` closes the CSV while notifications are still arriving:

```
16:35:13.613  Ending trial...        <- clear_plots() also runs here
16:35:13.670  CSV file closed
16:35:13.708  [shutdown-debug] cmd='P' event_data='100n'  /  '500n'
```

A 38 ms window where data is plotted but never written — onto a **freshly cleared, autoscaling**
axis, so one bogus sample renders as a full-height spike. `RtBridge.feed_bytes` keeps `_buffer`,
`_payload`, `_command` as persistent state that is **never reset at trial end**, so a truncated
frame leaves stale digits that concatenate with the next fragment (`'15'` + `'100'` → `15100` →
151.0). Every legitimate channel is under ~70.

**The torque sensor was fine throughout.** Do not treat that spike as evidence.

---

## Not caused by the 2026-07-23 changes; present on `backup_branch_with_UW_edits`

Verified on that branch — every ingredient is byte-for-byte identical: `ankle = AK60v3`,
`ankleGearRatio = 4.5`, `ankleFlipMotorDir = right`; the `if (!is_AK60v3)` guard in `run_joint` with
the same omission in `check_response`; the same `0xFC/0xFD` frame on the AK60v3 command ID;
`read_data()` gated on `enabled`; the same single-pop `CAN::read()`; and `motors_off` never touching
status. (`ankleDefaultController` is `PJMC` there rather than `zeroTorque` — irrelevant, the fault
bypasses the controller.)

The only material difference is the End Trial command order:

| | `backup_branch_with_UW_edits` | `fix_spline_jitter` |
|---|---|---|
| order | `G` → `w` → `Z` | `Z` → `G` → `w` |
| reliability | all three `response=False` | `'Z'` with-response + retry; `G`/`w` best-effort |
| reset | inline reboot | deferred 3 control cycles |

The backup order sends `'G'` (the only command carrying `update_status(trial_off)`) **first**, so the
status flip is usually queued ahead of any `enabled = 0`, which closes the `check_response` window.
That protection is **accidental and probabilistic, not structural** — `'G'` is itself best-effort
there, so losing it re-arms the bug. The reorder on the current branch was made for a good reason
(`'G'`-first hung on WinRT under trial congestion) and it moved the reliable command to the front,
but it left the status change riding on the two best-effort writes. **The backup branch is not safe,
only luckier**, and it lacks the deferred-reset fix.

**Second trigger, not End-Trial-specific:** `set_controller()` for the `disabled` controller sets
`motor.enabled = false` **without** changing status (`Joint.cpp:1169`, and the equivalent for every
joint). Selecting the "disabled" controller mid-trial should arm the same re-enable path.

---

## Recommended fixes (none implemented — hardware is down)

Ordered by how directly they address the failure:

1. ~~**Guard the `enable(true)` call in `check_response()` with `is_AK60v3`**~~ — **APPLIED
   2026-07-23** (`Motor.cpp`, `check_response()`), matching the guard already in `run_joint`.
   **Not compiled or flashed** — no Teensy toolchain in the editing environment and no hardware to
   test on. This prevents the malformed frame from ever being transmitted, which is the destructive
   part. It does **not** fix the spurious re-enable itself: `_motor_data->enabled` is still set back
   to `true`, so an AK60v3 can still come back alive at End Trial and resume taking normal
   controller commands. That is fix 2, and it is still open.
2. **Delete or redesign the variance re-enable.** It is broken regardless: its input (`_motor_data->i`)
   cannot update while the motor is disabled, so it always fires eventually.
3. **Make `motors_off` set `trial_off`**, or have `update_status(trial_off)` disable motors in the
   same handler, so the "trial off but motor enabled" window cannot exist.
4. **Route CAN frames by ID, not queue position** (see fix directions above), and re-enable
   `timeout_count++` so starvation is detectable.
5. **Clamp the final motor command** and **reject non-finite values** in `send_data()`. `constrain()`
   is a macro and passes NaN through unchanged; `(unsigned int)NaN` on Cortex-M7 saturates to 0,
   which the motor decodes as `-I_MAX` — i.e. **any NaN becomes full negative torque**. Not implicated
   in this incident, but the same "glitch becomes full scale" hazard.
6. **GUI:** block parameter writes while the controller matrix is flagged incomplete (it was flagged
   in *every* session on 2026-07-23, up to `12 of 42 rows lost`), and reset `RtBridge`'s parser state
   at trial end.

---

## Hypotheses investigated and ruled out

Recorded so they are not re-explored:

- **Today's spline node change (`node5_x 26 → 20.5`)** — replicated `_spline_interpolate` exactly and
  swept both node sets. Peak setpoint ~12 Nm before and after; the change removed a +0.887 Nm
  overshoot as intended. Worst setpoint step per control cycle 0.28 Nm in both. **Clean.**
- **Lost torque-sensor calibration** (`_calibration = 0` after reboot ⇒ reading ~90–130 Nm). Correct
  arithmetic, but `_pid` bails out on `torque_offset_reading == 0` (`Controller.cpp:210`), so it
  cannot produce motor torque. Also, the sensor was fine — see the artifact section.
- **Torque sensor cable/connector fault** — contradicted directly: readings were normal right up to
  the click, on every occurrence.
- **D-term amplification** (`ewma alpha 0.5 → 1.0`, plus `time_good` flipping true when RT streaming
  stops). Real effects, but the arithmetic does not reach damage levels: with `d_gain = 0.001`,
  producing 51 Nm needs a ~100 Nm single-sample step in the reading.
- **NaN/inf reaching `send_data`** — a genuine and serious hazard (see fix 5), but no source was ever
  identified, and it does not explain End-Trial exclusivity.
- **AK60v3 holding a stale non-zero command through the reboot** — the deferred-reset fix works as
  designed; the zero frame does go out. And under ZeroTorque every in-trial frame is ~0 Nm, so a
  held command could not be large.

---

## Evidence sources

- `Python_GUI/Saved_Data/logs/app_crash_20260723_163325.log`,
  `device_manager_20260723_163326.log` — End Trial timings, CSV-close vs notification ordering,
  shutdown steps, controller-matrix corruption warnings, `Update Controller` presses.
- `Python_GUI/Saved_Data/trial_20260723_163502.csv` (and the three earlier trials) — normal torque
  throughout; recording stops before the event.
- `Test results/Motor logs/0009/Motor_L_log.txt`, `Motor_R_log.txt`, `debug_log.txt` — CAN staleness
  measurements and loop-rate data.
- Firmware: `Motor.cpp`, `Joint.cpp`, `Exo.cpp`, `Controller.cpp`, `CAN.h`, `ble_commands.h`,
  `uart_commands.h`. GUI: `MainWindow.py`, `RtBridge.py`, `QtExoDeviceManager.py`.

## Confirmation still outstanding

The chain is inferred, not observed. To confirm on rebuilt hardware: instrument `enable()` to log
every transmission with the caller and the exo status, then reproduce the End Trial ordering race
(a long BLE link, or an artificially delayed `'G'`). Expect an `enable()` call from
`check_response()` with the status still `trial_on`/`fsr_refinement`, on the starved motor.
