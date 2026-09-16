# Nano reboot mid-trial: state resynchronisation — FUTURE TASK, not yet started

**Date raised:** 2026-09-12
**Status:** **PARKED 2026-09-12 by decision. No code written.** The disconnect root cause is fixed, so
the need for mid-trial reconnection largely evaporates - and §2.5 shows the GUI makes the reconnect path
structurally unsafe, so this is a bigger project than it first looked. **§2.6 is the zero-cost interim
procedure that makes the hazard moot in the meantime; read that even if nothing here is ever built.**
**Raised by:** the operator, after the watchdog and stall detector made mid-trial recovery possible —
observing that the recovery's real value so far has been *debugging* (no power cycle means evidence
survives), and that it does **not** rescue a real experiment.

**Read first:** `Nano-Hang-Watchdog-And-Breadcrumbs.md` §0 — the disconnect root cause is now known
(connection interval) and the fix is in. This document is about the *residual* problem: what happens to a
trial when the Nano reboots anyway.

---

## 1. The operator's claim, checked against the code

> *"What this doesn't help is in a real experiment if the Nano glitched, I can't save the trial: Teensy
> doesn't handle Nano reconnection. Maybe I want to modify Teensy so it does."*

**Verdict: the premise is right, the consequence is not — and the fix is smaller than feared.**
All of the following is read out of the source, and **none of it is bench-verified.** It should be
confirmed on hardware before anything is built on it.

### 1.1 TRUE: there is no resynchronisation, at all

The Nano is the sole authority on trial status. `ble_commands.h` sets it locally and pushes it to the
Teensy with `UART_command_names::update_status`. The Teensy never asks and never volunteers.

On a Nano reboot the Nano's `ExoData` constructor sets `_status = trial_off` (`ExoData.cpp:17`) and the
Nano **does not send that to the Teensy**. So the two MCUs silently diverge: Nano believes `trial_off`,
Teensy still believes `trial_on`.

### 1.2 RETRACTED 2026-09-12 - the trial data IS lost. The SD logger is switched OFF.

**This section originally said "FALSE, the trial is NOT lost - the Teensy keeps running and SD logging
continues uninterrupted." That was right about the code and wrong about reality. The operator pointed out
that they do not use the SD logger at all.**

**Confirmed in the repo:** `SDCard/config.ini:120` -> `sdLogEnabled = 0`. `SdLogger::update()` reads that
into `_enabled` (`SdLogger.cpp:154`) and then early-returns at `if (!_available || !_enabled) return;`
(`SdLogger.cpp:47`). **No session is opened and nothing is written.**

**Why it is off, and why it is likely to stay off:** the logger introduces unacceptable delays in the Teensy
control loop. Earlier sessions spent substantial effort trying to fix that and concluded it may not be
achievable on this hardware - the SD writes contend with a 500 Hz control loop on a shared bus. See
`SD-Logger-Disabled-Behavioral-Audit.md`, which audits the `sdLogEnabled = 0` path specifically and notes
that even *disabled*, `_begin_if_needed()` still performs a blocking `SD.begin()` on the first `loop()`
iteration.

**So the operator's original statement was correct:**

> *"in a real experiment if the Nano glitched, I can't save the trial"*

**Correct. On a mid-trial Nano reboot, that trial's data is lost** beyond whatever the GUI had already
written to its CSV before the link died. There is no second copy.

**What still stands from the original section:** the **Teensy keeps running** - status stays `trial_on`,
motors stay enabled, the controller keeps executing. That is unchanged and it is why losing the Nano is safe
rather than dangerous (§2.2). It is only the *data* claim that is void.

**And this makes the GUI CSV the sole record of any trial**, which raises the stakes on two things:
§2.7's cheap mitigation (at minimum, do not make the surviving record worse), and the CSV-append idea in
§3 item 5.

### 1.3 FALSE: the Teensy probably needs no change

`get_status` (**0x03**) already exists, and `UART_command_handlers::get_status`
(`uart_commands.h:239`) already replies with the Teensy's own status. The plumbing for "Nano asks the
Teensy what state it is in" is **already implemented on both sides.** The Nano simply never calls it.

And the Nano-side receive path is already safe: `update_status`'s `set_default_parameters()` call is
guarded by `#if defined(ARDUINO_TEENSY36) || defined(ARDUINO_TEENSY41)`, so a Nano adopting a status
just adopts it.

**So the smallest viable change is Nano-side and GUI-side, not Teensy-side.**

---

## 2. Two problems found while checking this that nobody had raised

### 2.1 HAZARD: restarting the trial after a reconnect silently wipes tuned parameters

This is the worst thing in this document and it is live **today**.

```cpp
// uart_commands.h:252
inline static void update_status(UARTHandler *handler, ExoData *exo_data, UART_msg_t msg)
{
    exo_data->set_status(msg.data[...STATUS]);
#if defined(ARDUINO_TEENSY36) || defined(ARDUINO_TEENSY41)
    if (msg.data[...STATUS] == status_defs::messages::trial_on)
    {
        exo_data->set_default_parameters();      // <-- HERE
    }
#endif
}
```

After a mid-trial Nano reboot, the natural operator action is: reconnect, press Start Trial. That sends
`update_status(trial_on)`, and the Teensy **resets every controller parameter to its defaults** — in the
middle of a session, with no indication to the operator that anything changed.

Any tuning done for that trial (and anything the external-control orchestrator or the UDP listener had
set) is gone. The trial continues, and the data afterwards is from a **different controller
configuration** than the data before, with nothing in the log saying so.

**This is a data-integrity bug, not just an inconvenience.** It deserves attention independently of
whether the rest of this document is ever built.

### 2.2 CORRECTED 2026-09-12 - loss of supervision is NOT the hazard. **Reconnection is.**

**An earlier version of this section claimed that a rebooted Nano leaving motors enabled with no
supervisor was a safety hazard, and proposed a Teensy-side dead-man's switch. The operator corrected
this, and they are right. Both the claim and the proposed fix are withdrawn.**

**Why losing the Nano is not dangerous.** The Teensy is running a controller from its own validated set,
with parameters that were correctly received while the link was healthy. Provided each individual
Nano->Teensy message was processed properly - and there is no evidence any was not - **the Teensy can only
ever be operating inside its available, safe configuration space.** Losing the Nano means losing *control*,
not losing *safety*. The trial is over, nothing hazardous happens, and the device can simply be powered
off. That is an inconvenience, not a risk.

**A dead-man's switch would therefore be actively harmful**: it would destroy the one thing that currently
works (§1.2 - the Teensy keeps running and the SD log survives) in exchange for solving a problem that
does not exist. Withdrawn.

**The real hazard is the opposite event. After a reboot the TEENSY HOLDS THE GROUND TRUTH** - live status,
tuned parameters, motor enable, torque-sensor zeros - **and the Nano comes back with a stale, default-ish
copy of all of it.** If the Nano then broadcasts, the Teensy's good state is overwritten by the Nano's bad
state. Nothing is wrong until the Nano speaks; **everything is wrong the moment it does.**

That is why the design question is not *"how does the Teensy handle reconnection"* but
**"should the Nano be allowed to reconnect mid-trial at all, and if so, what is it allowed to say?"**

### 2.3 The injection surface, enumerated from source

**Good news first: the reconnect itself injects nothing.** `ExoCode.ino`'s boot path contains **zero**
`uart_handler->UART_msg(...)` calls - the Nano pushes nothing to the Teensy on its own initiative. Every
mutating command originates in `ble_commands.h`, i.e. **from a GUI action.** A reconnected-but-idle Nano is
harmless.

So the danger is precisely **the first action taken after reconnecting.** There are eight vectors:

| # | UART command | triggered by | what it corrupts |
|---|---|---|---|
| 1 | `update_cal_trq_sensor` | **Calibrate Torque** | **WORST - see 2.4** |
| 2 | `update_status(trial_on)` | **Start Trial** | calls `set_default_parameters()` on the Teensy - wipes all tuning (§2.1) |
| 3 | `update_motor_enable_disable` | Start/Stop Trial | enables or disables motors out of band |
| 4 | `update_controller_params` | controller change | replaces the whole parameter set from the Nano's stale copy |
| 5 | `update_controller_param` | single param edit | **also fired automatically by the external-control orchestrator and the UDP listener** |
| 6 | `update_cal_fsr` | Calibrate FSR | re-zeros FSRs mid-gait |
| 7 | `update_refine_fsr` | Calibrate FSR | as above |
| 8 | `update_FSR_thesholds` | threshold edit | replaces thresholds from the stale copy |

**Vector 5 is the one that needs no human at all.** If the external control orchestrator or the UDP
listener is running - as it is for any externally driven experiment - it will resume pushing
`update_controller_param` on reconnect with nobody pressing anything. That is a silent, automated
corruption path.

### 2.4 The sharpest vector: re-zeroing torque sensors while the motors are driving

`cal_trq` (`ble_commands.h:257`) sets `calibrate_torque_sensor = 1` on every used joint, and the Teensy
re-zeros its torque sensors. **If that happens while the Teensy is actively commanding torque, the loaded
reading becomes the new zero.** Every subsequent torque measurement is then offset by whatever load was
present at that instant, and for a controller closing a loop on the torque sensor - which PJMC and the
spline controllers do - the feedback reference shifts underneath it.

**And this is the operator's habitual reconnect reflex.** The device-manager logs from 2026-09-12 show the
sequence literally every time: `connect()` -> `calibrateTorque()` -> `beginTrial()`. So the most damaging
of the eight vectors is also the first thing a human naturally does after a reconnect.

**This is a genuine safety concern**, and it arrives exactly where the operator said it would: not from
losing the Nano, but from the Nano coming back and broadcasting something new.

### 2.5 THE WORST VECTOR IS MANDATORY, NOT HABITUAL - the GUI enforces it

**Added after the operator pointed out what the GUI actually does:** *"the GUI by design prevents one from
clicking Start Trial before clicking Calibrate Torque."*

So the `connect -> calibrateTorque -> beginTrial` sequence in every log is **not** an operator reflex that
could be trained away. It is **the only path the GUI permits.** Which means:

> **In the current GUI there is no way to reconnect and resume a trial without firing `cal_trq` first** -
> the single most damaging of the eight vectors (§2.4), re-zeroing torque sensors while the motors are
> driving them.

This makes the reconnect path **structurally** unsafe rather than merely risky, and it raises the cost of
doing this properly: any real fix needs GUI state handling, not just a firmware gate. That is the main
reason this work is parked.

### 2.6 INTERIM PROCEDURE - zero cost, and it makes the hazard moot

**The status quo is already safe, provided nobody reconnects *and then acts*.** The hazard requires a
mutating command, and the Nano sends none on its own (§2.3). So:

> **After an unexpected mid-trial disconnect: do NOT reconnect and continue. Treat the disconnect as the
> end of that trial.**
>
> 1. **Do not press anything in the GUI** - in particular not Calibrate Torque, which the GUI will put in
>    front of you first.
> 2. The exo is still running its last valid controller and is **safe** (§2.2). Stop it physically or power
>    it off when convenient.
> 3. **Accept that this trial's data ends here.** The SD logger is off (§1.2), so the GUI's CSV - whatever
>    it managed to write before the link died - is the only record. There is nothing to rescue by
>    reconnecting, which removes the main temptation to do so.
> 4. **Reconnecting purely to READ is safe** - the reset-reason banner, telemetry, nothing mutating. That is
>    exactly what was done all through 2026-09-12 to collect `CPi`/`UPi` readings.
> 5. Restart for the next trial from a clean power cycle.

**The GUI-side record is the ONLY record**, and the portion after the disconnect is genuinely gone - not merely split. That is the real cost of a mid-trial reboot, and it is why the interval fix matters more than any recovery mechanism could.

### 2.7 A cheap interim mitigation, if one is wanted before the full project

Not built, and not required - but it is GUI-only and needs no firmware:

The GUI **already knows** when this situation has arisen: `_mark_disconnected` logs
`Intentional: False`, and it knows whether a trial was active. So on the next connect after an
unintentional mid-trial disconnect it could show a blocking warning - *"the exo may still be running this
trial; do not calibrate or start a trial"* - and optionally grey out Calibrate Torque and Start Trial until
the operator confirms.

That converts §2.6 from a procedure somebody has to remember into something the software enforces, for a
fraction of the cost of the real fix. **It deliberately does NOT need `get_status`**, because it relies only
on host-side knowledge the GUI already has - which is why it is worth considering separately from the rest
of this document.

## 3. Sketch of what a fix would look like

Not a plan - a starting point. **Note the shape has changed since §2.2 was corrected: the thing to gate is
the MUTATING COMMANDS, not the connection.**

1. **Nano, at boot or on BLE connect: send `get_status` (0x03) and adopt the Teensy's reply.** Both ends
   already exist (§1.3). This is what lets the Nano know a trial is live.
2. **If a trial IS live, the Nano refuses to forward any of the eight mutating commands** until the
   operator explicitly takes over. Telemetry, status and the reset-reason banner keep flowing - so the
   reconnect stays fully useful for diagnosis, which is the benefit the operator actually values - while
   the Teensy's ground truth is untouchable by default.
3. **An explicit takeover action**, deliberately awkward, that either (a) resyncs the Nano FROM the Teensy
   and then permits normal operation, or (b) ends the trial cleanly. Anything implicit defeats the point.
4. **Fix §2.1 regardless**, and fix it narrowly: `set_default_parameters()` should not fire on a `trial_on`
   that is a *resume*. A distinct "resume" status, or a flag on the message, keeps today's behaviour for a
   genuine fresh start.
5. **GUI: on reconnect into a live trial, append to the existing CSV** rather than opening a new one, and
   mark the gap (the `_mark_counter` column already exists for annotations). The GUI should also *not*
   offer Calibrate Torque in that state (§2.4).
6. **The automated pushers need the same gate.** The external control orchestrator and the UDP listener
   must be blocked by the same mechanism, since they need no human to fire vector 5.

**Explicitly NOT proposed any more:** a Teensy-side dead-man's switch. See §2.2 - it solves a non-problem
and destroys the behaviour that currently works.

**The genuinely open question is §2.2's last line:** is gating enough, or is forbidding mid-trial
reconnection outright the safer call? Gating keeps the diagnostic value; forbidding is simpler to reason
about and has no partial-failure mode. That is a judgement call for the operator, not a technical one.

## 3b. THE DECISION, 2026-09-12

**Parked.** The operator's reasoning, and it is sound: *"I don't see why we would, if we can fix the
disconnection."*

The disconnect root cause is now fixed and validated (8,002 s clean, zero failures - see §0 of the main
document). Mid-trial reboots should become rare enough that the ability to resume one is not worth a
three-codebase project carrying a safety decision.

**The cost of parking is higher than first written.** §1.2's retraction means a mid-trial reboot loses that trial outright - there is no SD fallback. Parking is still the right call because the fix makes reboots rare, but the decision is "accept occasional total loss of a trial", not "accept a split file". Worth stating plainly so nobody re-reads this later and thinks it was cheaper than it was.

**What remains true even so**, and is the reason §2.6 exists rather than this document simply being deleted:

- **The watchdog and stall detector have not been removed** and will still reboot the Nano if anything else
  goes wrong. So the reconnect hazard is **live today** whenever a reboot happens for any reason.
- **Presentation A is reported upstream under weak signal** (issue #45, §13 of the main document) and that
  path has never been eliminated - only the interval-driven one has. A subject walking away from the laptop,
  or body-blocking the antenna, can still produce a reboot.
- In exactly that situation - subject on a treadmill, mid-experiment - the operator will be most tempted to
  reconnect and carry on. **That is what §2.6 is for.**

## 4. Why this probably wants its own branch

- It spans all three codebases: Nano firmware, Teensy firmware, and the GUI.
- Item 5 is a **safety** decision about a device worn on a leg, and it deserves isolated review rather
  than being folded into a debugging branch.
- `disconnection_troubleshooting` is already carrying a large amount of diagnostic scaffolding that §11 of
  the main document plans to remove. Stacking a behavioural change on top of that makes both harder to
  review and harder to revert.
- Item 2.1 is arguably an **exception**: it is a small, self-contained bug fix with no design questions
  attached, and it could reasonably go in ahead of the rest.

---

## 5. Open questions for the discussion

1. **Gate the mutating commands, or forbid mid-trial reconnection entirely?** Gating preserves the
   diagnostic value of reconnecting (the operator's stated main benefit); forbidding has no partial-failure
   mode. §2.2, §3.
2. Should a resumed trial be one CSV with a marked gap, or two files plus a documented join? One file is
   friendlier to analysis; two are more honest about what happened.
3. Should the Teensy log the Nano reboot into its own SD debug log, so the SD record carries the same
   discontinuity the CSV does? Currently the SD log would show a seamless session that was not seamless.
4. How does this interact with the **external control orchestrator** and the UDP parameter listener, which
   both push parameters continuously? A `set_default_parameters()` wipe mid-session is worse for them than
   for a hand-tuned trial.

---

## 6. What is verified and what is not

| claim | basis |
|---|---|
| No resync exists; Nano boots to `trial_off` without telling the Teensy | **read from source** |
| Teensy keeps `trial_on`, keeps motors enabled, keeps running the controller | **read from source**, and consistent with the operator's 2026-09-10 bench observation |
| ~~Teensy keeps SD logging, so the trial survives~~ | **RETRACTED** - `sdLogEnabled = 0` in `SDCard/config.ini:120`; the logger is off and not used, because it stalls the Teensy loop. The trial data IS lost (§1.2) |
| Teensy continuing unsupervised is SAFE, not hazardous - it stays inside its validated configuration | **operator's domain judgement**, accepted; supersedes this document's earlier claim |
| The Nano pushes NOTHING to the Teensy at boot - all 8 mutating commands are GUI-initiated | **read from source** (`ExoCode.ino` boot path has no `UART_msg` calls) |
| `cal_trq` re-zeros torque sensors, and the GUI's reconnect reflex fires it first | **read from source** + the operator's own 2026-09-12 logs |
| `get_status` 0x03 exists and the Teensy already answers it | **read from source** |
| `set_default_parameters()` fires on a `trial_on` message, Teensy only | **read from source** |
| GUI splits the CSV across a reconnect | **read from source** |
| **Any of the above on real hardware** | **NOT VERIFIED — confirm on the bench before building** |
