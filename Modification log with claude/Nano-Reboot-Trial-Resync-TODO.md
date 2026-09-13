# Nano reboot mid-trial: state resynchronisation — FUTURE TASK, not yet started

**Date raised:** 2026-09-12
**Status:** **DISCUSSION DOCUMENT. No code written. Probably wants its own branch.**
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

### 1.2 FALSE, and this is the important correction: the trial is NOT lost

The Teensy keeps running, and **SD logging continues uninterrupted.**

- `sd_logger.update(ran)` is called unconditionally from the Teensy main loop (`ExoCode.ino:693`).
- `SdLogger::_handle_session()` opens/closes a session purely on edges of
  `status == trial_on || fsr_calibration || fsr_refinement` (`SdLogger.cpp:179`).
- A Nano reboot changes none of that, so `active` stays true, **no `_close_session()` fires**, and the
  motor/ground-strike/debug logs keep being written to the same files.

**So the motor-side record of the trial survives a Nano reboot completely.** This matches the operator's
own bench observation on 2026-09-10: *"Right after it froze, everything is working perfectly. The GUI
froze then disconnected but exo is perfectly working."*

**What is actually lost is the HOST-side record:** the GUI closes its CSV on disconnect and opens a new one
for the next trial, so the BLE telemetry is split across two files with a gap in between. That is a real
loss — but it is a GUI continuity problem, not "the trial is gone."

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

### 2.2 SAFETY: a rebooted Nano leaves motors enabled with no supervisor

Motor enable is a separate UART message (`update_motor_enable_disable`) held in the Teensy's `JointData`.
A Nano reboot does not touch it, so **the Teensy keeps driving commanded torque**.

For an experiment that is arguably the desired behaviour — the trial does not collapse. But it means that
after a Nano reboot the device is actuating a leg with **no reachable supervisor**: the only ways to stop
it are to reconnect the GUI or cut power. The e-stop path (`_data->estop`) should be checked to see
whether it still applies; that was not traced here.

**This needs a deliberate decision rather than a default**, and it is the main reason this work may want
its own branch and its own review.

---

## 3. Sketch of what a fix would look like

Not a plan — a starting point for the discussion.

1. **Nano, at boot or on BLE connect:** send `get_status` (0x03) and adopt the Teensy's reply. One call;
   both ends already exist.
2. **Nano -> GUI:** report the adopted status, so the GUI learns "the exo is already mid-trial" instead of
   assuming a fresh start. The reset-reason `ErrorChar` read at connect is an obvious carrier — it already
   happens at exactly the right moment and already has a `BLESTALL` marker saying a recovery occurred.
3. **GUI:** on reconnect into an already-running trial, **append to the existing CSV** rather than opening a
   new one, and mark the gap explicitly (the `_mark_counter` column already exists for annotations).
4. **Fix 2.1 regardless:** do not call `set_default_parameters()` on a `trial_on` that is a *resume*. The
   cleanest fix is a separate "resume" status or a flag on the message, so a genuine fresh start keeps
   today's behaviour.
5. **Decide 2.2 explicitly:** either a Teensy-side dead-man's switch (no Nano contact for N seconds ->
   ramp torque to zero) or a documented decision that the trial continuing unsupervised is acceptable.
   **Note that a dead-man's switch directly contradicts 1.2's benefit** — it would end the trial that
   currently survives. This is a genuine trade-off, not an oversight to be fixed.

---

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

1. Is "the trial survives a Nano reboot" actually what we want, or should a lost supervisor stop the exo?
   §1.2 and §2.2 pull in opposite directions and **both cannot be satisfied.**
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
| Teensy keeps `trial_on`, keeps motors enabled, keeps SD logging | **read from source**, and consistent with the operator's 2026-09-10 bench observation |
| `get_status` 0x03 exists and the Teensy already answers it | **read from source** |
| `set_default_parameters()` fires on a `trial_on` message, Teensy only | **read from source** |
| GUI splits the CSV across a reconnect | **read from source** |
| **Any of the above on real hardware** | **NOT VERIFIED — confirm on the bench before building** |
