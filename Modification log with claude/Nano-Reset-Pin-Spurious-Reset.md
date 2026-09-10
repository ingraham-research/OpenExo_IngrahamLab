# Nano spurious resets — RESETREAS says RESETPIN

**Date:** 2026-09-09
**Scope:** The mid-trial freeze / "unexpectedly disconnected". First bench data from the `RESETREAS`
instrument, flashed for the first time today.
> ## !! CORRECTION 2026-09-09 (evening) - READ THIS FIRST !!
>
> **The central conclusion below is CONFOUNDED and should not be acted on.** The user pointed out that
> the gaps between an End-Trial `'Z'` and the next reading are time the exo spent **powered OFF**, not
> idle. They confirmed there was no 3.3-hour session.
>
> That matters because of what we did *not* see: a genuine nRF52840 power-on clears `RESETREAS` to zero,
> which this firmware reports as `PORBOR`. **`PORBOR` appears zero times in 13 readings**, while the user
> was definitely power-cycling. The only consistent explanation is that **this board asserts nRESET at
> power-up** (a reset supervisor or RC on the reset net), so `RESETPIN` is simply what a normal power-on
> looks like here - and carries no diagnostic information at all.
>
> **What this retracts:**
> - Section 5b's "the resets happen while the exo is idle" - **withdrawn entirely.** Those gaps are
>   power-off time.
> - Section 2's "never `LOCKUP`, therefore not a firmware crash." **Withdrawn.** The firmware latches and
>   clears `RESETREAS` at each boot, so a power cycle after a freeze overwrites the freeze's real reason
>   before it is ever read. A `LOCKUP` would have been erased. **The Mbed-hardfault and ArduinoBLE
>   `while (_pendingPkt >= _maxPkt)` spin hypotheses are back in play.**
> - The leakage/EMI theory is **unsupported** by this data. It may still be true; nothing here shows it.
>
> **What still stands:**
> - The two `SREQ` readings, taken 15 s and 20 s after an End-Trial `'Z'`. Those prove the readout works
>   and that End Trial performs a clean `NVIC_SystemReset` as designed.
> - Everything in `Mid-Trial-Freeze-Nano-Radio-Silence.md` about the 9.6 s supervision timeout and the
>   Teensy surviving. That was measured independently.
>
> ### CALIBRATED 2026-09-09 (evening) - the confound is CONFIRMED, and the readout is usable again
>
> The user ran the calibration. Result:
>
> | Action | Reads |
> |---|---|
> | **Power the exo on** | **`RESETPIN`** |
> | Connect, disconnect, reconnect | `SREQ` |
>
> So `RESETPIN` really is this board's normal power-on signature - it asserts nRESET at power-up - and
> the retractions above stand.
>
> **But the instrument is not useless. It just needs one operating rule:**
>
> > **After a freeze, do not power-cycle, and make the FIRST reconnect the one you read.**
>
> `RESETREAS` is latched and write-1-cleared at every boot, and the resulting string sits in `ErrorChar`
> until overwritten. So the first connection after a freeze carries that freeze's true cause. A power
> cycle replaces it with `RESETPIN`; a further reset replaces it again. Under that rule a reading of
> `LOCKUP` would be real evidence of a firmware crash, and a `RESETPIN` with no power cycle in between
> would be real evidence of a spurious pin assertion.
>
> ### RESOLVED, and it points at a HANG rather than a reset (2026-09-09, late)
>
> The loose end below is closed: **this GUI has no Disconnect control at all - End Trial is the only way
> to drop the link.** So the "connect / disconnect / reconnect -> `SREQ`" test was really connect ->
> **End Trial** -> reconnect, which sends `ble_names::reset_system`. Code and observation agree exactly,
> with nothing unexplained.
>
> That closure has a much larger consequence. **The Nano has exactly two reset paths:**
>
> 1. power-on -> `RESETPIN`
> 2. End Trial (`reset_system` -> `ComsMCU::_schedule_system_reset()`) -> `SREQ`
>
> There is **no watchdog** anywhere in the firmware (verified by grep), and a plain BLE link drop does not
> reset the Nano. **So during a freeze, nothing in the firmware is capable of resetting it.** A hardware
> pin assertion or brownout could - but either would reboot the Nano and leave it advertising and
> connectable, and the user reports that after a freeze they generally **cannot** reconnect and are forced
> to power-cycle.
>
> **Therefore the Nano is most likely not resetting during a freeze at all - it is HANGING.** A hung MCU
> never touches `RESETREAS`, which is exactly why every reading ever taken has been the recovery power
> cycle. This also fits the earlier log mining: absent from scans for 7.4-413 s, and 14 entries where the
> PC connected but the Nano failed to serve its GATT table. Neither is the profile of a clean reboot.
>
> This argument rests on verified code structure plus the operational fact, **not** on the confounded
> readings, so it should be weighted above everything claimed earlier in this document. It means the
> spurious-pin-reset and leakage/EMI theories are very likely **dead as an explanation for the freezes**,
> and the Mbed-hardfault / ArduinoBLE-spin hypotheses are the live ones again.
>
> **What follows from it:**
> - `RESETREAS` cannot answer this question. Neither can `GPREGRET` (cleared by power-on) or `ErrorChar`
>   (RAM). **Nothing on the Nano survives the forced power cycle.**
> - The instrument must live on the **Teensy** (proven to survive the event) and write to the **SD card**
>   (survives the power cycle). The Teensy currently has **no Nano-liveness tracking at all** - verified.
> - A **watchdog on the Nano** would both mitigate and identify: it turns a hang into an automatic
>   recovery (no power cycle) and makes `RESETREAS` read `DOG`, which the GUI already decodes. The care
>   needed is where the kick lives - fed from a Cordio RTOS thread that survives the hang, it would never
>   fire.
>
> **Original loose end, now answered:** the code path that makes the Nano call `NVIC_SystemReset` is
> `ComsMCU::_schedule_system_reset()`, reached only from `ble_names::reset_system` - the GUI's End-Trial
> shutdown command (`ComsMCU.cpp:364`). A *plain* BLE link drop should not reset the Nano at all. So the
> `SREQ` seen after a bare connect/disconnect/reconnect is not fully explained: either the GUI's
> disconnect flow also sends `reset_system`, or an End Trial was in the sequence. Worth confirming, since
> it decides whether a disconnect quietly overwrites the evidence.
>
> **Two things to do before trusting any of this again:**
> 1. **Calibrate the instrument.** Deliberately power-cycle, connect, read. `RESETPIN` confirms power-on
>    is indistinguishable from a pin reset on this board. `PORBOR` would mean the readings *are* clean and
>    only section 5b needs withdrawing.
> 2. **Fix the confound in firmware.** `NRF_POWER->GPREGRET` is retained across a warm reset but cleared
>    by a true power-on/brownout. Writing a magic value there at boot and checking it on the next boot
>    separates "the user turned it on" from "it reset while powered" - which is the distinction this whole
>    investigation actually needs, and which `RESETREAS` alone cannot give us on this board.
> 3. **The one clean reading available today:** after a freeze, do NOT power-cycle. Let it re-advertise on
>    its own, connect, and read. That value is uncontaminated.

**Status:** **CONFOUNDED - see the correction above. Root cause NOT established.** (Original status:
Root cause NARROWED, not proven.) No code change made for this yet. The evidence below
is from logs, is reproducible, and rules out the three hypotheses the investigation was previously
built on. The remaining mechanism is a hardware question that needs a schematic check.

---

## 1. What the instrument said

Five connects on 2026-09-09, from `Python_GUI/Saved_Data/logs/device_manager_20260909_113917.log`
and `..._121333.log`:

| Connect | Preceding event | `RESETREAS` |
|---|---|---|
| 11:48:12 | the re-flash | `RESETPIN` — expected, discard |
| 11:50:42 | deliberate End-Trial `'Z'` | **`SREQ`** ← control case |
| 11:55:23 | spontaneous freeze @ 11:52:39 | **`RESETPIN`** |
| 12:00:46 | spontaneous freeze @ 11:57:29 | **`RESETPIN`** |
| 12:14:59 | spontaneous freeze @ 12:02:02 | **`RESETPIN`** |

All three freezes logged `Reason: link lost, Intentional: False` with no `'Z'` beforehand, at 108 s,
114 s and 54 s into their trials.

**The `SREQ` row is what makes the rest trustworthy.** The same hardware, an hour apart, correctly
distinguished a software reset from a pin reset. The readout is not stuck.

## 2. What this rules out

- **Never `LOCKUP`.** No hard fault, no Mbed fault-handler escalation. This kills the "Nano MCU
  crash" hypothesis **and** the ArduinoBLE `while (_pendingPkt >= _maxPkt) poll();` spin theory
  (`HCI.cpp:418`), which were the two leading candidates in `Mid-Trial-Freeze-Nano-Radio-Silence.md`.
- **Never `PORBOR`.** Not a brownout. Note this also disposes of that document's stated ambiguity:
  a hand power-cycle reads `PORBOR` too, so `RESETPIN` cannot be produced by anyone touching the
  power. Power loss and pin reset are now cleanly separable.
- **Not a hang.** At 12:00:39 the GUI connected straight to the saved MAC with no scan and succeeded
  in 6 s, so the Nano was already advertising. It rebooted and recovered unaided.

Three things can set `RESETPIN`: the button, a re-flash, or **the nRESET line being pulled low**.
The board is worn inside a closed box with nothing near it, and no re-flash happened between 11:52
and 12:15. That leaves the third.

## 3. The mechanism, and the one thing to check first

nRF52840 nRESET is P0.18, held high only by a pull-up. A high-impedance node on a board carrying
motor cables, CAN traffic and switching supplies is exactly what couples in an unintended low. The
user has seen this same failure on a lab RPi, where leakage into the reset pin produced spurious
"system will shut down now" events.

**Concrete lead, needs the schematic to confirm.** The board is `AK_Board_V0_5_1` (`Config.h:24`).
That block defines, for the Teensy, in the inter-MCU SPI group:

```
ExoCode/src/Board.h:423    const unsigned int rst_pin = 4;
```

**Nothing in the firmware ever reads or writes it** — verified by grepping every `.cpp`/`.h`; there
are zero uses outside the declaration. An unconfigured Teensy 4.1 pin is a high-impedance input. If
that trace reaches the Nano's RST on the V0_5_1 PCB, the reset line is a floating conductor running
across a noisy board, which is the mechanism.

**This has not been confirmed against the schematic.** Do that before acting on it.

## 4. Options, cheapest and most reversible first

**Step 0 — confirm the wiring.** Does Teensy pin 4 reach the Nano nRESET on V0_5_1? Everything below
branches on this.

**Option A — hold the line, one line of firmware.** If pin 4 is wired to nRESET:
`pinMode(logic_micro_pins::rst_pin, INPUT_PULLUP);` in Teensy setup. Adds a ~20-50 kΩ pull-up,
removes the floating condition, needs no hardware work, and is trivially reversible. Prefer
`INPUT_PULLUP` over driving the pin `OUTPUT HIGH`: if the Nano's own reset button pulls that node to
GND, a push-pull output would be shorted on every button press.

**Option B — instrument it instead of fixing it.** Attach a `FALLING` interrupt to the same pin,
latch a flag plus the exo timestamp, and report it. This *proves* or *disproves* the mechanism rather
than masking it, and catches glitches far too short for a 500 Hz poll. Best evidence per unit of
effort if Option A's result is ambiguous.

**Option C — hardware.** External 4.7 kΩ pull-up plus 100 nF to GND on RST. The classic fix for
coupled noise on a reset line. Needs soldering; not reversible in software.

**Option D — disable the reset pin outright (the RPi-style band-aid the user asked about).** On
nRF52840 the reset function is selected by `UICR.PSELRESET[0]`/`[1]`. Clearing them makes P0.18 an
ordinary GPIO and there is no hardware reset at all — a low on that pin then does nothing.

> **Read this before choosing D.** Disabling nRESET removes **double-tap-reset bootloader entry**,
> which is the documented way to get a Nano 33 BLE into its bootloader. The IDE's 1200-baud-touch
> auto-reset is a *software* path (the running sketch's USB CDC triggers it) and should still work —
> **but verify that on a spare board before committing the worn unit**, because if the sketch ever
> hangs with USB dead, recovery needs SWD hardware. UICR is also written by flash-page erase, so
> this is not a casual toggle.
>
> Recommendation: treat D as the *confirmation* test, not the first move. A and B are cheaper, safer
> and more informative. If A stops the disconnects, the diagnosis is made and D is unnecessary.

## 5. Caveat on the correlation with torque

Today's freezes came at 54-114 s into a trial; the historical stress tests in
`Mid-Trial-Freeze-Nano-Radio-Silence.md` ran to 606-609 s. Today was also the first day on the raised
torque ceiling (`MAX_JOINT_TORQUE_NM` 25 -> 30, spline feed-forward 15 -> 25 — see
`External-Control-Orchestrator.md`).

**But the user did not actually raise the commanded torque today**, and the session was interrupted
external-control testing rather than steady walking — which plausibly makes electrical transients
*more* frequent, not less. So the shorter intervals are not evidence about torque. The baseline
freeze-interval distribution across the 86 historical trials has not been computed; until it is,
treat "freezes got faster" as unquantified.

## 5b. The resets happen while the exo is IDLE (added 2026-09-09, evening)

The user asked why a **cleanly ended trial** still reads `RESETPIN` on the next connect. Correlating all
13 readings from the day against the End-Trial `'Z'` that preceded them answers it, and the separation
is total:

| Reading | Reason | Gap since the End-Trial `'Z'` |
|---|---|---|
| 11:50:42 | **`SREQ`** | **20 s** |
| 12:52:40 | **`SREQ`** | **15 s** |
| 12:21:13 | `RESETPIN` | 273 s |
| 13:25:57 | `RESETPIN` | 430 s |
| 13:18:15 | `RESETPIN` | 665 s |
| 13:06:48 | `RESETPIN` | 809 s |
| 17:07:03 | `RESETPIN` | 1102 s |
| 12:14:59 | `RESETPIN` | 1477 s |
| 12:50:42 | `RESETPIN` | 2042 s |
| 16:43:38 | `RESETPIN` | 11786 s (3.3 h) |

**Both `SREQ` readings are the two shortest gaps. Every gap of 4.5 minutes or more reads `RESETPIN`.**

So End Trial *does* work exactly as designed: the Nano performs a clean `NVIC_SystemReset`, and if you
reconnect within ~20 s you see that `SREQ`. `RESETREAS` is latched and cleared at every boot, so the
value always describes the **most recent** boot. Reading `RESETPIN` minutes later therefore means a
**new, pin-caused reset happened after the clean one** — while the exo sat idle.

**This is the important part: the spurious resets are not correlated with motor activity, walking, BLE
traffic, or trials at all.** They occur with the device powered and doing nothing, at least once in
every idle gap of 4.5 min or longer, including one 3.3-hour gap. That argues against EMI from motor
switching and toward something constant — leakage, a marginal or floating connection on the reset net,
or the exposed `RESET` header.

Also worth noting: every code is `0x00000001`, i.e. `RESETPIN` **alone**. Never `0x00000005`
(`SREQ|RESETPIN`), which is what a Teensy reset propagating onto a shared reset line during the
End-Trial sequence would have produced. The two events are separate.

**This makes the diagnosis far cheaper to chase.** No participant, no walking, no motors:

> Power the exo. Leave it completely alone for ~15 minutes without connecting. Then connect and read
> the reason. If it says `RESETPIN`, the Nano reset itself while doing literally nothing — which
> isolates the fault from every software and motor variable in one step. Repeat with the `RESET`
> header unplugged/shorted appropriately, or with Option A applied, to bisect.

## 6. What would settle it

1. Schematic: is Teensy pin 4 on the Nano nRESET net?
2. Option B's falling-edge counter — does the line actually dip, and when relative to motor activity?
3. Option A for a few trials — do the disconnects stop?
