# Mid-Trial Plot Freeze and Auto-Disconnect — the Nano Goes Radio-Silent

**Date:** 2026-09-02
**Scope:** Diagnosis spans `Python_GUI/services/QtExoDeviceManager.py`, `Python_GUI/MainWindow.py`,
`ExoCode/src/ExoBLE.cpp`, `ExoCode/src/ComsMCU.cpp`, `ExoCode/src/RealTimeI2C.cpp`,
`ExoCode/src/uart_commands.h`, and the vendored/installed `ArduinoBLE`. Code changed:
`ExoCode/src/SystemReset.h`, `ExoCode/src/ExoBLE.cpp`, `Python_GUI/services/QtExoDeviceManager.py`,
`Python_GUI/MainWindow.py`.
**Status:** **The trigger is fully explained; the root cause is NOT yet identified.** This document
narrows the search from "random BLE flakiness" to "the Nano's MCU stops or resets", and ships an
instrument to close the remaining gap. The instrument is committed as `8f79871` on
`disconnection_troubleshooting` and **compiles clean for both `arduino:mbed_nano:nano33ble` (4.6.0)
and `teensy:avr:teensy41`** — but it has **never been flashed and never been bench-validated.**
No motors were run for any of this work; the entire diagnosis is retrospective analysis of
182 device-manager logs and 86 trial CSVs already on disk.

**Read alongside:** `BLE-Handshake-Controller-List-Loss.md` (the same link, the same RF marginality,
and the handshake burst this change deliberately avoids contending with),
`Motor-Freeze-Controller-Change-And-End-Trial.md` and `End-Trial-Diagnosis-Correction.md` (the
End-Trial `'Z'` reboot, which turns out to be the control experiment for this investigation), and
`Spline-Run-Analysis-And-RT-Stream-Fix.md` (the Teensy→Nano→GUI real-time stream whose loss rate is
measured again here).

---

## Symptom

Mid-trial, the GUI's plots stop updating while the buttons stay clickable. Some seconds later the
GUI pops up "The device has been unexpectedly disconnected", saves the CSV and returns to the Scan
page. Reported as random: it happens regardless of the distance between the Nano and the PC, and
without any obvious trigger. Under deliberate stress testing it tends to happen near the ten-minute
mark; otherwise it is infrequent and unpredictable.

---

## 1. What actually fires the auto-disconnect

There is exactly one path, and the GUI does not choose to take it:

```
Bleak / WinRT  ->  _disc_cb()                        QtExoDeviceManager.py:323-329
               ->  _mark_disconnected("link lost")   QtExoDeviceManager.py:164
               ->  disconnected.emit()               QtExoDeviceManager.py:178
               ->  MainWindow._on_dev_disconnected() MainWindow.py:1179
                     |- popup "unexpectedly disconnected"   MainWindow.py:1166 / 1204
                     |- stack.setCurrentWidget(scan_page)
                     `- flush + close the trial CSV
```

*(Line numbers are as of this commit and include the changes described in §7 — they will drift.)*

`_disc_cb` is the `disconnected_callback` handed to `BleakClient`. It runs only when the **Windows
Bluetooth stack** declares the link dead. The GUI is purely reactive.

### The GUI cannot be the cause

Every timer in `Python_GUI/` was checked. There is **no data-loss watchdog, no stall timer, no
keepalive, and no auto-disconnect-on-silence anywhere.** The only other producer of a disconnect is
`_ensure_connected()` (`QtExoDeviceManager.py:661`), which is passive — it runs only when a command
is sent, i.e. when you press a button.

Across all **182 device-manager logs / 129 disconnect events**, the logged reason was `link lost`
**100 % of the time**. Zero `stale client`, zero `event loop died`. The GUI has never once initiated
one of these.

---

## 2. The measurement that pins it down: a 9.6 s constant

Cross-referencing the 86 trial CSVs against the logs, and isolating drops with no `'Z'` and no
manual disconnect in the preceding 60 s, gives **12 genuinely spontaneous mid-trial drops**. For the
8 with a matching CSV, the interval from *last data sample received* to *disconnect callback* is:

```
9.58  9.58  9.60  9.60  9.61  9.61  9.61  9.61   seconds
```

30 ms of spread, across events 8 days apart. Every normally-ended trial closes its CSV within 0.1 s
of its last row, so the cluster is unambiguous.

That is not a network effect — it is a **hard timer**, and the only hard timer in the chain is the
**BLE link supervision timeout**. This matters because of what a supervision timeout means: it fires
only when *nothing at all* arrives at the radio level, not merely when application notifications
stop. **If the firmware had stopped streaming while staying connected, the link would survive
indefinitely and there would be no disconnect at all.**

Supporting facts: the firmware never calls `BLE.setSupervisionTimeout`, so
`L2CAPSignaling._supervisionTimeout` stays 0 and whatever Windows proposes is accepted;
`ExoBLE.cpp:265` sets `BLE.setConnectionInterval(6, 6)` = a 7.5 ms interval with slave latency 0.

### The control experiment was already in the data

End Trial sends `'Z'`, which deliberately reboots the board — a known-good "peripheral goes
radio-silent" event. All 71 of them:

```
'Z' sent -> link lost:   9.6 … 10.0 s   (median 9.90)
```

**The mystery freeze has the same fingerprint as a deliberate reboot.**

---

## 3. Localising the failure: the Teensy is alive, the Nano dies

`rx_msg.data[11] = (float)millis()/1000` (`uart_commands.h:609`) — the channel the GUI labels
**"Exoskeleton time" is the Teensy's clock**, not the Nano's. In every freeze it advances smoothly
and in real time right up to the final delivered sample.

**The control board never stops.** Motors, sensors, CAN and the control loop are exonerated. The
failure is entirely on the Nano or the link.

### A hung `loop()` cannot explain it — and this is the useful part

On the Nano 33 BLE, ArduinoBLE does **not** run the link layer in the Arduino loop.
`HCICordioTransport.cpp` spawns a **separate RTOS thread** (`bleLoopThread`) that drives
`wsfOsDispatcher()` and the WSF timers.

So if the Arduino `loop()` merely hung, the Cordio thread would keep the link alive with empty PDUs,
and the result would be **a frozen plot with no disconnect, forever.** We always get the disconnect.
Therefore **the whole MCU stops or resets** — a chip-level event, not an application-level stall.

Two corroborating firmware facts: there is **no watchdog** anywhere in the firmware, and the
firmware's own reset path (`ComsMCU::_maybe_system_reset`, `SystemReset.h`) is reachable only via
the GUI's `'Z'` command, which was not sent.

> **Version note.** The above was first read in the repo's vendored `Libraries/ArduinoBLE`
> (**1.2.1**) and then re-verified against the sketchbook's **2.1.0**, which is what actually
> compiles. `bleLoopThread`, the `sendAclPkt` spin and `_supervisionTimeout(0)` are identical in
> both. See §8.

---

## 4. It is not RF range or interference

This was the obvious first explanation and the evidence is against it.

**The exo never comes back instantly.** Time from drop to the exo reappearing in a scan (these are
*upper* bounds — capped by when the operator clicked Scan):

```
7.4   9.1   24.0   42.9   44.9   54.9   82.7   177.8   413.0   seconds
```

Minimum 7.4 s, while the same scans were finding **39–51 other BLE devices** throughout. A fade ends
when the fade ends; the peripheral keeps advertising and reappears in the next scan window. Something
absent for 40+ s while the adapter happily enumerates 50 neighbours is **down, not shadowed**.

**The Nano sometimes accepts a connection and then fails to serve its own GATT table.** Across the
logs:

```
8 x  "Characteristic 6e400003-b5a3-f393-e0a9-e50e24dcca9e was not found!"
6 x  "Could not start notify on 000B: Unreachable"
2 x  "Could not write value b'$' to characteristic 000E: Unreachable"
```

In one case it connected at 16:16:14 and hung for 30 s before failing the notify subscribe. Distance
cannot produce that — a device out of range fails to connect; it does not connect and *then* fail to
describe itself. That is a half-alive BLE stack.

**Fatal stalls never recover, ordinary ones always do.** The RF-looking stalls in surviving trials
top out around 6.3 s and come back. The fatal ones go to zero and stay.

---

## 5. The ten-minute ceiling, and chronic stream loss

Two trials reached **606.0 s** and **609.4 s**. **Nothing in all 86 trials ever exceeded 609.4 s.**
Both died mid-stride at full rate with status still `2.0` (`trial_on`) — no taper, no shutdown.

Loss rate (Teensy samples produced vs. delivered to the GUI), per 60 s bucket:

| Trial | Profile |
|---|---|
| `trial_20260828_152438` | 11 13 11 9 10 7 8 6 9 10 % — flat, dies at 606 s |
| `trial_20260828_153903` | 37 32 32 36 29 27 43 47 40 43 % — noisy, dies at 609 s |
| `trial_20260820_161959` | 31 → 34 → **57 → 58 → 75 %** — climbs monotonically, dies at 275 s |

Two independent findings here. First, **even the best run loses 6–13 % of samples and one loses
~40 % continuously** — the real-time stream is marginal all the time, not only at failure. Second,
the third trial shows backpressure building right into the crash.

A mechanism that ties those together: `real_time_i2c::on_receive` overwrites `byte_buffer` whether or
not `poll()` has consumed the previous packet, so **any slowdown in the Nano superloop silently drops
I2C packets**. Loss rate is therefore a proxy for how blocked the Nano's loop is.

---

## 6. Where the root cause probably is (NOT established)

Ranked, with the honest status of each:

1. **Nano MCU fault** — an Mbed hard fault / lockup / RTOS error halts or reboots the chip. Fits the
   abrupt, taper-free death and the need for a manual power cycle. **Untested.**
2. **The unbounded spin in `HCI.cpp` `sendAclPkt`:**
   ```cpp
   while (_pendingPkt >= _maxPkt) {   // both uint8_t
     poll();
   }
   ```
   `_pendingPkt` only decrements when the Cordio thread delivers Number-Of-Completed-Packets events
   (`handleNumCompPkts`). The firmware pushes a notification every 9 ms
   (`Config.h` `_real_time_msg_delay = 9000`) onto a 7.5 ms connection interval. When completions
   lag, `send_message` blocks here. **Caveat, stated plainly: backpressure alone should wedge the
   loop while leaving the link alive.** For a supervision timeout it must escalate — the spin
   starving the same-priority Cordio thread, or a fault underneath it. That gap is not closed.
3. **Brownout on the Nano.** Abrupt and random, and would also be masked by any USB-tethered test.

---

## 7. What was built: the reset-reason readout

The nRF52840 latches why it last reset in `POWER->RESETREAS`. Reading it on the next connect
distinguishes all three hypotheses without a cable — which matters, because **USB-tethering the Nano
for a serial crash dump would confound the investigation twice over**: it moves the Nano next to the
PC (changing RF geometry) *and* powers it from the host (masking a brownout).

### Design decisions

- **Reuse `ErrorChar`, do not add a characteristic.** Windows caches GATT tables per device; a new
  characteristic can be served stale from that cache and read back as missing. Nothing about the
  table changes.
- **Read, not notify.** The GUI reads it once right after `client.connect()` and **before**
  `start_notify`. That window is idle, so the round trip cannot contend with the handshake payload
  burst — the burst that `BLE-Handshake-Controller-List-Loss.md` shows already drops controller rows
  on ~20 % of connections. Do not move this read later.
- **Latch once and clear.** `RESETREAS` is cumulative across resets until written back, so it is
  captured on first call and cleared, leaving the register clean to describe the *next* reset.
- Note `send_error(0, 0)` in `ExoBLE::setup()` is a no-op at that point (it early-returns while
  `_connected` is 0), so the new direct write is what `ErrorChar` actually holds at boot.

### Files

| File | Change |
|---|---|
| `ExoCode/src/SystemReset.h` | `exo_reset_reason_code()` latches + clears `RESETREAS`; `exo_reset_reason_string()` formats `RST:0x<hex>:<NAMES>`. Carries the interpretation table as a doc comment |
| `ExoCode/src/ExoBLE.cpp` | `setup()` parks that string in `ErrorChar` |
| `Python_GUI/services/QtExoDeviceManager.py` | New `resetReasonReceived` signal; reads `ERROR_CHAR_UUID` in the idle window, 3 s timeout, never fatal |
| `Python_GUI/MainWindow.py` | `_on_reset_reason` prints a terminal banner and logs at WARNING |

Two compile hazards were found by hand-audit before the toolchain was located, and both are real:
CMSIS defines `NVIC_SystemReset` as a **macro** aliasing a `__STATIC_INLINE`, so including `nrf.h`
before the existing `extern "C"` declaration would expand it into a non-static redeclaration of a
static inline (fixed by including first and guarding with `#ifndef`); and `"0" + hex` — a
`const char*` on the left of a `String` — has no overload in Arduino's `WString.h` and does not
compile (replaced with `snprintf`).

### How to read it at the bench

The banner prints at connect, before you would touch Calibrate Torque:

```
====================================================================
 NANO LAST RESET REASON
 RESETREAS = 0x00000008  (LOCKUP)
   LOCKUP: CPU LOCKUP - a hard fault escalated. This is a firmware crash.
====================================================================
```

| Value | Meaning |
|---|---|
| `LOCKUP` | Hard fault escalated — a firmware crash. Confirms hypothesis 1 |
| `SREQ` | Software reset. **Expected** right after an End Trial `'Z'`; **unexpected** after a mystery freeze, where it would suggest an Mbed fault-handler auto-reboot |
| `PORBOR` | Power-on or brownout — **ambiguous**, see below |
| `RESETPIN` | Reset button, or a re-flash |
| `DOG` | Watchdog — but this firmware configures none, so investigate |

### Known limits of the instrument

- **`PORBOR` is ambiguous.** A Nano that *hangs* with its radio dead and is then power-cycled by
  hand reads back identically to a brownout. Disambiguate by recording whether the exo re-advertised
  on its own. The GUI change that would measure this automatically — auto-scan on unexpected
  disconnect — **was proposed but not built.**
- **It reports why the Nano last *reset*.** If the true failure is a hang with no reset, this
  instrument shows the *previous* reset, not the fault.
- **If every connect reports `PORBOR`, including straight after a re-flash**, suspect the mbed
  bootloader is clearing `RESETREAS` before `setup()` runs. The instrument is then blind and
  `GPREGRET` (a retained register that survives soft reset, usable as a breadcrumb) is the fallback.

### The other clean test, not yet run

Raise `_real_time_msg_delay` from 9000 to ~20000 µs (≈50 Hz) in `Config.h` for one run. No proximity
confound. If the ~607 s ceiling moves out or disappears, the failure is load-dependent and hypothesis
2 gains a lot of weight; if it dies at the same time regardless, it is time-dependent and both
hypothesis 2 and the loss-rate story are wrong.

---

## 8. Toolchain trap found along the way

**The repo's vendored `Libraries/ArduinoBLE` is 1.2.1. The build uses the sketchbook's 2.1.0.**
Editing the vendored copy has no effect on the firmware. Every claim in §3 was re-verified against
2.1.0 and holds, but anyone reasoning about BLE internals from the repo copy is reading the wrong
source.

For the record, this firmware **does** compile on this PC. Arduino IDE 2.x redirects its data
directory via `~/.arduinoIDE/arduino-cli.yaml` to `F:\Random storage\Arduino15`, so the bundled
`arduino-cli` must be invoked with `--config-file` or it reads an empty default and falsely reports
"No platforms installed":

```
ACLI="/c/Program Files/Arduino IDE/resources/app/lib/backend/resources/arduino-cli.exe"
CFG="$HOME/.arduinoIDE/arduino-cli.yaml"
"$ACLI" --config-file "$CFG" compile --fqbn arduino:mbed_nano:nano33ble ExoCode/ExoCode.ino
"$ACLI" --config-file "$CFG" compile --fqbn teensy:avr:teensy41    ExoCode/ExoCode.ino
```

Both are needed for a change to `SystemReset.h`, because `uart_commands.h` includes it and compiles
on both boards. Whole-firmware builds emit many pre-existing warnings in untouched files
(`Board.h`, `Utilities.cpp`, `Logger.h`, `UART_msg_t.h`, `error_types.h`) — filter with
`grep -E ": (error|fatal error):"`.

---

## What would falsify this document

- A freeze whose CSV-to-disconnect interval is **not** ~9.6 s would break §2 and mean there is more
  than one failure mode in play.
- Exo time (the Teensy clock) **stalling** before a freeze would break §3 and move the fault back to
  the control board.
- A freeze with **no disconnect at all** — plots frozen but the link alive — would be the signature
  of the loop hanging while Cordio survives, and would point squarely at hypothesis 2 without the
  escalation step §6 admits it cannot explain.
