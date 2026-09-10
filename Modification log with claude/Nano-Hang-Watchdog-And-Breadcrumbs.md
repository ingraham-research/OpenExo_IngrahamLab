# Nano hang: watchdog + loop breadcrumbs

**Date:** 2026-09-10
**Branch:** `disconnection_troubleshooting`
**Status:** **Implemented, compiles clean for `arduino:mbed_nano:nano33ble` AND `teensy:avr:teensy41`,
NOT flashed, NOT bench-validated.** Nothing committed.
**Covers TWO changes:** the hardware watchdog + breadcrumbs (§3.1-3.4), and the **BLE link-stall
detector** (§3.5), which adds a **GUI<->firmware protocol dependency** (a 2 s ping) and therefore
touches `Python_GUI/` as well as `ExoCode/`. Both are temporary.
**Intended lifetime:** TEMPORARY. This is diagnostic scaffolding for the mid-trial freeze. §5 is a
complete, mechanical removal procedure — read it before deleting anything by hand.

---

## 0. TL;DR

The mid-trial Nano freeze was **proven to be a hang** — not a crash, not a reset, not a brownout.
Nothing in the firmware could ever recover it, and the only recovery available (a power cycle)
destroyed the evidence every time. This change adds the nRF52840 hardware watchdog so a hang
**self-recovers warm in ~5 s**, plus a one-byte breadcrumb so the next boot reports **which phase of
`loop()` it died in**.

This does not fix the bug. It converts an unrecoverable dead end into a data source. Once the root
cause is found and fixed, **all of it should come out** — see §5.

---

## 1. The finding: it is a HANG (this is the part worth keeping)

Three sessions of instrumentation had failed to identify the freeze. 2026-09-10 settled it, on two
independent channels.

### 1.1 The reset register, read warm for the first time

Previously every recovery was a power cycle, and a true power-on clears both `RESETREAS` and
`GPREGRET` — so every reading ever taken described the recovery, not the freeze (see
`Nano-Reset-Pin-Spurious-Reset.md` for that confound).

This time the user pressed the Nano's **RESET BUTTON** instead. A pin reset is *warm*: both registers
survive. The reading came back **`0x00000001` — `RESETPIN` alone**.

The firmware zeroes `RESETREAS` on every boot (`SystemReset.h`, `exo_reset_reason_code()`), so any
reset occurring during the freeze would have left its bit set. None had:

| bit | meaning | present? | therefore |
|---|---|---|---|
| `LOCKUP` | CPU double-faulted into hardware lockup | no | no lockup |
| `SREQ` | software reset | no | `mbed_error_hook`'s `NVIC_SystemReset` never ran |
| `DOG` | watchdog | no | (expected — none was configured) |
| `0x0`/`PORBOR` | power-on or brownout | no (`0x1` is not `0x0`) | no brownout, no power loss |
| `RESETPIN` | pin reset | **yes** | the button, and nothing else |

**Nothing reset the MCU. It stopped and stayed stopped until a thumb.** The absent `CRASH_0x` record
is consistent: a hang never calls `mbed_error()`, so nothing is ever written.

### 1.2 The onboard LED, independently

`ComsMCU::local_sample()` calls `ComsLed::life_pulse()` on **every** `loop()` pass
(`ComsMCU.cpp:117`), which toggles the green and blue pins every 100 calls. Colours are binary
(`digitalWrite`, not PWM), pins 22/23/24, active-low (`Board.h:107-110`).

So a live Nano's LED **never shows a steady colour** — it alternates fast enough to blend:

| state | alternates between | looks like |
|---|---|---|
| config OK, advertising | green / blue | teal |
| config OK, connected | cyan / off | dim cyan |
| config timeout, advertising | yellow / magenta | orange-pink |
| config timeout, connected | **white / red** | **pinkish white** |

At the freeze the LED **froze on one endpoint of the toggle** — white on one freeze, **red** on the
next. Random phase, which is exactly what a stopped 50/50 toggle looks like; a meaningful *state*
colour would be the same every time. That means `life_pulse()` stopped, which means `local_sample()`
stopped, which means **`loop()` stopped.**

### 1.3 Scope of the stop

`Mid-Trial-Freeze-Nano-Radio-Silence.md` established that ArduinoBLE runs Cordio in its own RTOS
thread, so a stalled `loop()` alone leaves the BLE link **alive** — frozen plots, no disconnect. This
freeze produced the 9.6 s supervision timeout, so the Cordio thread stopped too.
**Both threads. MCU-wide stop, with no fault and no reset.**

### 1.4 Ruled out along the way — do not re-chase these

- **Not the Teensy, not power loss.** The user confirmed the exo kept assisting normally through the
  freeze. The Teensy drives the motors independently of the Nano, so "the exo still works" clears the
  Teensy and rules out a dead battery, but says nothing about the Nano.
- **Not the host or GUI.** The trial CSV shows the host *healthiest* in its final 15 s window
  (largest receive gap 0.082 s, versus 0.211 s earlier in the run) with the wall-vs-exo lag returning
  to −25 ms rather than running away. No backlog, no progressive stall. "The GUI froze then
  disconnected" is the plots stopping when data stopped, then the 9.6 s timeout firing.
- **Not the actuation load, on this evidence.** The 103 s before the freeze was electrically violent
  — the PID term exceeded the setpoint in 86 % of samples and pinned the right joint to the 30 Nm
  clamp for over 5 % of the window, off an 11 Nm setpoint — but that is a separate (real) problem,
  and the LED/RESETREAS evidence points at a software hang, not a brownout. Time-to-death was ~316 s
  here versus 606/609 s in earlier stress runs; **n=1**, worth an A/B someday, not a conclusion.
- **Config timeout is NOT the cause.** `get_config` does appear to time out (the red channel is lit,
  and red is set in exactly one place — `ExoCode.ino:821`, the timeout branch), leaving the Nano's
  `ExoData` built from a zeroed config. But `ComsMCU` has **zero** references to `get_joint_with`,
  `is_used` or joint IDs, and every BLE handler sets only a local shadow flag before forwarding the
  command to the Teensy unconditionally — so a zeroed config is both invisible in normal operation
  *and* has no surface on which to wedge the loop. Worth fixing on its own; not this bug.

---

## 2. Why a watchdog, and why it need not know where the bug is

The WDT counts off the 32.768 kHz LFCLK, independently of the CPU. Code must periodically feed it; if
it is not fed within the timeout, the chip resets. **You do not install it at the fault site** — you
install one feed on a path known to be healthy, and it fires when anything, anywhere, stops that path
being reached. §1.2 proved which path that is.

This is precisely why it succeeds where every software timeout failed: a software timeout has to be
executed by the very CPU that has stopped executing.

**A watchdog reset is warm**, so `DOG` lands in `RESETREAS` and `GPREGRET`/`GPREGRET2` survive —
breaking the loop where recovery required the power cycle that erased the evidence.

---

## 3. What was added

### 3.1 `ExoCode/src/SystemReset.h`

| # | Change | Notes |
|---|---|---|
| 1 | Moved the four `EXO_CRASH_*` defines **above** the new watchdog block | Cosmetic but load-bearing: the watchdog's boot-loop guard uses `EXO_CRASH_MAGIC_MASK`/`_COUNT_MASK`, and macros must be defined before use |
| 2 | Forward declaration `inline void exo_crash_record(uint8_t*, uint8_t*);` | So `exo_wdt_boot_count()` can latch the crash record before overwriting `GPREGRET` |
| 3 | The `HARDWARE WATCHDOG` block | `EXO_HAVE_WDT`, `EXO_WDT_MAGIC 0xD0`, `EXO_WDT_MAX_AUTO_RESETS 3`, `EXO_WDT_TIMEOUT_S 5`, `EXO_WDT_RELOAD_MAGIC`, `EXO_WDT_LFCLK_HZ`, the six `EXO_STAGE_*` codes, and `exo_wdt_stage()` / `exo_wdt_stage_record()` / `exo_wdt_feed()` / `exo_wdt_boot_count()` / `exo_wdt_start()` |
| 4 | Three edits inside `exo_crash_record()` | `(void)exo_wdt_stage_record();` at the top; a `latched_marker_raw` copy; the else-branch now only zeroes `GPREGRET` when it is not holding the dog count |
| 5 | A `DOG` block in `exo_reset_reason_string()` | Appends `,STAGE_n_dogN` |

`exo_wdt_start()` configuration: `CRV = 5 * 32768 - 1`, `RREN` = reload register 0 only,
`CONFIG = SLEEP:Run | HALT:Pause`. **`SLEEP=Run` is essential** — mbed idles the core between events,
so without it a hang that parks in sleep would never trip the dog, which is most of them.
`HALT=Pause` stops a debugger single-step from resetting the board.

### 3.2 `ExoCode/ExoCode.ino`

| # | Change | Where |
|---|---|---|
| 6 | `#include "src/SystemReset.h"` | Nano block, after `#include "src/ComsLed.h"` |
| 7 | `exo_wdt_start();` | **End** of the Nano's `setup()` |
| 8 | `exo_wdt_feed()` plus six `exo_wdt_stage()` calls | Nano `loop()`, wrapping the five `mcu->...()` calls |

**Why it arms last (#7):** boot spends up to 18 s in `readSingleMessageBlocking()`
(10 s `kReadyTimeoutMs` + 8 s `kReceiveTimeoutMs`) and up to 8 s in `get_config()`, with nothing
feeding. Arming earlier would reset the board mid-boot, **forever**. The nRF52840 WDT has no stop
task — once started it cannot be turned off, which is why the arming point matters.

**Why the feed is in `loop()` and not a callback (#8):** Cordio's RTOS thread survives a stalled
`loop()` (§1.3). Feeding from anything that outlives the hang would defeat the entire mechanism.

### 3.3 Three bugs found and fixed while integrating with the existing crash trap

1. **The watchdog had no boot-loop guard, and it defeated the crash trap's own.**
   `mbed_error_hook` deliberately gives up after `EXO_CRASH_MAX_AUTO_RESETS` so a reproducible fault
   leaves the device wedged-but-visible rather than thrashing — but a dog simply reboots the halted
   mbed, and a Nano that hangs on every boot would reboot forever with nothing counting.
   Fixed: `EXO_WDT_MAGIC 0xD0` count in `GPREGRET`; after 3 consecutive `DOG` reboots
   `exo_wdt_start()` **stops arming**, so the device stays up and connectable and the breadcrumb can
   actually be read instead of being rebooted away every 5 s.
2. **The dog counter would have eaten the crash record.** `exo_wdt_boot_count()` runs at the end of
   `setup()`; `exo_crash_record()` runs later, from `ExoBLE::setup()`. On a fault -> mbed halt -> dog
   reboot, the counter would have overwritten `GPREGRET` before anything read the `CRASH_0x` code.
   Fixed by latching the record first (hence the forward declaration).
3. **`exo_crash_record()` would have wiped the dog counter.** Its else-branch zeroes `GPREGRET` on
   every boot with no crash magic — which is *every watchdog boot* — silently disarming the guard on
   its first use. Fixed with the `latched_marker_raw` test.

### 3.4 Verified safe against false trips

The End-Trial reset is a **non-blocking state machine** (`ComsMCU::_maybe_system_reset()`): the 3 s
`_reset_ack_timeout_ms` and 300 ms `_reset_flush_ms` are elapsed-time checks, not blocking waits, so
`loop()` keeps iterating and feeding through the whole sequence. `update_UART` polls at
`CONT_MCU_TIMEOUT` = 1000 us. 5 s is large headroom.

**A watchdog that fires on healthy code is worse than none.** First flash: connect, run a short
normal trial, confirm no surprise reboots. If a spurious `DOG` appears, the `STAGE_n` in the banner
localizes it immediately.

---

### 3.5 The BLE link-stall detector - SECOND failure mode, and it spans the GUI

**2026-09-10, later the same session: a second, distinct failure was caught in the act.** The GUI
auto-disconnected, but the Nano's **RGB was still blinking and the green LED_PWR still flashing** -
`loop()` alive AND RT data still arriving over I2C (`_life_pulse()` at `ComsMCU.cpp:197` is called
*only* inside the `if (new_rt_data)` branch). The user could **not** reconnect, and the RGB **did not
change colour** at the drop - so `advertising_onoff(0)` never ran, so `BLE.connected()` never
changed, so **the Nano never noticed the disconnect.** A button reset recovered it.

Two presentations:

| | `loop()` | RT data | LED | re-advertises? |
|---|---|---|---|---|
| **Mode 1 - hang** | stopped | stopped | frozen SOLID on a toggle endpoint | no |
| **Mode 2 - link stall** | alive | alive | still blinking, unchanged | no |

**Likely one root cause.** ArduinoBLE 2.1.0 - the sketchbook copy that actually builds, *not* the
vendored 1.2.1, see `arduino-toolchain-on-this-pc` - at `HCI.cpp:636`:

```cpp
int HCIClass::sendAclPkt(uint16_t handle, uint8_t cid, uint8_t plen, void* data)
{
  while (_pendingPkt >= _maxPkt) {   // no timeout, no bound, no escape
    poll();
  }
  ...
  _pendingPkt++;
```

`_pendingPkt` is decremented **only** by `handleNumCompPkts()` (`HCI.cpp:861`), driven by the
controller's "Number of Completed Packets" event. If those stop arriving it climbs monotonically.
Below `_maxPkt` you get **mode 2** - `loop()` runs, nothing reaches the host, the disconnect is never
processed. At `_maxPkt` you get **mode 1** - the next send spins forever inside `loop()`. `_maxPkt` is
single-digit on the nRF52, so which one you see is a race, **which is why the trigger has never been
reproducible.** A spin is also not a fault, which is exactly why `RESETREAS` read `RESETPIN` alone
with no crash record. **Strong hypothesis that fits every observation - `_pendingPkt` was never
directly measured.**

**The detector does not depend on that hypothesis being right.** It asks only "can the GUI still
reach us", which is the actual user-visible failure whatever causes it. From inside the firmware
nothing looks wrong in mode 2 - `sendAclPkt()` returns 0, `writeValue()` succeeds, our queue drains -
so the only trustworthy signal is end-to-end, and that requires the GUI to say something
periodically. Hence the ping, and hence this change touching `Python_GUI/`.

| # | Change | Where |
|---|---|---|
| 9 | `EXO_STALL_MAGIC 0xE0`, `EXO_BLE_STALL_MS 8000`, `exo_ble_stall_reset()`, `exo_stall_record()`, `,BLESTALL_n` reporting, and the stall magic added to the `exo_crash_record()` preservation test | `SystemReset.h` |
| 10 | `ble_names::ping = 'p'` plus its `{ble_names::ping, 0}` row in the command table | `ble_commands.h` |
| 11 | `case ble_names::ping: break;` - a no-op, present only so the parser does not log an unknown command every 2 s | `ComsMCU.cpp` |
| 12 | `s_last_rx_ms` / `s_ping_seen` file statics; RX stamped in `on_rx_recieved()` **before** parsing; the detector in `handle_updates()`, placed **before** the unchanged-status early return; clock re-armed on connect | `ExoBLE.cpp` |
| 13 | `pingDevice()` - fire-and-forget `b"p"`, silent on both success and failure | `Python_GUI/services/QtExoDeviceManager.py` |
| 14 | `self._ble_ping_timer`, 2000 ms, wired to `qt_dev.pingDevice` | `Python_GUI/MainWindow.py` |

**Fail-safe by design:** the timeout is enforced **only after a ping has actually been seen**
(`s_ping_seen`). Connecting with a GUI, or any other client, that does not ping can never reboot the
board. 2 s ping against an 8 s timeout is four missed pings, deliberately under the ~9.6 s host-side
supervision timeout so recovery starts before the host even declares the link gone.

**It leaves a trace, and that is the point.** `exo_ble_stall_reset()` writes `EXO_STALL_MAGIC` and
warm-resets, so `GPREGRET` survives and the next banner reads **`,BLESTALL_n1`**. That is positive
confirmation from the device that the link died while it was still running - it turns the diagnosis
above from "fits the evidence" into "the detector fired". Checked ahead of the DOG and plain SREQ
branches so it is never confused with an End-Trial reboot, the other thing that produces `SREQ`.

**Version-skew note, matters when dismantling:** new firmware + old GUI is safe - no pings, so
`s_ping_seen` stays false and the detector never arms. Old firmware + new GUI is harmless but noisy:
`BleParser::_handle_command` logs `Command is not in list: p` every 2 s. **So remove the GUI half
first, or both at once.**

---

## 4. How to read the output

The reset reason is parked in `ErrorChar` during `ExoBLE::setup()` and printed by the GUI at connect
(`MainWindow._on_reset_reason`). No GUI change was needed — the stage is appended into the names
field the same way `CRASH_` already was.

| banner | meaning |
|---|---|
| `RST:0x00000004:SREQ,BLESTALL_n1` | **Mode 2 confirmed** - the link died while the Nano kept running, and our own detector rebooted it. This is the line that proves the diagnosis |
| `RST:0x00000002:DOG,STAGE_4_dog1` | **Hung**, watchdog recovered it. Died in `update_UART`. First consecutive dog reboot |
| `RST:0x...:...,CRASH_0x1234_n1` | A real trapped mbed fault (unchanged behaviour) |
| `RST:0x00000001:RESETPIN` | Button, re-flash, or this board's power-on (confounded — see `Nano-Reset-Pin-Spurious-Reset.md`) |
| `dog3` and no further recovery | Boot-loop guard tripped: 3 consecutive hangs, watchdog no longer arming, device deliberately left up so it can be read |

Stage codes (`SystemReset.h`): 1 `LOOP_TOP` (completed a clean pass), 2 `HANDLE_BLE`,
3 `LOCAL_SAMPLE`, 4 `UPDATE_UART`, 5 `UPDATE_GUI`, 6 `HANDLE_ERRORS`. A stage of 0 means no
breadcrumb was recorded.

---

## 5. HOW TO DISMANTLE

Remove in this order; later items are referenced by earlier ones. Every edit carries a comment naming
the watchdog, so `grep -rn "exo_wdt\|EXO_WDT\|EXO_STAGE" ExoCode/` finds all eight sites.

1. **`ExoCode/ExoCode.ino`, Nano `loop()`** — delete the `exo_wdt_feed()` call and all six
   `exo_wdt_stage(...)` lines, restoring the plain five-call block:
   ```cpp
   mcu->handle_ble();
   mcu->local_sample();
   mcu->update_UART();
   mcu->update_gui();
   mcu->handle_errors();
   ```
2. **`ExoCode/ExoCode.ino`, end of Nano `setup()`** — delete `exo_wdt_start();` and its comment.
3. **`ExoCode/ExoCode.ino`, Nano include block** — delete `#include "src/SystemReset.h"` **only if**
   nothing else there needs it. It also arrives transitively via `uart_commands.h`, so leaving it is
   harmless.
4. **`SystemReset.h`, `exo_reset_reason_string()`** — delete the `if (reasons & 0x00000002ul)` DOG
   block. Leave the `CRASH_` block above it alone.
5. **`SystemReset.h`, `exo_crash_record()`** — revert all three edits: drop
   `(void)exo_wdt_stage_record();`, drop `const uint8_t latched_marker_raw = latched_marker;`, and
   restore the else-branch to the unconditional pair:
   ```cpp
   NRF_POWER->GPREGRET  = 0;
   NRF_POWER->GPREGRET2 = 0;
   ```
6. **`SystemReset.h`** — delete the whole `HARDWARE WATCHDOG` block, from the banner comment down to
   the end of `exo_wdt_start()`.
7. **`SystemReset.h`** — delete the `exo_crash_record` forward declaration above that banner.
8. **`SystemReset.h`** — optional: move the four `EXO_CRASH_*` defines back below where the watchdog
   block was, and drop the two-line comment added above them. Purely cosmetic; leaving them is fine.

**The link-stall detector (items 9-14) - remove the GUI half FIRST**, per the version-skew note in
§3.5, or old firmware logs an unknown command every 2 s:

9. **`Python_GUI/MainWindow.py`** - delete the `self._ble_ping_timer` block (four lines plus its
   comment) from `__init__`.
10. **`Python_GUI/services/QtExoDeviceManager.py`** - delete the whole `pingDevice()` method.
11. **`ExoCode/src/ExoBLE.cpp`** - delete the detector block in `handle_updates()`, the
    `s_last_rx_ms = millis();` line added to the connect branch, the stamping block in
    `on_rx_recieved()`, and the two file statics `s_last_rx_ms` / `s_ping_seen`.
12. **`ExoCode/src/ComsMCU.cpp`** - delete `case ble_names::ping: break;`.
13. **`ExoCode/src/ble_commands.h`** - delete `ble_names::ping` and its `{ble_names::ping, 0}` row.
14. **`ExoCode/src/SystemReset.h`** - delete `EXO_STALL_MAGIC`, `EXO_BLE_STALL_MS`,
    `exo_ble_stall_reset()`, `exo_stall_record()`, the `BLESTALL_n` block in
    `exo_reset_reason_string()`, and restore the `exo_crash_record()` preservation test to check
    `EXO_WDT_MAGIC` alone (or to unconditional zeroing if the watchdog is going too).

**Then verify** (see the `arduino-toolchain-on-this-pc` note — `--config-file` is mandatory):

```
ACLI="/c/Program Files/Arduino IDE/resources/app/lib/backend/resources/arduino-cli.exe"
CFG="$HOME/.arduinoIDE/arduino-cli.yaml"
"$ACLI" --config-file "$CFG" compile --fqbn arduino:mbed_nano:nano33ble ExoCode/ExoCode.ino
"$ACLI" --config-file "$CFG" compile --fqbn teensy:avr:teensy41    ExoCode/ExoCode.ino
```

**If you keep one thing, keep the watchdog itself** (items 2, 3, 6 — plus 5 and 7, which item 6
depends on) and drop only the breadcrumbs (items 1 and 4). The breadcrumbs are pure diagnostics; the
watchdog is a robustness property on a device that straps to a leg and drives 30 Nm motors. That is a
judgement call for the user, not a requirement — the procedure above removes everything cleanly.

**Unrelated, do not confuse:** `EXO_CRASH_TRAP_SELFTEST` in `SystemReset.h` is pre-existing, still
`0`, and untouched by this change. It is still worth running once (set to 1, boot, set back) —
until it fires, an empty `GPREGRET` cannot distinguish "no fault occurred" from "the trap is broken".

---

## 6. Deliberately NOT done: the Teensy answering `'R'` at runtime

A watchdog reset recovers BLE but lands in a degraded session, and this is why:

`readSingleMessageBlocking()` (`ExoCode.ino`) spams `'R'` waiting for the bulk controller-parameter
payload, but `send_bulk_char()` is only called from the **Teensy's `setup()`** — never re-sent at
runtime. So a Nano-only reset gets no answer: ~18 s of blocked boot, then `rxBuffer_bulkStr` is left
empty — and `ExoBLE.cpp:374` gates the handshake on that buffer being non-empty, so **the GUI
connects but never receives the controller list or parameter names.** This is very likely the same
mechanism behind the old "PC connects but the Nano will not serve its GATT table" log entries.

It was scoped and rejected as larger than it looks: `send_bulk_char()` uses **Serial8**, the same
physical port as `MY_SERIAL`/`UARTHandler` (`UARTHandler.h:40`), and the Nano's side likewise shares
Serial1. A raw `'R'` sniff inside the Teensy's 500 Hz control loop would compete with the framed UART
protocol for bytes. A clean fix needs a proper chunked UART command, because the payload is far
larger than one `UART_msg_t`.

**User decision 2026-09-10: not needed.** A warm reboot that reports the bug is sufficient; a clean
session afterwards is not the point of this scaffolding.

---

## 7. Known limits

- **This does not fix the freeze.** It recovers from it and reports where it stopped.
- **The stage is a phase, not a line number.** It says which of five `ComsMCU` calls was executing.
  Narrowing further means adding sub-stages inside whichever phase turns up.
- **A fault overwrites the stage.** `mbed_error_hook` writes its error byte to `GPREGRET2`, the same
  register the breadcrumb uses. They are told apart by the `GPREGRET` magic, and the `CRASH_` branch
  takes priority. Both registers are 8 bits on the nRF52840, so there is no room for both — and
  fault-vs-hang are mutually exclusive anyway.
- **Never flashed.** Compile-clean on both boards is the only validation so far.
- **After 3 consecutive hangs the watchdog stops arming** (by design, §3.3). A device in that state
  looks like the old behaviour — hung, needing a power cycle. Check the banner for `dog3`.
