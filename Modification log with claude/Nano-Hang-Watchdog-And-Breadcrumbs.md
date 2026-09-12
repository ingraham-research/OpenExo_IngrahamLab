# Nano hang: watchdog + loop breadcrumbs

**Date:** 2026-09-10
**Branch:** `disconnection_troubleshooting`
**Status:** **FLASHED AND BENCH-TESTED. Mode 2 confirmed by the device (§3.7); the failure is now
characterised as an INTERRUPT-LEVEL STOP (§3.8), not a loop hang.** Root cause still unknown.
**BISECT COMPLETE (§3.9): the BLE notification stream is a NECESSARY CONDITION; I2C is not.**
Raw observations are now kept separately in `Nano-Disconnect-EVIDENCE-ONLY.md` - read that first.
Superseded note: build B16 was a `REAL_TIME_I2C 0` bisect - see §3.8/§3.9 before reading
anything into blank plots. The watchdog fired on the real hardware and the
Nano recovered itself - the first time in this whole investigation that a disconnect did not require a
power cycle. That first run also exposed two bugs in this change; both are fixed (§3.6) and recompile
clean for `arduino:mbed_nano:nano33ble` AND `teensy:avr:teensy41`, but the FIXES are not yet flashed.
Watchdog + breadcrumbs are committed (`d76707d`); the link-stall detector is committed (`8d214fd`);
the §3.6 fixes are uncommitted.
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

### 3.6 FIRST BENCH RESULT, and the two bugs it exposed

**2026-09-10, first flash.** Mid-trial the GUI reported `link lost`. The LEDs showed mode 2 - RGB
still blinking, green LED_PWR still flashing - and then, without any intervention, **the device came
back and the GUI reconnected.** That had never happened before; every previous disconnect in this
investigation required a power cycle, which is precisely what kept destroying the evidence.

Banner on reconnect:

```
NANO LAST RESET REASON
 RESETREAS = 0x00000002  (DOG,STAGE_0_dog1)
```

So the watchdog fired and recovered the board. **But `STAGE_0` is the "no breadcrumb recorded"
value** - the loop stamps 1-6 and never 0 - so the dog did not bite during a normal loop phase. Two
bugs, both mine, both in this change:

**Bug A - the dog was armed before anything fed it.** `exo_wdt_start()` was called at the end of
`setup()`, but the first `exo_wdt_feed()` is on the first `loop()` pass *after* the one-time
constructions:

```
exo_wdt_start();        // END of setup() - ARMED here
loop() pass 1:
  new ExoData(...)      // unfed
  new ComsMCU(...)      // unfed - ExoBLE::setup() runs in here: BLE.begin(),
                        // GATT registration, advertising - AND exo_crash_record(),
                        // which ZEROES GPREGRET2
  exo_wdt_feed();       // first feed, only now
```

BLE initialisation runs in that window with nothing feeding the dog, and `exo_crash_record()` blanks
the breadcrumb inside the same window - hence exactly `DOG,STAGE_0_dog1`. **That was a false trip on
the boot path, not the hang we are hunting.**

*Fix:* `setup()` now only latches the previous boot's breadcrumb
(`(void)exo_wdt_stage_record();`), and `exo_wdt_start()` moved to the **end of `loop()`**, so the dog
is armed only after one complete pass has proven the loop can finish. `exo_wdt_start()` returns early
on `RUNSTATUS`, so calling it every pass costs one register read. A new `EXO_STAGE_ARMED (7)` is
stamped at arm time, so a boot-path trip is now identifiable instead of showing as a bare `STAGE_0`.

**Bug B - a watchdog reset erased any pending BLESTALL record.** `exo_wdt_boot_count()` overwrites
`GPREGRET` with the dog magic, and it deliberately latched the *crash* record first (§3.3 item 2) -
but the **stall** record lives in the same register and was never latched. So the plausible sequence
here (link stalls -> detector fires and warm-resets -> boot-path false trip -> dog reset) would
silently destroy the `BLESTALL_n` marker, which is the one piece of evidence the detector exists to
produce. **We may well have lost a genuine mode-2 confirmation to this.**

*Fix:* `exo_wdt_boot_count()` now calls `exo_stall_record()` alongside `exo_crash_record()` before
touching `GPREGRET` (forward-declared for ordering, same pattern as the crash record).

**Also fixed:** `MainWindow.py`'s `DOG` explanation still read *"but this firmware configures no
watchdog. Investigate."* - true when written, wrong now. It now explains the reset and points at the
`STAGE_n` field.

**Bug C - THE WATCHDOG SURVIVES A WARM RESET, and the boot path cannot outlive it.**
Found after reflashing with the Bug A/B fixes and getting `DOG,STAGE_0_dog1` *again*, which the fixed
code should have made impossible. It was not impossible, and the banner said why:
`exo_wdt_start()` early-returns on `RUNSTATUS` **before** it stamps `EXO_STAGE_ARMED`, and that early
return is the only remaining path that leaves the breadcrumb at 0. So the dog was **already running
at boot**.

The nRF52840 WDT is **not cleared by `NVIC_SystemReset`, nor by a pin reset, nor by a re-flash** -
only by a power-on reset, or by the dog itself firing. It was assumed to start clean. It does not.
Consequence:

```
link stalls -> stall detector NVIC_SystemReset  ->  WDT STILL RUNNING, 5 s window
            -> boot path: readSingleMessageBlocking up to 18 s
                          + get_config up to 8 s      <- NOTHING FEEDING
            -> dog bites at 5 s -> DOG, breadcrumb blank -> STAGE_0
                                -> and exo_wdt_boot_count() overwrites the BLESTALL marker
            -> THAT reset finally clears the WDT, so the next boot comes up clean
```

So **every warm reset cost an extra reboot and destroyed the evidence it was supposed to carry** -
which is exactly the observed `DOG,STAGE_0_dog1`, repeatedly. Flashing does the same thing, because
the upload reset also leaves the old dog running.

*Fix:* feed the dog everywhere the boot path can block longer than the window. `exo_wdt_feed()` is one
register write and is harmless when no watchdog is running, so these are unconditional:

| site | why |
|---|---|
| `GetBulkChar.cpp`, the `while (!Serial1.available())` 'R' loop | blocks up to `kReadyTimeoutMs` = 10 s |
| `GetBulkChar.cpp`, the `while (!messageComplete)` loop | blocks up to `kReceiveTimeoutMs` = 8 s |
| `uart_commands.h`, the `get_config()` `while (1)` retry loop | blocks up to `CONFIG_TIMEOUT` = 8 s |
| `ExoBLE.cpp`, around `BLE.begin()` and after GATT registration | longest single block on the rest of the boot path |
| `ExoCode.ino`, around the `new ExoData` / `new ComsMCU` constructions in `loop()` | these run on pass 1 before the loop reaches its normal feed |

**Lesson worth keeping even if all of this is dismantled:** on nRF52840, once the watchdog has ever
been started, *every* code path that can block longer than the window must feed it - including the
whole boot path - because a warm reset does not give you a clean slate.

**What this run does and does not establish.** Establishes: the watchdog works, fires on real
hardware, and recovers the board warm with the evidence intact - the core dead-end of this whole
investigation is broken. Does **not** establish anything about the hang itself: `STAGE_0` was a
boot-path false trip, so the next run with the §3.6 fixes is the first one whose `STAGE_n` will
actually mean something.

---

### 3.7 MODE 2 CONFIRMED BY THE DEVICE - 2026-09-11

```
2026-09-11 13:11:31 | Reset reason characteristic read: 'RST:0x00000004:SREQ,BLESTALL_n1'
```

**This is the result the whole exercise was for.** `BLESTALL_n1` is written only by
`exo_ble_stall_reset()`, so the Nano is reporting, in its own words, that it stopped hearing the
GUI's 2 s pings for 8 s **while its own stack still reported `BLE.connected()` as true**, and
rebooted itself. Mode 2 is real, and it is not an inference any more.

The `SREQ` is expected - our detector calls `NVIC_SystemReset()` deliberately. An actual Mbed fault
would have printed `CRASH_0x..` instead (that branch takes priority in `exo_reset_reason_string()`).

Timeline, and every number lands where the model says it should:

| time | event |
|---|---|
| 13:10:13.7 | trial starts |
| ~13:10:53.9 | Nano stops transmitting (back-calculated from the 9.6 s supervision timeout) |
| ~13:11:01.9 | 8 s of ping silence -> `EXO_BLE_STALL_MS` expires, warm reset |
| 13:11:03.5 | GUI logs `link lost` - **after** the Nano had already rebooted |
| 13:11:29.8 | reconnect, banner intact |

The ~28 s from reset to reconnect is the ~18 s blocked bulk read (§6) plus the scan - the predicted
boot cost.

**Three fixes validated at once by this single line:**
- the **stall detector works** and fires on a real mode-2 event;
- **Bug B's fix held** - the `BLESTALL` marker survived to be read;
- **Bug C's fix held** - there is **no `DOG`** on the line, so the watchdog that survives a warm
  reset no longer bites its way through the boot path and overwrites the evidence.

**Also fixed here:** `MainWindow.py`'s `SREQ` text still read *"unexpected otherwise - could be an
Mbed fault handler reboot"*, which led the user to suspect a crash on seeing this. It now enumerates
the three legitimate `SREQ` sources (`BLESTALL_n` = recovery working as designed, `CRASH_0x` = real
Mbed fault, bare `SREQ` = End Trial) and says which case is actually unexplained.

**What is still NOT known:** *why* the link dies. Mode 2 is confirmed as a phenomenon; the
`_pendingPkt` spin (§3.5) remains the leading explanation and is still unmeasured. What has changed
is that the failure is now survivable and self-reporting instead of a dead end.

---

### 3.8 2026-09-11, second half: the failure characterised (and one retraction)

The single most important result of the whole investigation, and it reframes everything above.

**The evidence, from one run (B15, 626 s trial, banner
`DOG,STAGEn0_g0_dog1,B15_b1,I2Cf64k_e278x10_c2_t266d_w0d_x300`):**

| field / observation | reading |
|---|---|
| `f64k` | 64,000 RT frames over a 626 s trial = ~100 Hz. The counters are sound |
| `w0d` | worst gap between successful I2C sends **under 100 ms** for the whole trial |
| `x300` (capped) | then **>=300 consecutive failures** - at 100 Hz, **>=3 s of unbroken silence** |
| `c2` | address NACK: **the Nano stopped acknowledging on the bus** |
| user observation | onboard RGB **solid red** AND green LED_PWR **solid, not flashing** - both frozen |

`w0d` next to `x300` is the important pairing: the link was perfectly healthy to the last moment and
then fell off a cliff. Not a degradation of the chronic loss - a distinct, total, unrecovered stop.
`w` stayed 0 because `worst_gap` only updates on a SUCCESS, and there was never another one.

**!! RETRACTED, same day - see the correction immediately below. !!** This section originally argued
that the Nano must have "stopped servicing interrupts entirely", on the reasoning that the I2C slave
ACK is generated by the peripheral and its ISR, so a stalled `loop()` could not stop it.

**That reasoning was wrong, and it was written before reading the core's `Wire.cpp`.** On
`mbed_nano` 4.6.0 the I2C slave is **`arduino::MbedI2C::receiveThd()` - a `while(1)
{ slave->receive(); ... }` polling RTOS THREAD**, not an interrupt handler. So ACKs stopping requires
only that **thread** to be starved of CPU. No interrupt-level stop is needed, and there is no evidence
for one. Every conclusion in this document that rests on "interrupts were disabled" should be read as
**"the main thread stopped yielding"** instead.

**This also explains why every instrument came back empty**, and that line is now closed:

| store | result | conclusion |
|---|---|---|
| `GPREGRET2` | `g0` every time, with a known non-zero stage in flight | wiped by a **watchdog** reset (it survives SREQ fine - BLESTALL proves that) |
| `.noinit` RAM | `b1` on every boot from the boot counter added in B14 | the mbed linker does not honour the section; the variable lives in `.bss` and is zeroed at startup |
| the LEDs | frozen | die with the interrupts |

**THE CORRECTED MODEL (2026-09-11 evening).** One root cause, two presentations, no interrupt-level
stop required:

> The **BLE controller stops completing packets**. That simultaneously (a) kills the link, so the host
> hits its ~9.6 s supervision timeout and disconnects, and (b) stops `_pendingPkt` from ever
> decrementing, so it saturates and the main thread enters ArduinoBLE's unbounded
> `while (_pendingPkt >= _maxPkt) { poll(); }` (`HCI.cpp:636`) and never leaves. A main thread that
> never yields freezes the LEDs (`life_pulse()` lives there) and starves `receiveThd`, which stops
> servicing the TWIS - so the Teensy sees address NACKs.

The spin is therefore the **amplifier**, not the root: it turns "the BLE link died" into "the whole
Nano looks dead". And the two observed modes are the same failure caught at different stages:

| mode | when it was caught |
|---|---|
| **2** (`BLESTALL`, loop alive, RT still flowing) | BEFORE `_pendingPkt` saturated |
| **1** (`DOG`, everything frozen solid) | AFTER it saturated |

That is what a filling queue looks like, and it explains why the presentation seemed random.

This also resolves the objection that killed the naive version: `Mid-Trial-Freeze-Nano-Radio-Silence.md`
established that a stalled `loop()` ALONE leaves the link alive (frozen plots, no disconnect), yet a
disconnect always happens. Putting the root failure at the controller - one level below both threads -
produces the disconnect and the freeze from a single cause.

**So ignore every `STAGEn`/`g` value in this document's history** - the breadcrumb never had a working
home. Three builds went into that store. Do not build a fourth without first proving the store
survives, which is what the `b<n>` boot counter is for.

**A separate real finding: the RT I2C link is chronically lossy.** Error rates of 4.3% (B15, 2,780 of
64,000) and 13.6% (B14, 2,850 of 21,000) in *healthy* operation, all address NACKs, at the Wire
default of 100 kHz on both sides (nothing calls `setClock`). That independently matches the
**6-13% RT sample loss** measured months ago in `Mid-Trial-Freeze-Nano-Radio-Silence.md` from a
completely different angle, and is very likely its mechanism. Arithmetic that makes it plausible:
~28 bytes at 100 kHz is ~2.5 ms per transfer arriving every 10 ms, a 25% bus duty cycle, so any gap
in the slave re-arming between transfers collides. **This is worth fixing on its own merits and is
probably not the disconnect bug** - the chronic loss is scattered singles (`w0d`), the failure is one
unbroken run of hundreds.

**Ruled out this session (do not re-chase):**

- **The 2 s GUI ping is not the fix.** One 13-minute run (past the 609 s ceiling of 86 prior trials)
  looked like a breakthrough; the very next run with the ping still on died at 191 s. Variance, and
  an over-read of a single sample.
- **Not board-specific hardware** - reproduced across **3 different Nano boards**.
- **Not the host** - already established from CSV timing (§1.4), and unchanged since.
- **Not the RT payload rate alone** - failures span 40 s to 13 min with identical traffic.

**Numbers worth keeping.** Time-to-failure across this session: 40 s, 1.9 min, 2.5 min, 3.2 min,
4.3 min, 10.4 min, 13.2 min. Enormous spread, no visible trend against torque, trial length, or
whether motors were engaged. Any theory has to explain that variance.

**The bisect build (B16).** `REAL_TIME_I2C` forced to **0** in `Config.h`, which stops the Teensy
pushing RT frames over I2C and stops the Nano forwarding them over BLE. Chosen over lowering the RT
rate because runs already reach 10+ minutes, so a proportional stretch would be indistinguishable
from "fixed" in any practical window - a clean yes/no is worth more than a scaling curve.
**Expected while running it:** no plots, no CSV data, and the green LED_PWR permanently solid (it is
toggled only inside the `if (new_rt_data)` branch). That is the experiment working, not a new fault.
The banner carries **`rt0`** so the build is never in doubt. The UART fallback for RT is dead
(`uart_commands.h:695` - `rt_data::float_values` is `static` in a header, so writer and reader are
different translation units), which is why this removes the data rather than rerouting it.
**If it runs indefinitely the RT path is implicated; if it still freezes, it is not.**

**Leading root-cause hypotheses, none confirmed.** In order of fit to "interrupts stop being
serviced, both threads die, correlated with BLE activity, wildly variable time-to-failure":

1. **A spin or deadlock in the BLE stack that starves everything.** ArduinoBLE 2.1.0 has a known
   unbounded spin - `while (_pendingPkt >= _maxPkt) { poll(); }` at `HCI.cpp:636` - in exactly the
   subsystem involved. If that is ever entered while holding an RTOS lock the Cordio thread needs,
   or from a context with interrupts masked, both threads stop and only the hardware WDT recovers.
   Still unmeasured: `_pendingPkt` is private and reading it means patching the sketchbook library.
2. **A high-priority ISR that never returns.** The radio ISR outranks everything on this chip; if it
   wedges, I2C and the main loop both starve while the CPU is still technically executing - which is
   exactly the observed signature.
3. **Stack overflow corrupting RTOS state.** mbed threads have fixed stacks. Would not necessarily
   produce a clean fault, and a corrupted scheduler could stop both threads. No `LOCKUP` was ever
   seen in `RESETREAS`, which argues against a double fault but not against silent corruption.

**The definitive tool is SWD**: attach a probe, let it freeze, halt the CPU and read the PC. Every
indirect method tried so far dies along with the MCU. Not currently available - the Nano's SWD pads
are not easily reachable on this build - which is why the bisect is the chosen path.

---

### 3.9 2026-09-11 evening: the bisect result, and a re-diagnosis from evidence only

**All raw observations now live in `Nano-Disconnect-EVIDENCE-ONLY.md`.** That file was created
because this one had accumulated several confident explanations that turned out to be wrong, and it
had become hard to tell measured facts from inferences. Read it first; this section is interpretation.

#### The bisect

| build | RT over I2C | RT over BLE | result |
|---|---|---|---|
| baseline | yes | yes | **failed every run**, 50-789 s across 9 runs |
| B16 `rt0` | no | no | **~40 min, no failure** |
| B17 `rt1_fw0` | **yes** | no | **~30+ min, no failure** |

Round 2 is the single-variable test: I2C was fully restored (user confirmed the green LED_PWR
blinking throughout, which only happens inside `if (new_rt_data)`), and **nothing changed**.

**Conclusion, and it is the strongest thing in this investigation: the BLE notification stream is a
NECESSARY CONDITION for the failure. I2C is not.** That retires the pull-up theory, the bus-lockup
theory, and the "did upstream secretly add resistors" puzzle. The user refused the pull-up theory on
the grounds that upstream runs the same hardware and would have mentioned a deal-breaker component;
that objection was correct.

#### What one writeValue() actually does

Read out of the library rather than assumed (two earlier estimates in this document were wrong
because they were not):

```cpp
// ArduinoBLE 2.1.0, ATT.cpp:601 handleNotify
uint8_t notification[_peers[i].mtu];
length = min((uint16_t)(_peers[i].mtu - notificationLength), (uint16_t)length);
HCI.sendAclPkt(...);          // exactly ONE packet, never chunked
```

So **one `writeValue()` produces exactly one ACL packet**, truncated to the negotiated MTU. And since
RT frames arrive intact in the GUI, **the MTU must already be negotiated well above 23** - the comment
in `ExoBLE.cpp` about a 23-byte default describes the pre-negotiation value, not what is in use.
Frames are ~70 bytes (`BleParser::package_raw_data` writes ASCII decimal of `int(value*100)` plus a
delimiter, 3-7 bytes per value), so the link carries roughly 100 packets/s and ~56 kbps.
**The link is NOT bandwidth-saturated**; an earlier claim in this document that it was is withdrawn.

#### The two presentations, kept separate on purpose

| | main thread | RT data | BLE | recovery |
|---|---|---|---|---|
| **A** | **alive** (RGB blinking, green flashing) | flowing | dead, never re-advertises | `BLESTALL_n1` fired once |
| **B** | **stopped** (RGB solid, green solid) | stopped | dead | watchdog only |

**These are not obviously the same failure and this document previously asserted they were.** In A
the main thread demonstrably keeps running, which rules out anything that requires it to be wedged.
Whether A and B share a root cause is **not established**.

#### The mechanism hypothesis for presentation B

`_pendingPkt` increments on every ACL packet and is decremented **only** by `handleNumCompPkts()`,
driven by the controller's completed-packet events. `_maxPkt` comes from the controller's LE Read
Buffer Size and is single-digit on typical Cordio builds. At ~100 notifications/s:

> If the controller stops returning completed-packet events for only **~40-100 ms**, `_pendingPkt`
> reaches `_maxPkt` and the main thread enters `while (_pendingPkt >= _maxPkt) { poll(); }`
> (`HCI.cpp:636`) - which has **no timeout, no bound and no escape** - and never leaves. A main thread
> that never returns freezes both LEDs (`life_pulse()` lives there) and starves `receiveThd`, the
> polling thread that services the I2C slave, so the Teensy sees address NACKs.

That reproduces presentation B exactly, and explains the bisect: at 100 notifications/s a ~100 ms
stall bricks it, whereas with `fw0` the occasional status/battery traffic would need **many seconds**
of stalled completions to accumulate the same backlog, so it is never reached.

**Status: PLAUSIBLE, NOT PROVEN.** `_pendingPkt` and `_maxPkt` have never been read at runtime.

**The trigger is probably mundane.** A ~100 ms stall in packet completion is the kind of thing RF
interference or a busy central produces routinely. If this hypothesis is right, **the defect is not
the stall - it is that a 100 ms transient becomes a permanently dead exoskeleton** because a library
wait loop has no timeout. Everything else built in this investigation (watchdog, stall detector,
breadcrumbs) is downstream of that one missing bound.

#### The experiment that is also the fix

Bounding the spin tests the hypothesis and repairs it in the same change:

- presentation **B disappears**, failures become brief recoverable glitches -> hypothesis confirmed
- presentation **B persists unchanged** -> hypothesis wrong, cheaply, and the spin is exonerated

See §8 for the patch itself. It lives in the sketchbook copy of ArduinoBLE, **outside this repo**,
which is why it is documented here in detail.

#### Errors made this session, recorded so the pattern is visible

1. The 2 s GUI ping called a likely fix on one 13-minute run; the next run with it still enabled
   failed at 191 s.
2. "Interrupt-level stop" asserted before reading `Wire.cpp`, which shows the I2C slave is a polling
   **thread**, so thread starvation suffices and no interrupt claim was ever needed.
3. Missing I2C pull-ups proposed as likely cause despite the user having already pointed out that
   upstream runs the same hardware.
4. "~10 ACL packets per RT frame" - assumed MTU 23 without reading `handleNotify`.
5. "13 bytes per float" - assumed fixed-width fields without reading `package_raw_data`.

The common thread is reasoning from assumed constants instead of opening the file first. The
evidence-only document exists to make that failure mode harder to repeat.

---

## 4. How to read the output

The reset reason is parked in `ErrorChar` during `ExoBLE::setup()` and printed by the GUI at connect
(`MainWindow._on_reset_reason`). No GUI change was needed — the stage is appended into the names
field the same way `CRASH_` already was.

| banner | meaning |
|---|---|
| `RST:0x00000004:SREQ,BLESTALL_n1` | **Mode 2 confirmed** - the link died while the Nano kept running, and our own detector rebooted it. This is the line that proves the diagnosis |
| `RST:0x00000002:DOG,STAGE_7_dog1` | `STAGE_7` = ARMED: died on the boot path before the loop's first breadcrumb. Before the §3.6 fix this presented as the uninformative `STAGE_0` |
| `RST:0x00000002:DOG,STAGE_4_dog1` | **Hung**, watchdog recovered it. Died in `update_UART`. First consecutive dog reboot |
| `RST:0x...:...,CRASH_0x1234_n1` | A real trapped mbed fault (unchanged behaviour) |
| `RST:0x00000001:RESETPIN` | Button, re-flash, or this board's power-on (confounded — see `Nano-Reset-Pin-Spurious-Reset.md`) |
| `dog3` and no further recovery | Boot-loop guard tripped: 3 consecutive hangs, watchdog no longer arming, device deliberately left up so it can be read |

Stage codes (`SystemReset.h`): 1 `LOOP_TOP` (completed a clean pass), 2 `HANDLE_BLE`,
3 `LOCAL_SAMPLE`, 4 `UPDATE_UART`, 5 `UPDATE_GUI`, 6 `HANDLE_ERRORS`, 7 `ARMED` (armed, but the
loop had not yet reached its first breadcrumb - a boot-path trip). A stage of 0 now means only
that the previous boot recorded nothing, e.g. it ended in a true power-on.

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
2. **`ExoCode/ExoCode.ino`** - delete `exo_wdt_start();` and its comment from the **end of the Nano
   `loop()`** (it was moved there from `setup()` by the §3.6 fix), and delete
   `(void)exo_wdt_stage_record();` and its comment from the end of the Nano `setup()`.
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

15. **`Python_GUI/MainWindow.py`** - restore the `"DOG":` entry in `_RESET_REASON_MEANING` to its
    original text, or leave it: it is one descriptive string and is harmless either way.

16. **The Bug C boot-path feeds** - delete the `exo_wdt_feed();` calls (each carries a comment
    naming the watchdog) from: the two wait loops in `ExoCode/src/GetBulkChar.cpp` plus its
    `#include "SystemReset.h"`; the `get_config()` retry loop in `ExoCode/src/uart_commands.h`;
    the two in `ExoBLE::setup()` in `ExoCode/src/ExoBLE.cpp`; and the three around the one-time
    constructions in the Nano `loop()` in `ExoCode/ExoCode.ino`.

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

### 7.0 SIDE EFFECT: the watchdog breaks Nano uploads (workaround: power-cycle first)

**Symptom:** `bossac` reports a SAM-BA error partway through the upload, consistently around
**page 10-15 of 91**. Seen on two different Nanos. Never on the Teensy.

**Why it is the watchdog.** The upload *starts*, so the 1200-baud touch worked and the board reached
the bootloader - this is not a touch-reset failure. Something resets the chip mid-flash. 91 pages x
4 KB is ~373 KB, matching the ~371 KB sketch, so failing at page 10-15 is ~40-60 KB written, roughly
11-16% of the way, which at typical bossac CDC throughput is about 4-6 s. `EXO_WDT_TIMEOUT_S` is
**5**. The nRF52840 WDT survives a warm reset (§3.6 Bug C), so double-tapping into the bootloader
leaves it counting, and the bootloader does not feed it. It resets the chip out of the bootloader
mid-upload. It also explains why repeatedly tapping reset eventually works: once the dog actually
fires it clears itself, and the next tap gets a clean window.

**Timeline confirms it:** the user reports these failures began 2026-09-10, i.e. with the first
watchdog flash. They did not happen before.

**Workaround, use this every time:** **power-cycle the board before uploading**, do not merely tap
reset. A true power-on reset is the only thing that clears the WDT.

**Considered and rejected as a full fix:** arming only once the GUI subscribes would keep an idle
just-booted board uploadable, but it is only partial - a dog armed in a previous session is still
running after a stall-reset, so the power-cycle rule remains the real answer. Not worth the
complexity while this is temporary scaffolding.

**A second candidate was ruled out but is worth remembering** if the symptom ever outlives the
watchdog: brownout during flash writes (erase/program draws more current than idle, so a marginal
cable or port can sag) produces the same mid-upload, repeatable-page signature, and would also spare
the Teensy. The timeline is what separates them.

### 7.1 Other limits

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

---

## 8. The ArduinoBLE bounded-wait patch (OUTSIDE this repository)

**Applied 2026-09-11. This is the candidate fix AND the experiment that tests §3.9's hypothesis.**

### 8.1 What was changed

One file, one function:

```
C:\Users\<user>\Documents\Arduino\libraries\ArduinoBLE\src\utility\HCI.cpp
  int HCIClass::sendAclPkt(uint16_t handle, uint8_t cid, uint8_t plen, void* data)
```

Before — no timeout, no bound, no escape:

```cpp
while (_pendingPkt >= _maxPkt) {
    poll();
}
```

After — gives up and drops the packet:

```cpp
const unsigned long _aclWaitStart = millis();
while (_pendingPkt >= _maxPkt) {
    poll();
    if ((millis() - _aclWaitStart) >= ARDUINOBLE_ACL_WAIT_TIMEOUT_MS) {
        return -1;
    }
}
```

`ARDUINOBLE_ACL_WAIT_TIMEOUT_MS` is 50 by default and overridable. A long explanatory comment sits
immediately above the function in that file, covering the reasoning, the evidence, the return-value
change and how to revert — deliberately placed there because **anyone debugging BLE on this setup
will be reading that function anyway**, which is exactly where the warning is useful.

### 8.2 Why 50 ms

At the configured 7.5 ms connection interval that is ~7 connection events — far longer than any
healthy congestion needs to clear, so it should never fire in normal operation. If the link really is
dead, every send then costs 50 ms and the sketch loop drops to roughly 20 Hz: degraded but **alive**,
which also means the watchdog (5 s) will not fire and the BLE link-stall detector (8 s of GUI silence)
can do its job and warm-reboot cleanly.

### 8.3 Return value

`0` still means "handed to the transport"; the timeout path returns `-1`. **Verified safe for
ArduinoBLE 2.1.0:** all eleven callers are in `ATT.cpp` and every one discards the result. If a future
library version starts checking it, make sure it does not respond by retrying — that would defeat the
entire point.

### 8.4 This is outside version control

The build uses the **sketchbook** copy of ArduinoBLE (2.1.0), not the 1.2.1 vendored in this repo's
`Libraries/` folder — see `arduino-toolchain-on-this-pc`. So this change:

- is **invisible to this repository's git history**
- **will be silently lost** if ArduinoBLE is updated or reinstalled
- applies to **every** Arduino project on this machine that uses ArduinoBLE

A pristine copy was saved alongside it as `HCI.cpp.orig-openexo-backup` before patching.

**To verify the patch is still applied:**

```
grep -c ARDUINOBLE_ACL_WAIT_TIMEOUT_MS \
  "$HOME/Documents/Arduino/libraries/ArduinoBLE/src/utility/HCI.cpp"
```

Zero means it is gone — most likely the library was updated — and the freeze will return.

**To revert:** restore `HCI.cpp.orig-openexo-backup` over `HCI.cpp`, or delete the timeout lines and
the comment block.

### 8.5 What this build (B18) tests

B18 is normal operation (`rt1_fw1`) plus this patch. It is a genuine experiment, not just a fix:

| outcome | conclusion |
|---|---|
| **presentation B stops happening** (no more everything-frozen-solid, no more `DOG`) | the unbounded spin was the amplifier — §3.9's hypothesis confirmed, and this is a real fix |
| failures become brief glitches the exo rides through | same conclusion, even stronger |
| **presentation B continues unchanged** | the spin is exonerated, the hypothesis is wrong, and the cost was one afternoon |
| presentation **A** (`BLESTALL`, loop alive) still occurs | expected either way — A never depended on the spin, and the stall detector already handles it |

Note that A and B were never shown to share a root cause (§3.9), so this patch is only expected to
address B.

### 8.6 If it works, what should happen to it

This is a genuine defect in ArduinoBLE — an unbounded wait on a remote party in a library used on
battery-powered wearables — and it is worth reporting upstream to `arduino-libraries/ArduinoBLE`
regardless of what happens here. Doing so is also the only durable fix, since any local patch in the
sketchbook is one library update away from vanishing.

---

## 9. The connection interval - the first change aimed at WHY the link dies

Everything before this section manages the **aftermath** of the failure. This is the first change
that targets the failure itself.

### 9.1 What was changed

`ExoBLE.cpp`, one line:

```cpp
BLE.setConnectionInterval(6, 6);     // was: 7.5 ms, pinned as BOTH min and max
BLE.setConnectionInterval(12, 24);   // now: 15-30 ms, a range
```

Units are 1.25 ms. `(6, 6)` is **7.5 ms - the minimum the BLE spec allows - with min equal to max**,
so the central had no range to negotiate within.

### 9.2 Why that matters (the mechanism)

BLE is not a stream. Once connected, both radios sleep and wake together every *N* ms for a
**connection event**; data moves only during those events, and the **central owns the schedule**.

`(6, 6)` demanded **133 connection events per second, indefinitely, with no flexibility** from a
Windows host that is also running a mouse, keyboard, headphones, scanning, and - critically -
sharing the 2.4 GHz band with Wi-Fi through a coexistence scheme that makes the two take turns.
When that host is busy it does the only thing it can: **it skips connection events.**

With no slack in the interval, a burst of skipped events cascades:

1. Nothing can be transmitted - packets are queued with no appointment to send them in.
2. `_pendingPkt` saturates in **~40-100 ms** at ~100 notifications/s.
3. The main thread enters the `HCI.cpp:636` wait (forever pre-B18; 50 ms per send after).
4. ~9.6 s later the host's supervision timeout expires and it declares `link lost`.

**And the trapped main thread is why the device stayed unreachable.** At runtime, re-advertising
happens in exactly one place - `advertising_onoff(current_status == 0)` at `ExoBLE.cpp:470`, inside
`handle_updates()`, which is called from `ComsMCU.cpp:76` in the **main loop**. A trapped main thread
means `BLE.poll()` never processes the disconnect, `BLE.connected()` never changes, and the Nano
never re-advertises. **The link loss is transient; the trapped thread is what made the consequence
permanent.** (Credit where due: the user pointed this out - this document had it listed as an open
question when the answer was already established.)

**Throughput is not lost**, which is the counter-intuitive part: multiple packets can be exchanged
per connection event, so fewer, larger events carry the same data. §9.3 confirms this empirically.

### 9.3 Result - one run

| | reference 2026-09-10 (7.5 ms) | B19 (15-30 ms) |
|---|---|---|
| sample rate | 92.0 Hz | **90.2 Hz** |
| median gap | 10.0 ms | 10.0 ms |
| p90 / p99 / max gap | 20 / 30 / 80 ms | 20 / 30 / 70 ms |
| duration | 229 s, **failed** | **1,821 s (30 min), no failure, ended by the user** |

**The rate check matters as much as the duration.** Had the longer interval throttled the
notification rate, the run would have been confounded - reducing BLE traffic alone was already known
to suppress the failure for 30+ minutes (§3.9 bisect). It did not: 90.2 Hz against 92.0 Hz, with a
near-identical gap distribution. So the survival is attributable to the interval change itself.

Against a baseline mean time-to-failure of 225.5 s, a clean 1,821 s run has probability ~**1 in
3,700** under a constant-hazard null.

### 9.4 Status: CANDIDATE FIX, not confirmed

**One run.** The 2 s GUI ping produced a 13.2-minute run (~1 in 33) and was wrongly called a likely
fix; the next run with it enabled failed at 191 s. This is roughly 100x stronger evidence, but the
discipline is the same.

What is still needed:

- **Two or three independent runs**, spread over time. Exposure time is fungible under a
  constant-hazard model, but repeats cover what it cannot: possible elevated hazard right after
  connecting (three failures clustered at 12, 24 and 41 s), and RF conditions that drift.
- **Read back the negotiated interval** to confirm the host actually accepted 15-30 ms.

### 9.5 If it holds

Then the root cause was **a rigid, minimum-value connection interval**, and the fix is one line. The
watchdog, link-stall detector, bounded spin and breadcrumbs would all become a **safety net** rather
than the thing standing between the user and a dead trial - worth keeping the first three, and the
breadcrumbs can go (§5).

Worth noting the bounded-spin patch (§8) retains independent value regardless: an unbounded wait on
a remote party is a defect, and it is what turned a transient radio hiccup into a permanently
unreachable device.
