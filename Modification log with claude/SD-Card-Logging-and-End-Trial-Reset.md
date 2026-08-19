# Onboard SD Logging + End-Trial Reset / Shutdown Handshake

**Date:** 2026-07-06 → 2026-07-08
**Scope:** Teensy 4.1 firmware (`ExoCode/`), Nano 33 BLE firmware (`ExoCode/` Nano branch), Python GUI (`Python_GUI/`)
**Status:** Working and validated on-device (benchtop + worn, back-to-back trials, reconnect verified).
**Detailed design/plan (deeper reference):**
`docs/superpowers/specs/2026-07-06-teensy-sd-logging-design.md`,
`docs/superpowers/plans/2026-07-06-teensy-sd-logging.md`,
`docs/superpowers/specs/2026-07-07-end-trial-shutdown-progress-design.md`,
`docs/superpowers/plans/2026-07-07-end-trial-shutdown-progress.md`

This document is the high-level "what / why / how to modify" summary. It covers two intertwined
pieces of work: (A) the onboard SD data logger, and (B) the end-trial reset/shutdown fixes that
grew out of debugging why trials sometimes ended badly and corrupted the logs.

---

## Part A — Onboard SD data logger

### Why
Record per-motor and per-ground-strike data during a trial directly to the Teensy's SD card, in a
format similar to the hip-exo RPi logs, **without stalling the 500 Hz control loop**. Tunables live
in `config.ini` so they can be changed without reflashing.

### What it produces
Each trial opens a fresh session folder `/EXOLOG/000N/` (N auto-increments) containing:
- `Motor_L_log.txt`, `Motor_R_log.txt` — decimated per-motor rows. Columns:
  `Motor, Teensy_time_s, Status, Gait_phase, Position_rad, Velocity_rad_s, Torque_Nm,
  Commanded_Torque_Nm, Current_A, Filtered_Torque_Nm, Desired_Torque_Nm, Toe_FSR, Stance,
  Enabled, Timeout_ct, Error`
- `Ground_strike_log.txt` — one row per detected ground strike. Columns:
  `Leg, Teensy_time_s, Prev_step_ms, Expected_step_ms`
- `/EXOLOG/debug_log.txt` (only when `SD_LOG_DEBUG == 1`) — once/sec: status, loop `ran/s`, bytes/s.
  Recreated fresh each boot.

### Files
| File | Role |
|---|---|
| `ExoCode/src/SdLogger.h` | Logger class + all compile-time knobs + runtime-default macros. |
| `ExoCode/src/SdLogger.cpp` | Implementation: session open/close, decimated writes, flush, ground-strike capture, boot self-test, and `close_active()` (see Part B). |
| `ExoCode/ExoCode.ino` | `static SdLogger sd_logger(&exo_data);` in `loop()`; `sd_logger.update(ran);` **after** `exo.run()`. |
| `ExoCode/src/SideData.{h,cpp}` | Added `float last_step_duration;` (feeds the ground-strike log). |
| `SDCard/config.ini` | `[Logging]` section (runtime tunables). |

### How it stays off the control loop's back
- `sd_logger.update(ran)` runs in `loop()` **after** `exo.run()`. `exo.run()` self-paces via
  `delta_t`, so logging never delays the control cascade.
- Writes are **decimated**: only every Nth `ran==true` cycle writes a motor row (`sdLogDecimation`,
  default 5 → ~100 Hz).
- Flushing is **staggered**: `_maybe_flush()` syncs at most one of the files per `sdLogFlushMs`
  window, so no single cycle pays for three flushes.
- Ground-strike events are checked **every** control cycle (`_check_ground_strike_events()`) so a
  strike is never missed, but the row write is tiny.

### Configuration (`SDCard/config.ini`, runtime — no reflash)
```
[Logging]
    sdLogEnabled   = 1     ; 0 disables logging entirely
    sdLogDecimation = 5    ; write every Nth 500 Hz cycle (5 -> ~100 Hz)
    sdLogFlushMs   = 300   ; per-file flush cadence (ms)
```
Compile-time knobs live at the top of `SdLogger.h`: `ENABLE_SD_LOGGING`, `SD_LOG_DEBUG`,
`SD_LOG_SELFTEST` (boot-time raw-SD write test, then halt), `SD_LOG_SELFTEST_TRIAL` (boot-time fake
trial through the real `update()` path, then halt). All default to off except `ENABLE_SD_LOGGING`.

### Gotchas / notes
- The SD card **must be present** — `ParseIni` (config read) does `while(1)` forever if `SD.begin()`
  / `ini.open()` fails, which hangs the Teensy at boot and therefore hangs the Nano handshake (no
  advertising). "No LED after boot + can't find device" almost always means **no/failed SD card**.
  (We deliberately did *not* make boot non-fatal; power-cycle with a good card is the fix.)
- Log rows are the literal contents of `ExoData` (`motor.p/v/i`, etc.). Garbage motor values in a log
  mean garbage came off the CAN bus, not a logging bug — see the CAN note in Part B's "related
  findings."

---

## Part B — End-trial reset / shutdown handshake (and the log-corruption fixes)

### The original problem
Ending a trial was unreliable. Symptoms: sometimes the exo got "stuck" (unfindable until a manual
power-cycle), sometimes the just-written logs were **corrupt** (files present but un-openable and
un-deletable = FAT corruption). It was intermittent, and worse under load (worn trials) than
benchtop.

### Root causes found (in order of discovery)
1. **Reset abandons open log files.** End Trial sends `G` (stop), `w` (motor off), `Z` (firmware
   reset). `Z` → `exo_system_reset()` → `CPU_RESTART` = instant reboot with **zero cleanup**. If the
   log files were still open, the reboot corrupted the FAT. The stop→`trial_off` path *does* close
   logs, but only if that message arrives and a loop iteration runs before the reset — a race.
2. **The command channel (GUI↔exo) is unreliable, especially under load.** `G/w/Z` were sent
   fire-and-forget (BLE write *without response*). Under a worn trial (real-time data streaming), the
   last write in the burst (`Z`) got dropped before the Nano saw it → no reset → exo left running
   with logs open → the recovery power-cycle then corrupted them. This is the deeper issue behind
   "commands aren't always received."
3. **GUI mis-handled the reboot aftermath.** The reboot's BLE drop is detected ~10 s later (BLE
   supervision timeout), which is longer than the dialog's countdown, so the late drop was flagged
   as an "unexpected disconnect" popup and the scan buttons were left greyed.

### The fixes (all currently in place)

**1. Close logs before any reset — `SdLogger::close_active()`** (`SdLogger.cpp/.h`, `uart_commands.h`)
- `SdLogger` keeps a `static SdLogger* _instance` (set in its constructor) and exposes
  `static void close_active()` which flushes + closes the open session immediately (no-op if none).
- The Teensy reset handler `UART_command_handlers::get_system_reset` (`uart_commands.h`) calls
  `SdLogger::close_active()` **before** `exo_system_reset()`. So *any* reset closes the logs first,
  independent of whether the `trial_off` message ever arrived. Race eliminated.

**2. Reliable reset delivery — `send_end_trial_sequence()`** (`Python_GUI/services/QtExoDeviceManager.py`)
- End Trial calls `send_end_trial_sequence()` instead of three bare `write()`s. Order: **`Z` (reset)
  first, as a single BLE write-*with*-response** (ACK'd + retried, 2 s timeout + fire-and-forget
  fallback), then `G` (stop) / `w` (motor off) best-effort after.
- **Why `Z` must go first — this ordering is forced, not a preference (verified the hard way):**
  during an active trial the link is saturated with real-time-data notifications. Under that load a
  plain write-*without*-response (`G`/`w`) as the **first** command **hangs on WinRT** (bleak's
  `write_gatt_char` never returns) — we tried `G→w→Z` and the whole sequence stalled before `Z` was
  ever sent (no reset at all). Only a Write *Request* (write-with-response) punches through the
  congestion. `Z`'s handler is a complete shutdown on its own (disables motors + `trial_off` + closes
  logs before rebooting), and processing it stops the trial → stops RT streaming → relieves the
  congestion, so the trailing best-effort `G`/`w` backups can then get through. Only `Z` uses
  write-with-response (a 3× with-response burst wedged the 2nd write — a WinRT quirk with consecutive
  Write Requests).
- **Dead end to not re-try (and why the *original* `G→w→Z` "sometimes" worked but ours never did):**
  intuitively the `G`/`w` backups "should" precede the reset. We tried `G→w→Z` in
  `send_end_trial_sequence()` and it **never** worked — the lead `await` (fire-and-forget `G`) hangs
  under trial congestion and stalls the whole coroutine before `Z` is sent. The original code got
  away with `G→w→Z` only because it sent each via a **separate** `self.qt_dev.write()` call = a
  separate `run_coroutine_threadsafe` coroutine, so a blocked `G` didn't stop the *independent* `Z`
  coroutine (best-effort, three independent shots — `Z` still dropped under load, which is what
  caused the original corruption). Our `send_end_trial_sequence()` deliberately **awaits the three
  serially in one coroutine** so `Z` can be a reliable Write Request — the trade-off is that a
  blocked lead write kills everything behind it, so the reliable `Z` must lead. Serial + reliable
  `Z`-first was chosen over concurrent-but-unreliable.
- `MainWindow._on_end_trial` and `_on_retry_end_trial` both call it.

**3. Nano reset state machine + GUI progress dialog** (the "shutdown progress" feature)
- **Nano** (`ComsMCU.{h,cpp}`): `_maybe_system_reset()` is a non-blocking state machine
  `IDLE→PENDING→SENT→WAIT_ACK→SEND_REBOOT→REBOOTING`, driven every loop from `local_sample()`. Each
  state emits a `send_shutdown_progress` BLE message (step code 1–5). It forwards the reset to the
  Teensy in `SENT`, waits (≤3 s) for the Teensy's `reset_ack` in `WAIT_ACK`, then reboots.
- **Teensy** (`uart_commands.h` `get_system_reset`): sends a `reset_ack` UART message to the Nano
  **first**, then disables motors → `trial_off` → `close_active()` → `exo_system_reset()`.
- **Shared message defs**: `UART_command_names::reset_ack = 0x1C`; `ble_names::send_shutdown_progress
  = 'P'` + its length entry; `namespace shutdown_progress { RECEIVED=1, SENT=2, ACKED=3, TIMEOUT=4,
  REBOOTING=5 }` (all in `ble_commands.h` / `uart_commands.h`).
- **GUI parse** (`RtBridge.py`): parses the `'P'` message (payload is `int(value*100)`, `'n'`-
  delimited — same framing as every command payload) → emits `shutdownProgressReceived(int step)`.
- **GUI dialog** (`Python_GUI/Widgets/ShutdownDialog.py`, new): modal step checklist
  (Received / Sent / Teensy acknowledged / Rebooting) → 6 s "Exo rebooting…" countdown → returns to
  scan. The countdown starts on **step 5, OR** the device `disconnected` signal, **OR a ~5 s fallback
  timeout** — because progress notifications get dropped over BLE (fired faster than the connection
  interval, sometimes several lost), the dialog must **never depend on a tick to advance**. The reset
  is reliably delivered, so it proceeds regardless and never hangs. The progress ticks are purely
  informational.
- **Diagnostic value:** `ACKED` (step 3) vs `TIMEOUT` (step 4) tells you whether the reset actually
  reached the Teensy and it answered — the direct test of the flaky-UART theory.

**4. Clean GUI aftermath** (`MainWindow.py`)
- `_on_end_trial` opens the dialog and sets `self._shutting_down = True`; it no longer auto-disconnects
  at 200 ms.
- `_on_dev_disconnected` early-returns (no "unexpected disconnect" popup) while `_shutting_down`.
- `_on_shutdown_dialog_finished` (countdown done) **proactively** `qt_dev.disconnect()`s (intentional
  → no popup, and it pre-empts the ~10 s supervision-timeout wait), re-enables the Connect button,
  clears `_shutting_down`, returns to scan.

**5. `ACKED`/`REBOOTING` tick ordering — the `SEND_REBOOT` state** (`ComsMCU.cpp`)
- Two `_send_shutdown_progress()` calls in one loop iteration queue two BLE notifications before
  `BLE.poll()` can send the first, so under load the first got overwritten. `WAIT_ACK` now sends only
  `ACKED`/`TIMEOUT`, and the new `SEND_REBOOT` state sends `REBOOTING` in the next iteration — one
  notification per iteration.

### Known limitations / future work
- **Command channel still best-effort for non-critical commands.** Only the reset path is hardened
  (with-response). Param updates, etc. are still fire-and-forget and can drop under load. The real
  cure is ACK/retry on BLE writes **and** on the Nano↔Teensy UART. Larger, separate work.
- **Cosmetic (progress ticks):** the shutdown progress notifications are best-effort — fired faster
  than the BLE connection interval, so a *variable* number get dropped (seen anywhere from 0 to 3 of
  the 4 lost, worse under worn-trial congestion). The dialog is **robust** to this (it advances via
  the step-5 / disconnect / ~5 s fallback-timeout paths and never hangs — see `ShutdownDialog._on_stall`),
  and the functional shutdown is unaffected. A clean fix for *reliable* ticks would gate each progress
  step by ~one connection interval (~40 ms) in the Nano state machine (`_maybe_system_reset`) — a
  firmware change, not worth the added shutdown latency for a visual-only gain.
- **Motor CAN reliability (separate, unfixed):** during worn trials the right motor can drop out /
  return garbage feedback (encoding-max values like ±319 A). `Motor.cpp::read_data()` only updates
  `motor.p/v/i` on a valid matching CAN frame (else stale), and the timeout counter is **disabled**
  (`_handle_read_failure()` / `timeout_count++` are commented out "for AK60v3 integration"), so the
  firmware can't currently flag a dropout — `Timeout_ct`/`Error` in the logs stay 0. Examine the
  hardware; re-enabling timeout detection is a separate task.
- The `[shutdown-debug]` lines in `RtBridge.py` are **intentionally kept** (per user: terminal logs
  are only read when something's wrong, so more info is better). Remove if desired.

### Build gotcha (bit us once)
Switching the selected board between **Teensy 4.1** and **Nano 33 BLE** without clearing the build
cache produces a link failure: `uses VFP register arguments, ExoCode.ino.elf does not` +
a cascade of `undefined reference to String/pinMode/Serial8/...`. That's a **float-ABI mismatch from
stale mixed object files**, not a code error. Fix: select the right board, close the IDE, delete
`C:\Users\<user>\AppData\Local\arduino\sketches\<hash>\` (or the whole `sketches` cache), reopen,
rebuild.

### How to modify / debug in future
- **Change what's logged:** edit `SdLogger::_write_motor_row()` (columns) and the header string in
  `_open_session()`. Keep them in sync.
- **Change log rate / flush:** `config.ini [Logging]` (no reflash).
- **Change reset UX / steps:** the Nano state machine is `ComsMCU::_maybe_system_reset()`; step codes
  are `shutdown_progress::` in `ble_commands.h`; the GUI display is `ShutdownDialog.py` (`_STEP_LABELS`)
  and the parse is the `command == 'P'` branch in `RtBridge.feed_bytes`.
- **Adjust reset timing:** `ComsMCU.h` `_reset_ack_timeout_ms` (ack wait) and `_reset_flush_ms` (BLE
  flush before reboot); GUI countdown is `_REBOOT_COUNTDOWN_S` in `ShutdownDialog.py`.
- **Trace an end-trial live:** watch the GUI log for `End-trial reset 'Z' delivered` (reliable send
  worked) then `[shutdown-debug] parsed step=1..5`. `ACKED` (3) present = Teensy answered; `TIMEOUT`
  (4) = it didn't.
