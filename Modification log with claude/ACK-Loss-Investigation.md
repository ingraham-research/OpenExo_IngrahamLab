# Parameter-update ACK loss: "No ack ... Resending", yet the exo applied it

**Date:** 2026-09-16 · **Branch:** `fix_ack_failures` (= `main_working_branch` b3cff2c + this work)
**Status:** host-side fix implemented in `Python_GUI/external_control/` with 11 new headless tests
(49/49 in `Python_GUI/tests` pass). **Not yet run against the exo.** Firmware deliberately untouched.

## 0. Summary

The external loop sometimes prints `No ack for ... Resending...` for a write the Teensy did apply,
and gives up (and exits) when all attempts go unanswered. Two distinct mechanisms, both measured:

| | A. Garbled ACK | B. Silent ACK |
|---|---|---|
| Where the ACK dies | Teensy → Nano **UART** | Nano → GUI **BLE** (degraded link) |
| What the GUI receives | A fake "rejected, reason 1 (invalid message)" frame | Nothing |
| Dominates | Benchtop (09-09 stress run: 24 of 1707 writes) | Deliberately degraded link (09-14 stress test on the other laptop: 36 of 98 writes) |
| Command applied? | Proven for some frame shapes, ambiguous for others | **Proven** for at least one write (§2.3) |

No software race in the client, the remote service, RtBridge or MainWindow drops a well-formed ACK.

## 1. The ACK path

`main_external_control` → UDP → `remote/service.py` → `MainWindow._apply_param_update` →
`QtExoDeviceManager.updateTorqueValues` (**five** BLE write-without-response packets: `f` + four 8-byte
doubles) → Nano `BleParser` → `ble_handlers::update_param` → UART → Teensy
`UART_command_handlers::update_controller_param` → UART ACK → Nano `ComsMCU::update_UART` →
`_send_param_update_ack` → BLE notification → `RtBridge._handle_param_update_ack` →
`paramUpdateAckReceived` → MainWindow (5 s pending timer) and `service.publish_ack` → UDP → client.

## 2. Evidence

**Where to look.** `app_crash_*.log` is the main GUI log (all `OpenExo.*` loggers). RtBridge logs every
non-RT frame it receives, so every ACK that reached the GUI is there as
`[shutdown-debug] cmd='a' ... event_data='6800n1300n1000n100n0n'` (joint, controller, param, accepted,
reason, each ×100). `device_manager_*.log` is the BLE layer only and logs each write actually issued
(`Sending parameter update: ...`). In the 09-09 stress run the GUI issued all 1707 BLE writes, with no
write errors.

### 2.1 Mechanism A: garbled ACKs (UART)

When the Nano cannot parse the Teensy's UART ACK, `ComsMCU::_send_param_update_ack(UART_msg_t)`
(`ComsMCU.cpp:404-436`) **sends a made-up ACK**: `accepted=0, reason=1`, keeping only the fields it had
parsed before the failure. It checks length, then controller, then param, then accepted, then reason.
September logs on the lab PC (2269 writes):

| Frame (J, ctrl, param, acc, reason) | Count | Produced when | Also producible by the Teensy? |
|---|---|---|---|
| `(J, 0, 0, 0, 1)` | 16 | length or controller field broke | Yes: a damaged *command* gives the same frame |
| `(J, c, 0, 0, 1)` | 4 | param field broke | Yes, same |
| `(J, c, p, 0, 1)` | 8 | accepted/reason field broke | Only if the value became NaN/inf: practically no |
| `(J, c, p, 1, 1)` | 4 | reason field decoded as 1 | **No.** The Teensy always sends reason 0 with an acceptance |
| `(0, 0, 0, 0, 1)` | 1 | Nano BLE parser, malformed message | n/a |

The last two data rows prove that the Teensy's ACK gets damaged on the UART **after** the Teensy
applied the value. The first two rows can't be traced to one direction. If the damage was on the
incoming command, the value was **not** applied.

How the old client reacted:
- Rows 1–2 don't match the write's address, so they were ignored: 5 s of silence, then `No ack`.
- Row 3 matches, so it raised `Exo REJECTED ... invalid message` and **`main_external_control` exited
  without retrying.**
- Row 4 counted as success.

Cause of the UART damage: **unknown.** Two hypotheses:
- `UART_BAUD` 256000 is not a native nRF52840 rate (250000 is). The mbed driver that maps it is
  precompiled, so the rate it actually picks was not checked.
- The Nano's UART receive may be starved by BLE radio work (`UnbufferedSerial`, one byte per interrupt).

### 2.2 Mechanism B: silent ACKs on a degraded BLE link

In the 09-09 stress run, the RT stream had **0 exo-time gaps ≥45 ms in 27,746 frames**, none near any
failure. So no notification was dropped there. The 09-14 16:04 session (second laptop, 30 ms interval,
exo worn) was a **deliberate link stress test: the laptop was moved around on purpose** to see whether
disconnections come back. None did; the 16:31:32 "link lost" is the End-Trial reboot, 9.6 s after the
reset was delivered, and the GUI logged it as expected. In that session the same stream ran at **53 Hz with 217 gaps ≥45 ms per minute**, i.e. roughly 45% of notifications
lost. ACKs were lost at a similar rate: 36 of 98 attempts, all silent, only 1 garbled. The
patched `sendAclPkt` (`Libraries/ArduinoBLE/src/utility/HCI.cpp:815-825`) drops a notification after
50 ms without controller buffer space, and an ACK is just another notification. Silent writes sat in
slightly worse windows than answered ones (median 50% vs 37% of RT frames lost in the following 0.7 s).

The link was clean in the disconnect sense (`Nano-Hang-Watchdog-And-Breadcrumbs.md` §15.10), but not
in this sense. RT stream quality by condition:

| Trial | Setting | Rate | Gaps ≥45 ms per min |
|---|---|---|---|
| This laptop, bench, 09-12/13 | | 96 Hz | 0 |
| This laptop, worn, 09-13 | | 87–92 Hz | 16–31 |
| Other laptop, 09-14 14:01 | | 79.5 Hz | 52 |
| **Other laptop, worn, 09-14 16:04 (laptop moved on purpose)** | | **53 Hz** | **217** |

### 2.3 Proof that an unacknowledged write was applied

09-14, `app_crash_20260914_155432.log:139-144` and `trial_20260914_160454.csv`:
1. After a failed left peak-timing write, the loop parked to transparency. Left TorqScale 0 was accepted
   at 16:11:52.39.
2. Right TorqScale 0 was then sent at 16:11:52.39, 16:11:57.53 and 16:12:02.70. **No ACK frame of any
   kind** arrived for joint 36; the GUI timed out three times.
3. No other right-side write was sent until 16:13:29.
4. Right desired torque went from −6.9 N·m to exactly 0 in the 16:11:52–56 bin and stayed there.
5. Walking continued throughout: 6–7 stance onsets per 8 s on each leg, right toe FSR active.

### 2.4 Latency (sets the new timeout)

GUI-side, FIFO-matched per address, 5 s expiry:
- **All 2215 unilateral writes on the lab PC:** median 77 ms, p99.9 309 ms, maximum 453 ms.
- **September controller switches (22):** 45–453 ms.
- **09-14 link stress test:** median 285 ms, maximum 476 ms.

Every ACK slower than 1 s came from a July/August **bilateral** GUI-button write, which the
orchestrator never sends.

### 2.5 Ruled out

- **BLE congestion drops on the benchtop:** no RT gaps at all (§2.2).
- **Client / GUI / UDP software races:** code reviewed. The GUI issued every BLE write.
- **A lost BLE command write wedging `BleParser` and swallowing the retry:** in the stress run, 25 of
  25 retries succeeded. The single exception was End Trial. A wedge would show as
  "fail, fail, succeed", which survivorship bias would not hide.

## 3. What changed (host side only)

`Python_GUI/external_control/Utilities/OpenExoLink_utilities.py` (V0.2):
- **Garbled ACK = resend now.** Any ACK with `reason_code == 1` for the same joint is treated as
  "unknown" and resent at once, including accepted-looking ones. Every attempt carries the same
  absolute value, so resending is idempotent. Reasons 2–6 still raise.
- **Attempt bookkeeping.** Every attempt stays open until an ACK is matched to it, oldest first, and is
  written off **strictly before** it is resent: at `ack_timeout − ACK_WRITE_OFF_MARGIN` (0.1 s, capped at
  half the timeout). Both times are measured from one `time.monotonic()` stamp taken when the GUI
  replies. So no attempt is ever still open when its successor goes out, and a wall-clock adjustment
  can't reorder the two. An ACK landing inside the margin (0.9–1.0 s) is not credited and the write is
  resent; none has been seen above 0.48 s. An ACK that arrives when no attempt is open is dropped
  rather than held over. Every ACK frame is seen, via `_AckTrackingRemote._absorb`, including ones
  read during `set_param`'s own reply wait. This replaces the `last_ack()` snapshot.
  - **Why attempts close at the timeout (first built with 5 s, corrected the same day):** keeping them
    open longer protects against an ACK arriving later than the timeout being taken as confirmation of
    the next write. But on a lossy link it compounds:
    - A lost ACK leaves an open attempt, and the next write's ACK is credited to it.
    - That write then needs an extra attempt, and its own leftover attempt carries the debt forward.
    - At heel-strike rates the debt never expires, and each further lost ACK adds to it.
    - A test with a single lost ACK had the **third** write give up after 5 attempts.
  - **Remaining limit:** an ACK later than `ack_timeout` can still confirm the next write to the same
    parameter, the same exposure the old code had. None later than 476 ms has been seen. The fix is a
    value echo in the ACK (firmware).
- **One `Param_write_log` row per attempt.**
  - Result: `accepted`, `no_ack`, `garbled_ack`, `rejected (<reason>)`, `gui_refused` or `gave_up`.
  - Attempts: that attempt's number.
  - Filtering on `accepted` gives the old one-row-per-confirmed-write view.
  - **`gave_up` does not mean "not applied".**
- **Defaults:** `DEFAULT_ACK_TIMEOUT` 5.0 → 1.0 s, `DEFAULT_MAX_RETRIES` 3 → 5, poll step 0.2 → 0.05 s.

`main_external_control.py`: `ack_timeout = 1.0`, `max_write_retries = 5`, log-format comment.

`Python_GUI/tests/test_openexolink.py` (new): a fake GUI speaking the real wire protocol over localhost
UDP (11 tests; 49/49 in `Python_GUI/tests`). It covers:
- zeroed and full-address garbled ACKs;
- an accepted-looking garbled ACK;
- a real rejection;
- a lost ACK not taxing the writes that follow;
- an attempt being written off before it is resent;
- an ACK arriving between writes not confirming the next one;
- a clean ACK from an earlier attempt after a garbled resend;
- per-attempt logging, give-up and GUI refusal.

Run with the `biomotum` conda env:
`E:\MiniConda\envs\biomotum\python.exe -m pytest -q Python_GUI/tests`. The base MiniConda Python has no
pytest.

**Expected effect on a link as bad as the 09-14 stress test's** (37% silent, assuming independence; a
deliberate worst case, not a typical worn session):
- Chance of giving up per write: 0.37³ ≈ 5% before, 0.37⁵ ≈ 0.7% now.
- A silent attempt costs 1 s instead of 5 s.
- The link itself is unchanged, so about a third of writes will still need a second attempt.

## 3.1 First hardware run (2026-09-16 16:53, lab PC, benchtop, trial ON, TorqScale 0)

`stress_test_ble.py`, 226 s, stopped by the operator. Files: `Logs/Stress test/*_20260916_165324.txt`,
`app_crash_20260916_165143.log`, `device_manager_20260916_165148.log`, `trial_20260916_165214.csv`.

**The fix did its job:**
- 651 of 651 writes confirmed, none given up.
- Attempts needed: 548 took 1, 83 took 2, 15 took 3, 3 took 4 and **2 took 5**. With the old 3-attempt
  setting, 5 writes (0.8%) would have ended the run.
- Each miss cost 1 s instead of 5 s.
- No late ACKs: 651 ACK frames reached the GUI, one per accepted attempt, none after a timeout.
  Latency (each ACK against the latest write for its joint): median 120 ms, p99.9 221 ms, slowest 339 ms.
- No garbled ACKs.

**But 131 of 782 attempts (16.8%) got no ACK at all**, ten times the 09-09 benchtop run (1.6%, then at
a 7.5 ms interval and older firmware). The link was healthy: 30 ms interval, RT 89 Hz, 20 gaps ≥45 ms
per minute.

Ruled out for these losses:
- **Congestion drops:** windows after silent attempts show no larger RT gaps (median 30 ms both, 2% ≥100 ms
  both), and a 50 ms `sendAclPkt` give-up would leave one.
- **Ping collisions:** each update's coroutine starts about 1 ms after submission, and no ping was submitted
  inside that window.
- **A lost command packet wedging `BleParser`:** that would make the retry fail too, pushing P(fail | previous
  attempt failed) towards 0.8. Measured: 0.19 at attempt 2 and 0.25 at attempt 3, against 0.16 for a first
  attempt. So losses are close to independent, with mild clustering.
- **Late or garbled ACKs:** see above.

Still possible:
- The ACK notification is dropped inside the Nano's BLE stack without blocking (command applied).
- A whole command is lost (not applied).
- A UART packet is lost.

Discriminating zero-torque experiment: the same script with the **trial OFF**, which removes the ~100 Hz
RT notifications.

**Analysis trap:** pairing ACKs to writes with MainWindow-style bookkeeping (oldest first, 5 s expiry)
gives nonsense multi-second latencies for this run. The retries now come 1 s apart, so every lost ACK
shifts all later pairings by one. The GUI's own "Controller update failed" warnings during such a run
are mostly spurious for the same reason. The §2.4 figures are unaffected: those runs retried only after
5 s, when the GUI's record had already expired.

## 3.2 Trial-OFF control run (2026-09-16 17:21, same PC, interval, firmware and script)

289 s: **3071 of 3071 writes ACKed on the first attempt**, 0 timeouts, 3072 ACK frames at the GUI.
With the trial on, 16.8% of attempts were silent. **The loss needs the ~100 Hz RT stream to be
flowing.**

Checked against the trial-on CSV:
- **Not an ACK overwritten by an RT frame:** duplicate RT rows follow lost-ACK writes and ACKed writes
  equally often (12% vs 12% within 0.3 s). The build also uses `CORDIO_ZERO_COPY_HCI 1`
  (`mbed_config.h:52`), so each notification is copied into its own buffer.
- **Not the congestion give-up** (§3.1).

**Leading hypothesis (not proven): an ACK and an RT frame sent in the same Nano loop pass, with the
first one lost.**
- The Teensy loop runs at 500 Hz and sends an RT frame (I2C) on every ~5th pass (≥9 ms), so ~20% of
  passes send one.
- In such a pass the parameter-command ACK (UART) goes out just before the I2C RT write
  (`Exo.cpp:102-117`).
- The Nano then handles both in one pass: `update_UART` sends the ACK notification, and `update_gui`
  sends the RT notification straight after.
- `ComsMCU.cpp:489-492` already records, from the End-Trial work, that two `writeValue()` calls in one
  pass make the first one get *"overwritten/dropped"*. That was fixed there by giving each its own pass.
- The low-level mechanism is not identified.
- Expected loss: ~20% × P(drop). Observed: 16.8%.

**Consequence:** if this holds, the lost ACKs are for commands that **were applied**, which is the
original symptom. A Nano-only fix (send the RT frame one pass later whenever an ACK went out) is now
testable **at zero torque** with `stress_test_ble.py`, trial on: 17% → ~0% would confirm it.

## 3.3 Nano experiment (2026-09-16): hold the RT frame after an ACK. REFUTED and REVERTED

**Status: fully reverted.** `ExoCode/src/ComsMCU.h` and `ComsMCU.cpp` were restored to `HEAD` with
`git checkout`, so `ExoCode/` has no changes from this work. The Nano was last flashed with a hold
build (3 ms or 35 ms): **reflash from the current tree** before further tests, so the baseline matches
§3.1. The description below is kept for the record.

It was built for the Nano only (`ComsMCU.h`, `ComsMCU.cpp`) and compiled clean for `nano33ble` and
`teensy41`.
- `_send_param_update_ack()` and `_send_shutdown_progress()` now call `_hold_rt_frame()`, which starts a
  hold of `k_rt_hold_us` = 3 ms.
- `update_gui()` marks new RT data as pending and sends it only once no hold is active. With no hold, the
  frame goes out in the same pass exactly as before.
- **Why time and not "next pass":** `handle_updates()` calls `BLE.poll()` only every 1 ms, and a loop
  pass can be much shorter. So "next pass" guarantees nothing happens between the two notifications.
- 3 ms is still well under the ~9 ms RT spacing.
- `poll()` does not touch `rt_floats` when nothing new arrived, so the held frame survives. If a newer
  frame does arrive, the newer one is sent.
- The old `else` branch of `update_gui()` (whose actions are all commented out) now runs on
  `!fresh_rt_data`, so its behaviour is unchanged.

**Test:** flash the Nano, run `stress_test_ble.py` with the **trial on**, and compare the `no_ack` share
with §3.1 (16.8%). Also check that the trial CSV rate stays near 89–96 Hz.
- **If it doesn't improve:** set `k_rt_hold_us` to one connection interval (~35 ms). That tests a
  per-connection-event limit.
- **If that doesn't help either,** the same-pass hypothesis is wrong.

**Result: hypothesis REFUTED.**
- **3 ms hold** (18:04, 246 attempts): 15.0% silent.
- **35 ms hold** (18:16, 120 attempts): 15.8% silent. RT loss rose to ~12% from 4–7%, which proves the
  hold was active.
- Neither changed the ACK loss, so the change was reverted completely rather than kept at 3 ms. It
  touched the 100 Hz RT path for no measured benefit, and later firmware experiments are cleaner
  against the original code.
- Also measured: on the trial-on runs, ACKs are lost about 3× more often than RT notifications (16% vs
  4–7%, against the 09-09 0.961 no-loss reference).
- There were no decode errors. The UART carries only request/response traffic, so nothing competes with
  the ACK there.

## 3.4 Diagnostic build: counters in every ACK (2026-09-16, TEMPORARY)

Nano only: `ComsMCU.cpp` (`#define PARAM_ACK_DIAG 1`) and three members in `ComsMCU.h`.
**Compiled clean for `nano33ble`; the `teensy41` binary is byte-identical. Not yet flashed.**

Every parameter-update ACK now carries 8 fields. The extra ones are:

| Field | Counter | Incremented in |
|---|---|---|
| 5 | `ack_seq`: ACK notifications sent, this one included | `_send_param_update_ack(request, ...)` |
| 6 | `cmds_from_gui`: complete `update_param` BLE commands received, counted before validation | `_process_complete_gui_command` |
| 7 | `acks_from_teensy`: parameter ACKs received from the Teensy over UART, parsable or not | `_send_param_update_ack(UART_msg_t)` |

- Counters wrap at 1,000,000; only differences matter.
- The length is overridden only inside the `#if`; the shared command table still says 5.
- Set the flag to 0 for the original 5-field ACK.

**GUI compatibility:** RtBridge reads the first 5 fields and logs the whole frame, so no PC change is
needed. This is now pinned by `Python_GUI/tests/test_rtbridge_ack.py`; the suite passes 51/51.

**How to read it:** over the GUI commands sent since the first diagnostic ACK, and relative to it:
- **GUI → Nano lost** = GUI commands − `cmds_from_gui`
- **UART/Teensy lost** = `cmds_from_gui` − `acks_from_teensy`. A Nano-side rejection would also land
  here; expect 0 with valid values.
- **Nano → GUI lost** = `ack_seq` − ACKs received

The analysis script (session scratchpad `ack_diag.py`) was checked on a synthetic log with one loss of
each kind, and attributed each correctly.

### 3.4.1 RESULT (2026-09-16 18:42, 235 s, trial ON, benchtop, lab PC, 30 ms)

`app_crash_20260916_184125.log`, `trial_20260916_184236.csv`, `Logs/Stress test/*_20260916_184242.txt`.
1064 GUI commands after the first diagnostic ACK:

| Hop | Lost |
|---|---|
| GUI → Nano (command never arrived) | **0** |
| At the UART/Teensy (Teensy never ACKed) | **1** |
| **Nano → GUI (Nano sent the ACK, GUI never received it)** | **127** |

- **The original symptom is proven.** The silent "No ack" attempts are commands that reached the Nano, that
  the Teensy applied and ACKed, and whose ACK notification the Nano sent. They were lost between
  `writeValue()` and RtBridge.
- **The 3 garbled ACKs** had `acks_from_teensy` incremented, so the Teensy replied and the damage was on
  the Teensy → Nano UART. Mechanism A is confirmed in that direction.
- **Not the 50 ms `sendAclPkt` give-up:** lost ACKs had an RT gap ≥45 ms in the following 0.3 s only 1% of
  the time, the same as ACKs that arrived. The Nano loop did not stall.
- **Not the GUI application:** `QtExoDeviceManager._on_rx` forwards every notification unfiltered, and
  RtBridge logs every ACK frame it gets.
- **What's left:** the notification is dropped inside the Nano's Bluetooth controller or in Windows' stack.
  The RT stream loses about 11% of its notifications the same non-blocking way (89 Hz here).
- **Pattern:** loss appears at 30 ms and not at 7.5 ms (09-09, no RT loss), and not with the trial off.
  That points to a per-connection-event throughput limit. The Nano's Cordio LL has only 4 TX buffers
  (`MBED_CONF_CORDIO_LL_TX_BUFFERS 4`), and ~96 notifications/s at 30 ms is ~3 per event, of ~95 bytes
  each (ASCII RT frames). Not proven.
- **Retries coped:** 934 of 934 writes confirmed. Attempts needed: 822 took 1, 98 took 2, 9 took 3, 5 took
  4. ACK latency: median 97 ms, max 206 ms.

## 4. Open / not done

- **Next: a real worn session** with the diagnostic firmware and `main_external_control`, to see whether
  writes still give up completely with 5 attempts at 1 s. Bench testing is finished.
- **Remove the `PARAM_ACK_DIAG` counters after that session** (set the flag to 0, or revert
  `ComsMCU.h`/`.cpp`). They are temporary.
- **Duplicate ACKs were considered and deliberately not built** (2026-09-16). The 12% extra attempts are
  acceptable, and duplicates would not rescue bad-link stretches. Revisit only if the worn session shows a
  need.

- **Link quality is the real lever for mechanism B.** Laptop, body orientation, antenna placement. See
  the "human body" finding in `BLE-Handshake-Controller-List-Loss.md` and memory notes.
- **Firmware (on hold by decision, 2026-09-16; hard to bench-test without non-zero torque):**
  - a checksum on UART packets;
  - echo the value in the ACK;
  - a distinct reason code for "ACK damaged in transit" instead of reason 1;
  - try `UART_BAUD` 250000 or 115200;
  - give ACKs priority over the RT stream on the Nano.
- **Latent safety issue:** no UART checksum and no value echo. A command whose value is damaged but
  still within bounds would be applied **and** ACKed as accepted, and nothing could detect it. Not
  observed.
- `stress_test_ble.py` was brought in line on 2026-09-16:
  - 1 s timeout and 5 attempts.
  - A timestamped per-attempt log (`Logs/Stress test/param_write_<stamp>.txt`) and an attempt tally in
    the summary.
  - "Rejected" now means only genuine firmware refusals.
  - It was smoke-run end to end against a fake GUI; not yet on the exo.
- MainWindow's own 5 s pending timer still shows "Controller update failed: no device acknowledgement"
  for a write whose retry succeeded. This is cosmetic.
- About half of the benchtop timeouts were silent with no degraded link. Their cause, and whether those
  commands were applied, is unknown.
