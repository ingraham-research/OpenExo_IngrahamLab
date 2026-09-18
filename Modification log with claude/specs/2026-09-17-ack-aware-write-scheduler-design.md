# ACK-aware write scheduler for the external control loop: design

**Date:** 2026-09-17 · **Branch:** `fix_ack_failures` · **Status:** design agreed in chat, awaiting spec review
**Touches:** `Python_GUI/external_control/` only (PC side). No firmware or GUI change.

## 1. Why

Measured on the exo (diag firmware, 2026-09-16 bench and 2026-09-17 worn, about 1,740 commands):

- **A missing ACK almost never means a missing command.** In both worn sessions (675 attempts) every ACK
  carried equal counters (Nano ACKs sent = commands received from the GUI = replies from the Teensy), and the
  command count matched the GUI's send count. So every silent attempt reached the Teensy and was ACKed,
  and the ACK was then lost between the Nano and the GUI. Across all diag runs: 250 lost ACKs against 1
  command with no Teensy reply (bench, 09-16 18:42).
- **The Teensy stores the value before it ACKs** (`ExoCode/src/uart_commands.h:768` then `:778`).
- **About 17–20% of attempts get no ACK** on a worn link, independent of write rate.
- **Latency is the Nano's outgoing notification backlog** (shared with the RT stream): 110 ms with no backlog,
  about 340 ms with one, max 569 ms. The backlog in session 2 began before any write, so the writes don't
  cause it.

So today's loop spends 1 s per lost ACK re-sending a value the exo already holds, retries stale values when
a newer one is waiting, and ends the session after 5 silent attempts. The new loop does none of these.

### What an ACK carries (unchanged by this work)

`joint_id`, `controller_id`, `param_index`, `accepted`, `reason_code` (0 ok, 1 malformed, 2 joint, 3 controller,
4 bad index, 5 out of bounds, 6 not integer). **No value and no sequence number**, so two writes to the same
parameter on the same leg produce identical ACKs. That is why only one command may be on the air at a time.

## 2. Rules (agreed 2026-09-17)

1. **One command on the air at a time**, globally. A command goes out, then waits for its ACK or for
   `ack_timeout` (1.0 s), whichever comes first. The only history of two in flight (July/August GUI
   bilateral writes) is also the only history of ACKs slower than 1 s.
2. **Two kinds of write:**
   - **Setup writes** (engage splineAlt at TorqScale 0, then plantar and dorsi magnitudes, then the fixed
     peak timing in the plantar/dorsi modes) block and are retried **until confirmed**, with no cap, before
     the loop starts. They must finish first: a later write
     reaching a joint still on another controller would switch it and reload every parameter from the SD
     card defaults (`uart_commands.h:764`), including a non-zero TorqScale. A refusal during setup still ends
     the session, as today: nothing is being driven yet.
   - **Everything in the loop is a simple write**: peak timing and **every TorqScale value, including 0**.
     The game-theory backend never sends a session-end write, so no loop write is special.
3. **Simple writes never block the loop and never end it.**
4. **Parking at exit** (Ctrl-C, or game-theory completion) is retried until confirmed; a second Ctrl-C
   abandons it and prints `park NOT confirmed`. The command on the air, if any, resolves first. Every other
   waiting value is dropped and logged `unconfirmed at exit`, so only the two park commands go out.
5. **The GUI reporting the exo disconnected ends the loop.** The Nano cannot be reconnected mid-trial without
   side effects, so this is already fatal. Parking is skipped (nothing can reach the exo), and the terminal
   says so.
6. **Nothing else is destructive.** Every other problem is reported by the ACK check (section 3.3.1).

## 3. Scheduler

### 3.1 State, per address

An address is one (joint, controller, parameter). The loop uses four: peak timing and TorqScale on each leg.

| Field | Meaning |
|---|---|
| `wanted` | The next value to send, or none. Set by a new request, or by a timeout with nothing newer (retry). |
| `on_air` | The command currently sent and waiting: value, send time, attempt number. Only one address has it. |
| `confirmed` | Last value an accepted ACK confirmed. |
| `latest` | Last value requested. Used to ignore repeat requests. |
| `no_ack_tab` | Seconds this address's own commands have spent on the air without an ACK since its last ACK. |

`no_ack_tab` is the "running flag" from the operator's clarification:
- it grows **only** while this address has a command on the air, so at most +1 s per failed attempt;
- it **pauses** while other addresses are on the air or while this one waits its turn;
- a new value **does not clear it**;
- **only an ACK** for this address clears it (accepted, or a firmware refusal, since both prove the link
  delivered); a garbled ACK does not.

### 3.2 Requests

`request(address, value, requested=None, label="")` returns immediately.
- If `value` equals the address's `latest`, it's a no-op.
- Otherwise `latest` and `wanted` become `value`. If a previous value was waiting and never went on the air,
  that value is logged as `replaced before sending`.
- `requested` is the value before ActionMap's clamp, kept only for the machine-action log.

### 3.3 Each service pass

`service(budget)` is called once per loop pass and spends the loop's idle time waiting on the socket, so an
ACK is acted on as soon as it arrives instead of on the next 50 ms tick. For each frame read, and when the
wait ends:

1. **Status frames:** `disconnected` sets `link_down` (the loop ends, rule 5). `device_error` (a BLE-layer
   report such as "Not connected" or a failed BLE write, `MainWindow.py:212`) is **ignored**: the write it
   affected gets no ACK, so the ACK check in step 3 already reports it. The GUI's own log keeps the message.
2. **An ACK matching the command on the air** (same address) resolves it:
   - **accepted:** `confirmed` = its value, `no_ack_tab` = 0, command logged `accepted`;
   - **firmware refusal (reason 2–6):** loud warning, value dropped (not retried, since resending the same
     value is refused again), `no_ack_tab` = 0, command logged `rejected (<reason>)`;
   - **garbled (reason 1):** treated as a timeout (step 3), without waiting out the rest of the second.
   Matching uses today's `_ack_answers` rules, where a garbled ACK's zeroed fields match anything. An ACK
   that matches nothing on the air is ignored (a late ACK, or a GUI-button write), as today.
3. **Timeout** (`ack_timeout` since this attempt went out, or a garbled ACK): add the attempt's wait to
   `no_ack_tab`. If `wanted` already holds a newer value, the old command is logged `superseded (no ACK)`.
   Otherwise `wanted` = the same value (retry, attempt number + 1).
4. **If nothing is on the air**, send `wanted` for the next address in a fixed round-robin order, starting
   after the address sent last. After a timeout on the left leg, the right leg goes next if it has anything,
   and the left leg's next turn sends its newest value if one arrived, else a retry.
5. **Warnings:** while any address has `no_ack_tab` ≥ `NO_ACK_WARN_S` (5 s), print one line per second
   naming the leg and parameter, the tab, the attempts since its last ACK, and the value wanted.

### 3.3.1 Terminal output

One line per event, and no other warning source:

| Event | Line (example) | Printed |
|---|---|---|
| First send of a value | `-> Ankle(L) plantar peak time = 43.0 (first send)` | verbose |
| Retry of the same value | `-> Ankle(L) plantar peak time = 43.0 (resend, attempt 2)` | verbose |
| Newer value replaces an unconfirmed one | `-> Ankle(L) plantar peak time = 29.0 (replaces unconfirmed 43.0)` | verbose |
| Attempt failed | `No ACK for Ankle(L) plantar peak time = 43.0 (attempt 1), pending resend` (or `Garbled ACK ...`; ends `, newer value 29.0 waiting` when one is) | always |
| Accepted | `Exo ACCEPTED Ankle(L) plantar peak time = 43.0 (attempt 2)` | verbose |
| Firmware or GUI refusal | `Exo REJECTED ... : <reason>. Value dropped.` | always |
| No-ACK tab ≥ 5 s | `WARNING: no ACK for Ankle(L) plantar peak time for 5.0 s (5 failed attempts), wanted 29.0` | always, once per second |

`main_external_control` runs with verbose on. `stress_test_ble.py` runs with it off (a write every 50 ms would
drown the terminal), as today.

**Choosing 5 s:** a 1 s timeout per attempt makes the tab about 1 s per consecutive failure. The longest run
seen at one address in about 2,100 writes (09-16 bench, 09-17 worn) is 4 failures, so 5 s would never have
fired. A false warning costs only terminal lines.

**Write-off margin removed.** V0.2 wrote an attempt off 0.1 s before its resend so that no attempt was ever
open when its successor went out. With one command on the air, the timeout closes the attempt and only then
does the next send happen, so the order is guaranteed without a margin. An ACK at 0.9–1.0 s is now credited
instead of thrown away. Residual, unchanged from today: an ACK slower than the timeout, arriving after the
**same** address has been re-sent, confirms the re-send. The slowest seen is 569 ms.

### 3.4 Blocking wrapper

`set_param_confirmed(address, value, label, max_retries=...)` stays, rebuilt on the same engine: request,
then service until this command resolves.
- `max_retries=None`: unbounded, warnings as above. Used by setup. (Parking uses the scheduler directly,
  so both legs take turns; see section 4.)
- A number: raises `OpenExoLinkError` after that many silent or garbled attempts, as today. Used by
  `stress_test_ble.py` (5). Its current contract is kept: a firmware refusal raises with `reason` set, and
  giving up raises with `reason=None`.
- GUI refusal (`RemoteError` with a code: bad address or value) raises at once in both modes, as today. In
  the loop the same refusal is a loud warning and the value is dropped.
- No reply from the GUI at all (`RemoteError` without a code, the client's own 2 s timeout: a frozen GUI) is a
  failed attempt like a timeout, logged `gui_no_reply`, and is retried.
- Watches the status stream. `disconnected` during setup or parking stops the wait and raises.

## 4. Changes by file

- **`Utilities/WriteScheduler_utilities.py` (new):** the scheduler (3.1–3.3) as a pure state machine, with no
  sockets and no clock of its own (every call takes `now`), so the timing rules are testable with a fake clock.
- **`Utilities/OpenExoLink_utilities.py` (V0.3):** wires the scheduler to the GUI socket; the wrapper (3.4); the
  machine-action log (section 5); status frames are also caught while waiting on the GUI's reply to a
  `set_param` (today they are silently dropped there). `pump()` stays, as one non-blocking `service()` pass (never
  waits, like V0.2's), because `stress_test_ble.py` calls it every loop. `DEFAULT_MAX_RETRIES` remains for the wrapper.
- **`Utilities/ActionMap_utilities.py`:**
  - `engage_controller_safely` and `apply_torque_magnitudes` use the wrapper with `max_retries=None`.
  - `apply_torque_percentage` and `apply_peak_timing` become requests and return at once.
  - `park_to_transparency` requests TorqScale 0 on both legs, then services until both are confirmed; a
    second Ctrl-C abandons it.
  - `last_applied_action` goes. In its place, `latest_torque_request()` (both legs' `latest`, if equal) and
    per-leg `confirmed` values from the link.
- **`main_external_control.py`:**
  - The loop calls `service(remaining loop time)` instead of `pump()` plus `sleep`.
  - `link_down` ends the loop and skips the park.
  - The `except OpenExoLinkError: break` blocks around loop writes go (loop writes no longer raise).
  - UDP mode's "already zero" and "restore after a zero" checks use `latest_torque_request()`.
  - Machine-action rows are written by the link, not by the loop.
  - The 0.5 s UDP rate limit and the setup sequence are unchanged.
  - `max_write_retries` goes: nothing in this file is capped any more (setup and loop retry until an ACK).
- **`stress_test_ble.py`:** no change expected; must still run against the rebuilt wrapper.

## 5. Logs

**`Param_write_log.txt`, one row per attempt, format unchanged:** `Python time,Target,Value,Result,Attempts`.
Results: `accepted`, `no_ack`, `garbled_ack`, `rejected (<reason>)`, `gui_refused`, `gave_up` (capped
wrapper only).

**`Machine_action_log.txt`, one row per command, written when its fate is known:**
`Python time,Elapsed time,Target,Value requested,Value sent,Result,Attempts`.
- `Python time` is when the command was requested; `Elapsed time` is measured from the loop's `start_time`.
- Results:
  - `accepted`;
  - `superseded (no ACK)`: it went on the air, got no ACK, and a newer value replaced it;
  - `replaced before sending`;
  - `rejected (<reason>)`;
  - `unconfirmed at exit`.
- A superseded command was almost certainly applied (section 1). The label says what was observed, not what
  happened on the exo.

## 6. Testing

- **Headless, `tests/test_openexolink.py`,** with FakeGui extended to lose chosen ACKs, garble, refuse and
  report status, and to **fail the test if a second `set_param` arrives while one is unresolved**:
  - one command on the air at a time;
  - round-robin: after a timeout on the left leg the right leg goes next;
  - supersede on timeout when a newer value waits; retry when nothing newer waits;
  - `replaced before sending`; repeat requests of `latest` are no-ops;
  - the tab: pauses while other addresses are on the air, survives a supersede, clears only on an ACK,
    +1 s per timed-out attempt;
  - a warning at 5 s, repeated once per second while it holds, and silent after the clearing ACK;
  - a firmware refusal drops the value, clears the tab, and doesn't end the loop;
  - a garbled ACK acts as an immediate timeout;
  - `disconnected` sets `link_down`; `device_error` is ignored;
  - the terminal line for each event in 3.3.1, including first send, resend and replacement;
  - the wrapper: unbounded vs capped, and its exceptions (the stress-test contract);
  - both logs' rows and results.
- **The existing V0.2 tests** are kept where their rule still holds (garbled resend, refusal raises in
  the wrapper, logging, gave-up). Tests whose premise was two attempts open at once, or the 0.1 s
  write-off margin, are rewritten to the one-on-the-air rule, and each rewrite is named in the handover.
- **ActionMap and loop:** headless tests with a fake link, covering park-until-confirmed and park skipped on
  disconnect (a small `_park_or_skip` helper in `main_external_control`). The UDP zero/restore checks stay
  inline in `main()`, which is all `input()` prompts; they are covered by testing `latest_torque_request()`
  and by review, not by a loop test.
- **Full suite:** `E:\MiniConda\envs\biomotum\python.exe -m pytest -q Python_GUI/tests`.
- **On the exo (operator):**
  - first `stress_test_ble.py` at zero torque, for the wrapper;
  - then a worn `main_external_control` UDP session. Pass means: 0 loop exits except by the operator, the
    Machine_action_log accounts for every value, and warnings appear only during real ACK droughts.

## 7. Out of scope

- A value echo or tag in the ACK (firmware). Revisit only if a per-leg rate faster than about 1–2 Hz is
  needed, or exact per-iteration confirmation for the game-theory backend.
- MainWindow's own 5 s pending-ACK warnings (cosmetic, separate).
- Folding `PARAM_ACK_DIAG` into `EXO_DIAG` (diagnostic-cleanup plan, Task 5).
