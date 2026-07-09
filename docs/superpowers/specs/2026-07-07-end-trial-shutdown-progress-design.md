# End-Trial Shutdown Progress + Teensy-Ack Handshake — Design

**Date:** 2026-07-07
**Status:** Approved (pending written-spec review)
**Author:** Zijie Jin + Claude

## Goal

When the user clicks **End Trial**, replace the current "immediately return to scan + disconnect
after 200 ms" behavior with a **modal progress dialog** driven by live feedback from the exo, so
that:

1. **UX:** the user sees what the shutdown is actually doing (received → sent to Teensy → Teensy
   acknowledged → rebooting), then a reboot countdown, instead of an abrupt "unexpectedly
   disconnected" popup.
2. **Diagnostic:** the sequence exposes **where a bad trial-end fails** — in particular, whether
   the reset actually reaches the Teensy (the open question behind the intermittent corruption /
   stuck-boot). If the Teensy confirms, you see `ACKED`; if not, you catch `TIMEOUT` with the exact
   failing step.

## Non-goals

- Not fixing the unreliable Nano↔Teensy UART link itself (acks/retries) — this feature *observes*
  it. A reliable-UART overhaul is separate future work.
- Not changing the log-close-before-reset safeguard (already implemented: `SdLogger::close_active()`
  in the reset handler). This feature builds on it.
- No auto-reconnect after reboot — return to the scan screen (user's choice).

## Key constraint

**BLE drops the moment the Nano reboots.** So live feedback exists only in the *pre-reboot* window.
Fortunately the Nano already defers its reboot by `_reset_delay_ms = 5000` (`ComsMCU.h:102`), so the
entire handshake (tell Teensy → get its ack → relay it) fits inside that 5 s window, *before* the
Nano reboots. After the reboot, the GUI can only show a countdown and then reconnect — no live data
is physically possible.

This yields a **two-phase** design:
- **Phase 1 (live, ≤5 s):** Nano still connected, streams real progress messages.
- **Phase 2 (blind, during reboot):** BLE gone, GUI shows a ~6 s countdown, then returns to scan.

## Design decisions (locked)

| Decision | Choice |
|---|---|
| Teensy ack | **Include it** — Teensy sends a UART ack before rebooting; Nano waits (within the 5 s) and relays it. This is the core diagnostic. |
| GUI presentation | **Modal dialog** over the trial page (step checklist + countdown). |
| After reboot countdown | **Return to the scan screen** (manual reconnect). |
| On a stalled step | **Show the stalled step + manual options** (`Force disconnect`, `Retry`). "Reboot anyway" dropped — the Nano self-reboots on timeout, so it's redundant. |

## The protocol / sequence

```
GUI: End Trial → write G (stop), w (motor off), Z (reset) → open Shutdown modal, STAY connected

Nano, on Z (state machine inside the 5 s window, non-blocking):
  PENDING   → BLE progress RECEIVED  ("Received end request")
  SENT      → send reset (get_system_reset) to Teensy over UART
            → BLE progress SENT      ("Sent stop + reset to Teensy")
  WAIT_ACK  → poll update_UART for the Teensy reset_ack:
                ack received → BLE progress ACKED    ("Teensy acknowledged, logs closed")
                ~3 s no ack  → BLE progress TIMEOUT  ("Teensy did NOT confirm")
  REBOOTING → BLE progress REBOOTING ("Rebooting…") → brief BLE flush → exo_system_reset()

Teensy, on get_system_reset:
  send reset_ack UART to Nano FIRST (so it's on the wire before we die)
  → close_active() (flush+close logs; already implemented)
  → exo_system_reset()

BLE drops (Nano rebooted) → GUI modal switches to "Exo rebooting… ~6 s" countdown → return to scan
Any step stalls (GUI per-step timeout) → mark it failed + show [Force disconnect] / [Retry]
```

Secondary benefit: because the Nano now waits for the Teensy's ack before rebooting, the two MCUs
reboot closer together, which should reduce the desync that can leave the exo stuck at the
post-reboot config handshake.

## Message definitions

**New UART command — `reset_ack` (Teensy → Nano).** A short, data-less UART message the Teensy emits
in its `get_system_reset` handler before rebooting. Added to `UART_command_names` alongside the
existing `get_system_reset` / `update_system_reset`.

**New BLE message type — `send_shutdown_progress` (Nano → GUI).** A `BleMessage` with a new
`ble_names` command char and one data byte = step code:

| Step code | Meaning |
|---|---|
| 1 `RECEIVED` | Nano received the end/reset request |
| 2 `SENT` | Nano sent stop+reset to the Teensy over UART |
| 3 `ACKED` | Teensy acknowledged (received reset; logs will be closed) |
| 4 `TIMEOUT` | Teensy did not acknowledge within the window |
| 5 `REBOOTING` | Nano is about to reboot |

Sent via the existing `_exo_ble->send_message(...)` path (same mechanism as `send_real_time_data`).

## Components

### 1. Shared firmware definitions
- `UART_command_names::reset_ack` (new byte code) in `uart_commands.h`.
- `ble_names::send_shutdown_progress` (new char) + its data length in `ble_commands.h`.
- Step-code constants (shared enum/defines) so Nano and GUI agree on the codes.

### 2. Teensy — `get_system_reset` handler (`uart_commands.h`)
Send the `reset_ack` UART message to the Nano **first** (then `Serial`/UART flush so it's on the
wire), then the existing motor-disable → `set_status(trial_off)` → `close_active()` →
`exo_system_reset()`.

### 3. Nano — reset handshake state machine (`ComsMCU`)
Replace the current `_maybe_system_reset()` ("wait 5 s, send reset, `delay(10)`, reboot") with a
non-blocking state machine (`PENDING → SENT → WAIT_ACK → REBOOTING`). Each transition emits the
corresponding `send_shutdown_progress` BLE message. `WAIT_ACK` polls the UART for `reset_ack`
(detected in `update_UART`) with a ~3 s timeout; on ack or timeout it advances. Must stay
non-blocking so the Nano's loop keeps servicing UART (to receive the ack) and BLE (to send progress).
`_reset_delay_ms` (5 s) becomes the overall budget.

### 4. GUI — `RtBridge` (`services/RtBridge.py`)
Add parsing for the `send_shutdown_progress` message type → emit
`shutdownProgressReceived = QtCore.Signal(int)` carrying the step code.

### 5. GUI — `ShutdownDialog` (new, `Widgets/ShutdownDialog.py`)
Modal `QDialog` with a step checklist (Received / Sent to Teensy / Teensy acknowledged / Rebooting),
each row pending (spinner) → done (check) → failed (red). Behavior:
- Driven by `shutdownProgressReceived(step)`.
- On BLE `disconnected` → switch to a **~6 s countdown** ("Exo rebooting…"), then close and return
  to scan.
- Per-step GUI timeout (no progress for ~5 s) → mark that step failed and reveal `Force disconnect`
  (GUI-side BLE disconnect) and `Retry` (re-send G/w/Z) buttons.

### 6. GUI — `MainWindow._on_end_trial` (`MainWindow.py`)
Open the `ShutdownDialog` instead of navigating to scan + disconnecting at 200 ms (the Nano's reboot
now ends the link). Wire `rt_bridge.shutdownProgressReceived` and `qt_dev.disconnected` into the
dialog. Keep sending `G`, `w`, `Z` exactly as today (the command sequence is unchanged; only the
Nano's handling of `Z` and the GUI's post-send UX change).

## Error handling / edge cases

- **Teensy never acks** (UART reset-forward lost or Teensy hung): Nano emits `TIMEOUT`, GUI marks
  the "Teensy acknowledged" step failed, offers `Force disconnect` / `Retry`. This is the diagnostic
  win — it means the reset likely didn't reach the Teensy.
- **A progress message is lost over BLE:** the GUI per-step timeout still advances the UX (marks the
  step unknown/stalled) rather than hanging.
- **BLE drops earlier than expected** (e.g. Nano reboots sooner): the `disconnected` handler moves
  the dialog straight to the countdown.
- **User closes the dialog / Force disconnect:** GUI disconnects and returns to scan; the exo still
  reboots on its own timer.

## Testability
- The `ShutdownDialog` + `shutdownProgressReceived` signal can be exercised **without the exo** by
  feeding simulated step codes (unit/manual test of the UX and failure paths).
- The firmware handshake (Teensy ack, Nano state machine) needs on-device testing, but it runs only
  on the shutdown path (motors already being disabled), so it's safe.

## Open implementation details (resolve during build)
1. Exact byte codes for `reset_ack` and `send_shutdown_progress` (pick unused values; verify no
   collision with existing `UART_command_names` / `ble_names`).
2. The precise BLE-notification wire format for the progress message (match how `RtBridge.feed_bytes`
   frames existing typed messages).
3. Confirm the Nano's loop cadence is fast enough that the non-blocking `WAIT_ACK` reliably catches
   the Teensy `reset_ack` within the window.
