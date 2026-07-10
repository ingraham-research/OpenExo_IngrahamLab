# End-Trial Shutdown Progress + Teensy-Ack Handshake — Implementation Plan

> **For agentic workers:** Implement task-by-task. Steps use checkbox (`- [ ]`) syntax.
> This spans **on-target firmware** (no host test runner — "verify" = compile in the Arduino IDE +
> the stated on-device check) and the **Python GUI** (verify = `python -m py_compile` + a simulated
> feed, then on-device end-to-end). Per the user's instruction, **do NOT commit**; each task ends at
> a **review checkpoint**.

**Goal:** On End Trial, show a modal progress dialog driven by live Nano→GUI messages (including a
Teensy→Nano ack relayed up), then a reboot countdown — for better UX and to pinpoint where a bad
trial-end fails.

**Architecture:** New typed messages — `reset_ack` (Teensy→Nano UART) and `send_shutdown_progress`
(Nano→GUI BLE, 1 step-code byte). The Nano's reboot logic becomes a non-blocking state machine that
emits progress at each step, waits for the Teensy ack inside the existing 5 s window, then reboots.
The GUI parses the new message into a signal that drives a new `ShutdownDialog`.

**Tech Stack:** Arduino/Teensyduino C++ (Teensy 4.1 + Nano 33 BLE), ArduinoBLE, PySide/Qt GUI.

## Global Constraints

- Firmware compiles for **both** MCUs: Teensy 4.1 (`ARDUINO_TEENSY41`) and Nano 33 BLE
  (`ARDUINO_ARDUINO_NANO33BLE`). Teensy-only code is guarded `#if defined(ARDUINO_TEENSY36) || defined(ARDUINO_TEENSY41)`.
- **Reuse existing byte codes / chars; do not collide.** New: `UART_command_names::reset_ack = 0x1C`
  (next free after `0x1B`), `ble_names::send_shutdown_progress = 'P'` (unused char).
- Step codes (shared, one source of truth): `RECEIVED=1, SENT=2, ACKED=3, TIMEOUT=4, REBOOTING=5`.
- Reboot must still happen: the Nano self-reboots at the end of the state machine regardless of ack.
- Builds on the already-implemented `SdLogger::close_active()` in the reset handler — do not remove it.
- **Do NOT commit.** End each task at a review checkpoint.

## File structure

- **Modify** `ExoCode/src/uart_commands.h` — add `reset_ack` code; Teensy sends it in `get_system_reset`.
- **Modify** `ExoCode/src/ble_commands.h` — add `send_shutdown_progress` char, its `ble::commands`
  length entry, and the shared `shutdown_progress` step-code namespace.
- **Modify** `ExoCode/src/ComsMCU.h` / `ComsMCU.cpp` — reset state machine + progress sender + ack detect.
- **Modify** `Python_GUI/services/RtBridge.py` — parse `send_shutdown_progress` → `shutdownProgressReceived(int)`.
- **Create** `Python_GUI/Widgets/ShutdownDialog.py` — the modal.
- **Modify** `Python_GUI/MainWindow.py` — open the dialog on End Trial, wire signals, drop the 200 ms disconnect.

---

### Task 1: Shared message definitions

**Files:** Modify `ExoCode/src/uart_commands.h`, `ExoCode/src/ble_commands.h`

**Interfaces — Produces:**
- `UART_command_names::reset_ack` (uint8_t `0x1C`)
- `ble_names::send_shutdown_progress` (char `'P'`)
- `namespace shutdown_progress { RECEIVED=1, SENT=2, ACKED=3, TIMEOUT=4, REBOOTING=5 }`

- [ ] **Step 1: Add the UART code.** In `uart_commands.h`, in `namespace UART_command_names`, after
  `update_controller_param_ack = 0x1B;`:

```cpp
    static const uint8_t reset_ack = 0x1C;   // Teensy -> Nano: "got the reset, closing logs"
```

- [ ] **Step 2: Add the BLE char + shared step codes.** In `ble_commands.h`, in the
  `//Sending Commands (Firmware->GUI)` block of `namespace ble_names`, after `param_update_ack = 'a';`:

```cpp
    static const char send_shutdown_progress = 'P';   // Nano -> GUI: end-trial shutdown step
```

  And add the shared step-code namespace just after the `ble_names` namespace closes:

```cpp
namespace shutdown_progress   // step codes carried in send_shutdown_progress data[0]
{
    static const uint8_t RECEIVED  = 1;   // Nano got the end/reset request
    static const uint8_t SENT      = 2;   // Nano forwarded reset to the Teensy
    static const uint8_t ACKED     = 3;   // Teensy acknowledged (reset_ack received)
    static const uint8_t TIMEOUT   = 4;   // Teensy did not ack within the window
    static const uint8_t REBOOTING = 5;   // Nano about to reboot
}
```

- [ ] **Step 3: Register the BLE message length.** In the `ble::commands[]` array (Sending Commands
  section), after `{ble_names::param_update_ack, 5},`:

```cpp
        {ble_names::send_shutdown_progress, 1},   // 1 data byte = step code
```

- [ ] **Step 4: Verify.** Compile for **both** Teensy 4.1 and Nano 33 BLE — expect no errors (only
  new constants added; nothing references them yet).

- [ ] **Step 5: Review checkpoint** — user inspects the diff. (No commit.)

---

### Task 2: Teensy sends `reset_ack` before rebooting

**Files:** Modify `ExoCode/src/uart_commands.h` (the `get_system_reset` handler)

**Interfaces — Consumes:** `UART_command_names::reset_ack` (Task 1).

- [ ] **Step 1: Send the ack first, then reboot.** In `get_system_reset` (currently starts with
  `(void)handler;`), replace the body's start so the handler *uses* `handler` to send the ack before
  the existing safe-state + close + reset:

```cpp
    inline static void get_system_reset(UARTHandler *handler, ExoData *exo_data, UART_msg_t msg)
    {
        (void)msg;

        // Tell the Nano we received the reset FIRST, so the ack is on the wire before we die.
        // (The Nano relays this to the GUI as the ACKED shutdown step.)
        UART_msg_t ack;
        ack.command = UART_command_names::reset_ack;
        ack.joint_id = 0;
        ack.len = 0;
        handler->UART_msg(ack);

        // Put system in a safe state before rebooting
        exo_data->for_each_joint([](JointData* j_data, float* args)
        {
            (void)args;
            j_data->motor.enabled = 0;
            return;
        });
        exo_data->set_status(status_defs::messages::trial_off);
        delay(10);
#if defined(ARDUINO_TEENSY36) || defined(ARDUINO_TEENSY41)
        SdLogger::close_active();   // flush+close the SD log before rebooting (already added)
#endif
        exo_system_reset();
    }
```

- [ ] **Step 2: Verify.** Compile for Teensy 4.1 — no errors. (On-device behavior is exercised in
  Task 3's test, once the Nano relays it.)

- [ ] **Step 3: Review checkpoint** — user inspects the diff. (No commit.)

---

### Task 3: Nano reset handshake state machine

**Files:** Modify `ExoCode/src/ComsMCU.h`, `ExoCode/src/ComsMCU.cpp`

**Interfaces:**
- Consumes: `UART_command_names::reset_ack`, `ble_names::send_shutdown_progress`, `shutdown_progress::*` (Task 1); `UART_command_names::get_system_reset`.
- Produces: `send_shutdown_progress` BLE messages in the sequence RECEIVED→SENT→ACKED/TIMEOUT→REBOOTING.

- [ ] **Step 1: Add state members.** In `ComsMCU.h`, replace the three reset members
  (`_reset_pending`, `_reset_start_ms`, `_reset_delay_ms`) with the state machine:

```cpp
        enum class ResetState : uint8_t { IDLE, PENDING, SENT, WAIT_ACK, REBOOTING };
        ResetState _reset_state = ResetState::IDLE;
        bool _reset_ack_received = false;
        uint32_t _reset_step_ms = 0;
        const uint32_t _reset_ack_timeout_ms = 3000;   // wait up to 3s for the Teensy ack
        const uint32_t _reset_flush_ms = 300;          // let final BLE notification go out
```

  And declare the helper in the private section:

```cpp
        void _send_shutdown_progress(uint8_t step);
```

- [ ] **Step 2: Trigger the state machine.** In `ComsMCU.cpp`, change `_schedule_system_reset()`:

```cpp
void ComsMCU::_schedule_system_reset()
{
    _reset_state = ResetState::PENDING;
    _reset_step_ms = millis();
    _reset_ack_received = false;
}
```

- [ ] **Step 3: Detect the Teensy ack.** In `ComsMCU::update_UART()`, at the point it inspects
  `msg.command` (around the existing `if (msg.command == UART_command_names::update_controller_param_ack)`),
  add:

```cpp
            if (msg.command == UART_command_names::reset_ack)
            {
                _reset_ack_received = true;
            }
```

- [ ] **Step 4: Rewrite `_maybe_system_reset()` as the state machine:**

```cpp
void ComsMCU::_maybe_system_reset()
{
    switch (_reset_state)
    {
    case ResetState::IDLE:
        return;

    case ResetState::PENDING:
        _send_shutdown_progress(shutdown_progress::RECEIVED);
        _reset_state = ResetState::SENT;
        _reset_step_ms = millis();
        break;

    case ResetState::SENT:
    {
        UARTHandler* uart = UARTHandler::get_instance();
        UART_msg_t tx;
        tx.command = UART_command_names::get_system_reset;
        tx.joint_id = 0;
        tx.len = 0;
        uart->UART_msg(tx);
        _send_shutdown_progress(shutdown_progress::SENT);
        _reset_ack_received = false;
        _reset_state = ResetState::WAIT_ACK;
        _reset_step_ms = millis();
        break;
    }

    case ResetState::WAIT_ACK:
        if (_reset_ack_received)
        {
            _send_shutdown_progress(shutdown_progress::ACKED);
            _send_shutdown_progress(shutdown_progress::REBOOTING);
            _reset_state = ResetState::REBOOTING;
            _reset_step_ms = millis();
        }
        else if (millis() - _reset_step_ms >= _reset_ack_timeout_ms)
        {
            _send_shutdown_progress(shutdown_progress::TIMEOUT);
            _send_shutdown_progress(shutdown_progress::REBOOTING);
            _reset_state = ResetState::REBOOTING;
            _reset_step_ms = millis();
        }
        break;

    case ResetState::REBOOTING:
        // Give the final BLE notifications time to be sent before the radio dies.
        if (millis() - _reset_step_ms >= _reset_flush_ms)
        {
            exo_system_reset();
        }
        break;
    }
}
```

- [ ] **Step 5: Implement the progress sender** (model on the existing `send_real_time_data` build in
  `update_gui`, which sets `.command`, `.expecting`, `.data[]` then calls `_exo_ble->send_message`):

```cpp
void ComsMCU::_send_shutdown_progress(uint8_t step)
{
    BleMessage msg = BleMessage();
    msg.command = ble_names::send_shutdown_progress;
    msg.expecting = 1;
    msg.data[0] = (float)step;
    _exo_ble->send_message(msg);
}
```

- [ ] **Step 6: Verify (compile + on-device).**
  - Compile for **both** MCUs — no errors.
  - Flash both. Connect via GUI, start a trial, end it. Watch the GUI's serial/BLE log (or add a
    temporary print in `RtBridge` once Task 4 lands): you should see the step sequence
    `1 → 2 → 3` (ACKED) then reboot. If the Teensy is unplugged/hung, you should instead see
    `1 → 2 → 4` (TIMEOUT) then reboot — proving the diagnostic works.

- [ ] **Step 7: Review checkpoint** — user inspects the diff. (No commit.)

---

### Task 4: GUI — parse `send_shutdown_progress` → signal

**Files:** Modify `Python_GUI/services/RtBridge.py`

**Interfaces — Produces:** `RtBridge.shutdownProgressReceived = QtCore.Signal(int)` carrying the step code.

- [ ] **Step 1: Declare the signal.** In `RtBridge` (with the other `QtCore.Signal` declarations,
  next to `rtDataUpdated`):

```python
    shutdownProgressReceived = QtCore.Signal(int)
```

- [ ] **Step 2: Parse the message.** In `feed_bytes`, in the command-event dispatch — the same place
  that handles `if command == 'a':` by calling `_handle_param_update_ack(event_data, self._data_length)`
  (around line 312) — add a sibling case that extracts the 1-byte step from `event_data` the same way
  `_handle_param_update_ack` reads its payload, and emits the signal:

```python
            if command == 'P':
                # send_shutdown_progress: event_data carries one value = step code.
                # Parse it exactly like the other command payloads in this file (see
                # _handle_param_update_ack) and emit as an int.
                try:
                    step = int(round(float(event_data.strip().split()[0])))
                except Exception:
                    step = 0
                self.shutdownProgressReceived.emit(step)
                return
```

  NOTE for the implementer: confirm the on-the-wire format of a 1-data-byte command against
  `ExoBLE::send_message` framing and the existing `'a'` handler; adjust the `event_data` extraction so
  a firmware `data[0]=3` decodes to `step==3`. Verify with Step 3 before moving on.

- [ ] **Step 3: Verify (py_compile + simulated).**
  - Run: `python -m py_compile Python_GUI/services/RtBridge.py` → no output (success).
  - Simulate: in a scratch script, construct an `RtBridge`, connect `shutdownProgressReceived` to a
    collector, and `feed_bytes()` a byte string matching the firmware's frame for step 3; assert the
    collector received `3`. (Build the frame from the real `ExoBLE::send_message` output.)

- [ ] **Step 4: Review checkpoint** — user inspects the diff. (No commit.)

---

### Task 5: GUI — `ShutdownDialog` widget

**Files:** Create `Python_GUI/Widgets/ShutdownDialog.py`

**Interfaces — Produces:**
- `ShutdownDialog(QtWidgets.QDialog)` with:
  - `set_step(step_code: int)` — advance the checklist.
  - `on_disconnected()` — begin the reboot countdown.
  - signals: `forceDisconnectRequested = QtCore.Signal()`, `retryRequested = QtCore.Signal()`.

- [ ] **Step 1: Create the dialog.**

```python
from PySide6 import QtCore, QtWidgets

# Step codes must match firmware shutdown_progress:: (ble_commands.h)
_STEP_LABELS = [
    (1, "Received end request"),
    (2, "Sent stop + reset to Teensy"),
    (3, "Teensy acknowledged (logs closed)"),
    (5, "Rebooting…"),
]
_STEP_TIMEOUT_MS = 5000     # if no progress for this long, mark the current step stalled
_REBOOT_COUNTDOWN_S = 6


class ShutdownDialog(QtWidgets.QDialog):
    """Modal shown on End Trial. Displays the shutdown handshake steps, then a reboot
    countdown. Driven by RtBridge.shutdownProgressReceived and the device 'disconnected'
    signal (wired in MainWindow)."""

    forceDisconnectRequested = QtCore.Signal()
    retryRequested = QtCore.Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Ending Trial")
        self.setModal(True)
        self._rows = {}          # step_code -> QLabel
        self._current_index = 0
        self._rebooting = False

        layout = QtWidgets.QVBoxLayout(self)
        self._title = QtWidgets.QLabel("Shutting down…")
        f = self._title.font(); f.setBold(True); self._title.setFont(f)
        layout.addWidget(self._title)

        for code, text in _STEP_LABELS:
            row = QtWidgets.QLabel(f"⏳  {text}")
            self._rows[code] = row
            layout.addWidget(row)

        self._detail = QtWidgets.QLabel("")
        self._detail.setWordWrap(True)
        layout.addWidget(self._detail)

        # Failure buttons (hidden until a stall)
        self._btns = QtWidgets.QWidget()
        b_layout = QtWidgets.QHBoxLayout(self._btns)
        self._btn_force = QtWidgets.QPushButton("Force disconnect")
        self._btn_retry = QtWidgets.QPushButton("Retry")
        self._btn_force.clicked.connect(self.forceDisconnectRequested.emit)
        self._btn_retry.clicked.connect(self._on_retry)
        b_layout.addWidget(self._btn_force)
        b_layout.addWidget(self._btn_retry)
        self._btns.setVisible(False)
        layout.addWidget(self._btns)

        # Per-step stall timer
        self._stall_timer = QtCore.QTimer(self)
        self._stall_timer.setSingleShot(True)
        self._stall_timer.timeout.connect(self._on_stall)
        self._stall_timer.start(_STEP_TIMEOUT_MS)

        # Reboot countdown timer
        self._countdown_timer = QtCore.QTimer(self)
        self._countdown_timer.timeout.connect(self._tick_countdown)
        self._countdown_left = _REBOOT_COUNTDOWN_S

    def set_step(self, step_code: int):
        if self._rebooting:
            return
        self._stall_timer.start(_STEP_TIMEOUT_MS)   # progress arrived -> reset stall timer
        self._btns.setVisible(False)
        if step_code in self._rows:
            self._rows[step_code].setText(self._rows[step_code].text().replace("⏳", "✅").replace("❌", "✅"))
        elif step_code == 4:   # TIMEOUT -> mark the "Teensy acknowledged" row failed
            r = self._rows.get(3)
            if r:
                r.setText(r.text().replace("⏳", "❌"))
            self._detail.setText("Teensy did not confirm — the reset may not have reached it.")
            self._btns.setVisible(True)
        if step_code == 5:     # REBOOTING
            self.on_disconnected()   # move straight to countdown

    def on_disconnected(self):
        if self._rebooting:
            return
        self._rebooting = True
        self._stall_timer.stop()
        self._btns.setVisible(False)
        self._title.setText("Exo rebooting…")
        self._countdown_left = _REBOOT_COUNTDOWN_S
        self._detail.setText(f"Please wait {self._countdown_left}s, then reconnect from the scan screen.")
        self._countdown_timer.start(1000)

    def _tick_countdown(self):
        self._countdown_left -= 1
        if self._countdown_left <= 0:
            self._countdown_timer.stop()
            self.accept()   # closes the dialog; MainWindow returns to scan
        else:
            self._detail.setText(f"Please wait {self._countdown_left}s, then reconnect from the scan screen.")

    def _on_stall(self):
        self._detail.setText("No response — a step stalled. Force disconnect or retry.")
        self._btns.setVisible(True)

    def _on_retry(self):
        self._btns.setVisible(False)
        self._stall_timer.start(_STEP_TIMEOUT_MS)
        self.retryRequested.emit()
```

- [ ] **Step 2: Verify.** Run: `python -m py_compile Python_GUI/Widgets/ShutdownDialog.py` → success.
  Optional manual: instantiate it in a tiny script, call `set_step(1)`, `set_step(2)`, `set_step(4)`
  and confirm the rows update and failure buttons appear.

- [ ] **Step 3: Review checkpoint** — user inspects the file. (No commit.)

---

### Task 6: GUI — wire it into `MainWindow._on_end_trial`

**Files:** Modify `Python_GUI/MainWindow.py`

**Interfaces — Consumes:** `ShutdownDialog` (Task 5), `RtBridge.shutdownProgressReceived` (Task 4),
`qt_dev.disconnected` (existing), `qt_dev.write` / `qt_dev.disconnect` (existing).

- [ ] **Step 1: Import the dialog.** With the other page imports near the top of `MainWindow.py`:

```python
from Widgets.ShutdownDialog import ShutdownDialog
```

- [ ] **Step 2: Rework `_on_end_trial`.** Replace the "navigate to scan + 200 ms disconnect" tail
  (lines ~621–637: the scan-button resets, `clear_plots`, `stack.setCurrentWidget(self.scan_page)`,
  and `QTimer.singleShot(200, self.qt_dev.disconnect)`) with opening the dialog and wiring it. Keep
  the `write(b'G')` / `write(b'w')` / `write(b'Z')` block above unchanged:

```python
            # Open the shutdown-progress dialog; DO NOT auto-disconnect — the Nano's reboot
            # ends the link, and the dialog shows the handshake + reboot countdown.
            self._shutdown_dialog = ShutdownDialog(self)
            self.rt_bridge.shutdownProgressReceived.connect(self._shutdown_dialog.set_step)
            self.qt_dev.disconnected.connect(self._shutdown_dialog.on_disconnected)
            self._shutdown_dialog.forceDisconnectRequested.connect(self.qt_dev.disconnect)
            self._shutdown_dialog.retryRequested.connect(self._on_retry_end_trial)
            self._shutdown_dialog.finished.connect(self._on_shutdown_dialog_finished)
            self._shutdown_dialog.show()
```

- [ ] **Step 3: Add the retry + finished handlers** as new methods on `MainWindow`:

```python
    def _on_retry_end_trial(self):
        # Re-send the end sequence (same as End Trial's command block).
        try:
            self.qt_dev.write(b'G')
            self.qt_dev.write(b'w')
            self.qt_dev.write(b'Z')
        except Exception as e:
            self.logger.error(f"Retry end-trial failed: {e}")

    def _on_shutdown_dialog_finished(self, _result):
        # Countdown done or dialog closed -> return to scan and clean up.
        try:
            self.rt_bridge.shutdownProgressReceived.disconnect(self._shutdown_dialog.set_step)
            self.qt_dev.disconnected.disconnect(self._shutdown_dialog.on_disconnected)
        except Exception:
            pass
        self.stack.setCurrentWidget(self.scan_page)
        try:
            self.trial_page.clear_plots()
        except Exception:
            pass
```

- [ ] **Step 4: Keep the CSV close.** Ensure the existing "Stop CSV if running" block (that followed
  the removed disconnect) still runs inside `_on_end_trial` — leave it where it is; only the
  navigate/disconnect lines are replaced.

- [ ] **Step 5: Verify.**
  - Run: `python -m py_compile Python_GUI/MainWindow.py Python_GUI/Widgets/ShutdownDialog.py` → success.
  - On-device end-to-end: connect, run a trial, End Trial → the dialog shows `Received → Sent →
    Teensy acknowledged`, then a 6 s countdown, then returns to scan; reconnect works. Then test the
    failure path (e.g. Teensy powered off): dialog shows the `Teensy acknowledged` row red +
    Force disconnect / Retry.

- [ ] **Step 6: Review checkpoint** — user inspects the full feature diff. (No commit.)

---

## Self-review

- **Spec coverage:** shared defs (T1), Teensy ack (T2), Nano state machine + progress + ack detect
  (T3), GUI parse+signal (T4), ShutdownDialog with checklist/countdown/failure buttons (T5), MainWindow
  wiring + no-200ms-disconnect + return-to-scan (T6). Two-phase (live steps then countdown) and the
  TIMEOUT diagnostic are covered by T3+T5. ✔
- **Placeholders:** the two protocol-framing spots (T3 send, T4 parse) carry explicit "model on
  `send_real_time_data` / `_handle_param_update_ack`" pointers plus a verify step, because the exact
  BLE wire framing must be matched against the real code — not a vague TODO, a concrete model + test.
- **Type consistency:** step codes `1/2/3/4/5` identical across firmware (`shutdown_progress::`),
  GUI parse, and `ShutdownDialog._STEP_LABELS`; `shutdownProgressReceived(int)` used consistently;
  `set_step` / `on_disconnected` names match between T5 and T6.
- **Note:** `send_error_count` and `motors_off` both use `'w'`, and `send_cal_done`/`cal_fsr_finished`
  both use `'n'` in the existing `ble_names` — pre-existing collisions, not touched; `'P'` is clear.
