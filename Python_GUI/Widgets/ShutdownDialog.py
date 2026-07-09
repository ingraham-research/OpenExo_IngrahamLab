"""Modal shown on End Trial.

Displays the end-trial shutdown handshake steps (driven by RtBridge.shutdownProgressReceived),
then a reboot countdown. The countdown starts on step 5 (REBOOTING), OR on the device
'disconnected' signal, OR on a fallback timeout if the progress notifications get dropped over
BLE -- the reset is reliably delivered by send_end_trial_sequence(), so the exo is rebooting
regardless and the dialog must never hang waiting on an informational tick. Progress ticks are
best-effort (fired faster than the BLE connection interval, so some can be lost); the countdown
is what actually matters. Wiring lives in MainWindow._on_end_trial.

Step codes must match firmware shutdown_progress:: in ExoCode/src/ble_commands.h:
  1 RECEIVED, 2 SENT, 3 ACKED, 4 TIMEOUT, 5 REBOOTING
"""

try:
    from PySide6 import QtCore, QtWidgets
except ImportError as e:
    raise SystemExit("PySide6 is required. Install with: pip install PySide6") from e


# (step_code, label) for the rows that get a checklist entry. TIMEOUT (4) has no row of its
# own; it marks the ACKED (3) row failed.
_STEP_LABELS = [
    (1, "Received end request"),
    (2, "Sent stop + reset to Teensy"),
    (3, "Teensy acknowledged (logs closed)"),
    (5, "Rebooting…"),
]
_STEP_TIMEOUT_MS = 5000       # no progress for this long -> mark the current step stalled
_REBOOT_COUNTDOWN_S = 6


class ShutdownDialog(QtWidgets.QDialog):

    forceDisconnectRequested = QtCore.Signal()
    retryRequested = QtCore.Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Ending Trial")
        self.setModal(True)
        self.setMinimumSize(420, 360)   # roomy enough that the rows + countdown don't overlap
        self._rows = {}          # step_code -> QLabel
        self._rebooting = False

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setSpacing(12)
        self._title = QtWidgets.QLabel("Shutting down…")
        f = self._title.font(); f.setBold(True); self._title.setFont(f)
        layout.addWidget(self._title)

        for code, text in _STEP_LABELS:
            row = QtWidgets.QLabel(f"⏳  {text}")   # hourglass = pending
            self._rows[code] = row
            layout.addWidget(row)

        self._detail = QtWidgets.QLabel("")
        self._detail.setWordWrap(True)
        df = self._detail.font(); df.setPointSize(df.pointSize() + 1); self._detail.setFont(df)
        layout.addWidget(self._detail)

        # Failure buttons (hidden until a stall / timeout)
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

    @staticmethod
    def _mark(label: "QtWidgets.QLabel", symbol: str):
        # Replace whatever status glyph is currently at the front of the label.
        text = label.text()
        for glyph in ("⏳", "✅", "❌"):   # hourglass, check, cross
            text = text.replace(glyph, symbol)
        label.setText(text)

    @QtCore.Slot(int)
    def set_step(self, step_code: int):
        if self._rebooting:
            return
        self._stall_timer.start(_STEP_TIMEOUT_MS)   # progress arrived -> reset stall timer
        self._btns.setVisible(False)

        if step_code in self._rows:
            self._mark(self._rows[step_code], "✅")     # done -> check
        elif step_code == 4:                                # TIMEOUT -> ACKED row failed
            row = self._rows.get(3)
            if row is not None:
                self._mark(row, "❌")                   # failed -> cross
            self._detail.setText("Teensy did not confirm — the reset may not have reached it.")
            self._btns.setVisible(True)

        if step_code == 5:                                  # REBOOTING -> straight to countdown
            self.on_disconnected()

    @QtCore.Slot()
    def on_disconnected(self):
        if self._rebooting:
            return
        self._rebooting = True
        self._stall_timer.stop()
        self._btns.setVisible(False)
        self._title.setText("Exo rebooting…")
        self._countdown_left = _REBOOT_COUNTDOWN_S
        self._detail.setText(
            f"Please wait {self._countdown_left}s, then reconnect from the scan screen.")
        self._countdown_timer.start(1000)

    def _tick_countdown(self):
        self._countdown_left -= 1
        if self._countdown_left <= 0:
            self._countdown_timer.stop()
            self.accept()   # closes the dialog; MainWindow returns to scan
        else:
            self._detail.setText(
                f"Please wait {self._countdown_left}s, then reconnect from the scan screen.")

    def _on_stall(self):
        # Progress notifications can be dropped over BLE (they're fired faster than the connection
        # interval), but the reset itself was reliably delivered (send_end_trial_sequence confirmed
        # 'Z delivered'), so the exo IS rebooting regardless. Don't hang waiting for the step-5
        # notification -- proceed to the reboot countdown. The ticks are informational only.
        self.on_disconnected()

    def _on_retry(self):
        self._btns.setVisible(False)
        self._stall_timer.start(_STEP_TIMEOUT_MS)
        self.retryRequested.emit()
