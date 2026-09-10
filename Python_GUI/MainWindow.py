import sys
import csv
import time
import os
import logging
import traceback
from datetime import datetime

try:
    from PySide6 import QtCore, QtWidgets, QtGui
except ImportError as e:
    raise SystemExit("PySide6 is required. Install with: pip install PySide6") from e

from pages import (
    ScanWindowQt,
    ActiveTrialPage,
    ActiveTrialSettingsPage,
    ActiveTrialBasicSettingsPage,
    BioFeedbackPage,
)
from services import QtExoDeviceManager, RtBridge
from utils import SettingsManager, RemoteConfig
from remote.service import RemoteControlService
from Widgets.ShutdownDialog import ShutdownDialog


class MainWindow(QtWidgets.QMainWindow):
    _PARAM_UPDATE_REASONS = {
        1: "invalid message",
        2: "side or joint mismatch",
        3: "controller mismatch",
        4: "invalid parameter index",
        5: "value out of bounds",
        6: "value must be an integer",
    }

    def __init__(self):
        super().__init__()
        self.setWindowTitle("OpenExo - Qt")
        # Compact default window size (resizable)
        self.setMinimumSize(776, 400)
        self.resize(776, 400)
        
        # Setup logger for MainWindow
        self.logger = logging.getLogger("OpenExo.MainWindow")
        self.logger.info("Initializing MainWindow...")

        self.stack = QtWidgets.QStackedWidget()
        self.setCentralWidget(self.stack)

        # Pages
        self.scan_page = ScanWindowQt()
        self.trial_page = ActiveTrialPage()
        self.settings_page = ActiveTrialSettingsPage()
        self.basic_settings_page = ActiveTrialBasicSettingsPage()
        self.bio_feedback_page = BioFeedbackPage()

        self.stack.addWidget(self.scan_page)
        self.stack.addWidget(self.trial_page)
        self.stack.addWidget(self.settings_page)
        self.stack.addWidget(self.basic_settings_page)
        self.stack.addWidget(self.bio_feedback_page)
#     self.stack.setCurrentWidget(self.trial_page)

        # Simple top bar navigation
        toolbar = self.addToolBar("Nav")
        act_scan = QtGui.QAction("Scan", self)
        act_trial = QtGui.QAction("Active Trial", self)
        act_disc = QtGui.QAction("Disconnect", self)
        toolbar.addAction(act_scan)
        toolbar.addAction(act_trial)
        toolbar.addAction(act_disc)
        act_scan.triggered.connect(self._navigate_to_scan)
        act_trial.triggered.connect(lambda: self.stack.setCurrentWidget(self.trial_page))
        act_disc.triggered.connect(self._on_disconnect)
        # Hide manual navigation to prevent tab-like selection
        toolbar.setVisible(False)

        # When scan page connects, enable trial start
        self.scan_page.btn_start_trial.clicked.connect(self._go_trial)
        self.scan_page.connectRequested.connect(self._on_connect_requested)

        # Services
        self.qt_dev = QtExoDeviceManager(self)
        # Bind scan page scanner to use the Qt device manager for scanning
        try:
            self.scan_page.bind_device_manager(self.qt_dev)
        except Exception as e:
            self.logger.error(f"Failed to bind device manager to scan page: {e}")
            self.logger.debug(traceback.format_exc())
        self.rt_bridge = RtBridge(self)
        # Wire bytes to parser and route RT data to plots
        self.qt_dev.dataReceived.connect(self.rt_bridge.feed_bytes)
        self.rt_bridge.rtDataUpdated.connect(self._on_rt_update)
        self.rt_bridge.handshakeReceived.connect(self._on_handshake)
        self.rt_bridge.parameterNamesReceived.connect(self._on_param_names)
        self.rt_bridge.controllersReceived.connect(self._on_controllers)
        # Receive flattened 2D matrix of controllers and parameters
        self.rt_bridge.controllerMatrixReceived.connect(self._on_controller_matrix)
        self.rt_bridge.controllerMatrixIncomplete.connect(self._on_controller_matrix_incomplete)
        self.rt_bridge.controllerValuesReceived.connect(self._on_controller_values)
        self.rt_bridge.paramUpdateAckReceived.connect(self._on_param_update_ack)

        # CSV logging state
        self._csv_file = None
        self._csv_writer = None
        self._csv_header_written = False
        # Channel indices written to the CSV, fixed when the header is written so header and rows
        # can never disagree. See _csv_channel_indices().
        self._csv_indices = []
        self._param_names = []
        self._t0 = None
        self._csv_path_last = None
        self._mark_counter = 0  # Trial mark counter
        self._csv_preamble = ""  # Preamble for CSV filename
        # Store controller -> params 2D matrix
        self._controller_matrix = []
        # Warning text when the handshake delivered a short controller list ("" = looks fine).
        self._controller_matrix_warning = ""
        # Store controller values by (joint_id, controller_id)
        self._controller_values = {}
        self._pending_param_updates = {}
        self._pending_param_update_seq = 0
        # Device control wiring from ActiveTrialPage
        self.trial_page.deviceStartRequested.connect(self._on_device_start)
        self.trial_page.deviceStopRequested.connect(self._on_device_stop_motors)
        self.trial_page.csvPreambleChanged.connect(self._on_csv_preamble_changed)
        self.trial_page.recalibrateFSRRequested.connect(self._on_recal_fsr)
        self.trial_page.sendPresetFSRRequested.connect(self._on_send_preset_fsr)
        self.trial_page.recalibrateTorqueRequested.connect(self._on_recal_torque)
        self.trial_page.markTrialRequested.connect(self._on_mark)
        self.trial_page.endTrialRequested.connect(self._on_end_trial)
        self.trial_page.saveCsvRequested.connect(self._on_save_csv)
        self.trial_page.updateControllerRequested.connect(self._on_update_controller)
        self.trial_page.bioFeedbackRequested.connect(self._on_bio_feedback)
        self.trial_page.machineLearningRequested.connect(self._on_machine_learning)
        # Update Scan page status from device manager
        self.qt_dev.log.connect(self._on_dev_log)
        self.qt_dev.error.connect(self._on_dev_error)
        self.qt_dev.connected.connect(self._on_dev_connected)
        self.qt_dev.disconnected.connect(self._on_dev_disconnected)
        self.qt_dev.resetReasonReceived.connect(self._on_reset_reason)

        # Settings page wiring
        self.settings_page.applyRequested.connect(self._on_apply_settings)
        self.settings_page.cancelRequested.connect(lambda: self.stack.setCurrentWidget(self.trial_page))
        self.basic_settings_page.applyRequested.connect(self._on_apply_settings)
        self.basic_settings_page.cancelRequested.connect(lambda: self.stack.setCurrentWidget(self.trial_page))
        self.bio_feedback_page.backRequested.connect(self._on_bio_feedback_back)
        self.bio_feedback_page.deviceStartRequested.connect(self._on_device_start)
        self.bio_feedback_page.deviceStopRequested.connect(self._on_device_stop_motors)
        self.bio_feedback_page.recalibrateFSRRequested.connect(self._on_recal_fsr)
        self.bio_feedback_page.markTrialRequested.connect(self._on_mark)
        
        log_path = self.qt_dev.get_log_file_path()
        if log_path and log_path != "Log file not available":
            self.logger.info("Device manager log file: %s", log_path)

        # ----- UDP remote control (localhost only; default on) -----
        self.remote = None
        if RemoteConfig.ENABLED:
            try:
                self.remote = RemoteControlService(parent=self)
            except Exception as e:
                self.logger.error(f"Failed to construct remote control service: {e}")
                self.logger.debug(traceback.format_exc())
                self.remote = None

        if self.remote is not None and self.remote.is_bound():
            host, port = self.remote.address()
            # Inbound: remote commands drive the navigation-free apply path.
            self.remote.setParamRequested.connect(self._apply_param_update)
            # Outbound: fan existing BLE-derived signals to subscribers.
            self.rt_bridge.rtDataUpdated.connect(self.remote.publish_rt)
            self.rt_bridge.paramUpdateAckReceived.connect(self.remote.publish_ack)
            self.rt_bridge.controllerMatrixReceived.connect(self.remote.set_controller_matrix)
            self.rt_bridge.controllerValuesReceived.connect(self.remote.publish_values)
            self.rt_bridge.parameterNamesReceived.connect(self.remote.set_param_names)
            self.rt_bridge.shutdownProgressReceived.connect(
                lambda step: self.remote.publish_status("shutdown_progress", step=int(step)))
            self.qt_dev.connected.connect(
                lambda name, addr: self.remote.publish_status("connected", name=name, address=addr))
            self.qt_dev.disconnected.connect(
                lambda: self.remote.publish_status("disconnected"))
            self.qt_dev.deviceErrorReceived.connect(
                lambda msg: self.remote.publish_status("device_error", message=msg))

            banner = (
                "\n" + "=" * 60 + "\n"
                f" REMOTE CONTROL ACTIVE - listening on udp://{host}:{port}\n"
                " External scripts can change controller parameters.\n"
                " Localhost only. Disable: utils/config.py RemoteConfig.ENABLED\n"
                + "=" * 60
            )
            print(banner)
            self.logger.info("Remote control active on udp://%s:%s", host, port)
        elif RemoteConfig.ENABLED:
            print("\n" + "=" * 60 + "\n"
                  " REMOTE CONTROL FAILED TO BIND - continuing without it.\n"
                  " Another process may hold the port. See the log for details.\n"
                  + "=" * 60)
            self.logger.error("Remote control enabled but failed to bind; GUI continues without it.")

    def resizeEvent(self, event):
        super().resizeEvent(event)

    def _go_trial(self):
        self.stack.setCurrentWidget(self.trial_page)
        # Stop simulation so live data drives plots if available
        self.trial_page.stop_sim()
        # Clear old plot data
        try:
            self.trial_page.clear_plots()
        except Exception as e:
            self.logger.error(f"Failed to clear plots: {e}")
            self.logger.debug(traceback.format_exc())
        # Reset data rate monitoring
        try:
            self.rt_bridge.reset_monitoring()
        except Exception as e:
            self.logger.error(f"Failed to reset monitoring: {e}")
            self.logger.debug(traceback.format_exc())
        # Ensure CSV logging is started automatically with timestamped filename
        try:
            if self._csv_file is None:
                self._start_csv_auto()
        except Exception as e:
            self.logger.error(f"Failed to start CSV logging: {e}")
            self.logger.debug(traceback.format_exc())
        # Begin trial sequence (E -> L -> R + thresholds) to ensure FSRs stream
        try:
            self.qt_dev.beginTrial()
        except Exception as e:
            self.logger.error(f"Failed to begin trial: {e}")
            self.logger.debug(traceback.format_exc())

    @QtCore.Slot(str)
    def _on_connect_requested(self, mac: str):
        # Start BLE connection via Qt device manager
        self.qt_dev.set_mac(mac)
        self.qt_dev.connect()

    @QtCore.Slot(list)
    def _on_rt_update(self, values):
        # Map first four channels to two plots
        try:
            page = self.trial_page
            page.apply_values(values)
            try:
                if self.stack.currentWidget() == self.bio_feedback_page:
                    self.bio_feedback_page.apply_values(values)
            except Exception as e:
                self.logger.error(f"Failed to apply values to bio feedback page: {e}")
                self.logger.debug(traceback.format_exc())
            # CSV logging
            if self._csv_writer is not None:
                if not self._csv_header_written:
                    header = ["epoch", "mark"]
                    # Log EVERY advertised channel except the battery, which has its own readout.
                    #
                    # This used to be a hardcoded `[:10]`, which was only ever meant to drop the
                    # battery (then at index 10). When the payload grew from 11 to 13 channels
                    # that cap silently discarded the two new ones -- Commanded Torque (L/R), the
                    # post-PID post-clamp value that actually drives the motor. Selecting by name
                    # means the CSV follows whatever the firmware advertises, and survived the
                    # channel renumbering that moved battery to index 12.
                    self._csv_indices = self._csv_channel_indices(values)
                    if self._param_names:
                        header.extend(self._param_names[i] for i in self._csv_indices)
                    else:
                        header.extend(f"data{i}" for i in self._csv_indices)
                    try:
                        self._csv_writer.writerow(header)
                        self._csv_header_written = True
                        if self._t0 is None:
                            self._t0 = time.time()
                    except Exception as e:
                        self.logger.error(f"Failed to write CSV header: {e}")
                        self.logger.debug(traceback.format_exc())
                # Write row using the same channel selection as the header, so they can never
                # drift apart (a row shorter/longer than the header silently misaligns columns).
                epoch_time = time.time()
                row = [f"{epoch_time:.6f}", str(self._mark_counter)] + [
                    f"{values[i]:.6f}" if i < len(values) else "" for i in self._csv_indices]
                try:
                    self._csv_writer.writerow(row)
                except Exception as e:
                    self.logger.error(f"Failed to write CSV row: {e}")
                    self.logger.debug(traceback.format_exc())
            
            # Battery and status are located BY NAME, not by a hardcoded index. The channel
            # layout has been renumbered once already (to put Commanded Torque L/R adjacent) and
            # a stale literal here silently plots the wrong signal rather than failing.
            try:
                battery_voltage = self._channel_value(values, "Battery Level (Volts)", 12)
                if battery_voltage is not None:
                    self.trial_page.update_battery_level(battery_voltage)
            except Exception as e:
                self.logger.error(f"Failed to update battery level: {e}")
                self.logger.debug(traceback.format_exc())

            # Exo status readout from the hijacked status channel (firmware status word).
            try:
                status_value = self._channel_value(values, "Status", 10)
                if status_value is not None:
                    self.trial_page.update_exo_status(status_value)
            except Exception as e:
                self.logger.error(f"Failed to update exo status: {e}")
                self.logger.debug(traceback.format_exc())
        except Exception as e:
            self.logger.error(f"Failed to process RT update: {e}")
            self.logger.debug(traceback.format_exc())

    @QtCore.Slot(str)
    def _on_handshake(self, payload: str):
        try:
            self.logger.info("Handshake payload received")
        except Exception as e:
            self.logger.error(f"Failed to log handshake: {e}")
            self.logger.debug(traceback.format_exc())
        try:
            self.scan_page.status.setText("Handshake received; controller parameters incoming…")
        except Exception as e:
            self.logger.error(f"Failed to update status text for handshake: {e}")
            self.logger.debug(traceback.format_exc())

    # Channels excluded from the CSV. Battery has its own on-screen readout and changes far too
    # slowly to be worth a column at ~100 Hz. Matched by NAME so adding firmware channels never
    # silently drops one (see PlottingTitles.h / uart_commands.h get_real_time_data).
    _CSV_EXCLUDED_CHANNELS = ("Battery Level (Volts)",)

    def _channel_value(self, values, name: str, fallback_index: int):
        """Look up a real-time channel by its advertised name.

        Falls back to `fallback_index` only when the handshake delivered no names (which also
        happens when the controller list arrives truncated). Returns None if unavailable.
        """
        try:
            idx = self._param_names.index(name) if self._param_names else fallback_index
        except ValueError:
            idx = fallback_index
        return values[idx] if 0 <= idx < len(values) else None

    def _csv_channel_indices(self, values) -> list:
        """Which real-time channel indices get written to the CSV.

        Resolved once, when the header is written, and reused for every row so header and data
        can never misalign.
        """
        if self._param_names:
            return [i for i, name in enumerate(self._param_names)
                    if name not in self._CSV_EXCLUDED_CHANNELS]
        # No names from the handshake. RtBridge pads every packet to 16, so len(values) cannot
        # tell us how many channels the firmware actually sent; fall back to the bilateral_ankle
        # layout (13 channels, battery LAST at index 12) rather than logging padding zeros.
        return [i for i in range(min(13, len(values))) if i != 12]

    @QtCore.Slot(list)
    def _on_param_names(self, names):
        # After receiving parameter names list (ended by END), send ACK so firmware continues (controllers or data)
        try:
            self.logger.info(f"Received {len(names)} parameter names")
            self.scan_page.status.setText(f"Received {len(names)} param names; sending ACK…")
        except Exception as e:
            self.logger.error(f"Failed to update status for param names: {e}")
            self.logger.debug(traceback.format_exc())
        # Store for CSV header
        try:
            self._param_names = list(names)
        except Exception as e:
            self.logger.error(f"Failed to store param names: {e}")
            self.logger.debug(traceback.format_exc())
            self._param_names = []
        try:
            self.trial_page.set_channel_labels(self._param_names)
        except Exception as e:
            self.logger.error(f"Failed to set channel labels: {e}")
            self.logger.debug(traceback.format_exc())
        try:
            self.qt_dev.write(b'$')
        except Exception as e:
            self.logger.error(f"Failed to send ACK for param names: {e}")
            self.logger.debug(traceback.format_exc())

    @QtCore.Slot(list, list)
    def _on_controllers(self, controllers, controller_params):
        # After receiving controllers/params (!… !END), send ACK and optionally start streaming
        try:
            self.logger.info(f"Received {len(controllers)} controllers")
            self.scan_page.status.setText(f"Received {len(controllers)} controllers; sending ACK…")
        except Exception as e:
            self.logger.error(f"Failed to update status for controllers: {e}")
            self.logger.debug(traceback.format_exc())
        try:
            self.qt_dev.write(b'$')
        except Exception as e:
            self.logger.error(f"Failed to send ACK for controllers: {e}")
            self.logger.debug(traceback.format_exc())

    @QtCore.Slot(str)
    def _on_controller_matrix_incomplete(self, message: str):
        """Surface a short/garbled controller list instead of letting it fail silently.

        The list is fetched once per BLE connection, so a controller missing here stays
        missing for the whole session - reconnecting is the fix.
        """
        self._controller_matrix_warning = message or ""
        if not self._controller_matrix_warning:
            return
        self.logger.warning(f"Controller list incomplete: {self._controller_matrix_warning}")
        try:
            self.settings_page.set_param_update_status(self._controller_matrix_warning, warning=True)
        except Exception as e:
            self.logger.error(f"Failed to show controller list warning: {e}")
            self.logger.debug(traceback.format_exc())

    @QtCore.Slot(list)
    def _on_controller_matrix(self, matrix):
        # Save the 2D [ [controller, p1, p2], ... ] structure in memory
        try:
            self._controller_matrix = list(matrix)
            self.logger.info(f"Received controller matrix with {len(self._controller_matrix)} entries")
        except Exception as e:
            self.logger.error(f"Failed to store controller matrix: {e}")
            self.logger.debug(traceback.format_exc())
            self._controller_matrix = []
        has_matrix = bool(self._controller_matrix)
        try:
            self.settings_page.set_controller_matrix(self._controller_matrix if has_matrix else [])
        except Exception as e:
            self.logger.error(f"Failed to set controller matrix in settings page: {e}")
            self.logger.debug(traceback.format_exc())
        # Update Controller button is always enabled
        # If no matrix, it will show the legacy basic settings page
        try:
            self.trial_page.set_update_controller_enabled(True)
        except Exception as e:
            self.logger.error(f"Failed to enable update controller button: {e}")
            self.logger.debug(traceback.format_exc())

    def _normalized_controller_values(self, overlay: dict) -> dict:
        """
        Build a full value DB aligned to _controller_matrix: one entry per (joint_id, controller_id)
        with list length matching the parameter name count (row[4:]). Missing overlay entries pad with "0";
        excess overlay values are truncated.
        """
        out: dict = {}
        overlay = overlay or {}
        try:
            for row in self._controller_matrix:
                if len(row) < 4:
                    continue
                jid, cid = str(row[1]), str(row[3])
                key = (jid, cid)
                n_params = max(0, len(row) - 4)
                raw = overlay.get(key, [])
                vals = [str(v) for v in raw]
                if len(vals) < n_params:
                    vals.extend(["0"] * (n_params - len(vals)))
                elif len(vals) > n_params:
                    vals = vals[:n_params]
                out[key] = vals
        except Exception as e:
            self.logger.error(f"Failed to normalize controller values: {e}")
            self.logger.debug(traceback.format_exc())
        return out

    @QtCore.Slot(list)
    def _on_controller_values(self, rows):
        try:
            overlay = {}
            for row in rows:
                if len(row) >= 2:
                    key = (str(row[0]), str(row[1]))
                    overlay[key] = [str(v) for v in row[2:]]
            if self._controller_matrix:
                self._controller_values = self._normalized_controller_values(overlay)
            else:
                self._controller_values = {k: list(v) for k, v in overlay.items()}
            self.settings_page.set_controller_values(self._controller_values)
            self.logger.info(
                f"Controller value DB from device: {len(overlay)} raw keys, "
                f"{len(self._controller_values)} entries after matrix alignment"
            )
        except Exception as e:
            self.logger.error(f"Failed to store controller value DB: {e}")
            self.logger.debug(traceback.format_exc())

    def _queue_pending_param_updates(self, updates):
        for joint_id, controller_id, param_index, value in updates:
            key = (str(joint_id), str(controller_id), int(param_index))
            self._pending_param_update_seq += 1
            record = {
                "token": self._pending_param_update_seq,
                "value": value,
            }
            self._pending_param_updates.setdefault(key, []).append(record)
            QtCore.QTimer.singleShot(
                5000,
                lambda key=key, token=record["token"]: self._on_param_update_timeout(key, token),
            )

    def _pop_pending_param_update(self, key):
        records = self._pending_param_updates.get(key, [])
        if not records:
            return None
        record = records.pop(0)
        if records:
            self._pending_param_updates[key] = records
        else:
            self._pending_param_updates.pop(key, None)
        return record

    def _remove_pending_param_update(self, key, token):
        records = self._pending_param_updates.get(key, [])
        for idx, record in enumerate(records):
            if record.get("token") == token:
                removed = records.pop(idx)
                if records:
                    self._pending_param_updates[key] = records
                else:
                    self._pending_param_updates.pop(key, None)
                return removed
        return None

    def _update_controller_value_cache(self, joint_id, controller_id, param_index, value):
        key = (str(joint_id), str(controller_id))
        values = list(self._controller_values.get(key, []))
        while len(values) <= int(param_index):
            values.append("0")
        values[int(param_index)] = str(value)
        self._controller_values[key] = values
        self.settings_page.set_controller_values(self._controller_values)

    def _show_param_update_status(self, message: str, warning: bool = True):
        try:
            self.trial_page.set_param_update_status(message, warning=warning)
        except Exception as e:
            self.logger.error(f"Failed to update trial parameter status: {e}")
            self.logger.debug(traceback.format_exc())
        for page_name, page in (
            ("settings", self.settings_page),
            ("basic settings", self.basic_settings_page),
        ):
            try:
                page.set_param_update_status(message, warning=warning)
            except Exception as e:
                self.logger.error(f"Failed to update {page_name} parameter status: {e}")
                self.logger.debug(traceback.format_exc())
        if warning:
            try:
                self.scan_page.status.setText(message)
            except Exception as e:
                self.logger.error(f"Failed to update scan parameter status: {e}")
                self.logger.debug(traceback.format_exc())

    def _param_rejection_text(self, reason: int) -> str:
        return self._PARAM_UPDATE_REASONS.get(int(reason), f"reason {reason}")

    @QtCore.Slot(dict)
    def _on_param_update_ack(self, ack: dict):
        try:
            joint_id = str(ack.get("joint_id"))
            controller_id = str(ack.get("controller_id"))
            param_index = int(ack.get("param_index"))
            accepted = bool(ack.get("accepted"))
            reason = int(ack.get("reason", 0))
            key = (joint_id, controller_id, param_index)
            record = self._pop_pending_param_update(key)

            if accepted:
                if record is not None:
                    self._update_controller_value_cache(
                        joint_id,
                        controller_id,
                        param_index,
                        record["value"],
                    )
                self.logger.info(
                    "Parameter update accepted: joint=%s controller=%s index=%s",
                    joint_id,
                    controller_id,
                    param_index,
                )
                return

            reason_text = self._param_rejection_text(reason)
            self.logger.warning(
                "Parameter update rejected: joint=%s controller=%s index=%s reason=%s",
                joint_id,
                controller_id,
                param_index,
                reason_text,
            )
            self._show_param_update_status(f"Controller update rejected: {reason_text}", warning=True)
        except Exception as e:
            self.logger.error(f"Failed to handle parameter update ack: {e}")
            self.logger.debug(traceback.format_exc())

    def _on_param_update_timeout(self, key, token):
        record = self._remove_pending_param_update(key, token)
        if record is None:
            return
        joint_id, controller_id, param_index = key
        self.logger.warning(
            "Parameter update timed out waiting for ack: joint=%s controller=%s index=%s",
            joint_id,
            controller_id,
            param_index,
        )
        self._show_param_update_status("Controller update failed: no device acknowledgement", warning=True)

    @QtCore.Slot()
    def _on_device_start(self):
        # Resume motors (play functionality) - just turn motors back on
        try:
            self.logger.info("Turning motors ON")
            self.qt_dev.motorOn()
        except Exception as e:
            self.logger.error(f"Failed to turn motors on: {e}")
            self.logger.debug(traceback.format_exc())

    @QtCore.Slot()
    def _on_device_stop_motors(self):
        # Turn off motors (pause functionality)
        try:
            self.logger.info("Turning motors OFF")
            self.qt_dev.motorOff()
        except Exception as e:
            self.logger.error(f"Failed to turn motors off: {e}")
            self.logger.debug(traceback.format_exc())

    @QtCore.Slot(str)
    def _on_csv_preamble_changed(self, preamble: str):
        """Update CSV filename preamble."""
        self._csv_preamble = preamble
        self.logger.info(f"CSV preamble set to: {preamble}")
         # If we're currently logging, roll over immediately (no popup)
        if self._csv_file is not None:
            try:
                self._csv_file.flush()
                self._csv_file.close()
            except Exception as e:
                self.logger.error(f"Failed to close CSV file during preamble change: {e}")
                self.logger.debug(traceback.format_exc())

            # reset state (same reset you already do in _on_save_csv)
            self._csv_file = None
            self._csv_writer = None
            self._csv_header_written = False
            self._t0 = None
            self._csv_path_last = None

            # start a new CSV using the new prefix
            self._start_csv_auto()

            # optional: show a non-blocking confirmation somewhere
            try:
                self.trial_page.set_status_text(f"CSV prefix set. New file started: {self._csv_path_last}")
            except Exception as e:
                self.logger.error(f"Failed to update status text after preamble change: {e}")
                self.logger.debug(traceback.format_exc())

    @QtCore.Slot()
    def _on_recal_fsr(self):
        try:
            self.logger.info("Calibrating FSRs")
            self.qt_dev.calibrateFSRs()
        except Exception as e:
            self.logger.error(f"Failed to calibrate FSRs: {e}")
            self.logger.debug(traceback.format_exc())

    @QtCore.Slot()
    def _on_send_preset_fsr(self):
        try:
            self.logger.info("Sending preset FSR values")
            self.qt_dev.sendPresetFsrValues()
        except Exception as e:
            self.logger.error(f"Failed to send preset FSR values: {e}")
            self.logger.debug(traceback.format_exc())

    @QtCore.Slot()
    def _on_recal_torque(self):
        try:
            self.logger.info("Calibrating torque")
            self.qt_dev.calibrateTorque()
        except Exception as e:
            self.logger.error(f"Failed to calibrate torque: {e}")
            self.logger.debug(traceback.format_exc())

    @QtCore.Slot()
    def _on_mark(self):
        # Increment trial mark counter
        self._mark_counter += 1
        self.logger.info(f"Trial marked: {self._mark_counter}")
        try:
            self.scan_page.status.setText(f"Trial marked: {self._mark_counter}")
        except Exception as e:
            self.logger.error(f"Failed to update scan page status for mark: {e}")
            self.logger.debug(traceback.format_exc())
        try:
            self.trial_page.update_mark_count(self._mark_counter)
        except Exception as e:
            self.logger.error(f"Failed to update trial page mark count: {e}")
            self.logger.debug(traceback.format_exc())
        try:
            self.bio_feedback_page.update_mark_count(self._mark_counter)
        except Exception as e:
            self.logger.error(f"Failed to update bio feedback page mark count: {e}")
            self.logger.debug(traceback.format_exc())

    @QtCore.Slot()
    def _on_end_trial(self):
        try:
            self._destroy_controller_db()
            self.logger.info("Ending trial...")
            # Print trial summary diagnostics
            try:
                self.rt_bridge.print_trial_summary()
            except Exception as e:
                self.logger.error(f"Failed to print trial summary: {e}")
                self.logger.debug(traceback.format_exc())
            
            # Send stop trial, motor off, and reset RELIABLY (write-with-response, in order).
            # Fire-and-forget writes were being dropped under worn-trial load, losing the 'Z'
            # reset so the exo never rebooted. send_end_trial_sequence() ACKs + retries each.
            try:
                self.qt_dev.send_end_trial_sequence()  # G (stop), w (motor off), Z (reset)
                self.logger.info("Sent stop trial, motor off, and reset commands (reliable)")
            except Exception as e:
                self.logger.error(f"CRITICAL: Failed to send stop/motor off commands: {e}")
                self.logger.debug(traceback.format_exc())
            
            # Reset scan page buttons immediately
            self.scan_page.btn_start_trial.setEnabled(False)
            self.scan_page.btn_calibrate_torque.setEnabled(False)
            self.scan_page.btn_save_connect.setEnabled(False)
            
            # Clear trial page plots
            try:
                self.trial_page.clear_plots()
            except Exception as e:
                self.logger.error(f"Failed to clear trial page plots: {e}")
                self.logger.debug(traceback.format_exc())
            
            # Open the shutdown-progress dialog; DO NOT auto-disconnect or navigate away yet.
            # The Nano's reboot ends the BLE link, and the dialog shows the handshake + a reboot
            # countdown, then returns to scan when it closes (_on_shutdown_dialog_finished).
            self._shutting_down = True   # tells _on_dev_disconnected the coming drop is expected
            self._shutdown_dialog = ShutdownDialog(self)
            self.rt_bridge.shutdownProgressReceived.connect(self._shutdown_dialog.set_step)
            self.qt_dev.disconnected.connect(self._shutdown_dialog.on_disconnected)
            self._shutdown_dialog.forceDisconnectRequested.connect(self.qt_dev.disconnect)
            self._shutdown_dialog.retryRequested.connect(self._on_retry_end_trial)
            self._shutdown_dialog.finished.connect(self._on_shutdown_dialog_finished)
            self._shutdown_dialog.show()

            # Stop CSV if running
            if self._csv_file is not None:
                try:
                    self._csv_file.flush(); self._csv_file.close()
                    self.logger.info(f"CSV file closed: {self._csv_path_last}")
                except Exception as e:
                    self.logger.error(f"Failed to close CSV file: {e}")
                    self.logger.debug(traceback.format_exc())
                self._csv_file = None
                self._csv_writer = None
                self._csv_header_written = False
                self._t0 = None
                self._mark_counter = 0  # Reset mark counter
                try:
                    if self._csv_path_last:
                        self.scan_page.status.setText(f"Trial ended. CSV saved: {self._csv_path_last}")
                except Exception as e:
                    self.logger.error(f"Failed to update status text after trial end: {e}")
                    self.logger.debug(traceback.format_exc())
                try:
                    self.trial_page.update_mark_count(0)
                except Exception as e:
                    self.logger.error(f"Failed to reset mark count: {e}")
                    self.logger.debug(traceback.format_exc())
        except Exception as e:
            self.logger.error(f"Failed to end trial: {e}")
            self.logger.debug(traceback.format_exc())

    @QtCore.Slot()
    def _on_retry_end_trial(self):
        # Re-send the end sequence (same commands as End Trial) from the shutdown dialog's Retry.
        try:
            self.qt_dev.send_end_trial_sequence()   # reliable G/w/Z
            self.logger.info("Re-sent stop/motor-off/reset (retry)")
        except Exception as e:
            self.logger.error(f"Retry end-trial failed: {e}")
            self.logger.debug(traceback.format_exc())

    @QtCore.Slot(int)
    def _on_shutdown_dialog_finished(self, _result):
        # Countdown done: the exo has rebooted. Unwire the dialog, then proactively tear down our
        # (now-dead) BLE link. The PC's BLE supervision timeout takes ~10s to notice the reboot,
        # which is longer than the countdown -> if we just wait, that late drop is flagged as an
        # "unexpected disconnect" (popup) and the scan buttons stay greyed. Disconnecting here is
        # intentional (no popup) and preempts that wait, landing on a clean, ready scan screen.
        try:
            self.rt_bridge.shutdownProgressReceived.disconnect(self._shutdown_dialog.set_step)
        except Exception:
            pass
        try:
            self.qt_dev.disconnected.disconnect(self._shutdown_dialog.on_disconnected)
        except Exception:
            pass
        try:
            self.qt_dev.disconnect()   # intentional cleanup of the dead link -> no unexpected popup
        except Exception:
            pass
        self._shutting_down = False
        self.stack.setCurrentWidget(self.scan_page)
        try:
            self.scan_page.btn_save_connect.setEnabled(True)     # ready to reconnect
            self.scan_page.btn_start_trial.setEnabled(False)     # needs a live connection first
            self.scan_page.btn_calibrate_torque.setEnabled(False)
            self.scan_page.status.setText("Exo rebooted - reconnect when it reappears.")
        except Exception:
            pass
        try:
            self.trial_page.clear_plots()
        except Exception as e:
            self.logger.error(f"Failed to clear trial page plots: {e}")
            self.logger.debug(traceback.format_exc())

    @QtCore.Slot()
    def _on_disconnect(self):
        try:
            self._destroy_controller_db()
            self.logger.info("Manual disconnect requested")
            # Disconnect immediately (non-blocking, no popup)
            self.qt_dev.disconnect()
            
            # Navigate to scan page immediately
            self.stack.setCurrentWidget(self.scan_page)
            # Stop CSV if running
            if self._csv_file is not None:
                try:
                    self._csv_file.flush(); self._csv_file.close()
                    self.logger.info(f"CSV file closed on disconnect: {self._csv_path_last}")
                except Exception as e:
                    self.logger.error(f"Failed to close CSV on disconnect: {e}")
                    self.logger.debug(traceback.format_exc())
                self._csv_file = None
                self._csv_writer = None
                self._csv_header_written = False
                self._t0 = None
                self._mark_counter = 0  # Reset mark counter
                try:
                    if self._csv_path_last:
                        self.scan_page.status.setText(f"Disconnected. CSV saved: {self._csv_path_last}")
                except Exception as e:
                    self.logger.error(f"Failed to update status on disconnect: {e}")
                    self.logger.debug(traceback.format_exc())
                try:
                    self.trial_page.update_mark_count(0)
                except Exception as e:
                    self.logger.error(f"Failed to reset mark count on disconnect: {e}")
                    self.logger.debug(traceback.format_exc())
        except Exception as e:
            self.logger.error(f"Failed to disconnect: {e}")
            self.logger.debug(traceback.format_exc())

    @QtCore.Slot()
    def _navigate_to_scan(self):
        try:
            self._on_disconnect()
        except Exception as e:
            self.logger.error(f"Failed during navigate to scan: {e}")
            self.logger.debug(traceback.format_exc())
        self.stack.setCurrentWidget(self.scan_page)

    @QtCore.Slot()
    def _on_save_csv(self):
        """Save current CSV file and immediately start a new one."""
        try:
            saved_path = None
            # Close current CSV if open
            if self._csv_file is not None:
                self.logger.info("Saving current CSV and starting new one")
                try:
                    self._csv_file.flush()
                    self._csv_file.close()
                    saved_path = self._csv_path_last
                    self.logger.info(f"CSV saved: {saved_path}")
                except Exception as e:
                    self.logger.error(f"Failed to save CSV: {e}")
                    self.logger.debug(traceback.format_exc())
                self._csv_file = None
                self._csv_writer = None
                self._csv_header_written = False
                self._t0 = None
                self._csv_path_last = None
            
            # Start a new CSV file immediately
            self._start_csv_auto()
            
            # Show confirmation banner
            try:
                if saved_path:
                    save_dir = os.path.dirname(saved_path)
                    filename = os.path.basename(saved_path)
                    msg = f"✓ CSV saved: {filename}\nDirectory: {save_dir}"
                    self.scan_page.status.setText(msg)
                    # Also show a message box for better visibility
                    QtWidgets.QMessageBox.information(
                        self,
                        "CSV Saved",
                        f"CSV file saved successfully:\n{filename}\n\nLocation: {save_dir}"
                    )
                else:
                    self.scan_page.status.setText("New CSV logging started")
            except Exception as e:
                self.logger.error(f"Failed to show CSV save confirmation: {e}")
                self.logger.debug(traceback.format_exc())
        except Exception as e:
            self.logger.error(f"Failed in _on_save_csv: {e}")
            self.logger.debug(traceback.format_exc())

    @QtCore.Slot()
    def _on_update_controller(self):
        # Choose page based on whether we have controller metadata (handshake + matrix)
        has_matrix = bool(self._controller_matrix)
        self.logger.info(f"Update controller requested, has_matrix={has_matrix}")
        if has_matrix:
            try:
                self.settings_page.set_controller_matrix(self._controller_matrix)
            except Exception as e:
                self.logger.error(f"Failed to set controller matrix in settings: {e}")
                self.logger.debug(traceback.format_exc())
            # Re-show any incomplete-list warning: the one raised at connect has usually
            # timed out by now, and this page is where a missing controller is noticed.
            if self._controller_matrix_warning:
                try:
                    self.settings_page.set_param_update_status(
                        self._controller_matrix_warning, warning=True)
                except Exception as e:
                    self.logger.error(f"Failed to re-show controller list warning: {e}")
                    self.logger.debug(traceback.format_exc())
            self.stack.setCurrentWidget(self.settings_page)
        else:
            self.stack.setCurrentWidget(self.basic_settings_page)

    @QtCore.Slot()
    def _on_bio_feedback(self):
        self.logger.info("Navigating to bio feedback page")
        try:
            self.bio_feedback_page.start_plotting()
        except Exception as e:
            self.logger.error(f"Failed to start plotting on bio feedback page: {e}")
            self.logger.debug(traceback.format_exc())
        self.stack.setCurrentWidget(self.bio_feedback_page)

    def _on_bio_feedback_back(self):
        self.logger.info("Navigating back from bio feedback page")
        try:
            self.bio_feedback_page.stop_plotting()
        except Exception as e:
            self.logger.error(f"Failed to stop plotting on bio feedback page: {e}")
            self.logger.debug(traceback.format_exc())
        self.stack.setCurrentWidget(self.trial_page)

    @QtCore.Slot()
    def _on_machine_learning(self):
        # Placeholder hook
        pass

    def _apply_param_update(self, payload) -> bool:
        """Core parameter-update path shared by the GUI Apply button and the UDP
        remote. Validates, queues the pending ack, and sends over BLE. Does NOT
        navigate — callers that are GUI buttons handle navigation themselves."""
        self.logger.info(f"Applying param update: {payload}")
        try:
            updates = QtExoDeviceManager.build_parameter_updates(payload)
            self._queue_pending_param_updates(updates)
            self._show_param_update_status("", warning=False)
        except Exception as e:
            self.logger.error(f"Invalid parameter update request: {e}")
            self.logger.debug(traceback.format_exc())
            self._show_param_update_status(f"Controller update not sent: {e}", warning=True)
            return False

        try:
            self.qt_dev.updateTorqueValues(payload)
        except Exception as e:
            self.logger.error(f"Failed to update torque values: {e}")
            self.logger.debug(traceback.format_exc())
            return False
        return True

    @QtCore.Slot(list)
    def _on_apply_settings(self, payload):
        # payload: [isBilateral, joint, controller, parameter, value]
        self._apply_param_update(payload)
        # Return to trial page (GUI-button behavior only).
        try:
            self.stack.setCurrentWidget(self.trial_page)
        except Exception as e:
            self.logger.error(f"Failed to navigate to trial page after settings: {e}")
            self.logger.debug(traceback.format_exc())

    def _clear_ble_prefs_on_new_connection(self):
        """Purge saved Update-Controller prefs and in-memory selections for a new link.

        Does **not** clear ``_controller_matrix`` / ``_controller_values``. Handshake
        payloads can be processed before the ``connected`` slot runs; wiping the matrix
        here leaves ``has_matrix`` false and forces Basic settings forever.
        """
        try:
            SettingsManager.purge_ble_device_controller_prefs()
        except Exception as e:
            self.logger.error(f"Failed to purge saved controller prefs: {e}")
            self.logger.debug(traceback.format_exc())
        try:
            self.settings_page.clear_device_session_preferences()
            self.basic_settings_page.clear_device_session_preferences()
        except Exception as e:
            self.logger.error(f"Failed to reset settings page device prefs: {e}")
            self.logger.debug(traceback.format_exc())

    def _destroy_controller_db(self):
        self._clear_ble_prefs_on_new_connection()
        try:
            self._controller_matrix = []
            self._controller_values = {}
            self.settings_page.set_controller_matrix([])
            self.settings_page.set_controller_values({})
        except Exception as e:
            self.logger.error(f"Failed to destroy controller DB: {e}")
            self.logger.debug(traceback.format_exc())

    @QtCore.Slot(str)
    def _on_dev_log(self, msg: str):
        try:
            self.scan_page.status.setText(msg)
        except Exception as e:
            self.logger.error(f"Failed to update scan page status: {e}")
            self.logger.debug(traceback.format_exc())

    @QtCore.Slot(str)
    def _on_dev_error(self, msg: str):
        self.logger.error(f"Device error: {msg}")
        try:
            self.scan_page.status.setText(f"Connection failed: {msg}")
            has_manual_selection = bool(self.scan_page.list_devices.selectedItems())
            self.scan_page.btn_save_connect.setEnabled(has_manual_selection)
            self.scan_page.btn_start_trial.setEnabled(False)
            try:
                self.scan_page.btn_calibrate_torque.setEnabled(False)
            except Exception as e:
                self.logger.error(f"Failed to disable calibrate torque button: {e}")
                self.logger.debug(traceback.format_exc())
        except Exception as e:
            self.logger.error(f"Failed to handle device error UI update: {e}")
            self.logger.debug(traceback.format_exc())

    @QtCore.Slot(str, str)
    def _on_dev_connected(self, name: str, addr: str):
        self.logger.info(f"Device connected: {name} {addr}")
        # Clear stale **saved** prefs only; do not wipe handshake matrix (handshake can
        # arrive before this slot — _destroy_controller_db would erase it → Basic-only UI).
        try:
            self._clear_ble_prefs_on_new_connection()
        except Exception as e:
            self.logger.error(f"Failed to clear BLE prefs on connect: {e}")
            self.logger.debug(traceback.format_exc())
        try:
            self.rt_bridge.reset_for_new_ble_session()
        except Exception as e:
            self.logger.error(f"Failed to reset RtBridge for new session: {e}")
            self.logger.debug(traceback.format_exc())
        # Keep Update Controller enabled. The old "disable until handshake" logic raced BLE:
        # handshake + matrix can arrive before this slot runs, then we wiped state and left
        # the button stuck disabled. Basic vs full settings still chosen in _on_update_controller.
        try:
            self.trial_page.set_update_controller_enabled(True)
            QtCore.QTimer.singleShot(
                0,
                lambda: self.trial_page.set_update_controller_enabled(True),
            )
        except Exception as e:
            self.logger.error(f"Failed to enable Update Controller after connect: {e}")
            self.logger.debug(traceback.format_exc())
        try:
            self.scan_page.status.setText(f"Connected: {name} {addr}")
            # Don't enable Save & Connect after connection - it's already connected
            self.scan_page.btn_save_connect.setEnabled(False)
            self.scan_page.btn_start_trial.setEnabled(False)  # Enabled after torque calibration
            try:
                self.scan_page.btn_calibrate_torque.setEnabled(True)
            except Exception as e:
                self.logger.error(f"Failed to enable calibrate torque button: {e}")
                self.logger.debug(traceback.format_exc())
        except Exception as e:
            self.logger.error(f"Failed to update UI on connection: {e}")
            self.logger.debug(traceback.format_exc())
        # Clear old plot data on reconnect
        try:
            self.trial_page.clear_plots()
        except Exception as e:
            self.logger.error(f"Failed to clear plots on connection: {e}")
            self.logger.debug(traceback.format_exc())
        # If the handshake already ran, push matrix/values into the settings page.
        try:
            if self._controller_matrix:
                self.settings_page.set_controller_matrix(self._controller_matrix)
                self.settings_page.set_controller_values(self._controller_values)
        except Exception as e:
            self.logger.error(f"Failed to sync settings page after connect: {e}")
            self.logger.debug(traceback.format_exc())

    # Bits are nRF52840 POWER->RESETREAS; the firmware sends the names, this maps them to what
    # each one means for the mid-trial freeze investigation. See ExoCode/src/SystemReset.h.
    _RESET_REASON_MEANING = {
        "PORBOR":   "power-on or BROWNOUT. No crash was recorded. If nobody power-cycled it,\n"
                    "             this is a power event - or a hang that needed a manual cycle.",
        "LOCKUP":   "CPU LOCKUP - a hard fault escalated. This is a firmware crash.",
        "SREQ":     "software reset (NVIC_SystemReset). Expected right after an End Trial 'Z';\n"
                    "             unexpected otherwise - could be an Mbed fault handler reboot.",
        "DOG":      "watchdog timeout - but this firmware configures no watchdog. Investigate.",
        "RESETPIN": "reset pin - the button was pressed, or the board was re-flashed.",
        "VBUS":     "USB VBUS detected - board woke on USB power being applied.",
    }

    @QtCore.Slot(str)
    def _on_reset_reason(self, reason: str):
        """Print why the Nano last reset, at connect time, before the user touches anything.

        This is the discriminator for the mid-trial freeze: the GUI's "unexpectedly disconnected"
        is a BLE supervision timeout ~9.6 s after the Nano stops transmitting, and this says
        whether the Nano crashed, was reset, or lost power underneath it.
        """
        try:
            self.logger.warning("Nano last reset reason: %s", reason)

            if not reason.startswith("RST:"):
                # ErrorChar holds a runtime error report ("<code>:<joint>") instead - a device
                # error landed after boot and overwrote the parked value.
                body = (f" Raw value: {reason}\n"
                        " Not a reset code - a device error overwrote it after boot.")
            elif reason == "RST:UNAVAILABLE":
                body = " RESETREAS is not reachable on this core build."
            else:
                parts = reason.split(":")
                code = parts[1] if len(parts) > 1 else "?"
                names = parts[2] if len(parts) > 2 else ""
                lines = [f" RESETREAS = {code}  ({names})"]
                for name in [n for n in names.split(",") if n]:
                    meaning = self._RESET_REASON_MEANING.get(name)
                    if meaning:
                        lines.append(f"   {name}: {meaning}")
                body = "\n".join(lines)

            print("\n" + "=" * 68 + "\n"
                  " NANO LAST RESET REASON\n"
                  + body + "\n"
                  + "=" * 68 + "\n")
        except Exception as e:
            self.logger.error(f"Failed to report reset reason: {e}")
            self.logger.debug(traceback.format_exc())

    def _show_disconnect_warning(self):
        """Show disconnect warning dialog (called after disconnect handling completes)."""
        try:
            QtWidgets.QMessageBox.warning(
                self, 
                "Device Disconnected", 
                "The device has been unexpectedly disconnected.\n\nMotors have been turned off and the trial data has been saved."
            )
        except Exception as e:
            self.logger.error(f"Failed to show disconnect warning: {e}")
            self.logger.debug(traceback.format_exc())

    @QtCore.Slot()
    def _on_dev_disconnected(self):
        """Handle unexpected disconnects (intentional disconnects don't trigger this)."""
        if getattr(self, '_shutting_down', False):
            # Expected: the Nano rebooted as part of End Trial. The ShutdownDialog owns the UX
            # (countdown -> return to scan); don't show the "unexpected disconnect" warning.
            self.logger.info("Disconnect during shutdown handshake (expected) - deferring to ShutdownDialog")
            return
        self.logger.warning("Device disconnected unexpectedly")
        try:
            self._destroy_controller_db()
            self.scan_page.status.setText("Disconnected unexpectedly")
            self.scan_page.btn_save_connect.setEnabled(True)
            self.scan_page.btn_start_trial.setEnabled(False)
            try:
                self.scan_page.btn_calibrate_torque.setEnabled(False)
            except Exception as e:
                self.logger.error(f"Failed to disable calibrate torque button: {e}")
                self.logger.debug(traceback.format_exc())
            
            # Device is already disconnected, no need to send commands
            # (motorOff/stopTrial would fail with "Not connected" errors)
            self.logger.info("Device already disconnected - skipping motor off/stop trial commands")
            # Show non-blocking notification of unexpected disconnect
            # Use QTimer to defer the dialog so disconnect handling completes first
            try:
                QtCore.QTimer.singleShot(100, lambda: self._show_disconnect_warning())
            except Exception as e:
                self.logger.error(f"Failed to schedule disconnect warning dialog: {e}")
                self.logger.debug(traceback.format_exc())
            # Navigate back to the Scan page on unexpected disconnect
            self.stack.setCurrentWidget(self.scan_page)
            
            # Ensure CSV is closed and announce saved path
            if self._csv_file is not None:
                try:
                    self._csv_file.flush(); self._csv_file.close()
                    self.logger.info(f"CSV closed after disconnect: {self._csv_path_last}")
                except Exception as e:
                    self.logger.error(f"Failed to close CSV after disconnect: {e}")
                    self.logger.debug(traceback.format_exc())
                self._csv_file = None
                self._csv_writer = None
                self._csv_header_written = False
                self._t0 = None
                self._mark_counter = 0  # Reset mark counter
                try:
                    if self._csv_path_last:
                        self.scan_page.status.setText(f"Unexpected disconnect. CSV saved: {self._csv_path_last}")
                except Exception as e:
                    self.logger.error(f"Failed to update status after disconnect: {e}")
                    self.logger.debug(traceback.format_exc())
                try:
                    self.trial_page.update_mark_count(0)
                except Exception as e:
                    self.logger.error(f"Failed to reset mark count after disconnect: {e}")
                    self.logger.debug(traceback.format_exc())
        except Exception as e:
            self.logger.error(f"Failed to handle device disconnect: {e}")
            self.logger.debug(traceback.format_exc())

    def _start_csv_auto(self):
        # Save within Qt/Saved_Data
        base_dir = os.path.dirname(__file__)  # Python_GUI/Qt
        save_dir = os.path.join(base_dir, "Saved_Data")
        try:
            os.makedirs(save_dir, exist_ok=True)
        except Exception as e:
            self.logger.error(f"Failed to create Saved_Data directory: {e}")
            self.logger.debug(traceback.format_exc())
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        # Add preamble if set
        if self._csv_preamble:
            fname = os.path.join(save_dir, f"{self._csv_preamble}_trial_{ts}.csv")
        else:
            fname = os.path.join(save_dir, f"trial_{ts}.csv")
        try:
            self._csv_file = open(fname, "w", newline="")
            self._csv_writer = csv.writer(self._csv_file)
            self._csv_header_written = False
            self._t0 = None
            self._mark_counter = 0  # Reset mark counter for new trial
            self._csv_path_last = fname
            self.logger.info(f"Started CSV logging to: {fname}")
            try:
                self.scan_page.status.setText(f"Logging to {fname}")
            except Exception as e:
                self.logger.error(f"Failed to update status with CSV path: {e}")
                self.logger.debug(traceback.format_exc())
            try:
                self.trial_page.update_mark_count(0)
            except Exception as e:
                self.logger.error(f"Failed to reset mark count: {e}")
                self.logger.debug(traceback.format_exc())
        except Exception as e:
            self.logger.error(f"Failed to start CSV auto-logging: {e}")
            self.logger.debug(traceback.format_exc())
            self._csv_file = None
            self._csv_writer = None

