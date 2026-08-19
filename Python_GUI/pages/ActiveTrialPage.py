import sys
import math
import time
from collections import deque

try:
    from PySide6 import QtCore, QtWidgets, QtGui
except ImportError as e:
    raise SystemExit("PySide6 is required. Install with: pip install PySide6") from e

try:
    import pyqtgraph as pg
except ImportError as e:
    raise SystemExit("pyqtgraph is required. Install with: pip install pyqtgraph") from e

from utils import (
    UIConfig, PlotConfig,
    load_logo, create_separator, create_section_label,
    apply_button_style_batch, set_size_policy_fixed_height
)

# The exo clock ("Exoskeleton time (seconds)") reaches the GUI as a signed 16-bit
# fixed-point value: RealTimeI2C packs every RT channel as short(value * 100) across
# the Teensy->coms-MCU I2C hop (FIXED_POINT_FACTOR = 100). Uptime seconds * 100
# overflows int16 at 327.68s, so the raw value jumps +327.67 -> -327.68 (a full
# int16 span) and repeats every 655.36s. _x_for_sample undoes that wrap.
_EXO_TIME_WRAP_SPAN = 65536 / 100.0   # 655.36 s: one full int16 range of the *100 clock
_EXO_TIME_WRAP_HALF = _EXO_TIME_WRAP_SPAN / 2.0  # 327.68 s: wrap-detection threshold


class ActiveTrialPage(QtWidgets.QWidget):
    """Active Trial page with two stacked real-time plots (simulated data)."""

    # Which real-time channels each press of "Toggle Data Points" shows.
    #   4-channel page -> (top curve A, top curve B, bottom curve A, bottom curve B)
    #   2-channel page -> (top panel, bottom panel), one curve on each
    # Channel numbers must match ExoCode/src/PlottingTitles.h getColumnHeader/bilateral_ankle.
    # Channels 10-12 (Status, exo clock, battery) are deliberately absent: each already has its
    # own GUI readout, and their scales would wreck a shared torque axis.
    _PLOT_PAGES = (
        (0, 1, 2, 3),   # Desired vs Measured torque, per leg
        (4, 5, 6, 7),   # Toe FSR overlaid with the In Stance step change, per foot
        (8, 9),         # Commanded torque - one leg per panel, never overlaid
    )

    # Signals to be handled by MainWindow (placeholders can be wired later)
    endTrialRequested = QtCore.Signal()
    saveCsvRequested = QtCore.Signal()
    updateControllerRequested = QtCore.Signal()
    bioFeedbackRequested = QtCore.Signal()
    machineLearningRequested = QtCore.Signal()
    recalibrateFSRRequested = QtCore.Signal()
    sendPresetFSRRequested = QtCore.Signal()
    recalibrateTorqueRequested = QtCore.Signal()
    markTrialRequested = QtCore.Signal()
    deviceStartRequested = QtCore.Signal()
    deviceStopRequested = QtCore.Signal()
    csvPreambleChanged = QtCore.Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("ActiveTrialPage")
        self._base_button_font_size = 13  # Store base size for scaling
        self._build_ui()
        self._init_state()
    
    def resizeEvent(self, event):
        """Dynamically adjust font sizes and button heights."""
        super().resizeEvent(event)
        self._apply_responsive_layout()

    def showEvent(self, event):
        super().showEvent(event)
        QtCore.QTimer.singleShot(0, self._apply_responsive_layout)

    def _apply_responsive_layout(self):
        width = max(1, self.width())
        height = max(1, self.height())
        scale_w = width / 900.0
        scale_h = height / 700.0
        scale_factor = max(0.7, min(1.35, min(scale_w, scale_h)))

        new_font_size = max(9, int(self._base_button_font_size * scale_factor))
        new_btn_height = max(32, int(UIConfig.BTN_HEIGHT_SMALL * scale_factor))

        buttons = [
            self.btn_toggle_points, self.btn_end_trial, self.btn_save_csv,
            self.btn_set_preamble, self.btn_update_controller, self.btn_bio_feedback,
            self.btn_ml, self.btn_recal_fsr, self.btn_send_preset_fsr, self.btn_recal_torque,
            self.btn_mark, self.btn_pause_play,
        ]
        target_width = None
        try:
            if hasattr(self, "_controls_scroll") and self._controls_scroll:
                target_width = max(160, self._controls_scroll.viewport().width() - 12)
        except Exception:
            target_width = None

        for btn in buttons:
            f = btn.font()
            f.setPointSize(new_font_size)
            btn.setFont(f)
            btn.setMinimumHeight(new_btn_height)
            if target_width:
                btn.setFixedWidth(target_width)
            base_style = ""
            try:
                base_style = self._button_base_styles.get(btn, "")
            except Exception:
                base_style = ""
            btn.setStyleSheet(f"{base_style} font-size: {new_font_size}pt;")

        self.btn_end_trial.setMinimumHeight(int(new_btn_height * 1.25))
        self.btn_pause_play.setMinimumHeight(int(new_btn_height * 1.1))

    def _build_ui(self):
        # Main horizontal layout: left controls, right plots
        main = QtWidgets.QHBoxLayout(self)
        main.setContentsMargins(UIConfig.MARGIN_PAGE, UIConfig.MARGIN_PAGE, UIConfig.MARGIN_PAGE, UIConfig.MARGIN_PAGE)
        main.setSpacing(UIConfig.SPACING_XLARGE)

        # Left controls column
        controls = QtWidgets.QVBoxLayout()
        controls.setSpacing(UIConfig.SPACING_MEDIUM)
        
        # Top row with OpenExo logo and battery
        logo_row = QtWidgets.QHBoxLayout()
        
        # OpenExo logo on left - smaller for compact layout
        openexo_logo = load_logo(
            "OpenExo.png",
            UIConfig.LOGO_OPENEXO_SMALL_WIDTH,
            UIConfig.LOGO_OPENEXO_SMALL_HEIGHT
        )
        if openexo_logo:
            logo_row.addWidget(openexo_logo)
        
        logo_row.addStretch(1)
        
        controls.addLayout(logo_row)
        controls.addSpacing(UIConfig.SPACING_SMALL)
        
        title = QtWidgets.QLabel("Active Trial")
        f = title.font(); f.setPointSize(UIConfig.FONT_SUBTITLE); title.setFont(f)
        controls.addWidget(title)

        self.lbl_battery = QtWidgets.QLabel("Battery: --")
        self.lbl_battery.setStyleSheet(f"font-size: {UIConfig.FONT_SMALL}pt; color: {UIConfig.COLOR_SUCCESS}; font-weight: bold;")
        self.lbl_battery.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
        controls.addWidget(self.lbl_battery)

        # Exo status readout, driven by the HIJACKED RT Channel 8 (firmware exo status word).
        # Shows the live calibration/refinement/trial state; flips to "Trial On / Ready" the
        # moment FSR refinement finishes. See update_exo_status().
        self.lbl_exo_status = QtWidgets.QLabel("Exo: --")
        self.lbl_exo_status.setStyleSheet(f"font-size: {UIConfig.FONT_SMALL}pt; font-weight: bold;")
        self.lbl_exo_status.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
        controls.addWidget(self.lbl_exo_status)

        self.lbl_param_update_status = QtWidgets.QLabel("")
        self.lbl_param_update_status.setWordWrap(True)
        self.lbl_param_update_status.setStyleSheet(
            f"font-size: {UIConfig.FONT_TINY}pt; color: {UIConfig.COLOR_PARAM_REJECT}; font-weight: bold;"
        )
        controls.addWidget(self.lbl_param_update_status)

        controls.addSpacing(UIConfig.MARGIN_PAGE)

        # ═══════ PRIORITY: CRITICAL CONTROLS ═══════
        # End Trial - Big prominent button at top
        self.btn_end_trial = QtWidgets.QPushButton("END TRIAL")
        controls.addWidget(self.btn_end_trial)
        
        # Add more spacing between End Trial and Pause
        controls.addSpacing(0)
        
        # Pause/Play button
        self.btn_pause_play = QtWidgets.QPushButton("Pause")
        self.is_paused = False
        controls.addWidget(self.btn_pause_play)
        
        # Separator
        controls.addSpacing(UIConfig.SPACING_XLARGE)
        controls.addWidget(create_separator())
        
        # ═══════ COMMON ACTIONS ═══════
        controls.addWidget(create_section_label("Common Actions"))
        controls.addSpacing(UIConfig.SPACING_SMALL)
        
        self.btn_update_controller = QtWidgets.QPushButton("Update Controller")
        controls.addWidget(self.btn_update_controller)
        self.btn_update_controller.setEnabled(True)
        
        self.btn_mark = QtWidgets.QPushButton("Mark Trial (0)")
        controls.addWidget(self.btn_mark)
        
        self.btn_save_csv = QtWidgets.QPushButton("Save & New CSV")
        controls.addWidget(self.btn_save_csv)
        
        # Separator
        controls.addSpacing(UIConfig.SPACING_XLARGE)
        controls.addWidget(create_separator())
        
        # ═══════ SETTINGS ═══════
        controls.addWidget(create_section_label("Settings"))
        controls.addSpacing(UIConfig.SPACING_SMALL)
        
        self.btn_set_preamble = QtWidgets.QPushButton("Set CSV Prefix")
        controls.addWidget(self.btn_set_preamble)
        
        self.btn_toggle_points = QtWidgets.QPushButton("Toggle Data Points")
        controls.addWidget(self.btn_toggle_points)
        
        # Separator
        controls.addSpacing(UIConfig.SPACING_XLARGE)
        controls.addWidget(create_separator())
        
        # ═══════ ADVANCED ═══════
        controls.addWidget(create_section_label("Advanced"))
        controls.addSpacing(UIConfig.SPACING_SMALL)
        
        self.btn_bio_feedback = QtWidgets.QPushButton("Bio Feedback")
        controls.addWidget(self.btn_bio_feedback)
        
        self.btn_ml = QtWidgets.QPushButton("Machine Learning")
        controls.addWidget(self.btn_ml)
        
        self.btn_recal_fsr = QtWidgets.QPushButton("Recalibrate FSRs")
        controls.addWidget(self.btn_recal_fsr)
        
        self.btn_send_preset_fsr = QtWidgets.QPushButton("Send Preset FSR")
        controls.addWidget(self.btn_send_preset_fsr)

        self.btn_recal_torque = QtWidgets.QPushButton("Recalibrate Torque Sensor")
        controls.addWidget(self.btn_recal_torque)
        
        controls.addStretch(1)

        # Right plots column
        plots_col = QtWidgets.QVBoxLayout()
        # Create a GraphicsLayoutWidget to host two plot rows
        self.graph = pg.GraphicsLayoutWidget()
        plots_col.addWidget(self.graph, 1)

        # Top plot (e.g., Controller vs Measurement)
        self.plot_top = self.graph.addPlot(row=0, col=0)
        self.plot_top.showGrid(x=True, y=True, alpha=PlotConfig.GRID_ALPHA)
        self.plot_top.addLegend()
        self.plot_top.setLabel("left", "Top Signal")
        self.plot_top.setLabel("bottom", "t (s)")
        self.curve_top_cmd = self.plot_top.plot(
            pen=pg.mkPen(PlotConfig.COLOR_CONTROLLER, width=PlotConfig.CURVE_WIDTH),
            name='Controller'
        )
        self.curve_top_meas = self.plot_top.plot(
            pen=pg.mkPen(PlotConfig.COLOR_MEASUREMENT, width=PlotConfig.CURVE_WIDTH),
            name='Measurement'
        )

        # Bottom plot (e.g., additional signals)
        self.plot_bottom = self.graph.addPlot(row=1, col=0)
        self.plot_bottom.showGrid(x=True, y=True, alpha=PlotConfig.GRID_ALPHA)
        self.plot_bottom.addLegend()
        self.plot_bottom.setLabel("left", "Bottom Signal")
        self.plot_bottom.setLabel("bottom", "t (s)")
        self.curve_bot_a = self.plot_bottom.plot(
            pen=pg.mkPen(PlotConfig.COLOR_SIGNAL_A, width=PlotConfig.CURVE_WIDTH),
            name='Signal A'
        )
        self.curve_bot_b = self.plot_bottom.plot(
            pen=pg.mkPen(PlotConfig.COLOR_SIGNAL_B, width=PlotConfig.CURVE_WIDTH),
            name='Signal B'
        )

        # Wrap controls in a widget
        controls_widget = QtWidgets.QWidget()
        controls_widget.setLayout(controls)

        # Scrollable container to prevent overlap on small windows
        controls_scroll = QtWidgets.QScrollArea()
        controls_scroll.setWidgetResizable(True)
        controls_scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        controls_scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        controls_scroll.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAsNeeded)
        controls_scroll.setWidget(controls_widget)
        controls_widget.setMinimumWidth(200)
        self._controls_scroll = controls_scroll

        # Assemble - controls and plots both expand proportionally
        main.addWidget(controls_scroll, 1)  # Stretch factor 1 - controls can grow
        main.addLayout(plots_col, 3)  # Stretch factor 3 - plots get more space

        # Wiring (device control; sim controls available via methods if needed)
        self.btn_pause_play.clicked.connect(self._on_pause_play_clicked)
        self.btn_toggle_points.clicked.connect(self._toggle_points)
        # Emit-only wiring; MainWindow can connect these to actual actions
        self.btn_end_trial.clicked.connect(self.endTrialRequested.emit)
        self.btn_save_csv.clicked.connect(self.saveCsvRequested.emit)
        self.btn_set_preamble.clicked.connect(self._on_set_preamble_clicked)
        self.btn_update_controller.clicked.connect(self.updateControllerRequested.emit)
        self.btn_bio_feedback.clicked.connect(self.bioFeedbackRequested.emit)
        self.btn_ml.clicked.connect(self.machineLearningRequested.emit)
        self.btn_recal_fsr.clicked.connect(self.recalibrateFSRRequested.emit)
        self.btn_send_preset_fsr.clicked.connect(self.sendPresetFSRRequested.emit)
        self.btn_recal_torque.clicked.connect(self.recalibrateTorqueRequested.emit)
        self.btn_mark.clicked.connect(self.markTrialRequested.emit)

        # Apply consistent button styling
        buttons = [
            self.btn_toggle_points, self.btn_end_trial, self.btn_save_csv,
            self.btn_set_preamble, self.btn_update_controller, self.btn_bio_feedback,
            self.btn_ml, self.btn_recal_fsr, self.btn_send_preset_fsr, self.btn_recal_torque,
            self.btn_mark, self.btn_pause_play,
        ]
        apply_button_style_batch(buttons, height=UIConfig.BTN_HEIGHT_SMALL, padding="6px 10px")
        
        # Set size policy for all buttons
        for btn in buttons:
            set_size_policy_fixed_height(btn)
            btn.setStyleSheet(btn.styleSheet() + " margin-bottom: 2px;")
        
        # Special styling for End Trial button - make it prominent and red
        f = self.btn_end_trial.font()
        f.setPointSize(UIConfig.FONT_MEDIUM)
        f.setBold(True)
        self.btn_end_trial.setFont(f)
        self.btn_end_trial.setMinimumHeight(int(UIConfig.BTN_HEIGHT_LARGE * 1.2))
        self.btn_end_trial.setStyleSheet(
            f"background-color: {UIConfig.COLOR_CRITICAL}; color: white; padding: 10px; "
            "margin-bottom: 0px; font-weight: bold; border-radius: 4px;"
        )
        
        # Special styling for Pause button - make it blue
        self.btn_pause_play.setMinimumHeight(int(UIConfig.BTN_HEIGHT_LARGE * 1.05))
        self.btn_pause_play.setStyleSheet(
            f"background-color: {UIConfig.COLOR_ACTION}; color: white; padding: 8px; "
            "margin-bottom: 0px; font-weight: bold; border-radius: 4px;"
        )

        # Capture base styles for resizing updates
        self._button_base_styles = {btn: btn.styleSheet() for btn in buttons}

    def _init_state(self):
        # Fixed-size buffers for plotting (seconds-window * rate)
        self.rate_hz = PlotConfig.RATE_HZ
        self.window_secs = PlotConfig.WINDOW_SECS
        self.maxlen = self.rate_hz * self.window_secs
        self.t0 = time.time()

        self.t_vals = deque(maxlen=self.maxlen)
        self.top_cmd_vals = deque(maxlen=self.maxlen)
        self.top_meas_vals = deque(maxlen=self.maxlen)
        self.bot_a_vals = deque(maxlen=self.maxlen)
        self.bot_b_vals = deque(maxlen=self.maxlen)

        # Timer for updates
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._on_tick)
        self.timer.setTimerType(QtCore.Qt.PreciseTimer)
        # Toggle state: which 4-value block to plot (0..3 or 4..7)
        self._block_index = 0  # index into _PLOT_PAGES; see _page_slots()/_toggle_points()
        # Store dynamic parameter names from device
        self._param_names = []
        # Track real data start time
        self._real_data_t0 = None
        # Plot x-axis uses the exo's own timestamp (smooth ~100Hz) instead of bursty BLE
        # wall-clock arrival. Index is resolved from the channel labels (None -> fall back
        # to wall-clock); _exo_t0 anchors the axis to start near 0. See _x_for_sample.
        self._exo_time_idx = None
        self._exo_t0 = None
        # int16 wrap tracking for the exo clock (see _EXO_TIME_WRAP_SPAN, _x_for_sample)
        self._exo_prev_raw = None
        self._exo_wrap_offset = 0.0

    def set_update_controller_enabled(self, enabled: bool):
        try:
            self.btn_update_controller.setEnabled(bool(enabled))
        except Exception:
            pass

    def update_mark_count(self, count: int):
        """Update the Mark Trial button to show current count."""
        try:
            self.btn_mark.setText(f"Mark Trial ({count})")
        except Exception:
            pass

    def update_exo_status(self, status_value: float):
        """Show the exo state, driven by the hijacked RT Channel 8 (firmware status word,
        status_defs::messages). Flips to 'Trial On / Ready' when FSR refinement finishes.
        See uart_commands.h bilateral_ankle data[8] on the Teensy for the transport."""
        try:
            code = int(round(float(status_value)))
        except (ValueError, TypeError):
            return
        names = {
            0: "Off", 1: "Trial Off", 2: "Trial On / Ready", 3: "Test",
            4: "Torque Calibration", 5: "FSR Calibration",
            6: "FSR Refinement", 7: "Motor Startup",
        }
        if code in names:
            name = names[code]
        elif code >= 8:                       # error bit (8) or higher error/warning codes
            name = f"Error / Warning ({code})"
        else:
            name = f"Status {code}"
        # Colour cue: green = ready, orange = calibrating/refining/startup, red = error.
        if code == 2:
            color = UIConfig.COLOR_SUCCESS
        elif code in (4, 5, 6, 7):
            color = UIConfig.COLOR_PARAM_REJECT
        elif code >= 8:
            color = UIConfig.COLOR_WARNING
        else:
            color = None                      # off / trial_off -> default theme colour
        style = f"font-size: {UIConfig.FONT_SMALL}pt; font-weight: bold;"
        if color:
            style += f" color: {color};"
        try:
            self.lbl_exo_status.setText(f"Exo: {name}")
            self.lbl_exo_status.setStyleSheet(style)
        except Exception:
            pass

    def update_battery_level(self, voltage: float):
        """Update the battery level display."""
        try:
            self.lbl_battery.setText(f"Battery: {voltage:.2f}V")
            # Change color if low (< 11V is typical low for 3S lipo, or 0V)
            if voltage < UIConfig.BATTERY_LOW_VOLTAGE:
                self.lbl_battery.setStyleSheet(
                    f"font-size: {UIConfig.FONT_SMALL}pt; color: {UIConfig.COLOR_WARNING}; font-weight: bold;"
                )
            else:
                self.lbl_battery.setStyleSheet(
                    f"font-size: {UIConfig.FONT_SMALL}pt; color: {UIConfig.COLOR_SUCCESS}; font-weight: bold;"
                )
        except Exception:
            pass

    def set_param_update_status(self, message: str, warning: bool = True):
        try:
            text = message or ""
            color = UIConfig.COLOR_PARAM_REJECT if warning else UIConfig.COLOR_LABEL
            self.lbl_param_update_status.setStyleSheet(
                f"font-size: {UIConfig.FONT_TINY}pt; color: {color}; font-weight: bold;"
            )
            self.lbl_param_update_status.setText(text)
            if text and warning:
                QtCore.QTimer.singleShot(8000, lambda expected=text: self._clear_param_update_status(expected))
        except Exception:
            pass

    def _clear_param_update_status(self, expected: str):
        try:
            if self.lbl_param_update_status.text() == expected:
                self.lbl_param_update_status.setText("")
        except Exception:
            pass

    def clear_plots(self):
        """Clear all plot data and reset timing."""
        try:
            # Clear all data buffers
            self._clear_plot_buffers()
            
            # Reset pause state when clearing plots
            self.is_paused = False
            self.btn_pause_play.setText("Pause")
        except Exception:
            pass

    def _clear_plot_buffers(self):
        self.t_vals.clear()
        self.top_cmd_vals.clear()
        self.top_meas_vals.clear()
        self.bot_a_vals.clear()
        self.bot_b_vals.clear()

        self._real_data_t0 = None
        self._exo_t0 = None
        self._exo_prev_raw = None
        self._exo_wrap_offset = 0.0
        self.t0 = time.time()

        self.curve_top_cmd.setData([], [])
        self.curve_top_meas.setData([], [])
        self.curve_bot_a.setData([], [])
        self.curve_bot_b.setData([], [])

    def set_channel_labels(self, param_names: list):
        """Update plot labels with dynamic parameter names from device handshake."""
        try:
            self._param_names = list(param_names) if param_names else []
            # Resolve the exo-time channel used for the plot x-axis (see _x_for_sample).
            try:
                self._exo_time_idx = self._param_names.index("Exoskeleton time (seconds)")
            except ValueError:
                self._exo_time_idx = None
            # Update labels for current block
            self._update_labels()
        except Exception:
            pass

    def _update_labels(self):
        """Update plot titles and legend names for the current page (see _PLOT_PAGES).

        Also clears any curve the page does not use, so a 2-channel page leaves one clean trace
        per panel instead of a stale one from the previous page.
        """
        slots = self._page_slots()

        def channel_name(ch):
            if ch is None:
                return None
            if ch < len(self._param_names) and self._param_names[ch]:
                return self._param_names[ch]
            return f"Ch{ch}"

        def title(a, b):
            return f"{a} vs {b}" if (a and b) else (a or b or "")

        try:
            names = [channel_name(ch) for ch in slots]

            for plot, curves, pair in (
                (self.plot_top, (self.curve_top_cmd, self.curve_top_meas), (0, 1)),
                (self.plot_bottom, (self.curve_bot_a, self.curve_bot_b), (2, 3)),
            ):
                plot.setTitle(title(names[pair[0]], names[pair[1]]))
                plot.legend.clear()
                for curve, idx in zip(curves, pair):
                    if names[idx] is None:
                        curve.setData([], [])       # unused on this page
                        continue
                    curve.opts['name'] = names[idx]
                    plot.legend.addItem(curve, names[idx])
        except Exception:
            pass

    def _on_pause_play_clicked(self):
        """Toggle between pause and play states."""
        if not self.is_paused:
            # Pause
            self.is_paused = True
            self.btn_pause_play.setText("Play")
            self.deviceStopRequested.emit()  # Stop motors
        else:
            # Play
            self.is_paused = False
            self.btn_pause_play.setText("⏸ Pause")
            self.deviceStartRequested.emit()  # Start motors

    def _on_set_preamble_clicked(self):
        """Show dialog to set CSV filename preamble."""
        text, ok = QtWidgets.QInputDialog.getText(
            self,
            "Set CSV Filename Prefix + Save new CSV",
            "Enter prefix for CSV filenames (e.g., 'OutdoorShod'):",
            QtWidgets.QLineEdit.Normal,
            ""
        )
        if ok and text:
            # Sanitize filename
            text = "".join(c for c in text if c.isalnum() or c in ('_', '-'))
            self.csvPreambleChanged.emit(text)
            QtWidgets.QMessageBox.information(
                self,
                "CSV Prefix Set",
                f"CSV files will be saved as:\n{text}_trial_YYYYMMDD_HHMMSS.csv"
            )

    # Public API to integrate later with bridges
    def start_sim(self):
        if not self.timer.isActive():
            self.t0 = time.time()
            self._real_data_t0 = None  # Reset real data timing when starting sim
            self.timer.start(int(1000 / self.rate_hz))

    def stop_sim(self):
        if self.timer.isActive():
            self.timer.stop()
        # Reset real data timing when switching from sim to real data
        self._real_data_t0 = None

    def _page_slots(self) -> tuple:
        """Channel index for each of the four fixed curves, None where a curve is unused.

        Order is (top curve A, top curve B, bottom curve A, bottom curve B). A 2-channel page
        puts one channel alone on each panel, which is what Commanded Torque needs -- overlaying
        the two legs on a shared axis hides exactly the left/right difference we care about.
        """
        page = self._PLOT_PAGES[self._block_index % len(self._PLOT_PAGES)]
        if len(page) == 4:
            return (page[0], page[1], page[2], page[3])
        return (page[0], None, page[1], None)   # 2-channel page: one curve per panel

    def _page_label(self, block_index: int) -> str:
        page = self._PLOT_PAGES[block_index % len(self._PLOT_PAGES)]
        return f"{page[0]}-{page[-1]}" if len(page) > 1 else str(page[0])

    def _toggle_points(self):
        # Cycle through the pages. This used to flip between only two 4-channel blocks (0-3 and
        # 4-7), which left every channel from 8 up unreachable.
        self._block_index = (self._block_index + 1) % len(self._PLOT_PAGES)
        self._clear_plot_buffers()
        self._update_labels()
        # Button names the page it will switch TO, matching the previous behaviour.
        nxt = (self._block_index + 1) % len(self._PLOT_PAGES)
        self.btn_toggle_points.setText(f"Show Data {self._page_label(nxt)}")

    def _x_for_sample(self, values: list) -> float:
        """X-axis value (seconds) for an incoming real-time sample.

        Prefer the exo's own timestamp channel ("Exoskeleton time (seconds)"), which is
        evenly spaced at the control cadence, over wall-clock arrival time. BLE hands the
        GUI samples in bursts separated by gaps, so a wall-clock x-axis clumps points and
        makes the trace look jerky even though the underlying data is a steady ~100Hz.
        Falls back to wall-clock when no exo-time channel is present (e.g. hip/arm configs).

        NOTE: BioFeedbackPage.apply_values still uses raw wall-clock time and was left that
        way on purpose (out of scope when this was fixed). If that plot ever looks jerky,
        this method is the fix to port over.
        """
        idx = self._exo_time_idx
        if idx is not None and idx < len(values):
            t_exo = values[idx]
            # Undo the signed-16-bit fixed-point wrap of the exo clock. Every ~655.36s
            # the raw value drops by a full int16 span (e.g. +327.67 -> -327.68); left
            # alone this made the x-axis snap back to 0 for a few seconds while the stale
            # window flushed. A drop larger than half the span is a wrap, not real time
            # moving backward, so bump a cumulative offset to keep the clock monotonic.
            if self._exo_prev_raw is not None and (t_exo - self._exo_prev_raw) < -_EXO_TIME_WRAP_HALF:
                self._exo_wrap_offset += _EXO_TIME_WRAP_SPAN
            self._exo_prev_raw = t_exo
            t_exo_unwrapped = t_exo + self._exo_wrap_offset

            if self._exo_t0 is None:
                self._exo_t0 = t_exo_unwrapped
            t = t_exo_unwrapped - self._exo_t0
            if t < 0:  # genuine exo clock reset (reboot) -> re-anchor
                self._exo_t0 = t_exo_unwrapped
                t = 0.0
            return t
        # Fallback: wall-clock arrival time
        if self._real_data_t0 is None:
            self._real_data_t0 = time.time()
        return time.time() - self._real_data_t0

    def apply_values(self, values: list):
        """Update plots from incoming rtDataUpdated(values).

        Which channels are shown depends on the current page - see _PLOT_PAGES.
        """
        if not values:
            return
        slots = self._page_slots()
        # Every channel this page needs must be present, or the curves would desynchronise
        # from t_vals (a curve that skips a sample can never realign).
        if any(ch is not None and ch >= len(values) for ch in slots):
            return
        # X-axis from the exo timestamp (steady cadence) rather than bursty BLE arrival.
        t_next = self._x_for_sample(values)
        self.t_vals.append(t_next)
        # Unused curves (None) are left cleared by _update_labels; skipping them here keeps
        # their deque empty so it never gets out of step with t_vals.
        for buf, curve, ch in (
            (self.top_cmd_vals,  self.curve_top_cmd,  slots[0]),
            (self.top_meas_vals, self.curve_top_meas, slots[1]),
            (self.bot_a_vals,    self.curve_bot_a,    slots[2]),
            (self.bot_b_vals,    self.curve_bot_b,    slots[3]),
        ):
            if ch is None:
                continue
            buf.append(values[ch])
            curve.setData(self.t_vals, buf)

    # Update callback
    def _on_tick(self):
        t = time.time() - self.t0
        # Simulate smooth signals (replace later with real-time data)
        # Top: controller is a sine, measurement lags and has noise
        top_cmd = 0.7 * math.sin(2 * math.pi * 0.5 * t)
        top_meas = 0.7 * math.sin(2 * math.pi * 0.5 * (t - 0.05)) + 0.05 * math.sin(2 * math.pi * 4 * t)
        # Bottom: signals with different frequencies
        bot_a = 0.5 * math.sin(2 * math.pi * 0.2 * t)
        bot_b = 0.5 * math.cos(2 * math.pi * 0.35 * t)

        self.t_vals.append(t)
        self.top_cmd_vals.append(top_cmd)
        self.top_meas_vals.append(top_meas)
        self.bot_a_vals.append(bot_a)
        self.bot_b_vals.append(bot_b)

        # Update curves (x as t in seconds)
        # For speed: setData with lists (pyqtgraph accepts list/ndarray)
        self.curve_top_cmd.setData(self.t_vals, self.top_cmd_vals)
        self.curve_top_meas.setData(self.t_vals, self.top_meas_vals)
        self.curve_bot_a.setData(self.t_vals, self.bot_a_vals)
        self.curve_bot_b.setData(self.t_vals, self.bot_b_vals)


# Standalone demo
def _demo():
    app = QtWidgets.QApplication(sys.argv)
    w = ActiveTrialPage()
    w.resize(900, 600)
    w.show()
    w.start_sim()
    sys.exit(app.exec())


if __name__ == "__main__":
    _demo()
