"""SplineAlt tab of the spline designer: set the shape parameters (per-lobe peak
torque, peak timing, rise / dwell / fall) and preview the periodic torque curve
the SplineAlt controller would build from them, then export a splineAlt.csv
matching SDCard/ankleControllers/splineAlt.csv.

The curve here is generated, not drawn: there are no draggable nodes. The plot
shows the built node set as markers so it is clear where the shape parameters
land. Run standalone with: python spline_alt_designer.py
"""

import os
import sys

from PySide6 import QtWidgets
import pyqtgraph as pg

from spline_alt_csv import load_spline_alt_csv, save_spline_alt_csv
from spline_alt_math import NUM_PARAMS, PARAM_BOUNDS
from spline_alt_state import SplineAltState, X_BOUNDS

# The committed SDCard/ankleControllers/splineAlt.csv values.
DEFAULT_PARAMS = [15.0, 5.0, 50.0, 95.0, 26.0, 0.0, 17.0, 11.0, 0.0, 9.0,
                  100.0, 0.0, 1.0, 1.0, 3.0, 0.0, 0.01]

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_SPLINEALT_PATH = os.path.join(
    REPO_ROOT, "SDCard", "ankleControllers", "splineAlt.csv"
)

_Y_RANGE_PADDING_FRACTION = 0.1
_MIN_Y_SPAN = 4.0  # Nm, keeps the range from collapsing when the curve is flat

# (section title, [ (label, param index, decimals, single step) ... ])
_PARAM_SECTIONS = [
    ("Plantarflexion Lobe", [
        ("Peak torque (Nm, positive)", 0, 1, 1.0),
        ("Peak time (% gait)", 2, 1, 1.0),
        ("Rise (% gait)", 4, 1, 1.0),
        ("Dwell (% gait, 0 = none)", 5, 1, 1.0),
        ("Fall (% gait)", 6, 1, 1.0),
    ]),
    ("Dorsiflexion Lobe", [
        ("Peak torque (Nm, positive)", 1, 1, 1.0),
        ("Peak time (% gait)", 3, 1, 1.0),
        ("Rise (% gait)", 7, 1, 1.0),
        ("Dwell (% gait, 0 = none)", 8, 1, 1.0),
        ("Fall (% gait)", 9, 1, 1.0),
    ]),
    ("Global", [
        ("Torque scale (%)", 10, 1, 5.0),
    ]),
    ("Simulation & PID", [
        ("Simulate %gait (1 = yes)", 11, 0, 1.0),
        ("1 = %gait, 0 = %stance", 12, 0, 1.0),
        ("PID flag (1 = on)", 13, 0, 1.0),
        ("P Gain", 14, 3, 0.1),
        ("I Gain", 15, 3, 0.1),
        ("D Gain", 16, 3, 0.001),
    ]),
]


class SplineAltDesignerWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Ankle SplineAlt Designer")
        self.resize(1100, 650)

        self.state = SplineAltState(list(DEFAULT_PARAMS))
        self._updating = False  # guards against feedback loops while syncing

        self._build_ui()
        self._sync_all_from_state()

    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QHBoxLayout(central)

        plot_col = QtWidgets.QVBoxLayout()

        toolbar = QtWidgets.QHBoxLayout()
        btn_open = QtWidgets.QPushButton("Open...")
        btn_open.clicked.connect(self._on_open)
        btn_save = QtWidgets.QPushButton("Save As...")
        btn_save.clicked.connect(self._on_save_as)
        toolbar.addWidget(btn_open)
        toolbar.addWidget(btn_save)
        toolbar.addStretch(1)
        plot_col.addLayout(toolbar)

        self.plot = pg.PlotWidget()
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.plot.setLabel("bottom", "Percent Gait Cycle", units="%")
        self.plot.setLabel("left", "Torque", units="Nm")
        # The profile is periodic over one gait cycle; pin the view to 0-100%.
        self.plot.setLimits(xMin=X_BOUNDS[0], xMax=X_BOUNDS[1])
        self.plot.setXRange(*X_BOUNDS, padding=0)
        self.curve = self.plot.plot(pen=pg.mkPen(color="#0078D4", width=2))
        self.node_scatter = pg.ScatterPlotItem(
            size=9, pen=pg.mkPen("#0078D4"), brush=pg.mkBrush(255, 255, 255)
        )
        self.plot.addItem(self.node_scatter)
        plot_col.addWidget(self.plot, 1)

        self.status_label = QtWidgets.QLabel("")
        self.status_label.setWordWrap(True)
        plot_col.addWidget(self.status_label)

        layout.addLayout(plot_col, 2)

        param_col = QtWidgets.QVBoxLayout()
        param_col.addWidget(QtWidgets.QLabel("Shape Parameters"))
        self._spins = [None] * NUM_PARAMS
        for title, rows in _PARAM_SECTIONS:
            heading = QtWidgets.QLabel(title)
            font = heading.font()
            font.setBold(True)
            heading.setFont(font)
            param_col.addSpacing(6)
            param_col.addWidget(heading)

            grid = QtWidgets.QGridLayout()
            for row, (label, idx, decimals, step) in enumerate(rows):
                grid.addWidget(QtWidgets.QLabel(label), row, 0)
                lo, hi, _integer_only = PARAM_BOUNDS[idx]
                spin = QtWidgets.QDoubleSpinBox()
                spin.setDecimals(decimals)
                spin.setRange(lo, hi)
                spin.setSingleStep(step)
                # Commit only when the edit is finished (Enter/focus-out/arrows),
                # not on every keystroke -- matches the node designer's spin boxes.
                spin.setKeyboardTracking(False)
                spin.valueChanged.connect(
                    lambda val, i=idx: self._on_spin_changed(i, val)
                )
                grid.addWidget(spin, row, 1)
                self._spins[idx] = spin
            param_col.addLayout(grid)

        note = QtWidgets.QLabel(
            "Torque magnitudes are entered positive; plantarflexion is applied "
            "negative internally. The firmware clamps the feed-forward command "
            "to ±15 Nm."
        )
        note.setWordWrap(True)
        param_col.addSpacing(6)
        param_col.addWidget(note)
        param_col.addStretch(1)

        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        param_widget = QtWidgets.QWidget()
        param_widget.setLayout(param_col)
        scroll.setWidget(param_widget)
        layout.addWidget(scroll, 1)

    def _sync_all_from_state(self):
        self._updating = True
        try:
            for i in range(NUM_PARAMS):
                self._spins[i].setValue(self.state.params[i])
        finally:
            self._updating = False
        self._redraw_curve()

    def _redraw_curve(self):
        xs, ys, valid = self.state.curve_samples()
        self.curve.setData(xs, ys)
        node_x, node_y = self.state.built_nodes()
        self.node_scatter.setData(node_x, node_y)
        self._update_y_range(ys + node_y)

        if valid:
            self.status_label.setText(
                f"Built {len(node_x)} nodes "
                f"({len(node_x) + 4} with the periodic wrap-around copies)."
            )
        else:
            self.status_label.setText(
                "Degenerate profile — the firmware would command zero torque. "
                "Every active lobe needs rise > 0 and fall > 0, and the two lobes "
                "must not share a node position."
            )

    def _update_y_range(self, y_values):
        """Rescale the Y axis to fit y_values, padded and centered so negative
        (plantarflexion) torque shows as readily as positive."""
        if not y_values:
            self.plot.setYRange(-_MIN_Y_SPAN, _MIN_Y_SPAN, padding=0)
            return
        y_min, y_max = min(y_values), max(y_values)
        span = max(y_max - y_min, _MIN_Y_SPAN)
        pad = span * _Y_RANGE_PADDING_FRACTION
        mid = (y_min + y_max) / 2.0
        half = span / 2.0 + pad
        self.plot.setYRange(mid - half, mid + half, padding=0)

    def _on_spin_changed(self, index, value):
        if self._updating:
            return
        self.state.set_param(index, value)
        # Re-sync so a value clamped against spline_alt_bounds is reflected back
        # into the spin box, mirroring the node designer's behaviour.
        self._sync_all_from_state()

    def _on_open(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Open splineAlt.csv", DEFAULT_SPLINEALT_PATH, "CSV files (*.csv)"
        )
        if not path:
            return
        try:
            params = load_spline_alt_csv(path)
        except ValueError as e:
            QtWidgets.QMessageBox.critical(self, "Failed to open file", str(e))
            return
        self.state = SplineAltState(params)
        self._sync_all_from_state()
        self.status_label.setText(f"Loaded {path}")

    def _on_save_as(self):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Save splineAlt.csv", DEFAULT_SPLINEALT_PATH, "CSV files (*.csv)"
        )
        if not path:
            return
        try:
            save_spline_alt_csv(path, self.state.params)
        except OSError as e:
            QtWidgets.QMessageBox.critical(self, "Failed to save file", str(e))
            return
        self.status_label.setText(f"Saved to {path}")


def main():
    app = QtWidgets.QApplication(sys.argv)
    window = SplineAltDesignerWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
