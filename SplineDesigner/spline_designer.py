"""Standalone ankle spline designer: drag nodes on the plot or type their
values, then export a spline.csv matching SDCard/ankleControllers/spline.csv.

Run with: python spline_designer.py
"""

import os
import sys

from PySide6 import QtWidgets
import pyqtgraph as pg

from node_state import NodeState
from spline_alt_designer import SplineAltDesignerWindow
from spline_csv import NUM_NODES, load_spline_csv, save_spline_csv
from spline_math import NODE_X_BOUNDS, NODE_Y_BOUNDS

DEFAULT_X = [0.0, 5.0, 10.0, 20.0, 20.1, 20.2, 20.3, 20.4, 20.5, 20.6, 20.7, 20.8]
DEFAULT_Y = [0.0, -8.0, -12.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
DEFAULT_EXTRA = [0.0, 1.0, 1.0, 6.0, 0.0, 0.03]
EXTRA_LABELS = ["sim %gait (1=yes)", "1=%gait 0=%stance", "PID Flag", "P Gain", "I Gain", "D Gain"]

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_SPLINE_PATH = os.path.join(REPO_ROOT, "SDCard", "ankleControllers", "spline.csv")

_Y_RANGE_PADDING_FRACTION = 0.1
_MIN_Y_SPAN = 4.0  # Nm, keeps the range from collapsing when all values are equal


class SplineDesignerWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Ankle Spline Designer")
        self.resize(1100, 650)

        self.state = NodeState(list(DEFAULT_X), list(DEFAULT_Y), list(DEFAULT_EXTRA))
        self._updating = False  # guards against feedback loops while syncing widgets from state

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
        # Keep the view pinned to the valid gait-cycle span: no panning or
        # zooming the X axis outside 0-100%.
        self.plot.setLimits(xMin=NODE_X_BOUNDS[0], xMax=NODE_X_BOUNDS[1])
        self.plot.setXRange(*NODE_X_BOUNDS, padding=0)
        self.curve = self.plot.plot(pen=pg.mkPen(color="#0078D4", width=2))
        plot_col.addWidget(self.plot, 1)

        self.status_label = QtWidgets.QLabel("")
        plot_col.addWidget(self.status_label)

        layout.addLayout(plot_col, 2)

        self._targets = []
        for i in range(NUM_NODES):
            target = pg.TargetItem(pos=(self.state.x_nodes[i], self.state.y_nodes[i]), movable=True)
            target.setLabel(f"{i + 1}")
            target.sigPositionChanged.connect(lambda t, idx=i: self._on_target_moved(idx, t))
            self.plot.addItem(target)
            self._targets.append(target)

        node_col = QtWidgets.QVBoxLayout()
        node_col.addWidget(QtWidgets.QLabel("Nodes"))
        grid = QtWidgets.QGridLayout()
        grid.addWidget(QtWidgets.QLabel("X (% gait)"), 0, 1)
        grid.addWidget(QtWidgets.QLabel("Y (Nm)"), 0, 2)
        self._x_spins = []
        self._y_spins = []
        for i in range(NUM_NODES):
            grid.addWidget(QtWidgets.QLabel(f"Node {i + 1}"), i + 1, 0)

            spin_x = QtWidgets.QDoubleSpinBox()
            spin_x.setDecimals(2)
            spin_x.setRange(*NODE_X_BOUNDS)
            spin_x.setSingleStep(0.5)
            # Commit only when the edit is finished (Enter/focus-out/arrows), not on
            # every keystroke -- otherwise a half-typed X is clamped against the
            # neighbour node and written back before the user can finish typing.
            spin_x.setKeyboardTracking(False)
            spin_x.valueChanged.connect(lambda val, idx=i: self._on_x_spin_changed(idx, val))
            grid.addWidget(spin_x, i + 1, 1)
            self._x_spins.append(spin_x)

            spin_y = QtWidgets.QDoubleSpinBox()
            spin_y.setDecimals(2)
            spin_y.setRange(*NODE_Y_BOUNDS)
            spin_y.setSingleStep(0.5)
            spin_y.setKeyboardTracking(False)
            spin_y.valueChanged.connect(lambda val, idx=i: self._on_y_spin_changed(idx, val))
            grid.addWidget(spin_y, i + 1, 2)
            self._y_spins.append(spin_y)
        node_col.addLayout(grid)

        node_col.addWidget(QtWidgets.QLabel("Extra Parameters"))
        extra_grid = QtWidgets.QGridLayout()
        self._extra_spins = []
        for i, label in enumerate(EXTRA_LABELS):
            extra_grid.addWidget(QtWidgets.QLabel(label), i, 0)
            spin = QtWidgets.QDoubleSpinBox()
            spin.setDecimals(3)
            spin.setRange(-1000.0, 1000.0)
            spin.setKeyboardTracking(False)
            spin.valueChanged.connect(lambda val, idx=i: self._on_extra_spin_changed(idx, val))
            extra_grid.addWidget(spin, i, 1)
            self._extra_spins.append(spin)
        node_col.addLayout(extra_grid)
        node_col.addStretch(1)

        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        node_widget = QtWidgets.QWidget()
        node_widget.setLayout(node_col)
        scroll.setWidget(node_widget)
        layout.addWidget(scroll, 1)

    def _sync_all_from_state(self):
        self._updating = True
        try:
            for i in range(NUM_NODES):
                self._x_spins[i].setValue(self.state.x_nodes[i])
                self._y_spins[i].setValue(self.state.y_nodes[i])
                self._targets[i].setPos(self.state.x_nodes[i], self.state.y_nodes[i])
            for i in range(6):
                self._extra_spins[i].setValue(self.state.extra_params[i])
        finally:
            self._updating = False
        self._redraw_curve()

    def _redraw_curve(self):
        xs, ys, valid = self.state.curve_samples()
        self.curve.setData(xs, ys)
        self._update_y_range(ys + self.state.y_nodes)
        self.status_label.setText("" if valid else "Invalid node order (should be unreachable via this UI)")

    def _update_y_range(self, y_values):
        """Rescale the plot's Y axis to fit y_values, padded and centered on
        the data so negative torques are shown as readily as positive ones."""
        if not y_values:
            self.plot.setYRange(*NODE_Y_BOUNDS, padding=0)
            return
        y_min, y_max = min(y_values), max(y_values)
        span = max(y_max - y_min, _MIN_Y_SPAN)
        pad = span * _Y_RANGE_PADDING_FRACTION
        mid = (y_min + y_max) / 2.0
        half = span / 2.0 + pad
        self.plot.setYRange(mid - half, mid + half, padding=0)

    def _on_target_moved(self, index, target):
        if self._updating:
            return
        pos = target.pos()
        # Dragging snaps to whole % gait / Nm; typing into the spin boxes still
        # accepts decimals. _sync_all_from_state writes the snapped value back to
        # the target, so the node visibly lands on the integer grid.
        self.state.set_x(index, round(pos.x()))
        self.state.set_y(index, round(pos.y()))
        self._sync_all_from_state()

    def _on_x_spin_changed(self, index, value):
        if self._updating:
            return
        self.state.set_x(index, value)
        self._sync_all_from_state()

    def _on_y_spin_changed(self, index, value):
        if self._updating:
            return
        self.state.set_y(index, value)
        self._sync_all_from_state()

    def _on_extra_spin_changed(self, index, value):
        if self._updating:
            return
        self.state.extra_params[index] = value

    def _on_open(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Open spline.csv", DEFAULT_SPLINE_PATH, "CSV files (*.csv)"
        )
        if not path:
            return
        try:
            x_nodes, y_nodes, extra_params = load_spline_csv(path)
        except ValueError as e:
            QtWidgets.QMessageBox.critical(self, "Failed to open file", str(e))
            return
        self.state = NodeState(x_nodes, y_nodes, extra_params)
        self._sync_all_from_state()
        self.status_label.setText(f"Loaded {path}")

    def _on_save_as(self):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Save spline.csv", DEFAULT_SPLINE_PATH, "CSV files (*.csv)"
        )
        if not path:
            return
        try:
            save_spline_csv(path, self.state.x_nodes, self.state.y_nodes, self.state.extra_params)
        except OSError as e:
            QtWidgets.QMessageBox.critical(self, "Failed to save file", str(e))
            return
        self.status_label.setText(f"Saved to {path}")


def main():
    app = QtWidgets.QApplication(sys.argv)
    # Two designers behind a tab bar: the original 12-node Spline designer and
    # the shape-parameterised SplineAlt designer. SplineDesignerWindow is a
    # QMainWindow, which nests fine as a tab page; its own window title/size are
    # simply ignored while embedded.
    tabs = QtWidgets.QTabWidget()
    tabs.setWindowTitle("Ankle Spline Designer")
    tabs.resize(1150, 700)
    tabs.addTab(SplineDesignerWindow(), "Spline (12 nodes)")
    tabs.addTab(SplineAltDesignerWindow(), "SplineAlt (shape)")
    tabs.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
