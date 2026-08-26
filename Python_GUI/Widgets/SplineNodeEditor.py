try:
    from PySide6 import QtCore, QtWidgets
    import pyqtgraph as pg
except Exception as e:
    raise SystemExit("PySide6 and pyqtgraph are required")

from utils import UIConfig, style_spinbox, create_section_label
from utils.spline_math import pchip_curve, NODE_X_BOUNDS, NODE_Y_BOUNDS

# Fixed X display range (percent gait cycle is always 0-100%).
# Y range is recomputed on every redraw from the current node/curve values instead.
PLOT_X_RANGE = (0.0, 100.0)   # percent gait cycle
_DEFAULT_Y_RANGE = (-10.0, 10.0)  # Nm, shown before any nodes are configured
_Y_RANGE_PADDING_FRACTION = 0.1
_MIN_Y_SPAN = 4.0  # Nm, keeps the range from collapsing when all values are equal

_CURVE_SAMPLE_COUNT = 200


class SplineNodeEditor(QtWidgets.QWidget):
    """Live-updating spline preview plot (left) + node X/Y spinboxes (right).

    The node count is set via configure() rather than fixed at construction,
    since different joints' spline.csv configs currently define different
    node counts (e.g. ankle has 12 nodes, hip/arm still have 5) - the widget
    must not assume a fixed count.
    """

    nodesChanged = QtCore.Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._x_nodes = []
        self._y_nodes = []
        self._x_spins = []
        self._y_spins = []
        self._build_ui()

    def _build_ui(self):
        layout = QtWidgets.QHBoxLayout(self)
        layout.setSpacing(UIConfig.SPACING_SECTION)

        plot_col = QtWidgets.QVBoxLayout()
        self.plot = pg.PlotWidget()
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.plot.setLabel("bottom", "Percent Gait Cycle", units="%")
        self.plot.setLabel("left", "Torque", units="Nm")
        self.plot.setXRange(*PLOT_X_RANGE, padding=0)
        self.plot.setYRange(*_DEFAULT_Y_RANGE, padding=0)
        self.plot.setMouseEnabled(x=False, y=False)
        self.curve = self.plot.plot(pen=pg.mkPen(color="#0078D4", width=2))
        self.node_scatter = pg.ScatterPlotItem(
            size=11, brush=pg.mkBrush("#FF9800"), pen=pg.mkPen("#1E1E1E", width=1)
        )
        self.plot.addItem(self.node_scatter)
        plot_col.addWidget(self.plot, 1)

        self.lbl_warning = QtWidgets.QLabel("")
        self.lbl_warning.setWordWrap(True)
        self.lbl_warning.setStyleSheet(
            f"font-size: {UIConfig.FONT_TINY}pt; color: {UIConfig.COLOR_PARAM_REJECT}; font-weight: bold;"
        )
        plot_col.addWidget(self.lbl_warning)
        layout.addLayout(plot_col, 2)

        node_col = QtWidgets.QVBoxLayout()
        node_col.addWidget(create_section_label("Spline Nodes"))

        self._grid_container = QtWidgets.QWidget()
        self._grid = QtWidgets.QGridLayout(self._grid_container)
        self._grid.setHorizontalSpacing(UIConfig.SPACING_XLARGE)
        self._grid.setVerticalSpacing(UIConfig.SPACING_LARGE)

        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(self._grid_container)
        scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        node_col.addWidget(scroll, 1)

        layout.addLayout(node_col, 1)

    def configure(self, node_count: int, x_nodes=None, y_nodes=None):
        """(Re)build the node spinbox grid for node_count nodes.

        x_nodes/y_nodes seed the initial values (defaults to 0.0 for any node
        not provided). Existing spinboxes are discarded and rebuilt since the
        node count varies by joint/controller config.
        """
        while self._grid.count():
            item = self._grid.takeAt(0)
            w = item.widget()
            if w is not None:
                w.deleteLater()
        self._x_spins = []
        self._y_spins = []

        x_nodes = list(x_nodes) if x_nodes else [0.0] * node_count
        y_nodes = list(y_nodes) if y_nodes else [0.0] * node_count
        x_nodes = (x_nodes + [0.0] * node_count)[:node_count]
        y_nodes = (y_nodes + [0.0] * node_count)[:node_count]
        self._x_nodes = x_nodes
        self._y_nodes = y_nodes

        hdr_x = QtWidgets.QLabel("X (% gait)")
        hdr_y = QtWidgets.QLabel("Y (Nm)")
        for hdr in (hdr_x, hdr_y):
            f = hdr.font(); f.setPointSize(UIConfig.FONT_SMALL); f.setBold(True); hdr.setFont(f)
        self._grid.addWidget(hdr_x, 0, 1)
        self._grid.addWidget(hdr_y, 0, 2)

        for i in range(node_count):
            row = i + 1
            lbl = QtWidgets.QLabel(f"Node {i + 1}")
            lf = lbl.font(); lf.setPointSize(UIConfig.FONT_MEDIUM); lbl.setFont(lf)
            self._grid.addWidget(lbl, row, 0)

            spin_x = QtWidgets.QDoubleSpinBox()
            spin_x.setDecimals(1)
            spin_x.setRange(*NODE_X_BOUNDS)
            spin_x.setSingleStep(0.5)
            spin_x.setValue(self._x_nodes[i])
            style_spinbox(spin_x, height=UIConfig.BTN_HEIGHT_MEDIUM, font_size=UIConfig.FONT_MEDIUM)
            spin_x.valueChanged.connect(lambda val, idx=i: self._on_node_edited(idx, val, is_x=True))
            self._grid.addWidget(spin_x, row, 1)
            self._x_spins.append(spin_x)

            spin_y = QtWidgets.QDoubleSpinBox()
            spin_y.setDecimals(1)
            spin_y.setRange(*NODE_Y_BOUNDS)
            spin_y.setSingleStep(0.5)
            spin_y.setValue(self._y_nodes[i])
            style_spinbox(spin_y, height=UIConfig.BTN_HEIGHT_MEDIUM, font_size=UIConfig.FONT_MEDIUM)
            spin_y.valueChanged.connect(lambda val, idx=i: self._on_node_edited(idx, val, is_x=False))
            self._grid.addWidget(spin_y, row, 2)
            self._y_spins.append(spin_y)

        self._redraw()

    def get_nodes(self):
        """Return (x_nodes, y_nodes) as plain float lists."""
        return list(self._x_nodes), list(self._y_nodes)

    def node_count(self) -> int:
        return len(self._x_nodes)

    def _on_node_edited(self, idx, value, is_x):
        if is_x:
            self._x_nodes[idx] = value
        else:
            self._y_nodes[idx] = value
        self._redraw()
        self.nodesChanged.emit()

    def _update_y_range(self, y_values):
        """Rescale the plot's Y axis to fit y_values, padded and centered on
        the data so negative torques are shown as readily as positive ones.
        """
        if not y_values:
            self.plot.setYRange(*_DEFAULT_Y_RANGE, padding=0)
            return

        y_min, y_max = min(y_values), max(y_values)
        span = max(y_max - y_min, _MIN_Y_SPAN)
        pad = span * _Y_RANGE_PADDING_FRACTION
        mid = (y_min + y_max) / 2.0
        half = span / 2.0 + pad
        self.plot.setYRange(mid - half, mid + half, padding=0)

    def _redraw(self):
        if not self._x_nodes:
            self.curve.setData([], [])
            self.node_scatter.setData([], [])
            self.lbl_warning.setText("")
            self._update_y_range([])
            return

        samples_x = [
            PLOT_X_RANGE[0] + i * (PLOT_X_RANGE[1] - PLOT_X_RANGE[0]) / (_CURVE_SAMPLE_COUNT - 1)
            for i in range(_CURVE_SAMPLE_COUNT)
        ]
        samples_y, valid = pchip_curve(self._x_nodes, self._y_nodes, samples_x)
        self.curve.setData(samples_x, samples_y)
        self.node_scatter.setData(self._x_nodes, self._y_nodes)
        self._update_y_range(samples_y + self._y_nodes)

        if not valid:
            self.lbl_warning.setText(
                "Invalid node order: X values must strictly increase from Node 1 to the last node. "
                "Firmware would command 0 Nm for this configuration."
            )
        else:
            self.lbl_warning.setText("")
