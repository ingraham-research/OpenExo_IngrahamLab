"""Example: open the spline node editor with no exo connected.

ActiveTrialSettingsPage only shows the spline editor once it has a
controller matrix and values to render, which normally arrive from a live
device. This preloads fake data for a 12-node ankle spline controller so
the page can be exercised standalone. From the Python_GUI folder:

    python examples/spline_demo.py
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from PySide6 import QtWidgets  # noqa: E402
from pages.ActiveTrialSettingsPage import ActiveTrialSettingsPage  # noqa: E402


def _node_param_names(node_count):
    names = []
    for n in range(1, node_count + 1):
        names.append(f"node{n}_x")
        names.append(f"node{n}_y")
    names += ["sim_gait", "use_percent_gait", "use_pid", "p_gain", "i_gain", "d_gain"]
    return names


def _values_for(node_count, x_step=8.0, y_step=1.0):
    values = []
    for n in range(1, node_count + 1):
        values.append(str(n * x_step))
        values.append(str(n * y_step))
    values += ["1", "1", "0", "5", "0", "0.01"]
    return values


def main():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)

    page = ActiveTrialSettingsPage()
    page.setWindowTitle("Spline Editor Demo (no device)")

    node_count = 12
    row = ["Ankle(L) (68)", "68", "spline", "1", *_node_param_names(node_count)]
    page.set_controller_matrix([row])
    page.set_controller_values({("68", "1"): _values_for(node_count)})

    page.resize(900, 700)
    page.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
