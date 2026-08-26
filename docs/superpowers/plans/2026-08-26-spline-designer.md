# Standalone Ankle Spline Designer Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A standalone PySide6/pyqtgraph desktop app (`SplineDesigner/`), fully decoupled from `Python_GUI/`, for drag-or-type editing of the ankle Spline controller's 12-node profile and exporting a `spline.csv` in the exact format `SDCard/ankleControllers/spline.csv` uses.

**Architecture:** Three pure-Python modules with no Qt dependency (`spline_math.py` — PCHIP interpolation ported from the firmware; `spline_csv.py` — load/save for the 30-parameter ankle CSV format; `node_state.py` — in-memory node state with drag/type clamping) plus one PySide6 UI module (`spline_designer.py`) that wires them into a window. The pure modules are unit tested; the UI is verified manually.

**Tech Stack:** Python 3, PySide6, pyqtgraph (`pg.TargetItem` for draggable nodes — confirmed available in this environment, pyqtgraph 0.14.0), pytest.

**Spec:** `docs/superpowers/specs/2026-08-26-spline-designer-design.md`

## Global Constraints

- Ankle only: 12 nodes, 30 parameters (24 node values + 6 trailing values). No hip/arm (5-node/16-param) support.
- No BLE, no exo connection, no dependency on `Python_GUI/` — `SplineDesigner/` is fully standalone; `spline_math.py` is a duplicated copy, not an import.
- Node bounds: `NODE_X_BOUNDS = (0.0, 100.0)` (% gait), `NODE_Y_BOUNDS = (-100.0, 100.0)` (Nm).
- X-ordering: firmware requires strictly increasing X; this tool enforces it structurally via clamping (drag and type both clamp to `X_EPSILON = 0.1` inside neighbors), so no invalid/warning state is reachable through the UI.
- Format constants: `HEADER_ROW_COUNT = 5`, `PARAM_COUNT = 30`, `NUM_NODES = 12`.
- "Save As" always prompts for a path (never a silent overwrite); both Open and Save As default to `SDCard/ankleControllers/spline.csv`.

---

## Task 1: Project scaffolding + spline_math.py

**Files:**
- Create: `SplineDesigner/spline_math.py`
- Create: `SplineDesigner/requirements.txt`
- Create: `SplineDesigner/tests/__init__.py` (empty)
- Create: `SplineDesigner/tests/conftest.py`
- Test: `SplineDesigner/tests/test_spline_math.py`

**Interfaces:**
- Produces: `NODE_X_BOUNDS: tuple[float, float]`, `NODE_Y_BOUNDS: tuple[float, float]`, `nodes_strictly_increasing(x_nodes: list[float]) -> bool`, `pchip_curve(x_nodes: list[float], y_nodes: list[float], t_samples: list[float]) -> tuple[list[float], bool]`.

- [ ] **Step 1: Create the folder structure and dependency manifest**

```bash
mkdir -p SplineDesigner/tests
```

Create `SplineDesigner/requirements.txt`:

```
PySide6
pyqtgraph
pytest
```

Create `SplineDesigner/tests/__init__.py` (empty file).

Create `SplineDesigner/tests/conftest.py`:

```python
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
```

- [ ] **Step 2: Write the failing test**

Create `SplineDesigner/tests/test_spline_math.py`:

```python
import pytest

from spline_math import NODE_X_BOUNDS, NODE_Y_BOUNDS, nodes_strictly_increasing, pchip_curve


def test_bounds_match_firmware_config():
    assert NODE_X_BOUNDS == (0.0, 100.0)
    assert NODE_Y_BOUNDS == (-100.0, 100.0)


def test_nodes_strictly_increasing_true_for_sorted():
    assert nodes_strictly_increasing([0.0, 25.0, 50.0, 75.0, 100.0]) is True


def test_nodes_strictly_increasing_false_for_duplicate():
    assert nodes_strictly_increasing([0.0, 25.0, 25.0, 75.0, 100.0]) is False


def test_nodes_strictly_increasing_false_for_out_of_order():
    assert nodes_strictly_increasing([0.0, 50.0, 25.0, 75.0, 100.0]) is False


def test_pchip_curve_passes_through_nodes():
    x_nodes = [0.0, 25.0, 50.0, 75.0, 100.0]
    y_nodes = [0.0, 0.0, 20.0, 0.0, 0.0]
    values, valid = pchip_curve(x_nodes, y_nodes, x_nodes)
    assert valid is True
    for got, expected in zip(values, y_nodes):
        assert got == pytest.approx(expected, abs=1e-9)


def test_pchip_curve_invalid_for_non_increasing_x():
    x_nodes = [0.0, 50.0, 25.0, 75.0, 100.0]
    y_nodes = [0.0, 0.0, 20.0, 0.0, 0.0]
    t_samples = [0.0, 50.0, 100.0]
    values, valid = pchip_curve(x_nodes, y_nodes, t_samples)
    assert valid is False
    assert values == [0.0, 0.0, 0.0]
```

- [ ] **Step 3: Run test to verify it fails**

Run: `cd SplineDesigner && python -m pytest tests/test_spline_math.py -v`
Expected: FAIL / ERROR with `ModuleNotFoundError: No module named 'spline_math'`

- [ ] **Step 4: Write the implementation**

Create `SplineDesigner/spline_math.py` (verbatim port of `Python_GUI/utils/spline_math.py` — this file must stay hand-synced with the firmware's `ExoCode/src/Controller.cpp` interpolation, same as the exo GUI's copy):

```python
"""
Python port of the firmware's spline-controller interpolation
(ExoCode/src/Controller.cpp, Spline::_pchip_interpolate / _pchip_edge_tangent).

Kept in sync by hand with the firmware algorithm so this tool's preview plot matches
what the exoskeleton will actually command. If the firmware interpolation changes,
update this module to match. This is a deliberate duplicate of
Python_GUI/utils/spline_math.py — SplineDesigner has no dependency on Python_GUI, so
both copies must be updated together if the firmware algorithm ever changes.
"""

# Matches ExoCode/src/ControllerData.cpp spline_bounds (per-node bounds).
NODE_X_BOUNDS = (0.0, 100.0)   # percent gait cycle
NODE_Y_BOUNDS = (-100.0, 100.0)  # Nm


def _pchip_edge_tangent(h0, h1, m0, m1):
    """Port of Spline::_pchip_edge_tangent (Controller.cpp)."""
    d = ((2.0 * h0 + h1) * m0 - h0 * m1) / (h0 + h1)
    sign_d = d >= 0.0
    sign_m0 = m0 >= 0.0
    if sign_d != sign_m0:
        return 0.0
    sign_m1 = m1 >= 0.0
    if sign_m0 != sign_m1 and abs(d) > 3.0 * abs(m0):
        return 3.0 * m0
    return d


def nodes_strictly_increasing(x_nodes):
    """Port of the ordering guard at the top of Spline::_pchip_interpolate."""
    return all(x_nodes[i] > x_nodes[i - 1] for i in range(1, len(x_nodes)))


def pchip_curve(x_nodes, y_nodes, t_samples):
    """Port of Spline::_pchip_interpolate, evaluated at each of t_samples.

    Returns (y_values, valid). valid is False when x_nodes is not strictly
    increasing, matching the firmware's "return 0.0f for everything" behavior
    for an invalid/unordered node configuration.
    """
    n = len(x_nodes)
    if n < 3 or not nodes_strictly_increasing(x_nodes):
        return [0.0] * len(t_samples), False

    h = [x_nodes[i + 1] - x_nodes[i] for i in range(n - 1)]
    secant = [(y_nodes[i + 1] - y_nodes[i]) / h[i] for i in range(n - 1)]

    m = [0.0] * n
    for i in range(1, n - 1):
        m0, m1 = secant[i - 1], secant[i]
        if m0 == 0.0 or m1 == 0.0 or (m0 > 0.0) != (m1 > 0.0):
            m[i] = 0.0
        else:
            w1 = 2.0 * h[i] + h[i - 1]
            w2 = h[i] + 2.0 * h[i - 1]
            m[i] = (w1 + w2) / (w1 / m0 + w2 / m1)
    m[0] = _pchip_edge_tangent(h[0], h[1], secant[0], secant[1])
    m[n - 1] = _pchip_edge_tangent(h[n - 2], h[n - 3], secant[n - 2], secant[n - 3])

    def eval_one(t):
        if t <= x_nodes[0]:
            return y_nodes[0]
        if t >= x_nodes[n - 1]:
            return y_nodes[n - 1]
        k = 0
        for i in range(n - 1):
            if x_nodes[i] <= t <= x_nodes[i + 1]:
                k = i
                break
        h_k = x_nodes[k + 1] - x_nodes[k]
        s = (t - x_nodes[k]) / h_k
        s2 = s * s
        s3 = s2 * s
        h00 = 2.0 * s3 - 3.0 * s2 + 1.0
        h10 = s3 - 2.0 * s2 + s
        h01 = -2.0 * s3 + 3.0 * s2
        h11 = s3 - s2
        return h00 * y_nodes[k] + h10 * h_k * m[k] + h01 * y_nodes[k + 1] + h11 * h_k * m[k + 1]

    return [eval_one(t) for t in t_samples], True
```

- [ ] **Step 5: Run test to verify it passes**

Run: `cd SplineDesigner && python -m pytest tests/test_spline_math.py -v`
Expected: PASS (6 tests)

- [ ] **Step 6: Commit**

```bash
git add SplineDesigner/spline_math.py SplineDesigner/requirements.txt SplineDesigner/tests/__init__.py SplineDesigner/tests/conftest.py SplineDesigner/tests/test_spline_math.py
git commit -m "Add SplineDesigner scaffolding and PCHIP math module"
```

---

## Task 2: spline_csv.py — load/save the ankle spline.csv format

**Files:**
- Create: `SplineDesigner/spline_csv.py`
- Test: `SplineDesigner/tests/test_spline_csv.py`

**Interfaces:**
- Consumes: nothing from Task 1 (pure `csv`/stdlib only).
- Produces: `HEADER_ROW_COUNT = 5`, `PARAM_COUNT = 30`, `NUM_NODES = 12`, `load_spline_csv(path: str) -> tuple[list[float], list[float], list[float]]` (returns `(x_nodes[12], y_nodes[12], extra_params[6])`, raises `ValueError` on any format problem), `save_spline_csv(path: str, x_nodes: list[float], y_nodes: list[float], extra_params: list[float]) -> None` (raises `OSError` if the path can't be written, `ValueError` if the lists are the wrong length).

- [ ] **Step 1: Write the failing tests**

Create `SplineDesigner/tests/test_spline_csv.py`:

```python
import os

import pytest

from spline_csv import load_spline_csv, save_spline_csv

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
ANKLE_SPLINE_CSV = os.path.join(REPO_ROOT, "SDCard", "ankleControllers", "spline.csv")


def test_load_real_ankle_spline_csv():
    x_nodes, y_nodes, extra = load_spline_csv(ANKLE_SPLINE_CSV)
    assert len(x_nodes) == 12
    assert len(y_nodes) == 12
    assert len(extra) == 6
    assert x_nodes == [0.0, 5.0, 10.0, 20.0, 20.1, 20.2, 20.3, 20.4, 20.5, 20.6, 20.7, 20.8]
    assert y_nodes == [0.0, -8.0, -12.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    assert extra == [0.0, 1.0, 1.0, 6.0, 0.0, 0.03]


def test_round_trip_numeric_values(tmp_path):
    x_nodes, y_nodes, extra = load_spline_csv(ANKLE_SPLINE_CSV)
    out_path = str(tmp_path / "spline.csv")
    save_spline_csv(out_path, x_nodes, y_nodes, extra)
    x2, y2, extra2 = load_spline_csv(out_path)
    assert x2 == x_nodes
    assert y2 == y_nodes
    assert extra2 == extra


def test_save_rejects_wrong_node_count(tmp_path):
    out_path = str(tmp_path / "spline.csv")
    with pytest.raises(ValueError, match="12"):
        save_spline_csv(out_path, [0.0, 1.0], [0.0, 1.0], [0.0] * 6)


def test_save_rejects_wrong_extra_param_count(tmp_path):
    out_path = str(tmp_path / "spline.csv")
    with pytest.raises(ValueError, match="6"):
        save_spline_csv(out_path, [0.0] * 12, [0.0] * 12, [0.0] * 3)


def test_rejects_wrong_header_count_cell(tmp_path):
    p = tmp_path / "bad.csv"
    rows = [
        '4,"wrong header count"',
        '30,"x"',
        ",",
        ",",
        ",".join(f"n{i}" for i in range(30)),
        ",".join(["0"] * 30),
    ]
    p.write_text("\n".join(rows) + "\n")
    with pytest.raises(ValueError, match="header row count"):
        load_spline_csv(str(p))


def test_rejects_wrong_param_count_cell(tmp_path):
    p = tmp_path / "bad.csv"
    rows = [
        '5,"x"',
        '16,"wrong, this looks like a hip/arm file"',
        ",",
        ",",
        ",".join(f"n{i}" for i in range(16)),
        ",".join(["0"] * 16),
    ]
    p.write_text("\n".join(rows) + "\n")
    with pytest.raises(ValueError, match="parameter count"):
        load_spline_csv(str(p))


def test_rejects_wrong_value_count(tmp_path):
    p = tmp_path / "bad.csv"
    rows = [
        '5,"x"',
        '30,"x"',
        ",",
        ",",
        ",".join(f"n{i}" for i in range(30)),
        ",".join(["0"] * 29),
    ]
    p.write_text("\n".join(rows) + "\n")
    with pytest.raises(ValueError, match="expected 30 values"):
        load_spline_csv(str(p))


def test_rejects_non_numeric_value(tmp_path):
    p = tmp_path / "bad.csv"
    rows = [
        '5,"x"',
        '30,"x"',
        ",",
        ",",
        ",".join(f"n{i}" for i in range(30)),
        ",".join(["oops"] * 30),
    ]
    p.write_text("\n".join(rows) + "\n")
    with pytest.raises(ValueError, match="non-numeric"):
        load_spline_csv(str(p))
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd SplineDesigner && python -m pytest tests/test_spline_csv.py -v`
Expected: FAIL / ERROR with `ModuleNotFoundError: No module named 'spline_csv'`

- [ ] **Step 3: Write the implementation**

Create `SplineDesigner/spline_csv.py`:

```python
"""Load/save the ankle Spline controller's spline.csv format.

Format (see SDCard/ankleControllers/spline.csv):
  Row 1, cell 1: header row count (5) -- rows 1-5 are ignored by the firmware
                 parser except for this cell.
  Row 2, cell 1: parameter count (30) -- 24 node values (12 x,y pairs) + 6
                 trailing values.
  Rows 3-4:      free-text comments, not parsed for values.
  Row 5:         parameter names, for human reference only.
  Row 6:         the 30 comma-separated values, in the order named by row 5.
"""

import csv

HEADER_ROW_COUNT = 5
PARAM_COUNT = 30
NUM_NODES = 12

_PARAM_NAMES = [
    f"Node{i}_{axis}"
    for i in range(1, NUM_NODES + 1)
    for axis in ("x", "y")
] + ["1=sim %gait", "1=%gait 0=%stance", "PID Flag", "P Gain", "I Gain", "D Gain"]

_HEADER_ROWS = [
    ["5", "header Size, the first N rows will be ignored, except for this first cells in the first two rows"],
    ["30", "parameter number, the number of parameters to read per line"],
    ["", "Parameter list for the  spline controller"],
    ["", "Parameter order:"],
]


def load_spline_csv(path):
    """Returns (x_nodes[12], y_nodes[12], extra_params[6]). Raises ValueError
    if the file doesn't match the ankle spline.csv format."""
    with open(path, newline="") as f:
        rows = list(csv.reader(f))

    if len(rows) < HEADER_ROW_COUNT + 1:
        raise ValueError(f"{path}: expected at least {HEADER_ROW_COUNT + 1} rows, found {len(rows)}")

    header_count_cell = rows[0][0].strip() if rows[0] else ""
    if header_count_cell != str(HEADER_ROW_COUNT):
        raise ValueError(
            f"{path}: expected header row count '{HEADER_ROW_COUNT}' in row 1 cell 1, "
            f"found {header_count_cell!r}"
        )

    param_count_cell = rows[1][0].strip() if rows[1] else ""
    if param_count_cell != str(PARAM_COUNT):
        raise ValueError(
            f"{path}: expected parameter count '{PARAM_COUNT}' in row 2 cell 1, "
            f"found {param_count_cell!r} -- is this a hip/arm spline.csv (16 parameters) "
            "instead of an ankle one (30)?"
        )

    value_row = rows[HEADER_ROW_COUNT]
    cells = [c.strip() for c in value_row if c.strip() != ""]
    if len(cells) != PARAM_COUNT:
        raise ValueError(
            f"{path}: expected {PARAM_COUNT} values in row {HEADER_ROW_COUNT + 1}, found {len(cells)}"
        )

    try:
        values = [float(c) for c in cells]
    except ValueError as e:
        raise ValueError(f"{path}: non-numeric value in row {HEADER_ROW_COUNT + 1}: {e}") from e

    x_nodes = values[0:24:2]
    y_nodes = values[1:24:2]
    extra_params = values[24:30]
    return x_nodes, y_nodes, extra_params


def _format_value(v):
    """Trims trailing zeros to match the existing file's plain style (0 not
    0.0, 20.1 stays 20.1)."""
    if float(v).is_integer():
        return str(int(v))
    return "%g" % v


def save_spline_csv(path, x_nodes, y_nodes, extra_params):
    """Writes the ankle spline.csv format. Raises ValueError if the input
    lists are the wrong length, OSError if path can't be written."""
    if len(x_nodes) != NUM_NODES or len(y_nodes) != NUM_NODES:
        raise ValueError(
            f"expected {NUM_NODES} x_nodes and y_nodes, got {len(x_nodes)} and {len(y_nodes)}"
        )
    if len(extra_params) != 6:
        raise ValueError(f"expected 6 extra_params, got {len(extra_params)}")

    value_row = []
    for x, y in zip(x_nodes, y_nodes):
        value_row.append(_format_value(x))
        value_row.append(_format_value(y))
    value_row.extend(_format_value(v) for v in extra_params)

    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        for row in _HEADER_ROWS:
            writer.writerow(row)
        writer.writerow(_PARAM_NAMES)
        writer.writerow(value_row)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd SplineDesigner && python -m pytest tests/test_spline_csv.py -v`
Expected: PASS (8 tests)

- [ ] **Step 5: Commit**

```bash
git add SplineDesigner/spline_csv.py SplineDesigner/tests/test_spline_csv.py
git commit -m "Add spline_csv load/save for the ankle spline.csv format"
```

---

## Task 3: node_state.py — drag/type clamping logic

**Files:**
- Create: `SplineDesigner/node_state.py`
- Test: `SplineDesigner/tests/test_node_state.py`

**Interfaces:**
- Consumes: `NODE_X_BOUNDS`, `NODE_Y_BOUNDS`, `pchip_curve` from `spline_math` (Task 1).
- Produces: `X_EPSILON = 0.1`, `class NodeState` with constructor `NodeState(x_nodes: list[float], y_nodes: list[float], extra_params: list[float])`, attributes `x_nodes`, `y_nodes`, `extra_params` (plain lists, directly mutable), methods `set_x(index: int, new_x: float) -> None`, `set_y(index: int, new_y: float) -> None`, `curve_samples(sample_count: int = 200) -> tuple[list[float], list[float], bool]` (returns `(samples_x, samples_y, valid)`).

- [ ] **Step 1: Write the failing tests**

Create `SplineDesigner/tests/test_node_state.py`:

```python
import pytest

from node_state import NodeState, X_EPSILON


def _make_state():
    return NodeState([0.0, 50.0, 100.0], [0.0, 0.0, 0.0], [0.0] * 6)


def test_set_x_clamps_below_left_neighbor():
    ns = _make_state()
    ns.set_x(1, 0.0)
    assert ns.x_nodes[1] == pytest.approx(0.0 + X_EPSILON)


def test_set_x_clamps_above_right_neighbor():
    ns = _make_state()
    ns.set_x(1, 150.0)
    assert ns.x_nodes[1] == pytest.approx(100.0 - X_EPSILON)


def test_set_x_clamps_to_bounds_at_left_end():
    ns = _make_state()
    ns.set_x(0, -10.0)
    assert ns.x_nodes[0] == 0.0


def test_set_x_clamps_to_bounds_at_right_end():
    ns = _make_state()
    ns.set_x(2, 200.0)
    assert ns.x_nodes[2] == 100.0


def test_set_x_allows_normal_move():
    ns = _make_state()
    ns.set_x(1, 60.0)
    assert ns.x_nodes[1] == 60.0


def test_set_y_clamps_to_bounds():
    ns = _make_state()
    ns.set_y(1, 1000.0)
    assert ns.y_nodes[1] == 100.0
    ns.set_y(1, -1000.0)
    assert ns.y_nodes[1] == -100.0


def test_set_y_allows_normal_move():
    ns = _make_state()
    ns.set_y(1, 42.0)
    assert ns.y_nodes[1] == 42.0


def test_curve_samples_matches_pchip_curve():
    ns = NodeState([0.0, 50.0, 100.0], [0.0, 20.0, 0.0], [0.0] * 6)
    xs, ys, valid = ns.curve_samples(sample_count=50)
    assert valid is True
    assert len(xs) == 50
    assert len(ys) == 50
    assert xs[0] == pytest.approx(0.0)
    assert xs[-1] == pytest.approx(100.0)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd SplineDesigner && python -m pytest tests/test_node_state.py -v`
Expected: FAIL / ERROR with `ModuleNotFoundError: No module named 'node_state'`

- [ ] **Step 3: Write the implementation**

Create `SplineDesigner/node_state.py`:

```python
"""In-memory node state for the spline designer: holds the 12 node
positions and 6 extra parameters, and enforces the firmware's strictly-
increasing-X rule structurally (both drag and type go through set_x, so an
invalid ordering is never reachable through the UI).

Assumes the node x-values it's constructed with are already spaced at least
2 * X_EPSILON apart, which holds for any spline.csv already used with real
gait-cycle percentages.
"""

from spline_math import NODE_X_BOUNDS, NODE_Y_BOUNDS, pchip_curve

X_EPSILON = 0.1  # % gait -- minimum gap enforced between adjacent node x-values


class NodeState:
    def __init__(self, x_nodes, y_nodes, extra_params):
        if len(x_nodes) != len(y_nodes):
            raise ValueError(
                f"x_nodes and y_nodes must be the same length, got {len(x_nodes)} and {len(y_nodes)}"
            )
        self.x_nodes = list(x_nodes)
        self.y_nodes = list(y_nodes)
        self.extra_params = list(extra_params)

    def _clamp_x(self, index, new_x):
        lo = NODE_X_BOUNDS[0] if index == 0 else self.x_nodes[index - 1] + X_EPSILON
        hi = (
            NODE_X_BOUNDS[1]
            if index == len(self.x_nodes) - 1
            else self.x_nodes[index + 1] - X_EPSILON
        )
        return max(lo, min(new_x, hi))

    def _clamp_y(self, new_y):
        return max(NODE_Y_BOUNDS[0], min(new_y, NODE_Y_BOUNDS[1]))

    def set_x(self, index, new_x):
        self.x_nodes[index] = self._clamp_x(index, new_x)

    def set_y(self, index, new_y):
        self.y_nodes[index] = self._clamp_y(new_y)

    def curve_samples(self, sample_count=200):
        samples_x = [
            NODE_X_BOUNDS[0] + i * (NODE_X_BOUNDS[1] - NODE_X_BOUNDS[0]) / (sample_count - 1)
            for i in range(sample_count)
        ]
        samples_y, valid = pchip_curve(self.x_nodes, self.y_nodes, samples_x)
        return samples_x, samples_y, valid
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd SplineDesigner && python -m pytest tests/test_node_state.py -v`
Expected: PASS (8 tests)

- [ ] **Step 5: Commit**

```bash
git add SplineDesigner/node_state.py SplineDesigner/tests/test_node_state.py
git commit -m "Add NodeState with drag/type clamping for spline node x-ordering"
```

---

## Task 4: spline_designer.py — the desktop UI

**Files:**
- Create: `SplineDesigner/spline_designer.py`

**Interfaces:**
- Consumes: `NodeState` (`__init__`, `.x_nodes`, `.y_nodes`, `.extra_params`, `.set_x`, `.set_y`, `.curve_samples`) from Task 3; `NODE_X_BOUNDS`, `NODE_Y_BOUNDS` from Task 1; `load_spline_csv`, `save_spline_csv`, `NUM_NODES` from Task 2.
- Produces: `SplineDesignerWindow` (a `QtWidgets.QMainWindow`), `main()` entry point.

- [ ] **Step 1: Write the implementation**

Create `SplineDesigner/spline_designer.py`:

```python
"""Standalone ankle spline designer: drag nodes on the plot or type their
values, then export a spline.csv matching SDCard/ankleControllers/spline.csv.

Run with: python spline_designer.py
"""

import os
import sys

from PySide6 import QtWidgets
import pyqtgraph as pg

from node_state import NodeState
from spline_csv import NUM_NODES, load_spline_csv, save_spline_csv
from spline_math import NODE_X_BOUNDS, NODE_Y_BOUNDS

DEFAULT_X = [0.0, 5.0, 10.0, 20.0, 20.1, 20.2, 20.3, 20.4, 20.5, 20.6, 20.7, 20.8]
DEFAULT_Y = [0.0, -8.0, -12.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
DEFAULT_EXTRA = [0.0, 1.0, 1.0, 6.0, 0.0, 0.03]
EXTRA_LABELS = ["sim %gait (1=yes)", "1=%gait 0=%stance", "PID Flag", "P Gain", "I Gain", "D Gain"]

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_SPLINE_PATH = os.path.join(REPO_ROOT, "SDCard", "ankleControllers", "spline.csv")


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
        self.plot.setXRange(*NODE_X_BOUNDS, padding=0.05)
        self.plot.setYRange(*NODE_Y_BOUNDS, padding=0.05)
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
            spin_x.valueChanged.connect(lambda val, idx=i: self._on_x_spin_changed(idx, val))
            grid.addWidget(spin_x, i + 1, 1)
            self._x_spins.append(spin_x)

            spin_y = QtWidgets.QDoubleSpinBox()
            spin_y.setDecimals(2)
            spin_y.setRange(*NODE_Y_BOUNDS)
            spin_y.setSingleStep(0.5)
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
        self.status_label.setText("" if valid else "Invalid node order (should be unreachable via this UI)")

    def _on_target_moved(self, index, target):
        if self._updating:
            return
        pos = target.pos()
        self.state.set_x(index, pos.x())
        self.state.set_y(index, pos.y())
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
    window = SplineDesignerWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Smoke-test the window builds without a display**

Run:

```bash
cd SplineDesigner && QT_QPA_PLATFORM=offscreen python -c "
from PySide6 import QtWidgets
from spline_designer import SplineDesignerWindow
app = QtWidgets.QApplication([])
w = SplineDesignerWindow()
print('nodes:', list(zip(w.state.x_nodes, w.state.y_nodes)))
print('curve points:', len(w.curve.getData()[0]))
print('OK')
"
```

Expected: prints the 12 default `(x, y)` node pairs, `curve points: 200`, and `OK`, with no traceback (an unrelated NumPy/bottleneck warning from pyqtgraph's optional ImageView import may print — that's a pre-existing environment quirk, not an error from this code, and can be ignored as long as `OK` is printed at the end).

- [ ] **Step 3: Manually verify the interactive app**

Run: `cd SplineDesigner && python spline_designer.py`

Verify, in order:
1. The window opens showing the default 12-node curve (matches the current `SDCard/ankleControllers/spline.csv` shape) and the 6 extra-parameter fields show `0, 1, 1, 6, 0, 0.03`.
2. Dragging a node on the plot moves the curve live and updates that node's X/Y spinboxes to match.
3. Typing a new value into a node's X or Y spinbox moves that node on the plot and redraws the curve.
4. Dragging (or typing) a node's X past its right neighbor stops it just short of the neighbor (does not cross it); same for the left neighbor.
5. Click "Open...", select `SDCard/ankleControllers/spline.csv` — the plot/spinboxes/extra-params refresh to the loaded values (same as the defaults, since that's what they were seeded from).
6. Click "Save As...", save to a scratch path (e.g. `/tmp/test_spline.csv`) — no error dialog appears, and the status label shows the saved path.
7. Open the saved scratch file in a text editor (or re-open it with "Open...") and confirm the node values match what was in the editor when saved.

- [ ] **Step 4: Commit**

```bash
git add SplineDesigner/spline_designer.py
git commit -m "Add SplineDesigner UI: draggable node plot, spinboxes, Open/Save As"
```
