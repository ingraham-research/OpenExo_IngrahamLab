# Standalone Spline Designer — Design

**Date:** 2026-08-26
**Branch:** `spline_plotting_feature`
**Status:** awaiting review

## Goal

A local desktop app, completely separate from the exo GUI (`Python_GUI/`), for designing the ankle
Spline controller's node profile offline: drag nodes on a plot or type their values, preview the exact
curve the firmware will produce, and export a `spline.csv` in the exact format
`SDCard/ankleControllers/spline.csv` already uses — ready to drop onto the SD card.

No BLE, no exo connection, no dependency on `Python_GUI/`. It only needs to run and produce a correct
file.

## Non-goals

- **No hip/arm (5-node) support.** Ankle's 12-node / 30-parameter shape only.
- **No BLE / live device connection.** This tool never talks to the exo; it only reads/writes CSV files.
- **No dependency on `Python_GUI/`.** The small amount of shared math (PCHIP interpolation) is
  duplicated into this project rather than imported, so the two apps can be run, shared, and modified
  independently.
- **Not a general CSV editor.** It understands exactly one format: the ankle Spline controller's
  `spline.csv`.

## Background: the file format

`SDCard/ankleControllers/spline.csv`, current contents:

```
5,"header Size, the first N rows will be ignored, except for this first cells in the first two rows",...
30,"parameter number, the number of parameters to read per line",...
,Parameter list for the  spline controller,...
,Parameter order:,...
Node1_x,Node1_y,Node2_x,Node2_y,...,Node12_x,Node12_y,1=sim %gait,1=%gait 0=%stance,PID Flag,P Gain,I Gain,D Gain
0,0,5,-8,10,-12,20,0,20.1,0,20.2,0,20.3,0,20.4,0,20.5,0,20.6,0,20.7,0,20.8,0,0,1,1,6,0,0.03
```

- Row 1, cell 1: header row count (`5`) — rows 1-5 are ignored by the firmware parser except for this cell.
- Row 2, cell 1: parameter count (`30`) — 24 node values (12 × x,y) + 6 trailing values.
- Rows 3-4: free-text comments, not parsed for values.
- Row 5: parameter names, for human reference only.
- Row 6: the 30 comma-separated values, in the order named by row 5.
- Node value bounds (matches `Python_GUI/utils/spline_math.py` / firmware `ControllerData.cpp`):
  `x ∈ [0, 100]` (% gait), `y ∈ [-100, 100]` (Nm). X must be strictly increasing node-to-node or the
  firmware's interpolator returns 0 for everything.
- Trailing 6 values: sim-gait flag, %gait-vs-%stance flag, PID flag, P gain, I gain, D gain — opaque
  numbers as far as this tool is concerned; it just round-trips them through a small editable form.

## Architecture

New top-level folder, sibling to `Python_GUI/`:

```
SplineDesigner/
  spline_designer.py   # entry point: QApplication + main window, wires UI to spline_csv + spline_math
  spline_math.py        # PCHIP interpolation — copy of Python_GUI/utils/spline_math.py
  spline_csv.py         # load_spline_csv() / save_spline_csv() for the ankle spline.csv format
  tests/
    test_spline_csv.py
    test_spline_math.py
```

Nothing in `SplineDesigner/` imports from `Python_GUI/`. `spline_math.py` carries forward the existing
"kept in sync by hand with the firmware" warning comment from its source, since the two copies (exo GUI's
and this tool's) must both keep matching `ExoCode/src/Controller.cpp`'s interpolation — this is an
accepted, documented duplication, not an oversight.

## Components

### `spline_math.py`

Verbatim port of `Python_GUI/utils/spline_math.py`: `NODE_X_BOUNDS`, `NODE_Y_BOUNDS`,
`nodes_strictly_increasing()`, `pchip_curve(x_nodes, y_nodes, t_samples) -> (y_values, valid)`.

### `spline_csv.py`

```python
def load_spline_csv(path) -> tuple[list[float], list[float], list[float]]:
    """Returns (x_nodes[12], y_nodes[12], extra_params[6])."""

def save_spline_csv(path, x_nodes, y_nodes, extra_params) -> None:
    """Writes the same 7-line format, values formatted to match the existing file's plain style
    (e.g. -8, not -8.0)."""
```

`load_spline_csv`:
- Reads all rows. Validates row 1 cell 1 == `"5"` and row 2 cell 1 == `"30"`; raises a clear
  `ValueError` naming the file and what was expected/found if not (guards against pointing the tool at a
  hip/arm 16-parameter file by mistake).
- Reads row 6 (index 5), splits on comma, expects exactly 30 numeric cells; raises a clear error
  otherwise (wrong count, non-numeric cell).
- Splits into `x_nodes = values[0:24:2]`, `y_nodes = values[1:24:2]`, `extra_params = values[24:30]`.

`save_spline_csv`:
- Writes rows 1-5 verbatim (the same boilerplate text currently in the file — hardcoded constants in
  this module, not read from an existing file, so Save works even when no file was opened first).
- Writes row 6 as the interleaved `x_nodes`/`y_nodes` followed by `extra_params`, 30 values,
  comma-joined, each formatted with a trim-trailing-zeros rule (`0` not `0.0`, `20.1` stays `20.1`).
- No trailing blank row is required (row 7 in the current file is just an artifact of a trailing
  newline).

### `spline_designer.py`

Single-window PySide6 app:

- **Plot (left, pyqtgraph `PlotWidget`)** — draggable `ScatterPlotItem` for the 12 nodes plus a `PlotDataItem`
  curve re-sampled from `pchip_curve()` on every change. X range fixed `[0, 100]`, Y range auto-fit
  around current node/curve values (same padding approach as `Python_GUI/Widgets/SplineNodeEditor.py`'s
  `_update_y_range`).
- **Node panel (right)** — 12 rows of X/Y `QDoubleSpinBox` pairs, two-way bound to the same in-memory
  node state as the plot: dragging a point updates its spinboxes, editing a spinbox moves its point.
- **Extra-params form (bottom or right, below the node panel)** — 6 labeled fields for the trailing
  values, defaulting to the current `SDCard/ankleControllers/spline.csv` values (`0, 1, 1, 6, 0, 0.03`)
  until something is loaded or changed.
- **Toolbar** — `Open...` and `Save As...` buttons. Both file dialogs default to
  `SDCard/ankleControllers/spline.csv` (resolved relative to the repo root, found by walking up from
  `SplineDesigner/` — the folder is a fixed sibling of `SDCard/`, so this is a static relative path, no
  search needed).

**Drag/type clamping (shared rule, not a mode):** whether a node's X is changed by dragging or by typing
into its spinbox, the new value is clamped into
`(previous_node.x + EPSILON, next_node.x - EPSILON)` (and within `NODE_X_BOUNDS` at the two ends), where
`EPSILON = 0.1` (% gait). This makes an invalid/out-of-order state unreachable through either input path,
so there is no warning-label / invalid-curve case to handle in this tool (unlike the exo GUI's live
editor, which allows transient invalid states because it's editing a running controller). Y values only
clamp to `NODE_Y_BOUNDS`, no ordering constraint.

## Data flow

```
Open file ──> load_spline_csv() ──> (x_nodes, y_nodes, extra_params)
                                          │
                                          ▼
                         node state (drives plot + spinboxes + form)
                                          │
                     drag or type ────────┤ (clamped, then...)
                                          ▼
                         pchip_curve() ──> curve redraw
                                          │
Save As ──> save_spline_csv() <───────────┘
```

There is no other state and no persistence beyond the CSV file itself — closing the app without saving
discards edits, same as any editor.

## Error handling

- `Open` on a malformed/wrong-shape file: `load_spline_csv` raises `ValueError` with a specific message
  (e.g. "expected 30 parameters, found 16 — is this a hip/arm spline.csv?"); the UI catches it and shows
  a `QMessageBox` without changing current state.
- `Save As` to an unwritable path (permissions, missing parent dir): catch the `OSError`, show a
  `QMessageBox`, current state unchanged.
- No other failure modes exist — no network, no device, no background threads.

## Testing

- `tests/test_spline_math.py`: `pchip_curve` matches known values for a simple 3-4 node case;
  `nodes_strictly_increasing` rejects non-increasing input; `pchip_curve` returns `valid=False` and
  all-zero output for non-increasing x (mirrors firmware behavior).
- `tests/test_spline_csv.py`:
  - Round-trip: `load_spline_csv` on the real `SDCard/ankleControllers/spline.csv` then
    `save_spline_csv` to a temp file reproduces the same 30 values (row 6) — not necessarily
    byte-identical whitespace, but numerically identical and in the same column order.
  - Malformed input: wrong header-count cell, wrong parameter-count cell, wrong number of value columns,
    non-numeric value cell — each raises `ValueError` with a message identifying the problem.
- UI drag/type/clamp behavior is verified manually (launch the app, drag a node past its neighbor and
  confirm it stops at the clamp boundary, type an out-of-range X and confirm it clamps, Open the real
  ankle `spline.csv` and confirm the plot matches the exo GUI's `SplineNodeEditor` preview for the same
  values).
