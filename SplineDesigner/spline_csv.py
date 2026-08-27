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
