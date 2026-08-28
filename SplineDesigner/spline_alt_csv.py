"""Load/save the ankle SplineAlt controller's splineAlt.csv format.

Format (see SDCard/ankleControllers/splineAlt.csv):
  Row 1, cell 1: header row count (5) -- rows 1-5 are ignored by the firmware
                 parser except for this cell.
  Row 2, cell 1: parameter count (17).
  Rows 3-4:      free-text comments, not parsed for values.
  Row 5:         parameter names, for human reference only.
  Row 6:         the 17 comma-separated values, in the order named by row 5.

Torque magnitudes (PlantarNm, DorsiNm) are stored POSITIVE; the firmware applies
the plantarflexion sign internally.
"""

import csv

HEADER_ROW_COUNT = 5
PARAM_COUNT = 17
NUM_PARAMS = PARAM_COUNT

# Matches row 5 of the committed splineAlt.csv exactly.
_PARAM_NAMES = [
    "PlantarNm", "DorsiNm", "PlantarPk", "DorsiPk", "PlantRise", "PlantDwel",
    "PlantFall", "DorsiRise", "DorsiDwel", "DorsiFall", "TorqScale",
    "1=sim%gt", "1=%gait", "PID Flag", "P Gain", "I Gain", "D Gain",
]

_HEADER_ROWS = [
    ["5", "header Size, the first N rows will be ignored, except for this first cells in the first two rows"],
    ["17", "parameter number, the number of parameters to read per line"],
    ["", "Parameter list for the splineAlt controller. Torque magnitudes are POSITIVE; "
         "plantarflexion is applied negative internally. Rise/dwell/fall are durations in "
         "percent gait. Dwell 0 = no plateau. A lobe with magnitude 0 is disabled. Lobes "
         "may wrap past 100 percent."],
    ["", "Parameter order:"],
]


def load_spline_alt_csv(path):
    """Returns a list of 17 floats. Raises ValueError if the file doesn't match
    the ankle splineAlt.csv format."""
    with open(path, newline="") as f:
        rows = list(csv.reader(f))

    if len(rows) < HEADER_ROW_COUNT + 1:
        raise ValueError(
            f"{path}: expected at least {HEADER_ROW_COUNT + 1} rows, found {len(rows)}"
        )

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
            f"found {param_count_cell!r} -- is this a node spline.csv (30 parameters) "
            "instead of a splineAlt.csv (17)?"
        )

    value_row = rows[HEADER_ROW_COUNT]
    cells = [c.strip() for c in value_row if c.strip() != ""]
    if len(cells) != PARAM_COUNT:
        raise ValueError(
            f"{path}: expected {PARAM_COUNT} values in row {HEADER_ROW_COUNT + 1}, "
            f"found {len(cells)}"
        )

    try:
        values = [float(c) for c in cells]
    except ValueError as e:
        raise ValueError(
            f"{path}: non-numeric value in row {HEADER_ROW_COUNT + 1}: {e}"
        ) from e

    return values


def _format_value(v):
    """Trims trailing zeros to match the file's plain style (0 not 0.0)."""
    if float(v).is_integer():
        return str(int(v))
    return "%g" % v


def save_spline_alt_csv(path, params):
    """Writes the ankle splineAlt.csv format. Raises ValueError if params is the
    wrong length, OSError if path can't be written."""
    if len(params) != PARAM_COUNT:
        raise ValueError(f"expected {PARAM_COUNT} params, got {len(params)}")

    value_row = [_format_value(v) for v in params]

    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        for row in _HEADER_ROWS:
            writer.writerow(row)
        writer.writerow(_PARAM_NAMES)
        writer.writerow(value_row)
