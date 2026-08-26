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
