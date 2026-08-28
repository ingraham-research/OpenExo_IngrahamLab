"""Tests for load/save of SDCard/ankleControllers/splineAlt.csv."""

import os

import pytest

from spline_alt_csv import (
    NUM_PARAMS,
    PARAM_COUNT,
    load_spline_alt_csv,
    save_spline_alt_csv,
)

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
REAL_SPLINE_ALT = os.path.join(REPO_ROOT, "SDCard", "ankleControllers", "splineAlt.csv")
REAL_SPLINE = os.path.join(REPO_ROOT, "SDCard", "ankleControllers", "spline.csv")


def test_param_count_is_17():
    assert PARAM_COUNT == NUM_PARAMS == 17


def test_load_real_splinealt_csv():
    params = load_spline_alt_csv(REAL_SPLINE_ALT)
    assert params == pytest.approx(
        [15, 5, 50, 95, 26, 0, 17, 11, 0, 9, 100, 0, 1, 1, 3, 0, 0.01]
    )


def test_round_trip_numeric_values(tmp_path):
    params = [12.5, 4.0, 48.0, 92.0, 20.0, 3.0, 15.0, 10.0, 0.0, 8.0,
              80.0, 1.0, 1.0, 1.0, 2.5, 0.0, 0.02]
    path = tmp_path / "splineAlt.csv"
    save_spline_alt_csv(str(path), params)
    assert load_spline_alt_csv(str(path)) == pytest.approx(params)


def test_rejects_node_spline_csv():
    # spline.csv has 30 params, not 17 -- must fail with a clear message
    with pytest.raises(ValueError) as exc:
        load_spline_alt_csv(REAL_SPLINE)
    assert "17" in str(exc.value)


def test_rejects_wrong_value_count(tmp_path):
    path = tmp_path / "bad.csv"
    save_spline_alt_csv(
        str(path),
        [15, 5, 50, 95, 26, 0, 17, 11, 0, 9, 100, 0, 1, 1, 3, 0, 0.01],
    )
    lines = path.read_text().splitlines()
    lines[-1] = lines[-1] + ",99"   # 18 values now
    path.write_text("\n".join(lines) + "\n")
    with pytest.raises(ValueError):
        load_spline_alt_csv(str(path))


def test_rejects_non_numeric_value(tmp_path):
    path = tmp_path / "bad.csv"
    save_spline_alt_csv(
        str(path),
        [15, 5, 50, 95, 26, 0, 17, 11, 0, 9, 100, 0, 1, 1, 3, 0, 0.01],
    )
    text = path.read_text().replace("15,5,50", "oops,5,50")
    path.write_text(text)
    with pytest.raises(ValueError):
        load_spline_alt_csv(str(path))


def test_save_rejects_wrong_param_count(tmp_path):
    with pytest.raises(ValueError):
        save_spline_alt_csv(str(tmp_path / "x.csv"), [1, 2, 3])


def test_saved_file_has_expected_header(tmp_path):
    path = tmp_path / "splineAlt.csv"
    save_spline_alt_csv(
        str(path),
        [15, 5, 50, 95, 26, 0, 17, 11, 0, 9, 100, 0, 1, 1, 3, 0, 0.01],
    )
    rows = path.read_text().splitlines()
    assert rows[0].split(",")[0] == "5"
    assert rows[1].split(",")[0] == "17"
    assert rows[4].startswith("PlantarNm,DorsiNm,PlantarPk")
