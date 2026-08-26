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
