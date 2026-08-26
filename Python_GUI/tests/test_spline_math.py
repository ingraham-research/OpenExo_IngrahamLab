from utils.spline_math import pchip_curve, nodes_strictly_increasing


NODES_X = [0.0, 10.0, 25.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0, 95.0, 98.0, 100.0]
NODES_Y = [0.0, 2.0, 5.0, 15.0, 18.0, 10.0, -2.0, 3.0, 1.0, 0.5, 0.2, 0.0]


def test_curve_passes_through_every_node():
    # Evaluating exactly at each node's x must reproduce that node's y,
    # since a spline is an interpolant, not just an approximation.
    y_values, valid = pchip_curve(NODES_X, NODES_Y, NODES_X)
    assert valid
    for expected, actual in zip(NODES_Y, y_values):
        assert actual == expected


def test_flat_before_first_and_after_last_node():
    y_values, valid = pchip_curve(NODES_X, NODES_Y, [-5.0, 0.0, 100.0, 105.0])
    assert valid
    assert y_values[0] == NODES_Y[0]
    assert y_values[1] == NODES_Y[0]
    assert y_values[2] == NODES_Y[-1]
    assert y_values[3] == NODES_Y[-1]


def test_invalid_node_order_returns_zero_curve():
    # Mirrors Spline::_pchip_interpolate's guard: firmware commands 0 Nm
    # everywhere when x nodes aren't strictly increasing.
    bad_x = list(NODES_X)
    bad_x[5], bad_x[6] = bad_x[6], bad_x[5]  # break strictly-increasing order
    y_values, valid = pchip_curve(bad_x, NODES_Y, [0.0, 25.0, 50.0, 100.0])
    assert not valid
    assert y_values == [0.0, 0.0, 0.0, 0.0]


def test_nodes_strictly_increasing():
    assert nodes_strictly_increasing(NODES_X)
    assert not nodes_strictly_increasing([0.0, 10.0, 10.0, 20.0])
    assert not nodes_strictly_increasing([0.0, 20.0, 10.0])
