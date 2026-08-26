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
