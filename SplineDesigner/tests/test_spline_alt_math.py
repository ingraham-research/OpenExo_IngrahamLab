"""Tests for the SplineAlt node builder / PCHIP port in spline_alt_math.py.

These assert the behaviour of the committed firmware algorithm
(ExoCode/src/Controller.cpp, SplineAlt::_build_nodes / _pchip_interpolate as of
commit c22eb053), traced by hand:

  - each lobe emits (peak-rise, 0) (peak, amp) [ (peak+dwell, amp) ] (peak+dwell+fall, 0)
  - amplitude 0 -> the lobe emits nothing
  - dwell 0    -> the plateau node is skipped (3 nodes, not fatal)
  - rise 0 / fall 0 / an exact x-collision -> the strictly-increasing guard trips
    and the whole profile is rejected (build returns nothing)
  - raw nodes are wrapped mod 100, sorted, then extended by two periodic copies
    on each side, so the returned array has (interior count + 4) entries.

The prose summary tables in
"Modification log with claude/SplineAlt-Shape-Parameterised-Controller.md" are
internally inconsistent about node counts; these assertions follow the code.
"""

import math

import pytest

from spline_alt_math import (
    NUM_PARAMS,
    build_extended_nodes,
    clamp_param,
    interior_nodes,
    sample_curve,
)

# PlantarNm DorsiNm PlantarPk DorsiPk PlantRise PlantDwel PlantFall DorsiRise DorsiDwel DorsiFall
# TorqScale sim %gait PIDflag P I D  -- the committed SDCard/ankleControllers/splineAlt.csv
DEFAULT_PARAMS = [15.0, 5.0, 50.0, 95.0, 26.0, 0.0, 17.0, 11.0, 0.0, 9.0,
                  100.0, 0.0, 1.0, 1.0, 3.0, 0.0, 0.01]


def _params(**overrides):
    p = list(DEFAULT_PARAMS)
    names = {
        "plantar_nm": 0, "dorsi_nm": 1, "plantar_pk": 2, "dorsi_pk": 3,
        "plantar_rise": 4, "plantar_dwell": 5, "plantar_fall": 6,
        "dorsi_rise": 7, "dorsi_dwell": 8, "dorsi_fall": 9, "torque_scale": 10,
    }
    for k, v in overrides.items():
        p[names[k]] = v
    return p


def test_num_params_is_17():
    assert NUM_PARAMS == 17
    assert len(DEFAULT_PARAMS) == NUM_PARAMS


def test_default_profile_interior_nodes():
    xs, ys = interior_nodes(DEFAULT_PARAMS)
    # plantar: (24,0) (50,-15) (67,0);  dorsi: (84,0) (95,5) (104->4,0)
    assert xs == pytest.approx([4.0, 24.0, 50.0, 67.0, 84.0, 95.0])
    assert ys == pytest.approx([0.0, 0.0, -15.0, 0.0, 0.0, 5.0])


def test_default_profile_extended_node_count_is_interior_plus_four():
    xs, ys = build_extended_nodes(DEFAULT_PARAMS)
    ix, _ = interior_nodes(DEFAULT_PARAMS)
    assert len(xs) == len(ix) + 4 == 10
    assert len(ys) == len(xs)
    # periodic extension: two tail copies shifted -100 before, two head copies +100 after
    assert xs[0] == pytest.approx(ix[-2] - 100.0)
    assert xs[1] == pytest.approx(ix[-1] - 100.0)
    assert xs[-2] == pytest.approx(ix[0] + 100.0)
    assert xs[-1] == pytest.approx(ix[1] + 100.0)
    # strictly increasing overall
    assert all(b > a for a, b in zip(xs, xs[1:]))


def test_curve_passes_through_every_interior_node():
    """Hermite interpolation property -- independent of the design doc's numbers."""
    xs, ys = interior_nodes(DEFAULT_PARAMS)
    sampled, valid = sample_curve(DEFAULT_PARAMS, xs)
    assert valid
    assert sampled == pytest.approx(ys, abs=1e-6)


def test_curve_peaks_and_seam():
    samples = [0.0, 50.0, 95.0]
    ys, valid = sample_curve(DEFAULT_PARAMS, samples)
    assert valid
    assert ys[1] == pytest.approx(-15.0, abs=1e-6)   # plantar peak
    assert ys[2] == pytest.approx(5.0, abs=1e-6)     # dorsi peak
    assert ys[0] == pytest.approx(2.09, abs=0.05)    # doc: +2.09 Nm at 0 % gait
    # C1-continuous across the 0/100 seam -> nearly equal just either side of it
    near, _ = sample_curve(DEFAULT_PARAMS, [0.01, 99.99])
    assert near[0] == pytest.approx(near[1], abs=0.05)


def test_torque_scale_zero_is_degenerate():
    xs, ys = build_extended_nodes(_params(torque_scale=0.0))
    assert xs == [] and ys == []
    sampled, valid = sample_curve(_params(torque_scale=0.0), [0.0, 25.0, 50.0, 75.0])
    assert not valid
    assert sampled == [0.0, 0.0, 0.0, 0.0]


def test_torque_scale_50_halves_the_peaks():
    ys, valid = sample_curve(_params(torque_scale=50.0), [50.0, 95.0])
    assert valid
    assert ys[0] == pytest.approx(-7.5, abs=1e-6)
    assert ys[1] == pytest.approx(2.5, abs=1e-6)


def test_zero_magnitude_disables_a_lobe():
    xs, ys = interior_nodes(_params(dorsi_nm=0.0))
    assert xs == pytest.approx([24.0, 50.0, 67.0])
    assert ys == pytest.approx([0.0, -15.0, 0.0])
    _, valid = sample_curve(_params(dorsi_nm=0.0), [50.0])
    assert valid


def test_dwell_zero_is_tolerated_dwell_positive_adds_a_node():
    # default already has dwell 0 on both lobes -> 6 interior nodes
    assert len(interior_nodes(DEFAULT_PARAMS)[0]) == 6
    # a plateau on the plantar lobe adds exactly one node
    xs, ys = interior_nodes(_params(plantar_dwell=8.0))
    assert len(xs) == 7
    assert xs == pytest.approx([4.0, 24.0, 50.0, 58.0, 75.0, 84.0, 95.0])
    assert ys == pytest.approx([0.0, 0.0, -15.0, -15.0, 0.0, 0.0, 5.0])


def test_both_lobes_with_dwell_hit_the_twelve_node_maximum():
    xs, _ = build_extended_nodes(_params(plantar_dwell=6.0, dorsi_dwell=4.0))
    assert len(xs) == 12   # 8 interior + 4 wrap; doc: "12 nodes (the maximum)"


def test_zero_rise_is_rejected():
    xs, ys = build_extended_nodes(_params(plantar_rise=0.0))
    assert xs == [] and ys == []
    _, valid = sample_curve(_params(plantar_rise=0.0), [10.0, 50.0])
    assert not valid


def test_zero_fall_is_rejected():
    xs, _ = build_extended_nodes(_params(plantar_fall=0.0))
    assert xs == []


def test_exact_lobe_collision_is_rejected():
    # plantar ends at peak+dwell+fall = 50+0+34 = 84, which is the dorsi lobe's
    # first node (dorsi_pk 95 - dorsi_rise 11) -> exact x collision
    xs, _ = build_extended_nodes(_params(plantar_fall=34.0))
    assert xs == []


def test_interleaved_but_non_colliding_lobes_build_a_valid_curve():
    # doc limitation 2: overlapping in time but distinct node x's is not caught
    p = _params(plantar_pk=45.0, plantar_rise=15.0, plantar_fall=25.0,
                dorsi_pk=55.0, dorsi_rise=13.0, dorsi_fall=27.0)
    xs, _ = build_extended_nodes(p)
    assert xs != []
    assert all(b > a for a, b in zip(xs, xs[1:]))


def test_dorsi_peak_before_plantar_peak_is_expressible():
    p = _params(plantar_pk=70.0, dorsi_pk=20.0, dorsi_fall=8.0)
    xs, ys = interior_nodes(p)
    assert xs == sorted(xs)
    assert all(b > a for a, b in zip(xs, xs[1:]))


def test_plantar_lobe_wrapping_past_zero():
    # peak 8, rise 26 -> first node at -18 -> wraps to 82
    xs, _ = interior_nodes(_params(plantar_pk=8.0))
    assert min(xs) >= 0.0 and max(xs) < 100.0
    assert any(x > 50.0 for x in xs)   # the wrapped rise node landed in the 80s


def test_sample_curve_folds_percent_gait_modulo_100():
    a, _ = sample_curve(DEFAULT_PARAMS, [12.0])
    b, _ = sample_curve(DEFAULT_PARAMS, [112.0])
    assert a[0] == pytest.approx(b[0], abs=1e-9)


def test_clamp_param_enforces_bounds_and_integer_flags():
    assert clamp_param(0, 999.0) == 50.0      # torque max 50
    assert clamp_param(0, -999.0) == -50.0
    assert clamp_param(2, 150.0) == 100.0     # time max 100
    assert clamp_param(2, -3.0) == 0.0
    assert clamp_param(11, 0.4) == 0.0        # sim flag is integer-only
    assert clamp_param(11, 0.6) == 1.0
    assert clamp_param(13, 5.0) == 1.0        # pid flag clamped then rounded
    assert clamp_param(14, 20000.0) == 10000.0  # gain max
