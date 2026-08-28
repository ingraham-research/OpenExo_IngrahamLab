"""
Python port of the firmware's SplineAlt controller: the shape-parameter node
builder and the periodic PCHIP interpolator
(ExoCode/src/Controller.cpp, SplineAlt::_build_nodes / _pchip_interpolate /
_pchip_edge_tangent as of commit c22eb053).

Kept in sync BY HAND with the firmware algorithm so the SplineAlt tab's preview
plot matches what the exoskeleton will actually command. If the firmware
algorithm changes, update this module to match.

SplineAlt differs from Spline (see spline_math.py) in two ways:
  1. The 12 nodes are BUILT from 17 shape parameters instead of read as (x, y)
     pairs.
  2. The spline is PERIODIC across the 0/100 percent-gait seam -- a lobe may wrap
     around the end of the gait cycle -- so _pchip_interpolate takes a variable
     node count and the node set is extended by two periodic copies on each side.

This is a deliberate second copy of the Fritsch-Carlson PCHIP math, mirroring the
deliberate duplication in the firmware (Controller.cpp carries a banner comment
explaining why Spline's file was left untouched).
"""

import math

# Matches ExoCode/src/ControllerData.h namespace spline_alt (17 indices).
PLANTAR_NM_IDX = 0        # peak plantarflexion magnitude, Nm (applied NEGATIVE)
DORSI_NM_IDX = 1          # peak dorsiflexion magnitude, Nm (applied POSITIVE)
PLANTAR_PK_IDX = 2        # percent gait at which plantar torque first peaks
DORSI_PK_IDX = 3          # percent gait at which dorsi torque first peaks
PLANTAR_RISE_IDX = 4      # duration 0 -> peak
PLANTAR_DWELL_IDX = 5     # duration held AT peak (0 = no plateau)
PLANTAR_FALL_IDX = 6      # duration peak -> 0
DORSI_RISE_IDX = 7
DORSI_DWELL_IDX = 8
DORSI_FALL_IDX = 9
TORQUE_SCALE_IDX = 10     # 0-100 %, applied to both lobes
SIM_GAIT_IDX = 11
USE_PERCENT_GAIT_IDX = 12
USE_PID_IDX = 13
P_GAIN_IDX = 14
I_GAIN_IDX = 15
D_GAIN_IDX = 16
NUM_PARAMS = 17

# Largest node count SplineAlt::_build_nodes can emit: 2 lobes x 4 nodes, minus
# any collapsed dwell nodes, plus the 2 + 2 periodic wrap copies.
MAX_NODES = 12

# Matches ExoCode/src/ControllerData.cpp spline_alt_bounds[]. Each entry is
# (min, max, integer_only). The magnitude bounds are deliberately loose (+/-50);
# the firmware still applies Spline's +/-15 Nm feed-forward clamp downstream.
PARAM_BOUNDS = [
    (-50.0, 50.0, False),      # 0  plantar_nm
    (-50.0, 50.0, False),      # 1  dorsi_nm
    (0.0, 100.0, False),       # 2  plantar_pk
    (0.0, 100.0, False),       # 3  dorsi_pk
    (0.0, 100.0, False),       # 4  plantar_rise
    (0.0, 100.0, False),       # 5  plantar_dwell
    (0.0, 100.0, False),       # 6  plantar_fall
    (0.0, 100.0, False),       # 7  dorsi_rise
    (0.0, 100.0, False),       # 8  dorsi_dwell
    (0.0, 100.0, False),       # 9  dorsi_fall
    (0.0, 100.0, False),       # 10 torque_scale
    (0.0, 1.0, True),          # 11 sim_gait
    (0.0, 1.0, True),          # 12 use_percent_gait
    (0.0, 1.0, True),          # 13 use_pid
    (0.0, 10000.0, False),     # 14 p_gain
    (0.0, 10000.0, False),     # 15 i_gain
    (0.0, 10000.0, False),     # 16 d_gain
]

# Firmware feed-forward clamp in SplineAlt::calc_motor_cmd (informational -- not
# applied by this module, which mirrors spline_math.py's pure-interpolation scope).
FEED_FORWARD_CLAMP_NM = 15.0


def clamp_param(index, value):
    """Clamp one parameter to spline_alt_bounds, rounding integer-only flags."""
    lo, hi, integer_only = PARAM_BOUNDS[index]
    v = max(lo, min(float(value), hi))
    if integer_only:
        v = float(round(v))
    return v


def _wrap_mod_100(v):
    """Port of `fmodf(v, 100); if (v < 0) v += 100;` from _build_nodes."""
    v = math.fmod(v, 100.0)
    if v < 0.0:
        v += 100.0
    return v


def _build_raw_lobe_nodes(params):
    """The per-lobe node emission from SplineAlt::_build_nodes, before wrap/sort.

    Returns (nx, ny) lists (up to 8 entries). A lobe with amplitude exactly 0
    emits nothing; a lobe with dwell 0 skips its plateau node.
    """
    scale = params[TORQUE_SCALE_IDX] / 100.0
    amp = [
        -params[PLANTAR_NM_IDX] * scale,   # plantarflexion applied negative
        params[DORSI_NM_IDX] * scale,
    ]
    peak = [params[PLANTAR_PK_IDX], params[DORSI_PK_IDX]]
    rise = [params[PLANTAR_RISE_IDX], params[DORSI_RISE_IDX]]
    dwell = [params[PLANTAR_DWELL_IDX], params[DORSI_DWELL_IDX]]
    fall = [params[PLANTAR_FALL_IDX], params[DORSI_FALL_IDX]]

    nx, ny = [], []
    for lobe in range(2):
        if amp[lobe] == 0.0:
            continue  # direction disabled -- contributes nothing, cannot collide
        nx.append(peak[lobe] - rise[lobe]);            ny.append(0.0)
        nx.append(peak[lobe]);                         ny.append(amp[lobe])
        if dwell[lobe] > 0.0:
            nx.append(peak[lobe] + dwell[lobe]);       ny.append(amp[lobe])
        nx.append(peak[lobe] + dwell[lobe] + fall[lobe]); ny.append(0.0)
    return nx, ny


def _interior_nodes(params):
    """Raw lobe nodes wrapped mod 100 and sorted by x -- the "real" node set
    before periodic extension. Returns ([], []) for a degenerate profile
    (fewer than 3 nodes, or a non-strictly-increasing x after sorting: an exact
    lobe collision, a zero rise/fall, or a lobe longer than a full cycle)."""
    nx, ny = _build_raw_lobe_nodes(params)
    if len(nx) < 3:
        return [], []

    nx = [_wrap_mod_100(x) for x in nx]
    pairs = sorted(zip(nx, ny), key=lambda p: p[0])
    nx = [p[0] for p in pairs]
    ny = [p[1] for p in pairs]

    for i in range(1, len(nx)):
        if nx[i] <= nx[i - 1]:
            return [], []
    return nx, ny


def interior_nodes(params):
    """Public: the real (non-wrapped) node set the profile builds, sorted by x.
    ([], []) when the profile is degenerate."""
    return _interior_nodes(params)


def build_extended_nodes(params):
    """Port of SplineAlt::_build_nodes: the interior node set plus two periodic
    copies of the tail before the start and two of the head after the end
    (this is what _pchip_interpolate consumes). Returns ([], []) when the
    profile is degenerate."""
    nx, ny = _interior_nodes(params)
    if not nx:
        return [], []
    n = len(nx)
    x = [nx[n - 2] - 100.0, nx[n - 1] - 100.0]
    y = [ny[n - 2], ny[n - 1]]
    x.extend(nx)
    y.extend(ny)
    x.extend([nx[0] + 100.0, nx[1] + 100.0])
    y.extend([ny[0], ny[1]])
    return x, y


def _pchip_edge_tangent(h0, h1, m0, m1):
    """Port of SplineAlt::_pchip_edge_tangent (identical to Spline's)."""
    d = ((2.0 * h0 + h1) * m0 - h0 * m1) / (h0 + h1)
    sign_d = d >= 0.0
    sign_m0 = m0 >= 0.0
    if sign_d != sign_m0:
        return 0.0
    sign_m1 = m1 >= 0.0
    if sign_m0 != sign_m1 and abs(d) > 3.0 * abs(m0):
        return 3.0 * m0
    return d


def pchip_interpolate(x_nodes, y_nodes, t):
    """Port of SplineAlt::_pchip_interpolate: Fritsch-Carlson PCHIP evaluated at
    a single t, with the node count taken from len(x_nodes). Returns 0.0 for a
    node set that is too small/large or not strictly increasing, matching the
    firmware's all-or-nothing failure."""
    n = len(x_nodes)
    if n < 3 or n > MAX_NODES:
        return 0.0
    for i in range(1, n):
        if x_nodes[i] <= x_nodes[i - 1]:
            return 0.0

    if t <= x_nodes[0]:
        return y_nodes[0]
    if t >= x_nodes[n - 1]:
        return y_nodes[n - 1]

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


def sample_curve(params, t_samples):
    """Evaluate the SplineAlt torque profile at each t in t_samples.

    Mirrors SplineAlt::calc_motor_cmd: each percent-gait value is folded onto the
    periodic [0, 100) domain before interpolation. Returns (y_values, valid);
    valid is False for a degenerate profile, in which case every y is 0.0 (the
    firmware commands zero torque)."""
    x_nodes, y_nodes = build_extended_nodes(params)
    if not x_nodes:
        return [0.0] * len(t_samples), False
    out = []
    for t in t_samples:
        out.append(pchip_interpolate(x_nodes, y_nodes, _wrap_mod_100(t)))
    return out, True
