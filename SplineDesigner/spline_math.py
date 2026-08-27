"""
Python port of the firmware's spline-controller interpolation
(ExoCode/src/Controller.cpp, Spline::_pchip_interpolate / _pchip_edge_tangent).

Kept in sync by hand with the firmware algorithm so this tool's preview plot matches
what the exoskeleton will actually command. If the firmware interpolation changes,
update this module to match. This is a deliberate duplicate of
Python_GUI/utils/spline_math.py — SplineDesigner has no dependency on Python_GUI, so
both copies must be updated together if the firmware algorithm ever changes.
"""

# Matches ExoCode/src/ControllerData.cpp spline_bounds (per-node bounds).
NODE_X_BOUNDS = (0.0, 100.0)   # percent gait cycle
NODE_Y_BOUNDS = (-100.0, 100.0)  # Nm


def _pchip_edge_tangent(h0, h1, m0, m1):
    """Port of Spline::_pchip_edge_tangent (Controller.cpp)."""
    d = ((2.0 * h0 + h1) * m0 - h0 * m1) / (h0 + h1)
    sign_d = d >= 0.0
    sign_m0 = m0 >= 0.0
    if sign_d != sign_m0:
        return 0.0
    sign_m1 = m1 >= 0.0
    if sign_m0 != sign_m1 and abs(d) > 3.0 * abs(m0):
        return 3.0 * m0
    return d


def nodes_strictly_increasing(x_nodes):
    """Port of the ordering guard at the top of Spline::_pchip_interpolate."""
    return all(x_nodes[i] > x_nodes[i - 1] for i in range(1, len(x_nodes)))


def pchip_curve(x_nodes, y_nodes, t_samples):
    """Port of Spline::_pchip_interpolate, evaluated at each of t_samples.

    Returns (y_values, valid). valid is False when x_nodes is not strictly
    increasing, matching the firmware's "return 0.0f for everything" behavior
    for an invalid/unordered node configuration.
    """
    n = len(x_nodes)
    if n < 3 or not nodes_strictly_increasing(x_nodes):
        return [0.0] * len(t_samples), False

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

    def eval_one(t):
        if t <= x_nodes[0]:
            return y_nodes[0]
        if t >= x_nodes[n - 1]:
            return y_nodes[n - 1]
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

    return [eval_one(t) for t in t_samples], True
