"""In-memory parameter state for the SplineAlt tab: holds the 17 shape
parameters, clamps every write to spline_alt_bounds, and exposes the built node
set and a sampled preview curve.

Unlike NodeState there is no strictly-increasing structural invariant to enforce
here -- a degenerate parameter set (zero rise/fall, colliding lobes) is a valid
thing to type, and both spline_alt_math and the firmware report it by producing
no nodes / commanding zero torque. curve_samples() surfaces that via its `valid`
flag so the designer can warn instead of silently drawing a flat line.
"""

from spline_alt_math import (
    NUM_PARAMS,
    build_extended_nodes,
    clamp_param,
    interior_nodes,
    sample_curve,
)

X_BOUNDS = (0.0, 100.0)  # percent gait cycle, the periodic preview domain


class SplineAltState:
    def __init__(self, params):
        if len(params) != NUM_PARAMS:
            raise ValueError(
                f"expected {NUM_PARAMS} params, got {len(params)}"
            )
        self.params = [clamp_param(i, p) for i, p in enumerate(params)]

    def set_param(self, index, value):
        self.params[index] = clamp_param(index, value)

    def built_nodes(self):
        """The real (non-wrapped) nodes the firmware would build, for plotting.
        ([], []) when the profile is degenerate."""
        return interior_nodes(self.params)

    def extended_nodes(self):
        return build_extended_nodes(self.params)

    def curve_samples(self, sample_count=400):
        samples_x = [
            X_BOUNDS[0] + i * (X_BOUNDS[1] - X_BOUNDS[0]) / (sample_count - 1)
            for i in range(sample_count)
        ]
        samples_y, valid = sample_curve(self.params, samples_x)
        return samples_x, samples_y, valid
