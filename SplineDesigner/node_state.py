"""In-memory node state for the spline designer: holds the 12 node
positions and 6 extra parameters, and enforces the firmware's strictly-
increasing-X rule structurally (both drag and type go through set_x, so an
invalid ordering is never reachable through the UI).

Assumes the node x-values it's constructed with are already spaced at least
2 * X_EPSILON apart, which holds for any spline.csv already used with real
gait-cycle percentages.
"""

from spline_math import NODE_X_BOUNDS, NODE_Y_BOUNDS, pchip_curve

X_EPSILON = 0.1  # % gait -- minimum gap enforced between adjacent node x-values


class NodeState:
    def __init__(self, x_nodes, y_nodes, extra_params):
        if len(x_nodes) != len(y_nodes):
            raise ValueError(
                f"x_nodes and y_nodes must be the same length, got {len(x_nodes)} and {len(y_nodes)}"
            )
        self.x_nodes = list(x_nodes)
        self.y_nodes = list(y_nodes)
        self.extra_params = list(extra_params)

    def _clamp_x(self, index, new_x):
        lo = NODE_X_BOUNDS[0] if index == 0 else self.x_nodes[index - 1] + X_EPSILON
        hi = (
            NODE_X_BOUNDS[1]
            if index == len(self.x_nodes) - 1
            else self.x_nodes[index + 1] - X_EPSILON
        )
        return max(lo, min(new_x, hi))

    def _clamp_y(self, new_y):
        return max(NODE_Y_BOUNDS[0], min(new_y, NODE_Y_BOUNDS[1]))

    def set_x(self, index, new_x):
        self.x_nodes[index] = self._clamp_x(index, new_x)

    def set_y(self, index, new_y):
        self.y_nodes[index] = self._clamp_y(new_y)

    def curve_samples(self, sample_count=200):
        samples_x = [
            NODE_X_BOUNDS[0] + i * (NODE_X_BOUNDS[1] - NODE_X_BOUNDS[0]) / (sample_count - 1)
            for i in range(sample_count)
        ]
        samples_y, valid = pchip_curve(self.x_nodes, self.y_nodes, samples_x)
        return samples_x, samples_y, valid
