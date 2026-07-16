"""
Visualize the MonotonicSpline (PCHIP / Fritsch-Carlson) C++ class.

This is a direct port of spline_monotonic.hpp's math: weighted-harmonic-mean
interior tangents, shape-preserving three-point end tangents, and cubic
Hermite evaluation. Unlike the natural cubic spline, this never overshoots
the range of its neighboring y-values, so it won't produce artificial peaks
between flat/zero regions.

Supply nodes three ways:
  1. Interactively   -> just run the script with no arguments
  2. Command line     -> --x "0,25,50,75,100" --y "0,0,20,0,0"
  3. Edit the DEFAULT_X / DEFAULT_Y lists below and run directly
"""

import argparse
import numpy as np
import matplotlib.pyplot as plt


# Used only if you run the script with no CLI args and just hit Enter
# at the interactive prompts (i.e. these are the "defaults"). Chosen to
# show off the overshoot fix: two flat/zero regions flanking one bump.
DEFAULT_X = [0.0, 25.0, 50.0, 75.0, 100.0]
DEFAULT_Y = [0.0, 0.0, 20.0, 0.0, 0.0]


def _edge_case(h0, h1, m0, m1):
    """Shape-preserving three-point end-tangent formula (Fritsch-Carlson)."""
    d = ((2.0 * h0 + h1) * m0 - h0 * m1) / (h0 + h1)
    if (d >= 0.0) != (m0 >= 0.0):
        return 0.0
    if (m0 >= 0.0) != (m1 >= 0.0) and abs(d) > 3.0 * abs(m0):
        return 3.0 * m0
    return d


def compute_tangents(x, y):
    """Port of MonotonicSpline::set_points -- returns tangent m[i] at each node."""
    n = len(x)
    h = [x[i + 1] - x[i] for i in range(n - 1)]
    secant = [(y[i + 1] - y[i]) / h[i] for i in range(n - 1)]

    m = [0.0] * n

    if n == 2:
        m[0] = secant[0]
        m[1] = secant[0]
        return m

    # Interior tangents: weighted harmonic mean of adjacent secants, or
    # zero at local extrema / sign changes so the curve doesn't overshoot.
    for i in range(1, n - 1):
        m0 = secant[i - 1]
        m1 = secant[i]
        if m0 == 0.0 or m1 == 0.0 or (m0 > 0.0) != (m1 > 0.0):
            m[i] = 0.0
        else:
            w1 = 2.0 * h[i] + h[i - 1]
            w2 = h[i] + 2.0 * h[i - 1]
            m[i] = (w1 + w2) / (w1 / m0 + w2 / m1)

    m[0] = _edge_case(h[0], h[1], secant[0], secant[1])
    m[n - 1] = _edge_case(h[n - 2], h[n - 3], secant[n - 2], secant[n - 3])

    return m


def find_interval(x, t):
    """Binary search for the interval [x[k], x[k+1]] containing t."""
    n = len(x)
    lo, hi = 0, n - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if x[mid] <= t:
            lo = mid
        else:
            hi = mid
    return lo


def monotonic_spline_evaluate(x, y, m, t):
    """Port of MonotonicSpline::evaluate -- cubic Hermite eval at t."""
    if t <= x[0]:
        return y[0]
    if t >= x[-1]:
        return y[-1]

    k = find_interval(x, t)
    h = x[k + 1] - x[k]
    s = (t - x[k]) / h
    s2 = s * s
    s3 = s2 * s

    h00 = 2.0 * s3 - 3.0 * s2 + 1.0
    h10 = s3 - 2.0 * s2 + s
    h01 = -2.0 * s3 + 3.0 * s2
    h11 = s3 - s2

    return h00 * y[k] + h10 * h * m[k] + h01 * y[k + 1] + h11 * h * m[k + 1]


def parse_float_list(s):
    parts = [p for p in s.replace(",", " ").split() if p != ""]
    return [float(p) for p in parts]


def get_nodes_from_cli(args):
    if args.x is not None and args.y is not None:
        x = parse_float_list(args.x)
        y = parse_float_list(args.y)
        if len(x) != len(y):
            raise SystemExit(
                f"--x has {len(x)} values but --y has {len(y)} values; they must match."
            )
        if len(x) < 2:
            raise SystemExit("Need at least 2 nodes.")
        return x, y
    return None


def get_nodes_interactively():
    print("=== Spline node input ===")
    print(f"(Press Enter at any prompt to use the default {len(DEFAULT_X)}-node example)")

    n_str = input("Number of nodes: ").strip()
    if n_str == "":
        return list(DEFAULT_X), list(DEFAULT_Y)

    n = int(n_str)
    if n < 2:
        raise SystemExit("Need at least 2 nodes.")

    x, y = [], []
    for i in range(n):
        while True:
            try:
                xi = float(input(f"  x[{i}]: ").strip())
                yi = float(input(f"  y[{i}]: ").strip())
                x.append(xi)
                y.append(yi)
                break
            except ValueError:
                print("  Please enter a valid number.")
    return x, y


def main():
    parser = argparse.ArgumentParser(
        description="Visualize the MonotonicSpline (PCHIP) from spline_monotonic.hpp."
    )
    parser.add_argument("--x", type=str, default=None, help='Comma-separated x values')
    parser.add_argument("--y", type=str, default=None, help='Comma-separated y values')
    parser.add_argument(
        "--compare", action="store_true",
        help="Also plot the natural cubic spline for comparison",
    )
    parser.add_argument(
        "--output", type=str, default="/mnt/user-data/outputs/monotonic_spline_plot.png",
        help="Path to save the plot PNG",
    )
    args = parser.parse_args()

    nodes = get_nodes_from_cli(args)
    if nodes is None:
        x, y = get_nodes_interactively()
    else:
        x, y = nodes

    for i in range(1, len(x)):
        if x[i] <= x[i - 1]:
            raise SystemExit(
                f"x values must be strictly increasing; x[{i-1}]={x[i-1]} >= x[{i}]={x[i]}"
            )

    print(f"\nUsing {len(x)} nodes:")
    for xi, yi in zip(x, y):
        print(f"  ({xi}, {yi})")

    m = compute_tangents(x, y)

    span = x[-1] - x[0]
    pad = max(span * 0.1, 1.0)
    samples = np.linspace(x[0] - pad, x[-1] + pad, 600)
    values = [monotonic_spline_evaluate(x, y, m, float(t)) for t in samples]

    fig, ax = plt.subplots(figsize=(9, 5.5))

    ax.plot(samples, values, color="#16a34a", linewidth=2, label="Monotonic spline (PCHIP)")

    if args.compare:
        y2 = _natural_spline_y2(x, y)
        nat_values = [_natural_spline_eval(x, y, y2, float(t)) for t in samples]
        ax.plot(samples, nat_values, color="#dc2626", linewidth=2,
                 linestyle="--", alpha=0.8, label="Natural cubic spline (for comparison)")

    ax.scatter(x, y, color="black", zorder=5, s=60, label="Control points")

    ax.axvline(x[0], color="gray", linestyle="--", linewidth=1, alpha=0.6)
    ax.axvline(x[-1], color="gray", linestyle="--", linewidth=1, alpha=0.6)
    ax.axvspan(samples[0], x[0], color="gray", alpha=0.08)
    ax.axvspan(x[-1], samples[-1], color="gray", alpha=0.08)

    ax.set_title(f"Monotonic Cubic Spline (PCHIP) — {len(x)} nodes")
    ax.set_xlabel("percent_gait")
    ax.set_ylabel("interpolated value")
    ax.legend()
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    #fig.savefig(args.output, dpi=150)
    #print(f"\nSaved plot to {args.output}")
    plt.show()


# --- Natural spline, only used for --compare ---

def _natural_spline_y2(x, y):
    n = len(x)
    y2 = [0.0] * n
    u = [0.0] * (n - 1)
    for i in range(1, n - 1):
        sig = (x[i] - x[i - 1]) / (x[i + 1] - x[i - 1])
        p = sig * y2[i - 1] + 2.0
        y2[i] = (sig - 1.0) / p
        dd = (y[i + 1] - y[i]) / (x[i + 1] - x[i]) - (y[i] - y[i - 1]) / (x[i] - x[i - 1])
        u[i] = (6.0 * dd / (x[i + 1] - x[i - 1]) - sig * u[i - 1]) / p
    for k in range(n - 2, -1, -1):
        y2[k] = y2[k] * y2[k + 1] + u[k]
    return y2


def _natural_spline_eval(x, y, y2, t):
    n = len(x)
    if t <= x[0]:
        return y[0]
    if t >= x[-1]:
        return y[-1]
    k = 0
    for i in range(n - 1):
        if x[i] <= t <= x[i + 1]:
            k = i
            break
    h = x[k + 1] - x[k]
    a = (x[k + 1] - t) / h
    b = (t - x[k]) / h
    return a * y[k] + b * y[k + 1] + ((a**3 - a) * y2[k] + (b**3 - b) * y2[k + 1]) * (h * h) / 6.0


if __name__ == "__main__":
    main()
