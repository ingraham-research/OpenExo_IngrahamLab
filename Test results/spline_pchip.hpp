#pragma once

#include <vector>
#include <cmath>
#include <cassert>

// Monotonic cubic Hermite spline (PCHIP / Fritsch-Carlson method).
//
// Same interface as the natural-spline Spline class (set_points / evaluate),
// but chooses tangents at each node so the curve never overshoots the range
// of its neighboring y-values. Segments that are locally flat or monotonic
// in the data stay that way -- no "ringing" between control points.
//
// Usage is identical to the natural-spline version:
//   MonotonicSpline spline;
//   spline.set_points(x, y);          // O(n), call once (or whenever points change)
//   float v = spline.evaluate(t);     // O(log n) per call (O(1) amortized with hunt)
class MonotonicSpline
{
public:
    bool set_points(const std::vector<float>& x, const std::vector<float>& y)
    {
        if (x.size() != y.size() || x.size() < 2)
        {
            return false;
        }
        for (size_t i = 1; i < x.size(); ++i)
        {
            if (x[i] <= x[i - 1])
            {
                return false;
            }
        }

        x_ = x;
        y_ = y;
        const size_t n = x_.size();

        // Segment widths and secant slopes.
        std::vector<float> h(n - 1);
        std::vector<float> secant(n - 1);
        for (size_t i = 0; i < n - 1; ++i)
        {
            h[i] = x_[i + 1] - x_[i];
            secant[i] = (y_[i + 1] - y_[i]) / h[i];
        }

        m_.assign(n, 0.0f);

        if (n == 2)
        {
            // Only one segment: a straight line, tangents equal the secant.
            m_[0] = secant[0];
            m_[1] = secant[0];
        }
        else
        {
            // Interior tangents: weighted harmonic mean of the two adjacent
            // secant slopes, or zero at local extrema / sign changes so the
            // curve doesn't overshoot past a peak or valley in the data.
            for (size_t i = 1; i < n - 1; ++i)
            {
                const float m0 = secant[i - 1];
                const float m1 = secant[i];
                if (m0 == 0.0f || m1 == 0.0f || (m0 > 0.0f) != (m1 > 0.0f))
                {
                    m_[i] = 0.0f;
                }
                else
                {
                    const float w1 = 2.0f * h[i] + h[i - 1];
                    const float w2 = h[i] + 2.0f * h[i - 1];
                    m_[i] = (w1 + w2) / (w1 / m0 + w2 / m1);
                }
            }

            // Endpoint tangents: shape-preserving non-centered three-point
            // formula (same end condition scipy's PchipInterpolator uses).
            m_[0] = edge_case(h[0], h[1], secant[0], secant[1]);
            m_[n - 1] = edge_case(h[n - 2], h[n - 3], secant[n - 2], secant[n - 3]);
        }

        last_interval_ = 0;
        return true;
    }

    float evaluate(float t) const
    {
        const size_t n = x_.size();
        assert(n >= 2 && "call set_points() before evaluate()");

        if (t <= x_.front())
        {
            return y_.front();
        }
        if (t >= x_.back())
        {
            return y_.back();
        }

        const size_t k = find_interval(t);
        const float h = x_[k + 1] - x_[k];
        const float s = (t - x_[k]) / h;
        const float s2 = s * s;
        const float s3 = s2 * s;

        // Cubic Hermite basis functions.
        const float h00 = 2.0f * s3 - 3.0f * s2 + 1.0f;
        const float h10 = s3 - 2.0f * s2 + s;
        const float h01 = -2.0f * s3 + 3.0f * s2;
        const float h11 = s3 - s2;

        return h00 * y_[k] + h10 * h * m_[k]
             + h01 * y_[k + 1] + h11 * h * m_[k + 1];
    }

private:
    // Shape-preserving three-point end-tangent formula (Fritsch-Carlson).
    // h0, m0 belong to the segment touching the boundary; h1, m1 to the
    // next segment inward.
    static float edge_case(float h0, float h1, float m0, float m1)
    {
        float d = ((2.0f * h0 + h1) * m0 - h0 * m1) / (h0 + h1);
        const bool sign_d = d >= 0.0f;
        const bool sign_m0 = m0 >= 0.0f;
        if (sign_d != sign_m0)
        {
            return 0.0f;
        }
        const bool sign_m1 = m1 >= 0.0f;
        if (sign_m0 != sign_m1 && std::fabs(d) > 3.0f * std::fabs(m0))
        {
            return 3.0f * m0;
        }
        return d;
    }

    // Binary search with a cheap "hunt" check of the last-used interval
    // first -- fast when t increases monotonically call-to-call, as in a
    // gait cycle.
    size_t find_interval(float t) const
    {
        const size_t n = x_.size();

        if (t >= x_[last_interval_] && t <= x_[last_interval_ + 1])
        {
            return last_interval_;
        }

        size_t lo = 0;
        size_t hi = n - 1;
        while (hi - lo > 1)
        {
            const size_t mid = (lo + hi) / 2;
            if (x_[mid] <= t)
            {
                lo = mid;
            }
            else
            {
                hi = mid;
            }
        }

        last_interval_ = lo;
        return lo;
    }

    std::vector<float> x_;
    std::vector<float> y_;
    std::vector<float> m_;  // tangent (slope) at each node
    mutable size_t last_interval_ = 0;
};
