How to switch the Spline controller from natural cubic to PCHIP

Background — what's there today
--------------------------------
The live "Spline" controller (used by hip/ankle/arm_1/arm_2) is defined in:
- ExoCode/src/Controller.h:227-237 (class Spline)
- ExoCode/src/Controller.cpp:832-895 (Spline::calc_motor_cmd)
- ExoCode/src/Controller.cpp:897-963 (Spline::_spline_interpolate)

`_spline_interpolate(x, y, percent_gait)` takes the 5 fixed (x, y) nodes read
from parameters, solves a natural cubic spline for the second derivatives
`y2[]` via a tridiagonal sweep (natural boundary condition: `y2[0] = y2[4] = 0`),
then evaluates the standard cubic-spline piecewise formula. Natural cubic
splines can overshoot ("ring") past the y-range of neighboring nodes — e.g. a
sharp peak node can make the curve dip below zero just after it, which is
undesirable for a torque profile.

PCHIP (Piecewise Cubic Hermite Interpolating Polynomial, Fritsch-Carlson
method) fixes this: it's shape-preserving, so the curve never overshoots the
range of its neighboring y-values, and stays flat/monotonic where the data is.
A reference implementation already exists in this repo at:
- `Test results/spline_pchip.hpp` (class `MonotonicSpline`)
- `Test results/spline_pchip_visualize.py` (Python script that plots it
  against scipy's PchipInterpolator and the existing cubic spline to sanity
  check the math)

Important adaptation needed before porting
-------------------------------------------
`MonotonicSpline` in `spline_pchip.hpp` uses `std::vector<float>` and dynamic
sizing. The firmware almost never uses `std::vector` (the only hit in
`ExoCode/src` is `Time_Helper.h`) because this runs on a Teensy and the
project avoids heap churn in the control loop. `_spline_interpolate` uses
fixed `float[5]` C-arrays and stack-only locals — port the PCHIP math to that
same shape rather than dropping the vector-based class in directly.

Step-by-step
------------

1. In `ExoCode/src/Controller.h`, find the `Spline` class (line 227-237):

       class Spline: public _Controller
       {
           public:
               Spline(config_defs::joint_id id, ExoData* exo_data);
               ~Spline(){};

               float calc_motor_cmd();

           private:
               float _spline_interpolate(const float* x, const float* y, float percent_gait);
       };

   Replace the `private:` line with these three lines (this keeps
   `_spline_interpolate` declared for now so you can A/B the two — delete it
   in step 4 once you trust the new one):

       private:
               float _spline_interpolate(const float* x, const float* y, float percent_gait);
               float _pchip_interpolate(const float* x, const float* y, float percent_gait);
               static float _pchip_edge_tangent(float h0, float h1, float m0, float m1);

2. In `ExoCode/src/Controller.cpp`, paste this whole block immediately after
   the closing brace of `Spline::_spline_interpolate` (i.e. right after line
   963, before the `//****************************************************`
   separator / `FranksCollinsHip` section):

       float Spline::_pchip_edge_tangent(float h0, float h1, float m0, float m1)
       {
           float d = ((2.0f * h0 + h1) * m0 - h0 * m1) / (h0 + h1);
           const bool sign_d = d >= 0.0f;
           const bool sign_m0 = m0 >= 0.0f;
           if (sign_d != sign_m0)
           {
               return 0.0f;
           }
           const bool sign_m1 = m1 >= 0.0f;
           if (sign_m0 != sign_m1 && fabsf(d) > 3.0f * fabsf(m0))
           {
               return 3.0f * m0;
           }
           return d;
       }

       float Spline::_pchip_interpolate(const float* x, const float* y, float percent_gait)
       {
           const int n = 5;

           for (int i = 1; i < n; ++i)
           {
               if (x[i] <= x[i - 1])
               {
                   return 0.0f;
               }
           }

           if (percent_gait <= x[0])
           {
               return y[0];
           }
           if (percent_gait >= x[n - 1])
           {
               return y[n - 1];
           }

           float h[n - 1];
           float secant[n - 1];
           for (int i = 0; i < n - 1; ++i)
           {
               h[i] = x[i + 1] - x[i];
               secant[i] = (y[i + 1] - y[i]) / h[i];
           }

           float m[n];
           for (int i = 1; i < n - 1; ++i)
           {
               const float m0 = secant[i - 1];
               const float m1 = secant[i];
               if (m0 == 0.0f || m1 == 0.0f || (m0 > 0.0f) != (m1 > 0.0f))
               {
                   m[i] = 0.0f;
               }
               else
               {
                   const float w1 = 2.0f * h[i] + h[i - 1];
                   const float w2 = h[i] + 2.0f * h[i - 1];
                   m[i] = (w1 + w2) / (w1 / m0 + w2 / m1);
               }
           }
           m[0] = _pchip_edge_tangent(h[0], h[1], secant[0], secant[1]);
           m[n - 1] = _pchip_edge_tangent(h[n - 2], h[n - 3], secant[n - 2], secant[n - 3]);

           int k = 0;
           for (int i = 0; i < n - 1; ++i)
           {
               if (percent_gait >= x[i] && percent_gait <= x[i + 1])
               {
                   k = i;
                   break;
               }
           }

           const float h_k = x[k + 1] - x[k];
           const float s = (percent_gait - x[k]) / h_k;
           const float s2 = s * s;
           const float s3 = s2 * s;

           const float h00 = 2.0f * s3 - 3.0f * s2 + 1.0f;
           const float h10 = s3 - 2.0f * s2 + s;
           const float h01 = -2.0f * s3 + 3.0f * s2;
           const float h11 = s3 - s2;

           return h00 * y[k] + h10 * h_k * m[k]
                + h01 * y[k + 1] + h11 * h_k * m[k + 1];
       }

   This is the fixed-array (n = 5, no `std::vector`, no heap allocation) port
   of `spline_pchip.hpp`'s `MonotonicSpline::set_points` + `evaluate`, with
   the "hunt last interval" optimization dropped since `_spline_interpolate`
   doesn't have one either and a 4-segment linear scan is cheap every call.

3. Verify the port against the reference before touching the call site.
   Temporarily write a tiny standalone test (desktop C++, not on the Teensy)
   that feeds the same node arrays through both the new fixed-array
   `_pchip_interpolate` logic and `spline_pchip.hpp`'s `MonotonicSpline`, and
   confirm they match at a handful of `percent_gait` values. Cross-check
   shape visually with `spline_pchip_visualize.py` (adjust its input nodes to
   match your actual `spline.csv`, e.g.
   `SDCard/ankleControllers/spline.csv`).

4. Switch the call site.
   In `Spline::calc_motor_cmd()` (Controller.cpp:864), change:

       float torque_cmd = _spline_interpolate(x, y, percent_gait);

   to:

       float torque_cmd = _pchip_interpolate(x, y, percent_gait);

   Once you're confident in step 3, delete `_spline_interpolate` and its
   declaration (Controller.h:236) to avoid dead code, or leave it only if you
   want a runtime/compile-time toggle between the two (not required by "use
   PCHIP instead of cubic").

5. Compile.
   Open `ExoCode/ExoCode.ino` in the Arduino IDE (or your usual Teensy build
   command) and verify it compiles clean for the target board
   (`Config.h` → `BOARD_VERSION`). PCHIP needs no extra libraries — it's
   pure `<cmath>` arithmetic like the existing cubic spline.

6. Bench-test before trusting it on hardware.
   Same precautions as the existing controller (see
   `Useful guides by us/Using_spline_controller.md`):
   - Set `sim_gait = 1` in `spline.csv` first and watch the commanded torque
     sweep on the bench/GUI plot; confirm it hugs the node values with no
     dip/overshoot past a peak node (the exact symptom PCHIP fixes).
   - Re-confirm output stays within the existing ±15 Nm clamp
     (Controller.cpp:865-872) — that clamp is unchanged by this swap.
   - Do a direction calibration pass before running on a person — the sign
     convention and clamp are unaffected by which spline math is used, but
     it's good practice per that guide's issue #7.
   - Once bench behavior looks right, set `sim_gait = 0` and test with real
     gait as usual.

7. Optional cleanup.
   `Test results/spline_pchip.hpp` and `spline_pchip_visualize.py` were a
   prototype/scratch space for this — once the fixed-array version is ported
   into `Controller.cpp`/`Controller.h` and confirmed working, they can stay
   here as reference/test material (this is the Test results folder) or be
   deleted if you'd rather keep only the final ported code.
