"""Puts the SplineDesigner/ directory on sys.path so the tests can import the
designer modules (spline_math, spline_alt_math, ...) whether pytest is run from
the repo root or from inside SplineDesigner/."""

import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
