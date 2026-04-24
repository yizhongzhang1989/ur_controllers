"""Unit tests for the R2 signal-analysis helpers.

Pin the measured-signal side of the R2 assertion vocabulary
(``tests/integration/signal_analysis.py``) so the R2 integration tests
(M6.12–M6.14, still gated on M6.0) can be authored against a stable
surface. Matches the style of ``test_expectations_loader.py``: pure
stdlib, synthetic signals with known ground truth, no ROS.

Coverage:

* ``steady_state_error``, ``drift_peak_to_peak``, ``velocity_rms``
  on trailing windows, including the window-covers-too-few-samples
  and non-monotonic-time error paths.
* ``find_extrema`` on step, sine, and plateau inputs.
* ``damping_ratio_from_step`` recovers ``zeta`` from a synthesized
  underdamped response, returns ``None`` on overdamped / flat input,
  and rejects bad ``min_amplitude``.
* ``detect_limit_cycle`` finds an injected 5 Hz oscillation in the
  requested band, reports no detection on clean noise, and raises on
  non-uniform sampling.
"""

from __future__ import annotations

import importlib.util
import math
import random
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
MOD_PATH = REPO_ROOT / "tests" / "integration" / "signal_analysis.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("_signal_analysis", MOD_PATH)
    assert spec and spec.loader
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


SA = _load_module()


# ---------------------------------------------------------------------------
# Validation / shared error paths
# ---------------------------------------------------------------------------


def test_length_mismatch_raises():
    with pytest.raises(ValueError, match="length mismatch"):
        SA.steady_state_error([0.0, 0.1], [0.0], 0.0, window_s=0.1)


def test_non_monotonic_time_raises():
    with pytest.raises(ValueError, match="monotonic"):
        SA.drift_peak_to_peak([0.0, 0.2, 0.1], [1.0, 2.0, 3.0], window_s=0.2)


def test_window_nonpositive_raises():
    with pytest.raises(ValueError, match="window_s must be > 0"):
        SA.velocity_rms([0.0, 0.1, 0.2], [0.0, 0.0, 0.0], window_s=0.0)


def test_trailing_window_too_short_raises():
    # Only the last sample is in the window ⇒ fewer than 2 samples.
    t = [0.0, 1.0, 2.0]
    v = [1.0, 2.0, 3.0]
    with pytest.raises(ValueError, match="fewer than 2"):
        SA.steady_state_error(t, v, 3.0, window_s=0.1)


# ---------------------------------------------------------------------------
# Scalar metrics on trailing windows
# ---------------------------------------------------------------------------


def test_steady_state_error_constant_signal():
    t = [i * 0.01 for i in range(201)]  # 2 s at 100 Hz
    v = [0.5] * len(t)
    assert SA.steady_state_error(t, v, target=0.5, window_s=0.5) == pytest.approx(0.0)
    assert SA.steady_state_error(t, v, target=0.3, window_s=0.5) == pytest.approx(0.2)


def test_steady_state_error_ignores_transient():
    # Huge transient early, then settled at 1.0. Window of 0.5s picks up
    # only the settled tail.
    t = [i * 0.01 for i in range(201)]
    v = [10.0 if ti < 1.0 else 1.0 for ti in t]
    assert SA.steady_state_error(t, v, target=1.0, window_s=0.5) == pytest.approx(0.0)


def test_drift_peak_to_peak():
    t = [i * 0.01 for i in range(201)]
    # Oscillates ±0.05 about 1.0 in the last second.
    v = [1.0 + (0.05 if (i % 2) else -0.05) if t[i] >= 1.0 else 1.0 for i in range(len(t))]
    assert SA.drift_peak_to_peak(t, v, window_s=0.9) == pytest.approx(0.1, abs=1e-9)


def test_velocity_rms_constant_and_sine():
    # Constant velocity ⇒ RMS equals the constant value.
    t = [i * 0.01 for i in range(201)]
    v = [0.3] * len(t)
    assert SA.velocity_rms(t, v, window_s=1.0) == pytest.approx(0.3, abs=1e-9)

    # Sine wave amplitude A ⇒ RMS ≈ A / sqrt(2) over many periods.
    amp = 0.4
    v = [amp * math.sin(2.0 * math.pi * 5.0 * ti) for ti in t]
    assert SA.velocity_rms(t, v, window_s=2.0) == pytest.approx(amp / math.sqrt(2.0), rel=0.05)


# ---------------------------------------------------------------------------
# Extrema / damping ratio
# ---------------------------------------------------------------------------


def test_find_extrema_sine():
    # f = 2 Hz over 1 s at 1 kHz ⇒ exactly 2 maxima and 2 minima strictly inside.
    t = [i * 1e-3 for i in range(1001)]
    v = [math.sin(2.0 * math.pi * 2.0 * ti) for ti in t]
    peaks = SA.find_extrema(t, v)
    # Four interior extrema expected (peaks at 0.125, 0.625 and troughs
    # at 0.375, 0.875 — endpoints excluded).
    assert len(peaks) == 4
    signs = [1 if p[1] > 0 else -1 for p in peaks]
    assert signs == [1, -1, 1, -1]


def test_find_extrema_min_abs_gates_noise():
    t = [i * 1e-3 for i in range(1001)]
    v = [0.001 * math.sin(2.0 * math.pi * 50.0 * ti) for ti in t]
    assert SA.find_extrema(t, v, min_abs=0.01) == []


def test_damping_ratio_recovers_known_zeta():
    # Underdamped second-order step response about target 1.0:
    #   q(t) = 1 + (q0 - 1) * exp(-zeta * wn * t) * (
    #          cos(wd t) + zeta/sqrt(1-zeta^2) * sin(wd t) )  -- but for
    # the log-decrement we only need e(t) = q(t) - target to decay as
    # exp(-zeta*wn*t)*cos(wd*t + phi); half-cycle log-dec is invariant
    # to the initial phase. Use the simplest closed form.
    zeta_true = 0.12
    wn = 2.0 * math.pi * 3.0  # 3 Hz natural frequency
    wd = wn * math.sqrt(1.0 - zeta_true * zeta_true)
    target = 1.0
    A0 = 0.5  # initial error magnitude
    t = [i * 1e-3 for i in range(5001)]  # 5 s at 1 kHz
    v = [target + A0 * math.exp(-zeta_true * wn * ti) * math.cos(wd * ti) for ti in t]
    zeta_est = SA.damping_ratio_from_step(t, v, target=target)
    assert zeta_est is not None
    # Half-cycle log-dec is exact for this signal up to sampling
    # resolution; allow 5% relative error against ground truth.
    assert zeta_est == pytest.approx(zeta_true, rel=0.05)


def test_damping_ratio_overdamped_returns_none():
    # Pure exponential decay, no oscillation ⇒ no second extremum.
    t = [i * 1e-3 for i in range(2001)]
    v = [1.0 + 0.5 * math.exp(-5.0 * ti) for ti in t]
    assert SA.damping_ratio_from_step(t, v, target=1.0) is None


def test_damping_ratio_flat_signal_returns_none():
    t = [i * 1e-3 for i in range(101)]
    v = [0.7] * len(t)
    assert SA.damping_ratio_from_step(t, v, target=0.7) is None


def test_damping_ratio_rejects_negative_min_amplitude():
    t = [i * 1e-3 for i in range(10)]
    v = [float(i) for i in range(10)]
    with pytest.raises(ValueError, match="min_amplitude must be >= 0"):
        SA.damping_ratio_from_step(t, v, target=0.0, min_amplitude=-0.1)


# ---------------------------------------------------------------------------
# Limit-cycle / FFT detector
# ---------------------------------------------------------------------------


def test_detect_limit_cycle_finds_injected_peak():
    # 2 seconds at 250 Hz, 5 Hz pure sine with amplitude 0.1 + small DC
    # offset.  Look in 1..50 Hz band.
    fs = 250.0
    n = 500
    t = [i / fs for i in range(n)]
    v = [0.02 + 0.1 * math.sin(2.0 * math.pi * 5.0 * ti) for ti in t]
    detected, peak_hz, ratio = SA.detect_limit_cycle(
        t, v, f_min_hz=1.0, f_max_hz=50.0, peak_ratio=5.0
    )
    assert detected
    assert peak_hz is not None
    assert peak_hz == pytest.approx(5.0, abs=1.0 / (t[-1] - t[0]) + 0.01)
    assert ratio > 5.0


def test_detect_limit_cycle_noise_only_no_detection():
    # White noise ⇒ no coherent peak ⇒ detector should stay quiet at a
    # reasonably high ``peak_ratio``.
    rng = random.Random(42)
    fs = 250.0
    n = 500
    t = [i / fs for i in range(n)]
    v = [rng.gauss(0.0, 0.01) for _ in range(n)]
    detected, _, ratio = SA.detect_limit_cycle(t, v, f_min_hz=1.0, f_max_hz=50.0, peak_ratio=10.0)
    assert not detected
    assert ratio < 10.0


def test_detect_limit_cycle_rejects_nonuniform_sampling():
    t = [0.0, 0.01, 0.05, 0.06, 0.10]  # jitter far beyond 10% of median
    v = [0.0, 1.0, -1.0, 1.0, -1.0]
    with pytest.raises(ValueError, match="jitter"):
        SA.detect_limit_cycle(t, v, f_min_hz=1.0, f_max_hz=40.0, peak_ratio=5.0)


def test_detect_limit_cycle_empty_band_returns_false():
    # Band entirely above Nyquist ⇒ no bins ⇒ not detected.
    fs = 100.0
    n = 100
    t = [i / fs for i in range(n)]
    v = [math.sin(2.0 * math.pi * 5.0 * ti) for ti in t]
    detected, peak_hz, ratio = SA.detect_limit_cycle(
        t, v, f_min_hz=200.0, f_max_hz=300.0, peak_ratio=5.0
    )
    assert not detected
    assert peak_hz is None
    assert ratio == 0.0


def test_detect_limit_cycle_bad_params_raise():
    t = [i * 0.01 for i in range(50)]
    v = [0.0] * 50
    with pytest.raises(ValueError, match="peak_ratio must be > 1"):
        SA.detect_limit_cycle(t, v, f_min_hz=1.0, f_max_hz=10.0, peak_ratio=1.0)
    with pytest.raises(ValueError, match="f_max_hz"):
        SA.detect_limit_cycle(t, v, f_min_hz=5.0, f_max_hz=1.0, peak_ratio=5.0)
    with pytest.raises(ValueError, match="f_min_hz must be >= 0"):
        SA.detect_limit_cycle(t, v, f_min_hz=-1.0, f_max_hz=10.0, peak_ratio=5.0)
