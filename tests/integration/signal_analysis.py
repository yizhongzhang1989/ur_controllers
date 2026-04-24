"""Pure-stdlib signal-analysis helpers for R2 integration tests.

Pre-baked consumer API for the R2 assertion vocabulary defined in
``docs/ROADMAP.md`` §M6 R2 and the theoretical side already landed in
``tests/integration/expectations_loader.py``. Authored ahead of the R2
tests (M6.12–M6.14, still gated on the M6.0 operator decision) so those
tests — and anyone prototyping them against recorded bags — can rely on
a stable, unit-tested surface.

Scope:

* Trailing-window scalar metrics on sampled joint signals (steady-state
  error, peak-to-peak drift, velocity RMS).
* Limit-cycle / chatter detection on a joint-velocity trace via a naïve
  one-sided DFT over a frequency band (``[f_min_hz, f_max_hz]``).
* Damping-ratio extraction from a step response via half-cycle
  logarithmic decrement on the first two extrema of the error signal.

Non-goals:

* Anything that needs ROS, MuJoCo, or numpy. These helpers run in the
  unit-test gate and must stay dependency-free (matching
  ``expectations_loader.py``).
* Signal filtering / denoising. R2 tests pull straight from
  ``/joint_states`` at the sim's control rate; callers that want a
  low-pass should do it upstream.
* Scenario-wide metrics (RMSE, settling time, overshoot, control
  effort): those live in ``evaluation/compute_metrics.py`` (ADR-0010)
  and are reused by the R2 tests via that module, not duplicated here.

All public functions raise ``ValueError`` on inconsistent inputs
(length mismatch, non-monotonic time, non-uniform sampling where
uniformity is required) so assertion failures in the test gate point
at the data rather than silently passing on ``nan``.
"""

from __future__ import annotations

import cmath
import math
import statistics
from typing import List, Optional, Sequence, Tuple

__all__ = (
    "damping_ratio_from_step",
    "detect_limit_cycle",
    "drift_peak_to_peak",
    "find_extrema",
    "steady_state_error",
    "velocity_rms",
)


# ---------------------------------------------------------------------------
# Shared validation
# ---------------------------------------------------------------------------


def _check_pair(times: Sequence[float], values: Sequence[float], *, min_len: int = 2) -> None:
    if len(times) != len(values):
        raise ValueError(
            f"signal_analysis: times/values length mismatch ({len(times)} vs {len(values)})"
        )
    if len(times) < min_len:
        raise ValueError(f"signal_analysis: need at least {min_len} samples, got {len(times)}")
    for i in range(1, len(times)):
        if times[i] < times[i - 1]:
            raise ValueError(f"signal_analysis: times not monotonic at index {i}")


def _trailing_slice(
    times: Sequence[float], values: Sequence[float], window_s: float
) -> Tuple[List[float], List[float]]:
    if window_s <= 0.0:
        raise ValueError(f"signal_analysis: window_s must be > 0 (got {window_s})")
    t_end = times[-1]
    t_start = t_end - window_s
    # Find first index where times[i] >= t_start.
    # Linear scan is fine — R2 windows are at most a few thousand samples.
    idx = 0
    for i, t in enumerate(times):
        if t >= t_start:
            idx = i
            break
    else:
        idx = len(times) - 1
    t_win = list(times[idx:])
    v_win = list(values[idx:])
    if len(t_win) < 2:
        raise ValueError(
            f"signal_analysis: trailing window {window_s}s covers fewer than 2 samples"
        )
    return t_win, v_win


# ---------------------------------------------------------------------------
# Scalar metrics
# ---------------------------------------------------------------------------


def steady_state_error(
    times: Sequence[float],
    values: Sequence[float],
    target: float,
    window_s: float,
) -> float:
    """Absolute error between the mean of the trailing window and ``target``.

    The window is ``[t_end - window_s, t_end]``. Returns
    ``|mean(values_in_window) - target|``.
    """
    _check_pair(times, values)
    _, v_win = _trailing_slice(times, values, window_s)
    return abs(statistics.fmean(v_win) - float(target))


def drift_peak_to_peak(
    times: Sequence[float],
    values: Sequence[float],
    window_s: float,
) -> float:
    """Peak-to-peak drift of ``values`` over the trailing ``window_s``.

    Used by R2 stage-1 to assert "no drift over a 10 s hold": the return
    value is ``max(v) - min(v)`` over the window. R2 compares it against
    the per-controller ``steady_state_err_rad`` tolerance.
    """
    _check_pair(times, values)
    _, v_win = _trailing_slice(times, values, window_s)
    return max(v_win) - min(v_win)


def velocity_rms(
    times: Sequence[float],
    values: Sequence[float],
    window_s: float,
) -> float:
    """Root-mean-square of ``values`` over the trailing ``window_s``.

    R2 stage-1 uses this on joint velocity to assert "no sustained
    chatter" — the return value is compared against a per-controller
    ``chatter_vel_rms_rad_s`` threshold.
    """
    _check_pair(times, values)
    _, v_win = _trailing_slice(times, values, window_s)
    acc = 0.0
    for v in v_win:
        acc += v * v
    return math.sqrt(acc / len(v_win))


# ---------------------------------------------------------------------------
# Extrema / damping ratio
# ---------------------------------------------------------------------------


def find_extrema(
    times: Sequence[float],
    values: Sequence[float],
    *,
    min_abs: float = 0.0,
) -> List[Tuple[float, float]]:
    """Return ``(t, value)`` of strict local extrema of ``values``.

    A sample ``values[i]`` is a local extremum iff it is strictly greater
    than both neighbours (local max) or strictly less than both (local
    min). ``min_abs`` filters extrema whose absolute value is below the
    given magnitude — intended for use on a mean-subtracted error signal
    so that low-amplitude noise is ignored.

    Equal neighbouring samples (``values[i] == values[i-1]``) do not
    qualify as extrema. This matches the log-decrement use-case: we need
    distinct peaks, not plateaus.
    """
    _check_pair(times, values, min_len=3)
    out: List[Tuple[float, float]] = []
    n = len(values)
    for i in range(1, n - 1):
        vi, vp, vn = values[i], values[i - 1], values[i + 1]
        if (vi > vp and vi > vn) or (vi < vp and vi < vn):
            if abs(vi) >= min_abs:
                out.append((times[i], vi))
    return out


def damping_ratio_from_step(
    times: Sequence[float],
    values: Sequence[float],
    target: float,
    *,
    min_amplitude: Optional[float] = None,
) -> Optional[float]:
    """Estimate the closed-loop damping ratio from a step response.

    Uses the **half-cycle logarithmic decrement** on the first two
    strict extrema of the error signal ``e(t) = values(t) - target``:

    .. math::

        \\delta_{\\text{half}} = \\ln\\!\\left(\\frac{|e_0|}{|e_1|}\\right),
        \\qquad
        \\zeta = \\frac{\\delta_{\\text{half}}}
                     {\\sqrt{\\pi^2 + \\delta_{\\text{half}}^2}}.

    For a sampled underdamped second-order response about ``target``,
    consecutive extrema have opposite sign and lie half a damped period
    apart, so the half-cycle formula is the natural fit.

    Parameters
    ----------
    times, values:
        Sampled step response. Length ≥ 3.
    target:
        Step target. The error signal is ``values - target``.
    min_amplitude:
        Optional noise gate: extrema with ``|e| < min_amplitude`` are
        ignored. Defaults to ``max(|e|) * 0.01`` (1% of the peak excursion),
        which is enough to suppress digitisation noise in sim without
        rejecting genuinely small second peaks.

    Returns
    -------
    float in ``[0, 1)`` on success.
    ``None`` if the response is overdamped (fewer than 2 qualifying
    extrema after filtering): the caller should treat that as
    "no oscillation to fit" rather than an error.
    """
    _check_pair(times, values, min_len=3)
    err = [v - target for v in values]

    max_mag = max(abs(e) for e in err)
    if max_mag == 0.0:
        return None  # flat line ⇒ nothing to fit.

    gate = float(min_amplitude) if min_amplitude is not None else max_mag * 0.01
    if gate < 0.0:
        raise ValueError(f"damping_ratio_from_step: min_amplitude must be >= 0 (got {gate})")

    peaks = find_extrema(times, err, min_abs=gate)
    if len(peaks) < 2:
        return None

    a1 = abs(peaks[0][1])
    a2 = abs(peaks[1][1])
    if a1 <= 0.0 or a2 <= 0.0:
        return None
    if a2 >= a1:
        # Non-decaying: either the fit window is too short or the system
        # is unstable / energy-increasing. Either way, not a damping
        # ratio we can report. ``None`` lets the caller decide whether
        # that's a stage-1 failure.
        return None

    delta = math.log(a1 / a2)
    return delta / math.sqrt(math.pi * math.pi + delta * delta)


# ---------------------------------------------------------------------------
# Limit-cycle / oscillation detector
# ---------------------------------------------------------------------------


def _assert_uniform(times: Sequence[float]) -> float:
    """Return the sample period if ``times`` is near-uniformly sampled.

    Raises ``ValueError`` if the jitter exceeds 10% of the median period.
    """
    dts = [times[i] - times[i - 1] for i in range(1, len(times))]
    if any(dt <= 0.0 for dt in dts):
        raise ValueError("signal_analysis: non-increasing timestamps")
    med = statistics.median(dts)
    if med <= 0.0:
        raise ValueError("signal_analysis: zero median sample period")
    jitter = max(abs(dt - med) for dt in dts)
    if jitter > 0.1 * med:
        raise ValueError(
            f"signal_analysis: sampling jitter {jitter:.6g}s exceeds 10% of "
            f"median period {med:.6g}s; resample before FFT"
        )
    return med


def detect_limit_cycle(
    times: Sequence[float],
    values: Sequence[float],
    *,
    f_min_hz: float,
    f_max_hz: float,
    peak_ratio: float,
) -> Tuple[bool, Optional[float], float]:
    """Detect a dominant oscillation in ``[f_min_hz, f_max_hz]``.

    Uses a naïve one-sided DFT (O(N²)) over the mean-subtracted signal.
    R2 windows are short (≤ 5 s × ≤ 500 Hz ⇒ ≤ 2500 samples), so the
    O(N²) cost is <10 ms in practice and avoids pulling numpy into the
    unit-test gate. Callers with longer windows should downsample first.

    Parameters
    ----------
    times, values:
        Uniformly-sampled signal. Jitter >10% of the median period
        raises ``ValueError``.
    f_min_hz, f_max_hz:
        Band of interest. R2 stage-1 uses ``f_min_hz=1.0`` and
        ``f_max_hz=nyquist``. ``f_max_hz`` is clipped internally to the
        signal's Nyquist frequency.
    peak_ratio:
        Detection threshold. Returns ``detected=True`` iff the peak
        magnitude in the band exceeds ``peak_ratio * median_magnitude``
        over the band (a median-based noise floor is robust to a single
        spike).

    Returns
    -------
    ``(detected, peak_hz, ratio)``:
        * ``detected``: whether a limit cycle was found.
        * ``peak_hz``: frequency (Hz) of the dominant bin in the band,
          or ``None`` if the band has no bins.
        * ``ratio``: ``peak_magnitude / median_magnitude`` over the
          band — useful for diagnostic messages.
    """
    _check_pair(times, values, min_len=4)
    if f_min_hz < 0.0:
        raise ValueError(f"f_min_hz must be >= 0 (got {f_min_hz})")
    if f_max_hz <= f_min_hz:
        raise ValueError(f"f_max_hz ({f_max_hz}) must be > f_min_hz ({f_min_hz})")
    if peak_ratio <= 1.0:
        raise ValueError(f"peak_ratio must be > 1 (got {peak_ratio})")

    dt = _assert_uniform(times)
    n = len(values)
    duration = (n - 1) * dt
    fs = 1.0 / dt
    nyquist = 0.5 * fs
    f_hi = min(f_max_hz, nyquist)
    if f_hi <= f_min_hz:
        # Band is empty after Nyquist clipping.
        return False, None, 0.0

    # Mean subtraction removes the DC bin.
    mean = statistics.fmean(values)
    x = [v - mean for v in values]

    # One-sided DFT over bins ``k`` with frequency ``k / duration``.
    # Restrict ``k`` to the band.
    k_min = max(1, math.ceil(f_min_hz * duration))
    k_max = min(n // 2, math.floor(f_hi * duration))
    if k_max < k_min:
        return False, None, 0.0

    mags: List[float] = []
    for k in range(k_min, k_max + 1):
        acc = 0j
        # -2πj k n / N for n in 0..N-1
        base = -2.0j * math.pi * k / n
        for idx in range(n):
            acc += x[idx] * cmath.exp(base * idx)
        mags.append(abs(acc))

    if not mags:
        return False, None, 0.0
    peak_mag = max(mags)
    peak_idx = mags.index(peak_mag)
    peak_hz = (k_min + peak_idx) / duration
    # Robust noise floor: median over the band.
    floor = statistics.median(mags)
    if floor <= 0.0:
        # Band is silent apart from one bin ⇒ definitely a peak if there's
        # any energy at all.
        detected = peak_mag > 0.0
        ratio = math.inf if peak_mag > 0.0 else 0.0
    else:
        ratio = peak_mag / floor
        detected = ratio > peak_ratio
    return detected, peak_hz, ratio
