"""R2 stage-3 assertion harness: end-effector (TCP) trajectory.

Pre-baked consumer API for the R2 integration test matrix bullet M6.14
(still gated on the M6.0 operator decision **and** on the downstream
FK-source decision for the orchestrator — see ``docs/STATUS.md``).

Scope
-----

R2 stage 3 in ``docs/ROADMAP.md`` commands a known TCP trajectory via
``cartesian_motion_controller`` and — separately — via JTC driven by an
IK solver shim, and asserts:

* **TCP RMSE** below 5 mm, **peak** below 10 mm.
* **Yaw/pitch/roll peak error** below 3°.
* With payload attached (R3), steady-state TCP drift below 2 mm over
  30 s for gravity-comp-aware controllers.

This module provides pure-stdlib evaluators that operate on
**pre-computed TCP trajectories** — positions in metres (``Tuple[x, y,
z]``) and orientations as unit quaternions (``Tuple[x, y, z, w]``, MuJoCo
/ ROS convention). The FK-from-``q`` step lives in the orchestrator that
wraps these evaluators; pre-baking the assertion math here unblocks the
rest of the M6.14 pre-bake chain regardless of which FK source the
operator eventually picks (ADR-0010, ADR-0012).

Design choices mirror the stage-1 / stage-2 siblings:

* Functional, typed result (:class:`TcpStage3Result`); evaluators never
  raise on tolerance failure — they return the result and let the
  caller decide (assert vs. warn vs. log to
  ``evaluation/runs/<ts>/``).
* Pure stdlib; no numpy, no ROS, no MuJoCo.
* ``ValueError`` on genuinely inconsistent inputs (length mismatch,
  non-monotonic time, degenerate quaternion norm, ``steady_window_s``
  out of range).

Metric semantics
----------------

* ``position_rmse_mm`` — ``sqrt(mean(||measured - commanded||^2)) * 1000``.
* ``position_peak_err_mm`` — ``max(||measured - commanded||) * 1000``.
* ``orientation_peak_err_deg`` — maximum over ``t`` of the per-axis
  roll/pitch/yaw magnitude of the relative rotation ``q_measured *
  q_commanded^{-1}`` decomposed as intrinsic XYZ Euler angles (i.e.
  ``max(|roll(t)|, |pitch(t)|, |yaw(t)|)`` in degrees). Matches the
  ROADMAP R2 "Yaw/pitch/roll peak error below 3°" wording — this is
  deliberately stricter than (or equal to) the single geodesic
  rotation angle, which is the largest value the YPR decomposition
  can produce.
* ``steady_drift_mm`` — when ``steady_window_s >= 30.0`` and
  ``tcp_steady_drift_mm_per_30s`` is provided, computed as the
  Euclidean distance between the mean position over the first half of
  the trailing ``steady_window_s`` window and the mean over the second
  half. Two-half-means is a simple robust drift proxy: for a
  constant-linear drift of ``v`` mm/s it reports ``v * window/2`` mm,
  so steady-state wobble around a centre does **not** show up as
  drift (unlike ``max - median`` alternatives). The check fires only
  when the window is at least the ROADMAP-mandated 30 s — no linear
  extrapolation from shorter windows to avoid false-passing on
  ringing signals.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Dict, Mapping, Optional, Sequence, Tuple

if TYPE_CHECKING:
    from . import expectations_loader  # noqa: F401 — annotation-only

__all__ = (
    "TcpStage3Result",
    "evaluate_tcp_trajectory",
    "evaluate_tcp_trajectory_from_expectation",
)


# Quaternion norm must be within this band of 1.0 to be accepted. Traces
# logged from ROS typically drift a bit from unit due to float conversion
# but should be well within 1 %.
_QUAT_NORM_MIN = 0.5
_QUAT_NORM_MAX = 1.5

# Minimum steady-window length for the drift check. Matches the ROADMAP
# R2 "over 30 s" wording; shorter windows are skipped with a diagnostic
# metric rather than rejected.
_MIN_STEADY_WINDOW_S = 30.0


# ---------------------------------------------------------------------------
# Result dataclass
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class TcpStage3Result:
    """Outcome of an R2 stage-3 TCP-trajectory evaluation.

    Attributes
    ----------
    controller:
        Controller name (echoed from the call).
    metrics:
        Scalar metrics. Always includes ``position_rmse_mm``,
        ``position_peak_err_mm``, ``orientation_peak_err_deg``. Includes
        ``steady_drift_mm`` only when the drift check ran (i.e. caller
        supplied both ``tcp_steady_drift_mm_per_30s`` and a
        ``steady_window_s >= 30.0``); includes ``steady_drift_skipped``
        with a reason string when the caller asked for the check but
        the window was too short.
    failures:
        One-line strings, one per violated tolerance. Empty tuple ⇒
        all checks passed.
    """

    controller: str
    metrics: Mapping[str, float] = field(default_factory=dict)
    failures: Tuple[str, ...] = ()
    notes: Tuple[str, ...] = ()

    @property
    def ok(self) -> bool:
        return not self.failures

    def format(self) -> str:
        head = f"[R2 stage-3] controller={self.controller}: " f"{'OK' if self.ok else 'FAIL'}"
        lines = [head]
        for k, v in sorted(self.metrics.items()):
            lines.append(f"  metric {k} = {v:.6g}")
        for n in self.notes:
            lines.append(f"  note {n}")
        for f in self.failures:
            lines.append(f"  FAIL {f}")
        return "\n".join(lines)


# ---------------------------------------------------------------------------
# Quaternion helpers (private)
# ---------------------------------------------------------------------------


def _validate_times(times: Sequence[float]) -> None:
    if len(times) < 2:
        raise ValueError(f"r2_stage3_assertions: times must have length >= 2, got {len(times)}")
    for i in range(1, len(times)):
        if times[i] <= times[i - 1]:
            raise ValueError(
                f"r2_stage3_assertions: times must be strictly monotonic; "
                f"times[{i - 1}]={times[i - 1]} !< times[{i}]={times[i]}"
            )


def _validate_trace_length(name: str, trace: Sequence, n: int) -> None:
    if len(trace) != n:
        raise ValueError(f"r2_stage3_assertions: {name} length {len(trace)} != times length {n}")


def _normalize_quat(
    q: Tuple[float, float, float, float], where: str
) -> Tuple[float, float, float, float]:
    if len(q) != 4:
        raise ValueError(f"r2_stage3_assertions: {where}: expected 4-tuple, got length {len(q)}")
    x, y, z, w = (float(q[0]), float(q[1]), float(q[2]), float(q[3]))
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not (_QUAT_NORM_MIN <= norm <= _QUAT_NORM_MAX) or norm == 0.0:
        raise ValueError(
            f"r2_stage3_assertions: {where}: degenerate quaternion norm {norm:.6g} "
            f"(expected within [{_QUAT_NORM_MIN}, {_QUAT_NORM_MAX}])"
        )
    return (x / norm, y / norm, z / norm, w / norm)


def _quat_relative_rpy_deg(
    qm: Tuple[float, float, float, float],
    qc: Tuple[float, float, float, float],
) -> Tuple[float, float, float]:
    """Return (roll, pitch, yaw) in degrees of ``qm * qc^{-1}``.

    Uses the intrinsic XYZ Euler decomposition that matches tf2 /
    KDL's ``getRPY`` convention: rotate about fixed X (roll), then Y
    (pitch), then Z (yaw) — equivalently intrinsic ZYX. Input
    quaternions are assumed already normalized; the function handles
    antipodal pairs via the usual ``sign(dot)`` convention so that
    ``q`` and ``-q`` yield zero relative rotation.
    """
    # q_rel = qm * qc^{-1}; with unit quaternions, qc^{-1} = (-x, -y, -z, w).
    xm, ym, zm, wm = qm
    xc, yc, zc, wc = qc
    # Antipodal handling: if dot is negative, negate qc so q_rel is the
    # short-arc rotation, yielding zero for q == -q.
    dot = xm * xc + ym * yc + zm * zc + wm * wc
    if dot < 0.0:
        xc, yc, zc, wc = -xc, -yc, -zc, -wc
    # qm * qc^{-1}  (Hamilton convention, (x, y, z, w)).
    ix, iy, iz, iw = -xc, -yc, -zc, wc
    x = wm * ix + xm * iw + ym * iz - zm * iy
    y = wm * iy - xm * iz + ym * iw + zm * ix
    z = wm * iz + xm * iy - ym * ix + zm * iw
    w = wm * iw - xm * ix - ym * iy - zm * iz

    # Intrinsic XYZ (fixed-axis XYZ / tf2 RPY) decomposition:
    #   roll  = atan2(2(wx + yz),  1 - 2(x^2 + y^2))
    #   pitch = asin(clip(2(wy - zx), -1, 1))
    #   yaw   = atan2(2(wz + xy),  1 - 2(y^2 + z^2))
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    deg = 180.0 / math.pi
    return (roll * deg, pitch * deg, yaw * deg)


# ---------------------------------------------------------------------------
# Core evaluator
# ---------------------------------------------------------------------------


def evaluate_tcp_trajectory(
    controller: str,
    *,
    times: Sequence[float],
    measured_positions_m: Sequence[Tuple[float, float, float]],
    commanded_positions_m: Sequence[Tuple[float, float, float]],
    measured_orientations_quat: Sequence[Tuple[float, float, float, float]],
    commanded_orientations_quat: Sequence[Tuple[float, float, float, float]],
    tcp_rmse_mm: float,
    tcp_peak_err_mm: float,
    tcp_orientation_peak_deg: float,
    tcp_steady_drift_mm_per_30s: Optional[float] = None,
    steady_window_s: float = 0.0,
) -> TcpStage3Result:
    """Evaluate TCP-trajectory tolerances for a single controller run.

    Parameters
    ----------
    controller:
        Controller name, echoed in the result.
    times:
        Shared, strictly monotonic time vector (s) for every trace.
        Length must be ≥ 2.
    measured_positions_m, commanded_positions_m:
        Per-sample ``(x, y, z)`` TCP positions in metres.
    measured_orientations_quat, commanded_orientations_quat:
        Per-sample ``(x, y, z, w)`` TCP orientations as quaternions.
        Antipodal pairs represent the same rotation; the evaluator
        handles that correctly.
    tcp_rmse_mm, tcp_peak_err_mm, tcp_orientation_peak_deg:
        Stage-3 pass/fail tolerances (mirrors
        ``ArmExpectation.tcp_tolerances``).
    tcp_steady_drift_mm_per_30s:
        Optional drift tolerance. When ``None`` (or
        ``steady_window_s == 0``) the drift check is skipped.
    steady_window_s:
        Length (s) of the trailing window used for the steady-drift
        check. Must be ``>= 0``; must not exceed
        ``times[-1] - times[0]``. The check only runs when the window
        is at least :data:`_MIN_STEADY_WINDOW_S` (30 s, per ROADMAP
        R2). Shorter windows fall through with a
        ``steady_drift_skipped`` note rather than raising.

    Raises
    ------
    ValueError:
        On inconsistent inputs (non-monotonic ``times``, trace-length
        mismatch, degenerate quaternion norm, ``steady_window_s``
        negative or exceeding the trace span).
    """
    _validate_times(times)
    n = len(times)
    for name, trace in (
        ("measured_positions_m", measured_positions_m),
        ("commanded_positions_m", commanded_positions_m),
        ("measured_orientations_quat", measured_orientations_quat),
        ("commanded_orientations_quat", commanded_orientations_quat),
    ):
        _validate_trace_length(name, trace, n)

    if steady_window_s < 0.0:
        raise ValueError(
            f"r2_stage3_assertions: steady_window_s must be >= 0, got {steady_window_s}"
        )
    span = times[-1] - times[0]
    if steady_window_s > span:
        raise ValueError(
            f"r2_stage3_assertions: steady_window_s={steady_window_s} exceeds " f"trace span={span}"
        )

    # Position errors.
    sq_errs_m2: list = []
    peak_err_m2 = 0.0
    for i in range(n):
        mx, my, mz = measured_positions_m[i]
        cx, cy, cz = commanded_positions_m[i]
        dx = float(mx) - float(cx)
        dy = float(my) - float(cy)
        dz = float(mz) - float(cz)
        sq = dx * dx + dy * dy + dz * dz
        sq_errs_m2.append(sq)
        if sq > peak_err_m2:
            peak_err_m2 = sq
    rmse_m = math.sqrt(sum(sq_errs_m2) / n)
    peak_err_m = math.sqrt(peak_err_m2)

    metrics: Dict[str, float] = {
        "position_rmse_mm": rmse_m * 1000.0,
        "position_peak_err_mm": peak_err_m * 1000.0,
    }
    failures: list = []
    notes: list = []

    if metrics["position_rmse_mm"] > tcp_rmse_mm:
        failures.append(
            f"position_rmse_mm={metrics['position_rmse_mm']:.6g} > tol={tcp_rmse_mm:.6g}"
        )
    if metrics["position_peak_err_mm"] > tcp_peak_err_mm:
        failures.append(
            f"position_peak_err_mm={metrics['position_peak_err_mm']:.6g} > "
            f"tol={tcp_peak_err_mm:.6g}"
        )

    # Orientation: per-axis RPY peak over the whole trace.
    peak_ori_deg = 0.0
    for i in range(n):
        qm = _normalize_quat(
            tuple(measured_orientations_quat[i]),  # type: ignore[arg-type]
            f"measured_orientations_quat[{i}]",
        )
        qc = _normalize_quat(
            tuple(commanded_orientations_quat[i]),  # type: ignore[arg-type]
            f"commanded_orientations_quat[{i}]",
        )
        r, p, y = _quat_relative_rpy_deg(qm, qc)
        peak_ori_deg = max(peak_ori_deg, abs(r), abs(p), abs(y))
    metrics["orientation_peak_err_deg"] = peak_ori_deg
    if peak_ori_deg > tcp_orientation_peak_deg:
        failures.append(
            f"orientation_peak_err_deg={peak_ori_deg:.6g} > tol={tcp_orientation_peak_deg:.6g}"
        )

    # Steady drift (optional).
    drift_requested = tcp_steady_drift_mm_per_30s is not None and steady_window_s > 0.0
    if drift_requested:
        if steady_window_s < _MIN_STEADY_WINDOW_S:
            reason = (
                f"steady_window_s={steady_window_s:.6g}s < required "
                f"{_MIN_STEADY_WINDOW_S:.6g}s (ROADMAP R2 mandates a 30 s hold)"
            )
            metrics["steady_drift_skipped"] = 1.0
            notes.append(f"drift check skipped: {reason}")
        else:
            threshold = times[-1] - steady_window_s
            # Index of the first sample inside the trailing window.
            i0 = n - 1
            while i0 > 0 and times[i0 - 1] >= threshold:
                i0 -= 1
            n_window = n - i0
            if n_window < 4:
                reason = (
                    f"steady window has {n_window} samples, need >= 4 for "
                    f"two-half-means drift metric"
                )
                metrics["steady_drift_skipped"] = 1.0
                notes.append(f"drift check skipped: {reason}")
            else:
                mid = i0 + n_window // 2
                first_half = measured_positions_m[i0:mid]
                second_half = measured_positions_m[mid:n]
                n1 = len(first_half)
                n2 = len(second_half)
                mean1 = (
                    sum(float(p[0]) for p in first_half) / n1,
                    sum(float(p[1]) for p in first_half) / n1,
                    sum(float(p[2]) for p in first_half) / n1,
                )
                mean2 = (
                    sum(float(p[0]) for p in second_half) / n2,
                    sum(float(p[1]) for p in second_half) / n2,
                    sum(float(p[2]) for p in second_half) / n2,
                )
                drift_m = math.sqrt(
                    (mean2[0] - mean1[0]) ** 2
                    + (mean2[1] - mean1[1]) ** 2
                    + (mean2[2] - mean1[2]) ** 2
                )
                drift_mm = drift_m * 1000.0
                metrics["steady_drift_mm"] = drift_mm
                metrics["steady_window_samples"] = float(n_window)
                assert tcp_steady_drift_mm_per_30s is not None  # narrow for mypy
                if drift_mm > tcp_steady_drift_mm_per_30s:
                    failures.append(
                        f"steady_drift_mm={drift_mm:.6g} > tol={tcp_steady_drift_mm_per_30s:.6g}"
                    )

    return TcpStage3Result(
        controller=controller,
        metrics=metrics,
        failures=tuple(failures),
        notes=tuple(notes),
    )


# ---------------------------------------------------------------------------
# Expectation-driven wrapper
# ---------------------------------------------------------------------------


def evaluate_tcp_trajectory_from_expectation(
    arm_expectation: "expectations_loader.ArmExpectation",
    controller: str,
    *,
    times: Sequence[float],
    measured_positions_m: Sequence[Tuple[float, float, float]],
    commanded_positions_m: Sequence[Tuple[float, float, float]],
    measured_orientations_quat: Sequence[Tuple[float, float, float, float]],
    commanded_orientations_quat: Sequence[Tuple[float, float, float, float]],
    steady_window_s: float = 0.0,
) -> TcpStage3Result:
    """Run :func:`evaluate_tcp_trajectory` sourcing tolerances from a
    typed :class:`expectations_loader.ArmExpectation`.

    Sources the four TCP tolerances from
    ``arm_expectation.tcp_tolerances``:

    * ``tcp_rmse_mm`` → ``tcp_rmse_mm`` kwarg.
    * ``tcp_peak_err_mm`` → ``tcp_peak_err_mm`` kwarg.
    * ``tcp_orientation_peak_deg`` → ``tcp_orientation_peak_deg`` kwarg.
    * ``tcp_steady_drift_mm_per_30s`` → ``tcp_steady_drift_mm_per_30s``
      kwarg (always passed through, regardless of whether
      ``steady_window_s`` is large enough to exercise it).

    Duck-typed on the input: any object exposing ``.tcp_tol(key)``
    (returning a float) for the four keys above works, matching the
    stage-1 / stage-2 precedent.
    """
    return evaluate_tcp_trajectory(
        controller,
        times=times,
        measured_positions_m=measured_positions_m,
        commanded_positions_m=commanded_positions_m,
        measured_orientations_quat=measured_orientations_quat,
        commanded_orientations_quat=commanded_orientations_quat,
        tcp_rmse_mm=float(arm_expectation.tcp_tol("tcp_rmse_mm")),
        tcp_peak_err_mm=float(arm_expectation.tcp_tol("tcp_peak_err_mm")),
        tcp_orientation_peak_deg=float(arm_expectation.tcp_tol("tcp_orientation_peak_deg")),
        tcp_steady_drift_mm_per_30s=float(arm_expectation.tcp_tol("tcp_steady_drift_mm_per_30s")),
        steady_window_s=steady_window_s,
    )
