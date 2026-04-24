"""R2 stage-2 assertion harness: all-joints-together (cartesian-mode path).

Pre-baked consumer API for the R2 integration test matrix bullet M6.13
(still gated on the M6.0 operator decision). Companion to
``tests/integration/r2_stage2_assertions.py`` — that module carries the
*joint-space* path; this module carries the *cartesian-mode* path.

Scope
-----

R2 stage 2 in ``docs/ROADMAP.md`` asserts

    Kinematic consistency: TCP FK from measured ``q`` matches the
    expected trajectory within tolerance (joint-space) or **5 mm + 2°**
    (cartesian mode).

In *cartesian mode* (currently only ``cartesian_motion_controller``) the
stage-2 orchestrator

1. generates a commanded joint trajectory with
   :func:`r2_stage2_commands.home_to_pose_to_home_command`;
2. folds that trajectory through an injected FK to obtain a
   *commanded* :class:`~r2_stage3_commands.TcpTrajectory`
   (via :func:`r2_tcp_from_joints.tcp_trajectory_from_joints`);
3. folds the *measured* ``/joint_states`` stream through the same FK
   to obtain a *measured* :class:`~r2_stage3_commands.TcpTrajectory`;
4. calls :func:`evaluate_all_joints_cartesian` (this module) on the
   pair.

The deferral note in ``r2_stage2_assertions.py`` ("FK / TCP-level
checks — deferred to the M6.14 pre-bake once the kinematics source is
agreed") is resolved: the FK source is *injected by the caller* per
``r2_tcp_from_joints.py``'s contract, so the pre-bake can land now
without picking a backend.

This module deliberately does **not** re-implement the trajectory
math that the stage-3 harness already covers. It uses the same
intrinsic-XYZ Euler decomposition helper as
:mod:`r2_stage3_assertions` for the per-axis yaw/pitch/roll bound
(same semantics ⇒ a stage-2 orchestrator comparing its own results
against stage-3 gets directly comparable numbers).

Differences from the stage-3 harness
-----------------------------------

* Stage 3 asserts RMSE, peak position error, peak orientation error,
  *and* a 30 s steady drift. Stage 2 (ROADMAP §R2 text) only asserts
  **peak** position error (5 mm) and **peak** per-axis orientation
  error (2°) — there is no RMSE or drift requirement for the
  home → cluttered pose → home maneuver.
* Stage 3 ships with a tighter-than-stage-2 RMSE (5 mm) and a
  looser peak (10 mm); stage 2 is 5 mm peak because the ROADMAP
  wording is "within 5 mm + 2°" without qualification.

Non-goals
---------

* Reading FK from anywhere inside this module. The caller supplies a
  commanded + measured TCP trajectory already (produced by
  ``r2_tcp_from_joints``); this evaluator is pure math.
* Time resampling. The two :class:`TcpTrajectory` inputs must share
  a sample count. If they were produced from the same ``times`` vector
  (the common case for an orchestrator that uses the commanded trace's
  timebase to align the measured trace), they align by construction;
  otherwise the caller must resample first.
* Any dependency on ROS / MuJoCo / numpy — matches the sibling
  pre-bake posture so this evaluator runs in the unit-test gate.
"""

from __future__ import annotations

import importlib.util
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Dict, Mapping, Tuple

if TYPE_CHECKING:
    from . import expectations_loader  # noqa: F401 — annotation-only


# ---------------------------------------------------------------------------
# Sibling loading (tests/integration/ is not a package; follow the
# r2_stage2_assertions convention so `TcpTrajectory` identity matches the
# FK adapter's own loader key — the evaluator duck-types the inputs
# anyway but having a canonical handle is useful for type checkers).
# ---------------------------------------------------------------------------

_HERE = Path(__file__).resolve().parent


def _load_sibling(name: str):
    key = f"_r2s2c_{name}"
    mod = sys.modules.get(key)
    if mod is not None:
        return mod
    spec = importlib.util.spec_from_file_location(key, _HERE / f"{name}.py")
    if spec is None or spec.loader is None:
        raise ImportError(f"r2_stage2_cartesian: cannot load sibling {name!r}")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[key] = mod
    spec.loader.exec_module(mod)
    return mod


TcpTrajectory = _load_sibling("r2_stage3_commands").TcpTrajectory

__all__ = (
    "Stage2CartesianResult",
    "evaluate_all_joints_cartesian",
    "evaluate_all_joints_cartesian_from_expectation",
)


# Same norm band as the sibling stage-3 modules so an FK backend
# returning floats a few ULPs off unit is accepted.
_QUAT_NORM_MIN = 0.5
_QUAT_NORM_MAX = 1.5


# ---------------------------------------------------------------------------
# Result dataclass
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Stage2CartesianResult:
    """Outcome of an R2 stage-2 cartesian-mode evaluation.

    Attributes
    ----------
    controller:
        Controller name (echoed from the call).
    metrics:
        Scalar metrics. Always includes ``position_peak_err_mm`` and
        ``orientation_peak_err_deg``.
    failures:
        One-line strings, one per violated tolerance. Empty tuple ⇒
        all checks passed.
    """

    controller: str
    metrics: Mapping[str, float] = field(default_factory=dict)
    failures: Tuple[str, ...] = ()

    @property
    def ok(self) -> bool:
        return not self.failures

    def format(self) -> str:
        head = (
            f"[R2 stage-2 cartesian] controller={self.controller}: "
            f"{'OK' if self.ok else 'FAIL'}"
        )
        lines = [head]
        for k, v in sorted(self.metrics.items()):
            lines.append(f"  metric {k} = {v:.6g}")
        for f in self.failures:
            lines.append(f"  FAIL {f}")
        return "\n".join(lines)


# ---------------------------------------------------------------------------
# Duck-type validation
# ---------------------------------------------------------------------------


def _require_trajectory_shape(name: str, obj: object) -> None:
    for attr in ("times", "positions", "orientations"):
        if not hasattr(obj, attr):
            raise TypeError(
                f"r2_stage2_cartesian: {name} is not a TcpTrajectory-shaped "
                f"object (missing {attr!r})"
            )


def _validate_times(times, *, where: str) -> Tuple[float, ...]:
    n = len(times)
    if n < 2:
        raise ValueError(f"r2_stage2_cartesian: {where}.times must have length >= 2, got {n}")
    out = []
    prev = None
    for i, t in enumerate(times):
        tf = float(t)
        if not math.isfinite(tf):
            raise ValueError(f"r2_stage2_cartesian: {where}.times[{i}] is not finite ({tf})")
        if prev is not None and tf <= prev:
            raise ValueError(
                f"r2_stage2_cartesian: {where}.times must be strictly "
                f"increasing; times[{i}]={tf} <= times[{i - 1}]={prev}"
            )
        out.append(tf)
        prev = tf
    return tuple(out)


def _validate_positions(positions, n: int, *, where: str) -> Tuple[Tuple[float, float, float], ...]:
    if len(positions) != n:
        raise ValueError(
            f"r2_stage2_cartesian: {where}.positions length {len(positions)} "
            f"!= times length {n}"
        )
    out = []
    for i, p in enumerate(positions):
        if len(p) != 3:
            raise ValueError(
                f"r2_stage2_cartesian: {where}.positions[{i}] must have 3 "
                f"elements, got {len(p)}"
            )
        try:
            x, y, z = float(p[0]), float(p[1]), float(p[2])
        except (TypeError, ValueError) as exc:
            raise ValueError(
                f"r2_stage2_cartesian: {where}.positions[{i}] is not numeric " f"({p!r})"
            ) from exc
        for axis, val in zip("xyz", (x, y, z)):
            if not math.isfinite(val):
                raise ValueError(
                    f"r2_stage2_cartesian: {where}.positions[{i}].{axis} " f"is not finite ({val})"
                )
        out.append((x, y, z))
    return tuple(out)


def _validate_orientations(
    orientations, n: int, *, where: str
) -> Tuple[Tuple[float, float, float, float], ...]:
    if len(orientations) != n:
        raise ValueError(
            f"r2_stage2_cartesian: {where}.orientations length "
            f"{len(orientations)} != times length {n}"
        )
    out = []
    for i, q in enumerate(orientations):
        if len(q) != 4:
            raise ValueError(
                f"r2_stage2_cartesian: {where}.orientations[{i}] must have 4 "
                f"elements (x, y, z, w), got {len(q)}"
            )
        try:
            x, y, z, w = float(q[0]), float(q[1]), float(q[2]), float(q[3])
        except (TypeError, ValueError) as exc:
            raise ValueError(
                f"r2_stage2_cartesian: {where}.orientations[{i}] is not " f"numeric ({q!r})"
            ) from exc
        for axis, val in zip("xyzw", (x, y, z, w)):
            if not math.isfinite(val):
                raise ValueError(
                    f"r2_stage2_cartesian: {where}.orientations[{i}].{axis} "
                    f"is not finite ({val})"
                )
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm == 0.0 or not (_QUAT_NORM_MIN <= norm <= _QUAT_NORM_MAX):
            raise ValueError(
                f"r2_stage2_cartesian: {where}.orientations[{i}] "
                f"degenerate quaternion norm {norm:.6g} "
                f"(expected within [{_QUAT_NORM_MIN}, {_QUAT_NORM_MAX}])"
            )
        # Normalise so downstream math is on unit quaternions.
        out.append((x / norm, y / norm, z / norm, w / norm))
    return tuple(out)


# ---------------------------------------------------------------------------
# Quaternion → relative-RPY helper (mirrors r2_stage3_assertions)
# ---------------------------------------------------------------------------


def _quat_relative_rpy_deg(
    qm: Tuple[float, float, float, float],
    qc: Tuple[float, float, float, float],
) -> Tuple[float, float, float]:
    """Return (roll, pitch, yaw) in degrees of ``qm * qc^{-1}``.

    Mirrors :func:`r2_stage3_assertions._quat_relative_rpy_deg` so
    stage-2 and stage-3 report orientation errors in the same units
    and convention. Input quaternions are assumed unit-norm; antipodal
    pairs are folded into the short-arc representation so ``q`` and
    ``-q`` produce zero.
    """
    xm, ym, zm, wm = qm
    xc, yc, zc, wc = qc
    dot = xm * xc + ym * yc + zm * zc + wm * wc
    if dot < 0.0:
        xc, yc, zc, wc = -xc, -yc, -zc, -wc
    ix, iy, iz, iw = -xc, -yc, -zc, wc
    x = wm * ix + xm * iw + ym * iz - zm * iy
    y = wm * iy - xm * iz + ym * iw + zm * ix
    z = wm * iz + xm * iy - ym * ix + zm * iw
    w = wm * iw - xm * ix - ym * iy - zm * iz

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


def evaluate_all_joints_cartesian(
    controller: str,
    *,
    commanded_tcp,
    measured_tcp,
    position_peak_err_mm: float,
    orientation_peak_err_deg: float,
) -> Stage2CartesianResult:
    """Evaluate R2 stage-2 cartesian-mode tolerances for a single run.

    Parameters
    ----------
    controller:
        Controller name, echoed in the result.
    commanded_tcp, measured_tcp:
        :class:`~r2_stage3_commands.TcpTrajectory` (or duck-typed
        equivalent with ``times``, ``positions``, ``orientations``
        attributes). Both must have the same sample count; the
        caller is responsible for time alignment (stage-2
        orchestrators typically share the commanded trace's
        timebase and interpolate ``/joint_states`` onto it before
        running FK).
    position_peak_err_mm:
        Pass/fail bound on ``max_t ||p_m(t) - p_c(t)||``, expressed
        in **millimetres**. Must be ``> 0``.
    orientation_peak_err_deg:
        Pass/fail bound on
        ``max_t max(|roll|, |pitch|, |yaw|)`` of the relative
        rotation ``q_m * q_c^{-1}``, expressed in **degrees**.
        Must be ``> 0``.

    Returns
    -------
    Stage2CartesianResult
        Never raises on a tolerance failure — the caller decides
        whether to assert or record. All numeric metrics are in SI
        + mm / deg so they can be logged alongside
        :class:`~r2_stage3_assertions.TcpStage3Result`.

    Raises
    ------
    TypeError:
        If either trajectory is missing the ``times`` /
        ``positions`` / ``orientations`` attributes (duck-type).
    ValueError:
        On non-positive tolerances, length mismatch between the two
        trajectories, non-monotonic or non-finite times, non-finite
        / wrong-shape positions or orientations, or quaternion norms
        outside the ``[0.5, 1.5]`` band.
    """
    if not (position_peak_err_mm > 0.0) or not math.isfinite(position_peak_err_mm):
        raise ValueError(
            f"r2_stage2_cartesian: position_peak_err_mm must be > 0 and "
            f"finite, got {position_peak_err_mm}"
        )
    if not (orientation_peak_err_deg > 0.0) or not math.isfinite(orientation_peak_err_deg):
        raise ValueError(
            f"r2_stage2_cartesian: orientation_peak_err_deg must be > 0 and "
            f"finite, got {orientation_peak_err_deg}"
        )

    _require_trajectory_shape("commanded_tcp", commanded_tcp)
    _require_trajectory_shape("measured_tcp", measured_tcp)

    t_cmd = _validate_times(commanded_tcp.times, where="commanded_tcp")
    t_mea = _validate_times(measured_tcp.times, where="measured_tcp")
    if len(t_cmd) != len(t_mea):
        raise ValueError(
            f"r2_stage2_cartesian: commanded_tcp has {len(t_cmd)} samples but "
            f"measured_tcp has {len(t_mea)}; the evaluator does not resample "
            f"— align timebases before calling"
        )
    n = len(t_cmd)

    p_cmd = _validate_positions(commanded_tcp.positions, n, where="commanded_tcp")
    p_mea = _validate_positions(measured_tcp.positions, n, where="measured_tcp")
    q_cmd = _validate_orientations(commanded_tcp.orientations, n, where="commanded_tcp")
    q_mea = _validate_orientations(measured_tcp.orientations, n, where="measured_tcp")

    # Position peak error (mm).
    peak_pos_err_m = 0.0
    for (xc, yc, zc), (xm, ym, zm) in zip(p_cmd, p_mea):
        dx = xm - xc
        dy = ym - yc
        dz = zm - zc
        d = math.sqrt(dx * dx + dy * dy + dz * dz)
        if d > peak_pos_err_m:
            peak_pos_err_m = d
    peak_pos_err_mm = peak_pos_err_m * 1000.0

    # Orientation peak per-axis error (deg).
    peak_ori_err_deg = 0.0
    for qm, qc in zip(q_mea, q_cmd):
        roll, pitch, yaw = _quat_relative_rpy_deg(qm, qc)
        worst = max(abs(roll), abs(pitch), abs(yaw))
        if worst > peak_ori_err_deg:
            peak_ori_err_deg = worst

    metrics: Dict[str, float] = {
        "position_peak_err_mm": peak_pos_err_mm,
        "orientation_peak_err_deg": peak_ori_err_deg,
    }
    failures: list = []
    if peak_pos_err_mm > position_peak_err_mm:
        failures.append(
            f"position_peak_err_mm={peak_pos_err_mm:.6g} > " f"tol={position_peak_err_mm:.6g}"
        )
    if peak_ori_err_deg > orientation_peak_err_deg:
        failures.append(
            f"orientation_peak_err_deg={peak_ori_err_deg:.6g} > "
            f"tol={orientation_peak_err_deg:.6g}"
        )
    return Stage2CartesianResult(controller=controller, metrics=metrics, failures=tuple(failures))


# ---------------------------------------------------------------------------
# Expectation-driven convenience wrapper
# ---------------------------------------------------------------------------


def evaluate_all_joints_cartesian_from_expectation(
    arm_expectation: "expectations_loader.ArmExpectation",
    controller: str,
    *,
    commanded_tcp,
    measured_tcp,
) -> Stage2CartesianResult:
    """Run :func:`evaluate_all_joints_cartesian` sourcing tolerances
    from a typed :class:`expectations_loader.ArmExpectation`.

    Mirrors :func:`r2_stage2_assertions.evaluate_all_joints_from_expectation`
    so stage-2 orchestrators do not open-code the ``stage2_tcp``
    tolerance unpacking on every call.

    Sourced from ``arm_expectation``:

    * ``stage2_tcp.position_peak_err_mm`` → ``position_peak_err_mm``.
    * ``stage2_tcp.orientation_peak_err_deg`` → ``orientation_peak_err_deg``.

    Parameters
    ----------
    arm_expectation:
        Typed arm-level expectation as returned by
        :func:`expectations_loader.load_arm`. Duck-typed; any object
        exposing a ``stage2_tcp`` attribute with the two tolerance
        fields works.
    controller, commanded_tcp, measured_tcp:
        Forwarded to :func:`evaluate_all_joints_cartesian`.
    """
    tol = arm_expectation.stage2_tcp
    return evaluate_all_joints_cartesian(
        controller,
        commanded_tcp=commanded_tcp,
        measured_tcp=measured_tcp,
        position_peak_err_mm=float(tol.position_peak_err_mm),
        orientation_peak_err_deg=float(tol.orientation_peak_err_deg),
    )
