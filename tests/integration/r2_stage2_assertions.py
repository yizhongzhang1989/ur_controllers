"""R2 stage-2 assertion harness: all-joints-together (joint-space path).

Pre-baked consumer API for the R2 integration test matrix bullet M6.13
(still gated on the M6.0 operator decision). Companion to
``tests/integration/r2_stage1_assertions.py``; same design posture
(pure stdlib, functional evaluators returning a typed result instead
of raising) so that — once the live sim lands — stage-2 tests reduce
to sim-collection + one call per controller.

Scope
-----

R2 stage 2 in ``docs/ROADMAP.md`` commands all six joints simultaneously
from home → cluttered pose → home and asserts:

* **Kinematic consistency** — TCP FK from measured ``q`` matches the
  expected trajectory within tolerance (joint-space) or ``5 mm + 2°``
  (cartesian mode). In *joint-space* mode the check reduces to a
  per-joint tracking-error bound on ``|q(t) - q_cmd(t)|``; the FK /
  cartesian-mode variant will be added when M6.14 pre-bakes the FK
  helper (deferred — no pure-Python UR DH table lands without operator
  review).
* **No stall** — every joint completes the motion; no joint holds
  torque saturation for more than ``saturation_hold_ms`` contiguously.

This module covers the joint-space branch of *kinematic consistency*
plus the *no-stall* branch. Namely, per joint:

1. **Motion completion** — ``|q(t_end) - q_cmd(t_end)|`` within
   ``completion_tol_rad``. When ``settle_window_s > 0`` the
   completion error is the *mean* of ``|q(t) - q_cmd(t)|`` over the
   trailing ``settle_window_s`` seconds instead of the single final
   sample — the settled reading the stage-2 orchestrator uses so a
   tighter ``completion_tol_rad`` than ``peak_tracking_err_rad`` is
   meaningful (see STATUS note for ``M6.13``).
2. **Peak tracking error** — ``max_t |q(t) - q_cmd(t)|`` within
   ``peak_tracking_err_rad``.
3. **Torque-saturation hold** — if a torque trace and an effort
   limit are supplied for the joint, the longest contiguous window
   where ``|tau| >= effort_limit`` must be ≤ ``saturation_hold_ms``.

Non-goals
---------

* FK / TCP-level checks (deferred to the M6.14 pre-bake once the
  kinematics source is agreed).
* Reading tolerances from the expectation YAML: the schema does not
  yet have a stage-2 block (ADR-0013 flagged this as a follow-up).
  Callers pass tolerances as explicit kwargs; values will migrate
  into ``tests/integration/expectations/<arm>.yaml`` once reviewed.
* Anything needing ROS / MuJoCo / numpy — same dependency posture as
  the sibling modules so these helpers run in the unit-test gate.

All evaluators raise :class:`ValueError` on inconsistent inputs
(length mismatch, non-monotonic time, missing per-joint trace)
rather than silently degrading — matches ``r2_stage1_assertions``.
"""

from __future__ import annotations

import importlib.util
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Dict, Mapping, Optional, Sequence, Tuple

if TYPE_CHECKING:
    from . import expectations_loader  # noqa: F401 — annotation-only import

# ---------------------------------------------------------------------------
# Sibling loading (tests/integration/ is not a package; follow the
# r2_stage1_assertions convention)
# ---------------------------------------------------------------------------

_HERE = Path(__file__).resolve().parent


def _load_sibling(name: str):
    key = f"_r2s2_{name}"
    mod = sys.modules.get(key)
    if mod is not None:
        return mod
    spec = importlib.util.spec_from_file_location(key, _HERE / f"{name}.py")
    if spec is None or spec.loader is None:
        raise ImportError(f"r2_stage2_assertions: cannot load sibling {name!r}")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[key] = mod
    spec.loader.exec_module(mod)
    return mod


def _stage1():
    return _load_sibling("r2_stage1_assertions")


__all__ = (
    "JointStage2Metrics",
    "Stage2Result",
    "evaluate_all_joints_joint_space",
    "evaluate_all_joints_from_expectation",
)


# ---------------------------------------------------------------------------
# Result dataclasses
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class JointStage2Metrics:
    """Per-joint outcome of an R2 stage-2 evaluation.

    Attributes
    ----------
    joint:
        Joint name.
    metrics:
        Scalar metrics. Always includes ``final_err_rad`` and
        ``peak_tracking_err_rad``; includes ``longest_saturation_hold_ms``
        only when a torque trace and an effort limit were supplied for
        this joint.
    failures:
        One-line strings, one per violated tolerance. Empty tuple ⇒
        all checks passed for this joint.
    """

    joint: str
    metrics: Mapping[str, float] = field(default_factory=dict)
    failures: Tuple[str, ...] = ()

    @property
    def ok(self) -> bool:
        return not self.failures


@dataclass(frozen=True)
class Stage2Result:
    """Aggregate outcome of an R2 stage-2 evaluation over all joints.

    Attributes
    ----------
    controller:
        Echo of the controller name.
    per_joint:
        Per-joint breakdown in input order.
    """

    controller: str
    per_joint: Tuple[JointStage2Metrics, ...] = ()

    @property
    def ok(self) -> bool:
        return all(j.ok for j in self.per_joint)

    @property
    def failures(self) -> Tuple[str, ...]:
        """Flattened list of ``"<joint>: <failure>"`` strings."""
        out: list = []
        for j in self.per_joint:
            out.extend(f"{j.joint}: {f}" for f in j.failures)
        return tuple(out)

    def format(self) -> str:
        """Multi-line diagnostic string for pytest ``assert`` messages."""
        head = f"[R2 stage-2] controller={self.controller}"
        lines = [head]
        for j in self.per_joint:
            lines.append(f"  joint {j.joint}: {'OK' if j.ok else 'FAIL'}")
            for k, v in sorted(j.metrics.items()):
                lines.append(f"    metric {k} = {v:.6g}")
            for f in j.failures:
                lines.append(f"    FAIL {f}")
        if not self.per_joint:
            lines.append("  (no joints evaluated)")
        return "\n".join(lines)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _validate_times(times: Sequence[float]) -> None:
    if len(times) < 2:
        raise ValueError(f"r2_stage2_assertions: times must have length >= 2, got {len(times)}")
    for i in range(1, len(times)):
        if times[i] <= times[i - 1]:
            raise ValueError(
                f"r2_stage2_assertions: times must be strictly monotonic; "
                f"times[{i - 1}]={times[i - 1]} !< times[{i}]={times[i]}"
            )


def _validate_trace(name: str, joint: str, trace: Sequence[float], n: int) -> None:
    if len(trace) != n:
        raise ValueError(
            f"r2_stage2_assertions: {name}[{joint!r}] length {len(trace)} " f"!= times length {n}"
        )


# ---------------------------------------------------------------------------
# Joint-space evaluator
# ---------------------------------------------------------------------------


def evaluate_all_joints_joint_space(
    controller: str,
    *,
    times: Sequence[float],
    measured_positions: Mapping[str, Sequence[float]],
    commanded_positions: Mapping[str, Sequence[float]],
    completion_tol_rad: float,
    peak_tracking_err_rad: float,
    torques: Optional[Mapping[str, Sequence[float]]] = None,
    effort_limits_nm: Optional[Mapping[str, float]] = None,
    saturation_hold_ms: float = 100.0,
    settle_window_s: float = 0.0,
) -> Stage2Result:
    """Evaluate R2 stage-2 joint-space tolerances for all joints at once.

    Parameters
    ----------
    controller:
        Controller name (echoed in the result).
    times:
        Shared, strictly monotonic time vector for every per-joint
        trace. Length must be ≥ 2.
    measured_positions:
        Joint name → measured position trace ``q(t)`` (rad). Must be
        keyed identically to ``commanded_positions``.
    commanded_positions:
        Joint name → commanded position trace ``q_cmd(t)`` (rad).
    completion_tol_rad:
        Absolute bound on the completion error — by default the
        single-sample ``|q(t_end) - q_cmd(t_end)|``; when
        ``settle_window_s > 0`` this becomes the mean absolute error
        over the trailing settle window, matching the orchestrator
        convention noted in ``docs/STATUS.md`` for M6.13.
    peak_tracking_err_rad:
        Absolute bound on ``max_t |q(t) - q_cmd(t)|`` — the joint-space
        kinematic-consistency check for R2 stage-2.
    torques:
        Optional joint name → torque trace (Nm). When supplied the
        matching ``effort_limits_nm`` entry is used to bound the
        longest contiguous saturation window.
    effort_limits_nm:
        Joint name → per-joint effort bound (Nm). Only consulted when
        the matching ``torques`` entry is present.
    saturation_hold_ms:
        Tolerance (ms) for the longest contiguous run where
        ``|tau| >= effort_limit``. Default 100 ms matches R2 text in
        ``docs/ROADMAP.md``.
    settle_window_s:
        Length (s) of the trailing window used to sample the motion-
        completion error. Default ``0.0`` ⇒ single-sample final error
        (backwards compatible). When positive, the evaluator averages
        ``|q(t) - q_cmd(t)|`` over samples whose timestamp is within
        ``settle_window_s`` of ``times[-1]`` and uses that mean as the
        reported ``final_err_rad`` metric; also exposes
        ``settle_window_samples`` on every joint for diagnostics. The
        window spans at least the final sample (so a positive value
        smaller than the sample period still yields a defined mean).
        Must be ≥ 0; must not exceed ``times[-1] - times[0]``.

    Raises
    ------
    ValueError:
        On inconsistent inputs (non-monotonic ``times``, length
        mismatch between any trace and ``times``, a joint key
        present in one of ``measured_positions`` / ``commanded_positions``
        but missing from the other, or a ``settle_window_s`` that is
        negative or larger than the trace span).
    """
    _validate_times(times)
    n = len(times)

    if settle_window_s < 0.0:
        raise ValueError(
            f"r2_stage2_assertions: settle_window_s must be >= 0, got {settle_window_s}"
        )
    span = times[-1] - times[0]
    if settle_window_s > span:
        raise ValueError(
            f"r2_stage2_assertions: settle_window_s={settle_window_s} exceeds "
            f"trace span={span} (times[0]={times[0]}, times[-1]={times[-1]})"
        )

    # Index of the first sample inside the trailing settle window. When
    # settle_window_s == 0.0 we leave settle_start = n (sentinel: use the
    # last-sample path). When > 0, at minimum the final sample qualifies.
    if settle_window_s > 0.0:
        threshold = times[-1] - settle_window_s
        settle_start = n - 1
        while settle_start > 0 and times[settle_start - 1] >= threshold:
            settle_start -= 1
    else:
        settle_start = n  # sentinel; branch below selects final-sample mode

    measured_keys = set(measured_positions)
    commanded_keys = set(commanded_positions)
    if measured_keys != commanded_keys:
        only_m = sorted(measured_keys - commanded_keys)
        only_c = sorted(commanded_keys - measured_keys)
        raise ValueError(
            "r2_stage2_assertions: measured_positions and commanded_positions "
            f"must share keys; only-in-measured={only_m} only-in-commanded={only_c}"
        )

    torques_map: Mapping[str, Sequence[float]] = torques or {}
    limits_map: Mapping[str, float] = effort_limits_nm or {}

    longest_contig = _stage1()._longest_contiguous_ms

    per_joint: list = []
    # Preserve the caller's iteration order over measured_positions so
    # the diagnostic table lines up row-for-row with the commanded plan.
    for joint in measured_positions:
        q = measured_positions[joint]
        q_cmd = commanded_positions[joint]
        _validate_trace("measured_positions", joint, q, n)
        _validate_trace("commanded_positions", joint, q_cmd, n)

        metrics: Dict[str, float] = {}
        failures: list = []

        # 1. Completion: trailing-window mean when settle_window_s > 0,
        #    otherwise the single final-sample error.
        if settle_start < n:
            window_errs = [abs(q[i] - q_cmd[i]) for i in range(settle_start, n)]
            final_err = sum(window_errs) / len(window_errs)
            metrics["settle_window_samples"] = float(len(window_errs))
        else:
            final_err = abs(q[-1] - q_cmd[-1])
        metrics["final_err_rad"] = final_err
        if final_err > completion_tol_rad:
            failures.append(f"final_err_rad={final_err:.6g} > tol={completion_tol_rad:.6g}")

        # 2. Peak tracking error over the whole trace.
        peak_err = max(abs(a - b) for a, b in zip(q, q_cmd))
        metrics["peak_tracking_err_rad"] = peak_err
        if peak_err > peak_tracking_err_rad:
            failures.append(
                f"peak_tracking_err_rad={peak_err:.6g} > tol={peak_tracking_err_rad:.6g}"
            )

        # 3. Optional saturation-hold check.
        if joint in torques_map and joint in limits_map:
            tau = torques_map[joint]
            _validate_trace("torques", joint, tau, n)
            limit = float(limits_map[joint])
            longest_ms = longest_contig(times, [abs(t) >= limit for t in tau])
            metrics["longest_saturation_hold_ms"] = longest_ms
            if longest_ms > saturation_hold_ms:
                failures.append(
                    f"longest_saturation_hold_ms={longest_ms:.6g} > "
                    f"tol={saturation_hold_ms:.6g}"
                )

        per_joint.append(
            JointStage2Metrics(
                joint=joint,
                metrics=metrics,
                failures=tuple(failures),
            )
        )

    return Stage2Result(controller=controller, per_joint=tuple(per_joint))


# ---------------------------------------------------------------------------
# Expectation-driven convenience wrapper
# ---------------------------------------------------------------------------


def evaluate_all_joints_from_expectation(
    arm_expectation: "expectations_loader.ArmExpectation",
    controller: str,
    *,
    times: Sequence[float],
    measured_positions: Mapping[str, Sequence[float]],
    commanded_positions: Mapping[str, Sequence[float]],
    torques: Optional[Mapping[str, Sequence[float]]] = None,
    settle_window_s: float = 0.0,
) -> Stage2Result:
    """Run :func:`evaluate_all_joints_joint_space` sourcing tolerances
    from a typed :class:`expectations_loader.ArmExpectation`.

    Convenience wrapper so stage-2 orchestrators do not open-code the
    ``stage2`` tolerance unpacking + per-joint effort-limit assembly
    on every call. Mirrors the stage-1 convention where evaluators
    accept the loader's dataclass directly and ``.tol()`` it.

    Sourced from ``arm_expectation``:

    * ``stage2.completion_tol_rad`` → ``completion_tol_rad`` kwarg.
    * ``stage2.peak_tracking_err_rad`` → ``peak_tracking_err_rad`` kwarg.
    * ``stage2.saturation_hold_ms`` → ``saturation_hold_ms`` kwarg.
    * ``{j.name: j.effort_limit_nm for j in arm_expectation.joints}`` →
      ``effort_limits_nm`` kwarg. Always populated, but only consulted
      for joints whose key also appears in ``torques``.

    Parameters
    ----------
    arm_expectation:
        Typed arm-level expectation as returned by
        :func:`expectations_loader.load_arm`. Duck-typed; any object
        exposing ``stage2`` (with the three tolerance fields) and an
        iterable ``joints`` of items with ``.name`` and
        ``.effort_limit_nm`` works.
    controller:
        Controller name, echoed in the result.
    times, measured_positions, commanded_positions, torques,
    settle_window_s:
        Forwarded unchanged to :func:`evaluate_all_joints_joint_space`.
        ``settle_window_s`` defaults to ``0.0`` (single-sample
        completion), matching the underlying evaluator.

    Raises
    ------
    ValueError:
        If ``torques`` is supplied with a key that is not a joint of
        ``arm_expectation`` — the wrapper would otherwise silently drop
        the saturation-hold check for that joint. All ``ValueError``
        cases from :func:`evaluate_all_joints_joint_space` still
        propagate unchanged.
    """
    stage2 = arm_expectation.stage2
    effort_limits_nm: Dict[str, float] = {
        j.name: float(j.effort_limit_nm) for j in arm_expectation.joints
    }

    if torques is not None:
        unknown = sorted(set(torques) - set(effort_limits_nm))
        if unknown:
            raise ValueError(
                "r2_stage2_assertions: torques contains joints not in "
                f"arm_expectation: {unknown}; "
                f"known joints: {sorted(effort_limits_nm)}"
            )

    return evaluate_all_joints_joint_space(
        controller,
        times=times,
        measured_positions=measured_positions,
        commanded_positions=commanded_positions,
        completion_tol_rad=float(stage2.completion_tol_rad),
        peak_tracking_err_rad=float(stage2.peak_tracking_err_rad),
        torques=torques,
        effort_limits_nm=effort_limits_nm,
        saturation_hold_ms=float(stage2.saturation_hold_ms),
        settle_window_s=settle_window_s,
    )
