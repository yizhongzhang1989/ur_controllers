"""FK-injected adapter: measured joint-state stream -> ``TcpTrajectory``.

Pre-baked bridge between a *measured* joint-state time series (what the
sim or the real driver publishes on ``/joint_states``) and the TCP
trajectory format the R2 stage-3 assertion harness consumes
(:func:`tests.integration.r2_stage3_assertions.evaluate_tcp_trajectory`).

The forward-kinematics implementation itself is still gated on an
operator decision (source + licensing) — see ``docs/STATUS.md``. This
adapter keeps that decision *orthogonal* to the pre-bake chain by
taking the FK as an **injected callable**; a KDL-, Pinocchio-, or
mjcf-derived backend can all satisfy the contract without any change
to this module.

The adapter also serves R2 stage 2's "Kinematic consistency: TCP FK
from measured q matches the expected trajectory within tolerance"
check — same input shape, same output dataclass.

Contract of the injected FK
---------------------------

``fk(q) -> (pos_xyz_m, quat_xyzw)`` where

* ``q`` is a ``Sequence[float]`` of joint positions in the same order
  the caller supplied (six elements for a UR arm). Any ``arm`` /
  kinematic-model selection the backend needs must be **closed over**
  by the caller; this adapter is deliberately backend-agnostic and
  exposes no ``arm`` argument.
* ``pos_xyz_m`` is the TCP position in metres, in the robot's base
  frame, as a 3-tuple of finite floats. "TCP" here means whatever
  tool frame the caller's ``fk`` targets (e.g. ``tool0`` or a custom
  offset); the adapter does not impose a choice — it simply forwards
  whatever ``fk`` returns.
* ``quat_xyzw`` is a 4-tuple ``(x, y, z, w)`` in the Hamilton / ROS
  convention with norm in the ``[0.5, 1.5]`` band used by
  :mod:`tests.integration.r2_stage3_assertions` and
  :mod:`tests.integration.r2_stage3_commands`. The adapter
  normalises the returned quaternion to unit norm so downstream
  consumers never have to re-normalise.

Time base
---------

Measured joint-state traces rarely start at ``t == 0``. To preserve
the :class:`~tests.integration.r2_stage3_commands.TcpTrajectory`
invariant (``times[0] == 0.0``), the adapter **rebases** the input
``times`` by subtracting ``times[0]`` before constructing the
trajectory. The rebased vector is strictly monotonic iff the input
was strictly monotonic; this is validated.

Time alignment between commanded and measured TCP trajectories is
deliberately **out of scope** here — the stage-3 orchestrator is
expected to resample one onto the other's timestamps before handing
both to ``evaluate_tcp_trajectory``.

Design mirrors the sibling stage-3 modules: frozen dataclass result,
pure stdlib, ``ValueError`` on genuinely inconsistent inputs, with
FK-callable exceptions wrapped in a ``ValueError`` carrying the
sample index and timestamp so a long-trace failure is debuggable.
"""

from __future__ import annotations

import importlib.util
import math
import sys
from pathlib import Path
from typing import Callable, Sequence, Tuple

# tests/integration/ is not a package on sys.path; load the sibling
# TcpTrajectory dataclass by file path, mirroring r2_stage2_assertions.
_HERE = Path(__file__).resolve().parent


def _load_sibling(name: str):
    key = f"_r2tcp_{name}"
    mod = sys.modules.get(key)
    if mod is not None:
        return mod
    spec = importlib.util.spec_from_file_location(key, _HERE / f"{name}.py")
    if spec is None or spec.loader is None:
        raise ImportError(f"r2_tcp_from_joints: cannot load sibling {name!r}")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[key] = mod
    spec.loader.exec_module(mod)
    return mod


TcpTrajectory = _load_sibling("r2_stage3_commands").TcpTrajectory

__all__ = (
    "FkCallable",
    "TcpTrajectory",
    "tcp_trajectory_from_joints",
)


Vec3 = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]  # (x, y, z, w) — Hamilton / ROS

FkCallable = Callable[[Sequence[float]], Tuple[Vec3, Quat]]

# Same norm band as r2_stage3_commands / r2_stage3_assertions so an FK
# backend returning floats a few ULPs off unit is accepted, but a
# wrong-convention or zero quaternion is rejected loudly.
_QUAT_NORM_MIN = 0.5
_QUAT_NORM_MAX = 1.5


def _validate_times(times: Sequence[float]) -> Tuple[float, ...]:
    n = len(times)
    if n < 2:
        raise ValueError(f"r2_tcp_from_joints: times must have at least 2 samples, got {n}")
    out = []
    prev = None
    for i, t in enumerate(times):
        tf = float(t)
        if not math.isfinite(tf):
            raise ValueError(f"r2_tcp_from_joints: times[{i}] is not finite ({tf})")
        if prev is not None and tf <= prev:
            raise ValueError(
                f"r2_tcp_from_joints: times must be strictly increasing; "
                f"times[{i}]={tf} <= times[{i - 1}]={prev}"
            )
        out.append(tf)
        prev = tf
    return tuple(out)


def _validate_joint_samples(
    joint_samples: Sequence[Sequence[float]], expected_count: int
) -> Tuple[Tuple[float, ...], ...]:
    if len(joint_samples) != expected_count:
        raise ValueError(
            f"r2_tcp_from_joints: len(joint_samples)={len(joint_samples)} "
            f"does not match len(times)={expected_count}"
        )
    out = []
    first_dim: int | None = None
    for i, q in enumerate(joint_samples):
        try:
            coerced = tuple(float(v) for v in q)
        except (TypeError, ValueError) as exc:
            raise ValueError(
                f"r2_tcp_from_joints: joint_samples[{i}] is not a numeric " f"sequence ({q!r})"
            ) from exc
        if first_dim is None:
            first_dim = len(coerced)
            if first_dim == 0:
                raise ValueError(
                    "r2_tcp_from_joints: joint_samples[0] is empty; "
                    "FK needs at least one joint position"
                )
        elif len(coerced) != first_dim:
            raise ValueError(
                f"r2_tcp_from_joints: joint_samples[{i}] has dim "
                f"{len(coerced)}, expected {first_dim} (matches sample 0)"
            )
        for j, v in enumerate(coerced):
            if not math.isfinite(v):
                raise ValueError(
                    f"r2_tcp_from_joints: joint_samples[{i}][{j}] is not " f"finite ({v})"
                )
        out.append(coerced)
    return tuple(out)


def _coerce_fk_output(result, *, sample_index: int, sample_time_s: float) -> Tuple[Vec3, Quat]:
    context = f"sample {sample_index} (t={sample_time_s:.6g}s)"
    try:
        pos, quat = result
    except (TypeError, ValueError) as exc:
        raise ValueError(
            f"r2_tcp_from_joints: fk({context}) must return "
            f"(pos_xyz, quat_xyzw); got {result!r}"
        ) from exc

    try:
        px, py, pz = pos
    except (TypeError, ValueError) as exc:
        raise ValueError(
            f"r2_tcp_from_joints: fk({context}) returned pos of wrong "
            f"length; expected 3 floats, got {pos!r}"
        ) from exc
    try:
        pos_f = (float(px), float(py), float(pz))
    except (TypeError, ValueError) as exc:
        raise ValueError(
            f"r2_tcp_from_joints: fk({context}) returned non-numeric pos " f"({pos!r})"
        ) from exc
    for axis, val in zip("xyz", pos_f):
        if not math.isfinite(val):
            raise ValueError(
                f"r2_tcp_from_joints: fk({context}) returned non-finite " f"pos.{axis} ({val})"
            )

    try:
        qx, qy, qz, qw = quat
    except (TypeError, ValueError) as exc:
        raise ValueError(
            f"r2_tcp_from_joints: fk({context}) returned quat of wrong "
            f"length; expected 4 floats (x, y, z, w), got {quat!r}"
        ) from exc
    try:
        quat_f = (float(qx), float(qy), float(qz), float(qw))
    except (TypeError, ValueError) as exc:
        raise ValueError(
            f"r2_tcp_from_joints: fk({context}) returned non-numeric quat " f"({quat!r})"
        ) from exc
    for axis, val in zip("xyzw", quat_f):
        if not math.isfinite(val):
            raise ValueError(
                f"r2_tcp_from_joints: fk({context}) returned non-finite " f"quat.{axis} ({val})"
            )
    norm = math.sqrt(sum(v * v for v in quat_f))
    if norm == 0.0 or not (_QUAT_NORM_MIN <= norm <= _QUAT_NORM_MAX):
        raise ValueError(
            f"r2_tcp_from_joints: fk({context}) returned degenerate "
            f"quaternion norm {norm:.6g} "
            f"(expected within [{_QUAT_NORM_MIN}, {_QUAT_NORM_MAX}])"
        )
    quat_unit = tuple(v / norm for v in quat_f)
    return pos_f, quat_unit  # type: ignore[return-value]


def tcp_trajectory_from_joints(
    times: Sequence[float],
    joint_samples: Sequence[Sequence[float]],
    fk: FkCallable,
) -> TcpTrajectory:
    """Fold a measured joint-state stream through ``fk`` into a TCP trajectory.

    Parameters
    ----------
    times:
        Strictly increasing sample times (s). Rebased internally so
        the returned :class:`TcpTrajectory` satisfies its
        ``times[0] == 0.0`` invariant; ``times[-1]`` becomes
        ``times[-1] - times[0]``.
    joint_samples:
        One joint-position vector per sample, all of equal length.
        Must line up 1:1 with ``times``.
    fk:
        Injected FK callable. See module docstring for the full
        contract. Exceptions raised by ``fk`` are re-raised as
        ``ValueError`` carrying the sample index and timestamp.

    Returns
    -------
    TcpTrajectory
        With unit-normalised quaternions and finite positions.

    Raises
    ------
    ValueError
        On length mismatch, non-monotonic or non-finite times,
        inconsistent joint-sample dimension, non-finite joint values,
        FK callable errors, FK returning wrong-shape pos/quat,
        non-finite FK output, or FK returning a quaternion outside
        the ``[0.5, 1.5]`` norm band.
    """
    times_f = _validate_times(times)
    samples = _validate_joint_samples(joint_samples, len(times_f))

    t0 = times_f[0]
    rebased = tuple(t - t0 for t in times_f)
    # Guarantee exact 0.0 at the head; float accumulation can leave a
    # sub-ULP residual on the subtract.
    rebased = (0.0,) + rebased[1:]

    positions = []
    orientations = []
    for i, (t_abs, q) in enumerate(zip(times_f, samples)):
        try:
            raw = fk(q)
        except Exception as exc:  # noqa: BLE001 — wrap with context
            raise ValueError(
                f"r2_tcp_from_joints: fk raised at sample {i} " f"(t={t_abs:.6g}s): {exc!r}"
            ) from exc
        pos, quat = _coerce_fk_output(raw, sample_index=i, sample_time_s=t_abs)
        positions.append(pos)
        orientations.append(quat)

    return TcpTrajectory(
        times=rebased,
        positions=tuple(positions),
        orientations=tuple(orientations),
    )
