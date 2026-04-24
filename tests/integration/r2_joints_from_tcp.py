"""IK-injected adapter: commanded ``TcpTrajectory`` -> joint command stream.

Pre-baked mirror of :mod:`tests.integration.r2_tcp_from_joints`. Where
that module folds a *measured* joint-state stream through an injected
FK callable into a TCP trajectory (stage-3 measured side, and stage-2
kinematic-consistency check), **this** module folds a *commanded*
``TcpTrajectory`` through an injected IK callable into a joint-space
command trajectory — the stream a JTC (joint-trajectory-controller)
goal-builder needs for the ``JTC + ik_shim`` combo the R2 stage-3
bullet in ``docs/ROADMAP.md`` enumerates verbatim:

    Command a known TCP trajectory (line, then arc, then sine-in-z)
    via ``cartesian_motion_controller`` and — separately — via JTC
    driven by an IK solver shim.

The inverse-kinematics implementation itself is still gated on the
same operator-level FK/IK source decision noted in ``docs/STATUS.md``.
Keeping the IK an **injected callable** keeps that decision orthogonal
to the pre-bake chain: an analytic UR-6-DoF solver, a KDL closed-form,
Pinocchio's Newton iteration, or a MJCF-derived numerical shim can all
satisfy the contract without any change to this module.

Contract of the injected IK
---------------------------

``ik(pos_xyz_m, quat_xyzw, q_seed) -> Sequence[float]`` where

* ``pos_xyz_m`` is a 3-tuple of finite floats — TCP position in metres
  in the robot's base frame, forwarded verbatim from the input
  :class:`~tests.integration.r2_stage3_commands.TcpTrajectory`.
* ``quat_xyzw`` is a 4-tuple ``(x, y, z, w)`` in Hamilton / ROS
  convention, forwarded verbatim (the trajectory dataclass already
  enforces the ``[0.5, 1.5]`` norm band, so the adapter does not
  re-normalise).
* ``q_seed`` is a ``Tuple[float, ...]`` with exactly
  ``len(joint_names)`` elements — the continuation seed. For the
  first sample this is the caller-supplied ``q_seed`` argument; for
  every subsequent sample the adapter threads the previous solution
  as seed, so branch-continuity is the caller's responsibility only
  at t = 0.
* Return value is a ``Sequence[float]`` of exactly
  ``len(joint_names)`` finite joint positions (rad). Any exception
  raised by ``ik`` is wrapped in a :class:`ValueError` carrying the
  sample index and timestamp so long-trajectory failures are
  debuggable. ``None`` / wrong-shape / non-finite returns are
  rejected with the same style of contextual ``ValueError``.

Seed continuity
---------------

The adapter intentionally does *not* validate inter-sample jumps in
the IK output — an analytic UR solver legitimately flips branches
mid-trajectory if the commanded TCP crosses a singularity, and it is
not this module's job to second-guess that. If the orchestrator
wants branch-jump rejection, it should layer a post-processing check
on the returned :class:`JointCommandTrajectory`.

Design mirrors :mod:`tests.integration.r2_tcp_from_joints` and the
sibling stage-commanded modules:

* Frozen :class:`JointCommandTrajectory` dataclass with immutable
  tuples and a :class:`~types.MappingProxyType` ``positions`` mapping,
  matching the stage-2 :class:`AllJointsCommandTrace` shape so a JTC
  goal-builder can consume either without branching.
* ``times`` is forwarded **verbatim** from the input
  :class:`TcpTrajectory`; the dataclass guarantees ``times[0] == 0.0``
  and strict monotonicity, so the adapter inherits those invariants
  for free and does not re-validate.
* Pure stdlib; no numpy, no ROS, no MuJoCo.
* :class:`ValueError` on genuinely inconsistent inputs; IK exceptions
  re-raised as ``ValueError`` with sample index + timestamp context.
"""

from __future__ import annotations

import importlib.util
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Callable, Mapping, Sequence, Tuple

# tests/integration/ is not a package on sys.path; load the sibling
# TcpTrajectory dataclass by file path, mirroring r2_tcp_from_joints.
_HERE = Path(__file__).resolve().parent


def _load_sibling(name: str):
    key = f"_r2jft_{name}"
    mod = sys.modules.get(key)
    if mod is not None:
        return mod
    spec = importlib.util.spec_from_file_location(key, _HERE / f"{name}.py")
    if spec is None or spec.loader is None:
        raise ImportError(f"r2_joints_from_tcp: cannot load sibling {name!r}")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[key] = mod
    spec.loader.exec_module(mod)
    return mod


TcpTrajectory = _load_sibling("r2_stage3_commands").TcpTrajectory

__all__ = (
    "IkCallable",
    "JointCommandTrajectory",
    "TcpTrajectory",
    "joint_trajectory_from_tcp",
)


Vec3 = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]  # (x, y, z, w) — Hamilton / ROS

IkCallable = Callable[[Vec3, Quat, Tuple[float, ...]], Sequence[float]]


# ---------------------------------------------------------------------------
# Result dataclass
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class JointCommandTrajectory:
    """Parallel-array commanded joint trajectory.

    Shape-compatible with
    :class:`tests.integration.r2_stage2_commands.AllJointsCommandTrace`
    on the ``(joint_names, times, positions)`` axis so the same JTC
    goal-builder can consume either.

    Attributes
    ----------
    joint_names:
        Ordered, unique joint names. All entries of ``positions`` are
        keyed by these names.
    times:
        Strictly monotonic time vector (s), forwarded verbatim from the
        input :class:`TcpTrajectory`. ``times[0] == 0.0``.
    positions:
        Read-only mapping ``joint_name -> q_cmd(t) trace`` (rad). Every
        trace has ``len(times)`` samples.
    """

    joint_names: Tuple[str, ...]
    times: Tuple[float, ...]
    positions: Mapping[str, Tuple[float, ...]]

    def __len__(self) -> int:
        return len(self.times)

    @property
    def duration_s(self) -> float:
        if len(self.times) < 2:
            return 0.0
        return self.times[-1] - self.times[0]


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------


def _validate_joint_names(joint_names: Sequence[str]) -> Tuple[str, ...]:
    names = tuple(joint_names)
    if not names:
        raise ValueError("r2_joints_from_tcp: joint_names must be non-empty")
    if len(set(names)) != len(names):
        raise ValueError(f"r2_joints_from_tcp: joint_names contains duplicates: {names}")
    return names


def _validate_seed(q_seed: Sequence[float], expected_len: int) -> Tuple[float, ...]:
    if len(q_seed) != expected_len:
        raise ValueError(
            f"r2_joints_from_tcp: q_seed length {len(q_seed)} does not match "
            f"len(joint_names)={expected_len}"
        )
    out = []
    for i, v in enumerate(q_seed):
        try:
            vf = float(v)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"r2_joints_from_tcp: q_seed[{i}] is not numeric ({v!r})") from exc
        if not math.isfinite(vf):
            raise ValueError(f"r2_joints_from_tcp: q_seed[{i}] is not finite ({vf})")
        out.append(vf)
    return tuple(out)


def _coerce_ik_output(
    result,
    *,
    expected_len: int,
    sample_index: int,
    sample_time_s: float,
) -> Tuple[float, ...]:
    context = f"sample {sample_index} (t={sample_time_s:.6g}s)"
    if result is None:
        raise ValueError(f"r2_joints_from_tcp: ik({context}) returned None")
    try:
        seq = list(result)
    except TypeError as exc:
        raise ValueError(
            f"r2_joints_from_tcp: ik({context}) did not return a sequence " f"(got {result!r})"
        ) from exc
    if len(seq) != expected_len:
        raise ValueError(
            f"r2_joints_from_tcp: ik({context}) returned {len(seq)} joint "
            f"values, expected {expected_len}"
        )
    out = []
    for j, v in enumerate(seq):
        try:
            vf = float(v)
        except (TypeError, ValueError) as exc:
            raise ValueError(
                f"r2_joints_from_tcp: ik({context})[{j}] is not numeric " f"({v!r})"
            ) from exc
        if not math.isfinite(vf):
            raise ValueError(f"r2_joints_from_tcp: ik({context})[{j}] is not finite " f"({vf})")
        out.append(vf)
    return tuple(out)


# ---------------------------------------------------------------------------
# Public adapter
# ---------------------------------------------------------------------------


def joint_trajectory_from_tcp(
    trajectory: TcpTrajectory,
    ik: IkCallable,
    *,
    joint_names: Sequence[str],
    q_seed: Sequence[float],
) -> JointCommandTrajectory:
    """Fold a commanded ``TcpTrajectory`` through ``ik`` into joint commands.

    Parameters
    ----------
    trajectory:
        Commanded TCP trajectory (must be a
        :class:`~tests.integration.r2_stage3_commands.TcpTrajectory`
        instance so the ``times[0] == 0.0`` and quaternion-norm
        invariants are guaranteed).
    ik:
        Injected IK callable. See module docstring for the full
        contract. Exceptions raised by ``ik`` are re-raised as
        :class:`ValueError` carrying the sample index and timestamp.
    joint_names:
        Ordered, unique joint names. Determines the dimension expected
        of every ``ik`` return.
    q_seed:
        Seed joint positions for the **first** sample's IK call, same
        order as ``joint_names``. Subsequent samples are seeded with
        the previous solution, so the caller only has to pick a
        branch-consistent initial guess.

    Returns
    -------
    JointCommandTrajectory
        Parallel-array trace consumable by a JTC goal-builder or the
        stage-2 :func:`evaluate_all_joints_joint_space` evaluator.

    Raises
    ------
    TypeError
        If ``trajectory`` is not a :class:`TcpTrajectory` instance.
    ValueError
        On empty / duplicate ``joint_names``, ``q_seed`` length
        mismatch or non-finite values, ``ik`` raising, ``ik``
        returning ``None`` / a non-sequence / the wrong number of
        joints / non-finite values.
    """
    # Duck-type rather than isinstance: TcpTrajectory may be loaded
    # by file path from more than one sys.modules key (tests/integration/
    # is not a package), and two distinct class identities would break
    # a strict isinstance check without actually being incompatible.
    try:
        times = trajectory.times
        positions_list = trajectory.positions
        orientations_list = trajectory.orientations
    except AttributeError as exc:
        raise TypeError(
            f"r2_joints_from_tcp: trajectory must expose times/positions/"
            f"orientations (TcpTrajectory-shaped), got "
            f"{type(trajectory).__name__}: {exc}"
        ) from exc

    names = _validate_joint_names(joint_names)
    seed = _validate_seed(q_seed, len(names))

    # Accumulate per-joint traces; allocate list-of-lists once and
    # append per sample so the hot loop stays alloc-light.
    per_joint: list[list[float]] = [[] for _ in names]

    current_seed = seed
    for i, (t_s, pos, quat) in enumerate(zip(times, positions_list, orientations_list)):
        try:
            raw = ik(pos, quat, current_seed)
        except Exception as exc:  # noqa: BLE001 — wrap with context
            raise ValueError(
                f"r2_joints_from_tcp: ik raised at sample {i} " f"(t={t_s:.6g}s): {exc!r}"
            ) from exc
        q = _coerce_ik_output(
            raw,
            expected_len=len(names),
            sample_index=i,
            sample_time_s=t_s,
        )
        for j, val in enumerate(q):
            per_joint[j].append(val)
        current_seed = q

    positions_map = {name: tuple(per_joint[j]) for j, name in enumerate(names)}

    return JointCommandTrajectory(
        joint_names=names,
        times=tuple(times),
        positions=MappingProxyType(positions_map),
    )
