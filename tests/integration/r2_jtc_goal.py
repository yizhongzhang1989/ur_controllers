"""JTC goal-builder: parallel-array trace -> JointTrajectory-shaped struct.

Pre-baked glue for the R2 test matrix (M6.12 / M6.13 / M6.14, still
gated on the M6.0 operator decision before it can run live against
the sim). The three commanded-trace generators landed in prior
iterations —

* ``tests.integration.r2_stage1_commands.JointCommandTrace``
  (single-joint step / sine),
* ``tests.integration.r2_stage2_commands.AllJointsCommandTrace``
  (home → via → home all-joints cosine),
* ``tests.integration.r2_joints_from_tcp.JointCommandTrajectory``
  (IK-folded commanded TCP path),

— all share the exact same ``(joint_names, times, positions)`` shape.
Every one of them is documented as "consumable by a JTC goal-builder",
and this module *is* that builder: it converts any such trace into a
frozen :class:`JointTrajectoryGoal` carrying

* ordered, unique ``joint_names`` (tuple of str),
* a ``points`` tuple where each :class:`JointTrajectoryPoint` carries
  a ``positions`` tuple laid out in the same order as ``joint_names``
  and a scalar ``time_from_start_s`` in seconds.

The dataclass matches the layout of ROS 2's
``trajectory_msgs/JointTrajectory`` + ``JointTrajectoryPoint`` closely
enough that a thin ROS-side adapter (invoked only when the sim is
live) can materialise a ``FollowJointTrajectory.Goal`` from a
:class:`JointTrajectoryGoal` with no additional logic. That adapter is
explicitly **not** part of this pre-bake — it's the one piece that
needs ROS imports, and keeping it out of tree here keeps this module
pure stdlib, aligned with the rest of the R2 pre-bake chain.

Input contract (duck-typed)
---------------------------

A ``trace`` is accepted if it exposes three attributes:

* ``joint_names``: iterable of ``str`` (must be non-empty, unique).
* ``times``: iterable of ``float`` (must be non-empty, strictly
  monotonic increasing, all finite; ``times[0]`` is allowed to be
  anything — the builder applies ``time_offset_s`` *additively* on top
  of the trace's own timebase).
* ``positions``: mapping ``str -> Sequence[float]``; every key in
  ``joint_names`` must appear, every value must have ``len(times)``
  finite samples. Extra keys are an error (would silently drop data
  downstream).

We duck-type (not ``isinstance``) for the same reason as
:mod:`r2_joints_from_tcp`: ``tests/integration/`` is not a package on
``sys.path``, so two call sites loading the same trace dataclass via
different ``sys.modules`` keys would fail an ``isinstance`` check
against one legitimate class.

Options
-------

* ``time_offset_s`` (default ``0.0``): additive shift applied to every
  ``time_from_start_s``. Useful because some JTC implementations reject
  a first point at ``time_from_start == 0``. Must be finite and
  non-negative (a negative offset would push the first point's
  ``time_from_start`` negative, which ``trajectory_msgs/Duration``
  cannot represent).
* ``skip_initial_sample`` (default ``False``): drop ``times[0]`` from
  the output. This is the correct behaviour when the first sample is
  already the robot's current state (JTC interprets the zero-duration
  initial point as an instantaneous jump otherwise). Requires the
  input trace to have ``len(times) >= 2``.

Both options compose: the offset is applied to the remaining points
after the optional skip.

Validation errors all raise :class:`ValueError` / :class:`TypeError`
with a leading ``r2_jtc_goal:`` prefix matching the rest of the R2
pre-bake chain.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple

__all__ = (
    "JointTrajectoryPoint",
    "JointTrajectoryGoal",
    "jtc_goal_from_trace",
)


# ---------------------------------------------------------------------------
# Result dataclasses
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class JointTrajectoryPoint:
    """One waypoint of a ``JointTrajectoryGoal``.

    Attributes
    ----------
    positions:
        Commanded joint positions (rad), laid out in the same order as
        ``JointTrajectoryGoal.joint_names``. Length equals
        ``len(JointTrajectoryGoal.joint_names)``.
    time_from_start_s:
        Time from the start of the trajectory, in seconds. Always
        non-negative and finite.
    """

    positions: Tuple[float, ...]
    time_from_start_s: float


@dataclass(frozen=True)
class JointTrajectoryGoal:
    """Parallel-array JTC-goal-shaped payload.

    Attributes
    ----------
    joint_names:
        Ordered, unique joint names.
    points:
        Tuple of :class:`JointTrajectoryPoint` waypoints. The
        ``positions`` of each point align index-for-index with
        ``joint_names``. ``time_from_start_s`` is strictly monotonic
        increasing across points.
    """

    joint_names: Tuple[str, ...]
    points: Tuple[JointTrajectoryPoint, ...]

    def __len__(self) -> int:
        return len(self.points)

    @property
    def duration_s(self) -> float:
        if not self.points:
            return 0.0
        return self.points[-1].time_from_start_s - self.points[0].time_from_start_s


# ---------------------------------------------------------------------------
# Validation helpers
# ---------------------------------------------------------------------------


def _require_attr(trace, name: str):
    if not hasattr(trace, name):
        raise TypeError(
            f"r2_jtc_goal: trace is missing required attribute {name!r} "
            f"(got object of type {type(trace).__name__})"
        )
    return getattr(trace, name)


def _validate_joint_names(raw) -> Tuple[str, ...]:
    try:
        names = tuple(raw)
    except TypeError as exc:
        raise TypeError(f"r2_jtc_goal: joint_names is not iterable ({raw!r})") from exc
    if not names:
        raise ValueError("r2_jtc_goal: joint_names must be non-empty")
    for i, n in enumerate(names):
        if not isinstance(n, str):
            raise TypeError(
                f"r2_jtc_goal: joint_names[{i}] is not a str ({n!r}, " f"type={type(n).__name__})"
            )
    if len(set(names)) != len(names):
        raise ValueError(f"r2_jtc_goal: joint_names contains duplicates: {names}")
    return names


def _validate_times(raw) -> Tuple[float, ...]:
    try:
        times_list = list(raw)
    except TypeError as exc:
        raise TypeError(f"r2_jtc_goal: times is not iterable ({raw!r})") from exc
    if not times_list:
        raise ValueError("r2_jtc_goal: times must be non-empty")
    out = []
    for i, t in enumerate(times_list):
        try:
            tf = float(t)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"r2_jtc_goal: times[{i}] is not numeric ({t!r})") from exc
        if not math.isfinite(tf):
            raise ValueError(f"r2_jtc_goal: times[{i}] is not finite ({tf})")
        out.append(tf)
    for i in range(1, len(out)):
        if out[i] <= out[i - 1]:
            raise ValueError(
                f"r2_jtc_goal: times is not strictly monotonic increasing "
                f"(times[{i-1}]={out[i-1]}, times[{i}]={out[i]})"
            )
    return tuple(out)


def _validate_positions(
    raw,
    joint_names: Tuple[str, ...],
    n_samples: int,
) -> Tuple[Tuple[float, ...], ...]:
    # positions is expected to be a mapping name -> sequence. We don't
    # force Mapping ABC; we only need `in` / `[]` / a way to list keys.
    # Gather the raw per-joint traces first.
    try:
        keys = list(raw.keys())
    except AttributeError as exc:
        raise TypeError(
            f"r2_jtc_goal: positions must be a mapping (got " f"{type(raw).__name__})"
        ) from exc
    name_set = set(joint_names)
    key_set = set(keys)
    missing = name_set - key_set
    if missing:
        raise ValueError(
            f"r2_jtc_goal: positions is missing entries for joint(s) " f"{sorted(missing)}"
        )
    extra = key_set - name_set
    if extra:
        raise ValueError(
            f"r2_jtc_goal: positions has entries for joint(s) "
            f"{sorted(extra)} not in joint_names={list(joint_names)}"
        )
    # Validate and coerce per-joint traces; store column-major so we
    # can produce per-sample rows in joint_names order.
    per_joint: list = []
    for name in joint_names:
        seq = raw[name]
        try:
            values = list(seq)
        except TypeError as exc:
            raise TypeError(
                f"r2_jtc_goal: positions[{name!r}] is not iterable " f"({seq!r})"
            ) from exc
        if len(values) != n_samples:
            raise ValueError(
                f"r2_jtc_goal: positions[{name!r}] has {len(values)} "
                f"samples, expected {n_samples} (to match times)"
            )
        coerced = []
        for i, v in enumerate(values):
            try:
                vf = float(v)
            except (TypeError, ValueError) as exc:
                raise ValueError(
                    f"r2_jtc_goal: positions[{name!r}][{i}] is not " f"numeric ({v!r})"
                ) from exc
            if not math.isfinite(vf):
                raise ValueError(f"r2_jtc_goal: positions[{name!r}][{i}] is not " f"finite ({vf})")
            coerced.append(vf)
        per_joint.append(tuple(coerced))
    # Transpose to row-major (one row per time sample, in joint order).
    rows = tuple(tuple(per_joint[j][i] for j in range(len(joint_names))) for i in range(n_samples))
    return rows


# ---------------------------------------------------------------------------
# Public builder
# ---------------------------------------------------------------------------


def jtc_goal_from_trace(
    trace,
    *,
    time_offset_s: float = 0.0,
    skip_initial_sample: bool = False,
) -> JointTrajectoryGoal:
    """Convert a commanded-trace object into a :class:`JointTrajectoryGoal`.

    Parameters
    ----------
    trace:
        Any object exposing ``joint_names``, ``times``, ``positions``
        attributes matching the contract in the module docstring.
        ``JointCommandTrace``, ``AllJointsCommandTrace``, and
        ``JointCommandTrajectory`` from the sibling R2 modules all
        qualify.
    time_offset_s:
        Additive shift applied to every ``time_from_start_s`` of the
        output. Must be finite and ``>= 0.0``. Default ``0.0``.
    skip_initial_sample:
        When ``True``, drop ``times[0]`` (and the matching commanded
        positions) before building points. Requires
        ``len(trace.times) >= 2``. Default ``False``.

    Returns
    -------
    JointTrajectoryGoal
        Frozen dataclass whose ``joint_names`` echo the input's and
        whose ``points`` are in the same sample order (after the
        optional skip) with ``time_from_start_s`` rebased to the first
        surviving sample (``trace.times[start]``) plus ``time_offset_s``.

    Raises
    ------
    TypeError
        If ``trace`` is missing required attributes or any attribute
        has the wrong type.
    ValueError
        On empty / duplicate joint names, non-monotonic times, length
        mismatches, non-finite values, missing or extra ``positions``
        keys, negative ``time_offset_s``, or ``skip_initial_sample``
        with fewer than two samples.
    """
    try:
        raw_names = _require_attr(trace, "joint_names")
        raw_times = _require_attr(trace, "times")
        raw_positions = _require_attr(trace, "positions")
    except TypeError:
        raise

    if not math.isfinite(time_offset_s):
        raise ValueError(f"r2_jtc_goal: time_offset_s must be finite (got {time_offset_s})")
    if time_offset_s < 0.0:
        raise ValueError(f"r2_jtc_goal: time_offset_s must be >= 0 (got {time_offset_s})")

    joint_names = _validate_joint_names(raw_names)
    times = _validate_times(raw_times)
    rows = _validate_positions(raw_positions, joint_names, len(times))

    if skip_initial_sample:
        if len(times) < 2:
            raise ValueError(
                "r2_jtc_goal: skip_initial_sample=True requires at least " "2 samples (got 1)"
            )
        start = 1
    else:
        start = 0

    t_ref = times[start]
    points = tuple(
        JointTrajectoryPoint(
            positions=rows[i],
            time_from_start_s=float(times[i] - t_ref + time_offset_s),
        )
        for i in range(start, len(times))
    )
    return JointTrajectoryGoal(joint_names=joint_names, points=points)
