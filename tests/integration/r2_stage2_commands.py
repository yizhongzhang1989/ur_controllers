"""R2 stage-2 commanded-signal generators (all-joints home→pose→home).

Pre-baked companion to :mod:`tests.integration.r2_stage2_assertions`.
Authored as part of the M6.13 pre-bake chain (still gated on the
M6.0 operator decision before it can run live against the sim).

Scope
-----

R2 stage 2 in ``docs/ROADMAP.md`` says, verbatim:

    Commanded home → hand-picked cluttered pose → home, over 5 s,
    all six joints simultaneously.

This module emits the *commanded* side of that plan — a parallel
``joint_name -> q_cmd(t)`` mapping that the stage-2 assertion harness
(:func:`r2_stage2_assertions.evaluate_all_joints_joint_space`) already
knows how to ingest. The measured side comes from ``/joint_states``
once the M6.0 / M6.5 gates clear.

Why separate from stage 1 / stage 3?
------------------------------------

* Stage 1 is single-joint excitation (one active joint, the rest
  pinned at home). ``JointCommandTrace`` carries an ``active_joint``
  and a scalar ``target_rad``.
* Stage 3 is task-space (``TcpTrajectory`` in SE(3)).
* Stage 2 is joint-space **with every joint moving simultaneously**
  through a user-supplied cluttered pose. Emitting it as its own
  module keeps each generator single-purpose and matches the
  per-stage assertion harnesses.

Blend shape
-----------

The trajectory interpolates each joint independently with a cosine
pulse through the via pose at ``t = duration_s / 2``::

    s(t) = 0.5 * (1 - cos(2π * t / T))
    q_i(t) = home_i + (via_i - home_i) * s(t)

This has the properties stage-2 needs:

* ``q_i(0) == q_i(T) == home_i`` — the motion starts and ends at home.
* ``q_i(T/2) == via_i`` — passes exactly through the operator-chosen
  cluttered pose at the midpoint.
* ``dq/dt == 0`` at ``t ∈ {0, T/2, T}`` — smooth start, peak, and
  stop. No torque spike at the commanded level; any chatter in the
  measured trace is then attributable to the controller, not the
  commanded signal.

Design mirrors the sibling stage-1 / stage-3 commanded-signal modules:

* Typed, frozen :class:`AllJointsCommandTrace` dataclass — immutable
  tuples so a trace can be reused across multiple test scenarios
  without defensive copies.
* Pure stdlib; no numpy, no ROS, no MuJoCo.
* Raises :class:`ValueError` on genuinely inconsistent inputs
  (duplicate joint names, non-finite values, mismatched lengths,
  non-positive ``duration_s`` / ``dt_s``).
* ``times[0] == 0.0`` and ``times[-1] == duration_s`` exactly,
  matching the other stage modules' invariant so downstream
  consumers don't have to rebase timebases.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from types import MappingProxyType
from typing import Mapping, Sequence, Tuple

__all__ = (
    "AllJointsCommandTrace",
    "home_to_pose_to_home_command",
)

# Smallest sensible time step. Matches ``r2_stage1_commands._MIN_DT_S``
# and ``r2_stage3_commands._MIN_DT_S`` so the stage modules reject
# identical float-vs-ms unit mistakes.
_MIN_DT_S = 1e-5


# ---------------------------------------------------------------------------
# Result dataclass
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class AllJointsCommandTrace:
    """Parallel-array all-joints commanded trajectory.

    Attributes
    ----------
    joint_names:
        Ordered tuple of joint names. All entries of ``positions``
        are keyed by these names.
    times:
        Strictly monotonic time vector (s). ``times[0] == 0.0`` and
        ``times[-1] == duration_s`` (within floating-point rounding).
    positions:
        Read-only mapping ``joint_name -> q_cmd(t) trace`` (rad).
        Every trace has ``len(times)`` samples.
    home_positions_rad:
        Echo of the home pose, same order as ``joint_names``.
    via_pose_rad:
        Echo of the commanded via pose, same order as ``joint_names``.
    """

    joint_names: Tuple[str, ...]
    times: Tuple[float, ...]
    positions: Mapping[str, Tuple[float, ...]]
    home_positions_rad: Tuple[float, ...]
    via_pose_rad: Tuple[float, ...]

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


def _validate_timing(duration_s: float, dt_s: float) -> Tuple[float, ...]:
    if not math.isfinite(duration_s) or duration_s <= 0.0:
        raise ValueError(f"r2_stage2_commands: duration_s must be finite and > 0, got {duration_s}")
    if not math.isfinite(dt_s) or dt_s < _MIN_DT_S:
        raise ValueError(f"r2_stage2_commands: dt_s must be finite and >= {_MIN_DT_S}, got {dt_s}")
    if dt_s > duration_s:
        raise ValueError(f"r2_stage2_commands: dt_s={dt_s} exceeds duration_s={duration_s}")
    n_intervals = int(round(duration_s / dt_s))
    if n_intervals < 2:
        # Stage-2 needs at least a midpoint sample between the two
        # home endpoints for the via pose to actually show up in the
        # trace; reject degenerate 1-interval traces.
        raise ValueError(
            f"r2_stage2_commands: duration_s/dt_s rounds to < 2 intervals "
            f"(duration_s={duration_s}, dt_s={dt_s}); stage-2 requires a "
            f"midpoint sample for the via pose"
        )
    exact_dt = duration_s / n_intervals
    times = tuple(i * exact_dt for i in range(n_intervals + 1))
    # Pin the final sample to exactly duration_s so downstream
    # rebase-free consumers see times[-1] == duration_s.
    times = times[:-1] + (float(duration_s),)
    return times


def _validate_finite_sequence(name: str, values: Sequence[float]) -> Tuple[float, ...]:
    out = []
    for i, v in enumerate(values):
        vf = float(v)
        if not math.isfinite(vf):
            raise ValueError(f"r2_stage2_commands: {name}[{i}] is not finite ({vf})")
        out.append(vf)
    return tuple(out)


def _validate_joint_layout(
    joint_names: Sequence[str],
    home_positions_rad: Sequence[float],
    via_pose_rad: Sequence[float],
) -> Tuple[Tuple[str, ...], Tuple[float, ...], Tuple[float, ...]]:
    names = tuple(joint_names)
    if not names:
        raise ValueError("r2_stage2_commands: joint_names must be non-empty")
    if len(set(names)) != len(names):
        raise ValueError(f"r2_stage2_commands: joint_names contains duplicates: {names}")
    if len(home_positions_rad) != len(names):
        raise ValueError(
            "r2_stage2_commands: home_positions_rad must have the same length "
            f"as joint_names (got {len(home_positions_rad)} vs {len(names)})"
        )
    if len(via_pose_rad) != len(names):
        raise ValueError(
            "r2_stage2_commands: via_pose_rad must have the same length "
            f"as joint_names (got {len(via_pose_rad)} vs {len(names)})"
        )
    homes = _validate_finite_sequence("home_positions_rad", home_positions_rad)
    vias = _validate_finite_sequence("via_pose_rad", via_pose_rad)
    return names, homes, vias


# ---------------------------------------------------------------------------
# Public generator
# ---------------------------------------------------------------------------


def home_to_pose_to_home_command(
    *,
    joint_names: Sequence[str],
    home_positions_rad: Sequence[float],
    via_pose_rad: Sequence[float],
    duration_s: float,
    dt_s: float,
) -> AllJointsCommandTrace:
    """All-joints commanded trajectory: home → via_pose → home.

    Each joint is interpolated independently with a cosine pulse
    peaking at the supplied ``via_pose`` at ``t = duration_s / 2``::

        q_i(t) = home_i + (via_i - home_i) * 0.5 * (1 - cos(2π t / T))

    The commanded trace starts and ends at home with zero velocity,
    so any chatter in the measured trace at t = 0 or t = T is a
    controller artefact, not a commanded-signal artefact.

    Parameters
    ----------
    joint_names:
        Ordered joint names (must be unique, non-empty).
    home_positions_rad:
        Home-pose joint positions, same order as ``joint_names``.
    via_pose_rad:
        Cluttered pose the trajectory passes through at the midpoint,
        same order as ``joint_names``.
    duration_s, dt_s:
        Trajectory duration and sampling period (s). The returned
        ``times`` contains exactly ``round(duration_s/dt_s) + 1``
        samples with ``times[0] == 0.0`` and
        ``times[-1] == duration_s``. ``duration_s / dt_s`` must round
        to at least 2 intervals so the midpoint via sample lands in
        the trace.

    Returns
    -------
    AllJointsCommandTrace
        Parallel-array trace consumable by
        :func:`r2_stage2_assertions.evaluate_all_joints_joint_space`.
    """
    names, homes, vias = _validate_joint_layout(joint_names, home_positions_rad, via_pose_rad)
    times = _validate_timing(duration_s, dt_s)

    # Pre-compute the blend weight once per sample — same weight for
    # every joint keeps the math vectorisable if we ever drop numpy in.
    two_pi_over_T = 2.0 * math.pi / duration_s
    weights = tuple(0.5 * (1.0 - math.cos(two_pi_over_T * t)) for t in times)

    positions = {}
    for name, home, via in zip(names, homes, vias):
        delta = via - home
        positions[name] = tuple(home + delta * w for w in weights)

    return AllJointsCommandTrace(
        joint_names=names,
        times=times,
        positions=MappingProxyType(positions),
        home_positions_rad=homes,
        via_pose_rad=vias,
    )
