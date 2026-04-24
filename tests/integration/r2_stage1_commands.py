"""R2 stage-1 commanded-signal generators (single-joint step + sine).

Pre-baked companion to :mod:`tests.integration.r2_stage1_assertions`.
Authored as part of the M6.12 pre-bake chain (still gated on the
M6.0 operator decision before it can run live against the sim).

Scope
-----

R2 stage 1 in ``docs/ROADMAP.md`` says, verbatim:

    For each of the six joints individually, hold the other five at
    home and command a ±30° step + a 0.5 Hz sine.

This module emits the *commanded* side of that plan — per-joint
position traces `q_cmd(t)` for every joint in the arm, with exactly
one joint perturbed (step or sine) and the remaining five held at
their home positions. The measured side will come from
``/joint_states`` once the M6.0 / M6.5 gates clear; the stage-1
assertion harness already knows how to consume the pair.

Why separate from stage 3?
--------------------------

Stage 3 is task-space (``TcpTrajectory`` in SE(3)). Stage 1 is
joint-space with one axis active at a time. Emitting them in
separate modules keeps each generator single-purpose and mirrors
the per-stage assertion harnesses (`r2_stage1_assertions` /
`r2_stage3_assertions`) so a test body never has to untangle which
namespace it should pull from.

Design mirrors the sibling stage-3 commands module:

* Typed, frozen :class:`JointCommandTrace` dataclass — immutable
  tuples so a trace can be reused across multiple test scenarios
  without defensive copies.
* Pure stdlib; no numpy, no ROS, no MuJoCo.
* Raises :class:`ValueError` on genuinely inconsistent inputs
  (duplicate joint names, ``active_joint`` not in ``joint_names``,
  non-positive ``duration_s`` / ``dt_s``, non-finite values,
  non-positive ``frequency_hz``, ``t_step_s`` outside
  ``[0, duration_s]``).
* ``times[0] == 0.0`` and ``times[-1] == duration_s`` exactly,
  matching the ``TcpTrajectory`` invariant so downstream consumers
  (stage-2 / stage-3 evaluators, FK adapters, artefact writers)
  don't have to rebase timebases.

Scalar step target
------------------

The stage-1 damping-ratio-from-step evaluator (see
``r2_stage1_assertions.evaluate_second_order`` / ``evaluate_position_mode``)
takes a *scalar* ``target`` for the active joint, not a full trace.
``JointCommandTrace`` therefore records the commanded scalar target
for the active joint (``target_rad``) as metadata so the stage-1 test
body can pass it through without re-deriving from the trace.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from types import MappingProxyType
from typing import Mapping, Sequence, Tuple

__all__ = (
    "JointCommandTrace",
    "step_command",
    "sine_command",
)

# Smallest sensible time step. Matches ``r2_stage3_commands._MIN_DT_S``
# so the two modules reject the same float-vs-ms unit mistakes.
_MIN_DT_S = 1e-5


# ---------------------------------------------------------------------------
# Result dataclass
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class JointCommandTrace:
    """Parallel-array joint-space commanded trajectory.

    Attributes
    ----------
    joint_names:
        Ordered tuple of joint names. All entries of
        ``positions`` are keyed by these names.
    times:
        Strictly monotonic time vector (s). ``times[0] == 0.0`` and
        ``times[-1] == duration_s`` (within floating-point rounding).
    positions:
        Read-only mapping ``joint_name -> q_cmd(t) trace`` (rad).
        Every trace has ``len(times)`` samples.
    active_joint:
        The one joint that is being perturbed by this command. The
        other joints in ``joint_names`` are held at their home
        positions for all samples.
    target_rad:
        Scalar final target (rad) for the *active* joint, suitable
        for feeding into ``r2_stage1_assertions.evaluate_second_order``
        / ``evaluate_position_mode`` which take a scalar ``target``.
        For a step command this is ``home + step_rad``; for a sine
        command (where the commanded signal has no single final
        target) this is the mean of the sine, i.e. ``home`` — the
        value the joint would settle at once excitation stops.
    """

    joint_names: Tuple[str, ...]
    times: Tuple[float, ...]
    positions: Mapping[str, Tuple[float, ...]]
    active_joint: str
    target_rad: float

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
        raise ValueError(f"r2_stage1_commands: duration_s must be finite and > 0, got {duration_s}")
    if not math.isfinite(dt_s) or dt_s < _MIN_DT_S:
        raise ValueError(f"r2_stage1_commands: dt_s must be finite and >= {_MIN_DT_S}, got {dt_s}")
    if dt_s > duration_s:
        raise ValueError(f"r2_stage1_commands: dt_s={dt_s} exceeds duration_s={duration_s}")
    n_intervals = int(round(duration_s / dt_s))
    if n_intervals < 1:
        raise ValueError(
            f"r2_stage1_commands: duration_s/dt_s rounds to 0 intervals "
            f"(duration_s={duration_s}, dt_s={dt_s})"
        )
    exact_dt = duration_s / n_intervals
    times = tuple(i * exact_dt for i in range(n_intervals + 1))
    # Pin the final sample to exactly duration_s so downstream
    # rebase-free consumers see times[-1] == duration_s.
    times = times[:-1] + (float(duration_s),)
    return times


def _validate_joint_layout(
    joint_names: Sequence[str],
    home_positions_rad: Sequence[float],
    active_joint: str,
) -> Tuple[Tuple[str, ...], Tuple[float, ...]]:
    names = tuple(joint_names)
    if not names:
        raise ValueError("r2_stage1_commands: joint_names must be non-empty")
    if len(set(names)) != len(names):
        raise ValueError(f"r2_stage1_commands: joint_names contains duplicates: {names}")
    if len(home_positions_rad) != len(names):
        raise ValueError(
            "r2_stage1_commands: home_positions_rad must have the same length as "
            f"joint_names (got {len(home_positions_rad)} vs {len(names)})"
        )
    homes = []
    for name, q in zip(names, home_positions_rad):
        qf = float(q)
        if not math.isfinite(qf):
            raise ValueError(
                f"r2_stage1_commands: home_positions_rad[{name!r}] is not finite ({qf})"
            )
        homes.append(qf)
    if active_joint not in names:
        raise ValueError(
            f"r2_stage1_commands: active_joint={active_joint!r} not in joint_names={list(names)}"
        )
    return names, tuple(homes)


def _finite_value(name: str, value: float) -> float:
    vf = float(value)
    if not math.isfinite(vf):
        raise ValueError(f"r2_stage1_commands: {name} is not finite ({vf})")
    return vf


def _build_positions(
    joint_names: Tuple[str, ...],
    homes: Tuple[float, ...],
    active_joint: str,
    active_trace: Tuple[float, ...],
) -> Mapping[str, Tuple[float, ...]]:
    n = len(active_trace)
    positions = {}
    for name, home in zip(joint_names, homes):
        if name == active_joint:
            positions[name] = active_trace
        else:
            positions[name] = tuple(home for _ in range(n))
    return MappingProxyType(positions)


# ---------------------------------------------------------------------------
# Public generators
# ---------------------------------------------------------------------------


def step_command(
    *,
    joint_names: Sequence[str],
    home_positions_rad: Sequence[float],
    active_joint: str,
    step_rad: float,
    duration_s: float,
    dt_s: float,
    t_step_s: float = 0.0,
) -> JointCommandTrace:
    """Single-joint step command with the rest held at home.

    Before ``t_step_s`` every joint sits at its home position. From
    ``t_step_s`` onward the ``active_joint`` jumps to
    ``home + step_rad`` (instantaneous, zero-order-hold); the
    remaining joints stay at home for the full duration.

    Parameters
    ----------
    joint_names:
        Ordered joint names (must be unique, non-empty).
    home_positions_rad:
        Home-pose joint positions, same order as ``joint_names``.
    active_joint:
        Which joint receives the step. Must be in ``joint_names``.
    step_rad:
        Step magnitude (rad). For the ROADMAP ``±30°`` plan pass
        ``math.radians(30)`` or ``math.radians(-30)``.
    duration_s, dt_s:
        Trajectory duration and sampling period (s). The returned
        ``times`` contains exactly ``round(duration_s/dt_s) + 1``
        samples with ``times[0] == 0.0`` and ``times[-1] == duration_s``.
    t_step_s:
        When the step fires, in seconds from trajectory start. Must
        satisfy ``0.0 <= t_step_s <= duration_s``. Default ``0.0`` —
        the active joint starts at ``home + step_rad`` immediately.

    Returns
    -------
    JointCommandTrace
        With ``target_rad = home(active_joint) + step_rad``, suitable
        for feeding the scalar ``target`` argument of
        ``r2_stage1_assertions.evaluate_second_order`` and
        ``evaluate_position_mode``.
    """
    names, homes = _validate_joint_layout(joint_names, home_positions_rad, active_joint)
    step = _finite_value("step_rad", step_rad)
    times = _validate_timing(duration_s, dt_s)

    t_step = _finite_value("t_step_s", t_step_s)
    if t_step < 0.0 or t_step > duration_s:
        raise ValueError(
            "r2_stage1_commands: t_step_s must satisfy 0.0 <= t_step_s <= duration_s "
            f"(got t_step_s={t_step}, duration_s={duration_s})"
        )

    home_active = homes[names.index(active_joint)]
    target = home_active + step

    active_trace = tuple(home_active if t < t_step else target for t in times)

    return JointCommandTrace(
        joint_names=names,
        times=times,
        positions=_build_positions(names, homes, active_joint, active_trace),
        active_joint=active_joint,
        target_rad=target,
    )


def sine_command(
    *,
    joint_names: Sequence[str],
    home_positions_rad: Sequence[float],
    active_joint: str,
    amplitude_rad: float,
    frequency_hz: float,
    duration_s: float,
    dt_s: float,
    phase_rad: float = 0.0,
) -> JointCommandTrace:
    """Single-joint sine excitation with the rest held at home.

    The active joint follows
    ``q_cmd(t) = home + amplitude_rad * sin(2π * frequency_hz * t + phase_rad)``;
    remaining joints stay at their home positions.

    Parameters
    ----------
    joint_names, home_positions_rad, active_joint:
        See :func:`step_command`.
    amplitude_rad:
        Sine amplitude (rad). Must be finite; zero is accepted (the
        active trace then degenerates to a constant ``home``) so
        tests can sweep the amplitude down to 0 without special
        casing.
    frequency_hz:
        Excitation frequency (Hz). Must be finite and > 0 — the
        ROADMAP baseline is ``0.5 Hz``.
    duration_s, dt_s:
        See :func:`step_command`.
    phase_rad:
        Optional phase offset (rad), default ``0.0`` (sine starts at
        home and rises first). Must be finite.

    Returns
    -------
    JointCommandTrace
        With ``target_rad = home(active_joint)`` — the value the
        commanded signal is centred on, which is what the stage-1
        evaluators should use as the steady-state reference for
        drift / steady-state-error checks on a sine input.
    """
    names, homes = _validate_joint_layout(joint_names, home_positions_rad, active_joint)
    amp = _finite_value("amplitude_rad", amplitude_rad)
    freq = _finite_value("frequency_hz", frequency_hz)
    if freq <= 0.0:
        raise ValueError(f"r2_stage1_commands: frequency_hz must be > 0, got {freq}")
    phase = _finite_value("phase_rad", phase_rad)
    times = _validate_timing(duration_s, dt_s)

    home_active = homes[names.index(active_joint)]
    omega = 2.0 * math.pi * freq
    active_trace = tuple(home_active + amp * math.sin(omega * t + phase) for t in times)

    return JointCommandTrace(
        joint_names=names,
        times=times,
        positions=_build_positions(names, homes, active_joint, active_trace),
        active_joint=active_joint,
        target_rad=home_active,
    )
