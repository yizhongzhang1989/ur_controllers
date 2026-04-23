"""Reference-trajectory generation for evaluation scenarios (schema v1).

Pure-Python, ROS-free, deterministic. Given a validated scenario document
(see ``evaluation/scenarios/README.md``) and an initial joint configuration,
produces a list of reference samples ``[(t_s, target_vector), ...]``
spanning ``[0, duration_s]`` at a fixed rate.

The generator is split out so:

* unit tests can exercise every ``scenario_type`` without ROS, and
* the orchestration layer in ``run_evaluation.py`` only has to call one
  function regardless of which scenario it is driving.

Currently only joint-space reference vectors are produced (length-6 vectors
in canonical UR joint order). Cartesian regulation is handled separately in
``run_evaluation.py`` because it has a different message shape and is
constant for the full window.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass

DEFAULT_RATE_HZ = 50.0
N_JOINTS = 6


@dataclass(frozen=True)
class JointReferenceSample:
    """One reference sample on the joint-space command stream."""

    t_s: float
    position: tuple[float, ...]  # length 6


def _check_initial(initial: list[float] | tuple[float, ...]) -> tuple[float, ...]:
    if len(initial) != N_JOINTS:
        raise ValueError(
            f"initial must be a length-{N_JOINTS} vector, got {len(initial)}"
        )
    return tuple(float(x) for x in initial)


def _times(duration_s: float, rate_hz: float) -> list[float]:
    if duration_s <= 0:
        raise ValueError(f"duration_s must be > 0, got {duration_s}")
    if rate_hz <= 0:
        raise ValueError(f"rate_hz must be > 0, got {rate_hz}")
    n = int(math.floor(duration_s * rate_hz)) + 1
    return [i / rate_hz for i in range(n)]


def generate_step(
    initial: list[float] | tuple[float, ...],
    per_joint_amplitude_rad: list[float] | tuple[float, ...],
    step_time_s: float,
    duration_s: float,
    rate_hz: float = DEFAULT_RATE_HZ,
) -> list[JointReferenceSample]:
    """Reference is ``initial`` for ``t < step_time_s``, then ``initial+amp``."""
    q0 = _check_initial(initial)
    if len(per_joint_amplitude_rad) != N_JOINTS:
        raise ValueError(
            "per_joint_amplitude_rad must be length 6, got "
            f"{len(per_joint_amplitude_rad)}"
        )
    if step_time_s < 0 or step_time_s >= duration_s:
        raise ValueError(
            f"step_time_s must satisfy 0 <= step_time_s < duration_s "
            f"({step_time_s} vs {duration_s})"
        )
    amp = tuple(float(a) for a in per_joint_amplitude_rad)
    samples: list[JointReferenceSample] = []
    for t in _times(duration_s, rate_hz):
        if t < step_time_s:
            samples.append(JointReferenceSample(t, q0))
        else:
            samples.append(
                JointReferenceSample(
                    t, tuple(q0[i] + amp[i] for i in range(N_JOINTS))
                )
            )
    return samples


def generate_sine(
    initial: list[float] | tuple[float, ...],
    per_joint_amplitude_rad: list[float] | tuple[float, ...],
    frequency_hz: float,
    phase_rad: float,
    duration_s: float,
    rate_hz: float = DEFAULT_RATE_HZ,
) -> list[JointReferenceSample]:
    """Reference is ``initial + amp * sin(2*pi*f*t + phase)`` per joint."""
    q0 = _check_initial(initial)
    if len(per_joint_amplitude_rad) != N_JOINTS:
        raise ValueError(
            "per_joint_amplitude_rad must be length 6, got "
            f"{len(per_joint_amplitude_rad)}"
        )
    if frequency_hz <= 0:
        raise ValueError(f"frequency_hz must be > 0, got {frequency_hz}")
    amp = tuple(float(a) for a in per_joint_amplitude_rad)
    omega = 2.0 * math.pi * float(frequency_hz)
    samples: list[JointReferenceSample] = []
    for t in _times(duration_s, rate_hz):
        s = math.sin(omega * t + float(phase_rad))
        samples.append(
            JointReferenceSample(
                t, tuple(q0[i] + amp[i] * s for i in range(N_JOINTS))
            )
        )
    return samples


def generate_regulation_joint(
    initial: list[float] | tuple[float, ...],
    hold,
    duration_s: float,
    rate_hz: float = DEFAULT_RATE_HZ,
) -> list[JointReferenceSample]:
    """Constant reference. ``hold`` is ``"initial"`` or a length-6 vector."""
    q0 = _check_initial(initial)
    if hold == "initial":
        target = q0
    else:
        if not isinstance(hold, (list, tuple)) or len(hold) != N_JOINTS:
            raise ValueError(
                "regulation hold must be 'initial' or a length-6 vector"
            )
        target = tuple(float(x) for x in hold)
    return [JointReferenceSample(t, target) for t in _times(duration_s, rate_hz)]


def generate_random_waypoints(
    initial: list[float] | tuple[float, ...],
    num_waypoints: int,
    dwell_s: float,
    seed: int,
    bounds_lower: list[float] | tuple[float, ...],
    bounds_upper: list[float] | tuple[float, ...],
    duration_s: float,
    rate_hz: float = DEFAULT_RATE_HZ,
) -> list[JointReferenceSample]:
    """Pick ``num_waypoints`` uniform-random joint targets in ``[lower, upper]``,
    each held for ``dwell_s`` seconds. After the last waypoint is consumed,
    the reference latches on the final waypoint until ``duration_s``.
    """
    _check_initial(initial)
    if num_waypoints < 1:
        raise ValueError(f"num_waypoints must be >= 1, got {num_waypoints}")
    if dwell_s <= 0:
        raise ValueError(f"dwell_s must be > 0, got {dwell_s}")
    if num_waypoints * dwell_s > duration_s + 1e-9:
        raise ValueError(
            f"num_waypoints*dwell_s ({num_waypoints * dwell_s}) "
            f"must be <= duration_s ({duration_s})"
        )
    if len(bounds_lower) != N_JOINTS or len(bounds_upper) != N_JOINTS:
        raise ValueError("bounds_lower and bounds_upper must be length 6")
    lo = [float(x) for x in bounds_lower]
    hi = [float(x) for x in bounds_upper]
    for i, (a, b) in enumerate(zip(lo, hi)):
        if not a < b:
            raise ValueError(f"bounds: lower[{i}] ({a}) must be < upper[{i}] ({b})")
    rng = random.Random(int(seed))
    waypoints: list[tuple[float, ...]] = [
        tuple(rng.uniform(lo[i], hi[i]) for i in range(N_JOINTS))
        for _ in range(int(num_waypoints))
    ]
    samples: list[JointReferenceSample] = []
    for t in _times(duration_s, rate_hz):
        idx = min(int(t // dwell_s), num_waypoints - 1)
        samples.append(JointReferenceSample(t, waypoints[idx]))
    return samples


def generate_joint_reference(
    scenario: dict,
    initial: list[float] | tuple[float, ...],
    rate_hz: float = DEFAULT_RATE_HZ,
) -> list[JointReferenceSample]:
    """Dispatch to the per-``scenario_type`` generator above.

    Only valid for ``target.space == 'joint'``. Cartesian scenarios are
    handled in ``run_evaluation.py`` (different command shape, currently
    constant).
    """
    target = scenario.get("target", {})
    if target.get("space") != "joint":
        raise ValueError(
            "generate_joint_reference: scenario.target.space must be 'joint'"
        )
    stype = scenario.get("scenario_type")
    cmd = scenario.get("command", {})
    duration_s = float(scenario["duration_s"])
    if stype == "step":
        return generate_step(
            initial,
            cmd["per_joint_amplitude_rad"],
            float(cmd["step_time_s"]),
            duration_s,
            rate_hz,
        )
    if stype == "sine":
        return generate_sine(
            initial,
            cmd["per_joint_amplitude_rad"],
            float(cmd["frequency_hz"]),
            float(cmd.get("phase_rad", 0.0)),
            duration_s,
            rate_hz,
        )
    if stype == "regulation":
        return generate_regulation_joint(
            initial, cmd.get("hold", "initial"), duration_s, rate_hz
        )
    if stype == "random_waypoints":
        bounds = cmd["bounds"]
        if bounds == "urdf":
            raise ValueError(
                "random_waypoints with bounds='urdf' must be resolved to "
                "explicit lower/upper before calling generate_joint_reference"
            )
        return generate_random_waypoints(
            initial,
            int(cmd["num_waypoints"]),
            float(cmd["dwell_s"]),
            int(cmd["seed"]),
            bounds["lower"],
            bounds["upper"],
            duration_s,
            rate_hz,
        )
    raise ValueError(f"unknown scenario_type: {stype!r}")
