"""R2 stage-3 commanded TCP trajectory generators.

Pre-baked companion to :mod:`tests.integration.r2_stage3_assertions`.
The stage-3 assertion harness consumes *parallel arrays* of
``(times, positions, quats)`` for both the **commanded** and the
**measured** side of a TCP trajectory. This module emits the
commanded side for the three trajectory shapes the ROADMAP R2 stage-3
bullet calls out verbatim:

    Command a known TCP trajectory (line, then arc, then sine-in-z)
    via ``cartesian_motion_controller`` and — separately — via JTC
    driven by an IK solver shim.

Why a separate module?
----------------------

* The **measured** TCP trajectory is produced by FK over the
  sim/real-robot ``/joint_states`` stream, which is still gated on
  the FK-source decision noted in ``docs/STATUS.md``.
* The **commanded** TCP trajectory is a purely kinematic, deterministic
  signal the test knows up front — no ROS, no MuJoCo, no FK.

Emitting it here as pure stdlib keeps the stage-3 orchestrator small
once the FK source and the M6.0 gate clear: the orchestrator builds
a commanded trajectory with these helpers, streams the matching
goals to the controller, collects the measured-side samples via FK,
and hands both sides to ``evaluate_tcp_trajectory``.

Design mirrors the sibling stage-1 / stage-2 / stage-3-assertion
modules:

* Typed, frozen :class:`TcpTrajectory` dataclass — immutable tuples
  so a trajectory can be reused across multiple test scenarios
  without defensive copies.
* Pure stdlib; no numpy, no ROS, no MuJoCo.
* ``ValueError`` on genuinely inconsistent inputs (non-positive
  ``duration_s`` / ``dt_s``, zero-length arc radius, non-finite
  values, unknown ``plane`` keyword).

Orientation is held fixed along each generated trajectory. Real
tests overlay a commanded orientation slew if required; pinning the
orientation here keeps the generators single-purpose and makes it
obvious where orientation drift in a measured trace originates.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple

__all__ = (
    "TcpTrajectory",
    "arc_trajectory",
    "line_trajectory",
    "sine_in_z_trajectory",
)


Vec3 = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]  # (x, y, z, w), Hamilton / ROS

_IDENTITY_QUAT: Quat = (0.0, 0.0, 0.0, 1.0)

# Quaternion norm must sit within this band of 1.0 to be accepted as
# "unit-ish" — identical tolerance to :mod:`r2_stage3_assertions`.
_QUAT_NORM_MIN = 0.5
_QUAT_NORM_MAX = 1.5

# Smallest sensible time step. Controllers in this workspace run at
# 500 Hz (crisp, simple_jimp) or 125 Hz (cartesian_motion); anything
# below 10 µs is almost certainly a unit mistake (ms passed as s).
_MIN_DT_S = 1e-5


# ---------------------------------------------------------------------------
# Result dataclass
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class TcpTrajectory:
    """Parallel-array TCP trajectory ready for the stage-3 evaluator.

    Attributes
    ----------
    times:
        Strictly monotonic time vector (s). ``times[0] == 0.0`` and
        ``times[-1] == duration_s`` (within floating-point rounding).
    positions:
        ``(x, y, z)`` per sample, in metres.
    orientations:
        ``(x, y, z, w)`` per sample, unit quaternions (Hamilton / ROS
        convention).
    """

    times: Tuple[float, ...]
    positions: Tuple[Vec3, ...]
    orientations: Tuple[Quat, ...]

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
        raise ValueError(f"r2_stage3_commands: duration_s must be finite and > 0, got {duration_s}")
    if not math.isfinite(dt_s) or dt_s < _MIN_DT_S:
        raise ValueError(f"r2_stage3_commands: dt_s must be finite and >= {_MIN_DT_S}, got {dt_s}")
    if dt_s > duration_s:
        raise ValueError(f"r2_stage3_commands: dt_s={dt_s} exceeds duration_s={duration_s}")
    # Sample count: N = round(duration/dt) + 1 so the trajectory
    # contains both endpoints. Using round() rather than floor() means
    # callers passing duration_s / dt_s that differ by float noise
    # still get the expected sample count.
    n_intervals = int(round(duration_s / dt_s))
    if n_intervals < 1:
        raise ValueError(
            f"r2_stage3_commands: duration_s/dt_s rounds to 0 intervals "
            f"(duration_s={duration_s}, dt_s={dt_s})"
        )
    # Recompute dt from the requested duration so times[-1] hits the
    # requested endpoint exactly, regardless of float rounding in
    # duration_s / dt_s.
    exact_dt = duration_s / n_intervals
    times = tuple(i * exact_dt for i in range(n_intervals + 1))
    # Guarantee that times[-1] == duration_s exactly; float accumulation
    # can drift by a few ULPs.
    times = times[:-1] + (float(duration_s),)
    return times


def _validate_vec3(name: str, v) -> Vec3:
    try:
        x, y, z = v
    except (TypeError, ValueError):
        raise ValueError(f"r2_stage3_commands: {name} must be a 3-tuple, got {v!r}") from None
    xf, yf, zf = float(x), float(y), float(z)
    for coord, coord_name in ((xf, "x"), (yf, "y"), (zf, "z")):
        if not math.isfinite(coord):
            raise ValueError(f"r2_stage3_commands: {name}.{coord_name} is not finite ({coord})")
    return (xf, yf, zf)


def _validate_quat(name: str, q) -> Quat:
    try:
        x, y, z, w = q
    except (TypeError, ValueError):
        raise ValueError(
            f"r2_stage3_commands: {name} must be a 4-tuple (x, y, z, w), got {q!r}"
        ) from None
    xf, yf, zf, wf = float(x), float(y), float(z), float(w)
    for coord, coord_name in ((xf, "x"), (yf, "y"), (zf, "z"), (wf, "w")):
        if not math.isfinite(coord):
            raise ValueError(f"r2_stage3_commands: {name}.{coord_name} is not finite ({coord})")
    norm = math.sqrt(xf * xf + yf * yf + zf * zf + wf * wf)
    if norm == 0.0 or not (_QUAT_NORM_MIN <= norm <= _QUAT_NORM_MAX):
        raise ValueError(
            f"r2_stage3_commands: {name}: degenerate quaternion norm {norm:.6g} "
            f"(expected within [{_QUAT_NORM_MIN}, {_QUAT_NORM_MAX}])"
        )
    # Normalise so the returned trajectory always carries unit quats.
    return (xf / norm, yf / norm, zf / norm, wf / norm)


# ---------------------------------------------------------------------------
# Public generators
# ---------------------------------------------------------------------------


def line_trajectory(
    *,
    start_pos_m: Vec3,
    end_pos_m: Vec3,
    duration_s: float,
    dt_s: float,
    orientation_quat: Quat = _IDENTITY_QUAT,
) -> TcpTrajectory:
    """Straight-line TCP trajectory at constant linear velocity.

    Position is linearly interpolated from ``start_pos_m`` at ``t=0``
    to ``end_pos_m`` at ``t=duration_s`` inclusive. Orientation is
    held at ``orientation_quat`` (defaults to identity).
    """
    s = _validate_vec3("start_pos_m", start_pos_m)
    e = _validate_vec3("end_pos_m", end_pos_m)
    q = _validate_quat("orientation_quat", orientation_quat)
    times = _validate_timing(duration_s, dt_s)

    n = len(times)
    duration = times[-1] - times[0]
    # duration > 0 guaranteed by _validate_timing (duration_s > 0).
    positions = []
    for i, t in enumerate(times):
        # Use i / (n-1) for the final sample to hit end_pos_m exactly.
        if i == n - 1:
            alpha = 1.0
        else:
            alpha = (t - times[0]) / duration
        positions.append(
            (
                s[0] + alpha * (e[0] - s[0]),
                s[1] + alpha * (e[1] - s[1]),
                s[2] + alpha * (e[2] - s[2]),
            )
        )
    orientations = tuple(q for _ in range(n))
    return TcpTrajectory(
        times=times,
        positions=tuple(positions),
        orientations=orientations,
    )


def arc_trajectory(
    *,
    center_m: Vec3,
    radius_m: float,
    start_angle_rad: float,
    end_angle_rad: float,
    plane: str = "xy",
    duration_s: float,
    dt_s: float,
    orientation_quat: Quat = _IDENTITY_QUAT,
) -> TcpTrajectory:
    """Planar circular-arc TCP trajectory at constant angular rate.

    The arc lies in one of the canonical world-axis planes (``"xy"``,
    ``"xz"``, or ``"yz"``), centred at ``center_m``, with the given
    ``radius_m``. The sweep is ``start_angle_rad`` → ``end_angle_rad``
    linearly in time (constant angular velocity). Out-of-plane
    coordinate is held at the ``center_m`` component. Orientation is
    held at ``orientation_quat``.
    """
    c = _validate_vec3("center_m", center_m)
    q = _validate_quat("orientation_quat", orientation_quat)
    times = _validate_timing(duration_s, dt_s)

    if not math.isfinite(radius_m) or radius_m <= 0.0:
        raise ValueError(f"r2_stage3_commands: radius_m must be finite and > 0, got {radius_m}")
    for ang, ang_name in ((start_angle_rad, "start_angle_rad"), (end_angle_rad, "end_angle_rad")):
        if not math.isfinite(ang):
            raise ValueError(f"r2_stage3_commands: {ang_name} is not finite ({ang})")
    if plane not in ("xy", "xz", "yz"):
        raise ValueError(
            f"r2_stage3_commands: plane must be one of 'xy', 'xz', 'yz'; got {plane!r}"
        )

    n = len(times)
    duration = times[-1] - times[0]
    d_ang = end_angle_rad - start_angle_rad

    positions = []
    for i, t in enumerate(times):
        if i == n - 1:
            alpha = 1.0
        else:
            alpha = (t - times[0]) / duration
        ang = start_angle_rad + alpha * d_ang
        cos_a = math.cos(ang)
        sin_a = math.sin(ang)
        if plane == "xy":
            positions.append((c[0] + radius_m * cos_a, c[1] + radius_m * sin_a, c[2]))
        elif plane == "xz":
            positions.append((c[0] + radius_m * cos_a, c[1], c[2] + radius_m * sin_a))
        else:  # "yz"
            positions.append((c[0], c[1] + radius_m * cos_a, c[2] + radius_m * sin_a))
    orientations = tuple(q for _ in range(n))
    return TcpTrajectory(
        times=times,
        positions=tuple(positions),
        orientations=orientations,
    )


def sine_in_z_trajectory(
    *,
    base_pos_m: Vec3,
    amplitude_m: float,
    frequency_hz: float,
    duration_s: float,
    dt_s: float,
    orientation_quat: Quat = _IDENTITY_QUAT,
    phase_rad: float = 0.0,
) -> TcpTrajectory:
    """Vertical-sine TCP trajectory about ``base_pos_m``.

    ``z(t) = base_pos_m.z + amplitude_m * sin(2π * frequency_hz * t +
    phase_rad)``. ``x`` and ``y`` are held at ``base_pos_m``.
    Orientation is held at ``orientation_quat``.
    """
    b = _validate_vec3("base_pos_m", base_pos_m)
    q = _validate_quat("orientation_quat", orientation_quat)
    times = _validate_timing(duration_s, dt_s)

    if not math.isfinite(amplitude_m):
        raise ValueError(f"r2_stage3_commands: amplitude_m is not finite ({amplitude_m})")
    if not math.isfinite(frequency_hz) or frequency_hz <= 0.0:
        raise ValueError(
            f"r2_stage3_commands: frequency_hz must be finite and > 0, got {frequency_hz}"
        )
    if not math.isfinite(phase_rad):
        raise ValueError(f"r2_stage3_commands: phase_rad is not finite ({phase_rad})")

    omega = 2.0 * math.pi * frequency_hz
    positions = tuple(
        (b[0], b[1], b[2] + amplitude_m * math.sin(omega * t + phase_rad)) for t in times
    )
    orientations = tuple(q for _ in range(len(times)))
    return TcpTrajectory(times=times, positions=positions, orientations=orientations)
