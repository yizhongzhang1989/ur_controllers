#!/usr/bin/env python3
"""Validator for evaluation scenario YAML files (schema v1).

The schema is documented in ``evaluation/scenarios/README.md``. This module
exposes two public functions:

* :func:`validate_scenario` — validate an already-parsed mapping.
* :func:`validate_file`     — load YAML and validate.

Both return a list of error strings; an empty list means the scenario is
valid. The CLI entrypoint validates one or more files and exits non-zero if
any of them produce errors.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any, Iterable

import yaml

CANONICAL_JOINTS: tuple[str, ...] = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
N_JOINTS = len(CANONICAL_JOINTS)

ALLOWED_ROBOTS = {"ur5e", "ur15"}
ALLOWED_SCENARIO_TYPES = {"step", "sine", "regulation", "random_waypoints"}
ALLOWED_SPACES = {"joint", "cartesian"}
ALLOWED_METRICS = {"rmse", "settling_time", "overshoot", "control_effort"}

JOINT_PASS_KEYS = {
    "max_rmse_rad",
    "max_settling_time_s",
    "max_overshoot_pct",
    "max_control_effort_nm",
}
CARTESIAN_PASS_KEYS = {
    "max_rmse_m",
    "max_rmse_rad",
    "max_settling_time_s",
    "max_control_effort_nm",
}

# Map pass_criteria key -> required metric name. Used to ensure thresholds
# refer to metrics that will actually be computed.
PASS_KEY_TO_METRIC: dict[str, str] = {
    "max_rmse_rad": "rmse",
    "max_rmse_m": "rmse",
    "max_settling_time_s": "settling_time",
    "max_overshoot_pct": "overshoot",
    "max_control_effort_nm": "control_effort",
}


def _is_finite_number(x: Any) -> bool:
    if isinstance(x, bool):  # bool is an int subclass — exclude
        return False
    if not isinstance(x, (int, float)):
        return False
    f = float(x)
    return f == f and f not in (float("inf"), float("-inf"))


def _check_float_vector(name: str, vec: Any, length: int, errors: list[str]) -> bool:
    if not isinstance(vec, list):
        errors.append(f"{name}: must be a list, got {type(vec).__name__}")
        return False
    if len(vec) != length:
        errors.append(f"{name}: must have length {length}, got {len(vec)}")
        return False
    ok = True
    for i, v in enumerate(vec):
        if not _is_finite_number(v):
            errors.append(f"{name}[{i}]: must be a finite number, got {v!r}")
            ok = False
    return ok


def _validate_target(target: Any, errors: list[str]) -> str | None:
    if not isinstance(target, dict):
        errors.append("target: must be a mapping")
        return None
    space = target.get("space")
    if space not in ALLOWED_SPACES:
        errors.append(
            f"target.space: must be one of {sorted(ALLOWED_SPACES)}, got {space!r}"
        )
        return None
    if space == "joint":
        joints = target.get("joints")
        if joints != list(CANONICAL_JOINTS):
            errors.append(
                "target.joints: must equal the canonical 6-joint list "
                f"{list(CANONICAL_JOINTS)}"
            )
        for forbidden in ("frame_id", "end_effector"):
            if forbidden in target:
                errors.append(
                    f"target.{forbidden}: not allowed when space==joint"
                )
    else:  # cartesian
        if "joints" in target:
            errors.append("target.joints: not allowed when space==cartesian")
        for required in ("frame_id", "end_effector"):
            v = target.get(required)
            if not isinstance(v, str) or not v:
                errors.append(
                    f"target.{required}: required non-empty string when space==cartesian"
                )
    return space


def _validate_command_step(cmd: dict, duration_s: float, errors: list[str]) -> None:
    _check_float_vector(
        "command.per_joint_amplitude_rad",
        cmd.get("per_joint_amplitude_rad"),
        N_JOINTS,
        errors,
    )
    if "amplitude_rad" in cmd:
        errors.append(
            "command.amplitude_rad: scalar amplitude is not accepted in schema v1; "
            "use per_joint_amplitude_rad (length 6, set inactive joints to 0.0)"
        )
    step_time = cmd.get("step_time_s")
    if not _is_finite_number(step_time) or float(step_time) < 0:
        errors.append("command.step_time_s: must be a finite number >= 0")
    elif float(step_time) >= duration_s:
        errors.append(
            f"command.step_time_s ({step_time}) must be < duration_s ({duration_s})"
        )


def _validate_command_sine(cmd: dict, errors: list[str]) -> None:
    _check_float_vector(
        "command.per_joint_amplitude_rad",
        cmd.get("per_joint_amplitude_rad"),
        N_JOINTS,
        errors,
    )
    if "amplitude_rad" in cmd:
        errors.append(
            "command.amplitude_rad: scalar amplitude is not accepted in schema v1; "
            "use per_joint_amplitude_rad"
        )
    freq = cmd.get("frequency_hz")
    if not _is_finite_number(freq) or float(freq) <= 0:
        errors.append("command.frequency_hz: must be a finite number > 0")
    if "phase_rad" in cmd and not _is_finite_number(cmd["phase_rad"]):
        errors.append("command.phase_rad: must be a finite number")


def _validate_command_regulation(cmd: dict, space: str, errors: list[str]) -> None:
    hold = cmd.get("hold")
    if hold == "initial":
        return
    if space == "joint":
        if not isinstance(hold, list):
            errors.append(
                "command.hold: must be 'initial' or a length-6 list of floats"
            )
            return
        _check_float_vector("command.hold", hold, N_JOINTS, errors)
    else:  # cartesian
        if not isinstance(hold, dict):
            errors.append(
                "command.hold: must be 'initial' or a mapping with "
                "position_xyz_m + orientation_xyzw"
            )
            return
        _check_float_vector(
            "command.hold.position_xyz_m", hold.get("position_xyz_m"), 3, errors
        )
        if _check_float_vector(
            "command.hold.orientation_xyzw",
            hold.get("orientation_xyzw"),
            4,
            errors,
        ):
            q = [float(x) for x in hold["orientation_xyzw"]]
            n2 = sum(x * x for x in q)
            if abs(n2 - 1.0) > 1e-3:
                errors.append(
                    f"command.hold.orientation_xyzw: must be a unit quaternion "
                    f"(|q|^2 = {n2:.6f})"
                )


def _validate_command_random_waypoints(
    cmd: dict, duration_s: float, errors: list[str]
) -> None:
    n = cmd.get("num_waypoints")
    if not isinstance(n, int) or isinstance(n, bool) or n < 1:
        errors.append("command.num_waypoints: must be an int >= 1")
        n = None
    dwell = cmd.get("dwell_s")
    if not _is_finite_number(dwell) or float(dwell) <= 0:
        errors.append("command.dwell_s: must be a finite number > 0")
        dwell = None
    if n is not None and dwell is not None and n * float(dwell) > duration_s:
        errors.append(
            f"command: num_waypoints * dwell_s ({n * float(dwell):.3f}) "
            f"must be <= duration_s ({duration_s})"
        )
    seed = cmd.get("seed")
    if not isinstance(seed, int) or isinstance(seed, bool):
        errors.append("command.seed: must be an int")
    bounds = cmd.get("bounds")
    if bounds == "urdf":
        return
    if not isinstance(bounds, dict):
        errors.append(
            "command.bounds: must be 'urdf' or a mapping with lower + upper"
        )
        return
    lower_ok = _check_float_vector(
        "command.bounds.lower", bounds.get("lower"), N_JOINTS, errors
    )
    upper_ok = _check_float_vector(
        "command.bounds.upper", bounds.get("upper"), N_JOINTS, errors
    )
    if lower_ok and upper_ok:
        for i, (lo, hi) in enumerate(zip(bounds["lower"], bounds["upper"])):
            if float(lo) >= float(hi):
                errors.append(
                    f"command.bounds: lower[{i}] ({lo}) must be < upper[{i}] ({hi})"
                )


def _validate_metrics(metrics: Any, errors: list[str]) -> set[str]:
    if not isinstance(metrics, list) or not metrics:
        errors.append("metrics: must be a non-empty list")
        return set()
    seen: set[str] = set()
    for m in metrics:
        if m in seen:
            errors.append(f"metrics: duplicate entry {m!r}")
        seen.add(m)
        if m not in ALLOWED_METRICS:
            errors.append(
                f"metrics: {m!r} is not one of {sorted(ALLOWED_METRICS)}"
            )
    return seen


def _validate_pass_criteria(
    pc: Any, space: str, declared_metrics: set[str], errors: list[str]
) -> None:
    if not isinstance(pc, dict) or not pc:
        errors.append("pass_criteria: must be a non-empty mapping")
        return
    allowed = JOINT_PASS_KEYS if space == "joint" else CARTESIAN_PASS_KEYS
    for key, val in pc.items():
        if key not in allowed:
            errors.append(
                f"pass_criteria.{key}: not allowed for space=={space} "
                f"(allowed: {sorted(allowed)})"
            )
            continue
        # All current pass_criteria values are floats; max_overshoot_pct may be 0.
        if not _is_finite_number(val):
            errors.append(f"pass_criteria.{key}: must be a finite number")
            continue
        fv = float(val)
        if key == "max_overshoot_pct":
            if fv < 0:
                errors.append(f"pass_criteria.{key}: must be >= 0")
        else:
            if fv <= 0:
                errors.append(f"pass_criteria.{key}: must be > 0")
        required_metric = PASS_KEY_TO_METRIC.get(key)
        if required_metric and required_metric not in declared_metrics:
            errors.append(
                f"pass_criteria.{key}: requires metric {required_metric!r} "
                f"to be listed under metrics"
            )


def _has_unique_strings(name: str, items: Any, errors: list[str]) -> None:
    if not isinstance(items, list) or not items:
        errors.append(f"{name}: must be a non-empty list")
        return
    seen: set[str] = set()
    for it in items:
        if not isinstance(it, str):
            errors.append(f"{name}: entries must be strings, got {it!r}")
            continue
        if it in seen:
            errors.append(f"{name}: duplicate entry {it!r}")
        seen.add(it)


def validate_scenario(doc: Any) -> list[str]:
    """Validate a parsed scenario mapping against schema v1."""
    errors: list[str] = []
    if not isinstance(doc, dict):
        return ["top-level: must be a mapping"]

    if doc.get("schema_version") != 1:
        errors.append(
            f"schema_version: must be 1, got {doc.get('schema_version')!r}"
        )

    name = doc.get("name")
    if not isinstance(name, str) or not name:
        errors.append("name: must be a non-empty string")
    elif name != name.lower() or " " in name or "_" in name:
        errors.append(f"name: must be kebab-case (lowercase, hyphenated), got {name!r}")

    if not isinstance(doc.get("description"), str) or not doc["description"]:
        errors.append("description: must be a non-empty string")

    stype = doc.get("scenario_type")
    if stype not in ALLOWED_SCENARIO_TYPES:
        errors.append(
            f"scenario_type: must be one of {sorted(ALLOWED_SCENARIO_TYPES)}, "
            f"got {stype!r}"
        )

    duration = doc.get("duration_s")
    duration_ok = _is_finite_number(duration) and float(duration) > 0
    if not duration_ok:
        errors.append("duration_s: must be a finite number > 0")

    robots = doc.get("robots")
    _has_unique_strings("robots", robots, errors)
    if isinstance(robots, list):
        for r in robots:
            if isinstance(r, str) and r not in ALLOWED_ROBOTS:
                errors.append(
                    f"robots: {r!r} is not one of {sorted(ALLOWED_ROBOTS)}"
                )

    space = _validate_target(doc.get("target"), errors)

    declared_metrics = _validate_metrics(doc.get("metrics"), errors)

    cmd = doc.get("command")
    if not isinstance(cmd, dict):
        errors.append("command: must be a mapping")
    elif stype in ALLOWED_SCENARIO_TYPES and duration_ok:
        d = float(duration)
        if stype == "step":
            if space != "joint":
                errors.append("scenario_type=step requires target.space==joint")
            else:
                _validate_command_step(cmd, d, errors)
        elif stype == "sine":
            if space != "joint":
                errors.append("scenario_type=sine requires target.space==joint")
            else:
                _validate_command_sine(cmd, errors)
        elif stype == "regulation":
            if space is not None:
                _validate_command_regulation(cmd, space, errors)
        elif stype == "random_waypoints":
            if space != "joint":
                errors.append(
                    "scenario_type=random_waypoints requires target.space==joint"
                )
            else:
                _validate_command_random_waypoints(cmd, d, errors)

    if space is not None:
        _validate_pass_criteria(
            doc.get("pass_criteria"), space, declared_metrics, errors
        )

    known_top = {
        "schema_version",
        "name",
        "description",
        "scenario_type",
        "duration_s",
        "robots",
        "target",
        "command",
        "metrics",
        "pass_criteria",
    }
    for k in doc.keys():
        if k not in known_top:
            errors.append(f"unknown top-level key: {k!r}")

    return errors


def validate_file(path: str | Path) -> list[str]:
    """Load YAML from ``path`` and validate it. IO/parse errors are returned
    as a single-element error list, not raised."""
    p = Path(path)
    try:
        with p.open("r", encoding="utf-8") as f:
            doc = yaml.safe_load(f)
    except (OSError, yaml.YAMLError) as e:
        return [f"{p}: failed to load YAML: {e}"]
    return validate_scenario(doc)


def _main(argv: Iterable[str]) -> int:
    paths = list(argv)
    if not paths:
        print("usage: validate.py <scenario.yaml> [...]", file=sys.stderr)
        return 2
    rc = 0
    for p in paths:
        errs = validate_file(p)
        if errs:
            rc = 1
            print(f"{p}: INVALID")
            for e in errs:
                print(f"  - {e}")
        else:
            print(f"{p}: ok")
    return rc


if __name__ == "__main__":
    sys.exit(_main(sys.argv[1:]))
