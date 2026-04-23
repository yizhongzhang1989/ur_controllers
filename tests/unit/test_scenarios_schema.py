"""Unit tests for the evaluation-scenario schema validator (schema v1).

Covers:
 * each example YAML under ``evaluation/scenarios/*.example.yaml`` validates clean,
 * representative malformed inputs each produce a specific, expected error.

Pure-Python (no ROS), runs under ``scripts/run_tests.sh`` stage 1.
"""

from __future__ import annotations

import copy
import importlib.util
import sys
from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
SCENARIO_DIR = REPO_ROOT / "evaluation" / "scenarios"


def _load_validator():
    spec = importlib.util.spec_from_file_location(
        "_scenario_validate", SCENARIO_DIR / "validate.py"
    )
    assert spec and spec.loader
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


validate = _load_validator()


# A complete, valid scenario used as the base for negative tests.
def _base_step() -> dict:
    return {
        "schema_version": 1,
        "name": "step-test",
        "description": "test",
        "scenario_type": "step",
        "duration_s": 5.0,
        "robots": ["ur5e", "ur15"],
        "target": {
            "space": "joint",
            "joints": list(validate.CANONICAL_JOINTS),
        },
        "command": {
            "per_joint_amplitude_rad": [0.0, 0.1, 0.0, 0.0, 0.0, 0.0],
            "step_time_s": 1.0,
        },
        "metrics": ["rmse", "settling_time", "overshoot", "control_effort"],
        "pass_criteria": {
            "max_rmse_rad": 0.05,
            "max_settling_time_s": 1.5,
            "max_overshoot_pct": 20.0,
            "max_control_effort_nm": 50.0,
        },
    }


# --- Examples on disk -------------------------------------------------------


@pytest.mark.parametrize("path", sorted(SCENARIO_DIR.glob("*.example.yaml")), ids=lambda p: p.name)
def test_example_yaml_validates(path: Path) -> None:
    errors = validate.validate_file(path)
    assert errors == [], f"{path}: {errors}"


def test_examples_cover_all_scenario_types() -> None:
    seen = set()
    for p in SCENARIO_DIR.glob("*.example.yaml"):
        with p.open("r", encoding="utf-8") as f:
            seen.add(yaml.safe_load(f)["scenario_type"])
    assert seen == validate.ALLOWED_SCENARIO_TYPES


# --- Top-level fields -------------------------------------------------------


def test_top_level_must_be_mapping() -> None:
    assert validate.validate_scenario([1, 2, 3]) == ["top-level: must be a mapping"]


def test_wrong_schema_version() -> None:
    d = _base_step()
    d["schema_version"] = 2
    errs = validate.validate_scenario(d)
    assert any("schema_version" in e for e in errs), errs


def test_unknown_top_level_key_rejected() -> None:
    d = _base_step()
    d["bogus"] = 1
    errs = validate.validate_scenario(d)
    assert any("unknown top-level key: 'bogus'" in e for e in errs), errs


def test_name_must_be_kebab_case() -> None:
    d = _base_step()
    d["name"] = "Step_Test"
    errs = validate.validate_scenario(d)
    assert any("kebab-case" in e for e in errs), errs


def test_duration_must_be_positive() -> None:
    d = _base_step()
    d["duration_s"] = 0
    errs = validate.validate_scenario(d)
    assert any(e.startswith("duration_s") for e in errs), errs


def test_robots_must_be_subset() -> None:
    d = _base_step()
    d["robots"] = ["ur5e", "ur10"]
    errs = validate.validate_scenario(d)
    assert any("ur10" in e for e in errs), errs


def test_robots_no_duplicates() -> None:
    d = _base_step()
    d["robots"] = ["ur5e", "ur5e"]
    errs = validate.validate_scenario(d)
    assert any("duplicate" in e for e in errs), errs


# --- target -----------------------------------------------------------------


def test_joint_target_requires_canonical_joint_list() -> None:
    d = _base_step()
    d["target"]["joints"] = list(validate.CANONICAL_JOINTS)[::-1]
    errs = validate.validate_scenario(d)
    assert any("canonical 6-joint list" in e for e in errs), errs


def test_cartesian_target_rejected_for_step() -> None:
    d = _base_step()
    d["target"] = {
        "space": "cartesian",
        "frame_id": "base",
        "end_effector": "tool0",
    }
    errs = validate.validate_scenario(d)
    assert any("scenario_type=step requires target.space==joint" in e for e in errs), errs


def test_cartesian_regulation_requires_unit_quaternion() -> None:
    d = _base_step()
    d["scenario_type"] = "regulation"
    d["target"] = {
        "space": "cartesian",
        "frame_id": "base",
        "end_effector": "tool0",
    }
    d["command"] = {
        "hold": {
            "position_xyz_m": [0.4, 0.1, 0.3],
            "orientation_xyzw": [0.0, 0.0, 0.0, 0.5],  # |q|=0.5, not 1
        }
    }
    d["pass_criteria"] = {"max_rmse_m": 0.01, "max_rmse_rad": 0.05}
    d["metrics"] = ["rmse"]
    errs = validate.validate_scenario(d)
    assert any("unit quaternion" in e for e in errs), errs


def test_cartesian_regulation_clean_validates() -> None:
    d = _base_step()
    d["scenario_type"] = "regulation"
    d["target"] = {
        "space": "cartesian",
        "frame_id": "base",
        "end_effector": "tool0",
    }
    d["command"] = {
        "hold": {
            "position_xyz_m": [0.4, 0.1, 0.3],
            "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
        }
    }
    d["pass_criteria"] = {"max_rmse_m": 0.01, "max_rmse_rad": 0.05}
    d["metrics"] = ["rmse"]
    assert validate.validate_scenario(d) == []


# --- command: step ----------------------------------------------------------


def test_step_scalar_amplitude_rejected() -> None:
    d = _base_step()
    d["command"]["amplitude_rad"] = 0.1
    errs = validate.validate_scenario(d)
    assert any("scalar amplitude" in e for e in errs), errs


def test_step_per_joint_amplitude_wrong_length() -> None:
    d = _base_step()
    d["command"]["per_joint_amplitude_rad"] = [0.0, 0.1, 0.0]
    errs = validate.validate_scenario(d)
    assert any("length 6" in e for e in errs), errs


def test_step_time_must_be_less_than_duration() -> None:
    d = _base_step()
    d["duration_s"] = 1.0
    d["command"]["step_time_s"] = 1.0
    errs = validate.validate_scenario(d)
    assert any("must be < duration_s" in e for e in errs), errs


# --- command: sine ----------------------------------------------------------


def test_sine_frequency_must_be_positive() -> None:
    d = _base_step()
    d["scenario_type"] = "sine"
    d["command"] = {
        "per_joint_amplitude_rad": [0.0] * 6,
        "frequency_hz": 0.0,
    }
    errs = validate.validate_scenario(d)
    assert any("frequency_hz" in e for e in errs), errs


# --- command: regulation ----------------------------------------------------


def test_regulation_hold_initial_validates() -> None:
    d = _base_step()
    d["scenario_type"] = "regulation"
    d["command"] = {"hold": "initial"}
    assert validate.validate_scenario(d) == []


def test_regulation_explicit_joint_pose_wrong_length() -> None:
    d = _base_step()
    d["scenario_type"] = "regulation"
    d["command"] = {"hold": [0.0, 0.0, 0.0]}
    errs = validate.validate_scenario(d)
    assert any("command.hold" in e and "length 6" in e for e in errs), errs


# --- command: random_waypoints ----------------------------------------------


def _base_random() -> dict:
    d = _base_step()
    d["scenario_type"] = "random_waypoints"
    d["command"] = {
        "num_waypoints": 4,
        "dwell_s": 2.0,
        "seed": 0,
        "bounds": "urdf",
    }
    return d


def test_random_dwell_times_count_must_fit_duration() -> None:
    d = _base_random()
    d["duration_s"] = 5.0
    d["command"]["num_waypoints"] = 4
    d["command"]["dwell_s"] = 2.0  # 4*2 = 8 > 5
    errs = validate.validate_scenario(d)
    assert any("must be <= duration_s" in e for e in errs), errs


def test_random_bounds_lower_must_be_below_upper() -> None:
    d = _base_random()
    d["command"]["bounds"] = {
        "lower": [0.0] * 6,
        "upper": [0.0] * 6,
    }
    errs = validate.validate_scenario(d)
    assert any("must be <" in e for e in errs), errs


def test_random_bounds_length_mismatch() -> None:
    d = _base_random()
    d["command"]["bounds"] = {
        "lower": [-1.0, -1.0],
        "upper": [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
    }
    errs = validate.validate_scenario(d)
    assert any("command.bounds.lower" in e and "length 6" in e for e in errs), errs


def test_random_seed_required_int() -> None:
    d = _base_random()
    d["command"].pop("seed")
    errs = validate.validate_scenario(d)
    assert any("command.seed" in e for e in errs), errs


# --- metrics & pass_criteria ------------------------------------------------


def test_unknown_metric_rejected() -> None:
    d = _base_step()
    d["metrics"] = ["rmse", "magic"]
    errs = validate.validate_scenario(d)
    assert any("magic" in e for e in errs), errs


def test_duplicate_metric_rejected() -> None:
    d = _base_step()
    d["metrics"] = ["rmse", "rmse"]
    errs = validate.validate_scenario(d)
    assert any("duplicate" in e for e in errs), errs


def test_pass_criterion_requires_corresponding_metric() -> None:
    d = _base_step()
    d["metrics"] = ["rmse"]
    # max_settling_time_s requires settling_time metric, which is absent.
    d["pass_criteria"] = {
        "max_rmse_rad": 0.05,
        "max_settling_time_s": 1.5,
    }
    errs = validate.validate_scenario(d)
    assert any("requires metric 'settling_time'" in e for e in errs), errs


def test_cartesian_pass_keys_rejected_for_joint() -> None:
    d = _base_step()
    d["pass_criteria"]["max_rmse_m"] = 0.01
    errs = validate.validate_scenario(d)
    assert any("max_rmse_m" in e and "not allowed" in e for e in errs), errs


def test_pass_criteria_thresholds_must_be_positive() -> None:
    d = _base_step()
    d["pass_criteria"]["max_rmse_rad"] = 0.0
    errs = validate.validate_scenario(d)
    assert any("must be > 0" in e for e in errs), errs


def test_overshoot_zero_allowed() -> None:
    d = _base_step()
    d["pass_criteria"]["max_overshoot_pct"] = 0.0
    assert validate.validate_scenario(d) == []


# --- File loader ------------------------------------------------------------


def test_validate_file_reports_yaml_error(tmp_path: Path) -> None:
    p = tmp_path / "bad.yaml"
    p.write_text(":\n: not yaml: [\n", encoding="utf-8")
    errs = validate.validate_file(p)
    assert len(errs) == 1 and "failed to load YAML" in errs[0]


def test_validate_file_reports_missing_file(tmp_path: Path) -> None:
    errs = validate.validate_file(tmp_path / "nope.yaml")
    assert len(errs) == 1 and "failed to load YAML" in errs[0]


# Sanity: the base fixture itself must be valid (guards future schema changes
# from silently invalidating every negative test).
def test_base_fixture_is_valid() -> None:
    assert validate.validate_scenario(copy.deepcopy(_base_step())) == []
