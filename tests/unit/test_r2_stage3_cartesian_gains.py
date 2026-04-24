"""Unit tests for :mod:`tests.integration.r2_stage3_cartesian_gains`.

Covers: export surface; happy paths on the committed
``bringup/config/crisp_cartesian_impedance.{ur5e,ur15}.yaml``; return-
type invariants; argument-rejection matrix (unknown controller, unknown
arm, missing file, malformed YAML); value-validation matrix (non-finite,
negative, non-numeric, ``bool``); cross-arm equality invariant (both
committed YAMLs ship the same Cartesian task stiffnesses, per the file
comment in the UR15 YAML); default-config-dir resolution.
"""

from __future__ import annotations

import importlib.util
import math
import sys
from pathlib import Path
from types import ModuleType

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
INT_DIR = REPO_ROOT / "tests" / "integration"
CFG_DIR = REPO_ROOT / "bringup" / "config"


def _load(name: str, path: Path) -> ModuleType:
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)  # type: ignore[arg-type]
    sys.modules[name] = mod
    assert spec and spec.loader
    spec.loader.exec_module(mod)
    return mod


EL = _load("expectations_loader", INT_DIR / "expectations_loader.py")
GR = _load("r2_stage3_cartesian_gains", INT_DIR / "r2_stage3_cartesian_gains.py")


# ---------------------------------------------------------------------------
# Export surface
# ---------------------------------------------------------------------------


def test_exports_match_all():
    assert set(GR.__all__) == {
        "SUPPORTED_CONTROLLERS",
        "SUPPORTED_ARMS",
        "AXES",
        "DEFAULT_CONFIG_DIR",
        "resolve_cartesian_impedance_gains",
    }


def test_supported_controllers_pinned():
    # Narrow by design: position-mode `cartesian_motion` is deliberately
    # excluded (different semantics). A future companion resolver may
    # add it, but not under this symbol.
    assert GR.SUPPORTED_CONTROLLERS == ("crisp_cartesian_impedance",)


def test_supported_arms_matches_expectations_loader():
    assert GR.SUPPORTED_ARMS == EL.SUPPORTED_ARMS


def test_axes_pinned():
    assert GR.AXES == ("x", "y", "z")


def test_default_config_dir_points_at_repo_bringup():
    assert GR.DEFAULT_CONFIG_DIR == CFG_DIR
    assert GR.DEFAULT_CONFIG_DIR.is_dir()


# ---------------------------------------------------------------------------
# Happy path: committed YAMLs
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("arm", ["ur5e", "ur15"])
def test_committed_yaml_matches_yaml_values(arm):
    gains = GR.resolve_cartesian_impedance_gains("crisp_cartesian_impedance", arm)
    raw = yaml.safe_load((CFG_DIR / f"crisp_cartesian_impedance.{arm}.yaml").read_text())
    task = raw["cartesian_impedance_controller"]["ros__parameters"]["task"]
    for axis in ("x", "y", "z"):
        assert gains["translational"][axis] == pytest.approx(float(task[f"k_pos_{axis}"]))
        assert gains["rotational"][axis] == pytest.approx(float(task[f"k_rot_{axis}"]))


@pytest.mark.parametrize("arm", ["ur5e", "ur15"])
def test_committed_yaml_keys_have_fixed_order(arm):
    gains = GR.resolve_cartesian_impedance_gains("crisp_cartesian_impedance", arm)
    assert tuple(gains.keys()) == ("translational", "rotational")
    assert tuple(gains["translational"].keys()) == ("x", "y", "z")
    assert tuple(gains["rotational"].keys()) == ("x", "y", "z")


def test_ur5e_ur15_share_task_stiffness():
    # File comment in crisp_cartesian_impedance.ur15.yaml states the
    # gains match ur5e for first-light bring-up. Pin that invariant so
    # an accidental per-arm drift is caught.
    g5 = GR.resolve_cartesian_impedance_gains("crisp_cartesian_impedance", "ur5e")
    g15 = GR.resolve_cartesian_impedance_gains("crisp_cartesian_impedance", "ur15")
    assert g5 == g15


def test_committed_stiffness_values_positive():
    for arm in ("ur5e", "ur15"):
        gains = GR.resolve_cartesian_impedance_gains("crisp_cartesian_impedance", arm)
        for sub in gains.values():
            for v in sub.values():
                assert v > 0.0 and math.isfinite(v)


# ---------------------------------------------------------------------------
# Return-type invariants
# ---------------------------------------------------------------------------


def test_return_type_plain_dict_of_plain_floats():
    gains = GR.resolve_cartesian_impedance_gains("crisp_cartesian_impedance", "ur5e")
    assert type(gains) is dict  # noqa: E721
    assert type(gains["translational"]) is dict  # noqa: E721
    assert type(gains["rotational"]) is dict  # noqa: E721
    for sub in gains.values():
        for key, val in sub.items():
            assert type(key) is str  # noqa: E721
            assert type(val) is float  # noqa: E721
            assert math.isfinite(val)


def test_returned_dicts_are_independent():
    a = GR.resolve_cartesian_impedance_gains("crisp_cartesian_impedance", "ur5e")
    b = GR.resolve_cartesian_impedance_gains("crisp_cartesian_impedance", "ur5e")
    assert a is not b
    assert a["translational"] is not b["translational"]
    a["translational"]["x"] = -1.0
    assert b["translational"]["x"] != -1.0


# ---------------------------------------------------------------------------
# Argument rejection
# ---------------------------------------------------------------------------


def test_unknown_controller_raises():
    with pytest.raises(ValueError, match="unsupported controller"):
        GR.resolve_cartesian_impedance_gains("cartesian_motion", "ur5e")


def test_unknown_controller_joint_impedance_raises():
    # Guard against a caller mistakenly reaching for the joint-impedance
    # name on this resolver.
    with pytest.raises(ValueError, match="unsupported controller"):
        GR.resolve_cartesian_impedance_gains("crisp_joint_impedance", "ur5e")


def test_unknown_arm_raises():
    with pytest.raises(ValueError, match="unsupported arm"):
        GR.resolve_cartesian_impedance_gains("crisp_cartesian_impedance", "ur10e")


def test_missing_file_raises_file_not_found(tmp_path):
    with pytest.raises(FileNotFoundError, match="controller config not found"):
        GR.resolve_cartesian_impedance_gains(
            "crisp_cartesian_impedance", "ur5e", config_dir=tmp_path
        )


# ---------------------------------------------------------------------------
# YAML-structure validation (via temp configs)
# ---------------------------------------------------------------------------


def _write_cfg(tmp_path: Path, arm: str, body: dict) -> Path:
    path = tmp_path / f"crisp_cartesian_impedance.{arm}.yaml"
    path.write_text(yaml.safe_dump(body))
    return tmp_path


def _full_task() -> dict:
    return {
        "k_pos_x": 100.0,
        "k_pos_y": 200.0,
        "k_pos_z": 300.0,
        "k_rot_x": 10.0,
        "k_rot_y": 20.0,
        "k_rot_z": 30.0,
    }


def _full_cfg(task: dict | None = None) -> dict:
    return {
        "cartesian_impedance_controller": {
            "ros__parameters": {
                "task": task if task is not None else _full_task(),
            }
        }
    }


def test_happy_path_temp_cfg(tmp_path):
    cfg_dir = _write_cfg(tmp_path, "ur5e", _full_cfg())
    gains = GR.resolve_cartesian_impedance_gains(
        "crisp_cartesian_impedance", "ur5e", config_dir=cfg_dir
    )
    assert gains == {
        "translational": {"x": 100.0, "y": 200.0, "z": 300.0},
        "rotational": {"x": 10.0, "y": 20.0, "z": 30.0},
    }


def test_int_values_coerced_to_float(tmp_path):
    task = {k: int(v) for k, v in _full_task().items()}
    cfg_dir = _write_cfg(tmp_path, "ur5e", _full_cfg(task))
    gains = GR.resolve_cartesian_impedance_gains(
        "crisp_cartesian_impedance", "ur5e", config_dir=cfg_dir
    )
    for sub in gains.values():
        for v in sub.values():
            assert type(v) is float  # noqa: E721


def test_zero_stiffness_allowed(tmp_path):
    # A zero stiffness is a valid "no task stiffness on this axis"
    # request; the validator must allow it.
    task = _full_task()
    task["k_pos_y"] = 0.0
    cfg_dir = _write_cfg(tmp_path, "ur5e", _full_cfg(task))
    gains = GR.resolve_cartesian_impedance_gains(
        "crisp_cartesian_impedance", "ur5e", config_dir=cfg_dir
    )
    assert gains["translational"]["y"] == 0.0


def test_missing_top_key_raises(tmp_path):
    body = {"not_the_right_top": {}}
    cfg_dir = _write_cfg(tmp_path, "ur5e", body)
    with pytest.raises(KeyError, match="cartesian_impedance_controller"):
        GR.resolve_cartesian_impedance_gains(
            "crisp_cartesian_impedance", "ur5e", config_dir=cfg_dir
        )


def test_missing_ros_parameters_raises(tmp_path):
    body = {"cartesian_impedance_controller": {}}
    cfg_dir = _write_cfg(tmp_path, "ur5e", body)
    with pytest.raises(KeyError, match="ros__parameters"):
        GR.resolve_cartesian_impedance_gains(
            "crisp_cartesian_impedance", "ur5e", config_dir=cfg_dir
        )


def test_missing_task_raises(tmp_path):
    body = {"cartesian_impedance_controller": {"ros__parameters": {}}}
    cfg_dir = _write_cfg(tmp_path, "ur5e", body)
    with pytest.raises(KeyError, match=r"task"):
        GR.resolve_cartesian_impedance_gains(
            "crisp_cartesian_impedance", "ur5e", config_dir=cfg_dir
        )


@pytest.mark.parametrize(
    "missing_key",
    ["k_pos_x", "k_pos_y", "k_pos_z", "k_rot_x", "k_rot_y", "k_rot_z"],
)
def test_missing_axis_key_raises(tmp_path, missing_key):
    task = _full_task()
    del task[missing_key]
    cfg_dir = _write_cfg(tmp_path, "ur5e", _full_cfg(task))
    with pytest.raises(KeyError, match=missing_key):
        GR.resolve_cartesian_impedance_gains(
            "crisp_cartesian_impedance", "ur5e", config_dir=cfg_dir
        )


def test_non_mapping_top_level_raises(tmp_path):
    path = tmp_path / "crisp_cartesian_impedance.ur5e.yaml"
    path.write_text(yaml.safe_dump([1, 2, 3]))
    with pytest.raises(ValueError, match="top-level mapping"):
        GR.resolve_cartesian_impedance_gains(
            "crisp_cartesian_impedance", "ur5e", config_dir=tmp_path
        )


def test_non_mapping_task_raises(tmp_path):
    body = {"cartesian_impedance_controller": {"ros__parameters": {"task": [1, 2, 3]}}}
    cfg_dir = _write_cfg(tmp_path, "ur5e", body)
    with pytest.raises(ValueError, match=r"task must be a mapping"):
        GR.resolve_cartesian_impedance_gains(
            "crisp_cartesian_impedance", "ur5e", config_dir=cfg_dir
        )


def test_negative_stiffness_raises(tmp_path):
    task = _full_task()
    task["k_pos_x"] = -1.0
    cfg_dir = _write_cfg(tmp_path, "ur5e", _full_cfg(task))
    with pytest.raises(ValueError, match="non-negative"):
        GR.resolve_cartesian_impedance_gains(
            "crisp_cartesian_impedance", "ur5e", config_dir=cfg_dir
        )


def test_nan_stiffness_raises(tmp_path):
    task = _full_task()
    task["k_rot_z"] = float("nan")
    cfg_dir = _write_cfg(tmp_path, "ur5e", _full_cfg(task))
    with pytest.raises(ValueError, match="finite"):
        GR.resolve_cartesian_impedance_gains(
            "crisp_cartesian_impedance", "ur5e", config_dir=cfg_dir
        )


def test_inf_stiffness_raises(tmp_path):
    task = _full_task()
    task["k_pos_z"] = float("inf")
    cfg_dir = _write_cfg(tmp_path, "ur5e", _full_cfg(task))
    with pytest.raises(ValueError, match="finite"):
        GR.resolve_cartesian_impedance_gains(
            "crisp_cartesian_impedance", "ur5e", config_dir=cfg_dir
        )


def test_non_numeric_stiffness_raises(tmp_path):
    task = _full_task()
    task["k_pos_x"] = "high"
    cfg_dir = _write_cfg(tmp_path, "ur5e", _full_cfg(task))
    with pytest.raises(TypeError, match="real number"):
        GR.resolve_cartesian_impedance_gains(
            "crisp_cartesian_impedance", "ur5e", config_dir=cfg_dir
        )


def test_bool_stiffness_raises(tmp_path):
    # Guard against the `bool`-is-`int` gotcha: `True` would otherwise
    # silently coerce to `1.0` and look like a stiffness.
    task = _full_task()
    task["k_rot_y"] = True
    cfg_dir = _write_cfg(tmp_path, "ur5e", _full_cfg(task))
    with pytest.raises(TypeError, match="real number"):
        GR.resolve_cartesian_impedance_gains(
            "crisp_cartesian_impedance", "ur5e", config_dir=cfg_dir
        )
