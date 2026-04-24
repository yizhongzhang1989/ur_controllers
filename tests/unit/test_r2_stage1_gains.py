"""Unit tests for :mod:`tests.integration.r2_stage1_gains`.

Covers: export surface; happy paths for both controllers on both arms
using the committed ``bringup/config/*.yaml`` files; damping auto-fill
via temp YAMLs; argument-rejection matrix (unknown controller, unknown
arm, missing file, malformed YAML); value-validation matrix (length
mismatch, duplicates, non-finite, negative K, non-numeric); the
returned gains thread cleanly through
:func:`r2_stage1_theoretical.theoretical_for_stage1` on the
``second_order`` branch; default-config-dir resolution.
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
SA = _load("signal_analysis", INT_DIR / "signal_analysis.py")
RA = _load("r2_run_artefact", INT_DIR / "r2_run_artefact.py")
S1 = _load("r2_stage1_assertions", INT_DIR / "r2_stage1_assertions.py")
S2 = _load("r2_stage2_assertions", INT_DIR / "r2_stage2_assertions.py")
S2C = _load("r2_stage2_cartesian", INT_DIR / "r2_stage2_cartesian.py")
S3 = _load("r2_stage3_assertions", INT_DIR / "r2_stage3_assertions.py")
BR = _load("r2_result_to_artefact", INT_DIR / "r2_result_to_artefact.py")
TH = _load("r2_stage1_theoretical", INT_DIR / "r2_stage1_theoretical.py")
GR = _load("r2_stage1_gains", INT_DIR / "r2_stage1_gains.py")


# ---------------------------------------------------------------------------
# Export surface
# ---------------------------------------------------------------------------


def test_exports_match_all():
    assert set(GR.__all__) == {
        "SUPPORTED_CONTROLLERS",
        "SUPPORTED_ARMS",
        "CANONICAL_JOINTS",
        "DEFAULT_CONFIG_DIR",
        "resolve_joint_impedance_gains",
    }


def test_supported_controllers_pinned():
    assert GR.SUPPORTED_CONTROLLERS == (
        "simple_joint_impedance",
        "crisp_joint_impedance",
    )


def test_supported_arms_matches_expectations_loader():
    assert GR.SUPPORTED_ARMS == EL.SUPPORTED_ARMS


def test_canonical_joints_matches_expectations_loader():
    assert GR.CANONICAL_JOINTS == EL.CANONICAL_JOINTS


def test_default_config_dir_points_at_repo_bringup():
    assert GR.DEFAULT_CONFIG_DIR == CFG_DIR
    assert GR.DEFAULT_CONFIG_DIR.is_dir()


# ---------------------------------------------------------------------------
# Happy path: committed YAMLs
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("arm", ["ur5e", "ur15"])
def test_simple_joint_impedance_committed_yaml(arm):
    gains = GR.resolve_joint_impedance_gains("simple_joint_impedance", arm)
    raw = yaml.safe_load((CFG_DIR / f"simple_joint_impedance.{arm}.yaml").read_text())
    params = raw["simple_joint_impedance_controller"]["ros__parameters"]
    assert list(gains.keys()) == list(params["joints"])
    for i, joint in enumerate(params["joints"]):
        k_expected = float(params["k"][i])
        d_raw = float(params["d"][i])
        d_expected = 2.0 * math.sqrt(k_expected) if d_raw < 0 else d_raw
        assert gains[joint] == pytest.approx((k_expected, d_expected))


@pytest.mark.parametrize("arm", ["ur5e", "ur15"])
def test_crisp_joint_impedance_committed_yaml(arm):
    gains = GR.resolve_joint_impedance_gains("crisp_joint_impedance", arm)
    raw = yaml.safe_load((CFG_DIR / f"crisp_joint_impedance.{arm}.yaml").read_text())
    params = raw["joint_impedance_controller"]["ros__parameters"]
    joints = list(params["joints"])
    k_expected = float(params["nullspace"]["stiffness"])
    d_raw = float(params["nullspace"]["damping"])
    d_expected = 2.0 * math.sqrt(k_expected) if d_raw < 0 else d_raw
    assert list(gains.keys()) == joints
    for joint in joints:
        assert gains[joint] == pytest.approx((k_expected, d_expected))


def test_crisp_auto_damping_holds_on_committed_yaml():
    # The committed crisp YAMLs set `nullspace.damping: -1.0`, so the
    # resolver must substitute critical damping. Pins the sentinel
    # contract so a silent edit of the YAML that drops the `-1.0`
    # sentinel still makes sense (or fails this test loudly).
    for arm in ("ur5e", "ur15"):
        gains = GR.resolve_joint_impedance_gains("crisp_joint_impedance", arm)
        for k, d in gains.values():
            assert d == pytest.approx(2.0 * math.sqrt(k))


def test_simple_vs_crisp_differ_per_arm():
    # The two controllers use completely different gain strategies;
    # resolver output must reflect that (not the other's values by
    # accident of a cross-wired top-level key).
    simple = GR.resolve_joint_impedance_gains("simple_joint_impedance", "ur5e")
    crisp = GR.resolve_joint_impedance_gains("crisp_joint_impedance", "ur5e")
    # crisp broadcasts a scalar K to all joints; simple does not.
    crisp_k_set = {k for k, _ in crisp.values()}
    assert len(crisp_k_set) == 1
    simple_k_set = {k for k, _ in simple.values()}
    assert len(simple_k_set) > 1


def test_ur15_simple_heavier_than_ur5e_simple():
    # Sanity check: UR15 carries more inertia; its committed gains are
    # strictly larger at every joint. Pins the "arm dispatch actually
    # picks the right YAML" behaviour beyond string matching.
    g5 = GR.resolve_joint_impedance_gains("simple_joint_impedance", "ur5e")
    g15 = GR.resolve_joint_impedance_gains("simple_joint_impedance", "ur15")
    assert set(g5.keys()) == set(g15.keys())
    for joint in g5:
        assert g15[joint][0] > g5[joint][0]


# ---------------------------------------------------------------------------
# Return-type invariants
# ---------------------------------------------------------------------------


def test_return_type_plain_dict_of_plain_floats():
    gains = GR.resolve_joint_impedance_gains("simple_joint_impedance", "ur5e")
    assert type(gains) is dict  # noqa: E721 - subclass would be surprising here
    for key, val in gains.items():
        assert type(key) is str  # noqa: E721
        assert type(val) is tuple  # noqa: E721
        assert len(val) == 2
        k, d = val
        assert type(k) is float  # noqa: E721
        assert type(d) is float  # noqa: E721
        assert math.isfinite(k) and math.isfinite(d)


def test_returned_dicts_are_independent():
    a = GR.resolve_joint_impedance_gains("simple_joint_impedance", "ur5e")
    b = GR.resolve_joint_impedance_gains("simple_joint_impedance", "ur5e")
    assert a is not b
    a["shoulder_pan_joint"] = (0.0, 0.0)
    assert b["shoulder_pan_joint"] != (0.0, 0.0)


# ---------------------------------------------------------------------------
# Argument rejection
# ---------------------------------------------------------------------------


def test_unknown_controller_raises():
    with pytest.raises(ValueError, match="unsupported controller"):
        GR.resolve_joint_impedance_gains("crisp_cartesian_impedance", "ur5e")


def test_unknown_arm_raises():
    with pytest.raises(ValueError, match="unsupported arm"):
        GR.resolve_joint_impedance_gains("simple_joint_impedance", "ur10e")


def test_missing_file_raises_file_not_found(tmp_path):
    with pytest.raises(FileNotFoundError, match="controller config not found"):
        GR.resolve_joint_impedance_gains("simple_joint_impedance", "ur5e", config_dir=tmp_path)


# ---------------------------------------------------------------------------
# YAML-structure validation (via temp configs)
# ---------------------------------------------------------------------------


def _write_simple(tmp_path: Path, arm: str, body: dict) -> Path:
    p = tmp_path / f"simple_joint_impedance.{arm}.yaml"
    p.write_text(yaml.safe_dump(body))
    return p


def _write_crisp(tmp_path: Path, arm: str, body: dict) -> Path:
    p = tmp_path / f"crisp_joint_impedance.{arm}.yaml"
    p.write_text(yaml.safe_dump(body))
    return p


def _good_simple(arm="ur5e"):
    return {
        "simple_joint_impedance_controller": {
            "ros__parameters": {
                "joints": [
                    "shoulder_pan_joint",
                    "shoulder_lift_joint",
                    "elbow_joint",
                    "wrist_1_joint",
                    "wrist_2_joint",
                    "wrist_3_joint",
                ],
                "k": [100.0, 200.0, 50.0, 10.0, 5.0, 2.0],
                "d": [20.0, 30.0, 10.0, -1.0, 2.0, -1.0],
            }
        }
    }


def _good_crisp(arm="ur5e"):
    return {
        "joint_impedance_controller": {
            "ros__parameters": {
                "joints": [
                    "shoulder_pan_joint",
                    "shoulder_lift_joint",
                    "elbow_joint",
                    "wrist_1_joint",
                    "wrist_2_joint",
                    "wrist_3_joint",
                ],
                "nullspace": {"stiffness": 64.0, "damping": -1.0},
            }
        }
    }


def test_simple_autodamping_applied(tmp_path):
    _write_simple(tmp_path, "ur5e", _good_simple())
    gains = GR.resolve_joint_impedance_gains("simple_joint_impedance", "ur5e", config_dir=tmp_path)
    # Explicit entries passed through.
    assert gains["shoulder_pan_joint"] == (100.0, 20.0)
    # Negative entries auto-fill with 2*sqrt(K).
    assert gains["wrist_1_joint"] == pytest.approx((10.0, 2.0 * math.sqrt(10.0)))
    assert gains["wrist_3_joint"] == pytest.approx((2.0, 2.0 * math.sqrt(2.0)))


def test_crisp_explicit_damping(tmp_path):
    body = _good_crisp()
    body["joint_impedance_controller"]["ros__parameters"]["nullspace"]["damping"] = 17.5
    _write_crisp(tmp_path, "ur5e", body)
    gains = GR.resolve_joint_impedance_gains("crisp_joint_impedance", "ur5e", config_dir=tmp_path)
    for k, d in gains.values():
        assert k == 64.0
        assert d == 17.5


def test_crisp_autodamping_sentinel(tmp_path):
    _write_crisp(tmp_path, "ur5e", _good_crisp())
    gains = GR.resolve_joint_impedance_gains("crisp_joint_impedance", "ur5e", config_dir=tmp_path)
    for k, d in gains.values():
        assert d == pytest.approx(2.0 * math.sqrt(k))


def test_simple_missing_top_key_raises(tmp_path):
    p = tmp_path / "simple_joint_impedance.ur5e.yaml"
    p.write_text(yaml.safe_dump({"wrong_top": {}}))
    with pytest.raises(KeyError, match="simple_joint_impedance_controller"):
        GR.resolve_joint_impedance_gains("simple_joint_impedance", "ur5e", config_dir=tmp_path)


def test_simple_missing_ros_params_raises(tmp_path):
    _write_simple(tmp_path, "ur5e", {"simple_joint_impedance_controller": {}})
    with pytest.raises(KeyError, match="ros__parameters"):
        GR.resolve_joint_impedance_gains("simple_joint_impedance", "ur5e", config_dir=tmp_path)


def test_simple_missing_joints_raises(tmp_path):
    body = _good_simple()
    del body["simple_joint_impedance_controller"]["ros__parameters"]["joints"]
    _write_simple(tmp_path, "ur5e", body)
    with pytest.raises(KeyError, match=r"\.joints"):
        GR.resolve_joint_impedance_gains("simple_joint_impedance", "ur5e", config_dir=tmp_path)


def test_simple_missing_k_raises(tmp_path):
    body = _good_simple()
    del body["simple_joint_impedance_controller"]["ros__parameters"]["k"]
    _write_simple(tmp_path, "ur5e", body)
    with pytest.raises(KeyError, match=r"\.k"):
        GR.resolve_joint_impedance_gains("simple_joint_impedance", "ur5e", config_dir=tmp_path)


def test_simple_length_mismatch_raises(tmp_path):
    body = _good_simple()
    body["simple_joint_impedance_controller"]["ros__parameters"]["k"] = [
        1.0,
        2.0,
    ]
    _write_simple(tmp_path, "ur5e", body)
    with pytest.raises(ValueError, match="matching length"):
        GR.resolve_joint_impedance_gains("simple_joint_impedance", "ur5e", config_dir=tmp_path)


def test_simple_duplicate_joint_raises(tmp_path):
    body = _good_simple()
    body["simple_joint_impedance_controller"]["ros__parameters"]["joints"][1] = "shoulder_pan_joint"
    _write_simple(tmp_path, "ur5e", body)
    with pytest.raises(ValueError, match="duplicate joint"):
        GR.resolve_joint_impedance_gains("simple_joint_impedance", "ur5e", config_dir=tmp_path)


def test_simple_empty_joints_raises(tmp_path):
    body = _good_simple()
    body["simple_joint_impedance_controller"]["ros__parameters"]["joints"] = []
    body["simple_joint_impedance_controller"]["ros__parameters"]["k"] = []
    body["simple_joint_impedance_controller"]["ros__parameters"]["d"] = []
    _write_simple(tmp_path, "ur5e", body)
    with pytest.raises(ValueError, match="non-empty list"):
        GR.resolve_joint_impedance_gains("simple_joint_impedance", "ur5e", config_dir=tmp_path)


def test_simple_negative_k_raises(tmp_path):
    body = _good_simple()
    body["simple_joint_impedance_controller"]["ros__parameters"]["k"][0] = -1.0
    _write_simple(tmp_path, "ur5e", body)
    with pytest.raises(ValueError, match="non-negative"):
        GR.resolve_joint_impedance_gains("simple_joint_impedance", "ur5e", config_dir=tmp_path)


def test_simple_nan_k_raises(tmp_path):
    body = _good_simple()
    body["simple_joint_impedance_controller"]["ros__parameters"]["k"][0] = float("nan")
    _write_simple(tmp_path, "ur5e", body)
    with pytest.raises(ValueError, match="finite"):
        GR.resolve_joint_impedance_gains("simple_joint_impedance", "ur5e", config_dir=tmp_path)


def test_simple_non_numeric_k_raises(tmp_path):
    body = _good_simple()
    body["simple_joint_impedance_controller"]["ros__parameters"]["k"][0] = "100"
    _write_simple(tmp_path, "ur5e", body)
    with pytest.raises(TypeError, match="real number"):
        GR.resolve_joint_impedance_gains("simple_joint_impedance", "ur5e", config_dir=tmp_path)


def test_simple_bool_k_raises(tmp_path):
    # YAML `true` is distinct from a real number; guarding against
    # ``bool`` (subclass of int) stays explicit.
    body = _good_simple()
    body["simple_joint_impedance_controller"]["ros__parameters"]["k"][0] = True
    _write_simple(tmp_path, "ur5e", body)
    with pytest.raises(TypeError, match="real number"):
        GR.resolve_joint_impedance_gains("simple_joint_impedance", "ur5e", config_dir=tmp_path)


def test_simple_d_inf_raises(tmp_path):
    body = _good_simple()
    body["simple_joint_impedance_controller"]["ros__parameters"]["d"][0] = float("inf")
    _write_simple(tmp_path, "ur5e", body)
    with pytest.raises(ValueError, match="finite"):
        GR.resolve_joint_impedance_gains("simple_joint_impedance", "ur5e", config_dir=tmp_path)


def test_crisp_missing_nullspace_raises(tmp_path):
    body = _good_crisp()
    del body["joint_impedance_controller"]["ros__parameters"]["nullspace"]
    _write_crisp(tmp_path, "ur5e", body)
    with pytest.raises(KeyError, match="nullspace"):
        GR.resolve_joint_impedance_gains("crisp_joint_impedance", "ur5e", config_dir=tmp_path)


def test_crisp_missing_stiffness_raises(tmp_path):
    body = _good_crisp()
    del body["joint_impedance_controller"]["ros__parameters"]["nullspace"]["stiffness"]
    _write_crisp(tmp_path, "ur5e", body)
    with pytest.raises(KeyError, match="stiffness"):
        GR.resolve_joint_impedance_gains("crisp_joint_impedance", "ur5e", config_dir=tmp_path)


def test_crisp_missing_damping_raises(tmp_path):
    body = _good_crisp()
    del body["joint_impedance_controller"]["ros__parameters"]["nullspace"]["damping"]
    _write_crisp(tmp_path, "ur5e", body)
    with pytest.raises(KeyError, match="damping"):
        GR.resolve_joint_impedance_gains("crisp_joint_impedance", "ur5e", config_dir=tmp_path)


def test_crisp_nullspace_not_mapping_raises(tmp_path):
    body = _good_crisp()
    body["joint_impedance_controller"]["ros__parameters"]["nullspace"] = [1, 2]
    _write_crisp(tmp_path, "ur5e", body)
    with pytest.raises((ValueError, KeyError)):
        GR.resolve_joint_impedance_gains("crisp_joint_impedance", "ur5e", config_dir=tmp_path)


def test_crisp_negative_stiffness_raises(tmp_path):
    body = _good_crisp()
    body["joint_impedance_controller"]["ros__parameters"]["nullspace"]["stiffness"] = -0.1
    _write_crisp(tmp_path, "ur5e", body)
    with pytest.raises(ValueError, match="non-negative"):
        GR.resolve_joint_impedance_gains("crisp_joint_impedance", "ur5e", config_dir=tmp_path)


def test_non_mapping_top_level_raises(tmp_path):
    p = tmp_path / "simple_joint_impedance.ur5e.yaml"
    p.write_text("- not\n- a\n- mapping\n")
    with pytest.raises(ValueError, match="top-level mapping"):
        GR.resolve_joint_impedance_gains("simple_joint_impedance", "ur5e", config_dir=tmp_path)


# ---------------------------------------------------------------------------
# End-to-end thread through the stage-1 theoretical builder
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("arm", ["ur5e", "ur15"])
@pytest.mark.parametrize("controller", ["simple_joint_impedance", "crisp_joint_impedance"])
def test_gains_thread_through_theoretical_for_stage1(arm, controller):
    gains = GR.resolve_joint_impedance_gains(controller, arm)
    arm_exp = EL.load_arm(arm)
    controller_exp = arm_exp.controller(controller)
    assert controller_exp.response_model == "second_order"
    for joint_name, (k, d) in gains.items():
        joint_exp = arm_exp.joint(joint_name)
        block = TH.theoretical_for_stage1(
            controller_exp,
            joint_exp=joint_exp,
            stiffness_k=k,
            damping_d=d,
        )
        assert block["response_model"] == "second_order"
        assert block["response"]["stiffness_k"] == pytest.approx(k)
        assert block["response"]["damping_d"] == pytest.approx(d)
        assert block["response"]["joint"] == joint_name
        # omega_n = sqrt(K / J_eff)
        J = joint_exp.effective_inertia_kg_m2
        assert block["response"]["omega_n_rad_s"] == pytest.approx(math.sqrt(k / J))
