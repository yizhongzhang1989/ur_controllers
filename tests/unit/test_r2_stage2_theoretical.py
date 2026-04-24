"""Unit tests for :mod:`tests.integration.r2_stage2_theoretical`.

Covers: export surface; dispatch for both stage-2 paths (joint-space
and cartesian); argument-type rejection; round-trip through
:func:`r2_result_to_artefact.result_to_artefact` and
:func:`r2_run_artefact.write_r2_artefact` so the builder's output
shape is provably compatible with the writer; real-YAML integration
matrix over ``{ur5e, ur15}``.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from types import ModuleType

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
INT_DIR = REPO_ROOT / "tests" / "integration"


def _load(name: str, path: Path) -> ModuleType:
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)  # type: ignore[arg-type]
    sys.modules[name] = mod
    assert spec and spec.loader
    spec.loader.exec_module(mod)
    return mod


# Load in dependency order so sibling modules share module keys.
EL = _load("expectations_loader", INT_DIR / "expectations_loader.py")
SA = _load("signal_analysis", INT_DIR / "signal_analysis.py")
RA = _load("r2_run_artefact", INT_DIR / "r2_run_artefact.py")
S1 = _load("r2_stage1_assertions", INT_DIR / "r2_stage1_assertions.py")
S2 = _load("r2_stage2_assertions", INT_DIR / "r2_stage2_assertions.py")
S2C = _load("r2_stage2_cartesian", INT_DIR / "r2_stage2_cartesian.py")
S3 = _load("r2_stage3_assertions", INT_DIR / "r2_stage3_assertions.py")
BR = _load("r2_result_to_artefact", INT_DIR / "r2_result_to_artefact.py")
TH = _load("r2_stage2_theoretical", INT_DIR / "r2_stage2_theoretical.py")


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


def _stage2(**kw):
    defaults = dict(
        completion_tol_rad=0.05,
        peak_tracking_err_rad=0.15,
        saturation_hold_ms=100.0,
    )
    defaults.update(kw)
    return EL.Stage2Tolerances(**defaults)


def _stage2_tcp(**kw):
    defaults = dict(
        position_peak_err_mm=5.0,
        orientation_peak_err_deg=2.0,
    )
    defaults.update(kw)
    return EL.Stage2TcpTolerances(**defaults)


# ---------------------------------------------------------------------------
# Export surface
# ---------------------------------------------------------------------------


def test_export_surface():
    assert set(TH.__all__) == {
        "SUPPORTED_MODES",
        "theoretical_for_stage2_joint_space",
        "theoretical_for_stage2_cartesian",
    }


def test_supported_modes_pinned():
    assert TH.SUPPORTED_MODES == ("joint_space", "cartesian")


# ---------------------------------------------------------------------------
# Joint-space happy path
# ---------------------------------------------------------------------------


def test_joint_space_happy_path():
    got = TH.theoretical_for_stage2_joint_space(_stage2())
    assert got == {
        "response_model": "kinematic_consistency_joint_space",
        "tolerances": {
            "completion_tol_rad": 0.05,
            "peak_tracking_err_rad": 0.15,
            "saturation_hold_ms": 100.0,
        },
    }


def test_joint_space_values_are_plain_floats():
    got = TH.theoretical_for_stage2_joint_space(_stage2())
    for key, val in got["tolerances"].items():
        assert type(val) is float, (key, type(val).__name__)


def test_joint_space_custom_values_propagate():
    got = TH.theoretical_for_stage2_joint_space(
        _stage2(completion_tol_rad=0.01, peak_tracking_err_rad=0.2, saturation_hold_ms=50.0)
    )
    assert got["tolerances"] == {
        "completion_tol_rad": 0.01,
        "peak_tracking_err_rad": 0.2,
        "saturation_hold_ms": 50.0,
    }


def test_joint_space_returns_plain_dict_not_mappingproxy():
    # r2_result_to_artefact treats its own 'theoretical' as immutable but
    # the builder itself must return a plain dict so callers can extend.
    got = TH.theoretical_for_stage2_joint_space(_stage2())
    assert type(got) is dict
    assert type(got["tolerances"]) is dict


# ---------------------------------------------------------------------------
# Cartesian happy path
# ---------------------------------------------------------------------------


def test_cartesian_happy_path():
    got = TH.theoretical_for_stage2_cartesian(_stage2_tcp())
    assert got == {
        "response_model": "kinematic_consistency_tcp",
        "tolerances": {
            "position_peak_err_mm": 5.0,
            "orientation_peak_err_deg": 2.0,
        },
    }


def test_cartesian_values_are_plain_floats():
    got = TH.theoretical_for_stage2_cartesian(_stage2_tcp())
    for key, val in got["tolerances"].items():
        assert type(val) is float, (key, type(val).__name__)


def test_cartesian_custom_values_propagate():
    got = TH.theoretical_for_stage2_cartesian(
        _stage2_tcp(position_peak_err_mm=3.5, orientation_peak_err_deg=1.0)
    )
    assert got["tolerances"] == {
        "position_peak_err_mm": 3.5,
        "orientation_peak_err_deg": 1.0,
    }


def test_cartesian_returns_plain_dict_not_mappingproxy():
    got = TH.theoretical_for_stage2_cartesian(_stage2_tcp())
    assert type(got) is dict
    assert type(got["tolerances"]) is dict


# ---------------------------------------------------------------------------
# Argument-type rejection
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "bad",
    [
        None,
        42,
        "stage2",
        # Plain dict with all the right fields: still wrong, because
        # accepting it silently would mean the pre-bake no longer
        # guarantees the numbers came from the expectation YAML.
        {"completion_tol_rad": 0.05, "peak_tracking_err_rad": 0.15, "saturation_hold_ms": 100.0},
    ],
)
def test_joint_space_rejects_non_stage2_tolerances(bad):
    with pytest.raises(ValueError, match="Stage2Tolerances"):
        TH.theoretical_for_stage2_joint_space(bad)


def test_joint_space_rejects_stage2_tcp_tolerances():
    # Wrong dataclass — must fail, else it would silently emit the
    # wrong response_model / tolerance keys.
    with pytest.raises(ValueError, match="Stage2Tolerances"):
        TH.theoretical_for_stage2_joint_space(_stage2_tcp())


@pytest.mark.parametrize(
    "bad",
    [
        None,
        42,
        "stage2_tcp",
        {"position_peak_err_mm": 5.0, "orientation_peak_err_deg": 2.0},
    ],
)
def test_cartesian_rejects_non_stage2_tcp_tolerances(bad):
    with pytest.raises(ValueError, match="Stage2TcpTolerances"):
        TH.theoretical_for_stage2_cartesian(bad)


def test_cartesian_rejects_stage2_tolerances():
    with pytest.raises(ValueError, match="Stage2TcpTolerances"):
        TH.theoretical_for_stage2_cartesian(_stage2())


# ---------------------------------------------------------------------------
# Integration with r2_result_to_artefact / r2_run_artefact
# ---------------------------------------------------------------------------


def test_round_trip_joint_space_through_writer(tmp_path):
    theoretical = TH.theoretical_for_stage2_joint_space(_stage2())
    j = S2.JointStage2Metrics(
        joint="shoulder_pan_joint",
        metrics={"completion_err_rad": 0.01, "peak_tracking_err_rad": 0.03},
        failures=(),
    )
    result = S2.Stage2Result(
        controller="joint_trajectory_controller",
        per_joint=(j,),
    )
    artefact = BR.result_to_artefact(
        stage=2,
        arm="ur5e",
        controller="joint_trajectory_controller",
        payload="no_payload",
        theoretical=theoretical,
        result=result,
    )
    path = RA.write_r2_artefact(tmp_path, artefact)
    data = yaml.safe_load(path.read_text())
    assert data["theoretical"] == {
        "response_model": "kinematic_consistency_joint_space",
        "tolerances": {
            "completion_tol_rad": 0.05,
            "peak_tracking_err_rad": 0.15,
            "saturation_hold_ms": 100.0,
        },
    }


def test_round_trip_cartesian_through_writer(tmp_path):
    theoretical = TH.theoretical_for_stage2_cartesian(_stage2_tcp())
    result = S2C.Stage2CartesianResult(
        controller="cartesian_motion_controller",
        metrics={"position_peak_err_mm": 2.0, "orientation_peak_err_deg": 0.5},
        failures=(),
    )
    artefact = BR.result_to_artefact(
        stage=2,
        arm="ur15",
        controller="cartesian_motion_controller",
        payload="large_payload",
        theoretical=theoretical,
        result=result,
    )
    path = RA.write_r2_artefact(tmp_path, artefact)
    data = yaml.safe_load(path.read_text())
    assert data["theoretical"] == {
        "response_model": "kinematic_consistency_tcp",
        "tolerances": {
            "position_peak_err_mm": 5.0,
            "orientation_peak_err_deg": 2.0,
        },
    }


# ---------------------------------------------------------------------------
# Real expectation YAML integration
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("arm", ["ur5e", "ur15"])
def test_builds_joint_space_from_real_expectation_yaml(arm):
    arm_exp = EL.load_arm(arm)
    got = TH.theoretical_for_stage2_joint_space(arm_exp.stage2)
    assert got["response_model"] == "kinematic_consistency_joint_space"
    assert got["tolerances"]["completion_tol_rad"] == arm_exp.stage2.completion_tol_rad
    assert got["tolerances"]["peak_tracking_err_rad"] == arm_exp.stage2.peak_tracking_err_rad
    assert got["tolerances"]["saturation_hold_ms"] == arm_exp.stage2.saturation_hold_ms


@pytest.mark.parametrize("arm", ["ur5e", "ur15"])
def test_builds_cartesian_from_real_expectation_yaml(arm):
    arm_exp = EL.load_arm(arm)
    got = TH.theoretical_for_stage2_cartesian(arm_exp.stage2_tcp)
    assert got["response_model"] == "kinematic_consistency_tcp"
    assert got["tolerances"]["position_peak_err_mm"] == arm_exp.stage2_tcp.position_peak_err_mm
    assert (
        got["tolerances"]["orientation_peak_err_deg"] == arm_exp.stage2_tcp.orientation_peak_err_deg
    )


@pytest.mark.parametrize("arm", ["ur5e", "ur15"])
def test_both_arms_share_stage2_values_via_builder(arm):
    # The schema test pins block equality between ur5e and ur15 for
    # stage2 / stage2_tcp — the builder must propagate that unchanged
    # (no hidden per-arm branching).
    ur5e = EL.load_arm("ur5e")
    ur15 = EL.load_arm("ur15")
    assert TH.theoretical_for_stage2_joint_space(
        ur5e.stage2
    ) == TH.theoretical_for_stage2_joint_space(ur15.stage2)
    assert TH.theoretical_for_stage2_cartesian(
        ur5e.stage2_tcp
    ) == TH.theoretical_for_stage2_cartesian(ur15.stage2_tcp)
    # Reference the parametrised arm so the matrix is observably
    # exercised even though equality is cross-arm.
    arm_exp = EL.load_arm(arm)
    assert arm_exp.arm == arm
