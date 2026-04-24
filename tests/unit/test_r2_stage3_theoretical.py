"""Unit tests for :mod:`tests.integration.r2_stage3_theoretical`.

Covers: export surface; stage-3 TCP-trajectory dispatch; argument-type
rejection; round-trip through
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
TH = _load("r2_stage3_theoretical", INT_DIR / "r2_stage3_theoretical.py")


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


def _arm_exp(**override_tcp):
    """Build a minimally-valid ArmExpectation whose tcp_tolerances are
    overridable. Other fields (joints, controllers, stage2,
    stage2_tcp) are pinned to placeholder values — stage-3 builder
    only reads ``tcp_tolerances``, so they can stay empty.
    """
    tcp = {
        "tcp_rmse_mm": 5.0,
        "tcp_peak_err_mm": 10.0,
        "tcp_orientation_peak_deg": 3.0,
        "tcp_steady_drift_mm_per_30s": 2.0,
    }
    tcp.update(override_tcp)
    return EL.ArmExpectation(
        arm="ur5e",
        draft=False,
        joints=(),
        controllers={},
        stage2=EL.Stage2Tolerances(
            completion_tol_rad=0.05, peak_tracking_err_rad=0.15, saturation_hold_ms=100.0
        ),
        stage2_tcp=EL.Stage2TcpTolerances(
            position_peak_err_mm=5.0, orientation_peak_err_deg=2.0
        ),
        tcp_tolerances=tcp,
    )


# ---------------------------------------------------------------------------
# Export surface
# ---------------------------------------------------------------------------


def test_export_surface():
    assert set(TH.__all__) == {
        "SUPPORTED_TOLERANCE_KEYS",
        "theoretical_for_stage3",
    }


def test_supported_tolerance_keys_pinned():
    # Order matters — the writer sorts before emit, but the builder's
    # contract is that the four keys (and only those four) are
    # present, so the test pins the tuple exactly.
    assert TH.SUPPORTED_TOLERANCE_KEYS == (
        "tcp_rmse_mm",
        "tcp_peak_err_mm",
        "tcp_orientation_peak_deg",
        "tcp_steady_drift_mm_per_30s",
    )


def test_supported_tolerance_keys_match_schema_test_set():
    # Keep in lock-step with the schema test so a new TCP tolerance
    # key trips both at once.
    schema_keys = {
        "tcp_rmse_mm",
        "tcp_peak_err_mm",
        "tcp_orientation_peak_deg",
        "tcp_steady_drift_mm_per_30s",
    }
    assert set(TH.SUPPORTED_TOLERANCE_KEYS) == schema_keys


# ---------------------------------------------------------------------------
# Happy path
# ---------------------------------------------------------------------------


def test_happy_path():
    got = TH.theoretical_for_stage3(_arm_exp())
    assert got == {
        "response_model": "tcp_trajectory_tracking",
        "tolerances": {
            "tcp_rmse_mm": 5.0,
            "tcp_peak_err_mm": 10.0,
            "tcp_orientation_peak_deg": 3.0,
            "tcp_steady_drift_mm_per_30s": 2.0,
        },
    }


def test_values_are_plain_floats():
    got = TH.theoretical_for_stage3(_arm_exp())
    for key, val in got["tolerances"].items():
        assert type(val) is float, (key, type(val).__name__)


def test_custom_values_propagate():
    got = TH.theoretical_for_stage3(
        _arm_exp(
            tcp_rmse_mm=3.5,
            tcp_peak_err_mm=7.5,
            tcp_orientation_peak_deg=1.5,
            tcp_steady_drift_mm_per_30s=1.0,
        )
    )
    assert got["tolerances"] == {
        "tcp_rmse_mm": 3.5,
        "tcp_peak_err_mm": 7.5,
        "tcp_orientation_peak_deg": 1.5,
        "tcp_steady_drift_mm_per_30s": 1.0,
    }


def test_returns_plain_dict_not_mappingproxy():
    got = TH.theoretical_for_stage3(_arm_exp())
    assert type(got) is dict
    assert type(got["tolerances"]) is dict


def test_int_tolerance_coerced_to_float():
    # The loader always produces floats, but if a future caller hands
    # in an ``int`` (e.g. via a test fixture) the builder must coerce
    # so ``write_r2_artefact`` sees only JSON-safe floats.
    got = TH.theoretical_for_stage3(_arm_exp(tcp_rmse_mm=5))  # int
    assert type(got["tolerances"]["tcp_rmse_mm"]) is float
    assert got["tolerances"]["tcp_rmse_mm"] == 5.0


# ---------------------------------------------------------------------------
# Argument-type rejection
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "bad",
    [
        None,
        42,
        "arm_exp",
        # Plain dict with all the right fields: still wrong, because
        # accepting it silently would mean the pre-bake no longer
        # guarantees the numbers came from an expectation YAML.
        {
            "tcp_rmse_mm": 5.0,
            "tcp_peak_err_mm": 10.0,
            "tcp_orientation_peak_deg": 3.0,
            "tcp_steady_drift_mm_per_30s": 2.0,
        },
    ],
)
def test_rejects_non_arm_expectation(bad):
    with pytest.raises(ValueError, match="ArmExpectation"):
        TH.theoretical_for_stage3(bad)


def test_rejects_stage2_tolerances():
    # Wrong dataclass: belongs to the stage-2 joint-space builder.
    st2 = EL.Stage2Tolerances(
        completion_tol_rad=0.05, peak_tracking_err_rad=0.15, saturation_hold_ms=100.0
    )
    with pytest.raises(ValueError, match="ArmExpectation"):
        TH.theoretical_for_stage3(st2)


def test_rejects_stage2_tcp_tolerances():
    st2c = EL.Stage2TcpTolerances(position_peak_err_mm=5.0, orientation_peak_err_deg=2.0)
    with pytest.raises(ValueError, match="ArmExpectation"):
        TH.theoretical_for_stage3(st2c)


def test_missing_tolerance_key_bubbles_loader_keyerror():
    # Drop a required key from the mapping — the loader's tcp_tol()
    # should bubble the "available: [...]" KeyError unchanged.
    arm = _arm_exp()
    bad_tcp = {k: v for k, v in arm.tcp_tolerances.items() if k != "tcp_rmse_mm"}
    arm_bad = EL.ArmExpectation(
        arm=arm.arm,
        draft=arm.draft,
        joints=arm.joints,
        controllers=arm.controllers,
        stage2=arm.stage2,
        stage2_tcp=arm.stage2_tcp,
        tcp_tolerances=bad_tcp,
    )
    with pytest.raises(KeyError, match="tcp_rmse_mm"):
        TH.theoretical_for_stage3(arm_bad)


# ---------------------------------------------------------------------------
# Integration with r2_result_to_artefact / r2_run_artefact
# ---------------------------------------------------------------------------


def test_round_trip_through_writer_no_notes(tmp_path):
    theoretical = TH.theoretical_for_stage3(_arm_exp())
    result = S3.TcpStage3Result(
        controller="cartesian_motion_controller",
        metrics={
            "position_rmse_mm": 1.2,
            "position_peak_err_mm": 3.4,
            "orientation_peak_err_deg": 0.7,
        },
        failures=(),
        notes=(),
    )
    artefact = BR.result_to_artefact(
        stage=3,
        arm="ur5e",
        controller="cartesian_motion_controller",
        payload="no_payload",
        theoretical=theoretical,
        result=result,
    )
    path = RA.write_r2_artefact(tmp_path, artefact)
    data = yaml.safe_load(path.read_text())
    assert data["theoretical"] == {
        "response_model": "tcp_trajectory_tracking",
        "tolerances": {
            "tcp_rmse_mm": 5.0,
            "tcp_peak_err_mm": 10.0,
            "tcp_orientation_peak_deg": 3.0,
            "tcp_steady_drift_mm_per_30s": 2.0,
        },
    }


def test_round_trip_through_writer_with_notes(tmp_path):
    theoretical = TH.theoretical_for_stage3(_arm_exp())
    result = S3.TcpStage3Result(
        controller="cartesian_motion_controller",
        metrics={
            "position_rmse_mm": 1.2,
            "position_peak_err_mm": 3.4,
            "orientation_peak_err_deg": 0.7,
        },
        failures=(),
        notes=("steady-drift check skipped: window below 30 s",),
    )
    artefact = BR.result_to_artefact(
        stage=3,
        arm="ur15",
        controller="cartesian_motion_controller",
        payload="large_payload",
        theoretical=theoretical,
        result=result,
    )
    path = RA.write_r2_artefact(tmp_path, artefact)
    data = yaml.safe_load(path.read_text())
    assert data["theoretical"]["response_model"] == "tcp_trajectory_tracking"
    assert set(data["theoretical"]["tolerances"]) == set(TH.SUPPORTED_TOLERANCE_KEYS)
    # Notes flow through via metadata['stage3_notes'] per
    # r2_result_to_artefact contract.
    assert data["metadata"]["stage3_notes"] == [
        "steady-drift check skipped: window below 30 s"
    ]


# ---------------------------------------------------------------------------
# Real expectation YAML integration
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("arm", ["ur5e", "ur15"])
def test_builds_from_real_expectation_yaml(arm):
    arm_exp = EL.load_arm(arm)
    got = TH.theoretical_for_stage3(arm_exp)
    assert got["response_model"] == "tcp_trajectory_tracking"
    for key in TH.SUPPORTED_TOLERANCE_KEYS:
        assert got["tolerances"][key] == arm_exp.tcp_tol(key)


@pytest.mark.parametrize("arm", ["ur5e", "ur15"])
def test_both_arms_share_tcp_values_via_builder(arm):
    # The schema test pins that ur5e and ur15 share TCP tolerance
    # values (ROADMAP R2: pass/fail thresholds are the same across
    # arms); the builder must propagate that unchanged (no hidden
    # per-arm branching).
    ur5e = EL.load_arm("ur5e")
    ur15 = EL.load_arm("ur15")
    assert TH.theoretical_for_stage3(ur5e) == TH.theoretical_for_stage3(ur15)
    # Reference the parametrised arm so the matrix is observably
    # exercised even though equality is cross-arm.
    arm_exp = EL.load_arm(arm)
    assert arm_exp.arm == arm
