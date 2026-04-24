"""Unit tests for :mod:`tests.integration.r2_result_to_artefact`.

Covers: export surface; happy paths for Stage1 / Stage2 (joint-space)
/ Stage2 (cartesian) / Stage3; stage↔result-type mismatches;
controller-echo mismatches; reserved-metadata-key clashes; stage-2
joint-space per-joint flattening; stage-2 joints metadata ordering;
stage-3 notes propagation (present + absent); theoretical / measured
/ reasons carry-through; pass/fail for both failing and passing
results; argument-type rejection matrix (stage / arm / controller /
payload / theoretical / metadata); round-trip through
:func:`write_r2_artefact`.
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


# Siblings share module keys, so load in dependency order.
EL = _load("expectations_loader", INT_DIR / "expectations_loader.py")
SA = _load("signal_analysis", INT_DIR / "signal_analysis.py")
RA = _load("r2_run_artefact", INT_DIR / "r2_run_artefact.py")
S1 = _load("r2_stage1_assertions", INT_DIR / "r2_stage1_assertions.py")
S2 = _load("r2_stage2_assertions", INT_DIR / "r2_stage2_assertions.py")
S2C = _load("r2_stage2_cartesian", INT_DIR / "r2_stage2_cartesian.py")
S3 = _load("r2_stage3_assertions", INT_DIR / "r2_stage3_assertions.py")
BR = _load("r2_result_to_artefact", INT_DIR / "r2_result_to_artefact.py")


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


def _s1(**kw):
    kw.setdefault("controller", "simple_jimp")
    kw.setdefault("joint", "shoulder_pan_joint")
    return S1.Stage1Result(**kw)


def _s2_joint(**kw):
    kw.setdefault("joint", "shoulder_pan_joint")
    return S2.JointStage2Metrics(**kw)


def _s2(**kw):
    kw.setdefault("controller", "jtc")
    kw.setdefault("per_joint", ())
    return S2.Stage2Result(**kw)


def _s2c(**kw):
    kw.setdefault("controller", "cartesian_motion")
    return S2C.Stage2CartesianResult(**kw)


def _s3(**kw):
    kw.setdefault("controller", "cartesian_motion")
    return S3.TcpStage3Result(**kw)


# ---------------------------------------------------------------------------
# Export surface
# ---------------------------------------------------------------------------


def test_exports():
    assert BR.__all__ == ("result_to_artefact",)
    assert callable(BR.result_to_artefact)


# ---------------------------------------------------------------------------
# Stage 1
# ---------------------------------------------------------------------------


def test_stage1_happy_path_pass():
    r = _s1(metrics={"steady_state_err_rad": 1e-4}, failures=())
    art = BR.result_to_artefact(
        stage=1,
        arm="ur5e",
        controller="simple_jimp",
        payload="no_payload",
        theoretical={"tol_rad": 1e-3},
        result=r,
    )
    assert isinstance(art, RA.R2Artefact)
    assert art.stage == 1
    assert art.arm == "ur5e"
    assert art.controller == "simple_jimp"
    assert art.payload == "no_payload"
    assert art.passed is True
    assert art.reasons == ()
    assert art.measured == {"steady_state_err_rad": 1e-4}
    assert art.theoretical == {"tol_rad": 1e-3}
    assert art.metadata == {"joint": "shoulder_pan_joint"}


def test_stage1_happy_path_fail_carries_reasons():
    r = _s1(
        metrics={"steady_state_err_rad": 1.0},
        failures=("steady-state error 1.000 rad > tol 0.001 rad",),
    )
    art = BR.result_to_artefact(
        stage=1,
        arm="ur15",
        controller="simple_jimp",
        payload="no_payload",
        theoretical={},
        result=r,
    )
    assert art.passed is False
    assert art.reasons == ("steady-state error 1.000 rad > tol 0.001 rad",)


def test_stage1_metadata_carries_through_and_adds_joint():
    r = _s1()
    art = BR.result_to_artefact(
        stage=1,
        arm="ur5e",
        controller="simple_jimp",
        payload="no_payload",
        theoretical={},
        result=r,
        metadata={"run_id": "abc"},
    )
    assert art.metadata == {"run_id": "abc", "joint": "shoulder_pan_joint"}


def test_stage1_requires_stage1_result():
    r = _s2c(metrics={}, failures=())
    with pytest.raises(ValueError, match="stage=1 requires a Stage1Result"):
        BR.result_to_artefact(
            stage=1,
            arm="ur5e",
            controller="cartesian_motion",
            payload="no_payload",
            theoretical={},
            result=r,
        )


def test_stage1_controller_echo_mismatch():
    r = _s1(controller="crisp")
    with pytest.raises(ValueError, match="controller echo mismatch"):
        BR.result_to_artefact(
            stage=1,
            arm="ur5e",
            controller="simple_jimp",
            payload="no_payload",
            theoretical={},
            result=r,
        )


def test_stage1_metadata_joint_clash():
    r = _s1()
    with pytest.raises(ValueError, match="'joint'.*reserved"):
        BR.result_to_artefact(
            stage=1,
            arm="ur5e",
            controller="simple_jimp",
            payload="no_payload",
            theoretical={},
            result=r,
            metadata={"joint": "other"},
        )


# ---------------------------------------------------------------------------
# Stage 2 (joint-space)
# ---------------------------------------------------------------------------


def test_stage2_joint_space_flattens_per_joint_metrics():
    per = (
        _s2_joint(joint="j1", metrics={"final_err_rad": 0.01, "peak_tracking_err_rad": 0.02}),
        _s2_joint(joint="j2", metrics={"final_err_rad": 0.03, "peak_tracking_err_rad": 0.04}),
    )
    r = _s2(per_joint=per)
    art = BR.result_to_artefact(
        stage=2,
        arm="ur5e",
        controller="jtc",
        payload="no_payload",
        theoretical={"tol": 0.05},
        result=r,
    )
    assert art.measured == {
        "j1.final_err_rad": 0.01,
        "j1.peak_tracking_err_rad": 0.02,
        "j2.final_err_rad": 0.03,
        "j2.peak_tracking_err_rad": 0.04,
    }
    assert art.metadata == {"joints": ("j1", "j2")}
    assert art.passed is True
    assert art.reasons == ()


def test_stage2_joint_space_reasons_from_flattened_failures():
    per = (
        _s2_joint(joint="j1", metrics={}, failures=("final 1.0 > tol 0.1",)),
        _s2_joint(joint="j2", metrics={}, failures=()),
    )
    r = _s2(per_joint=per)
    art = BR.result_to_artefact(
        stage=2,
        arm="ur5e",
        controller="jtc",
        payload="no_payload",
        theoretical={},
        result=r,
    )
    assert art.passed is False
    assert art.reasons == ("j1: final 1.0 > tol 0.1",)


def test_stage2_joint_space_empty_per_joint_passes():
    r = _s2(per_joint=())
    art = BR.result_to_artefact(
        stage=2,
        arm="ur5e",
        controller="jtc",
        payload="no_payload",
        theoretical={},
        result=r,
    )
    assert art.measured == {}
    assert art.metadata == {"joints": ()}
    assert art.passed is True


def test_stage2_joint_space_metadata_joints_clash():
    r = _s2(per_joint=())
    with pytest.raises(ValueError, match="'joints'.*reserved"):
        BR.result_to_artefact(
            stage=2,
            arm="ur5e",
            controller="jtc",
            payload="no_payload",
            theoretical={},
            result=r,
            metadata={"joints": ("x",)},
        )


def test_stage2_joint_space_controller_echo_mismatch():
    r = _s2(controller="other", per_joint=())
    with pytest.raises(ValueError, match="controller echo mismatch"):
        BR.result_to_artefact(
            stage=2,
            arm="ur5e",
            controller="jtc",
            payload="no_payload",
            theoretical={},
            result=r,
        )


# ---------------------------------------------------------------------------
# Stage 2 (cartesian)
# ---------------------------------------------------------------------------


def test_stage2_cartesian_happy_path():
    r = _s2c(
        metrics={"position_peak_err_mm": 3.2, "orientation_peak_err_deg": 1.4},
        failures=(),
    )
    art = BR.result_to_artefact(
        stage=2,
        arm="ur5e",
        controller="cartesian_motion",
        payload="no_payload",
        theoretical={"pos_mm": 5.0, "orient_deg": 2.0},
        result=r,
    )
    assert art.measured == {"position_peak_err_mm": 3.2, "orientation_peak_err_deg": 1.4}
    # No reserved keys added for the cartesian branch.
    assert art.metadata == {}
    assert art.passed is True


def test_stage2_cartesian_controller_echo_mismatch():
    r = _s2c(controller="other")
    with pytest.raises(ValueError, match="controller echo mismatch"):
        BR.result_to_artefact(
            stage=2,
            arm="ur5e",
            controller="cartesian_motion",
            payload="no_payload",
            theoretical={},
            result=r,
        )


def test_stage2_requires_stage2_result_type():
    r = _s3()
    with pytest.raises(
        ValueError, match="stage=2 requires a Stage2Result or Stage2CartesianResult"
    ):
        BR.result_to_artefact(
            stage=2,
            arm="ur5e",
            controller="cartesian_motion",
            payload="no_payload",
            theoretical={},
            result=r,
        )


def test_stage2_cartesian_metadata_carried_through():
    r = _s2c()
    art = BR.result_to_artefact(
        stage=2,
        arm="ur5e",
        controller="cartesian_motion",
        payload="no_payload",
        theoretical={},
        result=r,
        metadata={"run_id": "abc"},
    )
    assert art.metadata == {"run_id": "abc"}


# ---------------------------------------------------------------------------
# Stage 3
# ---------------------------------------------------------------------------


def test_stage3_happy_path_no_notes():
    r = _s3(metrics={"position_rmse_mm": 2.1}, failures=())
    art = BR.result_to_artefact(
        stage=3,
        arm="ur5e",
        controller="cartesian_motion",
        payload="no_payload",
        theoretical={"rmse_mm": 5.0},
        result=r,
    )
    assert art.passed is True
    assert art.measured == {"position_rmse_mm": 2.1}
    # Notes absent → no stage3_notes key.
    assert art.metadata == {}


def test_stage3_notes_propagate_to_metadata():
    r = _s3(
        metrics={"position_rmse_mm": 2.1},
        failures=(),
        notes=("steady-drift window too short; skipped",),
    )
    art = BR.result_to_artefact(
        stage=3,
        arm="ur5e",
        controller="cartesian_motion",
        payload="no_payload",
        theoretical={},
        result=r,
    )
    assert art.metadata == {
        "stage3_notes": ("steady-drift window too short; skipped",),
    }


def test_stage3_notes_clash_with_caller_metadata():
    r = _s3(notes=("n1",))
    with pytest.raises(ValueError, match="'stage3_notes'.*reserved"):
        BR.result_to_artefact(
            stage=3,
            arm="ur5e",
            controller="cartesian_motion",
            payload="no_payload",
            theoretical={},
            result=r,
            metadata={"stage3_notes": ("caller",)},
        )


def test_stage3_requires_stage3_result_type():
    r = _s1()
    with pytest.raises(ValueError, match="stage=3 requires a TcpStage3Result"):
        BR.result_to_artefact(
            stage=3,
            arm="ur5e",
            controller="cartesian_motion",
            payload="no_payload",
            theoretical={},
            result=r,
        )


def test_stage3_controller_echo_mismatch():
    r = _s3(controller="other")
    with pytest.raises(ValueError, match="controller echo mismatch"):
        BR.result_to_artefact(
            stage=3,
            arm="ur5e",
            controller="cartesian_motion",
            payload="no_payload",
            theoretical={},
            result=r,
        )


# ---------------------------------------------------------------------------
# Argument rejection matrix
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "bad_stage",
    [0, 4, 99, -1, "1", 1.0, True, None],
)
def test_rejects_bad_stage(bad_stage):
    r = _s1()
    with pytest.raises(ValueError):
        BR.result_to_artefact(
            stage=bad_stage,
            arm="ur5e",
            controller="simple_jimp",
            payload="no_payload",
            theoretical={},
            result=r,
        )


@pytest.mark.parametrize("field", ["arm", "controller", "payload"])
@pytest.mark.parametrize("bad", [None, 1, [], "", (1,)])
def test_rejects_bad_string_field(field, bad):
    r = _s1()
    kwargs = dict(
        stage=1,
        arm="ur5e",
        controller="simple_jimp",
        payload="no_payload",
        theoretical={},
        result=r,
    )
    kwargs[field] = bad
    with pytest.raises(ValueError):
        BR.result_to_artefact(**kwargs)


@pytest.mark.parametrize("bad_theoretical", [None, 1, "x", (1,), [1]])
def test_rejects_bad_theoretical(bad_theoretical):
    r = _s1()
    with pytest.raises(ValueError, match="theoretical must be a Mapping"):
        BR.result_to_artefact(
            stage=1,
            arm="ur5e",
            controller="simple_jimp",
            payload="no_payload",
            theoretical=bad_theoretical,
            result=r,
        )


@pytest.mark.parametrize("bad_metadata", [1, "x", (1,), [1]])
def test_rejects_bad_metadata(bad_metadata):
    r = _s1()
    with pytest.raises(ValueError, match="metadata must be a Mapping"):
        BR.result_to_artefact(
            stage=1,
            arm="ur5e",
            controller="simple_jimp",
            payload="no_payload",
            theoretical={},
            result=r,
            metadata=bad_metadata,
        )


def test_accepts_none_metadata():
    r = _s1()
    art = BR.result_to_artefact(
        stage=1,
        arm="ur5e",
        controller="simple_jimp",
        payload="no_payload",
        theoretical={},
        result=r,
        metadata=None,
    )
    assert art.metadata == {"joint": "shoulder_pan_joint"}


# ---------------------------------------------------------------------------
# End-to-end round-trip via the writer
# ---------------------------------------------------------------------------


def test_artefact_round_trips_through_writer(tmp_path):
    r = _s1(metrics={"steady_state_err_rad": 1e-4}, failures=())
    art = BR.result_to_artefact(
        stage=1,
        arm="ur5e",
        controller="simple_jimp",
        payload="no_payload",
        theoretical={"tol_rad": 1e-3},
        result=r,
    )
    path = RA.write_r2_artefact(tmp_path, art)
    assert path.exists()
    data = yaml.safe_load(path.read_text())
    assert data["stage"] == 1
    assert data["passed"] is True
    assert data["measured"] == {"steady_state_err_rad": 1e-4}
    assert data["theoretical"] == {"tol_rad": 1e-3}
    assert data["metadata"]["joint"] == "shoulder_pan_joint"


def test_stage2_round_trips_through_writer(tmp_path):
    per = (
        _s2_joint(joint="j1", metrics={"final_err_rad": 0.01}),
        _s2_joint(joint="j2", metrics={"final_err_rad": 0.02}),
    )
    r = _s2(per_joint=per)
    art = BR.result_to_artefact(
        stage=2,
        arm="ur5e",
        controller="jtc",
        payload="no_payload",
        theoretical={"tol": 0.05},
        result=r,
    )
    path = RA.write_r2_artefact(tmp_path, art)
    data = yaml.safe_load(path.read_text())
    assert data["measured"] == {"j1.final_err_rad": 0.01, "j2.final_err_rad": 0.02}
    # tuple -> list via writer normalisation
    assert data["metadata"]["joints"] == ["j1", "j2"]


def test_stage3_with_notes_round_trips_through_writer(tmp_path):
    r = _s3(
        metrics={"position_rmse_mm": 2.1},
        failures=(),
        notes=("window short",),
    )
    art = BR.result_to_artefact(
        stage=3,
        arm="ur5e",
        controller="cartesian_motion",
        payload="no_payload",
        theoretical={"rmse_mm": 5.0},
        result=r,
    )
    path = RA.write_r2_artefact(tmp_path, art)
    data = yaml.safe_load(path.read_text())
    assert data["metadata"]["stage3_notes"] == ["window short"]
