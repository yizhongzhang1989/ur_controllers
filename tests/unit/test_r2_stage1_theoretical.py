"""Unit tests for :mod:`tests.integration.r2_stage1_theoretical`.

Covers: export surface; dispatch across all three supported
``response_model`` values; argument-type / extraneous-argument
rejection matrix; unknown-response-model rejection; delegation to
:func:`expectations_loader.second_order_response` for the
``second_order`` branch (including the loader's K>0/J>0/D>=0
validation errors); round-trip through
:func:`r2_result_to_artefact.result_to_artefact` and
:func:`r2_run_artefact.write_r2_artefact` so the builder's output
shape is provably compatible with the writer.
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


def _load(name: str, path: Path) -> ModuleType:
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)  # type: ignore[arg-type]
    sys.modules[name] = mod
    assert spec and spec.loader
    spec.loader.exec_module(mod)
    return mod


# Load in dependency order so siblings share module keys.
EL = _load("expectations_loader", INT_DIR / "expectations_loader.py")
SA = _load("signal_analysis", INT_DIR / "signal_analysis.py")
RA = _load("r2_run_artefact", INT_DIR / "r2_run_artefact.py")
S1 = _load("r2_stage1_assertions", INT_DIR / "r2_stage1_assertions.py")
S2 = _load("r2_stage2_assertions", INT_DIR / "r2_stage2_assertions.py")
S2C = _load("r2_stage2_cartesian", INT_DIR / "r2_stage2_cartesian.py")
S3 = _load("r2_stage3_assertions", INT_DIR / "r2_stage3_assertions.py")
BR = _load("r2_result_to_artefact", INT_DIR / "r2_result_to_artefact.py")
TH = _load("r2_stage1_theoretical", INT_DIR / "r2_stage1_theoretical.py")


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


def _joint_exp(**kw):
    defaults = dict(
        name="shoulder_pan_joint",
        home_rad=0.0,
        effort_limit_nm=150.0,
        effective_inertia_kg_m2=3.4,
    )
    defaults.update(kw)
    return EL.JointExpectation(**defaults)


def _controller_exp(**kw):
    defaults = dict(
        name="crisp_joint_impedance",
        interface="effort",
        response_model="second_order",
        tolerances={
            "bounded_err_rad": 0.15,
            "chatter_velocity_rms_rad_s": 0.05,
            "damping_ratio_pct": 20.0,
        },
    )
    defaults.update(kw)
    return EL.ControllerExpectation(**defaults)


def _first_order():
    return _controller_exp(
        name="joint_trajectory_controller",
        interface="position",
        response_model="first_order_lag",
        tolerances={
            "steady_state_err_rad": 0.01,
            "drift_rad_per_10s": 0.001,
            "fft_peak_db_above_noise_floor": 6.0,
        },
    )


def _open_loop():
    return _controller_exp(
        name="forward_effort_controller",
        interface="effort",
        response_model="open_loop_torque",
        tolerances={
            "max_runaway_rad": 0.5,
            "saturation_hold_ms": 100.0,
        },
    )


# ---------------------------------------------------------------------------
# Export surface
# ---------------------------------------------------------------------------


def test_export_surface():
    assert set(TH.__all__) == {"SUPPORTED_RESPONSE_MODELS", "theoretical_for_stage1"}
    assert TH.SUPPORTED_RESPONSE_MODELS == (
        "first_order_lag",
        "open_loop_torque",
        "second_order",
    )


# ---------------------------------------------------------------------------
# first_order_lag
# ---------------------------------------------------------------------------


def test_first_order_happy_path():
    got = TH.theoretical_for_stage1(_first_order())
    assert got == {
        "response_model": "first_order_lag",
        "interface": "position",
        "tolerances": {
            "steady_state_err_rad": 0.01,
            "drift_rad_per_10s": 0.001,
            "fft_peak_db_above_noise_floor": 6.0,
        },
    }


def test_first_order_tolerances_are_plain_floats():
    got = TH.theoretical_for_stage1(_first_order())
    for v in got["tolerances"].values():
        assert isinstance(v, float)


def test_first_order_propagates_interface_verbatim():
    # Three first-order-lag controllers in the schema: JTC (position),
    # forward_position (position), forward_velocity (velocity). The
    # builder must echo whatever `interface` the loader surfaces.
    for interface in ("position", "velocity"):
        exp = _controller_exp(
            name=f"ctrl_{interface}",
            interface=interface,
            response_model="first_order_lag",
            tolerances={
                "steady_state_err_rad": 0.01,
                "drift_rad_per_10s": 0.001,
                "fft_peak_db_above_noise_floor": 6.0,
            },
        )
        assert TH.theoretical_for_stage1(exp)["interface"] == interface


@pytest.mark.parametrize(
    "kwargs",
    [
        {"joint_exp": _joint_exp()},
        {"stiffness_k": 100.0},
        {"damping_d": 5.0},
    ],
)
def test_first_order_rejects_second_order_only_kwargs(kwargs):
    with pytest.raises(ValueError, match="does not consume it"):
        TH.theoretical_for_stage1(_first_order(), **kwargs)


def test_first_order_missing_tolerance_key_raises():
    exp = _controller_exp(
        name="broken",
        interface="position",
        response_model="first_order_lag",
        tolerances={"steady_state_err_rad": 0.01},  # missing two keys
    )
    with pytest.raises(KeyError, match="drift_rad_per_10s|fft_peak_db_above_noise_floor"):
        TH.theoretical_for_stage1(exp)


# ---------------------------------------------------------------------------
# open_loop_torque
# ---------------------------------------------------------------------------


def test_open_loop_happy_path():
    got = TH.theoretical_for_stage1(_open_loop())
    assert got == {
        "response_model": "open_loop_torque",
        "interface": "effort",
        "tolerances": {
            "max_runaway_rad": 0.5,
            "saturation_hold_ms": 100.0,
        },
    }


@pytest.mark.parametrize(
    "kwargs",
    [
        {"joint_exp": _joint_exp()},
        {"stiffness_k": 100.0},
        {"damping_d": 5.0},
        {"joint_exp": _joint_exp(), "stiffness_k": 100.0, "damping_d": 5.0},
    ],
)
def test_open_loop_rejects_second_order_only_kwargs(kwargs):
    with pytest.raises(ValueError, match="does not consume it"):
        TH.theoretical_for_stage1(_open_loop(), **kwargs)


def test_open_loop_missing_tolerance_key_raises():
    exp = _controller_exp(
        name="broken",
        interface="effort",
        response_model="open_loop_torque",
        tolerances={"max_runaway_rad": 0.5},
    )
    with pytest.raises(KeyError, match="saturation_hold_ms"):
        TH.theoretical_for_stage1(exp)


# ---------------------------------------------------------------------------
# second_order
# ---------------------------------------------------------------------------


def test_second_order_happy_path():
    c = _controller_exp()
    j = _joint_exp()
    got = TH.theoretical_for_stage1(c, joint_exp=j, stiffness_k=100.0, damping_d=10.0)
    # omega_n = sqrt(100 / 3.4); zeta = 10 / (2*sqrt(100*3.4))
    expected_omega_n = math.sqrt(100.0 / 3.4)
    expected_zeta = 10.0 / (2.0 * math.sqrt(100.0 * 3.4))
    assert got["response_model"] == "second_order"
    assert got["interface"] == "effort"
    assert got["response"]["joint"] == "shoulder_pan_joint"
    assert got["response"]["stiffness_k"] == 100.0
    assert got["response"]["damping_d"] == 10.0
    assert got["response"]["effective_inertia_kg_m2"] == 3.4
    assert math.isclose(got["response"]["omega_n_rad_s"], expected_omega_n)
    assert math.isclose(got["response"]["zeta"], expected_zeta)
    assert got["tolerances"] == {
        "bounded_err_rad": 0.15,
        "chatter_velocity_rms_rad_s": 0.05,
        "damping_ratio_pct": 20.0,
    }


def test_second_order_response_values_are_plain_floats():
    got = TH.theoretical_for_stage1(
        _controller_exp(), joint_exp=_joint_exp(), stiffness_k=100, damping_d=10
    )
    for v in got["response"].values():
        if isinstance(v, (int, float)) or not isinstance(v, str):
            # numeric fields must be plain floats, not ints (so the
            # writer produces deterministic YAML regardless of caller
            # dtype). The 'joint' field is str.
            if v != got["response"]["joint"]:
                assert isinstance(v, float)


def test_second_order_accepts_int_k_d_and_coerces_to_float():
    got = TH.theoretical_for_stage1(
        _controller_exp(), joint_exp=_joint_exp(), stiffness_k=100, damping_d=10
    )
    assert got["response"]["stiffness_k"] == 100.0
    assert got["response"]["damping_d"] == 10.0


def test_second_order_overdamped_zeta_ge_one():
    # K=10, J=1, D=20 → zeta = 20 / (2*sqrt(10)) ≈ 3.16 (overdamped).
    got = TH.theoretical_for_stage1(
        _controller_exp(),
        joint_exp=_joint_exp(effective_inertia_kg_m2=1.0),
        stiffness_k=10.0,
        damping_d=20.0,
    )
    assert got["response"]["zeta"] > 1.0


def test_second_order_zero_damping():
    got = TH.theoretical_for_stage1(
        _controller_exp(),
        joint_exp=_joint_exp(effective_inertia_kg_m2=1.0),
        stiffness_k=25.0,
        damping_d=0.0,
    )
    assert got["response"]["zeta"] == 0.0
    assert math.isclose(got["response"]["omega_n_rad_s"], 5.0)


@pytest.mark.parametrize("missing", ["joint_exp", "stiffness_k", "damping_d"])
def test_second_order_missing_required_kwargs(missing):
    kwargs = {
        "joint_exp": _joint_exp(),
        "stiffness_k": 100.0,
        "damping_d": 10.0,
    }
    del kwargs[missing]
    with pytest.raises(ValueError, match=f"{missing} is required"):
        TH.theoretical_for_stage1(_controller_exp(), **kwargs)


def test_second_order_negative_k_raises_via_loader():
    with pytest.raises(ValueError, match="stiffness K must be > 0"):
        TH.theoretical_for_stage1(
            _controller_exp(),
            joint_exp=_joint_exp(),
            stiffness_k=-1.0,
            damping_d=10.0,
        )


def test_second_order_negative_d_raises_via_loader():
    with pytest.raises(ValueError, match="damping D must be >= 0"):
        TH.theoretical_for_stage1(
            _controller_exp(),
            joint_exp=_joint_exp(),
            stiffness_k=100.0,
            damping_d=-1.0,
        )


def test_second_order_zero_k_raises_via_loader():
    with pytest.raises(ValueError, match="stiffness K must be > 0"):
        TH.theoretical_for_stage1(
            _controller_exp(),
            joint_exp=_joint_exp(),
            stiffness_k=0.0,
            damping_d=10.0,
        )


def test_second_order_missing_tolerance_key_raises():
    exp = _controller_exp(tolerances={"bounded_err_rad": 0.15, "chatter_velocity_rms_rad_s": 0.05})
    with pytest.raises(KeyError, match="damping_ratio_pct"):
        TH.theoretical_for_stage1(exp, joint_exp=_joint_exp(), stiffness_k=100.0, damping_d=10.0)


# ---------------------------------------------------------------------------
# Type rejection matrix
# ---------------------------------------------------------------------------


def test_rejects_non_controller_exp():
    with pytest.raises(ValueError, match="controller_exp must be"):
        TH.theoretical_for_stage1("not an exp")


def test_rejects_non_joint_exp_for_second_order():
    with pytest.raises(ValueError, match="joint_exp must be"):
        TH.theoretical_for_stage1(
            _controller_exp(),
            joint_exp="not a joint",
            stiffness_k=100.0,
            damping_d=10.0,
        )


@pytest.mark.parametrize("bad_k", ["100", True, [100.0], None])
def test_rejects_non_numeric_stiffness_k(bad_k):
    # None hits the "required" branch; others hit the type branch.
    with pytest.raises(ValueError):
        TH.theoretical_for_stage1(
            _controller_exp(),
            joint_exp=_joint_exp(),
            stiffness_k=bad_k,
            damping_d=10.0,
        )


@pytest.mark.parametrize("bad_d", ["10", True, [10.0], None])
def test_rejects_non_numeric_damping_d(bad_d):
    with pytest.raises(ValueError):
        TH.theoretical_for_stage1(
            _controller_exp(),
            joint_exp=_joint_exp(),
            stiffness_k=100.0,
            damping_d=bad_d,
        )


def test_rejects_unknown_response_model():
    exp = _controller_exp(response_model="pid_cascade")
    with pytest.raises(ValueError, match="unknown response_model"):
        TH.theoretical_for_stage1(exp)


# ---------------------------------------------------------------------------
# Integration with r2_result_to_artefact / r2_run_artefact
# ---------------------------------------------------------------------------


def test_round_trip_first_order_through_writer(tmp_path):
    theoretical = TH.theoretical_for_stage1(_first_order())
    result = S1.Stage1Result(
        controller="joint_trajectory_controller",
        joint="shoulder_pan_joint",
        metrics={"steady_state_err_rad": 0.005},
        failures=(),
    )
    artefact = BR.result_to_artefact(
        stage=1,
        arm="ur5e",
        controller="joint_trajectory_controller",
        payload="no_payload",
        theoretical=theoretical,
        result=result,
    )
    path = RA.write_r2_artefact(tmp_path, artefact)
    data = yaml.safe_load(path.read_text())
    assert data["theoretical"] == {
        "interface": "position",
        "response_model": "first_order_lag",
        "tolerances": {
            "drift_rad_per_10s": 0.001,
            "fft_peak_db_above_noise_floor": 6.0,
            "steady_state_err_rad": 0.01,
        },
    }


def test_round_trip_second_order_through_writer(tmp_path):
    theoretical = TH.theoretical_for_stage1(
        _controller_exp(),
        joint_exp=_joint_exp(),
        stiffness_k=100.0,
        damping_d=10.0,
    )
    result = S1.Stage1Result(
        controller="crisp_joint_impedance",
        joint="shoulder_pan_joint",
        metrics={"peak_err_rad": 0.02, "theoretical_zeta": theoretical["response"]["zeta"]},
        failures=(),
    )
    artefact = BR.result_to_artefact(
        stage=1,
        arm="ur5e",
        controller="crisp_joint_impedance",
        payload="no_payload",
        theoretical=theoretical,
        result=result,
    )
    path = RA.write_r2_artefact(tmp_path, artefact)
    data = yaml.safe_load(path.read_text())
    # Writer sorts nested keys alphabetically; our builder's unsorted
    # dict must still round-trip value-for-value.
    assert data["theoretical"]["response_model"] == "second_order"
    assert data["theoretical"]["interface"] == "effort"
    assert data["theoretical"]["response"]["joint"] == "shoulder_pan_joint"
    assert math.isclose(
        data["theoretical"]["response"]["omega_n_rad_s"],
        theoretical["response"]["omega_n_rad_s"],
    )
    assert math.isclose(data["theoretical"]["response"]["zeta"], theoretical["response"]["zeta"])


def test_round_trip_open_loop_through_writer(tmp_path):
    theoretical = TH.theoretical_for_stage1(_open_loop())
    result = S1.Stage1Result(
        controller="forward_effort_controller",
        joint="shoulder_pan_joint",
        metrics={"runaway_rad": 0.1},
        failures=(),
    )
    artefact = BR.result_to_artefact(
        stage=1,
        arm="ur15",
        controller="forward_effort_controller",
        payload="large_payload",
        theoretical=theoretical,
        result=result,
    )
    path = RA.write_r2_artefact(tmp_path, artefact)
    data = yaml.safe_load(path.read_text())
    assert data["theoretical"] == {
        "interface": "effort",
        "response_model": "open_loop_torque",
        "tolerances": {
            "max_runaway_rad": 0.5,
            "saturation_hold_ms": 100.0,
        },
    }


# ---------------------------------------------------------------------------
# Real expectation YAML integration
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("arm", ["ur5e", "ur15"])
@pytest.mark.parametrize(
    "controller_name",
    [
        "joint_trajectory_controller",
        "forward_position_controller",
        "forward_velocity_controller",
        "forward_effort_controller",
    ],
)
def test_builds_from_real_expectation_yaml_non_second_order(arm, controller_name):
    arm_exp = EL.load_arm(arm)
    c = arm_exp.controller(controller_name)
    got = TH.theoretical_for_stage1(c)
    assert got["response_model"] in ("first_order_lag", "open_loop_torque")
    assert got["interface"] == c.interface
    assert "tolerances" in got


@pytest.mark.parametrize("arm", ["ur5e", "ur15"])
@pytest.mark.parametrize("controller_name", ["crisp_joint_impedance", "simple_joint_impedance"])
def test_builds_from_real_expectation_yaml_second_order(arm, controller_name):
    arm_exp = EL.load_arm(arm)
    c = arm_exp.controller(controller_name)
    j = arm_exp.joint("shoulder_pan_joint")
    got = TH.theoretical_for_stage1(c, joint_exp=j, stiffness_k=100.0, damping_d=10.0)
    assert got["response_model"] == "second_order"
    assert got["interface"] == "effort"
    assert got["response"]["joint"] == "shoulder_pan_joint"
    assert got["response"]["effective_inertia_kg_m2"] == j.effective_inertia_kg_m2
    assert got["response"]["omega_n_rad_s"] > 0.0
    assert got["response"]["zeta"] >= 0.0
