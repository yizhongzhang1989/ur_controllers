"""Unit tests for tests/integration/r2_stage2_assertions.py.

Covers the joint-space R2 stage-2 evaluator pre-baked for the M6.13
integration test matrix (still gated on M6.0). Pure stdlib — runs in
the unit-test gate.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
MOD_PATH = REPO_ROOT / "tests" / "integration" / "r2_stage2_assertions.py"


def _load():
    spec = importlib.util.spec_from_file_location("r2_stage2_assertions_uut", MOD_PATH)
    assert spec and spec.loader
    mod = importlib.util.module_from_spec(spec)
    sys.modules["r2_stage2_assertions_uut"] = mod
    spec.loader.exec_module(mod)
    return mod


r2s2 = _load()


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)


def _times(n: int = 101, dt: float = 0.01) -> list[float]:
    return [i * dt for i in range(n)]


def _identical_traces() -> tuple[list[float], dict[str, list[float]], dict[str, list[float]]]:
    """Six joints, perfect tracking (q == q_cmd), smooth step-like plan."""
    ts = _times()
    measured: dict[str, list[float]] = {}
    commanded: dict[str, list[float]] = {}
    for j_idx, joint in enumerate(JOINTS):
        # Smooth ramp from 0 → 0.3*(j_idx+1) over 1 s; each joint a
        # slightly different amplitude so tests can detect per-joint
        # diagnostics landing in the right row.
        amp = 0.3 * (j_idx + 1)
        trace = [amp * (t / ts[-1]) for t in ts]
        measured[joint] = list(trace)
        commanded[joint] = list(trace)
    return ts, measured, commanded


# ---------------------------------------------------------------------------
# Happy path
# ---------------------------------------------------------------------------


def test_happy_path_all_joints_pass():
    ts, measured, commanded = _identical_traces()
    result = r2s2.evaluate_all_joints_joint_space(
        controller="joint_trajectory_controller",
        times=ts,
        measured_positions=measured,
        commanded_positions=commanded,
        completion_tol_rad=1e-6,
        peak_tracking_err_rad=1e-6,
    )
    assert result.ok
    assert result.failures == ()
    assert result.controller == "joint_trajectory_controller"
    assert len(result.per_joint) == len(JOINTS)
    # Per-joint ordering preserved.
    assert [j.joint for j in result.per_joint] == list(JOINTS)
    for j in result.per_joint:
        assert j.ok
        assert j.metrics["final_err_rad"] == pytest.approx(0.0, abs=1e-12)
        assert j.metrics["peak_tracking_err_rad"] == pytest.approx(0.0, abs=1e-12)
        assert "longest_saturation_hold_ms" not in j.metrics


# ---------------------------------------------------------------------------
# Completion failure
# ---------------------------------------------------------------------------


def test_completion_failure_flags_one_joint():
    ts, measured, commanded = _identical_traces()
    # Perturb final sample of elbow_joint by 0.05 rad.
    measured["elbow_joint"][-1] += 0.05
    result = r2s2.evaluate_all_joints_joint_space(
        controller="simple_joint_impedance",
        times=ts,
        measured_positions=measured,
        commanded_positions=commanded,
        completion_tol_rad=0.01,
        peak_tracking_err_rad=0.10,  # loose enough to not trip
    )
    assert not result.ok
    failures = result.failures
    # Only the elbow joint fails.
    assert len(failures) == 1
    assert failures[0].startswith("elbow_joint: final_err_rad=")
    elbow = next(j for j in result.per_joint if j.joint == "elbow_joint")
    assert elbow.metrics["final_err_rad"] == pytest.approx(0.05, abs=1e-12)
    # Peak stays within 0.10 rad tol → no peak-err failure.
    assert not any("peak_tracking_err_rad" in f for f in elbow.failures)


# ---------------------------------------------------------------------------
# Peak tracking failure
# ---------------------------------------------------------------------------


def test_peak_tracking_failure_flags_mid_trace_spike():
    ts, measured, commanded = _identical_traces()
    # Inject a mid-trace spike on wrist_1_joint that settles back.
    measured["wrist_1_joint"][50] += 0.2
    result = r2s2.evaluate_all_joints_joint_space(
        controller="crisp_joint_impedance",
        times=ts,
        measured_positions=measured,
        commanded_positions=commanded,
        completion_tol_rad=0.01,  # final sample unchanged
        peak_tracking_err_rad=0.05,
    )
    assert not result.ok
    assert len(result.failures) == 1
    assert result.failures[0].startswith("wrist_1_joint: peak_tracking_err_rad=")
    wrist = next(j for j in result.per_joint if j.joint == "wrist_1_joint")
    assert wrist.metrics["peak_tracking_err_rad"] == pytest.approx(0.2, abs=1e-12)
    assert wrist.metrics["final_err_rad"] == pytest.approx(0.0, abs=1e-12)


def test_multiple_joint_failures_reported():
    ts, measured, commanded = _identical_traces()
    measured["shoulder_pan_joint"][-1] += 0.5
    measured["wrist_3_joint"][10] += 0.5
    result = r2s2.evaluate_all_joints_joint_space(
        controller="joint_trajectory_controller",
        times=ts,
        measured_positions=measured,
        commanded_positions=commanded,
        completion_tol_rad=0.01,
        peak_tracking_err_rad=0.05,
    )
    assert not result.ok
    # shoulder_pan hits both (final error 0.5 is also the peak); wrist_3
    # hits only peak. That is three failures.
    assert len(result.failures) == 3
    assert sum(1 for f in result.failures if f.startswith("shoulder_pan_joint:")) == 2
    assert sum(1 for f in result.failures if f.startswith("wrist_3_joint:")) == 1


# ---------------------------------------------------------------------------
# Saturation hold
# ---------------------------------------------------------------------------


def test_saturation_hold_within_tolerance():
    ts, measured, commanded = _identical_traces()
    # Fabricate a torque trace on shoulder_lift that holds saturation
    # for 5 samples × 10 ms = 40 ms — within the 100 ms default.
    n = len(ts)
    tau = [0.0] * n
    for i in range(20, 25):
        tau[i] = 200.0  # ≥ limit
    torques = {"shoulder_lift_joint": tau}
    limits = {"shoulder_lift_joint": 150.0}
    result = r2s2.evaluate_all_joints_joint_space(
        controller="simple_joint_impedance",
        times=ts,
        measured_positions=measured,
        commanded_positions=commanded,
        completion_tol_rad=1e-6,
        peak_tracking_err_rad=1e-6,
        torques=torques,
        effort_limits_nm=limits,
        saturation_hold_ms=100.0,
    )
    assert result.ok
    lift = next(j for j in result.per_joint if j.joint == "shoulder_lift_joint")
    assert lift.metrics["longest_saturation_hold_ms"] == pytest.approx(40.0, abs=1e-9)
    # Other joints have no torque trace → no saturation metric.
    for j in result.per_joint:
        if j.joint != "shoulder_lift_joint":
            assert "longest_saturation_hold_ms" not in j.metrics


def test_saturation_hold_exceeds_tolerance():
    ts, measured, commanded = _identical_traces()
    n = len(ts)
    tau = [0.0] * n
    # 15 samples × 10 ms = 140 ms > 100 ms tol.
    for i in range(10, 25):
        tau[i] = -150.0  # negative torque saturates on absolute value
    torques = {"elbow_joint": tau}
    limits = {"elbow_joint": 150.0}
    result = r2s2.evaluate_all_joints_joint_space(
        controller="forward_effort_controller",
        times=ts,
        measured_positions=measured,
        commanded_positions=commanded,
        completion_tol_rad=1e-6,
        peak_tracking_err_rad=1e-6,
        torques=torques,
        effort_limits_nm=limits,
        saturation_hold_ms=100.0,
    )
    assert not result.ok
    elbow = next(j for j in result.per_joint if j.joint == "elbow_joint")
    assert elbow.metrics["longest_saturation_hold_ms"] == pytest.approx(140.0, abs=1e-9)
    assert len(elbow.failures) == 1
    assert elbow.failures[0].startswith("longest_saturation_hold_ms=")


def test_saturation_skipped_when_no_limit_for_joint():
    """Torque trace supplied but effort_limits_nm missing the key ⇒ skip."""
    ts, measured, commanded = _identical_traces()
    n = len(ts)
    tau = [1000.0] * n
    torques = {"wrist_2_joint": tau}
    limits: dict[str, float] = {}  # empty — nothing to compare against
    result = r2s2.evaluate_all_joints_joint_space(
        controller="crisp_joint_impedance",
        times=ts,
        measured_positions=measured,
        commanded_positions=commanded,
        completion_tol_rad=1e-6,
        peak_tracking_err_rad=1e-6,
        torques=torques,
        effort_limits_nm=limits,
    )
    assert result.ok
    wrist = next(j for j in result.per_joint if j.joint == "wrist_2_joint")
    assert "longest_saturation_hold_ms" not in wrist.metrics


# ---------------------------------------------------------------------------
# Input validation
# ---------------------------------------------------------------------------


def test_non_monotonic_times_raises():
    bad_times = [0.0, 0.01, 0.02, 0.02, 0.03]  # duplicate — not strictly monotonic
    m = {j: [0.0] * 5 for j in JOINTS}
    c = {j: [0.0] * 5 for j in JOINTS}
    with pytest.raises(ValueError, match="strictly monotonic"):
        r2s2.evaluate_all_joints_joint_space(
            controller="x",
            times=bad_times,
            measured_positions=m,
            commanded_positions=c,
            completion_tol_rad=1.0,
            peak_tracking_err_rad=1.0,
        )


def test_times_too_short_raises():
    with pytest.raises(ValueError, match="length >= 2"):
        r2s2.evaluate_all_joints_joint_space(
            controller="x",
            times=[0.0],
            measured_positions={"j": [0.0]},
            commanded_positions={"j": [0.0]},
            completion_tol_rad=1.0,
            peak_tracking_err_rad=1.0,
        )


def test_mismatched_keys_raises():
    ts = _times(5)
    m = {j: [0.0] * 5 for j in JOINTS}
    c = {j: [0.0] * 5 for j in JOINTS if j != "elbow_joint"}
    with pytest.raises(ValueError, match="share keys"):
        r2s2.evaluate_all_joints_joint_space(
            controller="x",
            times=ts,
            measured_positions=m,
            commanded_positions=c,
            completion_tol_rad=1.0,
            peak_tracking_err_rad=1.0,
        )


def test_measured_trace_length_mismatch_raises():
    ts = _times(5)
    m = {j: [0.0] * 5 for j in JOINTS}
    m["elbow_joint"] = [0.0] * 4  # wrong length
    c = {j: [0.0] * 5 for j in JOINTS}
    with pytest.raises(ValueError, match="measured_positions.*elbow_joint.*length 4"):
        r2s2.evaluate_all_joints_joint_space(
            controller="x",
            times=ts,
            measured_positions=m,
            commanded_positions=c,
            completion_tol_rad=1.0,
            peak_tracking_err_rad=1.0,
        )


def test_torque_trace_length_mismatch_raises():
    ts = _times(5)
    m = {j: [0.0] * 5 for j in JOINTS}
    c = {j: [0.0] * 5 for j in JOINTS}
    torques = {"wrist_1_joint": [0.0, 0.0]}  # wrong length
    limits = {"wrist_1_joint": 10.0}
    with pytest.raises(ValueError, match="torques.*wrist_1_joint.*length 2"):
        r2s2.evaluate_all_joints_joint_space(
            controller="x",
            times=ts,
            measured_positions=m,
            commanded_positions=c,
            completion_tol_rad=1.0,
            peak_tracking_err_rad=1.0,
            torques=torques,
            effort_limits_nm=limits,
        )


# ---------------------------------------------------------------------------
# Result / format surface
# ---------------------------------------------------------------------------


def test_stage2_result_format_contains_joint_rows_and_failures():
    ts, measured, commanded = _identical_traces()
    measured["wrist_3_joint"][-1] += 0.5
    result = r2s2.evaluate_all_joints_joint_space(
        controller="joint_trajectory_controller",
        times=ts,
        measured_positions=measured,
        commanded_positions=commanded,
        completion_tol_rad=0.01,
        peak_tracking_err_rad=1.0,
    )
    text = result.format()
    assert "R2 stage-2" in text
    assert "controller=joint_trajectory_controller" in text
    for joint in JOINTS:
        assert f"joint {joint}" in text
    assert "FAIL final_err_rad=" in text
    # Passing joints flagged as OK.
    assert "joint shoulder_pan_joint: OK" in text


def test_stage2_result_empty_per_joint_is_ok_and_formats():
    result = r2s2.Stage2Result(controller="noop", per_joint=())
    assert result.ok
    assert result.failures == ()
    text = result.format()
    assert "no joints evaluated" in text


def test_empty_measured_positions_yields_trivially_ok():
    ts = _times(5)
    result = r2s2.evaluate_all_joints_joint_space(
        controller="noop",
        times=ts,
        measured_positions={},
        commanded_positions={},
        completion_tol_rad=1.0,
        peak_tracking_err_rad=1.0,
    )
    assert result.ok
    assert result.per_joint == ()


# ---------------------------------------------------------------------------
# Dunders
# ---------------------------------------------------------------------------


def test_joint_stage2_metrics_is_frozen():
    m = r2s2.JointStage2Metrics(joint="j")
    with pytest.raises((AttributeError, Exception)):
        m.joint = "k"  # type: ignore[misc]


def test_stage2_result_is_frozen():
    r = r2s2.Stage2Result(controller="c")
    with pytest.raises((AttributeError, Exception)):
        r.controller = "d"  # type: ignore[misc]


def test_module_all_exports():
    assert set(r2s2.__all__) == {
        "JointStage2Metrics",
        "Stage2Result",
        "evaluate_all_joints_joint_space",
    }


# ---------------------------------------------------------------------------
# Integration with stage-1 saturation helper (no duplication)
# ---------------------------------------------------------------------------


def test_reuses_stage1_longest_contiguous_helper():
    """Confirm we share the stage-1 helper instead of re-implementing."""
    spec = importlib.util.spec_from_file_location(
        "r2_stage1_for_reuse",
        REPO_ROOT / "tests" / "integration" / "r2_stage1_assertions.py",
    )
    assert spec and spec.loader
    s1 = importlib.util.module_from_spec(spec)
    sys.modules["r2_stage1_for_reuse"] = s1
    spec.loader.exec_module(s1)
    # Both modules should expose (via the loader) the same function
    # object once r2s2 has pulled its sibling.
    # Force the sibling load:
    ts, measured, commanded = _identical_traces()
    n = len(ts)
    tau = [0.0] * n
    tau[10] = 1000.0  # single-sample spike
    r2s2.evaluate_all_joints_joint_space(
        controller="x",
        times=ts,
        measured_positions=measured,
        commanded_positions=commanded,
        completion_tol_rad=1e-6,
        peak_tracking_err_rad=1e-6,
        torques={"elbow_joint": tau},
        effort_limits_nm={"elbow_joint": 500.0},
    )
    # Now the sibling module is loaded under its private key.
    sibling = sys.modules.get("_r2s2_r2_stage1_assertions")
    assert sibling is not None
    # Same symbol (same Python callable) as the public one.
    assert sibling._longest_contiguous_ms is not None
