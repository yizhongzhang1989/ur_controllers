"""Unit tests for tests/integration/r2_stage2_commands.py.

Covers the commanded all-joints home→via→home generator pre-baked
for the M6.13 R2 stage-2 integration matrix. Pure stdlib — runs in
the unit gate.

The tests exercise the generator in isolation (sample count,
endpoints, waveform properties, inactive joints that happen to match
home) and against the sibling ``r2_stage2_assertions`` evaluator so a
future orchestrator feeding this trace into
``evaluate_all_joints_joint_space`` is guaranteed to meet the
evaluator's input contract.
"""

from __future__ import annotations

import importlib.util
import math
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
CMDS_PATH = REPO_ROOT / "tests" / "integration" / "r2_stage2_commands.py"
ASSERT_PATH = REPO_ROOT / "tests" / "integration" / "r2_stage2_assertions.py"


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec and spec.loader
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


cmds = _load("r2_stage2_commands_uut", CMDS_PATH)
r2s2 = _load("r2_stage2_assertions_for_cmds", ASSERT_PATH)


UR_JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
UR_HOME = (0.0, -1.57, 1.57, -1.57, -1.57, 0.0)
# Hand-picked cluttered pose — all six joints move.
UR_VIA = (0.8, -1.0, 1.2, -0.9, -1.2, 0.4)


# ---------------------------------------------------------------------------
# Export surface / dataclass basics
# ---------------------------------------------------------------------------


def test_exports_public_surface():
    assert set(cmds.__all__) == {
        "AllJointsCommandTrace",
        "home_to_pose_to_home_command",
    }


def test_trace_is_frozen_dataclass():
    trc = cmds.home_to_pose_to_home_command(
        joint_names=UR_JOINTS,
        home_positions_rad=UR_HOME,
        via_pose_rad=UR_VIA,
        duration_s=5.0,
        dt_s=0.01,
    )
    with pytest.raises(Exception):
        trc.times = (0.0,)  # type: ignore[misc]


def test_len_and_duration_properties():
    trc = cmds.home_to_pose_to_home_command(
        joint_names=UR_JOINTS,
        home_positions_rad=UR_HOME,
        via_pose_rad=UR_VIA,
        duration_s=5.0,
        dt_s=0.01,
    )
    assert len(trc) == 501
    assert trc.duration_s == pytest.approx(5.0, abs=1e-12)


def test_trace_echoes_home_and_via():
    trc = cmds.home_to_pose_to_home_command(
        joint_names=UR_JOINTS,
        home_positions_rad=UR_HOME,
        via_pose_rad=UR_VIA,
        duration_s=5.0,
        dt_s=0.01,
    )
    assert trc.joint_names == UR_JOINTS
    assert trc.home_positions_rad == UR_HOME
    assert trc.via_pose_rad == UR_VIA


# ---------------------------------------------------------------------------
# Shape, endpoints, via pose
# ---------------------------------------------------------------------------


def test_times_endpoints_exact():
    trc = cmds.home_to_pose_to_home_command(
        joint_names=UR_JOINTS,
        home_positions_rad=UR_HOME,
        via_pose_rad=UR_VIA,
        duration_s=5.0,
        dt_s=0.01,
    )
    assert trc.times[0] == 0.0
    assert trc.times[-1] == pytest.approx(5.0, abs=1e-12)


def test_endpoints_at_home():
    trc = cmds.home_to_pose_to_home_command(
        joint_names=UR_JOINTS,
        home_positions_rad=UR_HOME,
        via_pose_rad=UR_VIA,
        duration_s=5.0,
        dt_s=0.01,
    )
    for name, home in zip(UR_JOINTS, UR_HOME):
        assert trc.positions[name][0] == pytest.approx(home, abs=1e-12)
        assert trc.positions[name][-1] == pytest.approx(home, abs=1e-12)


def test_midpoint_at_via_pose():
    # 5 s / 0.01 s = 500 intervals ⇒ midpoint sample index = 250.
    trc = cmds.home_to_pose_to_home_command(
        joint_names=UR_JOINTS,
        home_positions_rad=UR_HOME,
        via_pose_rad=UR_VIA,
        duration_s=5.0,
        dt_s=0.01,
    )
    mid_idx = (len(trc.times) - 1) // 2
    assert trc.times[mid_idx] == pytest.approx(2.5, abs=1e-9)
    for name, via in zip(UR_JOINTS, UR_VIA):
        assert trc.positions[name][mid_idx] == pytest.approx(via, abs=1e-12)


def test_waveform_matches_closed_form():
    duration_s = 5.0
    dt_s = 0.01
    trc = cmds.home_to_pose_to_home_command(
        joint_names=UR_JOINTS,
        home_positions_rad=UR_HOME,
        via_pose_rad=UR_VIA,
        duration_s=duration_s,
        dt_s=dt_s,
    )
    two_pi_over_T = 2.0 * math.pi / duration_s
    for name, home, via in zip(UR_JOINTS, UR_HOME, UR_VIA):
        trace = trc.positions[name]
        delta = via - home
        for t, q in zip(trc.times, trace):
            s = 0.5 * (1.0 - math.cos(two_pi_over_T * t))
            expected = home + delta * s
            assert q == pytest.approx(expected, abs=1e-12)


def test_joint_with_no_motion_stays_at_home_throughout():
    # Via pose matches home for elbow_joint only — that joint's trace
    # must stay at home for every sample; other joints still move.
    via = list(UR_VIA)
    via[UR_JOINTS.index("elbow_joint")] = UR_HOME[UR_JOINTS.index("elbow_joint")]
    trc = cmds.home_to_pose_to_home_command(
        joint_names=UR_JOINTS,
        home_positions_rad=UR_HOME,
        via_pose_rad=tuple(via),
        duration_s=5.0,
        dt_s=0.01,
    )
    home_elbow = UR_HOME[UR_JOINTS.index("elbow_joint")]
    for q in trc.positions["elbow_joint"]:
        assert q == pytest.approx(home_elbow, abs=1e-12)
    # A moving joint (shoulder_pan) should visit a sample that differs
    # from its home value by more than 1e-9 rad.
    pan_home = UR_HOME[0]
    assert any(abs(q - pan_home) > 1e-9 for q in trc.positions["shoulder_pan_joint"])


def test_zero_velocity_at_endpoints_numerical():
    # Finite-difference velocity at t=0 and t=T should be ~0.
    trc = cmds.home_to_pose_to_home_command(
        joint_names=UR_JOINTS,
        home_positions_rad=UR_HOME,
        via_pose_rad=UR_VIA,
        duration_s=5.0,
        dt_s=0.001,
    )
    for name in UR_JOINTS:
        q = trc.positions[name]
        dt = trc.times[1] - trc.times[0]
        v_start = (q[1] - q[0]) / dt
        v_end = (q[-1] - q[-2]) / dt
        # With dt=1 ms and the cosine shape, |v| at the first step is
        # at most (via-home)*(2π/T)² * dt / 2 ≈ 1.5 * 1.58 * 5e-4 ≈
        # 1.2e-3; bound loosely at 5e-3.
        assert abs(v_start) < 5e-3
        assert abs(v_end) < 5e-3


def test_positions_mapping_is_readonly():
    trc = cmds.home_to_pose_to_home_command(
        joint_names=UR_JOINTS,
        home_positions_rad=UR_HOME,
        via_pose_rad=UR_VIA,
        duration_s=5.0,
        dt_s=0.01,
    )
    with pytest.raises(TypeError):
        trc.positions["new_joint"] = (0.0,)  # type: ignore[index]


# ---------------------------------------------------------------------------
# Validation
# ---------------------------------------------------------------------------


def test_validation_empty_joint_names():
    with pytest.raises(ValueError, match="non-empty"):
        cmds.home_to_pose_to_home_command(
            joint_names=(),
            home_positions_rad=(),
            via_pose_rad=(),
            duration_s=5.0,
            dt_s=0.01,
        )


def test_validation_duplicate_joint_names():
    with pytest.raises(ValueError, match="duplicates"):
        cmds.home_to_pose_to_home_command(
            joint_names=("a", "a"),
            home_positions_rad=(0.0, 0.0),
            via_pose_rad=(0.1, 0.2),
            duration_s=5.0,
            dt_s=0.01,
        )


def test_validation_home_length_mismatch():
    with pytest.raises(ValueError, match="home_positions_rad"):
        cmds.home_to_pose_to_home_command(
            joint_names=UR_JOINTS,
            home_positions_rad=UR_HOME[:-1],
            via_pose_rad=UR_VIA,
            duration_s=5.0,
            dt_s=0.01,
        )


def test_validation_via_length_mismatch():
    with pytest.raises(ValueError, match="via_pose_rad"):
        cmds.home_to_pose_to_home_command(
            joint_names=UR_JOINTS,
            home_positions_rad=UR_HOME,
            via_pose_rad=UR_VIA[:-1],
            duration_s=5.0,
            dt_s=0.01,
        )


def test_validation_home_non_finite():
    home = list(UR_HOME)
    home[2] = math.inf
    with pytest.raises(ValueError, match="home_positions_rad"):
        cmds.home_to_pose_to_home_command(
            joint_names=UR_JOINTS,
            home_positions_rad=tuple(home),
            via_pose_rad=UR_VIA,
            duration_s=5.0,
            dt_s=0.01,
        )


def test_validation_via_non_finite():
    via = list(UR_VIA)
    via[3] = math.nan
    with pytest.raises(ValueError, match="via_pose_rad"):
        cmds.home_to_pose_to_home_command(
            joint_names=UR_JOINTS,
            home_positions_rad=UR_HOME,
            via_pose_rad=tuple(via),
            duration_s=5.0,
            dt_s=0.01,
        )


def test_validation_non_positive_duration():
    with pytest.raises(ValueError, match="duration_s"):
        cmds.home_to_pose_to_home_command(
            joint_names=UR_JOINTS,
            home_positions_rad=UR_HOME,
            via_pose_rad=UR_VIA,
            duration_s=0.0,
            dt_s=0.01,
        )


def test_validation_negative_duration():
    with pytest.raises(ValueError, match="duration_s"):
        cmds.home_to_pose_to_home_command(
            joint_names=UR_JOINTS,
            home_positions_rad=UR_HOME,
            via_pose_rad=UR_VIA,
            duration_s=-1.0,
            dt_s=0.01,
        )


def test_validation_duration_not_finite():
    with pytest.raises(ValueError, match="duration_s"):
        cmds.home_to_pose_to_home_command(
            joint_names=UR_JOINTS,
            home_positions_rad=UR_HOME,
            via_pose_rad=UR_VIA,
            duration_s=math.inf,
            dt_s=0.01,
        )


def test_validation_dt_below_minimum():
    with pytest.raises(ValueError, match="dt_s"):
        cmds.home_to_pose_to_home_command(
            joint_names=UR_JOINTS,
            home_positions_rad=UR_HOME,
            via_pose_rad=UR_VIA,
            duration_s=5.0,
            dt_s=1e-9,
        )


def test_validation_dt_exceeds_duration():
    with pytest.raises(ValueError, match="exceeds duration_s"):
        cmds.home_to_pose_to_home_command(
            joint_names=UR_JOINTS,
            home_positions_rad=UR_HOME,
            via_pose_rad=UR_VIA,
            duration_s=0.5,
            dt_s=1.0,
        )


def test_validation_rejects_single_interval_for_midpoint():
    # 1.0 / 0.5 ⇒ 2 intervals is the minimum; 1.0 / 0.6 rounds to 2,
    # but 1.0 / 0.9 rounds to 1 which must fail so the via sample
    # actually exists in the trace.
    with pytest.raises(ValueError, match="midpoint"):
        cmds.home_to_pose_to_home_command(
            joint_names=UR_JOINTS,
            home_positions_rad=UR_HOME,
            via_pose_rad=UR_VIA,
            duration_s=1.0,
            dt_s=0.9,
        )


# ---------------------------------------------------------------------------
# Interop with r2_stage2_assertions
# ---------------------------------------------------------------------------


def test_perfectly_tracked_trace_passes_stage2_evaluator():
    # Feed the commanded trace in as both the commanded and the
    # measured side → zero tracking error → evaluator must report OK.
    trc = cmds.home_to_pose_to_home_command(
        joint_names=UR_JOINTS,
        home_positions_rad=UR_HOME,
        via_pose_rad=UR_VIA,
        duration_s=5.0,
        dt_s=0.01,
    )
    result = r2s2.evaluate_all_joints_joint_space(
        "test_controller",
        times=trc.times,
        measured_positions=dict(trc.positions),
        commanded_positions=dict(trc.positions),
        completion_tol_rad=0.01,
        peak_tracking_err_rad=0.01,
    )
    assert result.ok, result.format()
    assert result.failures == ()
