"""Unit tests for tests/integration/r2_jtc_goal.py.

Covers the JTC goal-builder that converts any of the R2 commanded-trace
types (stage-1 step/sine, stage-2 all-joints via-pose, stage-3 IK-folded
joint trajectory) into a :class:`JointTrajectoryGoal`. Pure stdlib —
runs in the unit gate.
"""

from __future__ import annotations

import importlib.util
import math
import sys
from pathlib import Path
from types import MappingProxyType

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
GOAL_PATH = REPO_ROOT / "tests" / "integration" / "r2_jtc_goal.py"
STAGE1_PATH = REPO_ROOT / "tests" / "integration" / "r2_stage1_commands.py"
STAGE2_PATH = REPO_ROOT / "tests" / "integration" / "r2_stage2_commands.py"
JFT_PATH = REPO_ROOT / "tests" / "integration" / "r2_joints_from_tcp.py"
STAGE3_PATH = REPO_ROOT / "tests" / "integration" / "r2_stage3_commands.py"


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec and spec.loader
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


goal_mod = _load("r2_jtc_goal_uut", GOAL_PATH)
stage1 = _load("r2_stage1_cmds_for_goal", STAGE1_PATH)
stage2 = _load("r2_stage2_cmds_for_goal", STAGE2_PATH)
jft = _load("r2_joints_from_tcp_for_goal", JFT_PATH)
stage3 = _load("r2_stage3_cmds_for_goal", STAGE3_PATH)


JOINT_NAMES = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
HOME = (0.0, -1.57, 1.57, -1.57, -1.57, 0.0)


# ---------------------------------------------------------------------------
# Lightweight duck-typed fake for testing the validation surface directly.
# ---------------------------------------------------------------------------


class _FakeTrace:
    def __init__(self, *, joint_names=None, times=None, positions=None):
        if joint_names is not None:
            self.joint_names = joint_names
        if times is not None:
            self.times = times
        if positions is not None:
            self.positions = positions


def _simple_trace(times=(0.0, 1.0, 2.0)):
    names = ("a", "b")
    positions = {
        "a": tuple(float(t) for t in times),
        "b": tuple(2.0 * float(t) for t in times),
    }
    return _FakeTrace(joint_names=names, times=tuple(times), positions=positions)


# ---------------------------------------------------------------------------
# Export surface and dataclass contract
# ---------------------------------------------------------------------------


def test_export_surface():
    assert set(goal_mod.__all__) == {
        "JointTrajectoryPoint",
        "JointTrajectoryGoal",
        "jtc_goal_from_trace",
    }


def test_point_is_frozen():
    pt = goal_mod.JointTrajectoryPoint(positions=(1.0, 2.0), time_from_start_s=0.5)
    with pytest.raises(Exception):
        pt.time_from_start_s = 9.0  # type: ignore[misc]


def test_goal_is_frozen():
    g = goal_mod.JointTrajectoryGoal(joint_names=("a",), points=())
    with pytest.raises(Exception):
        g.joint_names = ("b",)  # type: ignore[misc]


def test_goal_len_and_duration():
    pts = (
        goal_mod.JointTrajectoryPoint(positions=(0.0,), time_from_start_s=0.0),
        goal_mod.JointTrajectoryPoint(positions=(1.0,), time_from_start_s=2.5),
    )
    g = goal_mod.JointTrajectoryGoal(joint_names=("a",), points=pts)
    assert len(g) == 2
    assert g.duration_s == pytest.approx(2.5)


def test_goal_duration_empty():
    g = goal_mod.JointTrajectoryGoal(joint_names=("a",), points=())
    assert g.duration_s == 0.0


# ---------------------------------------------------------------------------
# Happy path
# ---------------------------------------------------------------------------


def test_basic_build_preserves_order():
    trace = _simple_trace(times=(0.0, 1.0, 2.0))
    g = goal_mod.jtc_goal_from_trace(trace)
    assert g.joint_names == ("a", "b")
    assert len(g.points) == 3
    assert g.points[0].positions == (0.0, 0.0)
    assert g.points[1].positions == (1.0, 2.0)
    assert g.points[2].positions == (2.0, 4.0)


def test_time_from_start_uses_trace_times():
    trace = _simple_trace(times=(0.0, 0.5, 1.75))
    g = goal_mod.jtc_goal_from_trace(trace)
    assert g.points[0].time_from_start_s == pytest.approx(0.0)
    assert g.points[1].time_from_start_s == pytest.approx(0.5)
    assert g.points[2].time_from_start_s == pytest.approx(1.75)


def test_time_from_start_rebases_when_trace_times_do_not_start_at_zero():
    trace = _simple_trace(times=(2.0, 3.0, 5.0))
    g = goal_mod.jtc_goal_from_trace(trace)
    assert g.points[0].time_from_start_s == pytest.approx(0.0)
    assert g.points[1].time_from_start_s == pytest.approx(1.0)
    assert g.points[2].time_from_start_s == pytest.approx(3.0)


def test_time_offset_applied():
    trace = _simple_trace(times=(0.0, 1.0, 2.0))
    g = goal_mod.jtc_goal_from_trace(trace, time_offset_s=0.25)
    assert g.points[0].time_from_start_s == pytest.approx(0.25)
    assert g.points[-1].time_from_start_s == pytest.approx(2.25)


def test_skip_initial_sample_drops_first_point_and_rebases():
    trace = _simple_trace(times=(0.0, 1.0, 2.0))
    g = goal_mod.jtc_goal_from_trace(trace, skip_initial_sample=True)
    assert len(g.points) == 2
    assert g.points[0].positions == (1.0, 2.0)
    assert g.points[0].time_from_start_s == pytest.approx(0.0)
    assert g.points[1].time_from_start_s == pytest.approx(1.0)


def test_skip_and_offset_compose():
    trace = _simple_trace(times=(0.0, 1.0, 2.0))
    g = goal_mod.jtc_goal_from_trace(trace, skip_initial_sample=True, time_offset_s=0.1)
    assert g.points[0].time_from_start_s == pytest.approx(0.1)
    assert g.points[1].time_from_start_s == pytest.approx(1.1)


def test_positions_ordered_by_joint_names_not_dict_order():
    positions = {
        "b": (10.0, 20.0),  # note reversed insertion order
        "a": (1.0, 2.0),
    }
    trace = _FakeTrace(joint_names=("a", "b"), times=(0.0, 1.0), positions=positions)
    g = goal_mod.jtc_goal_from_trace(trace)
    assert g.points[0].positions == (1.0, 10.0)
    assert g.points[1].positions == (2.0, 20.0)


def test_accepts_list_joint_names_and_list_times():
    trace = _FakeTrace(
        joint_names=["a", "b"],
        times=[0.0, 1.0],
        positions={"a": [0.0, 1.0], "b": [0.0, -1.0]},
    )
    g = goal_mod.jtc_goal_from_trace(trace)
    assert g.joint_names == ("a", "b")
    assert g.points[0].positions == (0.0, 0.0)
    assert g.points[1].positions == (1.0, -1.0)


def test_single_sample_trace_builds_single_point():
    trace = _FakeTrace(joint_names=("a",), times=(0.0,), positions={"a": (1.5,)})
    g = goal_mod.jtc_goal_from_trace(trace)
    assert len(g.points) == 1
    assert g.points[0].positions == (1.5,)
    assert g.points[0].time_from_start_s == pytest.approx(0.0)


def test_accepts_mappingproxytype_positions():
    # All R2 command trace dataclasses wrap `positions` in MappingProxyType
    # — confirm the builder handles that transparently.
    positions = MappingProxyType({"a": (0.0, 1.0), "b": (2.0, 3.0)})
    trace = _FakeTrace(joint_names=("a", "b"), times=(0.0, 1.0), positions=positions)
    g = goal_mod.jtc_goal_from_trace(trace)
    assert g.points[1].positions == (1.0, 3.0)


# ---------------------------------------------------------------------------
# Validation matrix
# ---------------------------------------------------------------------------


def test_missing_attribute_raises_typeerror():
    with pytest.raises(TypeError, match="joint_names"):
        goal_mod.jtc_goal_from_trace(_FakeTrace(times=(0.0,), positions={}))
    with pytest.raises(TypeError, match="times"):
        goal_mod.jtc_goal_from_trace(_FakeTrace(joint_names=("a",), positions={"a": ()}))
    with pytest.raises(TypeError, match="positions"):
        goal_mod.jtc_goal_from_trace(_FakeTrace(joint_names=("a",), times=(0.0,)))


def test_empty_joint_names_rejected():
    trace = _FakeTrace(joint_names=(), times=(0.0,), positions={})
    with pytest.raises(ValueError, match="joint_names"):
        goal_mod.jtc_goal_from_trace(trace)


def test_duplicate_joint_names_rejected():
    trace = _FakeTrace(joint_names=("a", "a"), times=(0.0,), positions={"a": (0.0,)})
    with pytest.raises(ValueError, match="duplicates"):
        goal_mod.jtc_goal_from_trace(trace)


def test_non_string_joint_name_rejected():
    trace = _FakeTrace(
        joint_names=("a", 42),
        times=(0.0,),
        positions={"a": (0.0,), 42: (0.0,)},
    )
    with pytest.raises(TypeError, match="not a str"):
        goal_mod.jtc_goal_from_trace(trace)


def test_empty_times_rejected():
    trace = _FakeTrace(joint_names=("a",), times=(), positions={"a": ()})
    with pytest.raises(ValueError, match="non-empty"):
        goal_mod.jtc_goal_from_trace(trace)


def test_non_monotonic_times_rejected():
    trace = _FakeTrace(
        joint_names=("a",),
        times=(0.0, 1.0, 0.5),
        positions={"a": (0.0, 0.0, 0.0)},
    )
    with pytest.raises(ValueError, match="monotonic"):
        goal_mod.jtc_goal_from_trace(trace)


def test_repeated_times_rejected():
    trace = _FakeTrace(
        joint_names=("a",),
        times=(0.0, 1.0, 1.0),
        positions={"a": (0.0, 0.0, 0.0)},
    )
    with pytest.raises(ValueError, match="monotonic"):
        goal_mod.jtc_goal_from_trace(trace)


def test_non_finite_time_rejected():
    trace = _FakeTrace(
        joint_names=("a",),
        times=(0.0, float("inf")),
        positions={"a": (0.0, 0.0)},
    )
    with pytest.raises(ValueError, match="finite"):
        goal_mod.jtc_goal_from_trace(trace)


def test_nan_time_rejected():
    trace = _FakeTrace(
        joint_names=("a",),
        times=(0.0, float("nan")),
        positions={"a": (0.0, 0.0)},
    )
    with pytest.raises(ValueError, match="finite"):
        goal_mod.jtc_goal_from_trace(trace)


def test_non_numeric_time_rejected():
    trace = _FakeTrace(joint_names=("a",), times=(0.0, "x"), positions={"a": (0.0, 0.0)})
    with pytest.raises(ValueError, match="numeric"):
        goal_mod.jtc_goal_from_trace(trace)


def test_positions_missing_joint_rejected():
    trace = _FakeTrace(joint_names=("a", "b"), times=(0.0,), positions={"a": (0.0,)})
    with pytest.raises(ValueError, match="missing"):
        goal_mod.jtc_goal_from_trace(trace)


def test_positions_extra_joint_rejected():
    trace = _FakeTrace(
        joint_names=("a",),
        times=(0.0,),
        positions={"a": (0.0,), "b": (0.0,)},
    )
    with pytest.raises(ValueError, match="not in joint_names"):
        goal_mod.jtc_goal_from_trace(trace)


def test_positions_not_a_mapping_rejected():
    trace = _FakeTrace(joint_names=("a",), times=(0.0,), positions=[(0.0,)])
    with pytest.raises(TypeError, match="mapping"):
        goal_mod.jtc_goal_from_trace(trace)


def test_positions_wrong_sample_count_rejected():
    trace = _FakeTrace(
        joint_names=("a",),
        times=(0.0, 1.0, 2.0),
        positions={"a": (0.0, 1.0)},
    )
    with pytest.raises(ValueError, match="samples"):
        goal_mod.jtc_goal_from_trace(trace)


def test_non_finite_position_rejected():
    trace = _FakeTrace(
        joint_names=("a",),
        times=(0.0, 1.0),
        positions={"a": (0.0, float("inf"))},
    )
    with pytest.raises(ValueError, match="finite"):
        goal_mod.jtc_goal_from_trace(trace)


def test_non_numeric_position_rejected():
    trace = _FakeTrace(
        joint_names=("a",),
        times=(0.0, 1.0),
        positions={"a": (0.0, "x")},
    )
    with pytest.raises(ValueError, match="numeric"):
        goal_mod.jtc_goal_from_trace(trace)


def test_negative_time_offset_rejected():
    trace = _simple_trace()
    with pytest.raises(ValueError, match=">= 0"):
        goal_mod.jtc_goal_from_trace(trace, time_offset_s=-0.001)


def test_non_finite_time_offset_rejected():
    trace = _simple_trace()
    with pytest.raises(ValueError, match="finite"):
        goal_mod.jtc_goal_from_trace(trace, time_offset_s=float("nan"))


def test_skip_initial_with_single_sample_rejected():
    trace = _FakeTrace(joint_names=("a",), times=(0.0,), positions={"a": (0.0,)})
    with pytest.raises(ValueError, match="at least 2 samples"):
        goal_mod.jtc_goal_from_trace(trace, skip_initial_sample=True)


# ---------------------------------------------------------------------------
# Interop with the three R2 command-trace producers
# ---------------------------------------------------------------------------


def test_interop_with_stage1_step_command():
    trace = stage1.step_command(
        joint_names=JOINT_NAMES,
        home_positions_rad=HOME,
        active_joint="elbow_joint",
        step_rad=math.radians(30.0),
        duration_s=1.0,
        dt_s=0.1,
    )
    g = goal_mod.jtc_goal_from_trace(trace)
    assert g.joint_names == JOINT_NAMES
    assert len(g.points) == len(trace.times)
    # First point matches home; last point's active joint is home + step.
    elbow_idx = JOINT_NAMES.index("elbow_joint")
    assert g.points[0].positions[elbow_idx] == pytest.approx(HOME[elbow_idx] + math.radians(30.0))
    # time_from_start of last point == duration_s
    assert g.points[-1].time_from_start_s == pytest.approx(1.0)


def test_interop_with_stage1_sine_command():
    trace = stage1.sine_command(
        joint_names=JOINT_NAMES,
        home_positions_rad=HOME,
        active_joint="shoulder_pan_joint",
        amplitude_rad=math.radians(10.0),
        frequency_hz=0.5,
        duration_s=2.0,
        dt_s=0.05,
    )
    g = goal_mod.jtc_goal_from_trace(trace, time_offset_s=0.05)
    assert g.joint_names == JOINT_NAMES
    assert len(g.points) == len(trace.times)
    assert g.points[0].time_from_start_s == pytest.approx(0.05)
    # Every non-active joint stays at home in every point.
    for name in JOINT_NAMES:
        if name == "shoulder_pan_joint":
            continue
        idx = JOINT_NAMES.index(name)
        for pt in g.points:
            assert pt.positions[idx] == pytest.approx(HOME[idx])


def test_interop_with_stage2_home_to_via_to_home():
    via = (0.5, -1.0, 1.0, -1.0, -1.0, 0.5)
    trace = stage2.home_to_pose_to_home_command(
        joint_names=JOINT_NAMES,
        home_positions_rad=HOME,
        via_pose_rad=via,
        duration_s=5.0,
        dt_s=0.1,
    )
    g = goal_mod.jtc_goal_from_trace(trace, skip_initial_sample=True)
    # Skipped the t=0 home sample; first remaining point is at dt.
    assert g.points[0].time_from_start_s == pytest.approx(0.0)
    # Last point is back at home.
    for i, name in enumerate(JOINT_NAMES):
        assert g.points[-1].positions[i] == pytest.approx(HOME[i])
    # Midpoint (before skip was at idx T/2; after skip it's one fewer
    # — find it explicitly).
    midpoint_time = (trace.duration_s / 2.0) - trace.times[1]
    closest = min(g.points, key=lambda p: abs(p.time_from_start_s - midpoint_time))
    for i, name in enumerate(JOINT_NAMES):
        assert closest.positions[i] == pytest.approx(via[i], abs=1e-9)


def test_interop_with_joints_from_tcp_output():
    tcp = stage3.line_trajectory(
        start_pos_m=(0.4, 0.0, 0.3),
        end_pos_m=(0.5, 0.0, 0.3),
        orientation_quat=(0.0, 0.0, 0.0, 1.0),
        duration_s=1.0,
        dt_s=0.2,
    )

    def ik(pos, quat, q_seed):
        # Deterministic, seed-independent mapping so we can check plumbing.
        return (pos[0], pos[1], pos[2], 0.0, 0.0, 0.0)

    traj = jft.joint_trajectory_from_tcp(tcp, ik, joint_names=JOINT_NAMES, q_seed=(0.0,) * 6)
    g = goal_mod.jtc_goal_from_trace(traj)
    assert g.joint_names == JOINT_NAMES
    assert len(g.points) == len(tcp.times)
    # First point's x == 0.4; last point's x == 0.5
    assert g.points[0].positions[0] == pytest.approx(0.4)
    assert g.points[-1].positions[0] == pytest.approx(0.5)
    # time_from_start forwarded from TCP trajectory.
    assert g.points[-1].time_from_start_s == pytest.approx(1.0)
