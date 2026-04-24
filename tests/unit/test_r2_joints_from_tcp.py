"""Unit tests for tests/integration/r2_joints_from_tcp.py.

Covers the IK-injected adapter that converts a commanded
:class:`TcpTrajectory` into a joint-space command trajectory. Pure
stdlib — runs in the unit gate.

Tests use a dummy IK fixture so the module under test stays orthogonal
to whichever IK backend the operator eventually picks. The dummy IK
does not pretend to be kinematically accurate; it only has to satisfy
the adapter's contract so the adapter's validation + plumbing code can
be exercised.
"""

from __future__ import annotations

import importlib.util
import math
import sys
from pathlib import Path
from types import MappingProxyType

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
ADAPTER_PATH = REPO_ROOT / "tests" / "integration" / "r2_joints_from_tcp.py"
CMDS_PATH = REPO_ROOT / "tests" / "integration" / "r2_stage3_commands.py"


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec and spec.loader
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


adapter = _load("r2_joints_from_tcp_uut", ADAPTER_PATH)
cmds = _load("r2_stage3_commands_for_jft", CMDS_PATH)


JOINT_NAMES = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
ZERO_SEED = (0.0,) * 6
IDENTITY_QUAT = (0.0, 0.0, 0.0, 1.0)


def _seed_echo_ik(pos, quat, q_seed):
    """IK: just echo the seed back. Confirms seed-threading works."""
    return tuple(q_seed)


def _position_encoding_ik(pos, quat, q_seed):
    """IK: joint_i = pos.x + i*0.1 (no seed dependency).

    Lets us verify that TCP samples are forwarded verbatim into the IK
    call.
    """
    return tuple(pos[0] + 0.1 * i for i in range(len(q_seed)))


def _counter_ik_factory():
    """Returns an IK that increments seed[0] by 1 every call.

    Used to verify that each sample is seeded with the *previous*
    sample's output, not with the original caller-supplied seed.
    """

    def ik(pos, quat, q_seed):
        out = list(q_seed)
        out[0] = q_seed[0] + 1.0
        return out

    return ik


def _straight_line_trajectory(n=5, duration_s=1.0):
    """Build a simple 5-sample line trajectory for reuse."""
    dt = duration_s / (n - 1)
    return cmds.line_trajectory(
        start_pos_m=(0.3, 0.0, 0.4),
        end_pos_m=(0.5, 0.0, 0.4),
        orientation_quat=IDENTITY_QUAT,
        duration_s=duration_s,
        dt_s=dt,
    )


# ---------------------------------------------------------------------------
# 1. Export surface
# ---------------------------------------------------------------------------


def test_exports():
    assert set(adapter.__all__) == {
        "IkCallable",
        "JointCommandTrajectory",
        "TcpTrajectory",
        "joint_trajectory_from_tcp",
    }
    # The re-exported TcpTrajectory has the same attribute surface as
    # the stage-3 commands dataclass. (Object identity is not
    # asserted — tests/integration/ is not a package, so the adapter
    # loads TcpTrajectory through its own sys.modules key; the test
    # harness loads a sibling one. Duck-typing is what matters at the
    # API boundary.)
    import dataclasses

    fields = {f.name for f in dataclasses.fields(adapter.TcpTrajectory)}
    assert {"times", "positions", "orientations"} <= fields


# ---------------------------------------------------------------------------
# 2. Result dataclass properties
# ---------------------------------------------------------------------------


def test_result_is_frozen_dataclass():
    traj = _straight_line_trajectory()
    out = adapter.joint_trajectory_from_tcp(
        traj, _seed_echo_ik, joint_names=JOINT_NAMES, q_seed=ZERO_SEED
    )
    with pytest.raises((AttributeError, Exception)):
        out.times = (0.0,)


def test_len_and_duration_match_input():
    traj = _straight_line_trajectory(n=7, duration_s=1.2)
    out = adapter.joint_trajectory_from_tcp(
        traj, _seed_echo_ik, joint_names=JOINT_NAMES, q_seed=ZERO_SEED
    )
    assert len(out) == len(traj.times) == 7
    assert out.duration_s == pytest.approx(1.2)


def test_times_forwarded_verbatim():
    traj = _straight_line_trajectory()
    out = adapter.joint_trajectory_from_tcp(
        traj, _seed_echo_ik, joint_names=JOINT_NAMES, q_seed=ZERO_SEED
    )
    assert out.times == tuple(traj.times)
    assert out.times[0] == 0.0


def test_positions_mapping_is_read_only():
    traj = _straight_line_trajectory()
    out = adapter.joint_trajectory_from_tcp(
        traj, _seed_echo_ik, joint_names=JOINT_NAMES, q_seed=ZERO_SEED
    )
    assert isinstance(out.positions, MappingProxyType)
    with pytest.raises(TypeError):
        out.positions["shoulder_pan_joint"] = (1.0,)  # type: ignore[index]


def test_positions_keyed_by_joint_names_and_correct_length():
    traj = _straight_line_trajectory()
    out = adapter.joint_trajectory_from_tcp(
        traj, _seed_echo_ik, joint_names=JOINT_NAMES, q_seed=ZERO_SEED
    )
    assert tuple(out.positions.keys()) == JOINT_NAMES
    for name in JOINT_NAMES:
        assert len(out.positions[name]) == len(traj.times)


def test_duration_s_on_degenerate_single_sample_dataclass():
    # Direct dataclass construction — single-sample trace reports 0 s.
    bare = adapter.JointCommandTrajectory(
        joint_names=("a",),
        times=(0.0,),
        positions=MappingProxyType({"a": (0.1,)}),
    )
    assert len(bare) == 1
    assert bare.duration_s == 0.0


# ---------------------------------------------------------------------------
# 3. IK plumbing: seed threading, value forwarding
# ---------------------------------------------------------------------------


def test_identity_seed_echo_produces_constant_joint_streams():
    traj = _straight_line_trajectory()
    seed = (0.1, 0.2, 0.3, 0.4, 0.5, 0.6)
    out = adapter.joint_trajectory_from_tcp(
        traj, _seed_echo_ik, joint_names=JOINT_NAMES, q_seed=seed
    )
    for j, name in enumerate(JOINT_NAMES):
        assert all(math.isclose(v, seed[j]) for v in out.positions[name])


def test_ik_receives_tcp_position_verbatim():
    traj = _straight_line_trajectory(n=5, duration_s=1.0)
    out = adapter.joint_trajectory_from_tcp(
        traj, _position_encoding_ik, joint_names=JOINT_NAMES, q_seed=ZERO_SEED
    )
    # Joint 0 should equal pos.x of each sample; joint 1 = pos.x + 0.1 etc.
    for i, pos in enumerate(traj.positions):
        assert out.positions["shoulder_pan_joint"][i] == pytest.approx(pos[0])
        assert out.positions["shoulder_lift_joint"][i] == pytest.approx(pos[0] + 0.1)


def test_seed_threaded_from_previous_output_not_from_initial():
    traj = _straight_line_trajectory(n=5)
    ik = _counter_ik_factory()
    out = adapter.joint_trajectory_from_tcp(traj, ik, joint_names=JOINT_NAMES, q_seed=ZERO_SEED)
    # joint 0 should ramp 1, 2, 3, 4, 5 across the five samples — each
    # sample picks up the previous output's +1, never re-using the
    # caller's zero seed.
    assert out.positions["shoulder_pan_joint"] == (1.0, 2.0, 3.0, 4.0, 5.0)


def test_ik_receives_quaternion_verbatim():
    seen_quats = []

    def ik(pos, quat, q_seed):
        seen_quats.append(quat)
        return tuple(q_seed)

    traj = _straight_line_trajectory()
    adapter.joint_trajectory_from_tcp(traj, ik, joint_names=JOINT_NAMES, q_seed=ZERO_SEED)
    assert len(seen_quats) == len(traj.times)
    for q in seen_quats:
        assert q == IDENTITY_QUAT


def test_ik_seed_argument_length_matches_joint_names():
    # Three joints, seed of three — any ik that returns len(seed) works.
    traj = _straight_line_trajectory()
    names = ("a", "b", "c")
    seeds_seen = []

    def ik(pos, quat, q_seed):
        seeds_seen.append(tuple(q_seed))
        return (0.0, 0.0, 0.0)

    adapter.joint_trajectory_from_tcp(traj, ik, joint_names=names, q_seed=(0.1, 0.2, 0.3))
    # First seed is the caller-supplied; subsequent seeds are prior output.
    assert seeds_seen[0] == (0.1, 0.2, 0.3)
    for s in seeds_seen[1:]:
        assert s == (0.0, 0.0, 0.0)
        assert len(s) == len(names)


# ---------------------------------------------------------------------------
# 4. Input validation — trajectory
# ---------------------------------------------------------------------------


def test_rejects_non_TcpTrajectory_input():
    class Fake:
        # Missing `orientations` — duck-type check should fail.
        times = (0.0, 1.0)
        positions = ((0.0, 0.0, 0.0), (0.1, 0.0, 0.0))

    with pytest.raises(TypeError, match="TcpTrajectory-shaped"):
        adapter.joint_trajectory_from_tcp(
            Fake(), _seed_echo_ik, joint_names=JOINT_NAMES, q_seed=ZERO_SEED
        )


# ---------------------------------------------------------------------------
# 5. Input validation — joint_names
# ---------------------------------------------------------------------------


def test_rejects_empty_joint_names():
    traj = _straight_line_trajectory()
    with pytest.raises(ValueError, match="joint_names must be non-empty"):
        adapter.joint_trajectory_from_tcp(traj, _seed_echo_ik, joint_names=(), q_seed=())


def test_rejects_duplicate_joint_names():
    traj = _straight_line_trajectory()
    with pytest.raises(ValueError, match="duplicates"):
        adapter.joint_trajectory_from_tcp(
            traj,
            _seed_echo_ik,
            joint_names=("a", "b", "a"),
            q_seed=(0.0, 0.0, 0.0),
        )


# ---------------------------------------------------------------------------
# 6. Input validation — q_seed
# ---------------------------------------------------------------------------


def test_rejects_seed_length_mismatch():
    traj = _straight_line_trajectory()
    with pytest.raises(ValueError, match="q_seed length"):
        adapter.joint_trajectory_from_tcp(
            traj, _seed_echo_ik, joint_names=JOINT_NAMES, q_seed=(0.0, 0.0)
        )


@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf])
def test_rejects_non_finite_seed(bad):
    traj = _straight_line_trajectory()
    seed = (0.0, 0.0, 0.0, 0.0, 0.0, bad)
    with pytest.raises(ValueError, match="q_seed\\[5\\] is not finite"):
        adapter.joint_trajectory_from_tcp(traj, _seed_echo_ik, joint_names=JOINT_NAMES, q_seed=seed)


def test_rejects_non_numeric_seed():
    traj = _straight_line_trajectory()
    with pytest.raises(ValueError, match="q_seed\\[0\\] is not numeric"):
        adapter.joint_trajectory_from_tcp(
            traj,
            _seed_echo_ik,
            joint_names=JOINT_NAMES,
            q_seed=("not_a_number", 0.0, 0.0, 0.0, 0.0, 0.0),  # type: ignore[arg-type]
        )


# ---------------------------------------------------------------------------
# 7. IK output validation
# ---------------------------------------------------------------------------


def test_wraps_ik_exception_with_sample_context():
    traj = _straight_line_trajectory(n=4)

    def ik(pos, quat, q_seed):
        if pos[0] > 0.35:  # fires at sample index 1 on the 0.3..0.5 line
            raise RuntimeError("backend failure")
        return tuple(q_seed)

    with pytest.raises(ValueError, match="ik raised at sample 1"):
        adapter.joint_trajectory_from_tcp(traj, ik, joint_names=JOINT_NAMES, q_seed=ZERO_SEED)


def test_rejects_ik_returning_none():
    traj = _straight_line_trajectory()

    def ik(pos, quat, q_seed):
        return None

    with pytest.raises(ValueError, match="returned None"):
        adapter.joint_trajectory_from_tcp(traj, ik, joint_names=JOINT_NAMES, q_seed=ZERO_SEED)


def test_rejects_ik_returning_non_sequence():
    traj = _straight_line_trajectory()

    def ik(pos, quat, q_seed):
        return 42  # int is not iterable

    with pytest.raises(ValueError, match="did not return a sequence"):
        adapter.joint_trajectory_from_tcp(traj, ik, joint_names=JOINT_NAMES, q_seed=ZERO_SEED)


def test_rejects_ik_returning_wrong_length():
    traj = _straight_line_trajectory()

    def ik(pos, quat, q_seed):
        return (0.0, 0.0, 0.0)  # three values for a six-joint config

    with pytest.raises(ValueError, match="returned 3 joint values, expected 6"):
        adapter.joint_trajectory_from_tcp(traj, ik, joint_names=JOINT_NAMES, q_seed=ZERO_SEED)


@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf])
def test_rejects_ik_returning_non_finite(bad):
    traj = _straight_line_trajectory()

    def ik(pos, quat, q_seed):
        out = list(q_seed)
        out[2] = bad
        return out

    with pytest.raises(ValueError, match="\\[2\\] is not finite"):
        adapter.joint_trajectory_from_tcp(traj, ik, joint_names=JOINT_NAMES, q_seed=ZERO_SEED)


def test_rejects_ik_returning_non_numeric():
    traj = _straight_line_trajectory()

    def ik(pos, quat, q_seed):
        return ("a", 0.0, 0.0, 0.0, 0.0, 0.0)

    with pytest.raises(ValueError, match="\\[0\\] is not numeric"):
        adapter.joint_trajectory_from_tcp(traj, ik, joint_names=JOINT_NAMES, q_seed=ZERO_SEED)


# ---------------------------------------------------------------------------
# 8. Interop with stage-3 trajectory generators
# ---------------------------------------------------------------------------


def test_interop_with_arc_trajectory():
    """Full pipe: arc TCP trajectory -> IK -> JointCommandTrajectory."""
    traj = cmds.arc_trajectory(
        center_m=(0.4, 0.0, 0.4),
        radius_m=0.1,
        plane="xy",
        start_angle_rad=0.0,
        end_angle_rad=math.pi / 2,
        duration_s=1.0,
        dt_s=0.1,
    )
    out = adapter.joint_trajectory_from_tcp(
        traj, _position_encoding_ik, joint_names=JOINT_NAMES, q_seed=ZERO_SEED
    )
    assert len(out) == len(traj.times)
    # First sample: pos.x = center.x + radius*cos(0) = 0.5 → joint 0 = 0.5
    assert out.positions["shoulder_pan_joint"][0] == pytest.approx(0.5)
    # Last sample: pos.x = center.x + radius*cos(π/2) = 0.4
    assert out.positions["shoulder_pan_joint"][-1] == pytest.approx(0.4)


def test_interop_with_sine_in_z_trajectory():
    traj = cmds.sine_in_z_trajectory(
        base_pos_m=(0.4, 0.0, 0.4),
        amplitude_m=0.05,
        frequency_hz=1.0,
        duration_s=1.0,
        dt_s=0.1,
    )
    out = adapter.joint_trajectory_from_tcp(
        traj, _seed_echo_ik, joint_names=JOINT_NAMES, q_seed=ZERO_SEED
    )
    assert len(out) == len(traj.times)
    assert out.times == tuple(traj.times)
