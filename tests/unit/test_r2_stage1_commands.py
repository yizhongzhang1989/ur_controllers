"""Unit tests for tests/integration/r2_stage1_commands.py.

Covers the commanded joint-space trajectory generators pre-baked for
the M6.12 R2 stage-1 integration matrix. Pure stdlib — runs in the
unit gate.

The tests exercise each generator both in isolation (sample count,
endpoints, waveform properties, inactive joints held at home) and
against the sibling ``r2_stage1_assertions`` evaluators so a future
orchestrator feeding these traces into ``evaluate_position_mode`` /
``evaluate_second_order`` is guaranteed to meet their input contract.
"""

from __future__ import annotations

import importlib.util
import math
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
CMDS_PATH = REPO_ROOT / "tests" / "integration" / "r2_stage1_commands.py"
ASSERT_PATH = REPO_ROOT / "tests" / "integration" / "r2_stage1_assertions.py"


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec and spec.loader
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


cmds = _load("r2_stage1_commands_uut", CMDS_PATH)
r2s1 = _load("r2_stage1_assertions_for_cmds", ASSERT_PATH)


UR_JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
UR_HOME = (0.0, -1.57, 1.57, -1.57, -1.57, 0.0)


# ---------------------------------------------------------------------------
# Export surface / dataclass basics
# ---------------------------------------------------------------------------


def test_exports_public_surface():
    assert set(cmds.__all__) == {"JointCommandTrace", "step_command", "sine_command"}


def test_joint_command_trace_is_frozen_dataclass():
    trc = cmds.step_command(
        joint_names=UR_JOINTS,
        home_positions_rad=UR_HOME,
        active_joint="elbow_joint",
        step_rad=math.radians(30),
        duration_s=1.0,
        dt_s=0.01,
    )
    with pytest.raises(Exception):
        trc.times = (0.0,)  # type: ignore[misc]


def test_len_and_duration_properties():
    trc = cmds.step_command(
        joint_names=UR_JOINTS,
        home_positions_rad=UR_HOME,
        active_joint="elbow_joint",
        step_rad=math.radians(30),
        duration_s=2.0,
        dt_s=0.01,
    )
    assert len(trc) == 201
    assert trc.duration_s == pytest.approx(2.0, abs=1e-12)


# ---------------------------------------------------------------------------
# step_command — shape & semantics
# ---------------------------------------------------------------------------


def test_step_command_shape_and_endpoints():
    trc = cmds.step_command(
        joint_names=UR_JOINTS,
        home_positions_rad=UR_HOME,
        active_joint="elbow_joint",
        step_rad=math.radians(30),
        duration_s=1.0,
        dt_s=0.01,
    )
    assert trc.joint_names == UR_JOINTS
    assert len(trc.times) == 101
    assert trc.times[0] == 0.0
    assert trc.times[-1] == pytest.approx(1.0, abs=1e-12)
    for q_trace in trc.positions.values():
        assert len(q_trace) == len(trc.times)


def test_step_command_default_tstep_zero_entire_trace_at_target():
    trc = cmds.step_command(
        joint_names=UR_JOINTS,
        home_positions_rad=UR_HOME,
        active_joint="elbow_joint",
        step_rad=math.radians(30),
        duration_s=1.0,
        dt_s=0.01,
    )
    home_elbow = UR_HOME[UR_JOINTS.index("elbow_joint")]
    expected = home_elbow + math.radians(30)
    for q in trc.positions["elbow_joint"]:
        assert q == pytest.approx(expected, abs=1e-12)
    assert trc.target_rad == pytest.approx(expected, abs=1e-12)
    assert trc.active_joint == "elbow_joint"


def test_step_command_inactive_joints_held_at_home():
    trc = cmds.step_command(
        joint_names=UR_JOINTS,
        home_positions_rad=UR_HOME,
        active_joint="elbow_joint",
        step_rad=math.radians(30),
        duration_s=1.0,
        dt_s=0.01,
    )
    for name, home in zip(UR_JOINTS, UR_HOME):
        if name == "elbow_joint":
            continue
        for q in trc.positions[name]:
            assert q == home


def test_step_command_tstep_midway():
    trc = cmds.step_command(
        joint_names=UR_JOINTS,
        home_positions_rad=UR_HOME,
        active_joint="shoulder_pan_joint",
        step_rad=math.radians(30),
        duration_s=2.0,
        dt_s=0.01,
        t_step_s=1.0,
    )
    home_active = UR_HOME[0]
    target = home_active + math.radians(30)
    ts = trc.times
    qs = trc.positions["shoulder_pan_joint"]
    for t, q in zip(ts, qs):
        if t < 1.0:
            assert q == pytest.approx(home_active, abs=1e-12)
        else:
            assert q == pytest.approx(target, abs=1e-12)


def test_step_command_negative_step_supported():
    trc = cmds.step_command(
        joint_names=UR_JOINTS,
        home_positions_rad=UR_HOME,
        active_joint="wrist_3_joint",
        step_rad=math.radians(-30),
        duration_s=0.5,
        dt_s=0.01,
    )
    home_active = UR_HOME[-1]
    assert trc.target_rad == pytest.approx(home_active - math.radians(30))
    for q in trc.positions["wrist_3_joint"]:
        assert q == pytest.approx(home_active - math.radians(30), abs=1e-12)


def test_step_command_positions_mapping_is_readonly():
    trc = cmds.step_command(
        joint_names=UR_JOINTS,
        home_positions_rad=UR_HOME,
        active_joint="elbow_joint",
        step_rad=0.1,
        duration_s=1.0,
        dt_s=0.01,
    )
    with pytest.raises(TypeError):
        trc.positions["new_joint"] = (0.0,)  # type: ignore[index]


# ---------------------------------------------------------------------------
# sine_command — shape & semantics
# ---------------------------------------------------------------------------


def test_sine_command_shape_and_endpoints():
    trc = cmds.sine_command(
        joint_names=UR_JOINTS,
        home_positions_rad=UR_HOME,
        active_joint="wrist_1_joint",
        amplitude_rad=0.1,
        frequency_hz=0.5,
        duration_s=2.0,
        dt_s=0.01,
    )
    assert len(trc) == 201
    assert trc.times[0] == 0.0
    assert trc.times[-1] == pytest.approx(2.0, abs=1e-12)


def test_sine_command_waveform_matches_sine():
    home_active = UR_HOME[UR_JOINTS.index("wrist_1_joint")]
    amp = 0.1
    freq = 0.5
    trc = cmds.sine_command(
        joint_names=UR_JOINTS,
        home_positions_rad=UR_HOME,
        active_joint="wrist_1_joint",
        amplitude_rad=amp,
        frequency_hz=freq,
        duration_s=4.0,
        dt_s=0.01,
    )
    omega = 2.0 * math.pi * freq
    for t, q in zip(trc.times, trc.positions["wrist_1_joint"]):
        expected = home_active + amp * math.sin(omega * t)
        assert q == pytest.approx(expected, abs=1e-12)
    assert trc.positions["wrist_1_joint"][0] == pytest.approx(home_active, abs=1e-12)
    assert trc.target_rad == pytest.approx(home_active, abs=1e-12)


def test_sine_command_phase_offset_applied():
    home_active = UR_HOME[UR_JOINTS.index("wrist_1_joint")]
    amp = 0.1
    trc = cmds.sine_command(
        joint_names=UR_JOINTS,
        home_positions_rad=UR_HOME,
        active_joint="wrist_1_joint",
        amplitude_rad=amp,
        frequency_hz=0.5,
        duration_s=2.0,
        dt_s=0.01,
        phase_rad=math.pi / 2,  # sine -> cosine; starts at +amp
    )
    assert trc.positions["wrist_1_joint"][0] == pytest.approx(home_active + amp, abs=1e-12)


def test_sine_command_zero_amplitude_is_constant():
    trc = cmds.sine_command(
        joint_names=UR_JOINTS,
        home_positions_rad=UR_HOME,
        active_joint="elbow_joint",
        amplitude_rad=0.0,
        frequency_hz=0.5,
        duration_s=1.0,
        dt_s=0.01,
    )
    home_active = UR_HOME[UR_JOINTS.index("elbow_joint")]
    for q in trc.positions["elbow_joint"]:
        assert q == pytest.approx(home_active, abs=1e-12)


def test_sine_command_inactive_joints_held_at_home():
    trc = cmds.sine_command(
        joint_names=UR_JOINTS,
        home_positions_rad=UR_HOME,
        active_joint="elbow_joint",
        amplitude_rad=0.1,
        frequency_hz=0.5,
        duration_s=1.0,
        dt_s=0.01,
    )
    for name, home in zip(UR_JOINTS, UR_HOME):
        if name == "elbow_joint":
            continue
        for q in trc.positions[name]:
            assert q == home


# ---------------------------------------------------------------------------
# Interop with r2_stage1_assertions.evaluate_position_mode
# ---------------------------------------------------------------------------


class _FakeControllerExp:
    """Minimal duck-typed stand-in for ControllerExpectation.

    Exposes the ``name`` attribute and ``tol(key)`` method that
    ``r2_stage1_assertions.evaluate_position_mode`` reads — enough
    to exercise the interop without importing the loader.
    """

    def __init__(self, name: str, tols):
        self.name = name
        self._tols = dict(tols)

    def tol(self, key: str) -> float:
        return self._tols[key]


def test_step_command_feeds_position_mode_evaluator():
    home_active = UR_HOME[UR_JOINTS.index("elbow_joint")]
    trc = cmds.step_command(
        joint_names=UR_JOINTS,
        home_positions_rad=UR_HOME,
        active_joint="elbow_joint",
        step_rad=math.radians(30),
        duration_s=12.0,
        dt_s=0.01,
    )
    # Perfectly tracked measured trace: measurement == commanded.
    ts = list(trc.times)
    qs = list(trc.positions["elbow_joint"])
    qdots = [0.0] * len(ts)
    c = _FakeControllerExp(
        "joint_trajectory_controller",
        {
            "steady_state_err_rad": 0.01,
            "drift_rad_per_10s": 0.01,
            "fft_peak_db_above_noise_floor": 20.0,
        },
    )
    result = r2s1.evaluate_position_mode(
        c,
        joint="elbow_joint",
        times=ts,
        positions=qs,
        velocities=qdots,
        target=trc.target_rad,
    )
    assert result.failures == (), result.format()
    # target_rad matches the scalar target the evaluator was fed.
    assert trc.target_rad == pytest.approx(home_active + math.radians(30))


# ---------------------------------------------------------------------------
# Validation — step_command
# ---------------------------------------------------------------------------


def test_step_command_rejects_empty_joint_names():
    with pytest.raises(ValueError, match="joint_names must be non-empty"):
        cmds.step_command(
            joint_names=(),
            home_positions_rad=(),
            active_joint="x",
            step_rad=0.1,
            duration_s=1.0,
            dt_s=0.01,
        )


def test_step_command_rejects_duplicate_joint_names():
    with pytest.raises(ValueError, match="duplicates"):
        cmds.step_command(
            joint_names=("a", "b", "a"),
            home_positions_rad=(0.0, 0.0, 0.0),
            active_joint="a",
            step_rad=0.1,
            duration_s=1.0,
            dt_s=0.01,
        )


def test_step_command_rejects_home_length_mismatch():
    with pytest.raises(ValueError, match="same length as joint_names"):
        cmds.step_command(
            joint_names=UR_JOINTS,
            home_positions_rad=(0.0, 0.0, 0.0),
            active_joint="elbow_joint",
            step_rad=0.1,
            duration_s=1.0,
            dt_s=0.01,
        )


def test_step_command_rejects_nonfinite_home():
    with pytest.raises(ValueError, match="not finite"):
        cmds.step_command(
            joint_names=UR_JOINTS,
            home_positions_rad=(0.0, float("nan"), 0.0, 0.0, 0.0, 0.0),
            active_joint="elbow_joint",
            step_rad=0.1,
            duration_s=1.0,
            dt_s=0.01,
        )


def test_step_command_rejects_active_joint_not_in_names():
    with pytest.raises(ValueError, match="not in joint_names"):
        cmds.step_command(
            joint_names=UR_JOINTS,
            home_positions_rad=UR_HOME,
            active_joint="no_such_joint",
            step_rad=0.1,
            duration_s=1.0,
            dt_s=0.01,
        )


def test_step_command_rejects_nonfinite_step():
    with pytest.raises(ValueError, match="step_rad"):
        cmds.step_command(
            joint_names=UR_JOINTS,
            home_positions_rad=UR_HOME,
            active_joint="elbow_joint",
            step_rad=float("inf"),
            duration_s=1.0,
            dt_s=0.01,
        )


def test_step_command_rejects_nonpositive_duration():
    with pytest.raises(ValueError, match="duration_s"):
        cmds.step_command(
            joint_names=UR_JOINTS,
            home_positions_rad=UR_HOME,
            active_joint="elbow_joint",
            step_rad=0.1,
            duration_s=0.0,
            dt_s=0.01,
        )


def test_step_command_rejects_nonfinite_duration():
    with pytest.raises(ValueError, match="duration_s"):
        cmds.step_command(
            joint_names=UR_JOINTS,
            home_positions_rad=UR_HOME,
            active_joint="elbow_joint",
            step_rad=0.1,
            duration_s=float("inf"),
            dt_s=0.01,
        )


def test_step_command_rejects_dt_below_min():
    with pytest.raises(ValueError, match="dt_s"):
        cmds.step_command(
            joint_names=UR_JOINTS,
            home_positions_rad=UR_HOME,
            active_joint="elbow_joint",
            step_rad=0.1,
            duration_s=1.0,
            dt_s=1e-9,
        )


def test_step_command_rejects_dt_exceeds_duration():
    with pytest.raises(ValueError, match="exceeds duration_s"):
        cmds.step_command(
            joint_names=UR_JOINTS,
            home_positions_rad=UR_HOME,
            active_joint="elbow_joint",
            step_rad=0.1,
            duration_s=0.01,
            dt_s=0.1,
        )


def test_step_command_rejects_tstep_out_of_range_negative():
    with pytest.raises(ValueError, match="t_step_s"):
        cmds.step_command(
            joint_names=UR_JOINTS,
            home_positions_rad=UR_HOME,
            active_joint="elbow_joint",
            step_rad=0.1,
            duration_s=1.0,
            dt_s=0.01,
            t_step_s=-0.5,
        )


def test_step_command_rejects_tstep_out_of_range_above_duration():
    with pytest.raises(ValueError, match="t_step_s"):
        cmds.step_command(
            joint_names=UR_JOINTS,
            home_positions_rad=UR_HOME,
            active_joint="elbow_joint",
            step_rad=0.1,
            duration_s=1.0,
            dt_s=0.01,
            t_step_s=1.5,
        )


def test_step_command_rejects_nonfinite_tstep():
    with pytest.raises(ValueError, match="t_step_s"):
        cmds.step_command(
            joint_names=UR_JOINTS,
            home_positions_rad=UR_HOME,
            active_joint="elbow_joint",
            step_rad=0.1,
            duration_s=1.0,
            dt_s=0.01,
            t_step_s=float("nan"),
        )


# ---------------------------------------------------------------------------
# Validation — sine_command
# ---------------------------------------------------------------------------


def test_sine_command_rejects_nonpositive_frequency():
    with pytest.raises(ValueError, match="frequency_hz must be > 0"):
        cmds.sine_command(
            joint_names=UR_JOINTS,
            home_positions_rad=UR_HOME,
            active_joint="elbow_joint",
            amplitude_rad=0.1,
            frequency_hz=0.0,
            duration_s=1.0,
            dt_s=0.01,
        )


def test_sine_command_rejects_nonfinite_frequency():
    with pytest.raises(ValueError, match="frequency_hz"):
        cmds.sine_command(
            joint_names=UR_JOINTS,
            home_positions_rad=UR_HOME,
            active_joint="elbow_joint",
            amplitude_rad=0.1,
            frequency_hz=float("inf"),
            duration_s=1.0,
            dt_s=0.01,
        )


def test_sine_command_rejects_nonfinite_amplitude():
    with pytest.raises(ValueError, match="amplitude_rad"):
        cmds.sine_command(
            joint_names=UR_JOINTS,
            home_positions_rad=UR_HOME,
            active_joint="elbow_joint",
            amplitude_rad=float("nan"),
            frequency_hz=0.5,
            duration_s=1.0,
            dt_s=0.01,
        )


def test_sine_command_rejects_nonfinite_phase():
    with pytest.raises(ValueError, match="phase_rad"):
        cmds.sine_command(
            joint_names=UR_JOINTS,
            home_positions_rad=UR_HOME,
            active_joint="elbow_joint",
            amplitude_rad=0.1,
            frequency_hz=0.5,
            duration_s=1.0,
            dt_s=0.01,
            phase_rad=float("inf"),
        )


def test_sine_command_rejects_active_joint_not_in_names():
    with pytest.raises(ValueError, match="not in joint_names"):
        cmds.sine_command(
            joint_names=UR_JOINTS,
            home_positions_rad=UR_HOME,
            active_joint="bogus",
            amplitude_rad=0.1,
            frequency_hz=0.5,
            duration_s=1.0,
            dt_s=0.01,
        )


def test_sine_command_rejects_home_length_mismatch():
    with pytest.raises(ValueError, match="same length as joint_names"):
        cmds.sine_command(
            joint_names=UR_JOINTS,
            home_positions_rad=(0.0, 0.0),
            active_joint="elbow_joint",
            amplitude_rad=0.1,
            frequency_hz=0.5,
            duration_s=1.0,
            dt_s=0.01,
        )


def test_sine_command_rejects_duplicate_joint_names():
    with pytest.raises(ValueError, match="duplicates"):
        cmds.sine_command(
            joint_names=("a", "a"),
            home_positions_rad=(0.0, 0.0),
            active_joint="a",
            amplitude_rad=0.1,
            frequency_hz=0.5,
            duration_s=1.0,
            dt_s=0.01,
        )
