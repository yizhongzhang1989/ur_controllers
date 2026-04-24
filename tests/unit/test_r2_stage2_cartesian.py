"""Unit tests for tests/integration/r2_stage2_cartesian.py.

Covers the cartesian-mode R2 stage-2 evaluator that pairs with the
joint-space :mod:`r2_stage2_assertions` module. The evaluator folds
commanded + measured TCP trajectories (each a
``r2_stage3_commands.TcpTrajectory``, typically produced by
``r2_tcp_from_joints.tcp_trajectory_from_joints``) through the 5 mm +
2° stage-2 bound.

Pure stdlib; runs in the unit-test gate.
"""

from __future__ import annotations

import importlib.util
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
UUT_PATH = REPO_ROOT / "tests" / "integration" / "r2_stage2_cartesian.py"
CMD_PATH = REPO_ROOT / "tests" / "integration" / "r2_stage3_commands.py"
LOADER_PATH = REPO_ROOT / "tests" / "integration" / "expectations_loader.py"


def _load(mod_name: str, path: Path):
    spec = importlib.util.spec_from_file_location(mod_name, path)
    assert spec and spec.loader
    mod = importlib.util.module_from_spec(spec)
    sys.modules[mod_name] = mod
    spec.loader.exec_module(mod)
    return mod


uut = _load("r2_stage2_cartesian_uut", UUT_PATH)
cmds = _load("r2_stage3_commands_uut_for_s2c", CMD_PATH)
loader = _load("expectations_loader_uut_for_s2c", LOADER_PATH)


TcpTrajectory = cmds.TcpTrajectory


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


def _identity_trajectory(n: int = 11, dt: float = 0.1) -> TcpTrajectory:
    times = tuple(i * dt for i in range(n))
    positions = tuple((0.4, 0.0, 0.5) for _ in range(n))
    orientations = tuple((0.0, 0.0, 0.0, 1.0) for _ in range(n))
    return TcpTrajectory(times=times, positions=positions, orientations=orientations)


def _offset_positions(traj: TcpTrajectory, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0):
    return TcpTrajectory(
        times=traj.times,
        positions=tuple((x + dx, y + dy, z + dz) for (x, y, z) in traj.positions),
        orientations=traj.orientations,
    )


def _rotate_quat_about_z(traj: TcpTrajectory, angle_deg: float) -> TcpTrajectory:
    # Apply a constant rotation about Z to every sample's orientation.
    half = math.radians(angle_deg) / 2.0
    rz = (0.0, 0.0, math.sin(half), math.cos(half))
    rotated = []
    for x, y, z, w in traj.orientations:
        # Hamilton product: rz * q
        rx, ry, rzz, rw = rz
        nx = rw * x + rx * w + ry * z - rzz * y
        ny = rw * y - rx * z + ry * w + rzz * x
        nz = rw * z + rx * y - ry * x + rzz * w
        nw = rw * w - rx * x - ry * y - rzz * z
        rotated.append((nx, ny, nz, nw))
    return TcpTrajectory(
        times=traj.times,
        positions=traj.positions,
        orientations=tuple(rotated),
    )


# ---------------------------------------------------------------------------
# Export surface
# ---------------------------------------------------------------------------


def test_exports():
    assert set(uut.__all__) == {
        "Stage2CartesianResult",
        "evaluate_all_joints_cartesian",
        "evaluate_all_joints_cartesian_from_expectation",
    }


def test_result_is_frozen():
    r = uut.Stage2CartesianResult(controller="c")
    with pytest.raises(Exception):
        r.controller = "x"  # type: ignore[misc]


def test_result_ok_and_format():
    r = uut.Stage2CartesianResult(
        controller="cartesian_motion_controller",
        metrics={"position_peak_err_mm": 1.0, "orientation_peak_err_deg": 0.3},
    )
    assert r.ok is True
    assert "OK" in r.format()
    assert "position_peak_err_mm" in r.format()

    bad = uut.Stage2CartesianResult(controller="c", metrics={"x": 1.0}, failures=("foo",))
    assert bad.ok is False
    assert "FAIL" in bad.format()


# ---------------------------------------------------------------------------
# Happy-path evaluations
# ---------------------------------------------------------------------------


def test_identical_trajectories_pass():
    traj = _identity_trajectory()
    result = uut.evaluate_all_joints_cartesian(
        "cartesian_motion_controller",
        commanded_tcp=traj,
        measured_tcp=traj,
        position_peak_err_mm=5.0,
        orientation_peak_err_deg=2.0,
    )
    assert result.ok
    assert result.metrics["position_peak_err_mm"] == pytest.approx(0.0)
    assert result.metrics["orientation_peak_err_deg"] == pytest.approx(0.0)
    assert result.failures == ()


def test_antipodal_quaternion_pair_is_zero_error():
    traj = _identity_trajectory()
    flipped = TcpTrajectory(
        times=traj.times,
        positions=traj.positions,
        orientations=tuple((-x, -y, -z, -w) for (x, y, z, w) in traj.orientations),
    )
    result = uut.evaluate_all_joints_cartesian(
        "c",
        commanded_tcp=traj,
        measured_tcp=flipped,
        position_peak_err_mm=1.0,
        orientation_peak_err_deg=1.0,
    )
    assert result.ok
    assert result.metrics["orientation_peak_err_deg"] == pytest.approx(0.0, abs=1e-9)


def test_position_offset_measured_as_mm():
    traj = _identity_trajectory()
    offset = _offset_positions(traj, dx=0.003)  # 3 mm
    result = uut.evaluate_all_joints_cartesian(
        "c",
        commanded_tcp=traj,
        measured_tcp=offset,
        position_peak_err_mm=5.0,
        orientation_peak_err_deg=2.0,
    )
    assert result.ok
    assert result.metrics["position_peak_err_mm"] == pytest.approx(3.0, abs=1e-9)


def test_position_offset_beyond_tol_fails():
    traj = _identity_trajectory()
    offset = _offset_positions(traj, dz=0.007)  # 7 mm
    result = uut.evaluate_all_joints_cartesian(
        "c",
        commanded_tcp=traj,
        measured_tcp=offset,
        position_peak_err_mm=5.0,
        orientation_peak_err_deg=2.0,
    )
    assert not result.ok
    assert len(result.failures) == 1
    assert "position_peak_err_mm" in result.failures[0]
    assert result.metrics["position_peak_err_mm"] == pytest.approx(7.0, abs=1e-9)


def test_orientation_error_measured_in_degrees():
    traj = _identity_trajectory()
    rotated = _rotate_quat_about_z(traj, angle_deg=1.5)
    result = uut.evaluate_all_joints_cartesian(
        "c",
        commanded_tcp=traj,
        measured_tcp=rotated,
        position_peak_err_mm=5.0,
        orientation_peak_err_deg=2.0,
    )
    assert result.ok
    assert result.metrics["orientation_peak_err_deg"] == pytest.approx(1.5, abs=1e-6)


def test_orientation_error_beyond_tol_fails():
    traj = _identity_trajectory()
    rotated = _rotate_quat_about_z(traj, angle_deg=3.0)
    result = uut.evaluate_all_joints_cartesian(
        "c",
        commanded_tcp=traj,
        measured_tcp=rotated,
        position_peak_err_mm=5.0,
        orientation_peak_err_deg=2.0,
    )
    assert not result.ok
    assert len(result.failures) == 1
    assert "orientation_peak_err_deg" in result.failures[0]


def test_both_tolerances_violated_reports_both():
    traj = _identity_trajectory()
    bad = _offset_positions(_rotate_quat_about_z(traj, 5.0), dx=0.010)
    result = uut.evaluate_all_joints_cartesian(
        "c",
        commanded_tcp=traj,
        measured_tcp=bad,
        position_peak_err_mm=5.0,
        orientation_peak_err_deg=2.0,
    )
    assert not result.ok
    assert len(result.failures) == 2
    kinds = " ".join(result.failures)
    assert "position_peak_err_mm" in kinds
    assert "orientation_peak_err_deg" in kinds


def test_peak_taken_over_whole_trace():
    # Drift grows linearly to 8 mm at the final sample; peak must report 8 mm.
    traj = _identity_trajectory(n=9, dt=0.1)
    positions = tuple((0.4 + i * 0.001, 0.0, 0.5) for i in range(len(traj.times)))
    drifting = TcpTrajectory(times=traj.times, positions=positions, orientations=traj.orientations)
    result = uut.evaluate_all_joints_cartesian(
        "c",
        commanded_tcp=traj,
        measured_tcp=drifting,
        position_peak_err_mm=10.0,
        orientation_peak_err_deg=2.0,
    )
    assert result.ok
    assert result.metrics["position_peak_err_mm"] == pytest.approx(8.0, abs=1e-9)


def test_controller_name_echoed():
    traj = _identity_trajectory()
    result = uut.evaluate_all_joints_cartesian(
        "cartesian_motion_controller",
        commanded_tcp=traj,
        measured_tcp=traj,
        position_peak_err_mm=5.0,
        orientation_peak_err_deg=2.0,
    )
    assert result.controller == "cartesian_motion_controller"


# ---------------------------------------------------------------------------
# Validation: tolerances
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("val", [0.0, -1.0, float("nan"), float("inf")])
def test_bad_position_tol_rejected(val):
    traj = _identity_trajectory()
    with pytest.raises(ValueError):
        uut.evaluate_all_joints_cartesian(
            "c",
            commanded_tcp=traj,
            measured_tcp=traj,
            position_peak_err_mm=val,
            orientation_peak_err_deg=2.0,
        )


@pytest.mark.parametrize("val", [0.0, -1.0, float("nan"), float("inf")])
def test_bad_orientation_tol_rejected(val):
    traj = _identity_trajectory()
    with pytest.raises(ValueError):
        uut.evaluate_all_joints_cartesian(
            "c",
            commanded_tcp=traj,
            measured_tcp=traj,
            position_peak_err_mm=5.0,
            orientation_peak_err_deg=val,
        )


# ---------------------------------------------------------------------------
# Validation: trajectory shape / numeric contract
# ---------------------------------------------------------------------------


def test_missing_attributes_rejected():
    traj = _identity_trajectory()
    bad = SimpleNamespace(times=traj.times, positions=traj.positions)  # no orientations
    with pytest.raises(TypeError):
        uut.evaluate_all_joints_cartesian(
            "c",
            commanded_tcp=bad,
            measured_tcp=traj,
            position_peak_err_mm=5.0,
            orientation_peak_err_deg=2.0,
        )


def test_sample_count_mismatch_rejected():
    a = _identity_trajectory(n=10)
    b = _identity_trajectory(n=11)
    with pytest.raises(ValueError, match="does not resample"):
        uut.evaluate_all_joints_cartesian(
            "c",
            commanded_tcp=a,
            measured_tcp=b,
            position_peak_err_mm=5.0,
            orientation_peak_err_deg=2.0,
        )


def test_too_few_samples_rejected():
    a = SimpleNamespace(times=(0.0,), positions=((0, 0, 0),), orientations=((0, 0, 0, 1),))
    with pytest.raises(ValueError, match="length >= 2"):
        uut.evaluate_all_joints_cartesian(
            "c",
            commanded_tcp=a,
            measured_tcp=a,
            position_peak_err_mm=5.0,
            orientation_peak_err_deg=2.0,
        )


def test_non_monotonic_times_rejected():
    bad = SimpleNamespace(
        times=(0.0, 0.1, 0.05),
        positions=((0, 0, 0), (0, 0, 0), (0, 0, 0)),
        orientations=((0, 0, 0, 1), (0, 0, 0, 1), (0, 0, 0, 1)),
    )
    good = _identity_trajectory(n=3, dt=0.1)
    with pytest.raises(ValueError, match="strictly increasing"):
        uut.evaluate_all_joints_cartesian(
            "c",
            commanded_tcp=bad,
            measured_tcp=good,
            position_peak_err_mm=5.0,
            orientation_peak_err_deg=2.0,
        )


def test_non_finite_time_rejected():
    bad = SimpleNamespace(
        times=(0.0, float("nan")),
        positions=((0, 0, 0), (0, 0, 0)),
        orientations=((0, 0, 0, 1), (0, 0, 0, 1)),
    )
    good = _identity_trajectory(n=2, dt=0.1)
    with pytest.raises(ValueError, match="not finite"):
        uut.evaluate_all_joints_cartesian(
            "c",
            commanded_tcp=bad,
            measured_tcp=good,
            position_peak_err_mm=5.0,
            orientation_peak_err_deg=2.0,
        )


def test_position_wrong_length_rejected():
    traj = _identity_trajectory(n=3)
    bad = SimpleNamespace(
        times=traj.times,
        positions=((0.0, 0.0),) * 3,
        orientations=traj.orientations,
    )
    with pytest.raises(ValueError, match="3 "):
        uut.evaluate_all_joints_cartesian(
            "c",
            commanded_tcp=bad,
            measured_tcp=traj,
            position_peak_err_mm=5.0,
            orientation_peak_err_deg=2.0,
        )


def test_non_finite_position_rejected():
    traj = _identity_trajectory(n=3)
    bad = SimpleNamespace(
        times=traj.times,
        positions=(
            (0.0, 0.0, 0.0),
            (0.0, float("nan"), 0.0),
            (0.0, 0.0, 0.0),
        ),
        orientations=traj.orientations,
    )
    with pytest.raises(ValueError, match="is not finite"):
        uut.evaluate_all_joints_cartesian(
            "c",
            commanded_tcp=bad,
            measured_tcp=traj,
            position_peak_err_mm=5.0,
            orientation_peak_err_deg=2.0,
        )


def test_degenerate_quaternion_rejected():
    traj = _identity_trajectory(n=3)
    bad = SimpleNamespace(
        times=traj.times,
        positions=traj.positions,
        orientations=(
            (0.0, 0.0, 0.0, 1.0),
            (0.0, 0.0, 0.0, 0.0),  # zero quaternion
            (0.0, 0.0, 0.0, 1.0),
        ),
    )
    with pytest.raises(ValueError, match="degenerate quaternion"):
        uut.evaluate_all_joints_cartesian(
            "c",
            commanded_tcp=bad,
            measured_tcp=traj,
            position_peak_err_mm=5.0,
            orientation_peak_err_deg=2.0,
        )


def test_quaternion_wrong_length_rejected():
    traj = _identity_trajectory(n=2)
    bad = SimpleNamespace(
        times=traj.times,
        positions=traj.positions,
        orientations=((0.0, 0.0, 1.0), (0.0, 0.0, 1.0)),
    )
    with pytest.raises(ValueError, match="4 "):
        uut.evaluate_all_joints_cartesian(
            "c",
            commanded_tcp=bad,
            measured_tcp=traj,
            position_peak_err_mm=5.0,
            orientation_peak_err_deg=2.0,
        )


def test_positions_length_mismatch_rejected():
    traj = _identity_trajectory(n=3)
    bad = SimpleNamespace(
        times=traj.times,
        positions=((0.0, 0.0, 0.0),) * 2,  # wrong length
        orientations=traj.orientations,
    )
    with pytest.raises(ValueError, match="positions length"):
        uut.evaluate_all_joints_cartesian(
            "c",
            commanded_tcp=bad,
            measured_tcp=traj,
            position_peak_err_mm=5.0,
            orientation_peak_err_deg=2.0,
        )


# ---------------------------------------------------------------------------
# Expectation-driven wrapper
# ---------------------------------------------------------------------------


def test_from_expectation_pulls_tolerances_from_yaml():
    arm = loader.load_arm("ur5e")
    traj = _identity_trajectory()
    # 3 mm offset is within 5 mm; 1.5° rotation within 2°.
    bad_pos = _offset_positions(traj, dx=0.003)
    result = uut.evaluate_all_joints_cartesian_from_expectation(
        arm,
        "cartesian_motion_controller",
        commanded_tcp=traj,
        measured_tcp=bad_pos,
    )
    assert result.ok
    assert result.metrics["position_peak_err_mm"] == pytest.approx(3.0, abs=1e-9)


def test_from_expectation_fails_when_over_tolerance():
    arm = loader.load_arm("ur15")
    traj = _identity_trajectory()
    offset = _offset_positions(traj, dx=0.010)  # 10 mm > 5 mm
    result = uut.evaluate_all_joints_cartesian_from_expectation(
        arm,
        "cartesian_motion_controller",
        commanded_tcp=traj,
        measured_tcp=offset,
    )
    assert not result.ok
    assert any("position_peak_err_mm" in f for f in result.failures)


def test_from_expectation_duck_types_stage2_tcp():
    @dataclass(frozen=True)
    class _FakeStage2Tcp:
        position_peak_err_mm: float
        orientation_peak_err_deg: float

    @dataclass(frozen=True)
    class _FakeArm:
        stage2_tcp: _FakeStage2Tcp

    fake = _FakeArm(
        stage2_tcp=_FakeStage2Tcp(position_peak_err_mm=1.0, orientation_peak_err_deg=1.0)
    )
    traj = _identity_trajectory()
    offset = _offset_positions(traj, dx=0.002)  # 2 mm > 1 mm
    result = uut.evaluate_all_joints_cartesian_from_expectation(
        fake,
        "c",
        commanded_tcp=traj,
        measured_tcp=offset,
    )
    assert not result.ok
