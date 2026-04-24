"""Unit tests for tests/integration/r2_stage3_commands.py.

Covers the commanded TCP trajectory generators pre-baked for the
M6.14 integration test matrix. Pure stdlib — runs in the unit gate.

We sanity-check each generator both in isolation (sample count,
endpoints, waveform properties) and against the sibling
``r2_stage3_assertions.evaluate_tcp_trajectory`` evaluator so that a
future orchestrator feeding these trajectories into the evaluator is
guaranteed to meet its input contract.
"""

from __future__ import annotations

import importlib.util
import math
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
CMDS_PATH = REPO_ROOT / "tests" / "integration" / "r2_stage3_commands.py"
ASSERT_PATH = REPO_ROOT / "tests" / "integration" / "r2_stage3_assertions.py"


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec and spec.loader
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


cmds = _load("r2_stage3_commands_uut", CMDS_PATH)
r2s3 = _load("r2_stage3_assertions_for_cmds", ASSERT_PATH)


IDENTITY_Q = (0.0, 0.0, 0.0, 1.0)


# ---------------------------------------------------------------------------
# line_trajectory
# ---------------------------------------------------------------------------


class TestLineTrajectory:
    def test_sample_count_and_endpoints(self):
        traj = cmds.line_trajectory(
            start_pos_m=(0.0, 0.0, 0.0),
            end_pos_m=(1.0, 2.0, 3.0),
            duration_s=1.0,
            dt_s=0.1,
        )
        # 10 intervals → 11 samples.
        assert len(traj) == 11
        assert traj.times[0] == pytest.approx(0.0, abs=1e-12)
        assert traj.times[-1] == pytest.approx(1.0, abs=1e-12)
        assert traj.positions[0] == pytest.approx((0.0, 0.0, 0.0), abs=1e-12)
        assert traj.positions[-1] == pytest.approx((1.0, 2.0, 3.0), abs=1e-12)

    def test_monotonic_times(self):
        traj = cmds.line_trajectory(
            start_pos_m=(0.0, 0.0, 0.0),
            end_pos_m=(1.0, 0.0, 0.0),
            duration_s=0.5,
            dt_s=0.01,
        )
        for i in range(1, len(traj)):
            assert traj.times[i] > traj.times[i - 1]

    def test_linear_interpolation(self):
        traj = cmds.line_trajectory(
            start_pos_m=(0.0, 0.0, 0.0),
            end_pos_m=(2.0, 0.0, 0.0),
            duration_s=1.0,
            dt_s=0.25,
        )
        # At t=0.5s (midpoint) expect x=1.0.
        mid = len(traj) // 2
        assert traj.times[mid] == pytest.approx(0.5, abs=1e-12)
        assert traj.positions[mid][0] == pytest.approx(1.0, abs=1e-12)

    def test_orientation_preserved_and_normalized(self):
        q_in = (0.0, 0.0, 1.2, 0.0)  # norm = 1.2, points along +z rotation 180°
        traj = cmds.line_trajectory(
            start_pos_m=(0.0, 0.0, 0.0),
            end_pos_m=(1.0, 0.0, 0.0),
            duration_s=0.5,
            dt_s=0.1,
            orientation_quat=q_in,
        )
        for q in traj.orientations:
            norm = math.sqrt(sum(c * c for c in q))
            assert norm == pytest.approx(1.0, abs=1e-12)
            assert q == pytest.approx((0.0, 0.0, 1.0, 0.0), abs=1e-12)

    def test_default_orientation_is_identity(self):
        traj = cmds.line_trajectory(
            start_pos_m=(0.0, 0.0, 0.0),
            end_pos_m=(0.1, 0.0, 0.0),
            duration_s=0.1,
            dt_s=0.05,
        )
        assert all(q == IDENTITY_Q for q in traj.orientations)

    @pytest.mark.parametrize(
        "duration_s,dt_s",
        [(0.0, 0.1), (-1.0, 0.1), (float("nan"), 0.1), (1.0, 0.0), (1.0, -0.01), (1.0, 2.0)],
    )
    def test_rejects_bad_timing(self, duration_s, dt_s):
        with pytest.raises(ValueError):
            cmds.line_trajectory(
                start_pos_m=(0.0, 0.0, 0.0),
                end_pos_m=(1.0, 0.0, 0.0),
                duration_s=duration_s,
                dt_s=dt_s,
            )

    def test_rejects_non_finite_position(self):
        with pytest.raises(ValueError):
            cmds.line_trajectory(
                start_pos_m=(0.0, float("inf"), 0.0),
                end_pos_m=(1.0, 0.0, 0.0),
                duration_s=1.0,
                dt_s=0.1,
            )

    def test_rejects_degenerate_quaternion(self):
        with pytest.raises(ValueError):
            cmds.line_trajectory(
                start_pos_m=(0.0, 0.0, 0.0),
                end_pos_m=(1.0, 0.0, 0.0),
                duration_s=1.0,
                dt_s=0.1,
                orientation_quat=(0.0, 0.0, 0.0, 0.0),
            )


# ---------------------------------------------------------------------------
# arc_trajectory
# ---------------------------------------------------------------------------


class TestArcTrajectory:
    def test_sample_count_and_endpoints_xy(self):
        traj = cmds.arc_trajectory(
            center_m=(0.0, 0.0, 0.5),
            radius_m=0.1,
            start_angle_rad=0.0,
            end_angle_rad=math.pi / 2.0,
            plane="xy",
            duration_s=1.0,
            dt_s=0.1,
        )
        assert len(traj) == 11
        # t=0 → (r, 0, z0); t=T → (0, r, z0).
        assert traj.positions[0] == pytest.approx((0.1, 0.0, 0.5), abs=1e-12)
        assert traj.positions[-1] == pytest.approx((0.0, 0.1, 0.5), abs=1e-12)

    def test_radius_is_preserved_in_plane(self):
        center = (0.3, -0.2, 0.5)
        traj = cmds.arc_trajectory(
            center_m=center,
            radius_m=0.15,
            start_angle_rad=0.0,
            end_angle_rad=2.0 * math.pi,
            plane="xy",
            duration_s=1.0,
            dt_s=0.01,
        )
        for p in traj.positions:
            dx = p[0] - center[0]
            dy = p[1] - center[1]
            r = math.sqrt(dx * dx + dy * dy)
            assert r == pytest.approx(0.15, abs=1e-9)
            # Out-of-plane coord held at centre.
            assert p[2] == pytest.approx(center[2], abs=1e-12)

    def test_plane_xz(self):
        traj = cmds.arc_trajectory(
            center_m=(0.0, 0.2, 0.5),
            radius_m=0.1,
            start_angle_rad=0.0,
            end_angle_rad=math.pi,
            plane="xz",
            duration_s=0.5,
            dt_s=0.05,
        )
        for p in traj.positions:
            assert p[1] == pytest.approx(0.2, abs=1e-12)
        assert traj.positions[0] == pytest.approx((0.1, 0.2, 0.5), abs=1e-12)
        assert traj.positions[-1] == pytest.approx((-0.1, 0.2, 0.5), abs=1e-12)

    def test_plane_yz(self):
        traj = cmds.arc_trajectory(
            center_m=(0.1, 0.0, 0.4),
            radius_m=0.05,
            start_angle_rad=0.0,
            end_angle_rad=math.pi / 2.0,
            plane="yz",
            duration_s=0.2,
            dt_s=0.02,
        )
        for p in traj.positions:
            assert p[0] == pytest.approx(0.1, abs=1e-12)
        assert traj.positions[-1] == pytest.approx((0.1, 0.0, 0.45), abs=1e-12)

    def test_rejects_bad_radius(self):
        for bad in (0.0, -0.1, float("nan"), float("inf")):
            with pytest.raises(ValueError):
                cmds.arc_trajectory(
                    center_m=(0.0, 0.0, 0.0),
                    radius_m=bad,
                    start_angle_rad=0.0,
                    end_angle_rad=1.0,
                    duration_s=1.0,
                    dt_s=0.1,
                )

    def test_rejects_unknown_plane(self):
        with pytest.raises(ValueError):
            cmds.arc_trajectory(
                center_m=(0.0, 0.0, 0.0),
                radius_m=0.1,
                start_angle_rad=0.0,
                end_angle_rad=1.0,
                plane="zz",
                duration_s=1.0,
                dt_s=0.1,
            )

    def test_rejects_non_finite_angle(self):
        with pytest.raises(ValueError):
            cmds.arc_trajectory(
                center_m=(0.0, 0.0, 0.0),
                radius_m=0.1,
                start_angle_rad=float("inf"),
                end_angle_rad=1.0,
                duration_s=1.0,
                dt_s=0.1,
            )


# ---------------------------------------------------------------------------
# sine_in_z_trajectory
# ---------------------------------------------------------------------------


class TestSineInZTrajectory:
    def test_sample_count_and_bounds(self):
        traj = cmds.sine_in_z_trajectory(
            base_pos_m=(0.5, 0.0, 0.4),
            amplitude_m=0.05,
            frequency_hz=2.0,
            duration_s=1.0,
            dt_s=0.001,
        )
        assert len(traj) == 1001
        zs = [p[2] for p in traj.positions]
        assert min(zs) == pytest.approx(0.35, abs=1e-6)
        assert max(zs) == pytest.approx(0.45, abs=1e-6)
        # x/y held constant.
        for p in traj.positions:
            assert p[0] == pytest.approx(0.5, abs=1e-12)
            assert p[1] == pytest.approx(0.0, abs=1e-12)

    def test_zero_phase_starts_at_base(self):
        traj = cmds.sine_in_z_trajectory(
            base_pos_m=(0.0, 0.0, 0.5),
            amplitude_m=0.1,
            frequency_hz=1.0,
            duration_s=1.0,
            dt_s=0.1,
        )
        assert traj.positions[0][2] == pytest.approx(0.5, abs=1e-12)

    def test_phase_shift_quarter_period(self):
        # Phase π/2 ⇒ sin(π/2) = 1 at t=0.
        traj = cmds.sine_in_z_trajectory(
            base_pos_m=(0.0, 0.0, 0.5),
            amplitude_m=0.1,
            frequency_hz=1.0,
            duration_s=0.1,
            dt_s=0.01,
            phase_rad=math.pi / 2.0,
        )
        assert traj.positions[0][2] == pytest.approx(0.6, abs=1e-12)

    def test_rejects_bad_frequency(self):
        for bad in (0.0, -1.0, float("nan"), float("inf")):
            with pytest.raises(ValueError):
                cmds.sine_in_z_trajectory(
                    base_pos_m=(0.0, 0.0, 0.5),
                    amplitude_m=0.1,
                    frequency_hz=bad,
                    duration_s=1.0,
                    dt_s=0.01,
                )

    def test_rejects_non_finite_amplitude(self):
        with pytest.raises(ValueError):
            cmds.sine_in_z_trajectory(
                base_pos_m=(0.0, 0.0, 0.5),
                amplitude_m=float("nan"),
                frequency_hz=1.0,
                duration_s=1.0,
                dt_s=0.01,
            )


# ---------------------------------------------------------------------------
# Integration with the stage-3 assertion harness
# ---------------------------------------------------------------------------


class TestCompatibilityWithEvaluator:
    """Each generator must produce a trajectory that the stage-3 evaluator
    accepts on both the commanded and measured side. Feeding the same
    trajectory into both sides should yield zero error and OK status."""

    def _assert_zero_error(self, traj):
        result = r2s3.evaluate_tcp_trajectory(
            controller="dummy",
            times=traj.times,
            measured_positions_m=traj.positions,
            commanded_positions_m=traj.positions,
            measured_orientations_quat=traj.orientations,
            commanded_orientations_quat=traj.orientations,
            tcp_rmse_mm=5.0,
            tcp_peak_err_mm=10.0,
            tcp_orientation_peak_deg=3.0,
        )
        assert result.ok, result.format()
        assert result.metrics["position_rmse_mm"] == pytest.approx(0.0, abs=1e-9)
        assert result.metrics["position_peak_err_mm"] == pytest.approx(0.0, abs=1e-9)
        assert result.metrics["orientation_peak_err_deg"] == pytest.approx(0.0, abs=1e-9)

    def test_line_compatible(self):
        traj = cmds.line_trajectory(
            start_pos_m=(0.3, 0.0, 0.4),
            end_pos_m=(0.5, 0.1, 0.4),
            duration_s=2.0,
            dt_s=0.01,
        )
        self._assert_zero_error(traj)

    def test_arc_compatible(self):
        traj = cmds.arc_trajectory(
            center_m=(0.4, 0.0, 0.5),
            radius_m=0.05,
            start_angle_rad=0.0,
            end_angle_rad=math.pi,
            plane="xy",
            duration_s=1.0,
            dt_s=0.01,
        )
        self._assert_zero_error(traj)

    def test_sine_compatible(self):
        traj = cmds.sine_in_z_trajectory(
            base_pos_m=(0.4, 0.0, 0.5),
            amplitude_m=0.02,
            frequency_hz=1.0,
            duration_s=2.0,
            dt_s=0.01,
        )
        self._assert_zero_error(traj)


# ---------------------------------------------------------------------------
# TcpTrajectory dataclass
# ---------------------------------------------------------------------------


class TestTcpTrajectory:
    def test_immutable(self):
        traj = cmds.line_trajectory(
            start_pos_m=(0.0, 0.0, 0.0),
            end_pos_m=(1.0, 0.0, 0.0),
            duration_s=0.5,
            dt_s=0.1,
        )
        with pytest.raises((AttributeError, Exception)):
            traj.times = (0.0,)  # type: ignore[misc]

    def test_duration_property(self):
        traj = cmds.line_trajectory(
            start_pos_m=(0.0, 0.0, 0.0),
            end_pos_m=(1.0, 0.0, 0.0),
            duration_s=1.5,
            dt_s=0.1,
        )
        assert traj.duration_s == pytest.approx(1.5, abs=1e-12)
