"""Unit tests for tests/integration/r2_tcp_from_joints.py.

Covers the FK-injected adapter that converts a measured joint-state
stream into a :class:`TcpTrajectory`. Pure stdlib — runs in the unit
gate.

Tests use a dummy FK fixture so the module under test stays orthogonal
to whichever FK backend the operator eventually picks. The dummy FK
does not pretend to be kinematically accurate; it only has to satisfy
the adapter's contract so the adapter's validation + plumbing code can
be exercised.
"""

from __future__ import annotations

import importlib.util
import math
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
ADAPTER_PATH = REPO_ROOT / "tests" / "integration" / "r2_tcp_from_joints.py"
CMDS_PATH = REPO_ROOT / "tests" / "integration" / "r2_stage3_commands.py"
ASSERT_PATH = REPO_ROOT / "tests" / "integration" / "r2_stage3_assertions.py"


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec and spec.loader
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


adapter = _load("r2_tcp_from_joints_uut", ADAPTER_PATH)
cmds = _load("r2_stage3_commands_for_adapter", CMDS_PATH)
r2s3 = _load("r2_stage3_assertions_for_adapter", ASSERT_PATH)


IDENTITY_Q = (0.0, 0.0, 0.0, 1.0)


def _identity_fk(q):
    """Dummy FK: pos = (q0, q1, q2), quat = identity.

    Only requires ``len(q) >= 3``. Good enough for adapter plumbing.
    """
    return ((float(q[0]), float(q[1]), float(q[2])), IDENTITY_Q)


def _linear_fk(q):
    """Dummy FK: pos = sum-of-joints along x, identity orientation.

    Smoother than ``_identity_fk`` for tests that want monotonically
    increasing position across samples of varying length.
    """
    s = sum(float(v) for v in q)
    return ((s, 0.0, 0.0), IDENTITY_Q)


# ---------------------------------------------------------------------------
# Happy path
# ---------------------------------------------------------------------------


class TestHappyPath:
    def test_returns_tcp_trajectory(self):
        times = [0.1, 0.2, 0.3, 0.4]
        samples = [(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)] * 4
        traj = adapter.tcp_trajectory_from_joints(times, samples, _identity_fk)
        assert isinstance(traj, adapter.TcpTrajectory)
        # Dataclass attributes are the public contract (class identity
        # would depend on importlib caching when the sibling module
        # is loaded twice under different synthetic names).
        assert hasattr(traj, "times")
        assert hasattr(traj, "positions")
        assert hasattr(traj, "orientations")
        assert len(traj) == 4

    def test_times_rebased_to_zero(self):
        times = [5.0, 5.1, 5.2, 5.3]
        samples = [(0.0, 0.0, 0.0)] * 4
        traj = adapter.tcp_trajectory_from_joints(times, samples, _identity_fk)
        assert traj.times[0] == 0.0
        assert traj.times[-1] == pytest.approx(0.3, abs=1e-12)
        # Strictly monotonic.
        for i in range(1, len(traj.times)):
            assert traj.times[i] > traj.times[i - 1]

    def test_positions_from_fk(self):
        times = [0.0, 0.1, 0.2]
        samples = [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0), (7.0, 8.0, 9.0)]
        traj = adapter.tcp_trajectory_from_joints(times, samples, _identity_fk)
        assert traj.positions == ((1.0, 2.0, 3.0), (4.0, 5.0, 6.0), (7.0, 8.0, 9.0))

    def test_orientations_unit_normalised(self):
        def fk(q):
            # Deliberately non-unit (norm ~0.6).
            return ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.6))

        times = [0.0, 0.1]
        samples = [(0.0,), (0.0,)]
        traj = adapter.tcp_trajectory_from_joints(times, samples, fk)
        for q in traj.orientations:
            norm = math.sqrt(sum(v * v for v in q))
            assert norm == pytest.approx(1.0, abs=1e-12)
        # Direction preserved (only w component was non-zero).
        assert traj.orientations[0] == pytest.approx(IDENTITY_Q, abs=1e-12)

    def test_accepts_numpy_like_via_float_coerce(self):
        # Lists of Python floats coerced via ``float(...)`` already
        # handle anything supporting __float__ (numpy scalars, etc.).
        # Exercise with a generator of tuples to prove we don't
        # require a concrete list.
        times = (0.0, 0.01, 0.02)
        samples = [list(range(3)) for _ in range(3)]
        traj = adapter.tcp_trajectory_from_joints(times, samples, _identity_fk)
        assert len(traj) == 3

    def test_interop_with_evaluate_tcp_trajectory(self):
        """Adapter output plugs into stage-3 evaluator as both sides."""
        times = [0.0, 0.01, 0.02, 0.03, 0.04]
        samples = [(float(i), 0.0, 0.0) for i in range(5)]
        traj = adapter.tcp_trajectory_from_joints(times, samples, _linear_fk)
        result = r2s3.evaluate_tcp_trajectory(
            "dummy_controller",
            times=traj.times,
            measured_positions_m=traj.positions,
            commanded_positions_m=traj.positions,
            measured_orientations_quat=traj.orientations,
            commanded_orientations_quat=traj.orientations,
            tcp_rmse_mm=5.0,
            tcp_peak_err_mm=10.0,
            tcp_orientation_peak_deg=3.0,
        )
        # Zero tracking error when commanded == measured.
        assert result.ok
        assert result.metrics["position_rmse_mm"] == pytest.approx(0.0, abs=1e-9)
        assert result.metrics["position_peak_err_mm"] == pytest.approx(0.0, abs=1e-9)
        assert result.metrics["orientation_peak_err_deg"] == pytest.approx(0.0, abs=1e-9)


# ---------------------------------------------------------------------------
# Input validation
# ---------------------------------------------------------------------------


class TestInputValidation:
    def test_length_mismatch_rejected(self):
        with pytest.raises(ValueError, match="does not match"):
            adapter.tcp_trajectory_from_joints(
                [0.0, 0.1, 0.2],
                [(0.0,), (0.0,)],
                _identity_fk,
            )

    def test_empty_rejected(self):
        with pytest.raises(ValueError, match="at least 2 samples"):
            adapter.tcp_trajectory_from_joints([], [], _identity_fk)

    def test_single_sample_rejected(self):
        with pytest.raises(ValueError, match="at least 2 samples"):
            adapter.tcp_trajectory_from_joints(
                [0.0],
                [(0.0,)],
                _identity_fk,
            )

    def test_non_monotonic_times_rejected(self):
        with pytest.raises(ValueError, match="strictly increasing"):
            adapter.tcp_trajectory_from_joints(
                [0.0, 0.2, 0.1],
                [(0.0,), (0.0,), (0.0,)],
                _identity_fk,
            )

    def test_duplicate_times_rejected(self):
        with pytest.raises(ValueError, match="strictly increasing"):
            adapter.tcp_trajectory_from_joints(
                [0.0, 0.1, 0.1],
                [(0.0,), (0.0,), (0.0,)],
                _identity_fk,
            )

    def test_non_finite_times_rejected(self):
        with pytest.raises(ValueError, match="not finite"):
            adapter.tcp_trajectory_from_joints(
                [0.0, float("nan"), 0.2],
                [(0.0,), (0.0,), (0.0,)],
                _identity_fk,
            )
        with pytest.raises(ValueError, match="not finite"):
            adapter.tcp_trajectory_from_joints(
                [0.0, 0.1, float("inf")],
                [(0.0,), (0.0,), (0.0,)],
                _identity_fk,
            )

    def test_inconsistent_joint_dim_rejected(self):
        with pytest.raises(ValueError, match="dim"):
            adapter.tcp_trajectory_from_joints(
                [0.0, 0.1, 0.2],
                [(0.0, 0.0, 0.0), (0.0, 0.0), (0.0, 0.0, 0.0)],
                _identity_fk,
            )

    def test_empty_joint_vector_rejected(self):
        with pytest.raises(ValueError, match="empty"):
            adapter.tcp_trajectory_from_joints(
                [0.0, 0.1],
                [(), ()],
                _identity_fk,
            )

    def test_non_finite_joint_value_rejected(self):
        with pytest.raises(ValueError, match="not.*finite"):
            adapter.tcp_trajectory_from_joints(
                [0.0, 0.1],
                [(0.0, 0.0, 0.0), (float("nan"), 0.0, 0.0)],
                _identity_fk,
            )

    def test_non_numeric_joint_sample_rejected(self):
        with pytest.raises(ValueError, match="not a numeric sequence"):
            adapter.tcp_trajectory_from_joints(
                [0.0, 0.1],
                [(0.0, 0.0, 0.0), ("oops", 0.0, 0.0)],
                _identity_fk,
            )


# ---------------------------------------------------------------------------
# FK output validation
# ---------------------------------------------------------------------------


class TestFkOutput:
    def test_fk_exception_wrapped_with_sample_index(self):
        boom_at = 2

        def fk(q):
            fk.calls = getattr(fk, "calls", 0) + 1
            if fk.calls - 1 == boom_at:
                raise RuntimeError("backend blew up")
            return ((0.0, 0.0, 0.0), IDENTITY_Q)

        with pytest.raises(ValueError) as excinfo:
            adapter.tcp_trajectory_from_joints(
                [0.1, 0.2, 0.3, 0.4],
                [(0.0,)] * 4,
                fk,
            )
        msg = str(excinfo.value)
        assert "sample 2" in msg
        # Timestamp of the failing sample is the absolute (pre-rebase)
        # time; callers should be able to grep the raw joint-state log.
        assert "0.3" in msg
        assert "backend blew up" in msg

    def test_fk_wrong_pos_length_rejected(self):
        def fk(q):
            return ((0.0, 0.0), IDENTITY_Q)  # only 2 floats

        with pytest.raises(ValueError, match="pos of wrong length"):
            adapter.tcp_trajectory_from_joints([0.0, 0.1], [(0.0,), (0.0,)], fk)

    def test_fk_wrong_quat_length_rejected(self):
        def fk(q):
            return ((0.0, 0.0, 0.0), (0.0, 0.0, 1.0))  # only 3 floats

        with pytest.raises(ValueError, match="quat of wrong length"):
            adapter.tcp_trajectory_from_joints([0.0, 0.1], [(0.0,), (0.0,)], fk)

    def test_fk_wrong_output_shape_rejected(self):
        def fk(q):
            return (0.0, 0.0, 0.0)  # not a (pos, quat) pair

        with pytest.raises(ValueError, match=r"\(pos_xyz, quat_xyzw\)"):
            adapter.tcp_trajectory_from_joints([0.0, 0.1], [(0.0,), (0.0,)], fk)

    def test_fk_non_finite_pos_rejected(self):
        def fk(q):
            return ((float("nan"), 0.0, 0.0), IDENTITY_Q)

        with pytest.raises(ValueError, match="non-finite pos"):
            adapter.tcp_trajectory_from_joints([0.0, 0.1], [(0.0,), (0.0,)], fk)

    def test_fk_non_finite_quat_rejected(self):
        def fk(q):
            return ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, float("inf")))

        with pytest.raises(ValueError, match="non-finite quat"):
            adapter.tcp_trajectory_from_joints([0.0, 0.1], [(0.0,), (0.0,)], fk)

    def test_fk_zero_quat_rejected(self):
        def fk(q):
            return ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0))

        with pytest.raises(ValueError, match="degenerate quaternion"):
            adapter.tcp_trajectory_from_joints([0.0, 0.1], [(0.0,), (0.0,)], fk)

    def test_fk_oversize_quat_rejected(self):
        # Norm 2.0 sits outside the [0.5, 1.5] acceptance band.
        def fk(q):
            return ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 2.0))

        with pytest.raises(ValueError, match="degenerate quaternion"):
            adapter.tcp_trajectory_from_joints([0.0, 0.1], [(0.0,), (0.0,)], fk)


# ---------------------------------------------------------------------------
# Exports
# ---------------------------------------------------------------------------


class TestExports:
    def test_all_lists_public_surface(self):
        assert "FkCallable" in adapter.__all__
        assert "TcpTrajectory" in adapter.__all__
        assert "tcp_trajectory_from_joints" in adapter.__all__

    def test_tcp_trajectory_shares_sibling_attributes(self):
        # Adapter's TcpTrajectory exposes the same attribute surface as
        # the sibling dataclass the stage-3 evaluator consumes; exact
        # class identity depends on importlib cache keys so we do not
        # rely on ``is``. Inspect dataclass fields instead of instances.
        adapter_fields = set(adapter.TcpTrajectory.__dataclass_fields__)
        sibling_fields = set(cmds.TcpTrajectory.__dataclass_fields__)
        assert {"times", "positions", "orientations"} <= adapter_fields
        assert adapter_fields == sibling_fields
        # ``duration_s`` is a property, not a field.
        assert isinstance(getattr(adapter.TcpTrajectory, "duration_s"), property)
        assert isinstance(getattr(cmds.TcpTrajectory, "duration_s"), property)
