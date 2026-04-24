"""Unit tests for tests/integration/r2_stage3_assertions.py.

Covers the TCP-trajectory (R2 stage-3) evaluator pre-baked for the
M6.14 integration test matrix (still gated on M6.0 and the FK-source
decision — see ``docs/STATUS.md``). Pure stdlib — runs in the unit
gate.
"""

from __future__ import annotations

import importlib.util
import math
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
MOD_PATH = REPO_ROOT / "tests" / "integration" / "r2_stage3_assertions.py"
LOADER_PATH = REPO_ROOT / "tests" / "integration" / "expectations_loader.py"


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec and spec.loader
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


r2s3 = _load("r2_stage3_assertions_uut", MOD_PATH)
expectations_loader = _load("expectations_loader_for_s3", LOADER_PATH)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


IDENTITY_Q = (0.0, 0.0, 0.0, 1.0)


def _perfect_hold(n: int = 101, dt: float = 0.01, pos=(0.5, 0.0, 0.4)):
    """Stationary TCP, identity orientation, perfect tracking."""
    ts = [i * dt for i in range(n)]
    m_pos = [pos] * n
    c_pos = [pos] * n
    m_ori = [IDENTITY_Q] * n
    c_ori = [IDENTITY_Q] * n
    return ts, m_pos, c_pos, m_ori, c_ori


def _quat_from_axis_angle(axis, angle_rad):
    ax, ay, az = axis
    norm = math.sqrt(ax * ax + ay * ay + az * az)
    ax, ay, az = ax / norm, ay / norm, az / norm
    s = math.sin(angle_rad / 2.0)
    c = math.cos(angle_rad / 2.0)
    return (ax * s, ay * s, az * s, c)


# ---------------------------------------------------------------------------
# Happy path
# ---------------------------------------------------------------------------


def test_happy_path_perfect_tracking_passes():
    ts, mp, cp, mo, co = _perfect_hold()
    r = r2s3.evaluate_tcp_trajectory(
        controller="cartesian_motion_controller",
        times=ts,
        measured_positions_m=mp,
        commanded_positions_m=cp,
        measured_orientations_quat=mo,
        commanded_orientations_quat=co,
        tcp_rmse_mm=5.0,
        tcp_peak_err_mm=10.0,
        tcp_orientation_peak_deg=3.0,
    )
    assert r.ok
    assert r.failures == ()
    assert r.controller == "cartesian_motion_controller"
    assert r.metrics["position_rmse_mm"] == pytest.approx(0.0, abs=1e-9)
    assert r.metrics["position_peak_err_mm"] == pytest.approx(0.0, abs=1e-9)
    assert r.metrics["orientation_peak_err_deg"] == pytest.approx(0.0, abs=1e-9)
    # Drift check not requested → metric absent.
    assert "steady_drift_mm" not in r.metrics
    assert "steady_drift_skipped" not in r.metrics


# ---------------------------------------------------------------------------
# Position tolerances
# ---------------------------------------------------------------------------


def test_rmse_failure_flagged_in_mm():
    # Uniform 6 mm X offset on every sample.
    ts, _, cp, mo, co = _perfect_hold()
    mp = [(p[0] + 0.006, p[1], p[2]) for p in cp]
    r = r2s3.evaluate_tcp_trajectory(
        controller="c",
        times=ts,
        measured_positions_m=mp,
        commanded_positions_m=cp,
        measured_orientations_quat=mo,
        commanded_orientations_quat=co,
        tcp_rmse_mm=5.0,
        tcp_peak_err_mm=10.0,
        tcp_orientation_peak_deg=3.0,
    )
    assert not r.ok
    assert r.metrics["position_rmse_mm"] == pytest.approx(6.0, abs=1e-9)
    assert r.metrics["position_peak_err_mm"] == pytest.approx(6.0, abs=1e-9)
    # Only RMSE fails (peak tol is 10 mm, actual peak is 6 mm).
    assert len(r.failures) == 1
    assert r.failures[0].startswith("position_rmse_mm=")


def test_peak_failure_flagged_independently():
    ts, _, cp, mo, co = _perfect_hold()
    mp = [(p[0], p[1], p[2]) for p in cp]
    # One sample with a 15 mm excursion; RMSE stays tiny.
    mp[50] = (cp[50][0] + 0.015, cp[50][1], cp[50][2])
    r = r2s3.evaluate_tcp_trajectory(
        controller="c",
        times=ts,
        measured_positions_m=mp,
        commanded_positions_m=cp,
        measured_orientations_quat=mo,
        commanded_orientations_quat=co,
        tcp_rmse_mm=5.0,
        tcp_peak_err_mm=10.0,
        tcp_orientation_peak_deg=3.0,
    )
    assert not r.ok
    assert r.metrics["position_peak_err_mm"] == pytest.approx(15.0, abs=1e-9)
    assert r.metrics["position_rmse_mm"] < 5.0
    assert len(r.failures) == 1
    assert r.failures[0].startswith("position_peak_err_mm=")


# ---------------------------------------------------------------------------
# Orientation
# ---------------------------------------------------------------------------


def test_orientation_peak_flagged_on_yaw():
    ts, mp, cp, _, co = _perfect_hold()
    # Measured rotates 5° about Z (yaw) from sample 30 onwards.
    q_yaw5 = _quat_from_axis_angle((0.0, 0.0, 1.0), math.radians(5.0))
    mo = [IDENTITY_Q] * 30 + [q_yaw5] * (len(ts) - 30)
    r = r2s3.evaluate_tcp_trajectory(
        controller="c",
        times=ts,
        measured_positions_m=mp,
        commanded_positions_m=cp,
        measured_orientations_quat=mo,
        commanded_orientations_quat=co,
        tcp_rmse_mm=5.0,
        tcp_peak_err_mm=10.0,
        tcp_orientation_peak_deg=3.0,
    )
    assert not r.ok
    assert r.metrics["orientation_peak_err_deg"] == pytest.approx(5.0, abs=1e-6)
    assert any(f.startswith("orientation_peak_err_deg=") for f in r.failures)


def test_orientation_antipodal_quaternion_reports_zero():
    """q and -q represent the same rotation; per-axis RPY must be ~0."""
    ts, mp, cp, mo, _ = _perfect_hold()
    # Antipodal commanded; same rotation as measured identity.
    co = [(0.0, 0.0, 0.0, -1.0)] * len(ts)
    r = r2s3.evaluate_tcp_trajectory(
        controller="c",
        times=ts,
        measured_positions_m=mp,
        commanded_positions_m=cp,
        measured_orientations_quat=mo,
        commanded_orientations_quat=co,
        tcp_rmse_mm=5.0,
        tcp_peak_err_mm=10.0,
        tcp_orientation_peak_deg=3.0,
    )
    assert r.ok
    assert r.metrics["orientation_peak_err_deg"] == pytest.approx(0.0, abs=1e-9)


def test_orientation_ypr_stricter_than_geodesic_single_axis():
    """For a single-axis rotation, RPY-max == geodesic angle.

    This pins the convention so a future refactor to geodesic angle
    would be caught (that would only differ on compound rotations,
    but this is a regression against single-axis expectations).
    """
    ts, mp, cp, _, co = _perfect_hold()
    # 2.8° pitch — inside the 3° tolerance.
    q_pitch = _quat_from_axis_angle((0.0, 1.0, 0.0), math.radians(2.8))
    mo = [q_pitch] * len(ts)
    r = r2s3.evaluate_tcp_trajectory(
        controller="c",
        times=ts,
        measured_positions_m=mp,
        commanded_positions_m=cp,
        measured_orientations_quat=mo,
        commanded_orientations_quat=co,
        tcp_rmse_mm=5.0,
        tcp_peak_err_mm=10.0,
        tcp_orientation_peak_deg=3.0,
    )
    assert r.ok
    assert r.metrics["orientation_peak_err_deg"] == pytest.approx(2.8, abs=1e-4)


def test_non_unit_quaternion_normalized_internally():
    """Slightly non-unit quaternions (logged-float drift) pass."""
    ts, mp, cp, _, co = _perfect_hold()
    # Norm 1.01 — within the tolerant band. Evaluator should normalize.
    scale = 1.01
    mo = [(0.0, 0.0, 0.0, 1.0 * scale)] * len(ts)
    r = r2s3.evaluate_tcp_trajectory(
        controller="c",
        times=ts,
        measured_positions_m=mp,
        commanded_positions_m=cp,
        measured_orientations_quat=mo,
        commanded_orientations_quat=co,
        tcp_rmse_mm=5.0,
        tcp_peak_err_mm=10.0,
        tcp_orientation_peak_deg=3.0,
    )
    assert r.ok
    assert r.metrics["orientation_peak_err_deg"] == pytest.approx(0.0, abs=1e-9)


def test_degenerate_quaternion_norm_raises():
    ts, mp, cp, _, co = _perfect_hold()
    mo = [(0.0, 0.0, 0.0, 0.0)] * len(ts)  # zero norm
    with pytest.raises(ValueError, match="degenerate quaternion norm"):
        r2s3.evaluate_tcp_trajectory(
            controller="c",
            times=ts,
            measured_positions_m=mp,
            commanded_positions_m=cp,
            measured_orientations_quat=mo,
            commanded_orientations_quat=co,
            tcp_rmse_mm=5.0,
            tcp_peak_err_mm=10.0,
            tcp_orientation_peak_deg=3.0,
        )


# ---------------------------------------------------------------------------
# Steady drift
# ---------------------------------------------------------------------------


def _drift_fixture(total_s: float = 35.0, dt: float = 0.05, drift_per_s_m: float = 0.0):
    """Stationary baseline with optional linear drift along +X."""
    n = int(round(total_s / dt)) + 1
    ts = [i * dt for i in range(n)]
    m_pos = [(0.5 + drift_per_s_m * t, 0.0, 0.4) for t in ts]
    c_pos = [(0.5, 0.0, 0.4)] * n
    m_ori = [IDENTITY_Q] * n
    c_ori = [IDENTITY_Q] * n
    return ts, m_pos, c_pos, m_ori, c_ori


def test_drift_check_passes_when_stationary():
    ts, mp, cp, mo, co = _drift_fixture()
    r = r2s3.evaluate_tcp_trajectory(
        controller="c",
        times=ts,
        measured_positions_m=mp,
        commanded_positions_m=cp,
        measured_orientations_quat=mo,
        commanded_orientations_quat=co,
        tcp_rmse_mm=5.0,
        tcp_peak_err_mm=10.0,
        tcp_orientation_peak_deg=3.0,
        tcp_steady_drift_mm_per_30s=2.0,
        steady_window_s=30.0,
    )
    assert r.ok
    assert r.metrics["steady_drift_mm"] == pytest.approx(0.0, abs=1e-9)
    assert r.metrics["steady_window_samples"] > 4


def test_drift_check_fails_on_linear_drift():
    # 0.2 mm/s drift over the 30-s window ⇒ half-window 15 s ⇒
    # mean-first-half vs mean-second-half delta ≈ 0.2 * 15 = 3 mm. Exceeds 2 mm tol.
    ts, mp, cp, mo, co = _drift_fixture(drift_per_s_m=0.0002)
    r = r2s3.evaluate_tcp_trajectory(
        controller="c",
        times=ts,
        measured_positions_m=mp,
        commanded_positions_m=cp,
        measured_orientations_quat=mo,
        commanded_orientations_quat=co,
        tcp_rmse_mm=500.0,  # loose — the drift moves position too
        tcp_peak_err_mm=500.0,
        tcp_orientation_peak_deg=3.0,
        tcp_steady_drift_mm_per_30s=2.0,
        steady_window_s=30.0,
    )
    assert not r.ok
    assert r.metrics["steady_drift_mm"] == pytest.approx(3.0, rel=0.05)
    assert any(f.startswith("steady_drift_mm=") for f in r.failures)


def test_drift_skipped_when_window_too_short():
    ts, mp, cp, mo, co = _drift_fixture(total_s=20.0)
    r = r2s3.evaluate_tcp_trajectory(
        controller="c",
        times=ts,
        measured_positions_m=mp,
        commanded_positions_m=cp,
        measured_orientations_quat=mo,
        commanded_orientations_quat=co,
        tcp_rmse_mm=5.0,
        tcp_peak_err_mm=10.0,
        tcp_orientation_peak_deg=3.0,
        tcp_steady_drift_mm_per_30s=2.0,
        steady_window_s=15.0,
    )
    assert r.ok
    assert r.metrics["steady_drift_skipped"] == 1.0
    assert "steady_drift_mm" not in r.metrics
    assert any("drift check skipped" in n for n in r.notes)


def test_drift_skipped_when_tolerance_none():
    ts, mp, cp, mo, co = _drift_fixture()
    r = r2s3.evaluate_tcp_trajectory(
        controller="c",
        times=ts,
        measured_positions_m=mp,
        commanded_positions_m=cp,
        measured_orientations_quat=mo,
        commanded_orientations_quat=co,
        tcp_rmse_mm=5.0,
        tcp_peak_err_mm=10.0,
        tcp_orientation_peak_deg=3.0,
        tcp_steady_drift_mm_per_30s=None,
        steady_window_s=30.0,
    )
    assert r.ok
    assert "steady_drift_mm" not in r.metrics
    assert "steady_drift_skipped" not in r.metrics


# ---------------------------------------------------------------------------
# Validation
# ---------------------------------------------------------------------------


def test_non_monotonic_times_rejected():
    ts = [0.0, 0.1, 0.1, 0.2]
    four = [IDENTITY_Q] * 4
    pos = [(0.0, 0.0, 0.0)] * 4
    with pytest.raises(ValueError, match="strictly monotonic"):
        r2s3.evaluate_tcp_trajectory(
            controller="c",
            times=ts,
            measured_positions_m=pos,
            commanded_positions_m=pos,
            measured_orientations_quat=four,
            commanded_orientations_quat=four,
            tcp_rmse_mm=5.0,
            tcp_peak_err_mm=10.0,
            tcp_orientation_peak_deg=3.0,
        )


def test_trace_length_mismatch_rejected():
    ts = [0.0, 0.1, 0.2]
    with pytest.raises(ValueError, match="length 2 != times length 3"):
        r2s3.evaluate_tcp_trajectory(
            controller="c",
            times=ts,
            measured_positions_m=[(0.0, 0.0, 0.0)] * 2,
            commanded_positions_m=[(0.0, 0.0, 0.0)] * 3,
            measured_orientations_quat=[IDENTITY_Q] * 3,
            commanded_orientations_quat=[IDENTITY_Q] * 3,
            tcp_rmse_mm=5.0,
            tcp_peak_err_mm=10.0,
            tcp_orientation_peak_deg=3.0,
        )


def test_negative_steady_window_rejected():
    ts, mp, cp, mo, co = _perfect_hold()
    with pytest.raises(ValueError, match="steady_window_s must be >= 0"):
        r2s3.evaluate_tcp_trajectory(
            controller="c",
            times=ts,
            measured_positions_m=mp,
            commanded_positions_m=cp,
            measured_orientations_quat=mo,
            commanded_orientations_quat=co,
            tcp_rmse_mm=5.0,
            tcp_peak_err_mm=10.0,
            tcp_orientation_peak_deg=3.0,
            steady_window_s=-0.1,
        )


def test_steady_window_exceeds_trace_span_rejected():
    ts, mp, cp, mo, co = _perfect_hold(n=11, dt=0.1)  # span = 1.0 s
    with pytest.raises(ValueError, match="exceeds trace span"):
        r2s3.evaluate_tcp_trajectory(
            controller="c",
            times=ts,
            measured_positions_m=mp,
            commanded_positions_m=cp,
            measured_orientations_quat=mo,
            commanded_orientations_quat=co,
            tcp_rmse_mm=5.0,
            tcp_peak_err_mm=10.0,
            tcp_orientation_peak_deg=3.0,
            steady_window_s=2.0,
        )


# ---------------------------------------------------------------------------
# Expectation-driven wrapper
# ---------------------------------------------------------------------------


def test_wrapper_sources_tolerances_from_expectation():
    arm = expectations_loader.load_arm("ur5e")
    ts, mp, cp, mo, co = _perfect_hold()
    r = r2s3.evaluate_tcp_trajectory_from_expectation(
        arm,
        controller="cartesian_motion_controller",
        times=ts,
        measured_positions_m=mp,
        commanded_positions_m=cp,
        measured_orientations_quat=mo,
        commanded_orientations_quat=co,
    )
    assert r.ok
    assert r.controller == "cartesian_motion_controller"
    # Drift metric absent: caller left steady_window_s=0.
    assert "steady_drift_mm" not in r.metrics


def test_wrapper_steady_window_flows_through():
    arm = expectations_loader.load_arm("ur5e")
    ts, mp, cp, mo, co = _drift_fixture()
    r = r2s3.evaluate_tcp_trajectory_from_expectation(
        arm,
        controller="c",
        times=ts,
        measured_positions_m=mp,
        commanded_positions_m=cp,
        measured_orientations_quat=mo,
        commanded_orientations_quat=co,
        steady_window_s=30.0,
    )
    # YAML ships 2 mm per 30 s as the draft tolerance; stationary input
    # ⇒ drift 0 → passes.
    assert r.ok
    assert r.metrics["steady_drift_mm"] == pytest.approx(0.0, abs=1e-9)


def test_wrapper_propagates_position_failure():
    arm = expectations_loader.load_arm("ur15")
    ts, _, cp, mo, co = _perfect_hold()
    # Uniform 12 mm offset — exceeds peak tol of 10 mm.
    mp = [(p[0] + 0.012, p[1], p[2]) for p in cp]
    r = r2s3.evaluate_tcp_trajectory_from_expectation(
        arm,
        controller="c",
        times=ts,
        measured_positions_m=mp,
        commanded_positions_m=cp,
        measured_orientations_quat=mo,
        commanded_orientations_quat=co,
    )
    assert not r.ok
    # Both RMSE and peak fail at 12 mm (RMSE tol 5 mm, peak tol 10 mm).
    assert any(f.startswith("position_rmse_mm=") for f in r.failures)
    assert any(f.startswith("position_peak_err_mm=") for f in r.failures)


# ---------------------------------------------------------------------------
# Result formatting
# ---------------------------------------------------------------------------


def test_result_format_includes_metrics_and_failures():
    ts, _, cp, mo, co = _perfect_hold()
    mp = [(p[0] + 0.020, p[1], p[2]) for p in cp]
    r = r2s3.evaluate_tcp_trajectory(
        controller="crisp_cartesian_impedance",
        times=ts,
        measured_positions_m=mp,
        commanded_positions_m=cp,
        measured_orientations_quat=mo,
        commanded_orientations_quat=co,
        tcp_rmse_mm=5.0,
        tcp_peak_err_mm=10.0,
        tcp_orientation_peak_deg=3.0,
    )
    text = r.format()
    assert "crisp_cartesian_impedance" in text
    assert "FAIL" in text
    assert "position_rmse_mm" in text
    assert "position_peak_err_mm" in text
