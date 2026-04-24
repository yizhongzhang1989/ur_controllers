"""Unit tests for the R2 stage-1 assertion harness.

Pins the consumer API of
``tests/integration/r2_stage1_assertions.py`` so the R2 integration
tests (M6.12, still gated on M6.0) can be authored against a stable
surface. Style matches ``test_signal_analysis.py`` and
``test_expectations_loader.py`` — pure stdlib, synthetic signals with
known ground truth, no ROS.

Coverage:

* :func:`evaluate_position_mode` — pass path, steady-state fail,
  drift fail, FFT limit-cycle fail (injected 5 Hz).
* :func:`evaluate_open_loop_effort` — runaway bound pass/fail, and
  saturation-hold check with a synthetic torque trace.
* :func:`evaluate_second_order` — underdamped response where measured
  ζ matches theory, underdamped response where measured ζ is out of
  band, chatter RMS fail, overdamped theory ⇒ ``None`` measurement ⇒
  pass.
* :class:`Stage1Result` — ``ok``, ``format``.
* Validation guard: mismatched trace lengths.
"""

from __future__ import annotations

import importlib.util
import math
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
MOD_PATH = REPO_ROOT / "tests" / "integration" / "r2_stage1_assertions.py"


def _load_module():
    # Force a fresh load so sibling-cache state is predictable if other
    # tests already imported signal_analysis / expectations_loader under
    # different aliases. See tests/integration/r2_stage1_assertions.py
    # _load_sibling for the caching strategy.
    sys.modules.pop("_r2_stage1_assertions", None)
    spec = importlib.util.spec_from_file_location("_r2_stage1_assertions", MOD_PATH)
    assert spec and spec.loader
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope="module")
def mod():
    return _load_module()


# ---------------------------------------------------------------------------
# Fixtures / helpers
# ---------------------------------------------------------------------------


def _make_controller(name, response_model, interface, tolerances):
    # Construct a real ControllerExpectation via the expectations loader
    # module that the harness already imports, so the frozen dataclass
    # accepts the inputs and .tol() raises KeyError on missing keys —
    # matching the real consumer contract.
    spec = importlib.util.spec_from_file_location(
        "_el_for_tests", REPO_ROOT / "tests" / "integration" / "expectations_loader.py"
    )
    el = importlib.util.module_from_spec(spec)
    sys.modules["_el_for_tests"] = el
    spec.loader.exec_module(el)
    return el.ControllerExpectation(
        name=name,
        interface=interface,
        response_model=response_model,
        tolerances=dict(tolerances),
    )


def _make_joint(name="elbow_joint", home=0.0, effort=28.0, j_eff=0.10):
    spec = importlib.util.spec_from_file_location(
        "_el_for_tests2", REPO_ROOT / "tests" / "integration" / "expectations_loader.py"
    )
    el = importlib.util.module_from_spec(spec)
    sys.modules["_el_for_tests2"] = el
    spec.loader.exec_module(el)
    return el.JointExpectation(
        name=name,
        home_rad=home,
        effort_limit_nm=effort,
        effective_inertia_kg_m2=j_eff,
    )


def _uniform_times(n, dt=0.01):
    return [i * dt for i in range(n)]


# ---------------------------------------------------------------------------
# Stage1Result
# ---------------------------------------------------------------------------


def test_stage1_result_ok_and_format(mod):
    r_ok = mod.Stage1Result(
        controller="c",
        joint="j",
        metrics={"x": 1.234},
        failures=(),
    )
    assert r_ok.ok is True
    txt = r_ok.format()
    assert "c" in txt and "j" in txt
    assert "all tolerances passed" in txt
    assert "metric x = 1.234" in txt

    r_bad = mod.Stage1Result(
        controller="c",
        joint="j",
        metrics={"x": 9.0},
        failures=("drift too high",),
    )
    assert r_bad.ok is False
    assert "FAIL drift too high" in r_bad.format()


# ---------------------------------------------------------------------------
# Position-mode evaluator
# ---------------------------------------------------------------------------


POS_TOL = {
    "steady_state_err_rad": 0.01,
    "drift_rad_per_10s": 0.001,
    "fft_peak_db_above_noise_floor": 6.0,
}


def test_position_mode_passes_on_clean_regulation(mod):
    # Signal: q held exactly at target, no velocity.
    n = 2000
    dt = 0.005
    times = _uniform_times(n, dt)
    target = 0.25
    positions = [target for _ in range(n)]
    velocities = [0.0 for _ in range(n)]

    c = _make_controller("jtc", "first_order_lag", "position", POS_TOL)
    r = mod.evaluate_position_mode(
        c,
        joint="elbow_joint",
        times=times,
        positions=positions,
        velocities=velocities,
        target=target,
        drift_window_s=5.0,
    )
    assert r.ok, r.format()
    assert r.metrics["steady_state_err_rad"] == 0.0
    assert r.metrics["drift_rad"] == 0.0


def test_position_mode_fails_on_steady_state_offset(mod):
    n = 2000
    dt = 0.005
    times = _uniform_times(n, dt)
    target = 0.25
    positions = [target + 0.05 for _ in range(n)]  # well above 0.01 tol
    velocities = [0.0 for _ in range(n)]

    c = _make_controller("jtc", "first_order_lag", "position", POS_TOL)
    r = mod.evaluate_position_mode(
        c,
        joint="elbow_joint",
        times=times,
        positions=positions,
        velocities=velocities,
        target=target,
        drift_window_s=5.0,
    )
    assert not r.ok
    assert any("steady_state_err_rad" in f for f in r.failures)


def test_position_mode_fails_on_drift(mod):
    # Linear ramp of 0.01 rad over 10 s ⇒ drift over trailing 5 s ≈
    # 0.005 rad, well above the 1e-3 tol while staying below the
    # steady_state_err tol at 5 s (mean offset ≈ 0.0075 < 0.01).
    n = 2000
    dt = 0.005
    times = _uniform_times(n, dt)
    target = 0.25
    positions = [target + 0.01 * i / (n - 1) for i in range(n)]
    velocities = [0.0 for _ in range(n)]

    c = _make_controller("jtc", "first_order_lag", "position", POS_TOL)
    r = mod.evaluate_position_mode(
        c,
        joint="elbow_joint",
        times=times,
        positions=positions,
        velocities=velocities,
        target=target,
        drift_window_s=5.0,
    )
    # steady-state-err will likely also fail; we specifically check drift is reported.
    assert any("drift_rad" in f for f in r.failures)


def test_position_mode_fails_on_fft_limit_cycle(mod):
    # Velocity carries a clean 5 Hz sine well above the 6 dB threshold.
    n = 2000
    dt = 0.005  # Fs = 200 Hz, Nyquist 100 Hz.
    times = _uniform_times(n, dt)
    target = 0.25
    positions = [target for _ in range(n)]
    f0 = 5.0
    velocities = [0.1 * math.sin(2.0 * math.pi * f0 * t) for t in times]

    c = _make_controller("jtc", "first_order_lag", "position", POS_TOL)
    r = mod.evaluate_position_mode(
        c,
        joint="elbow_joint",
        times=times,
        positions=positions,
        velocities=velocities,
        target=target,
        drift_window_s=5.0,
        fft_f_min_hz=1.0,
    )
    assert not r.ok
    assert any("limit_cycle" in f for f in r.failures)
    # Peak should be near 5 Hz.
    assert abs(r.metrics["fft_peak_hz"] - f0) < 0.5


def test_position_mode_requires_matching_lengths(mod):
    c = _make_controller("jtc", "first_order_lag", "position", POS_TOL)
    with pytest.raises(ValueError):
        mod.evaluate_position_mode(
            c,
            joint="elbow_joint",
            times=[0.0, 0.01, 0.02],
            positions=[0.0, 0.0],
            velocities=[0.0, 0.0, 0.0],
            target=0.0,
        )


# ---------------------------------------------------------------------------
# Open-loop effort evaluator
# ---------------------------------------------------------------------------


EFFORT_TOL = {
    "max_runaway_rad": 0.5,
    "saturation_hold_ms": 100.0,
}


def test_open_loop_effort_passes_on_small_runaway(mod):
    n = 200
    dt = 0.01
    times = _uniform_times(n, dt)
    home = 0.0
    positions = [0.001 * i / n for i in range(n)]  # tiny drift

    c = _make_controller("fwd_effort", "open_loop_torque", "effort", EFFORT_TOL)
    r = mod.evaluate_open_loop_effort(
        c,
        joint="elbow_joint",
        times=times,
        positions=positions,
        home=home,
    )
    assert r.ok, r.format()
    assert r.metrics["max_runaway_rad"] < 0.01


def test_open_loop_effort_fails_on_runaway(mod):
    n = 200
    dt = 0.01
    times = _uniform_times(n, dt)
    home = 0.0
    positions = [0.01 * i for i in range(n)]  # runs to ~2 rad

    c = _make_controller("fwd_effort", "open_loop_torque", "effort", EFFORT_TOL)
    r = mod.evaluate_open_loop_effort(
        c,
        joint="elbow_joint",
        times=times,
        positions=positions,
        home=home,
    )
    assert not r.ok
    assert any("max_runaway_rad" in f for f in r.failures)


def test_open_loop_effort_saturation_hold_check(mod):
    # 0.2 s sampled at 1 kHz; torque saturates for 150 ms contiguously
    # at 28 Nm; tol is 100 ms ⇒ fail.
    dt = 0.001
    n = 200
    times = _uniform_times(n, dt)
    positions = [0.0 for _ in range(n)]
    # Saturation mask: samples 20..170 (inclusive) ⇒ 150 ms long.
    torques = []
    for i in range(n):
        if 20 <= i <= 170:
            torques.append(28.0)
        else:
            torques.append(1.0)

    c = _make_controller("fwd_effort", "open_loop_torque", "effort", EFFORT_TOL)
    r = mod.evaluate_open_loop_effort(
        c,
        joint="elbow_joint",
        times=times,
        positions=positions,
        home=0.0,
        torques=torques,
        effort_limit_nm=28.0,
    )
    assert not r.ok
    assert any("saturation_hold_ms" in f for f in r.failures)
    assert r.metrics["longest_saturation_hold_ms"] == pytest.approx(150.0, rel=1e-6)


def test_open_loop_effort_saturation_hold_passes_short_bursts(mod):
    dt = 0.001
    n = 200
    times = _uniform_times(n, dt)
    positions = [0.0 for _ in range(n)]
    # Two 50-ms bursts separated by a gap ⇒ longest run = 50 ms < 100 ms.
    torques = []
    for i in range(n):
        in_burst_1 = 20 <= i <= 70
        in_burst_2 = 120 <= i <= 170
        torques.append(28.0 if (in_burst_1 or in_burst_2) else 1.0)

    c = _make_controller("fwd_effort", "open_loop_torque", "effort", EFFORT_TOL)
    r = mod.evaluate_open_loop_effort(
        c,
        joint="elbow_joint",
        times=times,
        positions=positions,
        home=0.0,
        torques=torques,
        effort_limit_nm=28.0,
    )
    assert r.ok, r.format()
    assert r.metrics["longest_saturation_hold_ms"] == pytest.approx(50.0, rel=1e-6)


def test_open_loop_effort_skips_saturation_without_torques(mod):
    c = _make_controller("fwd_effort", "open_loop_torque", "effort", EFFORT_TOL)
    r = mod.evaluate_open_loop_effort(
        c,
        joint="elbow_joint",
        times=[0.0, 0.01],
        positions=[0.0, 0.0],
        home=0.0,
    )
    assert r.ok
    assert "longest_saturation_hold_ms" not in r.metrics


# ---------------------------------------------------------------------------
# Second-order evaluator
# ---------------------------------------------------------------------------


SECOND_ORDER_TOL = {
    "bounded_err_rad": 0.15,
    "chatter_velocity_rms_rad_s": 0.05,
    "damping_ratio_pct": 20.0,
}


def _synthesize_underdamped_step(
    omega_n: float, zeta: float, target: float, duration: float, dt: float
):
    """Analytical underdamped 2nd-order step response starting at 0.

    q(t) = target * [1 - e^{-zωt} (cos(ω_d t) + z/√(1-z²) sin(ω_d t))]
    """
    assert 0.0 < zeta < 1.0
    wd = omega_n * math.sqrt(1 - zeta * zeta)
    n = int(duration / dt) + 1
    times = _uniform_times(n, dt)
    positions = []
    velocities = []
    for t in times:
        env = math.exp(-zeta * omega_n * t)
        c = math.cos(wd * t)
        s = math.sin(wd * t)
        q = target * (1.0 - env * (c + (zeta / math.sqrt(1 - zeta * zeta)) * s))
        # qdot: derivative of q(t). Could compute analytically; use finite
        # diff for simplicity since we only need it for RMS / FFT, both
        # tolerant to a boundary artefact or two.
        positions.append(q)
    for i in range(n):
        if i == 0:
            velocities.append(0.0)
        else:
            velocities.append((positions[i] - positions[i - 1]) / dt)
    return times, positions, velocities


def test_second_order_passes_when_measured_matches_theory(mod):
    # Pick K, D, J so theoretical zeta = 0.5 exactly, omega_n = 10.
    # K = J * omega_n^2; D = 2 * zeta * sqrt(K J).
    j_eff = 0.10
    omega_n = 10.0
    zeta = 0.5
    K = j_eff * omega_n * omega_n  # 10.0
    D = 2.0 * zeta * math.sqrt(K * j_eff)  # = 2 * 0.5 * 1.0 = 1.0
    target = 0.10  # keep inside bounded_err_rad tol

    # Simulate with exactly these K,D,J ⇒ measured ζ should match theory.
    times, positions, velocities = _synthesize_underdamped_step(
        omega_n, zeta, target, duration=5.0, dt=0.002
    )
    c = _make_controller("crisp_joint_impedance", "second_order", "effort", SECOND_ORDER_TOL)
    j = _make_joint(j_eff=j_eff)
    r = mod.evaluate_second_order(
        c,
        j,
        stiffness_k=K,
        damping_d=D,
        times=times,
        positions=positions,
        velocities=velocities,
        target=target,
    )
    assert r.ok, r.format()
    assert abs(r.metrics["measured_zeta"] - zeta) < 0.05
    assert r.metrics["theoretical_zeta"] == pytest.approx(zeta, rel=1e-9)


def test_second_order_fails_when_damping_outside_band(mod):
    # Simulate with zeta = 0.2 but claim theoretical zeta = 0.5 via K/D
    # inputs. That's > ±20% ⇒ fail.
    j_eff = 0.10
    omega_n = 10.0
    simulated_zeta = 0.2
    claimed_zeta = 0.5
    K = j_eff * omega_n * omega_n
    D_claim = 2.0 * claimed_zeta * math.sqrt(K * j_eff)
    target = 0.10

    times, positions, velocities = _synthesize_underdamped_step(
        omega_n, simulated_zeta, target, duration=5.0, dt=0.002
    )
    c = _make_controller("crisp_joint_impedance", "second_order", "effort", SECOND_ORDER_TOL)
    j = _make_joint(j_eff=j_eff)
    r = mod.evaluate_second_order(
        c,
        j,
        stiffness_k=K,
        damping_d=D_claim,
        times=times,
        positions=positions,
        velocities=velocities,
        target=target,
    )
    assert not r.ok
    assert any("damping_ratio" in f for f in r.failures)


def test_second_order_fails_on_peak_error(mod):
    # Bounded-err tol is 0.15; craft a response whose peak exceeds it.
    j_eff = 0.10
    omega_n = 10.0
    zeta = 0.05  # very underdamped ⇒ ~85% overshoot on first peak
    K = j_eff * omega_n * omega_n
    D = 2.0 * zeta * math.sqrt(K * j_eff)
    # Overshoot = target * exp(-zeta*pi/sqrt(1-zeta^2)) ≈ 0.854*target.
    # Pick target=0.20 ⇒ peak_err ≈ 0.171 > 0.15 bounded_err tol.
    target = 0.20

    times, positions, velocities = _synthesize_underdamped_step(
        omega_n, zeta, target, duration=5.0, dt=0.002
    )
    c = _make_controller("crisp_joint_impedance", "second_order", "effort", SECOND_ORDER_TOL)
    j = _make_joint(j_eff=j_eff)
    r = mod.evaluate_second_order(
        c,
        j,
        stiffness_k=K,
        damping_d=D,
        times=times,
        positions=positions,
        velocities=velocities,
        target=target,
    )
    assert not r.ok
    assert any("peak_err_rad" in f for f in r.failures)


def test_second_order_fails_on_chatter(mod):
    # Synthesize a settled-at-target response (zeta=0.9, smooth) then
    # overlay high-frequency noise on velocity to blow the chatter tol.
    j_eff = 0.10
    omega_n = 10.0
    zeta = 0.9
    K = j_eff * omega_n * omega_n
    D = 2.0 * zeta * math.sqrt(K * j_eff)
    target = 0.10

    times, positions, velocities = _synthesize_underdamped_step(
        omega_n, zeta, target, duration=5.0, dt=0.002
    )
    # Inject chatter: ±0.2 rad/s velocity noise on the last 2 seconds.
    import random

    random.seed(0)
    chatter = [(random.random() - 0.5) * 0.4 for _ in times]
    velocities_noisy = [v + c_ for v, c_ in zip(velocities, chatter)]

    c = _make_controller("crisp_joint_impedance", "second_order", "effort", SECOND_ORDER_TOL)
    j = _make_joint(j_eff=j_eff)
    r = mod.evaluate_second_order(
        c,
        j,
        stiffness_k=K,
        damping_d=D,
        times=times,
        positions=positions,
        velocities=velocities_noisy,
        target=target,
    )
    assert not r.ok
    assert any("velocity_rms_rad_s" in f for f in r.failures)


def test_second_order_overdamped_theory_accepts_none_measurement(mod):
    # Theory: zeta >> 1 ⇒ overdamped. Simulate a monotonic response
    # (single exponential) so damping_ratio_from_step returns None.
    j_eff = 0.10
    K = 5.0
    D = 20.0  # very overdamped: zeta = 20 / (2 * sqrt(5 * 0.1)) ≈ 14.1
    target = 0.10
    dt = 0.01
    n = 500
    times = _uniform_times(n, dt)
    # Simple exponential approach — no extrema, no overshoot.
    positions = [target * (1.0 - math.exp(-5.0 * t)) for t in times]
    velocities = [0.0] + [(positions[i] - positions[i - 1]) / dt for i in range(1, n)]
    c = _make_controller("crisp_joint_impedance", "second_order", "effort", SECOND_ORDER_TOL)
    j = _make_joint(j_eff=j_eff)
    r = mod.evaluate_second_order(
        c,
        j,
        stiffness_k=K,
        damping_d=D,
        times=times,
        positions=positions,
        velocities=velocities,
        target=target,
    )
    assert r.ok, r.format()
    assert r.metrics["theoretical_zeta"] > 1.0
    assert math.isnan(r.metrics["measured_zeta"])


def test_second_order_underdamped_theory_rejects_none_measurement(mod):
    # Theory: zeta = 0.5 (underdamped) but we feed a *flat* signal so
    # the measurement returns None ⇒ must fail.
    j_eff = 0.10
    omega_n = 10.0
    zeta = 0.5
    K = j_eff * omega_n * omega_n
    D = 2.0 * zeta * math.sqrt(K * j_eff)
    target = 0.10
    n = 500
    dt = 0.01
    times = _uniform_times(n, dt)
    positions = [target for _ in range(n)]  # already at target ⇒ no extrema
    velocities = [0.0 for _ in range(n)]

    c = _make_controller("crisp_joint_impedance", "second_order", "effort", SECOND_ORDER_TOL)
    j = _make_joint(j_eff=j_eff)
    r = mod.evaluate_second_order(
        c,
        j,
        stiffness_k=K,
        damping_d=D,
        times=times,
        positions=positions,
        velocities=velocities,
        target=target,
    )
    assert not r.ok
    assert any("damping_ratio" in f and "None" in f for f in r.failures)


# ---------------------------------------------------------------------------
# Missing-tolerance guard
# ---------------------------------------------------------------------------


def test_missing_tolerance_key_raises_keyerror(mod):
    # Position-mode evaluator should raise when the expectation lacks a
    # required key rather than silently passing.
    c = _make_controller(
        "jtc",
        "first_order_lag",
        "position",
        {"steady_state_err_rad": 0.01},  # missing drift + fft
    )
    n = 200
    times = _uniform_times(n)
    positions = [0.25 for _ in range(n)]
    velocities = [0.0 for _ in range(n)]
    with pytest.raises(KeyError):
        mod.evaluate_position_mode(
            c,
            joint="elbow_joint",
            times=times,
            positions=positions,
            velocities=velocities,
            target=0.25,
            drift_window_s=1.0,
        )
