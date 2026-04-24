"""R2 stage-1 assertion harness: expectations ↔ measured signals.

Pre-baked consumer API for the R2 integration test matrix (M6.12, still
gated on the M6.0 operator decision). Pairs the two sides already in
place —

* ``tests/integration/expectations_loader.py`` — typed access to the
  per-arm / per-controller tolerance tables in
  ``tests/integration/expectations/*.yaml``.
* ``tests/integration/signal_analysis.py`` — pure-stdlib metrics on
  sampled ``/joint_states`` traces (steady-state error, drift, velocity
  RMS, limit-cycle FFT, damping-ratio-from-step).

— into a single callable per controller *response model*, so an R2
stage-1 test body reduces to:

.. code-block:: python

    arm = load_arm("ur5e")
    c = arm.controller("joint_trajectory_controller")
    result = evaluate_position_mode(
        c, times=ts, positions=qs, velocities=qdots, target=q_d
    )
    assert result.ok, result.format()

Design choices
--------------

* **Functional, no globals.** Every evaluator takes the expectation
  object plus raw measured arrays and returns a :class:`Stage1Result`.
  Tests decide whether ``result.failures`` is an assertion or a warning
  (e.g. the draft-expectations case today).
* **No side-effects on ``None`` failures.** Returning a result instead
  of raising lets the unit-test gate exercise both pass and fail paths
  without try/except gymnastics and lets the R2 tests accumulate
  metrics into run artefacts (``evaluation/runs/<ts>/``) regardless of
  outcome — ADR-0009 / R2 text in ``docs/ROADMAP.md``.
* **Dependency posture matches siblings.** Pure stdlib; imports
  ``signal_analysis`` and ``expectations_loader`` via ``importlib``
  because ``tests/integration/`` is not on ``sys.path`` as a package
  (matches ``evaluation/run_evaluation.py`` convention).
* **dB ↔ linear conversion for FFT.** The expectation YAMLs carry
  ``fft_peak_db_above_noise_floor`` (dB above a median-based noise
  floor); ``signal_analysis.detect_limit_cycle`` works in linear
  magnitude ratio. The evaluator converts internally via
  ``ratio = 10 ** (db / 20)`` (amplitude dB, matching R2 text in
  ROADMAP §M6 R2).

Scope
-----

Stage 1 only (single-joint motion, other five held at home). Stage 2
(all joints together) and stage 3 (TCP trajectory) will add their own
evaluators in M6.13 / M6.14 follow-ups; they can reuse this module's
:class:`Stage1Result` pattern but have different signal/expectation
pairings.
"""

from __future__ import annotations

import importlib.util
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Mapping, Optional, Sequence, Tuple

# ---------------------------------------------------------------------------
# Sibling loading (tests/integration/ is not a package; follow the
# evaluation/ and test_expectations_loader.py convention)
# ---------------------------------------------------------------------------

_HERE = Path(__file__).resolve().parent


def _load_sibling(name: str):
    key = f"_r2_{name}"
    mod = sys.modules.get(key)
    if mod is not None:
        return mod
    spec = importlib.util.spec_from_file_location(key, _HERE / f"{name}.py")
    assert spec and spec.loader, f"cannot locate sibling module {name!r}"
    mod = importlib.util.module_from_spec(spec)
    sys.modules[key] = mod
    spec.loader.exec_module(mod)
    return mod


signal_analysis = _load_sibling("signal_analysis")
expectations_loader = _load_sibling("expectations_loader")


__all__ = (
    "Stage1Result",
    "evaluate_open_loop_effort",
    "evaluate_position_mode",
    "evaluate_second_order",
)


# ---------------------------------------------------------------------------
# Result dataclass
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Stage1Result:
    """Outcome of a single controller×joint R2 stage-1 evaluation.

    Attributes
    ----------
    controller, joint:
        Echo of the inputs, so a list of results can be displayed in a
        single table.
    metrics:
        Computed scalar metrics — e.g. ``steady_state_err_rad``,
        ``measured_zeta``. Always populated when the metric could be
        computed; missing keys denote "not applicable" (e.g. theory
        overdamped ⇒ no measured ``zeta``).
    failures:
        Human-readable one-line strings, one per violated tolerance.
        Empty tuple ⇒ all checks passed.
    """

    controller: str
    joint: str
    metrics: Mapping[str, float] = field(default_factory=dict)
    failures: Tuple[str, ...] = ()

    @property
    def ok(self) -> bool:
        return not self.failures

    def format(self) -> str:
        """Multi-line diagnostic string for pytest ``assert`` messages."""
        head = f"[R2 stage-1] controller={self.controller} joint={self.joint}"
        metric_lines = [f"  metric {k} = {v:.6g}" for k, v in sorted(self.metrics.items())]
        fail_lines = [f"  FAIL {f}" for f in self.failures]
        body = metric_lines + (fail_lines if fail_lines else ["  all tolerances passed"])
        return "\n".join([head, *body])


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _db_to_ratio(db: float) -> float:
    """Amplitude dB → linear magnitude ratio: ``10 ** (db / 20)``."""
    return 10.0 ** (float(db) / 20.0)


def _validate_traces(*arrays: Sequence[float]) -> None:
    if not arrays:
        return
    n = len(arrays[0])
    for a in arrays[1:]:
        if len(a) != n:
            raise ValueError(
                f"r2_stage1_assertions: mismatched trace lengths {[len(x) for x in arrays]}"
            )


# ---------------------------------------------------------------------------
# Position-mode evaluator (JTC, forward_position, forward_velocity)
# ---------------------------------------------------------------------------


def evaluate_position_mode(
    controller_exp: "expectations_loader.ControllerExpectation",
    *,
    joint: str,
    times: Sequence[float],
    positions: Sequence[float],
    velocities: Sequence[float],
    target: float,
    drift_window_s: float = 10.0,
    steady_state_window_s: Optional[float] = None,
    fft_f_min_hz: float = 1.0,
    fft_f_max_hz: Optional[float] = None,
) -> Stage1Result:
    """Evaluate stage-1 R2 tolerances for a *position-mode* controller.

    Checks, in order:

    1. ``steady_state_err_rad`` — trailing-window mean error vs ``target``.
    2. ``drift_rad_per_10s`` — peak-to-peak drift over ``drift_window_s``
       (default 10 s, matching the R2 text in
       ``docs/ROADMAP.md`` §M6 R2).
    3. ``fft_peak_db_above_noise_floor`` — no limit-cycle peak above the
       configured dB threshold in ``[fft_f_min_hz, Nyquist]`` on the
       *velocity* trace. R2 explicitly runs this on the joint-velocity
       signal (oscillation is more visible in velocity than in the
       position residual at small amplitudes).

    Parameters
    ----------
    controller_exp:
        The per-controller block from
        :func:`expectations_loader.load_arm`. Must expose the three
        tolerance keys above; absence raises ``KeyError`` at check time.
    joint:
        Joint name, used in :class:`Stage1Result` only.
    times, positions, velocities:
        Synchronous, equal-length arrays sampled from
        ``/joint_states``.
    target:
        Commanded position (rad) held over the window (stage 1 regulates
        a single joint).
    drift_window_s:
        Size of the trailing window for the drift check. Defaults to
        10 s to match the ROADMAP R2 text.
    steady_state_window_s:
        Size of the trailing window for the steady-state check. Defaults
        to ``drift_window_s`` when ``None`` — the R2 text uses the same
        hold window for both metrics.
    fft_f_min_hz, fft_f_max_hz:
        Band for the FFT limit-cycle detector. ``fft_f_max_hz`` defaults
        to the signal's Nyquist frequency (the
        ``signal_analysis.detect_limit_cycle`` internal cap).
    """
    _validate_traces(times, positions, velocities)
    if steady_state_window_s is None:
        steady_state_window_s = drift_window_s
    metrics: Dict[str, float] = {}
    failures: list = []

    # 1. Steady-state error.
    sse = signal_analysis.steady_state_error(times, positions, target, steady_state_window_s)
    metrics["steady_state_err_rad"] = sse
    tol_sse = controller_exp.tol("steady_state_err_rad")
    if sse > tol_sse:
        failures.append(
            f"steady_state_err_rad={sse:.6g} > tol={tol_sse:.6g} "
            f"(window={steady_state_window_s}s)"
        )

    # 2. Drift peak-to-peak.
    drift = signal_analysis.drift_peak_to_peak(times, positions, drift_window_s)
    metrics["drift_rad"] = drift
    tol_drift = controller_exp.tol("drift_rad_per_10s")
    if drift > tol_drift:
        failures.append(
            f"drift_rad={drift:.6g} > tol={tol_drift:.6g} " f"(window={drift_window_s}s)"
        )

    # 3. FFT limit cycle on velocity.
    db_thresh = controller_exp.tol("fft_peak_db_above_noise_floor")
    ratio_thresh = _db_to_ratio(db_thresh)
    # Nyquist from median sample period; the detector also clips internally.
    if fft_f_max_hz is None:
        # Pass a very large value; detector clips to Nyquist internally.
        fft_f_max_hz = math.inf
    detected, peak_hz, ratio = signal_analysis.detect_limit_cycle(
        times,
        velocities,
        f_min_hz=fft_f_min_hz,
        f_max_hz=fft_f_max_hz,
        peak_ratio=ratio_thresh,
    )
    metrics["fft_peak_ratio"] = ratio
    if peak_hz is not None:
        metrics["fft_peak_hz"] = peak_hz
    if detected:
        peak_db = 20.0 * math.log10(ratio) if ratio > 0.0 else float("inf")
        failures.append(
            f"limit_cycle peak_hz={peak_hz:.4g} ratio={ratio:.3g} "
            f"({peak_db:.2f} dB) > tol={db_thresh:.2f} dB"
        )

    return Stage1Result(
        controller=controller_exp.name,
        joint=joint,
        metrics=metrics,
        failures=tuple(failures),
    )


# ---------------------------------------------------------------------------
# Open-loop-effort evaluator (forward_effort_controller)
# ---------------------------------------------------------------------------


def evaluate_open_loop_effort(
    controller_exp: "expectations_loader.ControllerExpectation",
    *,
    joint: str,
    times: Sequence[float],
    positions: Sequence[float],
    home: float,
    torques: Optional[Sequence[float]] = None,
    effort_limit_nm: Optional[float] = None,
) -> Stage1Result:
    """Evaluate stage-1 R2 tolerances for the *open-loop effort* mode.

    ``forward_effort_controller`` has no closed-loop law: R2 stage-1 only
    asserts that during the short probe window the joint does not
    *run away* (``max_runaway_rad``) and that if a commanded torque
    trace is supplied it does not hold saturation for longer than
    ``saturation_hold_ms``.

    Parameters
    ----------
    controller_exp:
        Per-controller block. Must expose ``max_runaway_rad``;
        ``saturation_hold_ms`` is only consulted when ``torques`` and
        ``effort_limit_nm`` are both provided.
    joint:
        Joint name, echoed in the result.
    times, positions:
        Measured trace. ``times`` must be monotonic; ``positions`` must
        match in length.
    home:
        Position (rad) the joint is regulated to *before* the effort
        probe fires — the runaway check measures ``|q(t) - home|``.
    torques:
        Optional commanded-torque trace (Nm), same length as ``times``.
        When supplied alongside ``effort_limit_nm``, the evaluator
        checks the longest contiguous run where ``|tau| >=
        effort_limit_nm`` does not exceed ``saturation_hold_ms``.
    effort_limit_nm:
        The arm's per-joint effort bound
        (:class:`JointExpectation.effort_limit_nm`). Required for the
        saturation check; ``None`` ⇒ skip that check and record nothing
        in ``metrics`` for it.
    """
    _validate_traces(times, positions)
    if torques is not None:
        _validate_traces(times, torques)
    metrics: Dict[str, float] = {}
    failures: list = []

    # 1. Runaway bound.
    runaway = max(abs(p - home) for p in positions) if positions else 0.0
    metrics["max_runaway_rad"] = runaway
    tol_runaway = controller_exp.tol("max_runaway_rad")
    if runaway > tol_runaway:
        failures.append(
            f"max_runaway_rad={runaway:.6g} > tol={tol_runaway:.6g} " f"(home={home:.6g})"
        )

    # 2. Saturation hold (optional).
    if torques is not None and effort_limit_nm is not None:
        tol_hold_ms = controller_exp.tol("saturation_hold_ms")
        longest_ms = _longest_contiguous_ms(
            times, [abs(t) >= float(effort_limit_nm) for t in torques]
        )
        metrics["longest_saturation_hold_ms"] = longest_ms
        if longest_ms > tol_hold_ms:
            failures.append(
                f"longest_saturation_hold_ms={longest_ms:.6g} > " f"tol={tol_hold_ms:.6g}"
            )

    return Stage1Result(
        controller=controller_exp.name,
        joint=joint,
        metrics=metrics,
        failures=tuple(failures),
    )


def _longest_contiguous_ms(times: Sequence[float], mask: Sequence[bool]) -> float:
    """Longest contiguous *True* run in ``mask`` expressed in milliseconds.

    The duration of a run spanning indices ``[i..j]`` is
    ``times[j] - times[i]`` — not ``(j - i + 1) * dt`` — so non-uniform
    sampling is handled correctly and a single-sample run reports 0 ms.
    """
    if len(times) != len(mask):
        raise ValueError(
            f"r2_stage1_assertions: mismatched times/mask lengths {len(times)} vs {len(mask)}"
        )
    best = 0.0
    run_start: Optional[int] = None
    for i, flag in enumerate(mask):
        if flag:
            if run_start is None:
                run_start = i
            run_len = times[i] - times[run_start]
            if run_len > best:
                best = run_len
        else:
            run_start = None
    return best * 1000.0


# ---------------------------------------------------------------------------
# Second-order evaluator (crisp_joint_impedance, simple_joint_impedance)
# ---------------------------------------------------------------------------


def evaluate_second_order(
    controller_exp: "expectations_loader.ControllerExpectation",
    joint_exp: "expectations_loader.JointExpectation",
    *,
    stiffness_k: float,
    damping_d: float,
    times: Sequence[float],
    positions: Sequence[float],
    velocities: Sequence[float],
    target: float,
    chatter_window_s: float = 2.0,
) -> Stage1Result:
    """Evaluate stage-1 R2 tolerances for a *second-order impedance* mode.

    Checks:

    1. ``bounded_err_rad`` — max absolute error ``|q(t) - target|`` over
       the whole trace (the second-order response should settle inside
       this bound even at the peak of the first overshoot).
    2. ``chatter_velocity_rms_rad_s`` — velocity RMS over the trailing
       ``chatter_window_s`` (default 2 s, matching the R2 text).
    3. ``damping_ratio_pct`` — measured ζ (via
       :func:`signal_analysis.damping_ratio_from_step`) within the
       configured ±% band of theoretical ζ computed from
       ``K``, ``D``, ``J_eff`` via
       :func:`expectations_loader.second_order_response`. When theory
       is *overdamped* (ζ ≥ 1) and the measurement returns ``None``
       (no two extrema ⇒ no oscillation), the check passes.

    Parameters
    ----------
    controller_exp:
        Per-controller block. Must expose the three tolerance keys above.
    joint_exp:
        Per-joint expectation, used for
        :attr:`JointExpectation.effective_inertia_kg_m2` — the reflected
        inertia at the home pose that feeds the ``omega_n``/``zeta``
        formula (see ADR-0013).
    stiffness_k, damping_d:
        Configured ``K`` and ``D`` for this joint on this controller,
        read from ``bringup/config/*.yaml`` by the caller. Kept as
        arguments (not pulled from YAML here) so this module stays
        decoupled from the controller configs.
    times, positions, velocities:
        Synchronous traces.
    target:
        Step target (rad).
    chatter_window_s:
        Size of the trailing window for the velocity-RMS chatter check.
    """
    _validate_traces(times, positions, velocities)
    metrics: Dict[str, float] = {}
    failures: list = []

    # Theoretical response from K, D, J_eff.
    _, theoretical_zeta = expectations_loader.second_order_response(
        stiffness_k, damping_d, joint_exp.effective_inertia_kg_m2
    )
    metrics["theoretical_zeta"] = theoretical_zeta

    # 1. Bounded error (peak excursion from target).
    peak_err = max(abs(p - target) for p in positions) if positions else 0.0
    metrics["peak_err_rad"] = peak_err
    tol_err = controller_exp.tol("bounded_err_rad")
    if peak_err > tol_err:
        failures.append(f"peak_err_rad={peak_err:.6g} > tol={tol_err:.6g}")

    # 2. Velocity RMS chatter.
    vrms = signal_analysis.velocity_rms(times, velocities, chatter_window_s)
    metrics["velocity_rms_rad_s"] = vrms
    tol_vrms = controller_exp.tol("chatter_velocity_rms_rad_s")
    if vrms > tol_vrms:
        failures.append(
            f"velocity_rms_rad_s={vrms:.6g} > tol={tol_vrms:.6g} " f"(window={chatter_window_s}s)"
        )

    # 3. Damping ratio vs theory.
    tol_pct = controller_exp.tol("damping_ratio_pct")
    measured_zeta = signal_analysis.damping_ratio_from_step(times, positions, target)
    if measured_zeta is None:
        if theoretical_zeta >= 1.0:
            # Overdamped theory + no oscillation observed ⇒ consistent.
            metrics["measured_zeta"] = float("nan")
        else:
            failures.append(
                f"damping_ratio: measurement returned None "
                f"(fewer than 2 extrema) but theory predicts underdamped "
                f"zeta={theoretical_zeta:.4g}"
            )
    else:
        metrics["measured_zeta"] = measured_zeta
        if not expectations_loader.damping_ratio_within_band(
            measured_zeta, theoretical_zeta, tol_pct
        ):
            failures.append(
                f"damping_ratio: measured={measured_zeta:.4g} outside "
                f"±{tol_pct:.1f}% band of theory={theoretical_zeta:.4g}"
            )

    return Stage1Result(
        controller=controller_exp.name,
        joint=joint_exp.name,
        metrics=metrics,
        failures=tuple(failures),
    )
