#!/usr/bin/env python3
"""Compute evaluation metrics for an M5 run directory (M5 bullet 3).

Consumes a run directory produced by ``evaluation/run_evaluation.py``::

    <run_dir>/
        manifest.yaml      # written by the runner
        target.csv         # planned reference (t_s + per-joint commands)
        joint_states.csv   # observed /joint_states (canonical UR joint order)
        tau_d.csv          # observed ~/tau_d (joint-impedance controllers only)

and writes two artefacts next to the inputs:

    <run_dir>/metrics.csv
    <run_dir>/metrics.yaml

The CLI exits ``0`` when every key listed under ``manifest.scenario.pass_criteria``
either passes its threshold or is recorded as ``not_applicable`` /
``skipped``. It exits ``1`` when any threshold is exceeded, and ``2`` on
malformed inputs (missing files, schema mismatch, etc.).

Supported metrics in v1:

* ``rmse`` — joint-space root-mean-square error in rad. Per joint and
  aggregated as the max across joints (the worst joint sets the bound).
* ``settling_time`` — for ``scenario_type == step`` only: time from the
  first sample at or after ``step_time_s`` until the joint stays within
  a 5%-of-amplitude (≥0.01 rad floor) tolerance band of the post-step
  target until the end of the run. Reported per joint as ``inf`` when the
  joint never settles. Aggregate is the max across joints with a non-zero
  amplitude. For other ``scenario_type`` values this metric is recorded
  as ``not_applicable`` and never fails the pass gate.
* ``overshoot`` — ``step`` only, percent of the step amplitude that the
  observed trajectory exceeded the target on the leading transient
  (max signed overshoot per joint, divided by ``|amplitude|``,
  multiplied by 100). ``not_applicable`` for non-step scenarios.
* ``control_effort`` — peak ``|tau|`` across joints and time, in N·m.
  Read from ``tau_d.csv``. ``skipped`` when ``tau_d.csv`` is absent
  (cartesian motion controller); never fails the pass gate when
  skipped.

Cartesian scenarios (``target.space == cartesian``) are documented as
deferred in v1: this script writes a metrics file with every metric
recorded as ``skipped`` (forward kinematics required, see
``docs/STATUS.md``) and exits ``0``. The comparison report (M5 bullet 4)
will surface these as "not yet evaluated".
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]

# ---------------------------------------------------------------------------
# CSV loading
# ---------------------------------------------------------------------------


@dataclass
class Series:
    """A time series read from a CSV file: ``t_s`` plus N value columns."""

    times: list[float]
    columns: dict[str, list[float]]

    @property
    def column_names(self) -> list[str]:
        return list(self.columns)


def load_series(path: Path) -> Series:
    """Load a CSV with header ``t_s,col1,col2,...`` into a :class:`Series`."""
    with path.open("r", encoding="utf-8") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if not header or header[0] != "t_s":
            raise ValueError(f"{path}: expected first column 't_s', got {header!r}")
        columns: dict[str, list[float]] = {name: [] for name in header[1:]}
        times: list[float] = []
        for row in reader:
            if not row:
                continue
            if len(row) != len(header):
                raise ValueError(f"{path}: row has {len(row)} cells, expected {len(header)}")
            try:
                times.append(float(row[0]))
                for name, cell in zip(header[1:], row[1:]):
                    columns[name].append(float(cell))
            except ValueError as e:
                raise ValueError(f"{path}: non-numeric cell: {e}") from e
    return Series(times=times, columns=columns)


# ---------------------------------------------------------------------------
# Resampling
# ---------------------------------------------------------------------------


def interp_at(times_src: list[float], values_src: list[float], t: float) -> float:
    """Linear interpolation of ``values_src(times_src)`` at ``t``.

    ``times_src`` must be non-empty and monotonically non-decreasing. Out-of-
    range queries are clamped to the nearest endpoint (zero-order hold at the
    boundaries).
    """
    if not times_src:
        raise ValueError("interp_at: empty source")
    if t <= times_src[0]:
        return values_src[0]
    if t >= times_src[-1]:
        return values_src[-1]
    # Binary search for the bracketing pair.
    lo, hi = 0, len(times_src) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if times_src[mid] <= t:
            lo = mid
        else:
            hi = mid
    t0, t1 = times_src[lo], times_src[hi]
    if t1 == t0:
        return values_src[lo]
    a = (t - t0) / (t1 - t0)
    return values_src[lo] + a * (values_src[hi] - values_src[lo])


def resample_series(src: Series, query_times: list[float]) -> dict[str, list[float]]:
    """Linearly resample every column of ``src`` onto ``query_times``."""
    out: dict[str, list[float]] = {}
    for name, vals in src.columns.items():
        out[name] = [interp_at(src.times, vals, t) for t in query_times]
    return out


# ---------------------------------------------------------------------------
# Metric computations (joint scenarios)
# ---------------------------------------------------------------------------


def rmse_per_joint(
    target: dict[str, list[float]], observed: dict[str, list[float]]
) -> dict[str, float]:
    """Per-joint RMSE (rad). ``target`` and ``observed`` must share the same
    keys and equal-length value lists."""
    out: dict[str, float] = {}
    for name, t_vals in target.items():
        o_vals = observed[name]
        if len(t_vals) != len(o_vals):
            raise ValueError(
                f"rmse_per_joint: length mismatch for {name!r}: " f"{len(t_vals)} vs {len(o_vals)}"
            )
        if not t_vals:
            out[name] = 0.0
            continue
        sq = sum((o - t) ** 2 for o, t in zip(o_vals, t_vals))
        out[name] = math.sqrt(sq / len(t_vals))
    return out


def settling_time_per_joint(
    times: list[float],
    target_post_step: dict[str, float],
    observed: dict[str, list[float]],
    amplitude: dict[str, float],
    step_time_s: float,
    tol_frac: float = 0.05,
    tol_floor: float = 0.01,
) -> dict[str, float]:
    """For each joint with non-zero ``amplitude``, return the time from
    ``step_time_s`` until the trajectory enters and stays within
    ``max(tol_frac * |amplitude|, tol_floor)`` of ``target_post_step``
    until the end of the window.

    Joints with zero amplitude are reported as ``0.0`` (already settled).
    Joints that never settle return ``math.inf``.
    """
    out: dict[str, float] = {}
    for name, amp in amplitude.items():
        target = target_post_step[name]
        obs = observed[name]
        if abs(amp) < 1e-12:
            out[name] = 0.0
            continue
        tol = max(tol_frac * abs(amp), tol_floor)
        # Walk from the end backwards: the settling time is the latest
        # excursion outside the band, measured from step_time_s.
        last_excursion_idx = -1
        for i, (t, v) in enumerate(zip(times, obs)):
            if t < step_time_s:
                continue
            if abs(v - target) > tol:
                last_excursion_idx = i
        if last_excursion_idx < 0:
            # Already inside the band at and after step_time_s.
            out[name] = 0.0
            continue
        if last_excursion_idx == len(times) - 1:
            out[name] = math.inf
            continue
        # The joint is back inside the band by the next sample; report that
        # instant (relative to step_time_s).
        settled_t = times[last_excursion_idx + 1]
        out[name] = max(0.0, settled_t - step_time_s)
    return out


def overshoot_per_joint(
    times: list[float],
    target_pre_step: dict[str, float],
    target_post_step: dict[str, float],
    observed: dict[str, list[float]],
    step_time_s: float,
) -> dict[str, float]:
    """Percent overshoot per joint relative to step amplitude.

    ``overshoot_pct = max(0, (peak - target) / |amplitude|) * 100`` where
    ``peak`` is the signed extremum of ``observed`` past ``target_post_step``
    in the direction of the step, evaluated only for samples with
    ``t >= step_time_s``. Joints with zero amplitude are reported as ``0.0``.
    """
    out: dict[str, float] = {}
    for name in target_post_step:
        amp = target_post_step[name] - target_pre_step[name]
        obs = observed[name]
        if abs(amp) < 1e-12:
            out[name] = 0.0
            continue
        sign = 1.0 if amp > 0 else -1.0
        peak_excess = 0.0
        for t, v in zip(times, obs):
            if t < step_time_s:
                continue
            excess = sign * (v - target_post_step[name])
            if excess > peak_excess:
                peak_excess = excess
        out[name] = 100.0 * peak_excess / abs(amp)
    return out


def control_effort_peak(tau_d: Series) -> tuple[float, dict[str, float]]:
    """Peak ``|tau|`` across the whole window. Returns ``(aggregate, per_joint)``."""
    per_joint: dict[str, float] = {}
    for name, vals in tau_d.columns.items():
        per_joint[name] = max((abs(v) for v in vals), default=0.0)
    aggregate = max(per_joint.values(), default=0.0)
    return aggregate, per_joint


# ---------------------------------------------------------------------------
# Pass-criteria evaluation
# ---------------------------------------------------------------------------

# Joint-space pass-criteria keys → (metric name, comparison kind).
# All current keys are upper-bound checks (value <= threshold).
_JOINT_PASS_KEYS: dict[str, str] = {
    "max_rmse_rad": "rmse",
    "max_settling_time_s": "settling_time",
    "max_overshoot_pct": "overshoot",
    "max_control_effort_nm": "control_effort",
}
_CARTESIAN_PASS_KEYS: dict[str, str] = {
    "max_rmse_m": "rmse_position",
    "max_rmse_rad": "rmse_orientation",
    "max_settling_time_s": "settling_time",
    "max_control_effort_nm": "control_effort",
}


# ---------------------------------------------------------------------------
# Top-level pipeline
# ---------------------------------------------------------------------------


def _need(path: Path) -> Path:
    if not path.exists():
        raise FileNotFoundError(f"required file missing: {path}")
    return path


def _amplitude_and_step_targets(
    scenario: dict, initial: list[float], joints: list[str]
) -> tuple[dict[str, float], dict[str, float], dict[str, float], float]:
    """For a ``step`` scenario return
    ``(amplitude, target_pre, target_post, step_time_s)``."""
    cmd = scenario.get("command", {})
    amp_vec = cmd["per_joint_amplitude_rad"]
    step_time_s = float(cmd["step_time_s"])
    if len(amp_vec) != len(joints) or len(initial) != len(joints):
        raise ValueError("step scenario: amplitude/initial length mismatch")
    amp = {n: float(amp_vec[i]) for i, n in enumerate(joints)}
    pre = {n: float(initial[i]) for i, n in enumerate(joints)}
    post = {n: pre[n] + amp[n] for n in joints}
    return amp, pre, post, step_time_s


def compute_metrics_for_run(run_dir: Path) -> dict:
    """Compute the metrics dictionary for ``run_dir`` (does not write files).

    Returns a dict of the shape that ``write_metrics_yaml`` expects.
    """
    manifest_path = _need(run_dir / "manifest.yaml")
    with manifest_path.open("r", encoding="utf-8") as f:
        manifest = yaml.safe_load(f)
    if not isinstance(manifest, dict):
        raise ValueError(f"{manifest_path}: not a YAML mapping")
    scenario = manifest.get("scenario") or {}
    metrics_listed = list(scenario.get("metrics") or [])
    pass_criteria = dict(scenario.get("pass_criteria") or {})
    controller = manifest.get("controller") or {}

    target_space = (
        manifest.get("target_space")
        or controller.get("target_space")
        # Fall back to the validated scenario file referenced in the manifest.
        or _scenario_target_space(run_dir, scenario)
    )

    result: dict = {
        "schema_version": 1,
        "kind": "evaluation_metrics",
        "run_dir": str(run_dir.relative_to(REPO_ROOT))
        if _is_relative_to(run_dir.resolve(), REPO_ROOT)
        else str(run_dir),
        "scenario_name": scenario.get("name"),
        "scenario_type": scenario.get("type"),
        "controller": controller.get("name"),
        "robot": manifest.get("robot"),
        "target_space": target_space,
        "metrics": {},
        "pass_criteria": pass_criteria,
        "pass_results": [],
        "overall_status": "pass",
    }

    if target_space == "cartesian":
        # Defer all cartesian metrics in v1 (need FK on the URDF chain to
        # convert /joint_states into a TCP pose for RMSE).
        for m in metrics_listed:
            result["metrics"][m] = {
                "status": "skipped",
                "reason": "cartesian metrics deferred (FK not implemented in v1)",
            }
        for key in pass_criteria:
            result["pass_results"].append(
                {
                    "key": key,
                    "threshold": pass_criteria[key],
                    "value": None,
                    "status": "skipped",
                    "reason": "cartesian metrics deferred in v1",
                }
            )
        return result

    # Joint-space metrics from here.
    target = load_series(_need(run_dir / "target.csv"))
    observed = load_series(_need(run_dir / "joint_states.csv"))
    if not target.times or not observed.times:
        raise ValueError("target.csv or joint_states.csv has no rows")
    if list(target.column_names) != list(observed.column_names):
        raise ValueError(
            "joint columns differ between target.csv and joint_states.csv: "
            f"{target.column_names} vs {observed.column_names}"
        )
    joints = list(target.column_names)
    # Resample observed onto the target time grid so RMSE etc. are aligned.
    observed_on_target = resample_series(observed, target.times)
    target_cols = target.columns

    if "rmse" in metrics_listed:
        per_joint = rmse_per_joint(target_cols, observed_on_target)
        agg = max(per_joint.values(), default=0.0)
        result["metrics"]["rmse"] = {
            "status": "ok",
            "unit": "rad",
            "aggregate": agg,
            "aggregate_kind": "max_across_joints",
            "per_joint": per_joint,
        }

    stype = scenario.get("type")
    if stype == "step":
        initial = manifest.get("initial_joint_positions")
        if not initial or len(initial) != len(joints):
            raise ValueError(
                "step scenario: manifest.initial_joint_positions missing or " "wrong length"
            )
        # Pull the step parameters from the canonical scenario file referenced
        # in the manifest. The manifest itself doesn't carry per_joint_amplitude.
        scen_doc = _load_scenario_doc(run_dir, scenario)
        amp, pre, post, step_time_s = _amplitude_and_step_targets(scen_doc, initial, joints)
        if "settling_time" in metrics_listed:
            per_joint = settling_time_per_joint(
                target.times, post, observed_on_target, amp, step_time_s
            )
            non_zero = [v for n, v in per_joint.items() if abs(amp[n]) > 0]
            agg = max(non_zero) if non_zero else 0.0
            result["metrics"]["settling_time"] = {
                "status": "ok",
                "unit": "s",
                "aggregate": agg if math.isfinite(agg) else float("inf"),
                "aggregate_kind": "max_across_active_joints",
                "per_joint": per_joint,
            }
        if "overshoot" in metrics_listed:
            per_joint = overshoot_per_joint(
                target.times, pre, post, observed_on_target, step_time_s
            )
            non_zero = [v for n, v in per_joint.items() if abs(amp[n]) > 0]
            agg = max(non_zero) if non_zero else 0.0
            result["metrics"]["overshoot"] = {
                "status": "ok",
                "unit": "pct",
                "aggregate": agg,
                "aggregate_kind": "max_across_active_joints",
                "per_joint": per_joint,
            }
    else:
        for m in ("settling_time", "overshoot"):
            if m in metrics_listed:
                result["metrics"][m] = {
                    "status": "not_applicable",
                    "reason": f"{m} only computed for scenario_type=='step' "
                    f"in v1 (got {stype!r})",
                }

    if "control_effort" in metrics_listed:
        tau_path = run_dir / "tau_d.csv"
        if tau_path.exists():
            tau = load_series(tau_path)
            agg, per_joint = control_effort_peak(tau)
            result["metrics"]["control_effort"] = {
                "status": "ok",
                "unit": "Nm",
                "aggregate": agg,
                "aggregate_kind": "peak_abs_across_joints",
                "per_joint": per_joint,
            }
        else:
            result["metrics"]["control_effort"] = {
                "status": "skipped",
                "reason": "tau_d.csv not present (controller does not publish tau_d)",
            }

    # Evaluate pass_criteria.
    for key, threshold in pass_criteria.items():
        metric_name = _JOINT_PASS_KEYS.get(key)
        if metric_name is None:
            result["pass_results"].append(
                {
                    "key": key,
                    "threshold": threshold,
                    "value": None,
                    "status": "skipped",
                    "reason": f"unknown pass_criteria key {key!r} for joint scenario",
                }
            )
            continue
        m = result["metrics"].get(metric_name)
        if m is None or m.get("status") in ("skipped", "not_applicable"):
            result["pass_results"].append(
                {
                    "key": key,
                    "threshold": threshold,
                    "value": None,
                    "status": "skipped",
                    "reason": (
                        m.get("reason")
                        if isinstance(m, dict)
                        else f"metric {metric_name!r} not computed"
                    ),
                }
            )
            continue
        value = m.get("aggregate")
        if value is None or not isinstance(value, (int, float)):
            result["pass_results"].append(
                {
                    "key": key,
                    "threshold": threshold,
                    "value": value,
                    "status": "skipped",
                    "reason": "no aggregate value to compare",
                }
            )
            continue
        ok = value <= float(threshold)
        result["pass_results"].append(
            {
                "key": key,
                "threshold": float(threshold),
                "value": float(value) if math.isfinite(value) else math.inf,
                "status": "pass" if ok else "fail",
            }
        )
        if not ok:
            result["overall_status"] = "fail"

    return result


def _scenario_target_space(run_dir: Path, scenario: dict) -> str:
    """Resolve target_space by re-reading the scenario file from disk."""
    doc = _load_scenario_doc(run_dir, scenario)
    return doc.get("target", {}).get("space", "joint")


def _load_scenario_doc(run_dir: Path, scenario: dict) -> dict:
    rel = scenario.get("path")
    if not rel:
        raise ValueError("manifest.scenario.path missing; can't resolve scenario")
    candidate = (REPO_ROOT / rel) if not Path(rel).is_absolute() else Path(rel)
    if not candidate.exists():
        candidate = run_dir / rel
    with _need(candidate).open("r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def _is_relative_to(child: Path, parent: Path) -> bool:
    try:
        child.relative_to(parent)
        return True
    except ValueError:
        return False


# ---------------------------------------------------------------------------
# Output writers
# ---------------------------------------------------------------------------


def write_metrics_yaml(path: Path, payload: dict) -> None:
    safe = _yaml_safe(payload)
    with path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(safe, f, sort_keys=False)


def _yaml_safe(obj):
    """Replace non-finite floats with ``.inf`` / ``.nan`` strings PyYAML can dump."""
    if isinstance(obj, dict):
        return {k: _yaml_safe(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return [_yaml_safe(v) for v in obj]
    if isinstance(obj, float):
        if math.isinf(obj):
            return ".inf" if obj > 0 else "-.inf"
        if math.isnan(obj):
            return ".nan"
    return obj


def write_metrics_csv(path: Path, payload: dict) -> None:
    """Flatten the metrics dict into a long-format CSV for easy diffing."""
    rows: list[list[str]] = []
    rows.append(
        [
            "metric",
            "joint",
            "value",
            "unit",
            "status",
            "reason",
        ]
    )
    for name, m in (payload.get("metrics") or {}).items():
        if not isinstance(m, dict):
            continue
        status = m.get("status", "")
        reason = m.get("reason", "")
        unit = m.get("unit", "")
        if status == "ok":
            agg = m.get("aggregate")
            rows.append(
                [
                    name,
                    "_aggregate_",
                    "" if agg is None else _fmt_float(agg),
                    unit,
                    status,
                    reason,
                ]
            )
            for j, v in (m.get("per_joint") or {}).items():
                rows.append([name, j, _fmt_float(v), unit, status, reason])
        else:
            rows.append([name, "_aggregate_", "", unit, status, reason])
    rows.append([])
    rows.append(["pass_key", "threshold", "value", "status", "reason", ""])
    for pr in payload.get("pass_results") or []:
        rows.append(
            [
                pr.get("key", ""),
                _fmt_float(pr.get("threshold")),
                _fmt_float(pr.get("value")),
                pr.get("status", ""),
                pr.get("reason", ""),
                "",
            ]
        )
    with path.open("w", encoding="utf-8", newline="") as f:
        w = csv.writer(f)
        for r in rows:
            w.writerow(r)


def _fmt_float(x) -> str:
    if x is None:
        return ""
    if isinstance(x, float):
        if math.isinf(x):
            return "inf" if x > 0 else "-inf"
        if math.isnan(x):
            return "nan"
        return f"{x:.9g}"
    return str(x)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Compute metrics for an evaluation run directory (M5)."
    )
    parser.add_argument(
        "--run-dir",
        required=True,
        type=Path,
        help="Path to a run directory containing manifest.yaml + CSVs.",
    )
    args = parser.parse_args(argv)

    run_dir = args.run_dir
    if not run_dir.is_dir():
        print(f"--run-dir: not a directory: {run_dir}", file=sys.stderr)
        return 2

    try:
        payload = compute_metrics_for_run(run_dir)
    except (FileNotFoundError, ValueError) as e:
        print(f"compute_metrics: {e}", file=sys.stderr)
        return 2

    write_metrics_yaml(run_dir / "metrics.yaml", payload)
    write_metrics_csv(run_dir / "metrics.csv", payload)

    overall = payload.get("overall_status", "pass")
    print(
        f"{run_dir}: overall={overall} "
        f"({sum(1 for p in payload['pass_results'] if p['status'] == 'pass')} pass, "
        f"{sum(1 for p in payload['pass_results'] if p['status'] == 'fail')} fail, "
        f"{sum(1 for p in payload['pass_results'] if p['status'] == 'skipped')} skipped)"
    )
    return 0 if overall == "pass" else 1


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
