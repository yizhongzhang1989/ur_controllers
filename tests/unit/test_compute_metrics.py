"""Unit tests for the M5 metrics computation (M5 bullet 3).

Pure-Python (no ROS). Covers:

* the low-level helpers in ``evaluation/compute_metrics.py``
  (``interp_at``, ``load_series``, ``rmse_per_joint``,
  ``settling_time_per_joint``, ``overshoot_per_joint``,
  ``control_effort_peak``),
* the end-to-end pipeline + CLI: synthesise CSVs + a manifest in a tmp
  run dir, invoke ``compute_metrics_for_run`` / ``main``, assert the
  emitted ``metrics.yaml`` / ``metrics.csv`` and the exit code.

The synthesised inputs let us cover step / regulation / sine scenarios
and joint vs cartesian targets without spinning a sim per scenario,
matching the same gating rationale as ``test_run_evaluation_dry.py``.
"""

from __future__ import annotations

import csv
import importlib.util
import math
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
EVAL_DIR = REPO_ROOT / "evaluation"


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec and spec.loader
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


CM = _load("_eval_compute_metrics", EVAL_DIR / "compute_metrics.py")


CANONICAL_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


# ---------------------------------------------------------------------------
# Low-level helpers
# ---------------------------------------------------------------------------


def test_interp_at_inside_range():
    times = [0.0, 1.0, 2.0]
    vals = [0.0, 10.0, 20.0]
    assert CM.interp_at(times, vals, 0.5) == pytest.approx(5.0)
    assert CM.interp_at(times, vals, 1.5) == pytest.approx(15.0)


def test_interp_at_clamps_outside_range():
    times = [1.0, 2.0]
    vals = [3.0, 5.0]
    # Below first sample -> first value.
    assert CM.interp_at(times, vals, 0.0) == 3.0
    # Above last sample -> last value.
    assert CM.interp_at(times, vals, 99.0) == 5.0


def test_interp_at_handles_duplicate_timestamps():
    # Duplicate timestamps must not cause a div-by-zero. The function
    # returns one of the values at the duplicate (implementation-defined
    # which one); the contract is just that it doesn't crash.
    times = [0.0, 1.0, 1.0, 2.0]
    vals = [0.0, 10.0, 12.0, 20.0]
    out = CM.interp_at(times, vals, 1.0)
    assert out in (10.0, 12.0)


def test_interp_at_empty_raises():
    with pytest.raises(ValueError):
        CM.interp_at([], [], 0.0)


def test_load_series_round_trip(tmp_path):
    p = tmp_path / "x.csv"
    p.write_text("t_s,a,b\n0.0,1,2\n0.1,3,4\n")
    s = CM.load_series(p)
    assert s.times == [0.0, 0.1]
    assert s.columns["a"] == [1.0, 3.0]
    assert s.columns["b"] == [2.0, 4.0]
    assert s.column_names == ["a", "b"]


def test_load_series_rejects_missing_t_column(tmp_path):
    p = tmp_path / "x.csv"
    p.write_text("nope,a\n0,1\n")
    with pytest.raises(ValueError, match="t_s"):
        CM.load_series(p)


def test_load_series_rejects_ragged_rows(tmp_path):
    p = tmp_path / "x.csv"
    p.write_text("t_s,a,b\n0,1\n")  # missing one cell
    with pytest.raises(ValueError, match="cells"):
        CM.load_series(p)


def test_resample_series_picks_target_grid(tmp_path):
    p = tmp_path / "obs.csv"
    p.write_text("t_s,a\n0.0,0\n1.0,10\n2.0,20\n")
    s = CM.load_series(p)
    out = CM.resample_series(s, [0.0, 0.5, 1.5, 2.0])
    assert out["a"] == pytest.approx([0.0, 5.0, 15.0, 20.0])


# ---------------------------------------------------------------------------
# Metric primitives
# ---------------------------------------------------------------------------


def test_rmse_zero_for_matching_curves():
    target = {"j": [0.0, 0.1, 0.2]}
    observed = {"j": [0.0, 0.1, 0.2]}
    assert CM.rmse_per_joint(target, observed) == {"j": 0.0}


def test_rmse_constant_offset_is_offset():
    target = {"j": [0.0] * 5}
    observed = {"j": [0.05] * 5}
    out = CM.rmse_per_joint(target, observed)
    assert out["j"] == pytest.approx(0.05)


def test_rmse_rejects_length_mismatch():
    with pytest.raises(ValueError):
        CM.rmse_per_joint({"j": [0.0, 0.1]}, {"j": [0.0]})


def test_settling_time_immediate_when_already_inside_band():
    # Trajectory hits the post-step target on the first post-step sample
    # and stays there.
    times = [0.0, 0.1, 0.2, 0.3]
    target_post = {"j": 1.0}
    observed = {"j": [0.0, 1.0, 1.0, 1.0]}
    amp = {"j": 1.0}
    out = CM.settling_time_per_joint(times, target_post, observed, amp, step_time_s=0.1)
    assert out["j"] == pytest.approx(0.0)


def test_settling_time_inf_when_never_settles():
    times = [0.0, 0.5, 1.0, 1.5]
    target_post = {"j": 1.0}
    # Observed is always 0.4 off (well above the 5% * 1.0 = 0.05 tolerance).
    observed = {"j": [0.0, 0.6, 0.6, 0.6]}
    amp = {"j": 1.0}
    out = CM.settling_time_per_joint(times, target_post, observed, amp, step_time_s=0.0)
    assert math.isinf(out["j"])


def test_settling_time_zero_amplitude_short_circuits():
    times = [0.0, 0.1, 0.2]
    out = CM.settling_time_per_joint(
        times, {"j": 0.0}, {"j": [0.5, 0.5, 0.5]}, {"j": 0.0}, step_time_s=0.0
    )
    assert out["j"] == 0.0


def test_settling_time_reports_recovery_time():
    # Excursion at 0.4 s, then settled by 0.5 s. step_time_s = 0.0.
    times = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    target_post = {"j": 1.0}
    observed = {"j": [0.0, 1.0, 1.0, 1.0, 0.7, 1.0, 1.0]}  # excursion at 0.4
    amp = {"j": 1.0}
    out = CM.settling_time_per_joint(times, target_post, observed, amp, step_time_s=0.0)
    # Last excursion is at index 4 (t=0.4). Settled at index 5 (t=0.5).
    assert out["j"] == pytest.approx(0.5)


def test_overshoot_positive_amplitude():
    times = [0.0, 0.1, 0.2, 0.3]
    pre = {"j": 0.0}
    post = {"j": 0.1}
    # Observed reaches 0.12 -> 20% overshoot relative to amplitude 0.1.
    observed = {"j": [0.0, 0.05, 0.12, 0.10]}
    out = CM.overshoot_per_joint(times, pre, post, observed, step_time_s=0.0)
    assert out["j"] == pytest.approx(20.0)


def test_overshoot_negative_amplitude_uses_sign():
    times = [0.0, 0.1, 0.2]
    pre = {"j": 0.0}
    post = {"j": -0.1}
    observed = {"j": [0.0, -0.13, -0.10]}  # -0.03 past target on the negative side.
    out = CM.overshoot_per_joint(times, pre, post, observed, step_time_s=0.0)
    assert out["j"] == pytest.approx(30.0)


def test_overshoot_no_overshoot_returns_zero():
    times = [0.0, 0.1, 0.2]
    pre = {"j": 0.0}
    post = {"j": 0.1}
    observed = {"j": [0.0, 0.05, 0.09]}  # never reaches target.
    out = CM.overshoot_per_joint(times, pre, post, observed, step_time_s=0.0)
    assert out["j"] == 0.0


def test_overshoot_zero_amplitude_returns_zero():
    times = [0.0, 0.1]
    out = CM.overshoot_per_joint(times, {"j": 1.0}, {"j": 1.0}, {"j": [1.0, 1.5]}, step_time_s=0.0)
    assert out["j"] == 0.0


def test_control_effort_peak_is_abs_max():
    s = CM.Series(
        times=[0.0, 0.1, 0.2],
        columns={"a": [1.0, -3.0, 2.0], "b": [0.0, 0.5, -1.0]},
    )
    agg, per_joint = CM.control_effort_peak(s)
    assert per_joint == {"a": 3.0, "b": 1.0}
    assert agg == 3.0


# ---------------------------------------------------------------------------
# End-to-end pipeline
# ---------------------------------------------------------------------------


def _write_csv(path: Path, header: list[str], rows: list[list[float]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        for r in rows:
            w.writerow([f"{x:.9g}" if isinstance(x, float) else x for x in r])


def _seed_run_dir(
    tmp_path: Path,
    *,
    scenario_doc: dict,
    initial: list[float],
    target_rows: list[list[float]] | None,
    observed_rows: list[list[float]] | None,
    tau_rows: list[list[float]] | None,
    target_space: str = "joint",
    cartesian_target: bool = False,
) -> Path:
    """Create a self-contained run directory the metrics script can consume."""
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    scenario_path = run_dir / "scenario.yaml"
    with scenario_path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(scenario_doc, f, sort_keys=False)
    manifest = {
        "schema_version": 1,
        "kind": "evaluation_run",
        "scenario": {
            "name": scenario_doc["name"],
            "type": scenario_doc["scenario_type"],
            # Path is resolved relative to REPO_ROOT first, then run_dir.
            "path": "scenario.yaml",
            "duration_s": scenario_doc["duration_s"],
            "metrics": scenario_doc.get("metrics"),
            "pass_criteria": scenario_doc.get("pass_criteria"),
        },
        "controller": {
            "name": "synthetic",
            "controller_node": "synthetic_controller",
            "sim_mode": "effort",
            "bringup_launch": "n/a",
            "bringup_args": [],
            "config": "n/a",
            "target_topic": "n/a",
            "displaced_controller": "n/a",
        },
        "robot": "ur5e",
        "publish_rate_hz": 50.0,
        "initial_joint_positions": initial,
        "artefacts": {
            "target_csv": "target.csv",
            "joint_states_csv": "joint_states.csv",
        },
        "started_at_utc": "2026-01-01T00:00:00Z",
        "finished_at_utc": "2026-01-01T00:00:10Z",
        "runner": {"script": "test", "version": 1, "dry_run": True},
        "target_space": target_space,
    }
    with (run_dir / "manifest.yaml").open("w", encoding="utf-8") as f:
        yaml.safe_dump(manifest, f, sort_keys=False)
    if cartesian_target:
        if target_rows is not None:
            _write_csv(
                run_dir / "target.csv",
                ["t_s", "px", "py", "pz", "qx", "qy", "qz", "qw"],
                target_rows,
            )
    else:
        if target_rows is not None:
            _write_csv(
                run_dir / "target.csv",
                ["t_s"] + CANONICAL_JOINTS,
                target_rows,
            )
        if observed_rows is not None:
            _write_csv(
                run_dir / "joint_states.csv",
                ["t_s"] + CANONICAL_JOINTS,
                observed_rows,
            )
    if tau_rows is not None:
        _write_csv(
            run_dir / "tau_d.csv",
            ["t_s"] + CANONICAL_JOINTS,
            tau_rows,
        )
    return run_dir


def _const_rows(times: list[float], q: list[float]) -> list[list[float]]:
    return [[t, *q] for t in times]


def test_regulation_pass_emits_metrics_files_and_exits_zero(tmp_path):
    q = [0.0, -1.5, 1.2, 0.0, 1.0, 0.0]
    times = [i * 0.1 for i in range(11)]  # 0..1.0 s
    scenario = {
        "schema_version": 1,
        "name": "reg-pass",
        "description": "test",
        "scenario_type": "regulation",
        "duration_s": 1.0,
        "robots": ["ur5e"],
        "target": {"space": "joint", "joints": CANONICAL_JOINTS},
        "command": {"hold": "initial"},
        "metrics": ["rmse", "control_effort"],
        "pass_criteria": {"max_rmse_rad": 0.05, "max_control_effort_nm": 5.0},
    }
    run_dir = _seed_run_dir(
        tmp_path,
        scenario_doc=scenario,
        initial=q,
        target_rows=_const_rows(times, q),
        observed_rows=_const_rows(times, q),
        tau_rows=_const_rows(times, [0.5] * 6),
    )
    rc = CM.main(["--run-dir", str(run_dir)])
    assert rc == 0
    assert (run_dir / "metrics.yaml").exists()
    assert (run_dir / "metrics.csv").exists()
    payload = yaml.safe_load((run_dir / "metrics.yaml").read_text())
    assert payload["overall_status"] == "pass"
    assert payload["metrics"]["rmse"]["aggregate"] == pytest.approx(0.0)
    assert payload["metrics"]["control_effort"]["aggregate"] == pytest.approx(0.5)
    statuses = {p["key"]: p["status"] for p in payload["pass_results"]}
    assert statuses == {"max_rmse_rad": "pass", "max_control_effort_nm": "pass"}


def test_regulation_fail_returns_exit_one(tmp_path):
    q = [0.0] * 6
    times = [i * 0.1 for i in range(11)]
    scenario = {
        "schema_version": 1,
        "name": "reg-fail",
        "description": "test",
        "scenario_type": "regulation",
        "duration_s": 1.0,
        "robots": ["ur5e"],
        "target": {"space": "joint", "joints": CANONICAL_JOINTS},
        "command": {"hold": "initial"},
        "metrics": ["rmse"],
        "pass_criteria": {"max_rmse_rad": 0.05},
    }
    # Observed has a 0.2 rad offset on shoulder_pan -> RMSE = 0.2, exceeds 0.05.
    observed = [[t, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0] for t in times]
    run_dir = _seed_run_dir(
        tmp_path,
        scenario_doc=scenario,
        initial=q,
        target_rows=_const_rows(times, q),
        observed_rows=observed,
        tau_rows=None,
    )
    rc = CM.main(["--run-dir", str(run_dir)])
    assert rc == 1
    payload = yaml.safe_load((run_dir / "metrics.yaml").read_text())
    assert payload["overall_status"] == "fail"
    pr = next(p for p in payload["pass_results"] if p["key"] == "max_rmse_rad")
    assert pr["status"] == "fail"
    assert pr["value"] == pytest.approx(0.2)


def test_step_scenario_computes_settling_and_overshoot(tmp_path):
    q0 = [0.0] * 6
    times = [i * 0.05 for i in range(21)]  # 0..1.0 s
    step_time_s = 0.2
    amp = [0.0, 0.1, 0.0, 0.0, 0.0, 0.0]
    target_rows = []
    for t in times:
        if t < step_time_s:
            target_rows.append([t, *q0])
        else:
            target_rows.append([t, *(q0[i] + amp[i] for i in range(6))])
    # Observed: pre-step zeros, then 0.12 (20% overshoot) for one sample,
    # then settled at 0.10. shoulder_lift only.
    observed_rows = []
    post_target = q0[1] + amp[1]
    for i, t in enumerate(times):
        row = [t] + list(q0)
        if t >= step_time_s:
            if i == int(step_time_s / 0.05):
                row[2] = post_target + 0.02  # 20% overshoot
            else:
                row[2] = post_target
        observed_rows.append(row)
    scenario = {
        "schema_version": 1,
        "name": "step-test",
        "description": "test",
        "scenario_type": "step",
        "duration_s": 1.0,
        "robots": ["ur5e"],
        "target": {"space": "joint", "joints": CANONICAL_JOINTS},
        "command": {"per_joint_amplitude_rad": amp, "step_time_s": step_time_s},
        "metrics": ["rmse", "settling_time", "overshoot", "control_effort"],
        "pass_criteria": {
            "max_rmse_rad": 0.05,
            "max_settling_time_s": 0.5,
            "max_overshoot_pct": 25.0,
            "max_control_effort_nm": 10.0,
        },
    }
    run_dir = _seed_run_dir(
        tmp_path,
        scenario_doc=scenario,
        initial=q0,
        target_rows=target_rows,
        observed_rows=observed_rows,
        tau_rows=_const_rows(times, [1.0] * 6),
    )
    rc = CM.main(["--run-dir", str(run_dir)])
    assert rc == 0
    payload = yaml.safe_load((run_dir / "metrics.yaml").read_text())
    assert payload["overall_status"] == "pass"
    # Overshoot should be 20% on shoulder_lift, 0 elsewhere; aggregate 20.
    assert payload["metrics"]["overshoot"]["aggregate"] == pytest.approx(20.0)
    # Settling: a single excursion at the step sample, settled by next sample.
    assert payload["metrics"]["settling_time"]["aggregate"] == pytest.approx(0.05)
    # Control effort = 1.0.
    assert payload["metrics"]["control_effort"]["aggregate"] == pytest.approx(1.0)


def test_step_overshoot_threshold_can_fail(tmp_path):
    q0 = [0.0] * 6
    times = [i * 0.05 for i in range(11)]  # 0..0.5
    step_time_s = 0.1
    amp = [0.0, 0.1, 0.0, 0.0, 0.0, 0.0]
    target_rows = []
    for t in times:
        target_rows.append([t, *(q0[i] + (amp[i] if t >= step_time_s else 0.0) for i in range(6))])
    # 50% overshoot on shoulder_lift -> exceeds 25% threshold.
    observed_rows = []
    for t in times:
        row = [t] + list(q0)
        if t >= step_time_s:
            row[2] = q0[1] + amp[1] + 0.05  # 50% overshoot
        observed_rows.append(row)
    scenario = {
        "schema_version": 1,
        "name": "step-overshoot-fail",
        "description": "test",
        "scenario_type": "step",
        "duration_s": 0.5,
        "robots": ["ur5e"],
        "target": {"space": "joint", "joints": CANONICAL_JOINTS},
        "command": {"per_joint_amplitude_rad": amp, "step_time_s": step_time_s},
        "metrics": ["overshoot"],
        "pass_criteria": {"max_overshoot_pct": 25.0},
    }
    run_dir = _seed_run_dir(
        tmp_path,
        scenario_doc=scenario,
        initial=q0,
        target_rows=target_rows,
        observed_rows=observed_rows,
        tau_rows=None,
    )
    rc = CM.main(["--run-dir", str(run_dir)])
    assert rc == 1
    payload = yaml.safe_load((run_dir / "metrics.yaml").read_text())
    assert payload["overall_status"] == "fail"
    assert payload["metrics"]["overshoot"]["aggregate"] == pytest.approx(50.0)


def test_settling_overshoot_not_applicable_for_non_step(tmp_path):
    q = [0.0] * 6
    times = [i * 0.1 for i in range(6)]
    scenario = {
        "schema_version": 1,
        "name": "sine-test",
        "description": "test",
        "scenario_type": "sine",
        "duration_s": 0.5,
        "robots": ["ur5e"],
        "target": {"space": "joint", "joints": CANONICAL_JOINTS},
        "command": {
            "per_joint_amplitude_rad": [0.0, 0.05, 0.0, 0.0, 0.0, 0.0],
            "frequency_hz": 0.5,
            "phase_rad": 0.0,
        },
        "metrics": ["rmse", "settling_time", "overshoot", "control_effort"],
        "pass_criteria": {
            "max_rmse_rad": 1.0,
            "max_settling_time_s": 1.0,
            "max_overshoot_pct": 100.0,
            "max_control_effort_nm": 10.0,
        },
    }
    run_dir = _seed_run_dir(
        tmp_path,
        scenario_doc=scenario,
        initial=q,
        target_rows=_const_rows(times, q),
        observed_rows=_const_rows(times, q),
        tau_rows=_const_rows(times, [0.1] * 6),
    )
    rc = CM.main(["--run-dir", str(run_dir)])
    assert rc == 0
    payload = yaml.safe_load((run_dir / "metrics.yaml").read_text())
    for m in ("settling_time", "overshoot"):
        assert payload["metrics"][m]["status"] == "not_applicable"
    skipped = {p["key"] for p in payload["pass_results"] if p["status"] == "skipped"}
    assert {"max_settling_time_s", "max_overshoot_pct"} <= skipped


def test_control_effort_skipped_when_no_tau_d_csv(tmp_path):
    q = [0.0] * 6
    times = [0.0, 0.1, 0.2]
    scenario = {
        "schema_version": 1,
        "name": "no-tau",
        "description": "test",
        "scenario_type": "regulation",
        "duration_s": 0.2,
        "robots": ["ur5e"],
        "target": {"space": "joint", "joints": CANONICAL_JOINTS},
        "command": {"hold": "initial"},
        "metrics": ["rmse", "control_effort"],
        "pass_criteria": {"max_rmse_rad": 0.1, "max_control_effort_nm": 1.0},
    }
    run_dir = _seed_run_dir(
        tmp_path,
        scenario_doc=scenario,
        initial=q,
        target_rows=_const_rows(times, q),
        observed_rows=_const_rows(times, q),
        tau_rows=None,
    )
    rc = CM.main(["--run-dir", str(run_dir)])
    assert rc == 0
    payload = yaml.safe_load((run_dir / "metrics.yaml").read_text())
    assert payload["metrics"]["control_effort"]["status"] == "skipped"
    pr = next(p for p in payload["pass_results"] if p["key"] == "max_control_effort_nm")
    assert pr["status"] == "skipped"


def test_cartesian_scenario_skips_all_metrics_in_v1(tmp_path):
    times = [0.0, 0.1]
    target_rows = [[t, 0.4, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0] for t in times]
    scenario = {
        "schema_version": 1,
        "name": "cart-test",
        "description": "test",
        "scenario_type": "regulation",
        "duration_s": 0.1,
        "robots": ["ur5e"],
        "target": {"space": "cartesian", "frame_id": "base", "end_effector": "tool0"},
        "command": {
            "hold": {
                "position_xyz_m": [0.4, 0.0, 0.3],
                "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
            }
        },
        "metrics": ["rmse"],
        "pass_criteria": {"max_rmse_m": 0.01},
    }
    run_dir = _seed_run_dir(
        tmp_path,
        scenario_doc=scenario,
        initial=[0.0] * 6,
        target_rows=target_rows,
        observed_rows=None,
        tau_rows=None,
        target_space="cartesian",
        cartesian_target=True,
    )
    rc = CM.main(["--run-dir", str(run_dir)])
    assert rc == 0
    payload = yaml.safe_load((run_dir / "metrics.yaml").read_text())
    assert payload["target_space"] == "cartesian"
    assert payload["metrics"]["rmse"]["status"] == "skipped"
    assert payload["pass_results"][0]["status"] == "skipped"


def test_metrics_csv_has_aggregate_and_per_joint_rows(tmp_path):
    q = [0.0] * 6
    times = [0.0, 0.1, 0.2]
    scenario = {
        "schema_version": 1,
        "name": "csv-shape",
        "description": "test",
        "scenario_type": "regulation",
        "duration_s": 0.2,
        "robots": ["ur5e"],
        "target": {"space": "joint", "joints": CANONICAL_JOINTS},
        "command": {"hold": "initial"},
        "metrics": ["rmse"],
        "pass_criteria": {"max_rmse_rad": 0.1},
    }
    run_dir = _seed_run_dir(
        tmp_path,
        scenario_doc=scenario,
        initial=q,
        target_rows=_const_rows(times, q),
        observed_rows=_const_rows(times, q),
        tau_rows=None,
    )
    assert CM.main(["--run-dir", str(run_dir)]) == 0
    rows = list(csv.reader((run_dir / "metrics.csv").open()))
    header = rows[0]
    assert header[:5] == ["metric", "joint", "value", "unit", "status"]
    rmse_rows = [r for r in rows if r and r[0] == "rmse"]
    # 1 aggregate + 6 per-joint rows.
    assert len(rmse_rows) == 7
    assert any(r[1] == "_aggregate_" for r in rmse_rows)
    assert {r[1] for r in rmse_rows if r[1] != "_aggregate_"} == set(CANONICAL_JOINTS)


def test_main_returns_two_when_run_dir_missing(tmp_path):
    rc = CM.main(["--run-dir", str(tmp_path / "does-not-exist")])
    assert rc == 2


def test_main_returns_two_when_manifest_missing(tmp_path):
    rc = CM.main(["--run-dir", str(tmp_path)])
    assert rc == 2


def test_compute_metrics_for_run_inf_settling_propagates(tmp_path):
    q0 = [0.0] * 6
    times = [i * 0.1 for i in range(6)]  # 0..0.5
    amp = [0.0, 0.1, 0.0, 0.0, 0.0, 0.0]
    step_time_s = 0.1
    target_rows = []
    for t in times:
        target_rows.append([t, *(q0[i] + (amp[i] if t >= step_time_s else 0.0) for i in range(6))])
    # Joint never settles within the 5%-of-amplitude band.
    observed_rows = []
    for t in times:
        row = [t] + list(q0)
        if t >= step_time_s:
            row[2] = q0[1] + amp[1] + 0.05  # always 50% off
        observed_rows.append(row)
    scenario = {
        "schema_version": 1,
        "name": "step-no-settle",
        "description": "test",
        "scenario_type": "step",
        "duration_s": 0.5,
        "robots": ["ur5e"],
        "target": {"space": "joint", "joints": CANONICAL_JOINTS},
        "command": {"per_joint_amplitude_rad": amp, "step_time_s": step_time_s},
        "metrics": ["settling_time"],
        "pass_criteria": {"max_settling_time_s": 0.5},
    }
    run_dir = _seed_run_dir(
        tmp_path,
        scenario_doc=scenario,
        initial=q0,
        target_rows=target_rows,
        observed_rows=observed_rows,
        tau_rows=None,
    )
    rc = CM.main(["--run-dir", str(run_dir)])
    assert rc == 1
    payload = yaml.safe_load((run_dir / "metrics.yaml").read_text())
    # The dumped scalar is the YAML token '.inf' — accept either form.
    agg = payload["metrics"]["settling_time"]["aggregate"]
    assert agg == ".inf" or (isinstance(agg, float) and math.isinf(agg))
    pr = next(p for p in payload["pass_results"] if p["key"] == "max_settling_time_s")
    assert pr["status"] == "fail"


def test_cli_help_does_not_crash():
    cp = subprocess.run(
        [sys.executable, str(EVAL_DIR / "compute_metrics.py"), "--help"],
        capture_output=True,
        text=True,
    )
    assert cp.returncode == 0
    assert "Compute metrics" in cp.stdout
