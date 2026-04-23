"""Unit tests for the M5 comparison report driver (M5 bullet 4).

Pure-Python (no ROS). Seeds synthetic run directories in a tmp path and
exercises ``evaluation/compare.py``'s aggregate-only path end-to-end:

* combo enumeration + compatibility gating,
* latest-valid run-dir lookup,
* metrics computation via the real ``compute_metrics_for_run``,
* report CSV + Markdown emission,
* overall exit code.
"""

from __future__ import annotations

import csv
import importlib.util
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


CMP = _load("_eval_compare", EVAL_DIR / "compare.py")


CANONICAL_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


# ---------------------------------------------------------------------------
# Fixtures: build a little scenario set + a runs-root on disk.
# ---------------------------------------------------------------------------


def _regulation_scenario(name: str) -> dict:
    return {
        "schema_version": 1,
        "name": name,
        "description": "test regulation",
        "scenario_type": "regulation",
        "duration_s": 1.0,
        "robots": ["ur5e", "ur15"],
        "target": {"space": "joint", "joints": list(CANONICAL_JOINTS)},
        "command": {"hold": "initial"},
        "metrics": ["rmse", "control_effort"],
        "pass_criteria": {"max_rmse_rad": 0.05, "max_control_effort_nm": 5.0},
    }


def _cartesian_regulation_scenario(name: str) -> dict:
    return {
        "schema_version": 1,
        "name": name,
        "description": "test cartesian regulation",
        "scenario_type": "regulation",
        "duration_s": 1.0,
        "robots": ["ur5e", "ur15"],
        "target": {"space": "cartesian", "frame_id": "base_link", "end_effector": "tool0"},
        "command": {
            "hold": {
                "position_xyz_m": [0.3, 0.0, 0.4],
                "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
            }
        },
        "metrics": ["rmse"],
        "pass_criteria": {"max_rmse_m": 0.02},
    }


def _write_scenario(dir_: Path, doc: dict) -> Path:
    p = dir_ / f"{doc['name']}.yaml"
    with p.open("w", encoding="utf-8") as f:
        yaml.safe_dump(doc, f, sort_keys=False)
    return p


def _write_csv(path: Path, header: list[str], rows: list[list]) -> None:
    with path.open("w", encoding="utf-8", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        for r in rows:
            w.writerow([f"{x:.9g}" if isinstance(x, float) else x for x in r])


def _const_rows(times: list[float], q: list[float]) -> list[list[float]]:
    return [[t, *q] for t in times]


def _seed_run_dir(
    runs_root: Path,
    *,
    scenario_doc: dict,
    scenario_path: Path,
    controller: str,
    robot: str,
    timestamp: str,
    initial: list[float],
    target_rows: list[list[float]] | None,
    observed_rows: list[list[float]] | None,
    tau_rows: list[list[float]] | None = None,
    skip_manifest: bool = False,
) -> Path:
    run_dir = (
        runs_root
        / f"{scenario_doc['name']}__{controller}__{robot}__{timestamp}"
    )
    run_dir.mkdir(parents=True)
    target_space = scenario_doc.get("target", {}).get("space", "joint")
    manifest = {
        "schema_version": 1,
        "kind": "evaluation_run",
        "scenario": {
            "name": scenario_doc["name"],
            "type": scenario_doc["scenario_type"],
            "path": str(scenario_path),
            "duration_s": scenario_doc["duration_s"],
            "metrics": scenario_doc.get("metrics"),
            "pass_criteria": scenario_doc.get("pass_criteria"),
        },
        "controller": {
            "name": controller,
            "controller_node": f"{controller}_node",
            "sim_mode": "effort",
            "bringup_launch": "n/a",
            "bringup_args": [],
            "config": "n/a",
            "target_topic": "n/a",
            "displaced_controller": "n/a",
        },
        "robot": robot,
        "publish_rate_hz": 50.0,
        "initial_joint_positions": initial,
        "artefacts": {"target_csv": "target.csv", "joint_states_csv": "joint_states.csv"},
        "started_at_utc": "2026-01-01T00:00:00Z",
        "finished_at_utc": "2026-01-01T00:00:01Z",
        "runner": {"script": "test", "version": 1, "dry_run": True},
        "target_space": target_space,
    }
    if not skip_manifest:
        with (run_dir / "manifest.yaml").open("w", encoding="utf-8") as f:
            yaml.safe_dump(manifest, f, sort_keys=False)
    if target_space == "joint":
        if target_rows is not None:
            _write_csv(run_dir / "target.csv", ["t_s"] + CANONICAL_JOINTS, target_rows)
        if observed_rows is not None:
            _write_csv(
                run_dir / "joint_states.csv", ["t_s"] + CANONICAL_JOINTS, observed_rows
            )
    else:
        if target_rows is not None:
            _write_csv(
                run_dir / "target.csv",
                ["t_s", "px", "py", "pz", "qx", "qy", "qz", "qw"],
                target_rows,
            )
    if tau_rows is not None:
        _write_csv(run_dir / "tau_d.csv", ["t_s"] + CANONICAL_JOINTS, tau_rows)
    return run_dir


# ---------------------------------------------------------------------------
# enumerate_combos
# ---------------------------------------------------------------------------


def test_enumerate_combos_filters_by_scenario_robots_list(tmp_path):
    doc = _regulation_scenario("reg-ur5e-only")
    doc["robots"] = ["ur5e"]  # scenario only applies to ur5e
    sp = _write_scenario(tmp_path, doc)
    combos, inc = CMP.enumerate_combos(
        [sp], ["simple_joint_impedance"], ["ur5e", "ur15"]
    )
    assert [c.robot for c in combos] == ["ur5e"]
    assert len(inc) == 1
    assert inc[0].robot == "ur15"
    assert any("not listed" in r for r in inc[0].reasons)


def test_enumerate_combos_filters_incompatible_cartesian_vs_joint(tmp_path):
    doc = _cartesian_regulation_scenario("cart-reg")
    sp = _write_scenario(tmp_path, doc)
    combos, inc = CMP.enumerate_combos(
        [sp], ["simple_joint_impedance", "cartesian_motion"], ["ur5e"]
    )
    # simple_joint_impedance is joint-only -> incompatible with cartesian scenario.
    combo_names = {(c.controller, c.robot) for c in combos}
    assert ("cartesian_motion", "ur5e") in combo_names
    assert ("simple_joint_impedance", "ur5e") not in combo_names
    assert any(i.controller == "simple_joint_impedance" for i in inc)


def test_enumerate_combos_respects_scenario_on_valid_joint_case(tmp_path):
    doc = _regulation_scenario("reg-all")
    sp = _write_scenario(tmp_path, doc)
    combos, inc = CMP.enumerate_combos(
        [sp], ["simple_joint_impedance", "crisp_joint_impedance"], ["ur5e", "ur15"]
    )
    # 2 controllers × 2 robots = 4 compatible, none rejected.
    assert len(combos) == 4
    assert inc == []


# ---------------------------------------------------------------------------
# find_latest_run_dir
# ---------------------------------------------------------------------------


def test_find_latest_run_dir_picks_newest_valid(tmp_path):
    runs = tmp_path / "runs"
    runs.mkdir()
    scen = _regulation_scenario("reg-x")
    sp = _write_scenario(tmp_path, scen)
    q = [0.0] * 6
    times = [i * 0.1 for i in range(5)]
    older = _seed_run_dir(
        runs, scenario_doc=scen, scenario_path=sp,
        controller="simple_joint_impedance", robot="ur5e",
        timestamp="20260101T000000Z",
        initial=q, target_rows=_const_rows(times, q),
        observed_rows=_const_rows(times, q),
    )
    newer = _seed_run_dir(
        runs, scenario_doc=scen, scenario_path=sp,
        controller="simple_joint_impedance", robot="ur5e",
        timestamp="20260102T000000Z",
        initial=q, target_rows=_const_rows(times, q),
        observed_rows=_const_rows(times, q),
    )
    found = CMP.find_latest_run_dir(
        runs, "reg-x", "simple_joint_impedance", "ur5e"
    )
    assert found == newer
    assert found != older


def test_find_latest_run_dir_skips_invalid_newer_dir(tmp_path):
    """A newer run dir without manifest.yaml must not mask an older valid one."""
    runs = tmp_path / "runs"
    runs.mkdir()
    scen = _regulation_scenario("reg-y")
    sp = _write_scenario(tmp_path, scen)
    q = [0.0] * 6
    times = [i * 0.1 for i in range(5)]
    valid = _seed_run_dir(
        runs, scenario_doc=scen, scenario_path=sp,
        controller="simple_joint_impedance", robot="ur5e",
        timestamp="20260101T000000Z",
        initial=q, target_rows=_const_rows(times, q),
        observed_rows=_const_rows(times, q),
    )
    _seed_run_dir(
        runs, scenario_doc=scen, scenario_path=sp,
        controller="simple_joint_impedance", robot="ur5e",
        timestamp="20260102T000000Z",
        initial=q, target_rows=_const_rows(times, q),
        observed_rows=_const_rows(times, q),
        skip_manifest=True,  # newer but not valid
    )
    found = CMP.find_latest_run_dir(
        runs, "reg-y", "simple_joint_impedance", "ur5e"
    )
    assert found == valid


def test_find_latest_run_dir_returns_none_when_missing(tmp_path):
    runs = tmp_path / "runs"
    runs.mkdir()
    assert (
        CMP.find_latest_run_dir(runs, "nope", "simple_joint_impedance", "ur5e")
        is None
    )
    # Also OK when runs-root doesn't exist at all.
    assert (
        CMP.find_latest_run_dir(
            tmp_path / "absent", "nope", "simple_joint_impedance", "ur5e"
        )
        is None
    )


# ---------------------------------------------------------------------------
# End-to-end: run_report + main CLI
# ---------------------------------------------------------------------------


def _seed_pass_run(
    runs: Path, scen: dict, sp: Path, controller: str, robot: str
) -> Path:
    q = [0.0, -1.5, 1.2, 0.0, 1.0, 0.0]
    times = [i * 0.1 for i in range(11)]
    return _seed_run_dir(
        runs, scenario_doc=scen, scenario_path=sp,
        controller=controller, robot=robot,
        timestamp=f"20260101T00000{hash((controller, robot)) % 10}Z",
        initial=q,
        target_rows=_const_rows(times, q),
        observed_rows=_const_rows(times, q),
        tau_rows=_const_rows(times, [0.5] * 6),
    )


def _seed_fail_run(
    runs: Path, scen: dict, sp: Path, controller: str, robot: str
) -> Path:
    q = [0.0, -1.5, 1.2, 0.0, 1.0, 0.0]
    times = [i * 0.1 for i in range(11)]
    # Observed drifts 0.3 rad on shoulder_pan -> RMSE above max_rmse_rad=0.05.
    observed = [[t, q[0] + 0.3, *q[1:]] for t in times]
    return _seed_run_dir(
        runs, scenario_doc=scen, scenario_path=sp,
        controller=controller, robot=robot,
        timestamp=f"20260101T00000{hash((controller, robot, 'f')) % 10}Z",
        initial=q,
        target_rows=_const_rows(times, q),
        observed_rows=observed,
        tau_rows=_const_rows(times, [0.5] * 6),
    )


def test_run_report_pass_row_and_exit_zero(tmp_path):
    scen = _regulation_scenario("reg-pass")
    sp = _write_scenario(tmp_path, scen)
    runs = tmp_path / "runs"
    runs.mkdir()
    _seed_pass_run(runs, scen, sp, "simple_joint_impedance", "ur5e")

    report_dir = tmp_path / "rep"
    rows, inc = CMP.run_report(
        scenarios=[sp],
        controllers=["simple_joint_impedance"],
        robots=["ur5e"],
        runs_root=runs,
        report_dir=report_dir,
    )
    assert len(rows) == 1
    assert rows[0].status == "pass"
    assert rows[0].run_dir is not None
    assert (report_dir / "report.csv").exists()
    assert (report_dir / "report.md").exists()
    # The metrics.yaml cache is written next to the run.
    assert (rows[0].run_dir / "metrics.yaml").exists()
    assert (rows[0].run_dir / "metrics.csv").exists()
    assert CMP.overall_exit_code(rows) == 0


def test_run_report_fail_row_makes_exit_nonzero(tmp_path):
    scen = _regulation_scenario("reg-fail")
    sp = _write_scenario(tmp_path, scen)
    runs = tmp_path / "runs"
    runs.mkdir()
    _seed_fail_run(runs, scen, sp, "simple_joint_impedance", "ur5e")

    report_dir = tmp_path / "rep"
    rows, _inc = CMP.run_report(
        scenarios=[sp],
        controllers=["simple_joint_impedance"],
        robots=["ur5e"],
        runs_root=runs,
        report_dir=report_dir,
    )
    assert len(rows) == 1
    assert rows[0].status == "fail"
    assert CMP.overall_exit_code(rows) == 1


def test_run_report_missing_run_dir_is_no_run_and_nonzero(tmp_path):
    scen = _regulation_scenario("reg-missing")
    sp = _write_scenario(tmp_path, scen)
    runs = tmp_path / "runs"
    runs.mkdir()
    # No run seeded.
    rows, _inc = CMP.run_report(
        scenarios=[sp],
        controllers=["simple_joint_impedance"],
        robots=["ur5e"],
        runs_root=runs,
        report_dir=tmp_path / "rep",
    )
    assert len(rows) == 1
    assert rows[0].status == "no_run"
    assert CMP.overall_exit_code(rows) == 1


def test_run_report_cartesian_is_not_yet_evaluated_without_failure(tmp_path):
    scen = _cartesian_regulation_scenario("cart-reg")
    sp = _write_scenario(tmp_path, scen)
    runs = tmp_path / "runs"
    runs.mkdir()
    # Seed a run dir to confirm cartesian is deferred even when a run exists.
    times = [i * 0.1 for i in range(5)]
    target_rows = [[t, 0.3, 0.0, 0.4, 0.0, 0.0, 0.0, 1.0] for t in times]
    _seed_run_dir(
        runs, scenario_doc=scen, scenario_path=sp,
        controller="cartesian_motion", robot="ur5e",
        timestamp="20260101T000000Z",
        initial=[0.0] * 6,
        target_rows=target_rows,
        observed_rows=None,
    )
    rows, _inc = CMP.run_report(
        scenarios=[sp],
        controllers=["cartesian_motion"],
        robots=["ur5e"],
        runs_root=runs,
        report_dir=tmp_path / "rep",
    )
    assert len(rows) == 1
    assert rows[0].status == "not_yet_evaluated"
    assert CMP.overall_exit_code(rows) == 0


def test_run_report_incompatible_combos_listed_separately(tmp_path):
    jscen = _regulation_scenario("reg-joint")
    cscen = _cartesian_regulation_scenario("cart-reg")
    jsp = _write_scenario(tmp_path, jscen)
    csp = _write_scenario(tmp_path, cscen)
    runs = tmp_path / "runs"
    runs.mkdir()
    # Seed the joint combo so it produces a pass row.
    _seed_pass_run(runs, jscen, jsp, "simple_joint_impedance", "ur5e")

    report_dir = tmp_path / "rep"
    rows, inc = CMP.run_report(
        scenarios=[jsp, csp],
        controllers=["simple_joint_impedance"],
        robots=["ur5e"],
        runs_root=runs,
        report_dir=report_dir,
    )
    # joint combo is compatible (1 row), cartesian combo with a joint
    # controller is incompatible.
    assert len(rows) == 1
    assert any(i.scenario_name == "cart-reg" for i in inc)
    md = (report_dir / "report.md").read_text()
    assert "Incompatible combos" in md
    assert "cart-reg" in md


def test_main_cli_exit_code_and_artefacts(tmp_path, capsys):
    scen = _regulation_scenario("reg-cli")
    sp = _write_scenario(tmp_path, scen)
    runs = tmp_path / "runs"
    runs.mkdir()
    _seed_pass_run(runs, scen, sp, "simple_joint_impedance", "ur5e")
    report_dir = tmp_path / "rep"
    rc = CMP.main([
        "--scenarios", str(sp),
        "--controllers", "simple_joint_impedance",
        "--robots", "ur5e",
        "--runs-root", str(runs),
        "--report-dir", str(report_dir),
    ])
    assert rc == 0
    assert (report_dir / "report.csv").exists()
    assert (report_dir / "report.md").exists()
    out = capsys.readouterr().out
    assert "compatible combos" in out


def test_main_cli_rejects_unknown_controller(tmp_path, capsys):
    scen = _regulation_scenario("reg-cli")
    sp = _write_scenario(tmp_path, scen)
    runs = tmp_path / "runs"
    runs.mkdir()
    rc = CMP.main([
        "--scenarios", str(sp),
        "--controllers", "not_a_controller",
        "--robots", "ur5e",
        "--runs-root", str(runs),
        "--report-dir", str(tmp_path / "rep"),
    ])
    assert rc == 2


def test_main_cli_rejects_missing_scenario_file(tmp_path):
    rc = CMP.main([
        "--scenarios", str(tmp_path / "does_not_exist.yaml"),
        "--controllers", "simple_joint_impedance",
        "--robots", "ur5e",
        "--runs-root", str(tmp_path / "runs"),
        "--report-dir", str(tmp_path / "rep"),
    ])
    assert rc == 2


def test_report_csv_contents_match_rows(tmp_path):
    scen = _regulation_scenario("reg-csv")
    sp = _write_scenario(tmp_path, scen)
    runs = tmp_path / "runs"
    runs.mkdir()
    _seed_pass_run(runs, scen, sp, "simple_joint_impedance", "ur5e")
    report_dir = tmp_path / "rep"
    CMP.run_report(
        scenarios=[sp],
        controllers=["simple_joint_impedance"],
        robots=["ur5e"],
        runs_root=runs,
        report_dir=report_dir,
    )
    with (report_dir / "report.csv").open() as f:
        reader = list(csv.reader(f))
    assert reader[0] == CMP._REPORT_CSV_HEADER
    data_row = reader[1]
    hdr = {name: i for i, name in enumerate(reader[0])}
    assert data_row[hdr["scenario"]] == "reg-csv"
    assert data_row[hdr["controller"]] == "simple_joint_impedance"
    assert data_row[hdr["robot"]] == "ur5e"
    assert data_row[hdr["overall_status"]] == "pass"
    # Numeric columns populated for the passing joint combo.
    assert data_row[hdr["rmse_rad"]] != ""
    assert data_row[hdr["control_effort_nm"]] != ""
    # settling_time and overshoot are not-applicable for regulation.
    assert data_row[hdr["settling_time_s"]] in ("not_applicable", "")
    assert data_row[hdr["overshoot_pct"]] in ("not_applicable", "")
