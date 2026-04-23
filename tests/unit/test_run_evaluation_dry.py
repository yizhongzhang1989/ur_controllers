"""Unit tests for the M5 evaluation runner.

Pure-Python (no ROS). Covers:

* the reference-trajectory generators in ``evaluation/_reference.py``
  for every ``scenario_type``,
* controller registry + scenario compatibility logic in
  ``evaluation/run_evaluation.py``,
* the ``--dry-run`` end-to-end CLI path: every committed example scenario
  paired with a compatible controller produces a valid run dir
  (``manifest.yaml`` + ``target.csv`` of the expected shape) without
  contacting ROS.

The dry-run tests exist so that the runner's scenario parsing, controller
dispatch, manifest emission, and trajectory plumbing are all gated by the
test suite even though we can't afford to spin a sim+controller per
scenario in ``scripts/run_tests.sh``. The live launch path is exercised
manually and incrementally by future M5 work; the existing per-controller
integration tests
(``test_crisp_*.py`` / ``test_simple_jimp_regulation.py`` /
``test_cartesian_motion.py``) already keep the bring-up halves honest.
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
SCENARIO_DIR = EVAL_DIR / "scenarios"


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec and spec.loader
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


REF = _load("_eval_reference", EVAL_DIR / "_reference.py")
RUN = _load("_eval_run", EVAL_DIR / "run_evaluation.py")


# ---------------------------------------------------------------------------
# Reference-generator unit tests
# ---------------------------------------------------------------------------


def test_step_holds_then_jumps():
    q0 = [0.1, -1.5, 1.2, 0.0, 1.0, 0.0]
    amp = [0.0, 0.1, 0.0, 0.0, 0.0, 0.0]
    samples = REF.generate_step(q0, amp, step_time_s=1.0, duration_s=2.0, rate_hz=10.0)
    assert samples[0].t_s == pytest.approx(0.0)
    # Last sample lands at floor(2.0*10)+1 = 21 samples; t = 2.0
    assert len(samples) == 21
    assert samples[-1].t_s == pytest.approx(2.0)
    # Pre-step sample
    pre = next(s for s in samples if s.t_s < 1.0 - 1e-9)
    assert list(pre.position) == pytest.approx(q0)
    # First sample at or after the step
    post = next(s for s in samples if s.t_s >= 1.0)
    expected = [q0[i] + amp[i] for i in range(6)]
    assert list(post.position) == pytest.approx(expected)


def test_step_rejects_bad_step_time():
    q0 = [0.0] * 6
    amp = [0.0] * 6
    with pytest.raises(ValueError):
        REF.generate_step(q0, amp, step_time_s=2.0, duration_s=2.0)
    with pytest.raises(ValueError):
        REF.generate_step(q0, amp, step_time_s=-0.1, duration_s=2.0)


def test_sine_starts_at_initial_when_phase_zero():
    q0 = [0.0, -1.0, 0.5, 0.0, 1.0, 0.0]
    amp = [0.0, 0.05, 0.0, 0.0, 0.0, 0.0]
    samples = REF.generate_sine(
        q0,
        amp,
        frequency_hz=0.5,
        phase_rad=0.0,
        duration_s=2.0,
        rate_hz=100.0,
    )
    assert list(samples[0].position) == pytest.approx(q0)
    # Quarter period for f=0.5 Hz is 0.5 s -> peak amplitude.
    quarter = next(s for s in samples if s.t_s >= 0.5 - 1e-9)
    expected = [q0[i] + amp[i] * math.sin(2 * math.pi * 0.5 * quarter.t_s) for i in range(6)]
    assert list(quarter.position) == pytest.approx(expected, abs=1e-9)


def test_regulation_initial_is_constant():
    q0 = [0.1, -1.5, 1.2, 0.0, 1.0, 0.0]
    samples = REF.generate_regulation_joint(q0, hold="initial", duration_s=1.0, rate_hz=10.0)
    assert all(list(s.position) == pytest.approx(q0) for s in samples)


def test_regulation_explicit_hold_overrides_initial():
    q0 = [0.0] * 6
    target = [0.2, 0.0, 0.0, 0.0, 0.0, 0.0]
    samples = REF.generate_regulation_joint(q0, hold=target, duration_s=0.5, rate_hz=10.0)
    assert all(list(s.position) == pytest.approx(target) for s in samples)


def test_random_waypoints_deterministic_from_seed():
    q0 = [0.0] * 6
    lo = [-0.5] * 6
    hi = [0.5] * 6
    a = REF.generate_random_waypoints(
        q0,
        num_waypoints=3,
        dwell_s=1.0,
        seed=42,
        bounds_lower=lo,
        bounds_upper=hi,
        duration_s=3.0,
        rate_hz=10.0,
    )
    b = REF.generate_random_waypoints(
        q0,
        num_waypoints=3,
        dwell_s=1.0,
        seed=42,
        bounds_lower=lo,
        bounds_upper=hi,
        duration_s=3.0,
        rate_hz=10.0,
    )
    assert [s.position for s in a] == [s.position for s in b]
    # Different seed -> different first waypoint (overwhelmingly likely).
    c = REF.generate_random_waypoints(
        q0,
        num_waypoints=3,
        dwell_s=1.0,
        seed=43,
        bounds_lower=lo,
        bounds_upper=hi,
        duration_s=3.0,
        rate_hz=10.0,
    )
    assert a[0].position != c[0].position


def test_random_waypoints_segments_change_at_dwell_boundary():
    q0 = [0.0] * 6
    lo = [-0.5] * 6
    hi = [0.5] * 6
    samples = REF.generate_random_waypoints(
        q0,
        num_waypoints=2,
        dwell_s=1.0,
        seed=7,
        bounds_lower=lo,
        bounds_upper=hi,
        duration_s=2.0,
        rate_hz=10.0,
    )
    pre = next(s for s in samples if s.t_s < 1.0 - 1e-9)
    post = next(s for s in samples if s.t_s >= 1.0)
    # Different waypoints (overwhelmingly likely with random uniform).
    assert pre.position != post.position


def test_random_waypoints_rejects_bad_bounds():
    q0 = [0.0] * 6
    with pytest.raises(ValueError):
        REF.generate_random_waypoints(
            q0,
            num_waypoints=1,
            dwell_s=1.0,
            seed=0,
            bounds_lower=[1.0] * 6,
            bounds_upper=[1.0] * 6,
            duration_s=1.0,
            rate_hz=10.0,
        )


def test_dispatch_rejects_cartesian_target_for_step():
    scenario = {
        "scenario_type": "step",
        "duration_s": 2.0,
        "target": {"space": "cartesian"},
        "command": {},
    }
    with pytest.raises(ValueError):
        REF.generate_joint_reference(scenario, [0.0] * 6)


def test_dispatch_rejects_urdf_bounds():
    scenario = {
        "scenario_type": "random_waypoints",
        "duration_s": 2.0,
        "target": {"space": "joint"},
        "command": {
            "num_waypoints": 1,
            "dwell_s": 1.0,
            "seed": 0,
            "bounds": "urdf",
        },
    }
    with pytest.raises(ValueError):
        REF.generate_joint_reference(scenario, [0.0] * 6)


# ---------------------------------------------------------------------------
# Controller registry / compatibility
# ---------------------------------------------------------------------------


def test_known_controllers_includes_each_bringup():
    known = set(RUN.known_controllers())
    assert known == {
        "crisp_joint_impedance",
        "crisp_cartesian_impedance",
        "crisp_gravity_compensation",
        "simple_joint_impedance",
        "cartesian_motion",
    }


@pytest.mark.parametrize(
    "name",
    [
        "crisp_joint_impedance",
        "crisp_cartesian_impedance",
        "crisp_gravity_compensation",
        "simple_joint_impedance",
        "cartesian_motion",
    ],
)
def test_controller_config_files_exist(name):
    spec = RUN.get_controller(name)
    for robot in ("ur5e", "ur15"):
        cfg = REPO_ROOT / "bringup" / "config" / f"{spec.config_stem}.{robot}.yaml"
        assert cfg.is_file(), f"missing controller config: {cfg}"


@pytest.mark.parametrize(
    "name",
    [
        "crisp_joint_impedance",
        "crisp_cartesian_impedance",
        "crisp_gravity_compensation",
        "simple_joint_impedance",
        "cartesian_motion",
    ],
)
def test_controller_bringup_files_exist(name):
    spec = RUN.get_controller(name)
    assert (REPO_ROOT / spec.bringup_launch).is_file()


def test_compatibility_joint_scenario_against_cartesian_controller():
    scenario = yaml.safe_load((SCENARIO_DIR / "step.example.yaml").read_text(encoding="utf-8"))
    spec = RUN.get_controller("cartesian_motion")
    errs = RUN.check_compatibility(spec, scenario)
    assert errs and "target.space" in errs[0]


def test_compatibility_gravity_only_supports_regulation():
    scenario = yaml.safe_load((SCENARIO_DIR / "step.example.yaml").read_text(encoding="utf-8"))
    spec = RUN.get_controller("crisp_gravity_compensation")
    errs = RUN.check_compatibility(spec, scenario)
    assert any("regulation" in e for e in errs)


def test_compatibility_cartesian_step_rejected():
    scenario = {
        "scenario_type": "step",
        "target": {"space": "cartesian"},
    }
    spec = RUN.get_controller("cartesian_motion")
    errs = RUN.check_compatibility(spec, scenario)
    # At least the joint/cartesian space mismatch fires (and the cartesian
    # non-regulation check, depending on order).
    assert errs


def test_compatibility_joint_regulation_against_joint_controller():
    scenario = yaml.safe_load(
        (SCENARIO_DIR / "regulation.example.yaml").read_text(encoding="utf-8")
    )
    for name in (
        "crisp_joint_impedance",
        "simple_joint_impedance",
        "crisp_gravity_compensation",
    ):
        spec = RUN.get_controller(name)
        assert RUN.check_compatibility(spec, scenario) == []


# ---------------------------------------------------------------------------
# CLI dry-run end-to-end
# ---------------------------------------------------------------------------

# scenario file -> compatible controller (one per scenario for coverage).
DRY_RUN_PAIRS = [
    ("step.example.yaml", "simple_joint_impedance"),
    ("sine.example.yaml", "crisp_joint_impedance"),
    ("regulation.example.yaml", "simple_joint_impedance"),
    ("regulation.example.yaml", "crisp_gravity_compensation"),
]


@pytest.mark.parametrize("scenario_file,controller", DRY_RUN_PAIRS)
def test_dry_run_emits_target_and_manifest(tmp_path, scenario_file, controller):
    scenario_path = SCENARIO_DIR / scenario_file
    rc = RUN.main(
        [
            "--scenario",
            str(scenario_path),
            "--controller",
            controller,
            "--robot",
            "ur5e",
            "--out-dir",
            str(tmp_path),
            "--dry-run",
            "--initial-joints",
            "0,-1.57,1.57,0,1.57,0",
            "--rate-hz",
            "20",
        ]
    )
    assert rc == 0
    runs = list(tmp_path.iterdir())
    assert len(runs) == 1
    run_dir = runs[0]
    assert run_dir.is_dir()
    assert run_dir.name.startswith(scenario_path.stem.replace(".example", ""))

    target_csv = run_dir / "target.csv"
    manifest = run_dir / "manifest.yaml"
    assert target_csv.is_file()
    assert manifest.is_file()

    # joint_states.csv / tau_d.csv only exist for live runs.
    assert not (run_dir / "joint_states.csv").exists()
    assert not (run_dir / "tau_d.csv").exists()

    with target_csv.open() as f:
        rows = list(csv.reader(f))
    assert rows[0][0] == "t_s"
    # Joint scenarios -> 7 columns (t + 6 joints). Cartesian regulation
    # would produce 8, but this parametrisation only feeds joint scenarios.
    assert len(rows[0]) == 7
    assert len(rows) >= 2
    # All rows after the header should have the same column count.
    for r in rows[1:]:
        assert len(r) == len(rows[0])

    payload = yaml.safe_load(manifest.read_text(encoding="utf-8"))
    assert payload["schema_version"] == 1
    assert payload["kind"] == "evaluation_run"
    assert payload["controller"]["name"] == controller
    assert payload["robot"] == "ur5e"
    assert payload["runner"]["dry_run"] is True
    assert payload["scenario"]["name"]
    assert payload["initial_joint_positions"] == pytest.approx([0.0, -1.57, 1.57, 0.0, 1.57, 0.0])


def test_dry_run_cartesian_regulation_emits_8col_target(tmp_path):
    # Build a cartesian regulation scenario inline.
    scenario = {
        "schema_version": 1,
        "name": "cartesian-regulation-fixture",
        "description": "fixture",
        "scenario_type": "regulation",
        "duration_s": 2.0,
        "robots": ["ur5e", "ur15"],
        "target": {
            "space": "cartesian",
            "frame_id": "base",
            "end_effector": "tool0",
        },
        "command": {
            "hold": {
                "position_xyz_m": [0.4, 0.0, 0.3],
                "orientation_xyzw": [0.0, 1.0, 0.0, 0.0],
            }
        },
        "metrics": ["rmse"],
        "pass_criteria": {"max_rmse_m": 0.05, "max_rmse_rad": 0.1},
    }
    scenario_file = tmp_path / "scenario.yaml"
    scenario_file.write_text(yaml.safe_dump(scenario), encoding="utf-8")
    out = tmp_path / "out"
    rc = RUN.main(
        [
            "--scenario",
            str(scenario_file),
            "--controller",
            "cartesian_motion",
            "--robot",
            "ur5e",
            "--out-dir",
            str(out),
            "--dry-run",
            "--rate-hz",
            "10",
        ]
    )
    assert rc == 0
    run_dir = next(out.iterdir())
    rows = list(csv.reader((run_dir / "target.csv").open()))
    assert rows[0] == ["t_s", "px", "py", "pz", "qx", "qy", "qz", "qw"]
    assert len(rows) >= 2
    # Constant pose every row.
    pose = [float(x) for x in rows[1][1:]]
    assert pose == pytest.approx([0.4, 0.0, 0.3, 0.0, 1.0, 0.0, 0.0])


def test_cli_rejects_invalid_scenario(tmp_path):
    bad = tmp_path / "bad.yaml"
    bad.write_text("not_a_scenario: true\n", encoding="utf-8")
    rc = RUN.main(
        [
            "--scenario",
            str(bad),
            "--controller",
            "simple_joint_impedance",
            "--robot",
            "ur5e",
            "--out-dir",
            str(tmp_path / "out"),
            "--dry-run",
        ]
    )
    assert rc == 2


def test_cli_rejects_robot_not_in_scenario(tmp_path):
    # Scenario only lists ur5e; reject ur15.
    scenario = {
        "schema_version": 1,
        "name": "ur5e-only",
        "description": "fixture",
        "scenario_type": "regulation",
        "duration_s": 1.0,
        "robots": ["ur5e"],
        "target": {
            "space": "joint",
            "joints": list(REF.__dict__.get("CANONICAL_JOINTS", []))
            or [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ],
        },
        "command": {"hold": "initial"},
        "metrics": ["rmse"],
        "pass_criteria": {"max_rmse_rad": 0.1},
    }
    scenario_file = tmp_path / "scenario.yaml"
    scenario_file.write_text(yaml.safe_dump(scenario), encoding="utf-8")
    rc = RUN.main(
        [
            "--scenario",
            str(scenario_file),
            "--controller",
            "simple_joint_impedance",
            "--robot",
            "ur15",
            "--out-dir",
            str(tmp_path / "out"),
            "--dry-run",
        ]
    )
    assert rc == 2


def test_cli_rejects_incompatible_controller(tmp_path):
    scenario_path = SCENARIO_DIR / "step.example.yaml"
    rc = RUN.main(
        [
            "--scenario",
            str(scenario_path),
            "--controller",
            "cartesian_motion",
            "--robot",
            "ur5e",
            "--out-dir",
            str(tmp_path),
            "--dry-run",
        ]
    )
    assert rc == 2


def test_module_runs_as_script(tmp_path):
    """``python3 evaluation/run_evaluation.py --help`` does not crash."""
    cp = subprocess.run(
        [sys.executable, str(EVAL_DIR / "run_evaluation.py"), "--help"],
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert cp.returncode == 0
    assert "evaluation scenario" in cp.stdout.lower()
