"""Structural tests for `.github/workflows/ci.yml`.

These tests do not invoke `act` or any GitHub-hosted runner; they pin the
structure of the workflow so a future edit cannot silently drop the
pre-commit gate, the colcon build step, or the unit-test step that
ROADMAP M0 requires.
"""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
WORKFLOW = REPO_ROOT / ".github" / "workflows" / "ci.yml"


@pytest.fixture(scope="module")
def workflow() -> dict:
    assert WORKFLOW.is_file(), f"missing {WORKFLOW}"
    with WORKFLOW.open() as f:
        data = yaml.safe_load(f)
    assert isinstance(data, dict), "workflow must be a mapping"
    return data


def test_workflow_has_name(workflow: dict) -> None:
    assert workflow.get("name"), "workflow must declare a name"


def test_workflow_triggers_push_and_pr(workflow: dict) -> None:
    # PyYAML parses the bare key `on:` to the Python boolean True. Accept either.
    triggers = workflow.get("on") if "on" in workflow else workflow.get(True)
    assert isinstance(triggers, dict), "workflow `on` block must be a mapping"
    assert "push" in triggers, "workflow must trigger on push"
    assert "pull_request" in triggers, "workflow must trigger on pull_request"


def test_workflow_jobs_present(workflow: dict) -> None:
    jobs = workflow.get("jobs")
    assert isinstance(jobs, dict)
    assert "lint" in jobs, "workflow must declare a `lint` job"
    assert "build" in jobs, "workflow must declare a `build` job"


def test_lint_job_runs_pre_commit(workflow: dict) -> None:
    steps = workflow["jobs"]["lint"]["steps"]
    runs = " \n ".join(s.get("run", "") for s in steps if isinstance(s, dict))
    assert "pre-commit run" in runs, "lint job must invoke `pre-commit run`"
    assert "--all-files" in runs, "lint job must run pre-commit against all files"


def test_lint_job_installs_pre_commit_and_ruff(workflow: dict) -> None:
    steps = workflow["jobs"]["lint"]["steps"]
    runs = " \n ".join(s.get("run", "") for s in steps if isinstance(s, dict))
    assert "pre-commit" in runs and "ruff" in runs, "lint job must pip-install pre-commit and ruff"


def test_build_job_uses_humble_container(workflow: dict) -> None:
    container = workflow["jobs"]["build"].get("container")
    # `container:` may be a string or a mapping with `image:`.
    image = container if isinstance(container, str) else (container or {}).get("image")
    assert image, "build job must run inside a container image"
    assert "humble" in image, f"build job must target ROS Humble, got {image!r}"


def test_build_job_checks_out_submodules(workflow: dict) -> None:
    steps = workflow["jobs"]["build"]["steps"]
    found = False
    for step in steps:
        if not isinstance(step, dict):
            continue
        uses = step.get("uses", "")
        if uses.startswith("actions/checkout"):
            with_block = step.get("with") or {}
            sub = with_block.get("submodules")
            if sub in (True, "true", "recursive"):
                found = True
                break
    assert found, "build job must checkout submodules (recursive)"


def test_build_job_runs_colcon_build(workflow: dict) -> None:
    steps = workflow["jobs"]["build"]["steps"]
    runs = " \n ".join(s.get("run", "") for s in steps if isinstance(s, dict))
    assert "colcon build" in runs, "build job must run `colcon build`"


def test_build_job_runs_unit_tests(workflow: dict) -> None:
    steps = workflow["jobs"]["build"]["steps"]
    runs = " \n ".join(s.get("run", "") for s in steps if isinstance(s, dict))
    assert "scripts/run_tests.sh" in runs, "build job must call scripts/run_tests.sh"
    assert "--unit-only" in runs, "build job must use --unit-only (integration needs a live sim)"


def test_build_job_runs_rosdep_install(workflow: dict) -> None:
    steps = workflow["jobs"]["build"]["steps"]
    runs = " \n ".join(s.get("run", "") for s in steps if isinstance(s, dict))
    assert "rosdep install" in runs, "build job must rosdep-install workspace deps"
    # Must skip the ADR-0005 packages so rosdep doesn't fail on MuJoCo/catkin deps.
    assert (
        "cartesian_controller_simulation" in runs
    ), "rosdep install must skip cartesian_controller_simulation (ADR-0005)"
    assert (
        "cartesian_controller_tests" in runs
    ), "rosdep install must skip cartesian_controller_tests (ADR-0005)"


def test_build_job_depends_on_lint(workflow: dict) -> None:
    needs = workflow["jobs"]["build"].get("needs")
    if isinstance(needs, str):
        needs = [needs]
    assert needs and "lint" in needs, "build job must depend on lint job"


def test_workflow_runs_on_ubuntu_2204(workflow: dict) -> None:
    for job_name in ("lint", "build"):
        runner = workflow["jobs"][job_name].get("runs-on")
        assert (
            runner == "ubuntu-22.04"
        ), f"job {job_name!r} must run on ubuntu-22.04 (ROS Humble target), got {runner!r}"
