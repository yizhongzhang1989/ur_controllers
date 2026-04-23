"""Structural tests for `scripts/setup_env.sh`.

These tests do **not** invoke `apt`, `rosdep`, or `pip`; they exercise the
`--dry-run` and `--help` paths and assert that the script's text pins the
prerequisites declared in README.md plus the ADR-0005 rosdep skip-keys.
The unit gate is headless and offline, matching the pattern used by
`tests/unit/test_pre_commit_config.py` and `tests/unit/test_ci_workflow.py`.
"""

from __future__ import annotations

import subprocess
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / "scripts" / "setup_env.sh"


@pytest.fixture(scope="module")
def script_text() -> str:
    assert SCRIPT.is_file(), f"missing {SCRIPT}"
    return SCRIPT.read_text()


def test_script_is_executable() -> None:
    assert SCRIPT.stat().st_mode & 0o111, f"{SCRIPT} must be executable"


def test_shebang_is_bash(script_text: str) -> None:
    first = script_text.splitlines()[0]
    assert first == "#!/usr/bin/env bash", f"unexpected shebang: {first!r}"


def test_help_flag_exits_zero_and_prints_usage() -> None:
    result = subprocess.run([str(SCRIPT), "--help"], capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stderr
    assert "setup_env.sh" in result.stdout
    assert "--dry-run" in result.stdout
    assert "--no-pre-commit" in result.stdout


def test_unknown_arg_exits_two() -> None:
    result = subprocess.run([str(SCRIPT), "--bogus"], capture_output=True, text=True, timeout=10)
    assert result.returncode == 2
    assert "unknown arg" in result.stderr


def test_dry_run_exits_zero_and_does_not_invoke_apt() -> None:
    """`--dry-run` must be safe to run on any host without side effects."""
    result = subprocess.run([str(SCRIPT), "--dry-run"], capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr
    out = result.stdout
    # All three stages are announced.
    assert "stage 1: apt install" in out
    assert "stage 2: rosdep" in out
    assert "stage 3: pip install pre-commit + ruff (user-site)" in out
    # Every actionable command is prefixed with DRY:.
    assert "DRY: " in out
    # Real `apt-get install` would emit progress lines like 'Reading package
    # lists' — the DRY path must not. Guard against accidental side effects.
    assert "Reading package lists" not in out
    assert "all stages passed" in out


def test_dry_run_no_pre_commit_skips_stage_three() -> None:
    result = subprocess.run(
        [str(SCRIPT), "--dry-run", "--no-pre-commit"],
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    out = result.stdout
    assert "stage 3: skipped (--no-pre-commit)" in out
    assert "pip install --user --upgrade pre-commit ruff" not in out


@pytest.mark.parametrize(
    "package",
    [
        # README.md "Prerequisites" block.
        "ros-humble-mujoco-ros2-control",
        "ros-humble-ur-description",
        "ros-humble-ros2-control",
        "ros-humble-ros2-controllers",
        "ros-humble-pinocchio",
        "ros-humble-rosbridge-suite",
        "ros-humble-ros-gz",
        "ros-humble-gz-ros2-control",
        # Build / test tooling our scripts assume.
        "python3-colcon-common-extensions",
        "python3-colcon-defaults",
        "python3-rosdep",
        "python3-pytest",
        "git",
        "build-essential",
    ],
)
def test_apt_package_listed(script_text: str, package: str) -> None:
    assert (
        package in script_text
    ), f"apt package '{package}' (from README Prerequisites) missing from setup_env.sh"


def test_rosdep_skip_keys_match_adr_0005(script_text: str) -> None:
    """ADR-0005: skip cartesian_controller_simulation + cartesian_controller_tests."""
    assert "--skip-keys" in script_text
    assert "cartesian_controller_simulation" in script_text
    assert "cartesian_controller_tests" in script_text


def test_rosdep_install_targets_src_and_third_party(script_text: str) -> None:
    assert "rosdep install --from-paths src third_party" in script_text
    # Defensive flags that match README.md and the CI workflow.
    assert "--ignore-src" in script_text
    assert "-r -y" in script_text


def test_rosdep_init_is_guarded(script_text: str) -> None:
    """`rosdep init` errors if already initialised — the script must guard."""
    assert "/etc/ros/rosdep/sources.list.d/20-default.list" in script_text


def test_pip_installs_pre_commit_and_ruff(script_text: str) -> None:
    assert "pre-commit ruff" in script_text
    assert "--user" in script_text


def test_pre_commit_install_wired(script_text: str) -> None:
    """Optional but useful per ROADMAP M0 bullet — wire the git hook."""
    assert "pre-commit install" in script_text


def test_uses_sudo_when_not_root(script_text: str) -> None:
    """The script must work both inside CI containers (root) and on a dev box."""
    assert "SUDO=" in script_text
    assert "command -v sudo" in script_text
