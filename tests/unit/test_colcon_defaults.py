"""Tests for the workspace-level colcon defaults file.

`colcon_defaults.yaml` at the repo root is auto-loaded by the
`python3-colcon-defaults` plugin and supplies the standard build/test
flags (ADR-0005). These tests pin its structure so a future edit can't
silently drop the skip list or the workspace boundaries.
"""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULTS_PATH = REPO_ROOT / "colcon_defaults.yaml"

# ADR-0005 — packages that must always be skipped.
REQUIRED_SKIPS = {
    "cartesian_controller_simulation",
    "cartesian_controller_tests",
}


@pytest.fixture(scope="module")
def defaults() -> dict:
    assert DEFAULTS_PATH.is_file(), f"missing {DEFAULTS_PATH}"
    data = yaml.safe_load(DEFAULTS_PATH.read_text())
    assert isinstance(data, dict), "colcon_defaults.yaml must be a mapping"
    return data


def test_has_build_and_test_sections(defaults: dict) -> None:
    assert "build" in defaults, "missing 'build' verb section"
    assert "test" in defaults, "missing 'test' verb section"
    assert isinstance(defaults["build"], dict)
    assert isinstance(defaults["test"], dict)


def test_build_uses_symlink_install(defaults: dict) -> None:
    assert defaults["build"].get("symlink-install") is True, (
        "build.symlink-install must be true so source edits don't require "
        "a rebuild for the python+launch packages"
    )


def test_build_base_paths(defaults: dict) -> None:
    base_paths = defaults["build"].get("base-paths")
    assert isinstance(base_paths, list), "build.base-paths must be a list"
    assert base_paths == ["src", "third_party"], (
        "build.base-paths must be exactly [src, third_party] to match the "
        "workspace layout documented in AGENTS.md §2"
    )


@pytest.mark.parametrize("verb", ["build", "test"])
def test_skip_list_present(defaults: dict, verb: str) -> None:
    skips = defaults[verb].get("packages-skip")
    assert isinstance(skips, list), f"{verb}.packages-skip must be a list"
    missing = REQUIRED_SKIPS - set(skips)
    assert not missing, (
        f"{verb}.packages-skip must include {sorted(REQUIRED_SKIPS)} "
        f"(ADR-0005); missing: {sorted(missing)}"
    )


def test_no_unknown_verbs(defaults: dict) -> None:
    # Guard against typos that would silently no-op (e.g. `built:` instead
    # of `build:`). Keep this list narrow; expanding it is intentional.
    allowed = {"build", "test"}
    unknown = set(defaults.keys()) - allowed
    assert not unknown, (
        f"unknown top-level verb(s) in colcon_defaults.yaml: {sorted(unknown)}; "
        "if you really want to add one, update this test deliberately"
    )
