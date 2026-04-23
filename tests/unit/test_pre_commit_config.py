"""Structural tests for `.pre-commit-config.yaml` and the related configs.

These tests do **not** invoke the `pre-commit` binary itself (that requires
network access for hook installs and is too slow for the unit-test gate).
They pin the structure of the config so a future edit cannot silently drop
one of the three hooks the AGENTS contract requires (clang-format, ruff,
trailing whitespace) or relax the third_party/build/install excludes.
"""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
PRE_COMMIT_CONFIG = REPO_ROOT / ".pre-commit-config.yaml"
CLANG_FORMAT_CONFIG = REPO_ROOT / ".clang-format"
RUFF_CONFIG = REPO_ROOT / "ruff.toml"


@pytest.fixture(scope="module")
def config() -> dict:
    assert PRE_COMMIT_CONFIG.is_file(), f"missing {PRE_COMMIT_CONFIG}"
    with PRE_COMMIT_CONFIG.open() as f:
        data = yaml.safe_load(f)
    assert isinstance(data, dict), "pre-commit config must be a mapping"
    return data


def test_repos_block_present(config: dict) -> None:
    assert "repos" in config, "pre-commit config must declare a 'repos' list"
    assert isinstance(config["repos"], list)
    assert len(config["repos"]) >= 1


def _hook_ids(config: dict) -> set[str]:
    ids: set[str] = set()
    for repo in config["repos"]:
        for hook in repo.get("hooks", []):
            hook_id = hook.get("id")
            if hook_id:
                ids.add(hook_id)
    return ids


@pytest.mark.parametrize(
    "required_hook",
    [
        "trailing-whitespace",
        "ruff",
        "ruff-format",
        "clang-format",
    ],
)
def test_required_hook_configured(config: dict, required_hook: str) -> None:
    """AGENTS.md §4 requires clang-format, ruff, and trailing-whitespace."""
    assert required_hook in _hook_ids(config), (
        f"hook '{required_hook}' missing from .pre-commit-config.yaml"
    )


@pytest.mark.parametrize(
    "must_match",
    ["third_party/", "build/", "install/", "log/"],
)
def test_exclude_pattern_covers_readonly_paths(config: dict, must_match: str) -> None:
    """Read-only submodules and build artefacts must be excluded."""
    exclude = config.get("exclude", "")
    assert isinstance(exclude, str) and exclude
    # The exclude pattern is a verbose regex. Substring presence is enough
    # to catch accidental deletion of the path; we don't try to compile and
    # match a fake file path because that ties the test to regex internals.
    assert must_match.rstrip("/") in exclude, (
        f"exclude pattern does not mention '{must_match}': {exclude!r}"
    )


def test_clang_format_config_exists_and_uses_llvm_base() -> None:
    """AGENTS.md §4 specifies clang-format with the LLVM base style."""
    assert CLANG_FORMAT_CONFIG.is_file(), f"missing {CLANG_FORMAT_CONFIG}"
    with CLANG_FORMAT_CONFIG.open() as f:
        data = yaml.safe_load(f)
    assert isinstance(data, dict)
    assert data.get("BasedOnStyle") == "LLVM", (
        f"clang-format BasedOnStyle must be LLVM (got {data.get('BasedOnStyle')!r})"
    )


def test_ruff_config_exists_with_pinned_target() -> None:
    """ruff.toml must pin a target Python version so behaviour is reproducible."""
    assert RUFF_CONFIG.is_file(), f"missing {RUFF_CONFIG}"
    text = RUFF_CONFIG.read_text()
    assert 'target-version = "py310"' in text, (
        "ruff.toml must pin target-version to py310 (AGENTS.md §4: Python ≥3.10)"
    )


def test_pre_commit_hook_revs_are_pinned(config: dict) -> None:
    """Every external hook repo must pin a non-empty `rev` for reproducibility."""
    for repo in config["repos"]:
        rev = repo.get("rev")
        url = repo.get("repo")
        assert isinstance(rev, str) and rev, f"repo {url!r} missing pinned rev"
