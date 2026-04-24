"""Unit tests for ``tests/integration/r2_run_dir.py``.

Pinned behaviour: a narrow UTC-timestamped run-directory helper for
the R2 integration test matrix (M6.12 / M6.13 / M6.14). The companion
writer :mod:`r2_run_artefact` declines to pick a timestamp — this
helper is the piece the writer's docstring refers to as "R2 tests will
reuse that convention".
"""

from __future__ import annotations

import importlib.util
import re
import sys
from datetime import datetime, timedelta, timezone
from pathlib import Path

import pytest

# Direct file-based import — tests/integration/ is not a package on
# sys.path (see the module-loading repo memory).
_MOD_PATH = Path(__file__).resolve().parents[1] / "integration" / "r2_run_dir.py"
_spec = importlib.util.spec_from_file_location("r2_run_dir", _MOD_PATH)
assert _spec is not None and _spec.loader is not None
r2_run_dir = importlib.util.module_from_spec(_spec)
sys.modules["r2_run_dir"] = r2_run_dir
_spec.loader.exec_module(r2_run_dir)


# --- export surface ---------------------------------------------------------


def test_export_surface():
    assert r2_run_dir.__all__ == (
        "DEFAULT_PREFIX",
        "DEFAULT_RUNS_ROOT",
        "TS_FORMAT",
        "make_r2_run_dir",
    )


def test_ts_format_matches_run_evaluation_convention():
    # Must match `evaluation/run_evaluation.py::make_run_dir` verbatim
    # so M5's `find_latest_run_dir` lexicographic sort keeps working
    # across R2 artefacts.
    assert r2_run_dir.TS_FORMAT == "%Y%m%dT%H%M%SZ"


def test_default_prefix_is_r2():
    assert r2_run_dir.DEFAULT_PREFIX == "r2"


def test_default_runs_root_points_at_repo_evaluation_runs():
    expected = Path(__file__).resolve().parents[2] / "evaluation" / "runs"
    assert r2_run_dir.DEFAULT_RUNS_ROOT == expected


# --- happy path -------------------------------------------------------------


def _pinned_clock(ts: datetime):
    return lambda: ts


def test_happy_path_creates_dir_with_expected_name(tmp_path):
    ts = datetime(2026, 4, 24, 22, 6, 25, tzinfo=timezone.utc)
    run_dir = r2_run_dir.make_r2_run_dir(tmp_path, clock=_pinned_clock(ts))
    assert run_dir == tmp_path / "r2__20260424T220625Z"
    assert run_dir.is_dir()


def test_happy_path_returns_path_object(tmp_path):
    ts = datetime(2026, 4, 24, tzinfo=timezone.utc)
    run_dir = r2_run_dir.make_r2_run_dir(tmp_path, clock=_pinned_clock(ts))
    assert isinstance(run_dir, Path)


def test_creates_missing_runs_root(tmp_path):
    deep = tmp_path / "a" / "b" / "c"
    assert not deep.exists()
    ts = datetime(2026, 1, 1, tzinfo=timezone.utc)
    run_dir = r2_run_dir.make_r2_run_dir(deep, clock=_pinned_clock(ts))
    assert run_dir.is_dir()
    assert run_dir.parent == deep


def test_custom_prefix(tmp_path):
    ts = datetime(2026, 4, 24, tzinfo=timezone.utc)
    run_dir = r2_run_dir.make_r2_run_dir(tmp_path, clock=_pinned_clock(ts), prefix="m6sweep")
    assert run_dir.name == "m6sweep__20260424T000000Z"


def test_suffix_appended_after_timestamp(tmp_path):
    ts = datetime(2026, 4, 24, 1, 2, 3, tzinfo=timezone.utc)
    run_dir = r2_run_dir.make_r2_run_dir(tmp_path, clock=_pinned_clock(ts), suffix="ur5e")
    assert run_dir.name == "r2__20260424T010203Z__ur5e"


def test_suffix_and_prefix_together(tmp_path):
    ts = datetime(2026, 4, 24, 1, 2, 3, tzinfo=timezone.utc)
    run_dir = r2_run_dir.make_r2_run_dir(
        tmp_path, clock=_pinned_clock(ts), prefix="sweep", suffix="worker7"
    )
    assert run_dir.name == "sweep__20260424T010203Z__worker7"


def test_default_runs_root_used_when_none(monkeypatch, tmp_path):
    monkeypatch.setattr(r2_run_dir, "DEFAULT_RUNS_ROOT", tmp_path / "runs")
    ts = datetime(2026, 4, 24, tzinfo=timezone.utc)
    run_dir = r2_run_dir.make_r2_run_dir(clock=_pinned_clock(ts))
    assert run_dir.parent == tmp_path / "runs"
    assert run_dir.name == "r2__20260424T000000Z"


def test_default_clock_produces_utc_timestamp_matching_format(tmp_path):
    # No clock injected — the default _utc_now() is used. Assert that the
    # directory name matches the pinned TS_FORMAT regex and parses back
    # to a datetime close to "now".
    before = datetime.now(timezone.utc)
    run_dir = r2_run_dir.make_r2_run_dir(tmp_path)
    after = datetime.now(timezone.utc)
    ts_part = run_dir.name.removeprefix("r2__")
    assert re.fullmatch(r"\d{8}T\d{6}Z", ts_part)
    parsed = datetime.strptime(ts_part, r2_run_dir.TS_FORMAT).replace(tzinfo=timezone.utc)
    # Allow a generous window for slow CI; strictly 'before <= parsed <= after'
    # after truncating to 1-second resolution.
    assert before - timedelta(seconds=1) <= parsed <= after + timedelta(seconds=1)


# --- collision handling -----------------------------------------------------


def test_collision_raises_file_exists_by_default(tmp_path):
    ts = datetime(2026, 4, 24, tzinfo=timezone.utc)
    r2_run_dir.make_r2_run_dir(tmp_path, clock=_pinned_clock(ts))
    with pytest.raises(FileExistsError):
        r2_run_dir.make_r2_run_dir(tmp_path, clock=_pinned_clock(ts))


def test_collision_allowed_with_exist_ok(tmp_path):
    ts = datetime(2026, 4, 24, tzinfo=timezone.utc)
    first = r2_run_dir.make_r2_run_dir(tmp_path, clock=_pinned_clock(ts))
    second = r2_run_dir.make_r2_run_dir(tmp_path, clock=_pinned_clock(ts), exist_ok=True)
    assert first == second


# --- lexicographic sort pinning --------------------------------------------


def test_names_sort_chronologically():
    # Pin the TS_FORMAT invariant that justifies M5's
    # `find_latest_run_dir` lexicographic sort.
    ts_a = datetime(2026, 4, 24, 1, 0, 0, tzinfo=timezone.utc).strftime(r2_run_dir.TS_FORMAT)
    ts_b = datetime(2026, 4, 24, 1, 0, 1, tzinfo=timezone.utc).strftime(r2_run_dir.TS_FORMAT)
    ts_c = datetime(2026, 4, 25, 0, 0, 0, tzinfo=timezone.utc).strftime(r2_run_dir.TS_FORMAT)
    ts_d = datetime(2027, 1, 1, 0, 0, 0, tzinfo=timezone.utc).strftime(r2_run_dir.TS_FORMAT)
    names = [f"r2__{t}" for t in (ts_d, ts_b, ts_a, ts_c)]
    assert sorted(names) == [
        f"r2__{ts_a}",
        f"r2__{ts_b}",
        f"r2__{ts_c}",
        f"r2__{ts_d}",
    ]


# --- validation: runs_root --------------------------------------------------


def test_runs_root_rejects_str():
    # Pinning "one canonical path type" — accepting str would let
    # callers drift away from pathlib.Path across the pre-bake chain.
    with pytest.raises(TypeError, match="runs_root must be pathlib.Path"):
        r2_run_dir.make_r2_run_dir("/tmp/foo")  # type: ignore[arg-type]


def test_runs_root_rejects_int():
    with pytest.raises(TypeError):
        r2_run_dir.make_r2_run_dir(42)  # type: ignore[arg-type]


# --- validation: prefix / suffix --------------------------------------------


@pytest.mark.parametrize(
    "bad,kwarg_name",
    [
        ("", "prefix"),
        ("", "suffix"),
        ("a/b", "prefix"),
        ("a/b", "suffix"),
        ("a\\b", "prefix"),
        ("a__b", "prefix"),
        ("a__b", "suffix"),
        (" leading", "prefix"),
        ("trailing ", "suffix"),
    ],
)
def test_invalid_prefix_or_suffix(tmp_path, bad, kwarg_name):
    ts = datetime(2026, 4, 24, tzinfo=timezone.utc)
    kwargs = {"clock": _pinned_clock(ts), kwarg_name: bad}
    with pytest.raises(ValueError, match=f"r2_run_dir: {kwarg_name}"):
        r2_run_dir.make_r2_run_dir(tmp_path, **kwargs)


@pytest.mark.parametrize("bad", [42, 3.14, b"bytes", ["list"], object()])
def test_prefix_non_str_rejected(tmp_path, bad):
    with pytest.raises(TypeError, match="prefix must be str"):
        r2_run_dir.make_r2_run_dir(tmp_path, prefix=bad)  # type: ignore[arg-type]


@pytest.mark.parametrize("bad", [42, 3.14, b"bytes", ["list"], object()])
def test_suffix_non_str_rejected(tmp_path, bad):
    ts = datetime(2026, 4, 24, tzinfo=timezone.utc)
    with pytest.raises(TypeError, match="suffix must be str"):
        r2_run_dir.make_r2_run_dir(
            tmp_path,
            clock=_pinned_clock(ts),
            suffix=bad,  # type: ignore[arg-type]
        )


def test_suffix_none_is_permitted(tmp_path):
    ts = datetime(2026, 4, 24, tzinfo=timezone.utc)
    run_dir = r2_run_dir.make_r2_run_dir(tmp_path, clock=_pinned_clock(ts), suffix=None)
    assert "__" in run_dir.name
    # Exactly one '__' separator when no suffix.
    assert run_dir.name.count("__") == 1


# --- validation: clock ------------------------------------------------------


def test_clock_non_callable_rejected(tmp_path):
    with pytest.raises(TypeError, match="clock must be callable"):
        r2_run_dir.make_r2_run_dir(tmp_path, clock=42)  # type: ignore[arg-type]


def test_clock_returns_non_datetime(tmp_path):
    with pytest.raises(TypeError, match="clock\\(\\) must return datetime"):
        r2_run_dir.make_r2_run_dir(tmp_path, clock=lambda: "not a datetime")


def test_clock_returns_naive_datetime(tmp_path):
    naive = datetime(2026, 4, 24)  # no tzinfo
    with pytest.raises(ValueError, match="naive datetime"):
        r2_run_dir.make_r2_run_dir(tmp_path, clock=lambda: naive)


def test_clock_returns_non_utc_datetime(tmp_path):
    tz = timezone(timedelta(hours=8))  # e.g. Asia/Shanghai
    non_utc = datetime(2026, 4, 24, tzinfo=tz)
    with pytest.raises(ValueError, match="expected UTC"):
        r2_run_dir.make_r2_run_dir(tmp_path, clock=lambda: non_utc)


# --- validation: exist_ok ---------------------------------------------------


@pytest.mark.parametrize("bad", [1, 0, "yes", None, [True]])
def test_exist_ok_non_bool_rejected(tmp_path, bad):
    ts = datetime(2026, 4, 24, tzinfo=timezone.utc)
    with pytest.raises(TypeError, match="exist_ok must be bool"):
        r2_run_dir.make_r2_run_dir(
            tmp_path,
            clock=_pinned_clock(ts),
            exist_ok=bad,  # type: ignore[arg-type]
        )


# --- integration: compose with r2_run_artefact ------------------------------


def test_compose_with_r2_run_artefact(tmp_path):
    # End-to-end: make_r2_run_dir -> write_r2_artefact. Proves the
    # helper returns a directory the companion writer accepts without
    # any glue code.
    import yaml

    # Load the writer the same way the writer's own tests do.
    writer_path = Path(__file__).resolve().parents[1] / "integration" / "r2_run_artefact.py"
    spec = importlib.util.spec_from_file_location("r2_run_artefact_compose", writer_path)
    assert spec is not None and spec.loader is not None
    writer = importlib.util.module_from_spec(spec)
    sys.modules["r2_run_artefact_compose"] = writer
    spec.loader.exec_module(writer)

    ts = datetime(2026, 4, 24, 22, 6, 25, tzinfo=timezone.utc)
    run_dir = r2_run_dir.make_r2_run_dir(tmp_path, clock=_pinned_clock(ts))

    artefact = writer.R2Artefact(
        stage=1,
        arm="ur5e",
        controller="joint_trajectory_controller",
        payload="no_payload",
        theoretical={"response_model": "first_order_lag"},
        measured={"steady_state_err_rad": 0.001},
        passed=True,
    )
    out = writer.write_r2_artefact(run_dir, artefact)
    assert out.parent == run_dir
    assert out.exists()
    data = yaml.safe_load(out.read_text())
    assert data["arm"] == "ur5e"
    assert data["passed"] is True
