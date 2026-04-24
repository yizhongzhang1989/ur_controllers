"""Unit tests for ``tests/integration/r2_aggregate.py``.

Pinned behaviour: a narrow aggregator that walks a run directory of
R2 artefacts (via :mod:`r2_find_artefacts` + :mod:`r2_read_artefact`)
and returns a :class:`R2RunSummary` with deterministic-ordered
artefacts plus per-axis :class:`R2Tally` breakdowns.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from types import MappingProxyType

import pytest

# --- module loading ---------------------------------------------------------
_INTEG_DIR = Path(__file__).resolve().parents[1] / "integration"


def _load(name: str):
    if name in sys.modules:
        return sys.modules[name]
    spec = importlib.util.spec_from_file_location(name, _INTEG_DIR / f"{name}.py")
    assert spec is not None and spec.loader is not None
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


_el = _load("expectations_loader")
_ra = _load("r2_run_artefact")
_find = _load("r2_find_artefacts")
_read = _load("r2_read_artefact")
r2_aggregate = _load("r2_aggregate")


# --- helpers ----------------------------------------------------------------


def _write(
    run_dir: Path,
    *,
    stage=1,
    arm="ur5e",
    controller="crisp_joint_impedance",
    payload="no_payload",
    passed=True,
    reasons=(),
    theoretical=None,
    measured=None,
    metadata=None,
) -> Path:
    """Write a minimal valid artefact and return its path."""
    art = _ra.R2Artefact(
        stage=stage,
        arm=arm,
        controller=controller,
        payload=payload,
        theoretical=theoretical if theoretical is not None else {"omega_n": 1.0},
        measured=measured if measured is not None else {"rms": 0.01},
        passed=passed,
        reasons=reasons,
        metadata=metadata if metadata is not None else {},
    )
    return _ra.write_r2_artefact(run_dir, art)


# --- export surface ---------------------------------------------------------


def test_export_surface():
    assert r2_aggregate.__all__ == (
        "R2RunSummary",
        "R2Tally",
        "aggregate_r2_run",
    )


def test_r2tally_is_frozen():
    t = r2_aggregate.R2Tally(total=2, passed=1, failed=1)
    with pytest.raises(Exception):
        t.total = 3  # type: ignore[misc]


def test_r2runsummary_is_frozen():
    s = r2_aggregate.R2RunSummary(
        run_dir=Path("."),
        artefacts=(),
        total=0,
        passed=0,
        failed=0,
        overall_pass=False,
    )
    with pytest.raises(Exception):
        s.total = 1  # type: ignore[misc]


# --- input validation -------------------------------------------------------


def test_aggregate_rejects_non_path_run_dir_str():
    with pytest.raises(TypeError, match=r"r2_aggregate: run_dir must be pathlib\.Path"):
        r2_aggregate.aggregate_r2_run(".")  # type: ignore[arg-type]


def test_aggregate_rejects_non_path_run_dir_none():
    with pytest.raises(TypeError, match=r"r2_aggregate:"):
        r2_aggregate.aggregate_r2_run(None)  # type: ignore[arg-type]


def test_aggregate_rejects_non_path_run_dir_int():
    with pytest.raises(TypeError, match=r"r2_aggregate:"):
        r2_aggregate.aggregate_r2_run(3)  # type: ignore[arg-type]


def test_aggregate_missing_run_dir_raises_filenotfounderror(tmp_path):
    missing = tmp_path / "does_not_exist"
    with pytest.raises(FileNotFoundError):
        r2_aggregate.aggregate_r2_run(missing)


def test_aggregate_file_as_run_dir_raises_notadirectoryerror(tmp_path):
    f = tmp_path / "not_a_dir"
    f.write_text("x", encoding="utf-8")
    with pytest.raises(NotADirectoryError):
        r2_aggregate.aggregate_r2_run(f)


def test_aggregate_half_matching_filename_raises(tmp_path):
    # A file matching r2_stage*.yaml but with a bogus stage number.
    (tmp_path / "r2_stage9_ur5e_foo_no_payload.yaml").write_text("x", encoding="utf-8")
    with pytest.raises(ValueError):
        r2_aggregate.aggregate_r2_run(tmp_path)


def test_aggregate_corrupted_artefact_raises(tmp_path):
    # Write a valid artefact, then overwrite its contents with junk.
    p = _write(tmp_path)
    p.write_text("not: [valid\n", encoding="utf-8")
    with pytest.raises(ValueError):
        r2_aggregate.aggregate_r2_run(tmp_path)


# --- empty run dir ----------------------------------------------------------


def test_aggregate_empty_run_dir(tmp_path):
    s = r2_aggregate.aggregate_r2_run(tmp_path)
    assert s.total == 0
    assert s.passed == 0
    assert s.failed == 0
    assert s.overall_pass is False
    assert s.artefacts == ()
    assert s.failed_artefacts == ()
    assert dict(s.by_stage) == {}
    assert dict(s.by_arm) == {}
    assert dict(s.by_controller) == {}
    assert dict(s.by_payload) == {}


def test_aggregate_run_dir_with_only_unrelated_files(tmp_path):
    # Files that don't match r2_stage*.yaml are silently skipped by find_r2_artefacts.
    (tmp_path / "notes.md").write_text("hello", encoding="utf-8")
    (tmp_path / "scratch.yaml").write_text("a: 1", encoding="utf-8")
    (tmp_path / "README").write_text("_", encoding="utf-8")
    s = r2_aggregate.aggregate_r2_run(tmp_path)
    assert s.total == 0
    assert s.overall_pass is False


def test_aggregate_preserves_run_dir_argument(tmp_path):
    s = r2_aggregate.aggregate_r2_run(tmp_path)
    assert s.run_dir == tmp_path
    assert isinstance(s.run_dir, Path)


# --- single artefact, happy path --------------------------------------------


def test_aggregate_single_passing_artefact(tmp_path):
    _write(tmp_path)
    s = r2_aggregate.aggregate_r2_run(tmp_path)
    assert s.total == 1
    assert s.passed == 1
    assert s.failed == 0
    assert s.overall_pass is True
    assert s.failed_artefacts == ()
    assert len(s.artefacts) == 1
    art = s.artefacts[0]
    assert art.stage == 1
    assert art.arm == "ur5e"
    assert art.controller == "crisp_joint_impedance"
    assert art.payload == "no_payload"
    assert art.passed is True


def test_aggregate_single_failing_artefact(tmp_path):
    _write(tmp_path, passed=False, reasons=("omega_n out of tolerance",))
    s = r2_aggregate.aggregate_r2_run(tmp_path)
    assert s.total == 1
    assert s.passed == 0
    assert s.failed == 1
    assert s.overall_pass is False
    assert len(s.failed_artefacts) == 1
    assert s.failed_artefacts[0].reasons == ("omega_n out of tolerance",)


def test_aggregate_artefact_is_R2Artefact_instance(tmp_path):
    _write(tmp_path)
    s = r2_aggregate.aggregate_r2_run(tmp_path)
    # Class identity via the importlib-based sibling loader is shared.
    assert isinstance(s.artefacts[0], _ra.R2Artefact)


# --- overall_pass semantics -------------------------------------------------


def test_overall_pass_true_requires_nonempty_and_all_passed(tmp_path):
    _write(tmp_path)
    s = r2_aggregate.aggregate_r2_run(tmp_path)
    assert s.overall_pass is True


def test_overall_pass_false_on_empty(tmp_path):
    s = r2_aggregate.aggregate_r2_run(tmp_path)
    assert s.total == 0 and s.overall_pass is False


def test_overall_pass_false_on_any_failure(tmp_path):
    _write(tmp_path, arm="ur5e")
    _write(tmp_path, arm="ur15", passed=False, reasons=("x",))
    s = r2_aggregate.aggregate_r2_run(tmp_path)
    assert s.total == 2
    assert s.passed == 1
    assert s.failed == 1
    assert s.overall_pass is False


def test_overall_pass_true_on_all_passed_mixed_combos(tmp_path):
    _write(tmp_path, stage=1, arm="ur5e", payload="no_payload")
    _write(tmp_path, stage=2, arm="ur15", payload="small_payload")
    _write(
        tmp_path,
        stage=3,
        arm="ur5e",
        controller="cartesian_motion_controller",
        payload="large_payload",
    )
    s = r2_aggregate.aggregate_r2_run(tmp_path)
    assert s.total == 3
    assert s.passed == 3
    assert s.failed == 0
    assert s.overall_pass is True


# --- artefact ordering ------------------------------------------------------


def test_aggregate_artefacts_sorted_by_path(tmp_path):
    # Write in deliberately mixed order.
    _write(tmp_path, stage=3, arm="ur15", payload="large_payload")
    _write(tmp_path, stage=1, arm="ur5e", payload="no_payload")
    _write(tmp_path, stage=2, arm="ur5e", payload="small_payload")
    s = r2_aggregate.aggregate_r2_run(tmp_path)
    paths = [
        _ra.artefact_filename(stage=a.stage, arm=a.arm, controller=a.controller, payload=a.payload)
        for a in s.artefacts
    ]
    assert paths == sorted(paths)


def test_aggregate_repeatable(tmp_path):
    _write(tmp_path, stage=1, arm="ur5e")
    _write(tmp_path, stage=2, arm="ur15")
    s1 = r2_aggregate.aggregate_r2_run(tmp_path)
    s2 = r2_aggregate.aggregate_r2_run(tmp_path)
    assert [a.stage for a in s1.artefacts] == [a.stage for a in s2.artefacts]
    assert dict(s1.by_stage) == dict(s2.by_stage)
    assert dict(s1.by_arm) == dict(s2.by_arm)


# --- counts consistency -----------------------------------------------------


@pytest.mark.parametrize("n_pass,n_fail", [(0, 0), (1, 0), (0, 1), (2, 1), (1, 2), (3, 3)])
def test_aggregate_counts_consistent(tmp_path, n_pass, n_fail):
    # Build unique combos so filenames don't collide.
    combos = []
    for i, a in enumerate(("ur5e", "ur15")):
        for j, p in enumerate(("no_payload", "small_payload", "large_payload")):
            for st in (1, 2, 3):
                combos.append((st, a, f"ctl{len(combos)}", p))
    i = 0
    for _ in range(n_pass):
        st, a, c, p = combos[i]
        i += 1
        _write(tmp_path, stage=st, arm=a, controller=c, payload=p, passed=True)
    for _ in range(n_fail):
        st, a, c, p = combos[i]
        i += 1
        _write(tmp_path, stage=st, arm=a, controller=c, payload=p, passed=False, reasons=("boom",))
    s = r2_aggregate.aggregate_r2_run(tmp_path)
    assert s.total == n_pass + n_fail
    assert s.passed == n_pass
    assert s.failed == n_fail
    assert len(s.artefacts) == s.total
    assert len(s.failed_artefacts) == n_fail
    assert all(not a.passed for a in s.failed_artefacts)


def test_failed_artefacts_preserve_order(tmp_path):
    _write(tmp_path, stage=1, arm="ur5e", passed=False, reasons=("a",))
    _write(tmp_path, stage=2, arm="ur5e", passed=True)
    _write(tmp_path, stage=3, arm="ur5e", passed=False, reasons=("b",))
    s = r2_aggregate.aggregate_r2_run(tmp_path)
    # Among artefacts: sorted by path gives stage1, stage2, stage3. The
    # failed subset preserves that sub-order.
    assert [a.stage for a in s.failed_artefacts] == [1, 3]


# --- per-axis tallies -------------------------------------------------------


def test_by_stage_tally(tmp_path):
    # stage 1: 2 passed, 1 failed; stage 2: 1 passed; stage 3: 1 failed.
    _write(tmp_path, stage=1, arm="ur5e", controller="c1", payload="no_payload")
    _write(tmp_path, stage=1, arm="ur15", controller="c1", payload="no_payload")
    _write(
        tmp_path,
        stage=1,
        arm="ur5e",
        controller="c1",
        payload="small_payload",
        passed=False,
        reasons=("x",),
    )
    _write(tmp_path, stage=2, arm="ur5e", controller="c2", payload="no_payload")
    _write(
        tmp_path,
        stage=3,
        arm="ur5e",
        controller="c3",
        payload="no_payload",
        passed=False,
        reasons=("y",),
    )
    s = r2_aggregate.aggregate_r2_run(tmp_path)
    assert dict(s.by_stage) == {
        1: r2_aggregate.R2Tally(total=3, passed=2, failed=1),
        2: r2_aggregate.R2Tally(total=1, passed=1, failed=0),
        3: r2_aggregate.R2Tally(total=1, passed=0, failed=1),
    }


def test_by_arm_tally(tmp_path):
    _write(tmp_path, stage=1, arm="ur5e", controller="c1", payload="no_payload")
    _write(
        tmp_path,
        stage=2,
        arm="ur5e",
        controller="c2",
        payload="no_payload",
        passed=False,
        reasons=("x",),
    )
    _write(tmp_path, stage=1, arm="ur15", controller="c1", payload="no_payload")
    s = r2_aggregate.aggregate_r2_run(tmp_path)
    assert dict(s.by_arm) == {
        "ur15": r2_aggregate.R2Tally(total=1, passed=1, failed=0),
        "ur5e": r2_aggregate.R2Tally(total=2, passed=1, failed=1),
    }


def test_by_controller_tally(tmp_path):
    _write(tmp_path, stage=1, arm="ur5e", controller="crisp_joint_impedance", payload="no_payload")
    _write(tmp_path, stage=1, arm="ur15", controller="crisp_joint_impedance", payload="no_payload")
    _write(
        tmp_path,
        stage=1,
        arm="ur5e",
        controller="simple_jimp",
        payload="no_payload",
        passed=False,
        reasons=("z",),
    )
    s = r2_aggregate.aggregate_r2_run(tmp_path)
    assert dict(s.by_controller) == {
        "crisp_joint_impedance": r2_aggregate.R2Tally(total=2, passed=2, failed=0),
        "simple_jimp": r2_aggregate.R2Tally(total=1, passed=0, failed=1),
    }


def test_by_payload_tally(tmp_path):
    for st, pl, ok in [
        (1, "no_payload", True),
        (2, "no_payload", True),
        (1, "small_payload", False),
        (3, "large_payload", True),
        (2, "large_payload", False),
    ]:
        _write(
            tmp_path,
            stage=st,
            arm="ur5e",
            controller=f"c{st}{pl}",
            payload=pl,
            passed=ok,
            reasons=() if ok else ("x",),
        )
    s = r2_aggregate.aggregate_r2_run(tmp_path)
    assert dict(s.by_payload) == {
        "large_payload": r2_aggregate.R2Tally(total=2, passed=1, failed=1),
        "no_payload": r2_aggregate.R2Tally(total=2, passed=2, failed=0),
        "small_payload": r2_aggregate.R2Tally(total=1, passed=0, failed=1),
    }


def test_tally_sums_match_top_level(tmp_path):
    _write(tmp_path, stage=1, arm="ur5e", controller="c1", payload="no_payload")
    _write(
        tmp_path,
        stage=2,
        arm="ur15",
        controller="c2",
        payload="small_payload",
        passed=False,
        reasons=("r",),
    )
    _write(tmp_path, stage=3, arm="ur5e", controller="c3", payload="large_payload")
    s = r2_aggregate.aggregate_r2_run(tmp_path)
    for axis in (s.by_stage, s.by_arm, s.by_controller, s.by_payload):
        assert sum(t.total for t in axis.values()) == s.total
        assert sum(t.passed for t in axis.values()) == s.passed
        assert sum(t.failed for t in axis.values()) == s.failed


def test_tally_total_equals_passed_plus_failed(tmp_path):
    _write(tmp_path, stage=1, arm="ur5e", controller="c1", payload="no_payload")
    _write(
        tmp_path,
        stage=1,
        arm="ur5e",
        controller="c2",
        payload="no_payload",
        passed=False,
        reasons=("r",),
    )
    s = r2_aggregate.aggregate_r2_run(tmp_path)
    for t in s.by_stage.values():
        assert t.total == t.passed + t.failed


# --- tally maps are read-only ----------------------------------------------


def test_by_stage_is_mappingproxy(tmp_path):
    _write(tmp_path)
    s = r2_aggregate.aggregate_r2_run(tmp_path)
    assert isinstance(s.by_stage, MappingProxyType)
    with pytest.raises(TypeError):
        s.by_stage[1] = r2_aggregate.R2Tally(0, 0, 0)  # type: ignore[index]


def test_by_arm_is_mappingproxy(tmp_path):
    _write(tmp_path)
    s = r2_aggregate.aggregate_r2_run(tmp_path)
    assert isinstance(s.by_arm, MappingProxyType)


def test_by_controller_is_mappingproxy(tmp_path):
    _write(tmp_path)
    s = r2_aggregate.aggregate_r2_run(tmp_path)
    assert isinstance(s.by_controller, MappingProxyType)


def test_by_payload_is_mappingproxy(tmp_path):
    _write(tmp_path)
    s = r2_aggregate.aggregate_r2_run(tmp_path)
    assert isinstance(s.by_payload, MappingProxyType)


def test_empty_tally_maps_are_mappingproxies(tmp_path):
    s = r2_aggregate.aggregate_r2_run(tmp_path)
    assert isinstance(s.by_stage, MappingProxyType)
    assert isinstance(s.by_arm, MappingProxyType)
    assert isinstance(s.by_controller, MappingProxyType)
    assert isinstance(s.by_payload, MappingProxyType)


# --- deterministic key ordering --------------------------------------------


def test_by_stage_iteration_order_stable(tmp_path):
    # Deliberately write in non-sorted order.
    _write(tmp_path, stage=3, arm="ur5e", controller="c3", payload="no_payload")
    _write(tmp_path, stage=1, arm="ur5e", controller="c1", payload="no_payload")
    _write(tmp_path, stage=2, arm="ur5e", controller="c2", payload="no_payload")
    s = r2_aggregate.aggregate_r2_run(tmp_path)
    # Sorted by stringified key: "1", "2", "3".
    assert list(s.by_stage.keys()) == [1, 2, 3]


def test_by_arm_iteration_order_stable(tmp_path):
    _write(tmp_path, arm="ur5e", controller="c1")
    _write(tmp_path, arm="ur15", controller="c2")
    s = r2_aggregate.aggregate_r2_run(tmp_path)
    assert list(s.by_arm.keys()) == ["ur15", "ur5e"]


def test_by_controller_iteration_order_stable(tmp_path):
    _write(tmp_path, controller="zulu")
    _write(tmp_path, arm="ur15", controller="alpha")
    _write(tmp_path, stage=2, controller="mike")
    s = r2_aggregate.aggregate_r2_run(tmp_path)
    assert list(s.by_controller.keys()) == ["alpha", "mike", "zulu"]


def test_by_payload_iteration_order_stable(tmp_path):
    _write(tmp_path, payload="small_payload")
    _write(tmp_path, stage=2, payload="no_payload")
    _write(tmp_path, stage=3, payload="large_payload")
    s = r2_aggregate.aggregate_r2_run(tmp_path)
    assert list(s.by_payload.keys()) == ["large_payload", "no_payload", "small_payload"]


# --- ignores unrelated files ------------------------------------------------


def test_aggregate_ignores_unrelated_files_with_artefacts_present(tmp_path):
    _write(tmp_path)
    (tmp_path / "notes.md").write_text("hi", encoding="utf-8")
    (tmp_path / "log.txt").write_text("x", encoding="utf-8")
    s = r2_aggregate.aggregate_r2_run(tmp_path)
    assert s.total == 1


# --- non-recursive discovery -----------------------------------------------


def test_aggregate_non_recursive(tmp_path):
    nested = tmp_path / "nested"
    nested.mkdir()
    _write(nested, stage=1, arm="ur5e")
    _write(tmp_path, stage=2, arm="ur5e")
    s = r2_aggregate.aggregate_r2_run(tmp_path)
    assert s.total == 1
    assert s.artefacts[0].stage == 2


# --- large end-to-end matrix ------------------------------------------------


def test_aggregate_full_matrix(tmp_path):
    # 3 stages x 2 arms x 3 payloads = 18 combos, all passed.
    for stage in (1, 2, 3):
        for arm in ("ur5e", "ur15"):
            for payload in ("no_payload", "small_payload", "large_payload"):
                _write(tmp_path, stage=stage, arm=arm, controller=f"ctl_s{stage}", payload=payload)
    s = r2_aggregate.aggregate_r2_run(tmp_path)
    assert s.total == 18
    assert s.passed == 18
    assert s.failed == 0
    assert s.overall_pass is True
    assert all(t.total == 6 for t in s.by_stage.values())
    assert all(t.total == 9 for t in s.by_arm.values())
    assert all(t.total == 6 for t in s.by_payload.values())


def test_aggregate_full_matrix_one_failure_flips_overall(tmp_path):
    for stage in (1, 2, 3):
        for arm in ("ur5e", "ur15"):
            for payload in ("no_payload", "small_payload", "large_payload"):
                passed = not (stage == 2 and arm == "ur15" and payload == "large_payload")
                _write(
                    tmp_path,
                    stage=stage,
                    arm=arm,
                    controller=f"ctl_s{stage}",
                    payload=payload,
                    passed=passed,
                    reasons=() if passed else ("boom",),
                )
    s = r2_aggregate.aggregate_r2_run(tmp_path)
    assert s.total == 18
    assert s.failed == 1
    assert s.passed == 17
    assert s.overall_pass is False
    assert len(s.failed_artefacts) == 1
    f = s.failed_artefacts[0]
    assert (f.stage, f.arm, f.payload) == (2, "ur15", "large_payload")


# --- independence from prior runs ------------------------------------------


def test_aggregate_two_separate_run_dirs_independent(tmp_path):
    a = tmp_path / "runA"
    b = tmp_path / "runB"
    a.mkdir()
    b.mkdir()
    _write(a, stage=1, arm="ur5e")
    _write(b, stage=3, arm="ur15", passed=False, reasons=("x",))
    sa = r2_aggregate.aggregate_r2_run(a)
    sb = r2_aggregate.aggregate_r2_run(b)
    assert sa.total == 1 and sa.overall_pass is True
    assert sb.total == 1 and sb.overall_pass is False
    assert sa.run_dir == a and sb.run_dir == b


# --- class identity across chain -------------------------------------------


def test_aggregate_artefact_identity_shared_with_reader(tmp_path):
    """The R2Artefact class returned by aggregate_r2_run is the same
    class the reader returns. Guards against the file-based importlib
    loader creating duplicate class objects under two module names."""
    p = _write(tmp_path)
    directly_read = _read.read_r2_artefact(p)
    s = r2_aggregate.aggregate_r2_run(tmp_path)
    assert type(s.artefacts[0]) is type(directly_read)
    assert type(s.artefacts[0]) is _ra.R2Artefact
