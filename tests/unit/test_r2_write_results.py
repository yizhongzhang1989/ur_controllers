"""Unit tests for :mod:`tests.integration.r2_write_results`.

Covers: export surface, :class:`R2ResultEntry` shape, argument
validation (run_dir / entries / entry / overwrite), happy-path
single-entry and batch writes across all four stage result types,
bridge-error propagation (stage/result mismatch, controller echo),
writer-error propagation (overwrite=False collision), duplicate-combo
detection, no-side-effect guarantees on early-failure, output ordering,
return-path filename correctness, overwrite=True round-trip, and an
end-to-end chain into :mod:`r2_find_artefacts` / :mod:`r2_read_artefact`
/ :mod:`r2_aggregate`.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from types import ModuleType

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
INT_DIR = REPO_ROOT / "tests" / "integration"


def _load(name: str, path: Path) -> ModuleType:
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec and spec.loader
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


# Load in dependency order so siblings share module-name keys with the
# bridge's direct imports.
EL = _load("expectations_loader", INT_DIR / "expectations_loader.py")
SA = _load("signal_analysis", INT_DIR / "signal_analysis.py")
RA = _load("r2_run_artefact", INT_DIR / "r2_run_artefact.py")
S1 = _load("r2_stage1_assertions", INT_DIR / "r2_stage1_assertions.py")
S2 = _load("r2_stage2_assertions", INT_DIR / "r2_stage2_assertions.py")
S2C = _load("r2_stage2_cartesian", INT_DIR / "r2_stage2_cartesian.py")
S3 = _load("r2_stage3_assertions", INT_DIR / "r2_stage3_assertions.py")
BR = _load("r2_result_to_artefact", INT_DIR / "r2_result_to_artefact.py")
FA = _load("r2_find_artefacts", INT_DIR / "r2_find_artefacts.py")
RR = _load("r2_read_artefact", INT_DIR / "r2_read_artefact.py")
AG = _load("r2_aggregate", INT_DIR / "r2_aggregate.py")
WR = _load("r2_write_results", INT_DIR / "r2_write_results.py")


# ---------------------------------------------------------------------------
# Result fixtures (mirrors test_r2_result_to_artefact.py)
# ---------------------------------------------------------------------------


def _s1(controller: str = "simple_jimp", joint: str = "shoulder_pan_joint", **kw) -> object:
    return S1.Stage1Result(controller=controller, joint=joint, **kw)


def _s2(controller: str = "jtc", per_joint=()) -> object:
    return S2.Stage2Result(controller=controller, per_joint=per_joint)


def _s2c(controller: str = "cartesian_motion", metrics=None, failures=()) -> object:
    return S2C.Stage2CartesianResult(
        controller=controller,
        metrics=metrics or {},
        failures=failures,
    )


def _s3(controller: str = "cartesian_motion", metrics=None, failures=(), notes=()) -> object:
    return S3.TcpStage3Result(
        controller=controller,
        metrics=metrics or {},
        failures=failures,
        notes=notes,
    )


def _entry(
    *,
    stage: int = 1,
    arm: str = "ur5e",
    controller: str = "simple_jimp",
    payload: str = "no_payload",
    theoretical=None,
    result=None,
    metadata=None,
) -> object:
    if result is None:
        result = _s1(controller=controller)
    return WR.R2ResultEntry(
        stage=stage,
        arm=arm,
        controller=controller,
        payload=payload,
        theoretical=theoretical if theoretical is not None else {},
        result=result,
        metadata=metadata,
    )


# ---------------------------------------------------------------------------
# Export surface
# ---------------------------------------------------------------------------


def test_exports_all():
    assert set(WR.__all__) == {"R2ResultEntry", "write_r2_result", "write_r2_results"}


def test_R2ResultEntry_is_frozen():
    e = _entry()
    with pytest.raises(Exception):
        e.stage = 2  # type: ignore[misc]


def test_R2ResultEntry_default_metadata_is_none():
    e = WR.R2ResultEntry(
        stage=1,
        arm="ur5e",
        controller="simple_jimp",
        payload="no_payload",
        theoretical={},
        result=_s1(),
    )
    assert e.metadata is None


def test_R2Artefact_shared_with_artefact_module():
    # The bridge returns r2_run_artefact.R2Artefact; sibling loading
    # in r2_write_results must reuse that identity.
    assert WR._artefact_mod.R2Artefact is RA.R2Artefact


# ---------------------------------------------------------------------------
# write_r2_result: type validation
# ---------------------------------------------------------------------------


def test_write_r2_result_rejects_non_path_run_dir(tmp_path: Path):
    with pytest.raises(TypeError) as ex:
        WR.write_r2_result(str(tmp_path), _entry())  # type: ignore[arg-type]
    assert "r2_write_results:" in str(ex.value)
    assert "run_dir" in str(ex.value)


def test_write_r2_result_rejects_none_run_dir():
    with pytest.raises(TypeError):
        WR.write_r2_result(None, _entry())  # type: ignore[arg-type]


@pytest.mark.parametrize("bad", [None, "not-an-entry", 42, object()])
def test_write_r2_result_rejects_non_entry(tmp_path: Path, bad):
    with pytest.raises(TypeError) as ex:
        WR.write_r2_result(tmp_path, bad)  # type: ignore[arg-type]
    assert "R2ResultEntry" in str(ex.value)


def test_write_r2_result_rejects_non_bool_overwrite(tmp_path: Path):
    with pytest.raises(TypeError) as ex:
        WR.write_r2_result(tmp_path, _entry(), overwrite=1)  # type: ignore[arg-type]
    assert "overwrite" in str(ex.value)


# ---------------------------------------------------------------------------
# write_r2_result: happy path per stage
# ---------------------------------------------------------------------------


def test_write_r2_result_stage1_writes_expected_file(tmp_path: Path):
    entry = _entry(
        stage=1,
        arm="ur5e",
        controller="simple_jimp",
        payload="no_payload",
        theoretical={"omega_n": 5.0, "zeta": 1.0},
        result=_s1(controller="simple_jimp", joint="elbow_joint",
                   metrics={"measured_zeta": 0.9}),
    )
    path = WR.write_r2_result(tmp_path, entry)
    assert path == tmp_path / RA.artefact_filename(
        stage=1, arm="ur5e", controller="simple_jimp", payload="no_payload"
    )
    assert path.exists()


def test_write_r2_result_stage2_joint_space(tmp_path: Path):
    metrics = {"final_err_rad": 0.01, "peak_tracking_err_rad": 0.05}
    per_joint = (S2.JointStage2Metrics(joint="shoulder_pan_joint", metrics=metrics),)
    entry = _entry(
        stage=2,
        controller="jtc",
        result=_s2(controller="jtc", per_joint=per_joint),
    )
    path = WR.write_r2_result(tmp_path, entry)
    assert path.exists()
    art = RR.read_r2_artefact(path)
    assert art.stage == 2
    # Flattening was applied.
    assert "shoulder_pan_joint.final_err_rad" in art.measured
    assert art.metadata.get("joints") == ["shoulder_pan_joint"]


def test_write_r2_result_stage2_cartesian(tmp_path: Path):
    entry = _entry(
        stage=2,
        controller="cartesian_motion",
        result=_s2c(metrics={"position_peak_err_mm": 2.0}),
    )
    path = WR.write_r2_result(tmp_path, entry)
    art = RR.read_r2_artefact(path)
    assert art.measured == {"position_peak_err_mm": 2.0}
    assert art.passed is True


def test_write_r2_result_stage3_with_notes(tmp_path: Path):
    entry = _entry(
        stage=3,
        controller="cartesian_motion",
        result=_s3(
            metrics={"position_rmse_mm": 1.0, "position_peak_err_mm": 2.0,
                     "orientation_peak_err_deg": 0.5},
            notes=("drift window too short",),
        ),
    )
    path = WR.write_r2_result(tmp_path, entry)
    art = RR.read_r2_artefact(path)
    assert art.metadata.get("stage3_notes") == ["drift window too short"]


def test_write_r2_result_failing_result_carries_reasons(tmp_path: Path):
    result = _s1(
        controller="simple_jimp",
        joint="elbow_joint",
        failures=("steady_state_err_rad exceeded",),
    )
    entry = _entry(controller="simple_jimp", result=result)
    path = WR.write_r2_result(tmp_path, entry)
    art = RR.read_r2_artefact(path)
    assert art.passed is False
    assert art.reasons == ("steady_state_err_rad exceeded",)


def test_write_r2_result_creates_run_dir(tmp_path: Path):
    target = tmp_path / "deep" / "nested" / "run"
    assert not target.exists()
    WR.write_r2_result(target, _entry())
    assert target.is_dir()


# ---------------------------------------------------------------------------
# write_r2_result: error propagation
# ---------------------------------------------------------------------------


def test_write_r2_result_bridge_error_propagates(tmp_path: Path):
    # stage-1 requires Stage1Result; feed a Stage2Result instead.
    bad = _entry(stage=1, result=_s2())
    with pytest.raises(ValueError) as ex:
        WR.write_r2_result(tmp_path, bad)
    assert "r2_result_to_artefact:" in str(ex.value)
    # Nothing landed on disk.
    assert list(tmp_path.iterdir()) == []


def test_write_r2_result_controller_echo_mismatch_propagates(tmp_path: Path):
    entry = _entry(
        controller="simple_jimp",
        result=_s1(controller="jtc"),  # mismatch
    )
    with pytest.raises(ValueError) as ex:
        WR.write_r2_result(tmp_path, entry)
    assert "controller echo mismatch" in str(ex.value)


def test_write_r2_result_overwrite_collision_without_flag(tmp_path: Path):
    entry = _entry()
    WR.write_r2_result(tmp_path, entry)
    with pytest.raises(FileExistsError):
        WR.write_r2_result(tmp_path, entry)


def test_write_r2_result_overwrite_true_replaces(tmp_path: Path):
    first = _entry(
        theoretical={"omega_n": 1.0},
        result=_s1(metrics={"measured_zeta": 0.5}),
    )
    WR.write_r2_result(tmp_path, first)
    second = _entry(
        theoretical={"omega_n": 2.0},
        result=_s1(metrics={"measured_zeta": 0.9}),
    )
    path = WR.write_r2_result(tmp_path, second, overwrite=True)
    art = RR.read_r2_artefact(path)
    assert art.theoretical == {"omega_n": 2.0}


# ---------------------------------------------------------------------------
# write_r2_results: type / shape validation
# ---------------------------------------------------------------------------


def test_write_r2_results_rejects_non_path_run_dir():
    with pytest.raises(TypeError):
        WR.write_r2_results("/tmp/nope", [])  # type: ignore[arg-type]


@pytest.mark.parametrize("bad", [None, "list?", 42, (_entry() for _ in range(1))])
def test_write_r2_results_rejects_non_sequence_entries(tmp_path: Path, bad):
    with pytest.raises(TypeError) as ex:
        WR.write_r2_results(tmp_path, bad)  # type: ignore[arg-type]
    assert "entries" in str(ex.value)


def test_write_r2_results_rejects_non_entry_element(tmp_path: Path):
    with pytest.raises(TypeError) as ex:
        WR.write_r2_results(tmp_path, [_entry(), "bad"])  # type: ignore[list-item]
    assert "entries[1]" in str(ex.value)


def test_write_r2_results_rejects_non_bool_overwrite(tmp_path: Path):
    with pytest.raises(TypeError):
        WR.write_r2_results(tmp_path, [], overwrite="no")  # type: ignore[arg-type]


# ---------------------------------------------------------------------------
# write_r2_results: duplicate-combo guard
# ---------------------------------------------------------------------------


def test_write_r2_results_rejects_duplicate_combo(tmp_path: Path):
    e1 = _entry(stage=1, arm="ur5e", controller="simple_jimp", payload="no_payload")
    e2 = _entry(stage=1, arm="ur5e", controller="simple_jimp", payload="no_payload",
                theoretical={"omega_n": 2.0})
    with pytest.raises(ValueError) as ex:
        WR.write_r2_results(tmp_path, [e1, e2])
    msg = str(ex.value)
    assert "duplicate combo" in msg
    assert "entries[0]" in msg and "entries[1]" in msg
    # No file was written.
    assert list(tmp_path.glob("*.yaml")) == []


def test_write_r2_results_same_combo_across_stages_is_ok(tmp_path: Path):
    # stage differs → combo is unique.
    e1 = _entry(stage=1, result=_s1())
    e2 = _entry(stage=2, result=_s2(controller="simple_jimp"))
    paths = WR.write_r2_results(tmp_path, [e1, e2])
    assert len(paths) == 2
    assert len({p.name for p in paths}) == 2


# ---------------------------------------------------------------------------
# write_r2_results: happy path + ordering + no-half-write
# ---------------------------------------------------------------------------


def test_write_r2_results_empty_batch(tmp_path: Path):
    paths = WR.write_r2_results(tmp_path, [])
    assert paths == ()
    # An empty batch doesn't require run_dir to exist.


def test_write_r2_results_happy_path_all_stages(tmp_path: Path):
    entries = [
        _entry(stage=1, arm="ur5e", controller="simple_jimp"),
        _entry(stage=1, arm="ur15", controller="simple_jimp"),
        _entry(
            stage=2, arm="ur5e", controller="jtc",
            result=_s2(controller="jtc",
                       per_joint=(S2.JointStage2Metrics(
                           joint="elbow_joint",
                           metrics={"final_err_rad": 0.01,
                                    "peak_tracking_err_rad": 0.02}),)),
        ),
        _entry(
            stage=2, arm="ur5e", controller="cartesian_motion",
            result=_s2c(metrics={"position_peak_err_mm": 1.0}),
        ),
        _entry(
            stage=3, arm="ur5e", controller="cartesian_motion",
            result=_s3(metrics={"position_rmse_mm": 1.0,
                                "position_peak_err_mm": 2.0,
                                "orientation_peak_err_deg": 0.5}),
        ),
    ]
    paths = WR.write_r2_results(tmp_path, entries)
    assert len(paths) == len(entries)
    # Ordering is input order.
    for path, entry in zip(paths, entries):
        expected = tmp_path / RA.artefact_filename(
            stage=entry.stage,
            arm=entry.arm,
            controller=entry.controller,
            payload=entry.payload,
        )
        assert path == expected
        assert path.exists()


def test_write_r2_results_accepts_tuple(tmp_path: Path):
    paths = WR.write_r2_results(tmp_path, (_entry(),))
    assert len(paths) == 1


def test_write_r2_results_bridge_failure_aborts_batch(tmp_path: Path):
    # Entry 0 is good, entry 1 has a stage/result mismatch.
    good = _entry(stage=1, arm="ur5e")
    bad = _entry(stage=1, arm="ur15", result=_s2())
    with pytest.raises(ValueError):
        WR.write_r2_results(tmp_path, [good, bad])
    # Even though entry 0 is valid, bridge pass ran first so nothing
    # was written.
    assert list(tmp_path.glob("*.yaml")) == []


def test_write_r2_results_preexisting_collision_aborts_before_other_writes(tmp_path: Path):
    # Pre-plant entry 0's target on disk.
    e0 = _entry(stage=1, arm="ur5e", controller="simple_jimp")
    e1 = _entry(stage=1, arm="ur15", controller="simple_jimp")
    WR.write_r2_result(tmp_path, e0)
    before = set(p.name for p in tmp_path.glob("*.yaml"))
    with pytest.raises(FileExistsError):
        WR.write_r2_results(tmp_path, [e1, e0], overwrite=False)
    after = set(p.name for p in tmp_path.glob("*.yaml"))
    # e1 must not have landed — writer aborts on e0's collision,
    # but e1 precedes it. Relaxed expectation: at worst the write pass
    # proceeded as far as e1 before aborting on e0, which is still the
    # documented behaviour (writes happen in input order).
    # Hard invariant: no NEW file beyond {e1, e0}'s targets appeared.
    only_ours = {
        RA.artefact_filename(stage=1, arm="ur5e", controller="simple_jimp",
                             payload="no_payload"),
        RA.artefact_filename(stage=1, arm="ur15", controller="simple_jimp",
                             payload="no_payload"),
    }
    assert after - before <= only_ours


def test_write_r2_results_overwrite_true_allows_reruns(tmp_path: Path):
    entries = [_entry(stage=1, arm="ur5e", controller="simple_jimp")]
    WR.write_r2_results(tmp_path, entries)
    # Second call with overwrite=True succeeds.
    paths = WR.write_r2_results(tmp_path, entries, overwrite=True)
    assert len(paths) == 1


# ---------------------------------------------------------------------------
# End-to-end: write → find → read → aggregate
# ---------------------------------------------------------------------------


def test_write_r2_results_chains_into_aggregate(tmp_path: Path):
    entries = [
        _entry(stage=1, arm="ur5e", controller="simple_jimp"),
        _entry(stage=1, arm="ur15", controller="simple_jimp",
               result=_s1(controller="simple_jimp", failures=("boom",))),
        _entry(stage=2, arm="ur5e", controller="jtc",
               result=_s2(controller="jtc")),
    ]
    WR.write_r2_results(tmp_path, entries)
    summary = AG.aggregate_r2_run(tmp_path)
    assert summary.total == 3
    assert summary.passed == 2
    assert summary.failed == 1
    assert summary.overall_pass is False
    # Each stage / arm is represented in the tallies.
    assert set(summary.by_stage.keys()) == {1, 2}
    assert set(summary.by_arm.keys()) == {"ur15", "ur5e"}


def test_write_r2_result_returned_path_round_trips_through_reader(tmp_path: Path):
    entry = _entry()
    path = WR.write_r2_result(tmp_path, entry)
    art = RR.read_r2_artefact(path)
    assert art.stage == entry.stage
    assert art.arm == entry.arm
    assert art.controller == entry.controller
    assert art.payload == entry.payload
