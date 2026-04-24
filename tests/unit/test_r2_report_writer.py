"""Unit tests for ``tests/integration/r2_report_writer.py``.

Pinned behaviour: given an :class:`r2_aggregate.R2RunSummary`, emit
``report.csv`` (one row per artefact, deterministic order) and
``report.md`` (failures-first layout with per-axis tally tables).
"""

from __future__ import annotations

import csv as _csv
import importlib.util
import sys
from pathlib import Path

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
_agg = _load("r2_aggregate")
rw = _load("r2_report_writer")


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


def _aggregate(run_dir: Path):
    return _agg.aggregate_r2_run(run_dir)


# --- export surface ---------------------------------------------------------


def test_export_surface():
    assert rw.__all__ == (
        "CSV_HEADER",
        "REPORT_CSV_NAME",
        "REPORT_MD_NAME",
        "write_r2_report_csv",
        "write_r2_report_markdown",
        "write_r2_reports",
    )


def test_csv_header_pinned():
    assert rw.CSV_HEADER == (
        "stage",
        "arm",
        "controller",
        "payload",
        "passed",
        "reasons",
        "artefact_filename",
    )


def test_report_filenames_pinned():
    assert rw.REPORT_CSV_NAME == "report.csv"
    assert rw.REPORT_MD_NAME == "report.md"


# --- input validation -------------------------------------------------------


@pytest.mark.parametrize("bad", [None, "runs", 42, object()])
def test_csv_rejects_non_path(tmp_path, bad):
    _write(tmp_path)
    summary = _aggregate(tmp_path)
    with pytest.raises(TypeError, match=r"r2_report_writer: path must be pathlib.Path"):
        rw.write_r2_report_csv(bad, summary)


@pytest.mark.parametrize("bad", [None, "runs", 42, object()])
def test_md_rejects_non_path(tmp_path, bad):
    _write(tmp_path)
    summary = _aggregate(tmp_path)
    with pytest.raises(TypeError, match=r"r2_report_writer: path must be pathlib.Path"):
        rw.write_r2_report_markdown(bad, summary)


@pytest.mark.parametrize("bad", [None, "x", 1, {"artefacts": []}, object()])
def test_csv_rejects_non_summary(tmp_path, bad):
    with pytest.raises(
        TypeError, match=r"r2_report_writer: summary must be an r2_aggregate.R2RunSummary"
    ):
        rw.write_r2_report_csv(tmp_path / "report.csv", bad)


@pytest.mark.parametrize("bad", [None, "x", 1, {"artefacts": []}, object()])
def test_md_rejects_non_summary(tmp_path, bad):
    with pytest.raises(
        TypeError, match=r"r2_report_writer: summary must be an r2_aggregate.R2RunSummary"
    ):
        rw.write_r2_report_markdown(tmp_path / "report.md", bad)


def test_md_rejects_non_str_generated_at(tmp_path):
    _write(tmp_path)
    summary = _aggregate(tmp_path)
    with pytest.raises(TypeError, match=r"r2_report_writer: generated_at_utc must be str or None"):
        rw.write_r2_report_markdown(tmp_path / "report.md", summary, generated_at_utc=42)  # type: ignore[arg-type]


def test_write_reports_rejects_non_path(tmp_path):
    _write(tmp_path)
    summary = _aggregate(tmp_path)
    with pytest.raises(TypeError, match=r"r2_report_writer: report_dir must be pathlib.Path"):
        rw.write_r2_reports("reports", summary)  # type: ignore[arg-type]


def test_write_reports_rejects_non_summary(tmp_path):
    with pytest.raises(
        TypeError, match=r"r2_report_writer: summary must be an r2_aggregate.R2RunSummary"
    ):
        rw.write_r2_reports(tmp_path / "reports", object())


# --- CSV happy path ---------------------------------------------------------


def _read_csv(path: Path) -> list[list[str]]:
    with path.open("r", encoding="utf-8", newline="") as f:
        return list(_csv.reader(f))


def test_csv_empty_run_dir_writes_header_only(tmp_path):
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    summary = _aggregate(run_dir)
    out = tmp_path / "report.csv"
    rw.write_r2_report_csv(out, summary)
    rows = _read_csv(out)
    assert rows == [list(rw.CSV_HEADER)]


def test_csv_single_passing_row(tmp_path):
    run_dir = tmp_path / "run"
    _write(run_dir, passed=True)
    summary = _aggregate(run_dir)
    out = tmp_path / "report.csv"
    rw.write_r2_report_csv(out, summary)
    rows = _read_csv(out)
    assert rows[0] == list(rw.CSV_HEADER)
    assert len(rows) == 2
    assert rows[1] == [
        "1",
        "ur5e",
        "crisp_joint_impedance",
        "no_payload",
        "true",
        "",
        "r2_stage1_ur5e_crisp_joint_impedance_no_payload.yaml",
    ]


def test_csv_single_failing_row(tmp_path):
    run_dir = tmp_path / "run"
    _write(
        run_dir,
        stage=2,
        arm="ur15",
        controller="simple_jimp",
        payload="large_payload",
        passed=False,
        reasons=("rmse too high", "drift > 5 mm"),
    )
    summary = _aggregate(run_dir)
    out = tmp_path / "report.csv"
    rw.write_r2_report_csv(out, summary)
    rows = _read_csv(out)
    assert rows[1] == [
        "2",
        "ur15",
        "simple_jimp",
        "large_payload",
        "false",
        "rmse too high; drift > 5 mm",
        "r2_stage2_ur15_simple_jimp_large_payload.yaml",
    ]


def test_csv_row_order_matches_summary(tmp_path):
    run_dir = tmp_path / "run"
    # Write in scrambled order; summary sorts by filesystem path.
    _write(run_dir, stage=3, arm="ur5e", controller="cartesian_motion", payload="no_payload")
    _write(
        run_dir, stage=1, arm="ur15", controller="crisp_joint_impedance", payload="small_payload"
    )
    _write(run_dir, stage=2, arm="ur5e", controller="simple_jimp", payload="no_payload")
    summary = _aggregate(run_dir)
    out = tmp_path / "report.csv"
    rw.write_r2_report_csv(out, summary)
    rows = _read_csv(out)
    # Skip header.
    emitted_names = [r[6] for r in rows[1:]]
    expected_names = [
        _ra.artefact_filename(stage=a.stage, arm=a.arm, controller=a.controller, payload=a.payload)
        for a in summary.artefacts
    ]
    assert emitted_names == expected_names


def test_csv_deterministic_across_two_writes(tmp_path):
    run_dir = tmp_path / "run"
    _write(run_dir, stage=1, arm="ur5e", controller="crisp_joint_impedance", payload="no_payload")
    _write(
        run_dir,
        stage=2,
        arm="ur15",
        controller="simple_jimp",
        payload="small_payload",
        passed=False,
        reasons=("x",),
    )
    summary = _aggregate(run_dir)
    out1 = tmp_path / "a.csv"
    out2 = tmp_path / "b.csv"
    rw.write_r2_report_csv(out1, summary)
    rw.write_r2_report_csv(out2, summary)
    assert out1.read_bytes() == out2.read_bytes()


def test_csv_reasons_with_pipe_and_comma_escape_properly(tmp_path):
    run_dir = tmp_path / "run"
    _write(
        run_dir,
        passed=False,
        reasons=("contains, comma", "contains|pipe", "plain"),
    )
    summary = _aggregate(run_dir)
    out = tmp_path / "report.csv"
    rw.write_r2_report_csv(out, summary)
    # Round-trip through csv reader: the joined cell must come back intact.
    rows = _read_csv(out)
    assert rows[1][5] == "contains, comma; contains|pipe; plain"


def test_csv_passed_true_vs_false_tokens(tmp_path):
    run_dir = tmp_path / "run"
    _write(run_dir, stage=1, payload="no_payload", passed=True)
    _write(run_dir, stage=2, payload="small_payload", passed=False, reasons=("oops",))
    summary = _aggregate(run_dir)
    out = tmp_path / "report.csv"
    rw.write_r2_report_csv(out, summary)
    rows = _read_csv(out)
    tokens = sorted(r[4] for r in rows[1:])
    assert tokens == ["false", "true"]


def test_csv_creates_parent_directory(tmp_path):
    run_dir = tmp_path / "run"
    _write(run_dir)
    summary = _aggregate(run_dir)
    out = tmp_path / "reports" / "deep" / "report.csv"
    rw.write_r2_report_csv(out, summary)
    assert out.is_file()


def test_csv_overwrites_existing_file(tmp_path):
    run_dir = tmp_path / "run"
    _write(run_dir)
    summary = _aggregate(run_dir)
    out = tmp_path / "report.csv"
    out.write_text("stale content", encoding="utf-8")
    rw.write_r2_report_csv(out, summary)
    assert "stale content" not in out.read_text(encoding="utf-8")
    rows = _read_csv(out)
    assert rows[0] == list(rw.CSV_HEADER)


def test_csv_uses_lf_line_terminator(tmp_path):
    run_dir = tmp_path / "run"
    _write(run_dir)
    summary = _aggregate(run_dir)
    out = tmp_path / "report.csv"
    rw.write_r2_report_csv(out, summary)
    data = out.read_bytes()
    # No CRLF.
    assert b"\r\n" not in data
    assert data.count(b"\n") == 2  # header + 1 row


# --- Markdown happy path ----------------------------------------------------


def test_md_empty_run_dir_reports_fail(tmp_path):
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    summary = _aggregate(run_dir)
    out = tmp_path / "report.md"
    rw.write_r2_report_markdown(out, summary)
    text = out.read_text(encoding="utf-8")
    assert text.startswith("# R2 run report\n")
    assert f"Run directory: `{run_dir}`" in text
    assert "Overall: **FAIL**" in text
    assert "total=0, passed=0, failed=0" in text
    assert "## Failures" in text
    assert "_No failures._" in text
    assert "_No artefacts found in the run directory._" in text


def test_md_all_passed_overall_pass(tmp_path):
    run_dir = tmp_path / "run"
    _write(run_dir, stage=1, arm="ur5e", payload="no_payload")
    _write(run_dir, stage=1, arm="ur15", payload="no_payload")
    summary = _aggregate(run_dir)
    out = tmp_path / "report.md"
    rw.write_r2_report_markdown(out, summary)
    text = out.read_text(encoding="utf-8")
    assert "Overall: **PASS**" in text
    assert "total=2, passed=2, failed=0" in text
    assert "_No failures._" in text


def test_md_failure_shown_in_failures_section(tmp_path):
    run_dir = tmp_path / "run"
    _write(run_dir, stage=1, arm="ur5e", payload="no_payload", passed=True)
    _write(
        run_dir,
        stage=2,
        arm="ur15",
        controller="simple_jimp",
        payload="large_payload",
        passed=False,
        reasons=("damping off by 30%",),
    )
    summary = _aggregate(run_dir)
    out = tmp_path / "report.md"
    rw.write_r2_report_markdown(out, summary)
    text = out.read_text(encoding="utf-8")
    assert "Overall: **FAIL**" in text
    assert "total=2, passed=1, failed=1" in text
    # Failure row in the Failures section.
    failures_idx = text.index("## Failures")
    artefacts_idx = text.index("## Artefacts")
    failures_section = text[failures_idx:artefacts_idx]
    assert "simple_jimp" in failures_section
    assert "damping off by 30%" in failures_section
    # Failures should appear before the full artefact table in the document.
    assert failures_idx < artefacts_idx


def test_md_includes_all_four_tally_tables(tmp_path):
    run_dir = tmp_path / "run"
    _write(run_dir, stage=1, arm="ur5e", payload="no_payload")
    _write(run_dir, stage=2, arm="ur15", payload="small_payload", controller="simple_jimp")
    summary = _aggregate(run_dir)
    out = tmp_path / "report.md"
    rw.write_r2_report_markdown(out, summary)
    text = out.read_text(encoding="utf-8")
    assert "### By stage" in text
    assert "### By arm" in text
    assert "### By controller" in text
    assert "### By payload" in text
    # Each table has its own column header.
    assert "| stage | total | passed | failed |" in text
    assert "| arm | total | passed | failed |" in text
    assert "| controller | total | passed | failed |" in text
    assert "| payload | total | passed | failed |" in text


def test_md_empty_tally_tables_render_placeholder(tmp_path):
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    summary = _aggregate(run_dir)
    out = tmp_path / "report.md"
    rw.write_r2_report_markdown(out, summary)
    text = out.read_text(encoding="utf-8")
    # Each of the four tally tables is empty in this case.
    assert text.count("_No artefacts._") == 4


def test_md_tally_rows_have_correct_counts(tmp_path):
    run_dir = tmp_path / "run"
    _write(run_dir, stage=1, arm="ur5e", payload="no_payload", passed=True)
    _write(run_dir, stage=1, arm="ur15", payload="no_payload", passed=False, reasons=("x",))
    _write(run_dir, stage=2, arm="ur5e", payload="small_payload", passed=True)
    summary = _aggregate(run_dir)
    out = tmp_path / "report.md"
    rw.write_r2_report_markdown(out, summary)
    text = out.read_text(encoding="utf-8")
    # by_stage: stage 1 -> total=2, passed=1, failed=1; stage 2 -> 1/1/0
    assert "| 1 | 2 | 1 | 1 |" in text
    assert "| 2 | 1 | 1 | 0 |" in text
    # by_arm
    assert "| ur5e | 2 | 2 | 0 |" in text
    assert "| ur15 | 1 | 0 | 1 |" in text


def test_md_generated_at_included_when_provided(tmp_path):
    run_dir = tmp_path / "run"
    _write(run_dir)
    summary = _aggregate(run_dir)
    out = tmp_path / "report.md"
    rw.write_r2_report_markdown(out, summary, generated_at_utc="2026-04-24T23:00:00Z")
    text = out.read_text(encoding="utf-8")
    assert "Generated: `2026-04-24T23:00:00Z`" in text


def test_md_generated_at_omitted_by_default(tmp_path):
    run_dir = tmp_path / "run"
    _write(run_dir)
    summary = _aggregate(run_dir)
    out = tmp_path / "report.md"
    rw.write_r2_report_markdown(out, summary)
    text = out.read_text(encoding="utf-8")
    assert "Generated:" not in text


def test_md_pipe_in_reasons_is_escaped(tmp_path):
    run_dir = tmp_path / "run"
    _write(run_dir, passed=False, reasons=("a|b",))
    summary = _aggregate(run_dir)
    out = tmp_path / "report.md"
    rw.write_r2_report_markdown(out, summary)
    text = out.read_text(encoding="utf-8")
    assert "a\\|b" in text
    # Raw pipe should not appear inside a table row (other than as a
    # column separator). Count ``| a|b |`` — the unescaped form — to
    # confirm it does NOT appear.
    assert "| a|b |" not in text


def test_md_deterministic_across_two_writes(tmp_path):
    run_dir = tmp_path / "run"
    _write(run_dir, stage=1, arm="ur5e", payload="no_payload")
    _write(run_dir, stage=2, arm="ur15", payload="small_payload", passed=False, reasons=("y",))
    summary = _aggregate(run_dir)
    out1 = tmp_path / "a.md"
    out2 = tmp_path / "b.md"
    rw.write_r2_report_markdown(out1, summary)
    rw.write_r2_report_markdown(out2, summary)
    assert out1.read_bytes() == out2.read_bytes()


def test_md_trailing_newline(tmp_path):
    run_dir = tmp_path / "run"
    _write(run_dir)
    summary = _aggregate(run_dir)
    out = tmp_path / "report.md"
    rw.write_r2_report_markdown(out, summary)
    data = out.read_text(encoding="utf-8")
    assert data.endswith("\n")


def test_md_creates_parent_directory(tmp_path):
    run_dir = tmp_path / "run"
    _write(run_dir)
    summary = _aggregate(run_dir)
    out = tmp_path / "reports" / "deep" / "report.md"
    rw.write_r2_report_markdown(out, summary)
    assert out.is_file()


def test_md_overwrites_existing_file(tmp_path):
    run_dir = tmp_path / "run"
    _write(run_dir)
    summary = _aggregate(run_dir)
    out = tmp_path / "report.md"
    out.write_text("stale content", encoding="utf-8")
    rw.write_r2_report_markdown(out, summary)
    text = out.read_text(encoding="utf-8")
    assert "stale content" not in text
    assert text.startswith("# R2 run report\n")


def test_md_artefact_table_uses_check_and_cross(tmp_path):
    run_dir = tmp_path / "run"
    _write(run_dir, stage=1, payload="no_payload", passed=True)
    _write(run_dir, stage=2, payload="small_payload", passed=False, reasons=("x",))
    summary = _aggregate(run_dir)
    out = tmp_path / "report.md"
    rw.write_r2_report_markdown(out, summary)
    text = out.read_text(encoding="utf-8")
    artefact_section = text[text.index("## Artefacts") :]
    assert "✅" in artefact_section
    assert "❌" in artefact_section


# --- write_r2_reports convenience ------------------------------------------


def test_write_reports_emits_both_files(tmp_path):
    run_dir = tmp_path / "run"
    _write(run_dir)
    summary = _aggregate(run_dir)
    report_dir = tmp_path / "reports" / "ts"
    csv_path, md_path = rw.write_r2_reports(report_dir, summary)
    assert csv_path == report_dir / "report.csv"
    assert md_path == report_dir / "report.md"
    assert csv_path.is_file()
    assert md_path.is_file()


def test_write_reports_creates_missing_report_dir(tmp_path):
    run_dir = tmp_path / "run"
    _write(run_dir)
    summary = _aggregate(run_dir)
    report_dir = tmp_path / "new" / "nested" / "reports"
    assert not report_dir.exists()
    rw.write_r2_reports(report_dir, summary)
    assert report_dir.is_dir()


def test_write_reports_forwards_generated_at(tmp_path):
    run_dir = tmp_path / "run"
    _write(run_dir)
    summary = _aggregate(run_dir)
    report_dir = tmp_path / "reports"
    _, md_path = rw.write_r2_reports(report_dir, summary, generated_at_utc="2026-01-01T00:00:00Z")
    assert "Generated: `2026-01-01T00:00:00Z`" in md_path.read_text(encoding="utf-8")


def test_write_reports_round_trip_csv(tmp_path):
    run_dir = tmp_path / "run"
    _write(run_dir, stage=1, payload="no_payload", passed=True)
    _write(run_dir, stage=2, payload="small_payload", passed=False, reasons=("x",))
    summary = _aggregate(run_dir)
    report_dir = tmp_path / "reports"
    csv_path, _ = rw.write_r2_reports(report_dir, summary)
    rows = _read_csv(csv_path)
    assert rows[0] == list(rw.CSV_HEADER)
    assert len(rows) == 3  # header + 2


# --- end-to-end: full R2 matrix --------------------------------------------


def test_full_matrix_csv_row_count_matches_summary(tmp_path):
    """Every (stage, arm, payload) combo for one controller per stage → 18 rows."""
    run_dir = tmp_path / "run"
    controllers = {1: "crisp_joint_impedance", 2: "simple_jimp", 3: "cartesian_motion"}
    stages = (1, 2, 3)
    arms = ("ur5e", "ur15")
    payloads = ("no_payload", "small_payload", "large_payload")
    for s in stages:
        for a in arms:
            for p in payloads:
                _write(run_dir, stage=s, arm=a, controller=controllers[s], payload=p)
    summary = _aggregate(run_dir)
    assert summary.total == 18
    assert summary.overall_pass
    out = tmp_path / "report.csv"
    rw.write_r2_report_csv(out, summary)
    rows = _read_csv(out)
    assert len(rows) == 19  # header + 18


def test_full_matrix_md_shows_all_axes(tmp_path):
    run_dir = tmp_path / "run"
    controllers = {1: "crisp_joint_impedance", 2: "simple_jimp", 3: "cartesian_motion"}
    for s in (1, 2, 3):
        for a in ("ur5e", "ur15"):
            for p in ("no_payload", "small_payload", "large_payload"):
                _write(run_dir, stage=s, arm=a, controller=controllers[s], payload=p)
    summary = _aggregate(run_dir)
    out = tmp_path / "report.md"
    rw.write_r2_report_markdown(out, summary)
    text = out.read_text(encoding="utf-8")
    # All three stages, two arms, three controllers, three payloads in tally tables.
    for stage in ("| 1 |", "| 2 |", "| 3 |"):
        assert stage in text
    for arm in ("| ur5e |", "| ur15 |"):
        assert arm in text
    for ctrl in ("| crisp_joint_impedance |", "| simple_jimp |", "| cartesian_motion |"):
        assert ctrl in text
    for p in ("| no_payload |", "| small_payload |", "| large_payload |"):
        assert p in text
