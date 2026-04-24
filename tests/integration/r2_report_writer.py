"""R2 run-report writer: ``R2RunSummary`` -> ``report.csv`` + ``report.md``.

Pre-baked helper for the R2 integration test matrix (M6.12 / M6.13 /
M6.14, still gated on the M6.0 operator decision). Closes the
"future R2 CSV / Markdown report writer over ``R2RunSummary``
(analogous to M5's ``evaluation/compare.py::render_report``)" seam
that :mod:`r2_aggregate`'s docstring pins (module docstring, lines
~26-30):

    A future CSV / Markdown emitter (analogous to M5's ``report.csv`` /
    ``report.md``) consumes this summary; keeping aggregation and
    rendering as two separate seams mirrors the split between
    :mod:`r2_find_artefacts` (discovery) and :mod:`r2_read_artefact`
    (per-file validation) already in tree.

This module is that rendering seam. Given the
:class:`r2_aggregate.R2RunSummary` returned by
:func:`r2_aggregate.aggregate_r2_run`, it emits two sibling files:

    * ``report.csv`` — one header row plus one row per artefact, with
      primary-key columns (``stage``, ``arm``, ``controller``,
      ``payload``), the pass/fail verdict, the joined reason string,
      and the absolute artefact path. Columns are fixed and
      ordered; two calls on the same summary produce byte-identical
      output.
    * ``report.md`` — human-readable Markdown: a header with the
      run directory, a status summary (``total``, ``passed``,
      ``failed``, ``overall_pass``), per-axis tally tables
      (``by_stage`` / ``by_arm`` / ``by_controller`` / ``by_payload``),
      a "Failures first" section listing every failing artefact with
      its reasons, and a full artefact table at the bottom.

Contract
--------

* Pure functions. Each writer takes a target path and a summary, and
  writes one file. No side effects other than creating the parent
  directory if missing.
* Failures first in the Markdown. A human reading the report wants
  to know *what broke* before scrolling past a hundred passing rows,
  so the ``failed_artefacts`` tuple (pre-computed on the summary) is
  surfaced in its own section above the full-matrix table. The CSV
  does not reorder rows — it preserves the summary's deterministic
  ``artefacts`` order so a downstream consumer can join on row
  index.
* Deterministic output. The CSV uses the summary's fixed artefact
  order (sorted by filesystem path, inherited from
  :func:`r2_find_artefacts.find_r2_artefacts`). The Markdown sorts
  each tally table by stringified key (same rule as the summary's
  tally map iteration) and iterates artefacts in summary order.
* Missing parent directories are created (``parents=True``,
  ``exist_ok=True``) — matching the ergonomics of
  :func:`r2_run_artefact.write_r2_artefact`. Existing files *are*
  overwritten silently: a report is an output artefact, not a log.
* ``write_r2_reports`` is a convenience over the two primitives: it
  takes a ``report_dir``, creates it, and writes ``report.csv`` +
  ``report.md`` into it, returning both paths. This mirrors M5's
  ``evaluation/compare.py::run_report`` shape so a future live R2
  driver reads familiarly.

Non-goals
---------

* No aggregation. The writer accepts a pre-aggregated
  :class:`R2RunSummary`; computing that summary is
  :mod:`r2_aggregate`'s concern.
* No cross-run comparison. One summary in, one report out. Diffing
  two runs is a separate pre-bake helper if ever needed.
* No ROS / MuJoCo / numpy imports. Pure stdlib (the summary is
  already a pure-Python dataclass).
* No schema migration. The CSV header / Markdown layout is pinned
  by the unit tests; a future incompatible change cuts a v2 writer.

Alignment with the rest of the pre-bake chain
---------------------------------------------

* Pure stdlib. No PyYAML / numpy / ROS imports.
* Sibling modules loaded via file-path ``importlib`` so the
  :class:`R2Artefact` / :class:`R2RunSummary` class identities stay
  stable across the chain (matches the :mod:`r2_aggregate` and
  :mod:`r2_read_artefact` convention; see the "module loading" repo
  memory).
* Error-message prefix: ``r2_report_writer:`` for this module's own
  invariants.
"""

from __future__ import annotations

import csv as _csv
import importlib.util
import io
import sys
from pathlib import Path
from typing import Iterable, Mapping, Optional, Tuple

_THIS_DIR = Path(__file__).resolve().parent


def _load_sibling(module_name: str):
    """Import a sibling ``tests/integration/`` module by file path.

    Mirrors the convention used by :mod:`r2_aggregate` and
    :mod:`r2_read_artefact` so the :class:`R2Artefact` /
    :class:`R2RunSummary` dataclass identities are shared across the
    chain.
    """
    if module_name in sys.modules:
        return sys.modules[module_name]
    path = _THIS_DIR / f"{module_name}.py"
    spec = importlib.util.spec_from_file_location(module_name, path)
    assert spec is not None and spec.loader is not None
    mod = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = mod
    spec.loader.exec_module(mod)
    return mod


_ra = _load_sibling("r2_run_artefact")
_agg = _load_sibling("r2_aggregate")


__all__ = (
    "CSV_HEADER",
    "REPORT_CSV_NAME",
    "REPORT_MD_NAME",
    "write_r2_report_csv",
    "write_r2_report_markdown",
    "write_r2_reports",
)


# ---------------------------------------------------------------------------
# Output layout constants
# ---------------------------------------------------------------------------


CSV_HEADER: Tuple[str, ...] = (
    "stage",
    "arm",
    "controller",
    "payload",
    "passed",
    "reasons",
    "artefact_filename",
)

REPORT_CSV_NAME: str = "report.csv"
REPORT_MD_NAME: str = "report.md"


# ---------------------------------------------------------------------------
# Validation helpers
# ---------------------------------------------------------------------------


def _raise_type(msg: str) -> None:
    raise TypeError(f"r2_report_writer: {msg}")


def _check_path(label: str, value: object) -> Path:
    if not isinstance(value, Path):
        _raise_type(f"{label} must be pathlib.Path, got {type(value).__name__}")
    return value  # type: ignore[return-value]


def _check_summary(summary: object) -> "_agg.R2RunSummary":
    if not isinstance(summary, _agg.R2RunSummary):
        _raise_type(f"summary must be an r2_aggregate.R2RunSummary, got {type(summary).__name__}")
    return summary  # type: ignore[return-value]


def _join_reasons(reasons: Iterable[str]) -> str:
    # ``reasons`` is already a ``tuple[str, ...]`` on the artefact
    # (pinned by the reader). Keep the join consistent with M5's
    # ``"; "`` separator for incompatibility reasons so a human reading
    # both styles of report sees the same shape.
    return "; ".join(reasons)


# ---------------------------------------------------------------------------
# CSV
# ---------------------------------------------------------------------------


def _artefact_row(artefact) -> list[str]:
    filename = _ra.artefact_filename(
        stage=artefact.stage,
        arm=artefact.arm,
        controller=artefact.controller,
        payload=artefact.payload,
    )
    return [
        str(artefact.stage),
        artefact.arm,
        artefact.controller,
        artefact.payload,
        "true" if artefact.passed else "false",
        _join_reasons(artefact.reasons),
        filename,
    ]


def write_r2_report_csv(path: Path, summary: "_agg.R2RunSummary") -> None:
    """Write a CSV report of ``summary`` to ``path``.

    Parameters
    ----------
    path
        Destination file. Parent directory is created with
        ``parents=True, exist_ok=True`` if missing. Existing files are
        overwritten.
    summary
        :class:`r2_aggregate.R2RunSummary` (typically from
        :func:`r2_aggregate.aggregate_r2_run`). Row order follows
        ``summary.artefacts`` (deterministic, sorted by filesystem
        path).

    Raises
    ------
    TypeError
        If ``path`` is not a :class:`pathlib.Path` or ``summary`` is
        not an :class:`R2RunSummary`, with ``r2_report_writer:``
        prefix.
    """
    path = _check_path("path", path)
    summary = _check_summary(summary)

    path.parent.mkdir(parents=True, exist_ok=True)

    # Write via an in-memory buffer first so a failure mid-iteration
    # doesn't leave a half-written file. CSV encoding is cheap; the
    # deterministic-bytes contract is worth the extra allocation.
    buf = io.StringIO(newline="")
    writer = _csv.writer(buf, lineterminator="\n")
    writer.writerow(list(CSV_HEADER))
    for artefact in summary.artefacts:
        writer.writerow(_artefact_row(artefact))

    path.write_text(buf.getvalue(), encoding="utf-8")


# ---------------------------------------------------------------------------
# Markdown
# ---------------------------------------------------------------------------


def _md_cell(s: str) -> str:
    # Escape `|` so a reason containing a pipe doesn't break the table
    # layout; newlines are flattened to spaces (reasons are already
    # single-line per the reader's contract, but be defensive).
    return (s or "").replace("|", "\\|").replace("\n", " ")


def _tally_table(
    heading: str,
    column: str,
    tallies: Mapping,
) -> list[str]:
    lines: list[str] = [f"### {heading}", ""]
    if not tallies:
        lines.append("_No artefacts._")
        lines.append("")
        return lines
    lines.append(f"| {column} | total | passed | failed |")
    lines.append("|---|---|---|---|")
    for key, tally in tallies.items():
        lines.append(f"| {_md_cell(str(key))} | {tally.total} | {tally.passed} | {tally.failed} |")
    lines.append("")
    return lines


def _artefact_table(summary: "_agg.R2RunSummary") -> list[str]:
    lines: list[str] = []
    if not summary.artefacts:
        lines.append("_No artefacts found in the run directory._")
        lines.append("")
        return lines
    lines.append("| stage | arm | controller | payload | passed | reasons |")
    lines.append("|---|---|---|---|---|---|")
    for artefact in summary.artefacts:
        lines.append(
            "| {stage} | {arm} | {controller} | {payload} | {passed} | {reasons} |".format(
                stage=artefact.stage,
                arm=_md_cell(artefact.arm),
                controller=_md_cell(artefact.controller),
                payload=_md_cell(artefact.payload),
                passed="✅" if artefact.passed else "❌",
                reasons=_md_cell(_join_reasons(artefact.reasons)),
            )
        )
    lines.append("")
    return lines


def _failures_section(summary: "_agg.R2RunSummary") -> list[str]:
    lines: list[str] = ["## Failures", ""]
    if not summary.failed_artefacts:
        lines.append("_No failures._")
        lines.append("")
        return lines
    lines.append("| stage | arm | controller | payload | reasons |")
    lines.append("|---|---|---|---|---|")
    for artefact in summary.failed_artefacts:
        lines.append(
            "| {stage} | {arm} | {controller} | {payload} | {reasons} |".format(
                stage=artefact.stage,
                arm=_md_cell(artefact.arm),
                controller=_md_cell(artefact.controller),
                payload=_md_cell(artefact.payload),
                reasons=_md_cell(_join_reasons(artefact.reasons)),
            )
        )
    lines.append("")
    return lines


def write_r2_report_markdown(
    path: Path,
    summary: "_agg.R2RunSummary",
    *,
    generated_at_utc: Optional[str] = None,
) -> None:
    """Write a Markdown report of ``summary`` to ``path``.

    Layout (pinned by the unit tests):

    1. Top-level heading.
    2. Optional ``Generated: <UTC>`` line when ``generated_at_utc``
       is provided.
    3. ``Run directory: <path>`` line.
    4. ``Overall: PASS|FAIL`` one-liner with ``total / passed /
       failed`` counts.
    5. ``## Status summary`` with four per-axis tally tables
       (``by_stage``, ``by_arm``, ``by_controller``, ``by_payload``).
    6. ``## Failures`` section (failures-first — empty section if
       ``overall_pass`` is ``True``).
    7. ``## Artefacts`` full table in summary order.

    Parameters
    ----------
    path
        Destination file. Parent directory is created if missing.
    summary
        :class:`r2_aggregate.R2RunSummary`.
    generated_at_utc
        Optional pre-formatted UTC timestamp string (e.g.
        ``"2026-04-24T23:00:00Z"``). Passed in (rather than computed
        here) so the writer stays deterministic under test; the
        top-level driver decides the timestamp format.

    Raises
    ------
    TypeError
        If ``path`` is not a :class:`pathlib.Path` or ``summary`` is
        not an :class:`R2RunSummary`.
    """
    path = _check_path("path", path)
    summary = _check_summary(summary)
    if generated_at_utc is not None and not isinstance(generated_at_utc, str):
        _raise_type(f"generated_at_utc must be str or None, got {type(generated_at_utc).__name__}")

    path.parent.mkdir(parents=True, exist_ok=True)

    lines: list[str] = ["# R2 run report", ""]
    if generated_at_utc is not None:
        lines.append(f"Generated: `{generated_at_utc}`")
        lines.append("")
    lines.append(f"Run directory: `{summary.run_dir}`")
    lines.append("")
    overall = "PASS" if summary.overall_pass else "FAIL"
    lines.append(
        f"Overall: **{overall}** — total={summary.total}, "
        f"passed={summary.passed}, failed={summary.failed}."
    )
    lines.append("")
    lines.append("## Status summary")
    lines.append("")
    lines.extend(_tally_table("By stage", "stage", summary.by_stage))
    lines.extend(_tally_table("By arm", "arm", summary.by_arm))
    lines.extend(_tally_table("By controller", "controller", summary.by_controller))
    lines.extend(_tally_table("By payload", "payload", summary.by_payload))
    lines.extend(_failures_section(summary))
    lines.append("## Artefacts")
    lines.append("")
    lines.extend(_artefact_table(summary))

    # Ensure a trailing newline so the file is POSIX-clean.
    text = "\n".join(lines)
    if not text.endswith("\n"):
        text += "\n"
    path.write_text(text, encoding="utf-8")


# ---------------------------------------------------------------------------
# Convenience top-level driver
# ---------------------------------------------------------------------------


def write_r2_reports(
    report_dir: Path,
    summary: "_agg.R2RunSummary",
    *,
    generated_at_utc: Optional[str] = None,
) -> Tuple[Path, Path]:
    """Write both ``report.csv`` and ``report.md`` into ``report_dir``.

    Parameters
    ----------
    report_dir
        Destination directory. Created with ``parents=True,
        exist_ok=True`` if missing.
    summary
        :class:`r2_aggregate.R2RunSummary`.
    generated_at_utc
        Forwarded to :func:`write_r2_report_markdown`; ignored by the
        CSV writer (which is already deterministic by construction).

    Returns
    -------
    (csv_path, md_path)
        The two written paths, in the same order as
        :data:`REPORT_CSV_NAME` / :data:`REPORT_MD_NAME`.

    Raises
    ------
    TypeError
        If ``report_dir`` is not a :class:`pathlib.Path` or ``summary``
        is not an :class:`R2RunSummary`.
    """
    report_dir = _check_path("report_dir", report_dir)
    summary = _check_summary(summary)

    report_dir.mkdir(parents=True, exist_ok=True)
    csv_path = report_dir / REPORT_CSV_NAME
    md_path = report_dir / REPORT_MD_NAME
    write_r2_report_csv(csv_path, summary)
    write_r2_report_markdown(md_path, summary, generated_at_utc=generated_at_utc)
    return csv_path, md_path
