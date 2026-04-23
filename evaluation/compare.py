#!/usr/bin/env python3
"""M5 bullet 4 — comparison report driver.

Wraps M5 bullets 2 + 3 (``evaluation/run_evaluation.py`` and
``evaluation/compute_metrics.py``) into a single driver that:

1. Enumerates every ``{scenario, controller, robot}`` combination in the
   requested matrix.
2. Filters out combinations that fail ``check_compatibility`` or whose
   ``robot`` is not listed under the scenario's ``robots:`` field.
3. For each remaining combination, locates the newest valid run directory
   under ``runs-root`` (matching ``{scenario}__{controller}__{robot}__*``
   and containing a ``manifest.yaml``). Missing run dirs are recorded as
   ``no_run``.
4. For each located run directory, invokes
   ``compute_metrics.compute_metrics_for_run`` (caching
   ``metrics.yaml`` + ``metrics.csv`` next to the run). Cartesian
   scenarios are surfaced as ``not_yet_evaluated`` (ADR-0010: cartesian
   metrics deferred in v1).
5. Emits ``report.csv`` + ``report.md`` under ``report-dir`` with one row
   per combination and one column per metric + overall status.

Default ``report-dir`` is ``evaluation/reports/<UTC-timestamp>/``.

Exit codes:
    0 — every compatible combination either passed, was explicitly
        incompatible, or is cartesian (``not_yet_evaluated``).
    1 — at least one compatible combination failed
        (``overall_status=fail``, ``no_run``, or ``metrics_error``).
    2 — driver misuse (bad CLI, missing scenarios, etc.).

Live dispatch (spinning sim + controller per combination) is *not*
implemented in this iteration; ``--aggregate-only`` is the default and
only mode. The test gate seeds synthetic run directories and exercises
the aggregation + reporting path end-to-end.
"""

from __future__ import annotations

import argparse
import importlib.util
import sys
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable

import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
EVAL_DIR = REPO_ROOT / "evaluation"
SCENARIO_DIR = EVAL_DIR / "scenarios"
RUNS_DIR_DEFAULT = EVAL_DIR / "runs"
REPORTS_DIR_DEFAULT = EVAL_DIR / "reports"


# ---------------------------------------------------------------------------
# Sibling module loading (match run_evaluation.py's file-based pattern so
# evaluation/ does not need to be a python package).
# ---------------------------------------------------------------------------


def _load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec and spec.loader
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


_RUN = _load_module("_evaluation_run", EVAL_DIR / "run_evaluation.py")
_METRICS = _load_module("_evaluation_compute_metrics", EVAL_DIR / "compute_metrics.py")
_VALIDATE = _load_module("_evaluation_scenario_validate_compare", SCENARIO_DIR / "validate.py")


# ---------------------------------------------------------------------------
# Combo enumeration
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Combo:
    scenario_path: Path
    scenario_name: str
    scenario_type: str
    target_space: str
    controller: str
    robot: str


@dataclass
class RowResult:
    combo: Combo
    status: str  # pass | fail | no_run | metrics_error | not_yet_evaluated
    run_dir: Path | None = None
    metrics: dict = field(default_factory=dict)  # metric_name -> {value, status}
    reason: str = ""


@dataclass
class IncompatibleCombo:
    scenario_name: str
    scenario_path: Path
    controller: str
    robot: str
    reasons: list[str]


def load_scenario(path: Path) -> dict:
    errs = _VALIDATE.validate_file(path)
    if errs:
        raise ValueError(f"{path}: invalid scenario\n  - " + "\n  - ".join(errs))
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def default_scenarios() -> list[Path]:
    """All YAMLs directly under evaluation/scenarios/ (excluding validate.py)."""
    return sorted(p for p in SCENARIO_DIR.glob("*.yaml"))


def enumerate_combos(
    scenarios: Iterable[Path],
    controllers: Iterable[str],
    robots: Iterable[str],
) -> tuple[list[Combo], list[IncompatibleCombo]]:
    """Expand the matrix and split into compatible/incompatible.

    A combo is *incompatible* if either:

    * the robot is not listed under ``scenario.robots``, or
    * ``run_evaluation.check_compatibility(spec, scenario)`` returns errors.

    Incompatible combos are still surfaced so the report can show them
    with a clear reason.
    """
    compatible: list[Combo] = []
    incompatible: list[IncompatibleCombo] = []
    controllers = list(controllers)
    robots = list(robots)
    for scen_path in scenarios:
        scenario = load_scenario(scen_path)
        scen_robots = list(scenario.get("robots") or [])
        target_space = scenario.get("target", {}).get("space", "joint")
        for controller in controllers:
            spec = _RUN.get_controller(controller)
            compat_errs = _RUN.check_compatibility(spec, scenario)
            for robot in robots:
                reasons: list[str] = []
                if robot not in scen_robots:
                    reasons.append(
                        f"robot {robot!r} not listed in scenario.robots " f"({scen_robots})"
                    )
                reasons.extend(compat_errs)
                if reasons:
                    incompatible.append(
                        IncompatibleCombo(
                            scenario_name=scenario["name"],
                            scenario_path=scen_path,
                            controller=controller,
                            robot=robot,
                            reasons=list(reasons),
                        )
                    )
                    continue
                compatible.append(
                    Combo(
                        scenario_path=scen_path,
                        scenario_name=scenario["name"],
                        scenario_type=scenario["scenario_type"],
                        target_space=target_space,
                        controller=controller,
                        robot=robot,
                    )
                )
    return compatible, incompatible


# ---------------------------------------------------------------------------
# Run dir lookup
# ---------------------------------------------------------------------------


def find_latest_run_dir(
    runs_root: Path, scenario_name: str, controller: str, robot: str
) -> Path | None:
    """Return the newest valid run dir for a combo, or None.

    Directory layout is ``{scenario}__{controller}__{robot}__<UTC-ts>``
    (UTC timestamp ``YYYYMMDDTHHMMSSZ`` — lexicographic sort is safe).
    A run dir is considered *valid* iff it contains a ``manifest.yaml``
    (the runner writes this last, so its presence indicates a successful
    run). Partial/failed run dirs without a manifest are ignored so they
    can't mask an older successful run.
    """
    if not runs_root.is_dir():
        return None
    prefix = f"{scenario_name}__{controller}__{robot}__"
    candidates = [p for p in runs_root.iterdir() if p.is_dir() and p.name.startswith(prefix)]
    candidates.sort(key=lambda p: p.name, reverse=True)
    for c in candidates:
        if (c / "manifest.yaml").exists():
            return c
    return None


# ---------------------------------------------------------------------------
# Metrics per combo
# ---------------------------------------------------------------------------


_REPORT_METRICS = ("rmse", "settling_time", "overshoot", "control_effort")


def _row_from_payload(combo: Combo, run_dir: Path, payload: dict) -> RowResult:
    target_space = payload.get("target_space") or combo.target_space
    if target_space == "cartesian":
        status = "not_yet_evaluated"
        reason = "cartesian metrics deferred in v1 (ADR-0010)"
    else:
        status = payload.get("overall_status", "pass")
        reason = ""
    metrics: dict[str, dict] = {}
    for name in _REPORT_METRICS:
        m = (payload.get("metrics") or {}).get(name)
        if not isinstance(m, dict):
            metrics[name] = {"value": None, "status": "absent", "unit": ""}
            continue
        metrics[name] = {
            "value": m.get("aggregate"),
            "status": m.get("status", ""),
            "unit": m.get("unit", ""),
            "reason": m.get("reason", ""),
        }
    return RowResult(
        combo=combo,
        status=status,
        run_dir=run_dir,
        metrics=metrics,
        reason=reason,
    )


def process_combo(combo: Combo, runs_root: Path) -> RowResult:
    """Find the combo's run dir, compute metrics (caching artefacts), and
    return a :class:`RowResult` summarising it."""
    run_dir = find_latest_run_dir(runs_root, combo.scenario_name, combo.controller, combo.robot)
    if run_dir is None:
        # Cartesian combos without a run are still surfaced as
        # "not_yet_evaluated" so the report reads consistently — they
        # would be deferred anyway.
        if combo.target_space == "cartesian":
            return RowResult(
                combo=combo,
                status="not_yet_evaluated",
                reason="no run dir found; cartesian metrics deferred in v1 (ADR-0010)",
            )
        return RowResult(
            combo=combo,
            status="no_run",
            reason=f"no valid run dir under {runs_root} matching prefix",
        )
    try:
        payload = _METRICS.compute_metrics_for_run(run_dir)
    except (FileNotFoundError, ValueError) as e:
        return RowResult(
            combo=combo,
            status="metrics_error",
            run_dir=run_dir,
            reason=str(e),
        )
    # Cache artefacts next to the run so the solo compute_metrics CLI
    # and this driver stay consistent.
    try:
        _METRICS.write_metrics_yaml(run_dir / "metrics.yaml", payload)
        _METRICS.write_metrics_csv(run_dir / "metrics.csv", payload)
    except OSError:  # best-effort cache; report still usable
        pass
    return _row_from_payload(combo, run_dir, payload)


# ---------------------------------------------------------------------------
# Report emission
# ---------------------------------------------------------------------------


_REPORT_CSV_HEADER = [
    "scenario",
    "scenario_type",
    "target_space",
    "controller",
    "robot",
    "overall_status",
    "rmse_rad",
    "settling_time_s",
    "overshoot_pct",
    "control_effort_nm",
    "run_dir",
    "reason",
]


def _fmt(v) -> str:
    if v is None:
        return ""
    if isinstance(v, float):
        if v != v:  # NaN
            return "nan"
        if v == float("inf"):
            return "inf"
        if v == float("-inf"):
            return "-inf"
        return f"{v:.6g}"
    return str(v)


def _metric_cell(row: RowResult, name: str) -> str:
    m = row.metrics.get(name)
    if not m:
        return ""
    status = m.get("status", "")
    if status == "ok":
        return _fmt(m.get("value"))
    if status in ("skipped", "not_applicable"):
        return status
    return ""


def _incompatible_row(inc: IncompatibleCombo) -> list[str]:
    # These rows have no metrics (compat gate rejected them up-front).
    return [
        inc.scenario_name,
        "",
        "",
        inc.controller,
        inc.robot,
        "incompatible",
        "",
        "",
        "",
        "",
        "",
        "; ".join(inc.reasons),
    ]


def _compatible_row(row: RowResult) -> list[str]:
    return [
        row.combo.scenario_name,
        row.combo.scenario_type,
        row.combo.target_space,
        row.combo.controller,
        row.combo.robot,
        row.status,
        _metric_cell(row, "rmse"),
        _metric_cell(row, "settling_time"),
        _metric_cell(row, "overshoot"),
        _metric_cell(row, "control_effort"),
        (
            str(row.run_dir.relative_to(REPO_ROOT))
            if row.run_dir and _is_relative_to(row.run_dir, REPO_ROOT)
            else (str(row.run_dir) if row.run_dir else "")
        ),
        row.reason,
    ]


def write_report_csv(
    path: Path, rows: list[RowResult], incompatibles: list[IncompatibleCombo]
) -> None:
    import csv as _csv

    lines: list[list[str]] = [list(_REPORT_CSV_HEADER)]
    for r in rows:
        lines.append(_compatible_row(r))
    for inc in incompatibles:
        lines.append(_incompatible_row(inc))
    with path.open("w", encoding="utf-8", newline="") as f:
        w = _csv.writer(f)
        for line in lines:
            w.writerow(line)


def write_report_markdown(
    path: Path,
    rows: list[RowResult],
    incompatibles: list[IncompatibleCombo],
    *,
    generated_at_utc: str,
    runs_root: Path,
) -> None:
    counts = _count_statuses(rows)
    out: list[str] = []
    out.append("# Evaluation comparison report")
    out.append("")
    out.append(f"Generated: `{generated_at_utc}`")
    out.append("")
    out.append(
        f"Runs root: `{_maybe_rel(runs_root)}` — "
        f"{len(rows)} compatible combo(s), "
        f"{len(incompatibles)} incompatible combo(s)."
    )
    out.append("")
    summary_bits = ", ".join(f"{k}={v}" for k, v in counts.items() if v)
    out.append(f"Status summary: {summary_bits or 'empty'}.")
    out.append("")
    out.append(
        "| scenario | type | space | controller | robot | status | rmse (rad) | settle (s) | overshoot (%) | effort (N·m) | run_dir |"
    )
    out.append("|---|---|---|---|---|---|---|---|---|---|---|")
    for r in rows:
        cells = _compatible_row(r)
        out.append(
            "| " + " | ".join(_md_cell(cells[i]) for i in (0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10)) + " |"
        )
    if incompatibles:
        out.append("")
        out.append("## Incompatible combos (filtered up-front)")
        out.append("")
        out.append("| scenario | controller | robot | reasons |")
        out.append("|---|---|---|---|")
        for inc in incompatibles:
            out.append(
                f"| {_md_cell(inc.scenario_name)} | {_md_cell(inc.controller)} "
                f"| {_md_cell(inc.robot)} | {_md_cell('; '.join(inc.reasons))} |"
            )
    out.append("")
    path.write_text("\n".join(out), encoding="utf-8")


def _md_cell(s: str) -> str:
    return (s or "").replace("|", "\\|").replace("\n", " ")


def _count_statuses(rows: list[RowResult]) -> dict[str, int]:
    counts: dict[str, int] = {
        "pass": 0,
        "fail": 0,
        "no_run": 0,
        "metrics_error": 0,
        "not_yet_evaluated": 0,
    }
    for r in rows:
        counts[r.status] = counts.get(r.status, 0) + 1
    return counts


def _is_relative_to(child: Path, parent: Path) -> bool:
    try:
        child.resolve().relative_to(parent.resolve())
        return True
    except ValueError:
        return False


def _maybe_rel(p: Path) -> str:
    return str(p.relative_to(REPO_ROOT)) if _is_relative_to(p, REPO_ROOT) else str(p)


# ---------------------------------------------------------------------------
# Top-level driver
# ---------------------------------------------------------------------------


def run_report(
    *,
    scenarios: list[Path],
    controllers: list[str],
    robots: list[str],
    runs_root: Path,
    report_dir: Path,
) -> tuple[list[RowResult], list[IncompatibleCombo]]:
    """Aggregate-only entry point. Returns (rows, incompatibles). Writes
    ``report.csv`` and ``report.md`` under ``report_dir`` (created if
    missing)."""
    combos, incompatibles = enumerate_combos(scenarios, controllers, robots)
    rows = [process_combo(c, runs_root) for c in combos]

    report_dir.mkdir(parents=True, exist_ok=True)
    generated_at_utc = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    write_report_csv(report_dir / "report.csv", rows, incompatibles)
    write_report_markdown(
        report_dir / "report.md",
        rows,
        incompatibles,
        generated_at_utc=generated_at_utc,
        runs_root=runs_root,
    )
    return rows, incompatibles


def overall_exit_code(rows: list[RowResult]) -> int:
    """Return 0 if every compatible combo is pass/not_yet_evaluated, else 1."""
    for r in rows:
        if r.status in ("fail", "no_run", "metrics_error"):
            return 1
    return 0


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Aggregate evaluation run directories into a comparison " "report (M5 bullet 4)."
        )
    )
    parser.add_argument(
        "--scenarios",
        nargs="+",
        type=Path,
        default=None,
        help=("Scenario YAML files to include (default: every " "evaluation/scenarios/*.yaml)."),
    )
    parser.add_argument(
        "--controllers",
        nargs="+",
        default=None,
        help=(
            "Controller names to include (default: all known: "
            + ", ".join(_RUN.known_controllers())
            + ")."
        ),
    )
    parser.add_argument(
        "--robots",
        nargs="+",
        default=["ur5e", "ur15"],
        choices=["ur5e", "ur15"],
    )
    parser.add_argument(
        "--runs-root",
        type=Path,
        default=RUNS_DIR_DEFAULT,
        help=f"Directory of run dirs (default: {RUNS_DIR_DEFAULT}).",
    )
    parser.add_argument(
        "--report-dir",
        type=Path,
        default=None,
        help=(
            "Destination dir for report.csv + report.md "
            f"(default: {REPORTS_DIR_DEFAULT}/<UTC-timestamp>/)."
        ),
    )
    parser.add_argument(
        "--aggregate-only",
        action="store_true",
        default=True,
        help=(
            "Only aggregate existing run dirs (default; live dispatch " "is not implemented yet)."
        ),
    )
    args = parser.parse_args(argv)

    scenarios = list(args.scenarios) if args.scenarios else default_scenarios()
    if not scenarios:
        print("compare: no scenarios found", file=sys.stderr)
        return 2
    for p in scenarios:
        if not p.is_file():
            print(f"compare: scenario not found: {p}", file=sys.stderr)
            return 2

    controllers = list(args.controllers) if args.controllers else _RUN.known_controllers()
    unknown = [c for c in controllers if c not in _RUN.known_controllers()]
    if unknown:
        print(
            f"compare: unknown controller(s): {unknown}; known: " f"{_RUN.known_controllers()}",
            file=sys.stderr,
        )
        return 2

    if args.report_dir is None:
        ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        report_dir = REPORTS_DIR_DEFAULT / ts
    else:
        report_dir = args.report_dir

    try:
        rows, incompatibles = run_report(
            scenarios=scenarios,
            controllers=controllers,
            robots=list(args.robots),
            runs_root=args.runs_root,
            report_dir=report_dir,
        )
    except ValueError as e:
        print(f"compare: {e}", file=sys.stderr)
        return 2

    counts = _count_statuses(rows)
    print(
        f"compare: wrote {report_dir}/report.{{csv,md}} — "
        f"{len(rows)} compatible combos ("
        + ", ".join(f"{k}={v}" for k, v in counts.items() if v)
        + f"), {len(incompatibles)} incompatible."
    )
    return overall_exit_code(rows)


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
