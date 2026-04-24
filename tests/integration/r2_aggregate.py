"""R2 run-level aggregator: run_dir of artefacts -> summary verdict.

Pre-baked helper for the R2 integration test matrix (M6.12 / M6.13 /
M6.14, still gated on the M6.0 operator decision). Closes the
"future M5-like R2 aggregation / reporting driver (analogous to
``evaluation/compare.py`` over R2 artefacts)" seam that
:mod:`r2_read_artefact`'s docstring pins (lines ~13-18):

    The future M5-like R2 aggregation driver (analogous to
    ``evaluation/compare.py`` over R2 artefacts) uses this module to
    turn a file path produced by
    :func:`r2_find_artefacts.find_r2_artefacts` into a validated
    :class:`R2Artefact` in one call, without duplicating schema
    knowledge across consumers.

This module is that aggregation layer — one call up from the reader:

    summary = aggregate_r2_run(run_dir)
    if not summary.overall_pass:
        ...

It chains :func:`r2_find_artefacts.find_r2_artefacts` ->
:func:`r2_read_artefact.read_r2_artefact` across a run directory and
returns a :class:`R2RunSummary` with deterministic-ordered artefacts
plus per-axis :class:`R2Tally` breakdowns by stage, arm, controller,
and payload. A future CSV / Markdown emitter (analogous to M5's
``report.csv`` / ``report.md``) consumes this summary; keeping
aggregation and rendering as two separate seams mirrors the split
between :mod:`r2_find_artefacts` (discovery) and
:mod:`r2_read_artefact` (per-file validation) already in tree.

Contract
--------

* One call: ``aggregate_r2_run(run_dir) -> R2RunSummary``. No
  filters — the caller filters upstream by narrowing ``run_dir``
  (each R2 run dir from :func:`r2_run_dir.make_r2_run_dir` already
  isolates one sweep). If per-axis filtering is genuinely needed
  later, add a sibling that wraps this one rather than bolting
  kwargs onto this signature.
* Deterministic artefact order — reuses
  :func:`r2_find_artefacts.find_r2_artefacts`'s sorted-by-path
  ordering. Two callers aggregating the same directory see
  byte-identical summaries.
* Validation is delegated: discovery errors surface as
  :mod:`r2_find_artefacts` would raise them (``FileNotFoundError``,
  ``NotADirectoryError``, ``ValueError`` / ``TypeError`` with a
  ``r2_find_artefacts:`` prefix on half-matching names); per-file
  schema errors surface as :mod:`r2_read_artefact` would raise them
  (``r2_read_artefact:`` prefix). This module itself contributes
  one error class, reported with a ``r2_aggregate:`` prefix: a bad
  ``run_dir`` type (``TypeError``). Duplicate primary-key tuples
  cannot occur in a well-formed run dir — the writer's filename is
  derived from ``(stage, arm, controller, payload)`` and filesystems
  enforce filename uniqueness within a directory, so two artefacts
  for the same combo cannot coexist under one ``run_dir``. The
  reader's filename / content round-trip check then forbids a
  hand-renamed file from presenting a different combo than its
  name. The aggregator therefore does not re-check uniqueness:
  doing so would be dead code given those two invariants.
* ``overall_pass`` is strict: ``True`` iff ``total > 0`` **and**
  every artefact ``passed``. An empty run dir yields
  ``total=0, overall_pass=False`` — a run without artefacts is
  almost always a test-harness bug, and the strict verdict lets a
  caller treat it as a failure without an extra guard.
* No side effects. Reads files; does not write, rename, or delete
  anything. The summary is a pure function of the directory snapshot
  at call time.

Non-goals
---------

* No report rendering (CSV / Markdown). That is a separate seam,
  consuming this module's output.
* No ROS / MuJoCo / numpy imports. Pure stdlib + the on-disk chain.
* No schema migration. ``schema_version: 1`` is pinned via the
  reader; a future incompatible change cuts a v2 aggregator rather
  than dual-supporting.
* No cross-run aggregation. One run directory in, one summary out.

Alignment with the rest of the pre-bake chain
---------------------------------------------

* Pure stdlib. No PyYAML / numpy / ROS imports in this module; the
  reader brings its own PyYAML dependency.
* Sibling modules loaded via file-path ``importlib`` so the
  ``R2Artefact`` class identity stays stable whether tests import
  this module directly (under the unit-test gate) or as part of the
  ``tests.integration`` package (under a future integration driver).
* Error-message prefix: ``r2_aggregate:`` for this module's own
  invariants; underlying layers keep their own prefixes when they
  raise.
"""

from __future__ import annotations

import importlib.util
import sys
from dataclasses import dataclass, field
from pathlib import Path
from types import MappingProxyType
from typing import Mapping, Tuple

_THIS_DIR = Path(__file__).resolve().parent


def _load_sibling(module_name: str):
    """Import a sibling ``tests/integration/`` module by file path.

    Mirrors the convention used by :mod:`r2_read_artefact` and
    :mod:`r2_find_artefacts` so the :class:`R2Artefact` dataclass
    identity is shared across the chain.
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
_find = _load_sibling("r2_find_artefacts")
_read = _load_sibling("r2_read_artefact")


__all__ = (
    "R2RunSummary",
    "R2Tally",
    "aggregate_r2_run",
)


# ---------------------------------------------------------------------------
# Result dataclasses
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class R2Tally:
    """Pass/fail counts for one slice of an R2 run.

    Attributes
    ----------
    total
        Total number of artefacts in this slice (always ``passed +
        failed``; pinned by :func:`aggregate_r2_run` construction).
    passed
        Number of artefacts with ``passed=True``.
    failed
        Number of artefacts with ``passed=False``.
    """

    total: int
    passed: int
    failed: int


_EMPTY_MAP_INT: Mapping[int, R2Tally] = MappingProxyType({})
_EMPTY_MAP_STR: Mapping[str, R2Tally] = MappingProxyType({})


@dataclass(frozen=True)
class R2RunSummary:
    """Aggregated verdict over every R2 artefact in a run directory.

    Attributes
    ----------
    run_dir
        The directory passed to :func:`aggregate_r2_run`, preserved
        verbatim (not resolved) so a caller can echo it in a report.
    artefacts
        Deterministic-ordered tuple of :class:`R2Artefact` instances,
        one per file in ``run_dir`` matching the R2 artefact name
        shape. Ordering follows :func:`r2_find_artefacts.find_r2_artefacts`
        (sorted by filesystem path).
    total, passed, failed
        Counts over :attr:`artefacts`; ``total == passed + failed``.
    overall_pass
        ``True`` iff ``total > 0`` and ``failed == 0``. An empty run
        directory yields ``False`` — see module docstring.
    failed_artefacts
        Convenience tuple of the failing subset of :attr:`artefacts`,
        in the same order. A report writer typically shows these
        first; pinning the subset here avoids each caller re-filtering.
    by_stage
        Maps ``stage`` (int, one of
        :data:`r2_run_artefact.SUPPORTED_STAGES`) to its
        :class:`R2Tally`. Absent stages are absent from the map —
        iterating ``by_stage.items()`` yields exactly the stages
        present in this run.
    by_arm, by_controller, by_payload
        Same structure as :attr:`by_stage`, keyed on ``arm`` / the
        controller name / ``payload`` respectively. All maps are
        read-only views (``MappingProxyType``) so a caller cannot
        mutate the summary post-construction.
    """

    run_dir: Path
    artefacts: Tuple[
        object, ...
    ]  # Tuple[R2Artefact, ...] — declared loose to avoid a forward-ref import dance.
    total: int
    passed: int
    failed: int
    overall_pass: bool
    failed_artefacts: Tuple[object, ...] = field(default_factory=tuple)
    by_stage: Mapping[int, R2Tally] = field(default_factory=lambda: _EMPTY_MAP_INT)
    by_arm: Mapping[str, R2Tally] = field(default_factory=lambda: _EMPTY_MAP_STR)
    by_controller: Mapping[str, R2Tally] = field(default_factory=lambda: _EMPTY_MAP_STR)
    by_payload: Mapping[str, R2Tally] = field(default_factory=lambda: _EMPTY_MAP_STR)


# ---------------------------------------------------------------------------
# Aggregation entrypoint
# ---------------------------------------------------------------------------


def _tally(passed: int, failed: int) -> R2Tally:
    return R2Tally(total=passed + failed, passed=passed, failed=failed)


def _group_tallies(keys: Tuple, passed_flags: Tuple[bool, ...]) -> Mapping:
    """Build a ``{key: R2Tally}`` map from parallel ``keys`` / ``passed_flags``.

    The returned mapping is a ``MappingProxyType`` so the caller
    cannot mutate it. Keys are iterated in first-seen order by
    sorting on the string form — since the underlying artefacts
    iterate in sorted-by-path order, string-sort order on the key
    gives a stable deterministic iteration regardless of key type.
    """
    accum: dict = {}
    for k, ok in zip(keys, passed_flags):
        p, f = accum.get(k, (0, 0))
        if ok:
            p += 1
        else:
            f += 1
        accum[k] = (p, f)
    # Deterministic iteration order: sort by stringified key so int
    # (stage) and str (arm / controller / payload) maps are both
    # stable across Python dict-ordering assumptions.
    ordered = {k: _tally(p, f) for k, (p, f) in sorted(accum.items(), key=lambda kv: str(kv[0]))}
    return MappingProxyType(ordered)


def aggregate_r2_run(run_dir: Path) -> R2RunSummary:
    """Walk ``run_dir``, read every R2 artefact, and return a summary.

    Parameters
    ----------
    run_dir
        Directory containing R2 artefact YAML files written by
        :func:`r2_run_artefact.write_r2_artefact`. Must be an existing
        directory. Files that don't match the R2 artefact name shape
        (``r2_stage*.yaml``) are silently skipped; half-matching
        names raise through :func:`r2_find_artefacts.find_r2_artefacts`.

    Returns
    -------
    R2RunSummary
        Aggregated verdict. Empty run dir yields
        ``R2RunSummary(total=0, overall_pass=False)``.

    Raises
    ------
    TypeError
        If ``run_dir`` is not a :class:`pathlib.Path`. (The find /
        read layers each raise their own ``TypeError`` for their
        inputs; ``run_dir`` is this module's input so this prefix is
        ``r2_aggregate:``.)
    FileNotFoundError, NotADirectoryError
        Re-raised from :func:`r2_find_artefacts.find_r2_artefacts`
        with its own prefix.
    ValueError
        Via the discovery / reader layers (half-matching filename,
        schema violation, filename/content mismatch, ...). This
        module does not raise ``ValueError`` on its own — see module
        docstring for why duplicate primary keys cannot occur in a
        well-formed ``run_dir``.
    """
    if not isinstance(run_dir, Path):
        raise TypeError(f"r2_aggregate: run_dir must be pathlib.Path, got {type(run_dir).__name__}")

    # Discovery layer does existence / type checks and filename parsing.
    locators = _find.find_r2_artefacts(run_dir)

    artefacts = tuple(_read.read_r2_artefact(loc.path) for loc in locators)

    total = len(artefacts)
    failed_artefacts = tuple(a for a in artefacts if not a.passed)
    failed = len(failed_artefacts)
    passed = total - failed
    overall_pass = total > 0 and failed == 0

    stages = tuple(a.stage for a in artefacts)
    arms = tuple(a.arm for a in artefacts)
    controllers = tuple(a.controller for a in artefacts)
    payloads = tuple(a.payload for a in artefacts)
    passed_flags = tuple(a.passed for a in artefacts)

    by_stage = _group_tallies(stages, passed_flags)
    by_arm = _group_tallies(arms, passed_flags)
    by_controller = _group_tallies(controllers, passed_flags)
    by_payload = _group_tallies(payloads, passed_flags)

    return R2RunSummary(
        run_dir=run_dir,
        artefacts=artefacts,
        total=total,
        passed=passed,
        failed=failed,
        overall_pass=overall_pass,
        failed_artefacts=failed_artefacts,
        by_stage=by_stage,
        by_arm=by_arm,
        by_controller=by_controller,
        by_payload=by_payload,
    )
