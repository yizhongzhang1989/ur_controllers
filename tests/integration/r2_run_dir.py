"""R2 run-directory helper: canonical ``evaluation/runs/r2__<UTC-ts>/`` path.

Pre-baked helper for the R2 integration test matrix (M6.12 / M6.13 /
M6.14, still gated on the M6.0 operator decision). The R2 hard
requirements (``docs/ROADMAP.md`` §"M6 hard requirements", bullet R2,
lines ~398-408) mandate that each stage publishes its artefact under
``evaluation/runs/<ts>/``. The companion writer
:mod:`r2_run_artefact` deliberately declines to pick a timestamp, noting
verbatim:

    No timestamping of ``run_dir`` — the caller decides (e.g. the M5
    compare driver already picks a UTC timestamp; R2 tests will reuse
    that convention). This helper writes *into* whatever directory it
    is given.

This module is the "R2 tests will reuse that convention" piece: it
picks the UTC timestamp, builds the canonical directory name, creates
the directory, and returns the :class:`~pathlib.Path`. The timestamp
format matches :func:`evaluation.run_evaluation.make_run_dir` so the
M5 compare driver's ``find_latest_run_dir`` lexicographic sort keeps
working across R2 artefacts written into the same ``runs/`` tree.

Contract
--------

* Timestamp format is pinned to ``TS_FORMAT = "%Y%m%dT%H%M%SZ"`` — the
  format :mod:`evaluation.run_evaluation` already writes. Lexicographic
  sort matches chronological order (pinned by a unit test).
* Directory layout: ``<runs_root>/<prefix>__<UTC-ts>`` with
  ``prefix`` defaulting to ``"r2"``. The double-underscore separator
  mirrors the M5 convention so globbing is uniform across stages.
* Optional ``suffix`` is appended as ``__<suffix>`` **after** the
  timestamp so multiple R2 sweeps started in the same second (e.g.
  one per ``{arm, payload}`` combination in a test worker pool) can
  be disambiguated without touching the timestamp itself.
* ``runs_root`` defaults to ``<repo_root>/evaluation/runs`` — the
  gitignored directory every existing runner uses.
* ``runs_root`` is created (parents too) if missing.
* The run directory is created with ``exist_ok=False`` by default so
  a sub-second collision surfaces as :class:`FileExistsError` instead
  of silently co-mingling artefacts from two unrelated runs. Callers
  that genuinely want to reuse a pre-existing directory pass
  ``exist_ok=True``.
* ``clock`` is injectable (``Callable[[], datetime]``) so tests can
  pin the timestamp without monkey-patching the ``datetime`` module.

Non-goals
---------

* No reading / listing of existing R2 run directories. That is the M5
  compare driver's ``find_latest_run_dir`` job; keeping this helper
  write-only avoids drift between two discovery implementations.
* No writing of artefacts. :func:`r2_run_artefact.write_r2_artefact`
  owns the on-disk contract; this helper only produces the parent
  path.
* No coupling to the R2 test matrix. ``prefix`` / ``suffix`` are
  caller-supplied strings — this helper never encodes stage / arm /
  controller / payload into the path. Those belong in the artefact
  filename (``artefact_filename`` in :mod:`r2_run_artefact`).

Alignment with the rest of the pre-bake chain
---------------------------------------------

* Pure stdlib; no PyYAML / numpy / ROS imports.
* Raises :class:`ValueError` / :class:`TypeError` with a leading
  ``r2_run_dir:`` prefix matching the rest of the R2 pre-bake chain.
"""

from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path
from typing import Callable, Optional

__all__ = (
    "DEFAULT_PREFIX",
    "DEFAULT_RUNS_ROOT",
    "TS_FORMAT",
    "make_r2_run_dir",
)

TS_FORMAT: str = "%Y%m%dT%H%M%SZ"
DEFAULT_PREFIX: str = "r2"

# ``tests/integration/r2_run_dir.py`` -> parents[2] == repo root.
_REPO_ROOT: Path = Path(__file__).resolve().parents[2]
DEFAULT_RUNS_ROOT: Path = _REPO_ROOT / "evaluation" / "runs"


def _raise_value(msg: str) -> None:
    raise ValueError(f"r2_run_dir: {msg}")


def _raise_type(msg: str) -> None:
    raise TypeError(f"r2_run_dir: {msg}")


def _validate_token(value: str, *, label: str) -> str:
    if not isinstance(value, str):
        _raise_type(f"{label} must be str, got {type(value).__name__}")
    if not value:
        _raise_value(f"{label} must be non-empty")
    if "/" in value or "\\" in value:
        _raise_value(f"{label}={value!r} must not contain path separators")
    if "__" in value:
        _raise_value(f"{label}={value!r} must not contain '__' " "(reserved as field separator)")
    # Leading / trailing whitespace would survive into the filename
    # and cause surprising glob behaviour; disallow up front.
    if value != value.strip():
        _raise_value(f"{label}={value!r} must not have leading / trailing whitespace")
    return value


def _utc_now() -> datetime:
    return datetime.now(timezone.utc)


def make_r2_run_dir(
    runs_root: Optional[Path] = None,
    *,
    clock: Optional[Callable[[], datetime]] = None,
    prefix: str = DEFAULT_PREFIX,
    suffix: Optional[str] = None,
    exist_ok: bool = False,
) -> Path:
    """Pick a UTC timestamp, create, and return a canonical R2 run dir.

    Parameters
    ----------
    runs_root:
        Parent directory under which the run dir is created. ``None``
        resolves to :data:`DEFAULT_RUNS_ROOT`
        (``<repo>/evaluation/runs``, which is gitignored). Must be a
        :class:`~pathlib.Path` when explicitly given — passing a
        string is rejected so call sites have one canonical path
        type throughout.
    clock:
        Optional zero-argument callable returning a
        :class:`~datetime.datetime` with ``tzinfo=timezone.utc``.
        Defaults to a module-private ``datetime.now(timezone.utc)``
        wrapper. Exposed for unit tests that need a pinned
        timestamp without monkey-patching :mod:`datetime`.
    prefix:
        First component of the directory name, preceding the
        timestamp. Defaults to ``"r2"``. Must be a non-empty string
        with no ``/``, ``\\``, ``__``, or surrounding whitespace.
    suffix:
        Optional trailing component appended as ``__<suffix>`` after
        the timestamp. Same validation rules as ``prefix``. Handy for
        disambiguating parallel R2 sweeps that would otherwise collide
        in the same UTC second.
    exist_ok:
        Forwarded to :meth:`Path.mkdir`. Defaults to ``False`` so
        sub-second collisions raise :class:`FileExistsError` instead
        of silently reusing an unrelated directory.

    Returns
    -------
    Path
        The created directory (``<runs_root>/<prefix>__<ts>[__<suffix>]``).

    Raises
    ------
    TypeError
        ``runs_root`` is not ``None`` / :class:`Path`; ``prefix`` /
        ``suffix`` are not ``str``; ``clock`` is not callable or does
        not return a :class:`datetime`; ``exist_ok`` is not
        :class:`bool`.
    ValueError
        ``prefix`` / ``suffix`` fail the non-empty / separator /
        whitespace checks; ``clock`` returns a naive datetime or a
        non-UTC datetime.
    FileExistsError
        The target directory already exists and ``exist_ok`` is
        ``False``.
    """
    if runs_root is None:
        runs_root_p = DEFAULT_RUNS_ROOT
    elif isinstance(runs_root, Path):
        runs_root_p = runs_root
    else:
        _raise_type(f"runs_root must be pathlib.Path or None, got {type(runs_root).__name__}")
        raise AssertionError  # pragma: no cover -- for type-checkers

    if not isinstance(exist_ok, bool):
        _raise_type(f"exist_ok must be bool, got {type(exist_ok).__name__}")

    prefix_v = _validate_token(prefix, label="prefix")
    suffix_v: Optional[str]
    if suffix is None:
        suffix_v = None
    else:
        suffix_v = _validate_token(suffix, label="suffix")

    if clock is None:
        clock_fn: Callable[[], datetime] = _utc_now
    elif callable(clock):
        clock_fn = clock
    else:
        _raise_type(f"clock must be callable, got {type(clock).__name__}")
        raise AssertionError  # pragma: no cover

    now = clock_fn()
    if not isinstance(now, datetime):
        _raise_type(f"clock() must return datetime, got {type(now).__name__}")
    if now.tzinfo is None:
        _raise_value("clock() returned a naive datetime; expected UTC-aware")
    # Require UTC specifically — the TS format drops tzinfo, so a
    # non-UTC datetime would silently write a mislabelled timestamp.
    if now.utcoffset() != timezone.utc.utcoffset(None):
        _raise_value(f"clock() returned tzinfo={now.tzinfo!r}; expected UTC")

    ts = now.strftime(TS_FORMAT)
    name = f"{prefix_v}__{ts}"
    if suffix_v is not None:
        name = f"{name}__{suffix_v}"

    run_dir = runs_root_p / name
    # Ensure the parent exists without raising on a pre-existing tree.
    runs_root_p.mkdir(parents=True, exist_ok=True)
    run_dir.mkdir(parents=False, exist_ok=exist_ok)
    return run_dir
