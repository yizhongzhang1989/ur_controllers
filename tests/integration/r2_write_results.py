"""R2 bulk results writer.

Pre-baked convenience helper that closes the "live orchestrator has a
batch of stage results + per-combo context and needs to deposit them as
YAML artefacts in one R2 run directory" seam. Chains
:func:`r2_result_to_artefact.result_to_artefact` into
:func:`r2_run_artefact.write_r2_artefact` so an M5-analogue R2 driver no
longer has to open-code the bridge/write loop or its duplicate-combo
guard at every caller.

Public API
----------

* :class:`R2ResultEntry` — frozen bundle of ``(stage, arm, controller,
  payload, theoretical, result, metadata)``. Same keyword shape as
  :func:`r2_result_to_artefact.result_to_artefact`; the metadata
  argument is carried through unchanged.
* :func:`write_r2_result` — single-entry shim; returns the written
  path.
* :func:`write_r2_results` — bulk driver over a sequence of
  entries; returns written paths in input order.

Design choices
--------------

* **Bridge first, then write.** All entries are converted to
  :class:`R2Artefact` up front via the bridge, then written in a
  second pass. This keeps the "never half-write" contract the
  single-artefact writer pins: a mid-batch bridging failure (e.g. a
  stage/result mismatch on entry 5/10) prevents any on-disk side
  effect, not just "stops further writes".
* **Duplicate-combo detection is bulk-only.** Within a single call,
  two entries sharing the same
  ``(stage, arm, controller, payload)`` 4-tuple are rejected up-front
  with a clear ``ValueError`` locating both indices. The filesystem
  would later catch this as :class:`FileExistsError` (overwrite=False)
  but the message is muddier and the first write would already have
  landed. Between calls, overwrite semantics are delegated to the
  single-artefact writer.
* **Sequence, not iterable.** ``entries`` must be ``list`` or
  ``tuple`` — we need to iterate twice (duplicate scan + bridge pass
  + write pass) and a single-shot generator would break that.
* **No Sequence[R2ResultEntry] isinstance coupling.** Each entry is
  validated individually; the caller can mix homogeneous stage-1 and
  heterogeneous stage-3 entries in one call.
* **Error prefix.** All direct errors are prefixed
  ``r2_write_results:`` — bridge errors keep their
  ``r2_result_to_artefact:`` prefix and writer errors keep their
  ``r2_run_artefact:`` prefix, matching the rest of the R2 pre-bake
  chain so callers can `grep` for the layer that rejected them.

Alignment with the R2 pre-bake chain
------------------------------------

* Sibling modules loaded via file-path ``importlib`` with plain
  module-name keys (matching the
  ``r2_result_to_artefact`` / ``r2_aggregate`` / ``r2_report_writer``
  convention, see the "module loading" repo memory) so the
  :class:`R2ResultEntry` we accept shares
  :class:`r2_run_artefact.R2Artefact` /
  :class:`r2_result_to_artefact` identities with the rest of the
  chain — an entry built against the bridge module imported directly
  by a test and an entry built through this module's re-export go
  through the same code paths.
* Pure stdlib imports in this module. PyYAML is pulled in transitively
  by the writer; no numpy / ROS imports here.
"""

from __future__ import annotations

import importlib.util
import sys
from dataclasses import dataclass, field
from pathlib import Path
from types import MappingProxyType
from typing import Any, List, Mapping, Optional, Sequence, Tuple

__all__ = (
    "R2ResultEntry",
    "write_r2_result",
    "write_r2_results",
)


_HERE = Path(__file__).resolve().parent


def _load_sibling(name: str):
    mod = sys.modules.get(name)
    if mod is not None:
        return mod
    key = f"_r2wr_{name}"
    mod = sys.modules.get(key)
    if mod is not None:
        return mod
    spec = importlib.util.spec_from_file_location(key, _HERE / f"{name}.py")
    assert spec and spec.loader, f"cannot locate sibling module {name!r}"
    mod = importlib.util.module_from_spec(spec)
    sys.modules[key] = mod
    spec.loader.exec_module(mod)
    return mod


_artefact_mod = _load_sibling("r2_run_artefact")
_bridge_mod = _load_sibling("r2_result_to_artefact")

R2Artefact = _artefact_mod.R2Artefact
_write_r2_artefact = _artefact_mod.write_r2_artefact
_artefact_filename = _artefact_mod.artefact_filename
_result_to_artefact = _bridge_mod.result_to_artefact


_EMPTY_METADATA: Mapping[str, Any] = MappingProxyType({})


@dataclass(frozen=True)
class R2ResultEntry:
    """One stage result plus the context the bridge needs to wrap it.

    Mirrors the keyword signature of
    :func:`r2_result_to_artefact.result_to_artefact`. Field ordering is
    chosen for readability — primary keys first, then the theoretical
    block, then the measured ``result``, then optional metadata.
    """

    stage: int
    arm: str
    controller: str
    payload: str
    theoretical: Mapping[str, Any]
    result: Any
    metadata: Optional[Mapping[str, Any]] = field(default=None)


def _raise_value(msg: str) -> None:
    raise ValueError(f"r2_write_results: {msg}")


def _raise_type(msg: str) -> None:
    raise TypeError(f"r2_write_results: {msg}")


def _check_run_dir(run_dir: Any) -> Path:
    if not isinstance(run_dir, Path):
        _raise_type(f"run_dir must be pathlib.Path, got {type(run_dir).__name__}")
    return run_dir


def _check_entry(entry: Any, *, where: str) -> R2ResultEntry:
    if not isinstance(entry, R2ResultEntry):
        _raise_type(
            f"{where} must be an R2ResultEntry, got {type(entry).__name__}"
            " (construct one with r2_write_results.R2ResultEntry(...))"
        )
    return entry


def _combo_key(entry: R2ResultEntry) -> Tuple[Any, Any, Any, Any]:
    return (entry.stage, entry.arm, entry.controller, entry.payload)


def _scan_duplicates(entries: Sequence[R2ResultEntry]) -> None:
    seen: dict = {}
    for i, entry in enumerate(entries):
        key = _combo_key(entry)
        if key in seen:
            _raise_value(
                f"duplicate combo {key!r} at entries[{seen[key]}] and entries[{i}] "
                "(each (stage, arm, controller, payload) must be unique within a batch)"
            )
        seen[key] = i


def write_r2_result(
    run_dir: Path,
    entry: R2ResultEntry,
    *,
    overwrite: bool = False,
) -> Path:
    """Bridge one :class:`R2ResultEntry` and write its artefact.

    Thin wrapper around ``result_to_artefact`` + ``write_r2_artefact``
    so a test writing a single artefact doesn't need to import both
    siblings. Argument validation is delegated to those modules; this
    shim only rejects obviously-wrong ``run_dir`` / ``entry`` /
    ``overwrite`` types before any side effect.
    """
    run_dir = _check_run_dir(run_dir)
    entry = _check_entry(entry, where="entry")
    if not isinstance(overwrite, bool):
        _raise_type(f"overwrite must be bool, got {type(overwrite).__name__}")

    artefact = _result_to_artefact(
        stage=entry.stage,
        arm=entry.arm,
        controller=entry.controller,
        payload=entry.payload,
        theoretical=entry.theoretical,
        result=entry.result,
        metadata=entry.metadata,
    )
    return _write_r2_artefact(run_dir, artefact, overwrite=overwrite)


def write_r2_results(
    run_dir: Path,
    entries: Sequence[R2ResultEntry],
    *,
    overwrite: bool = False,
) -> Tuple[Path, ...]:
    """Bridge and write a batch of entries into ``run_dir``.

    Parameters
    ----------
    run_dir:
        Destination directory; created (``parents=True``) by the
        underlying writer if missing.
    entries:
        ``list`` / ``tuple`` of :class:`R2ResultEntry`. Must not
        contain two entries with the same
        ``(stage, arm, controller, payload)`` combo — that is a bulk
        pre-condition rather than a filesystem collision, so it's
        rejected before any I/O.
    overwrite:
        Forwarded to the single-artefact writer for every entry.

    Returns
    -------
    tuple of Path
        Written paths in **input order**. A caller driving the M5-like
        R2 orchestrator can therefore zip the return value against
        ``entries`` without re-running
        :func:`r2_run_artefact.artefact_filename`.

    Raises
    ------
    TypeError
        ``run_dir`` not a :class:`Path`, ``entries`` not a list/tuple,
        an entry not an :class:`R2ResultEntry`, or ``overwrite`` not a
        ``bool``.
    ValueError
        Duplicate combo within ``entries``, or any bridge-level shape
        violation (prefixed ``r2_result_to_artefact:``) / writer-level
        shape violation (prefixed ``r2_run_artefact:``) propagated as
        raised.
    FileExistsError
        Propagated from the single-artefact writer when
        ``overwrite=False`` and a target file already exists on disk.
        Combined with the up-front bridge pass this means the first
        conflicting file aborts the batch **before any other** entry
        has been written, keeping the "never half-write" contract.
    """
    run_dir = _check_run_dir(run_dir)
    if not isinstance(entries, (list, tuple)):
        _raise_type(f"entries must be list or tuple, got {type(entries).__name__}")
    if not isinstance(overwrite, bool):
        _raise_type(f"overwrite must be bool, got {type(overwrite).__name__}")
    for i, entry in enumerate(entries):
        _check_entry(entry, where=f"entries[{i}]")

    _scan_duplicates(entries)

    # Bridge pass — fail before any I/O if any entry is malformed.
    artefacts: List[Any] = []
    for entry in entries:
        artefacts.append(
            _result_to_artefact(
                stage=entry.stage,
                arm=entry.arm,
                controller=entry.controller,
                payload=entry.payload,
                theoretical=entry.theoretical,
                result=entry.result,
                metadata=entry.metadata,
            )
        )

    # Write pass — only now may the filesystem see anything.
    paths: List[Path] = []
    for artefact in artefacts:
        paths.append(_write_r2_artefact(run_dir, artefact, overwrite=overwrite))
    return tuple(paths)
