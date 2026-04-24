"""R2 artefact reader: parse a written artefact YAML back into R2Artefact.

Pre-baked helper for the R2 integration test matrix (M6.12 / M6.13 /
M6.14, still gated on the M6.0 operator decision). Companion to the
writer :mod:`r2_run_artefact` and the discovery helper
:mod:`r2_find_artefacts` — specifically closes the seam the latter's
docstring defers (module docstring, lines ~58-59):

    No validation of the YAML schema. That is the artefact reader's
    concern (and the artefact is JSON-safe by construction per the
    writer's contract).

This module is that artefact reader. The future M5-like R2
aggregation driver (analogous to ``evaluation/compare.py`` over R2
artefacts) uses this module to turn a file path produced by
:func:`r2_find_artefacts.find_r2_artefacts` into a validated
:class:`R2Artefact` in one call, without duplicating schema knowledge
across consumers.

Contract
--------

* Pure re-read of the on-disk YAML written by
  :func:`r2_run_artefact.write_r2_artefact`. The writer's validation
  is the source of truth; the reader re-asserts every constraint so a
  hand-edited or corrupted file cannot silently feed the aggregator:
  schema version, exact top-level key set (order is intentionally
  *not* checked — that's the writer's deterministic-bytes concern),
  primary-key enums, strict ``bool`` vs ``int`` for ``passed``,
  ``list[str]`` reasons with the ``passed=False => non-empty`` rule,
  and recursive JSON-safeness of ``theoretical`` / ``measured`` /
  ``metadata`` (finite floats, non-empty str keys, leaves restricted
  to ``str|int|float|bool|None`` plus list / dict containers).
* Filename / content round-trip: the file's basename must equal
  :func:`r2_run_artefact.artefact_filename` applied to the parsed
  primary keys. A renamed file discovered under the wrong
  ``(stage, arm, controller, payload)`` tuple is almost always a bug
  at the writer/reader seam.
* Duplicate YAML mapping keys are rejected (silent last-wins would
  drop fields). Scalar NaN / Inf (YAML ``.nan`` / ``.inf``) are
  rejected for the same reason — the writer can't emit them, so their
  presence means the file was hand-edited or corrupted.

Non-goals
---------

* No recursion into subdirectories, no globbing. The caller supplies
  the concrete artefact path — typically the ``path`` field of a
  :class:`r2_find_artefacts.R2ArtefactLocator`.
* No schema migration. ``schema_version: 1`` is pinned; a future
  incompatible change cuts a v2 reader rather than dual-supporting.
* No coercion. The reader re-emits the dataclass the writer accepts
  (``reasons`` as a tuple; ``theoretical`` / ``measured`` /
  ``metadata`` as plain ``dict``) — round-tripping a read artefact
  through the writer is an explicit supported use case.

Alignment with the rest of the pre-bake chain
---------------------------------------------

* Pure stdlib plus ``PyYAML`` (already a repo dependency — see
  ``expectations_loader.py`` and the M6 R3 helpers).
* No ROS / MuJoCo / numpy imports.
* All error messages carry a leading ``r2_read_artefact:`` prefix.
  ``FileNotFoundError`` / ``IsADirectoryError`` / ``TypeError`` keep
  their native classes; every schema / content violation surfaces as
  ``ValueError`` with a locator that points at the offending field.
* Sibling modules loaded via file-path ``importlib`` with a plain
  module-name key so ``R2Artefact`` returned by this module is the
  very same class users get from ``r2_run_artefact.R2Artefact`` —
  ``isinstance(read, R2Artefact)`` holds no matter which module
  imported it first.
"""

from __future__ import annotations

import importlib.util
import math
import sys
from pathlib import Path
from typing import Any, Mapping, Tuple

import yaml

_THIS_DIR = Path(__file__).resolve().parent


def _load_sibling(module_name: str):
    """Import a sibling ``tests/integration/`` module by file path.

    Matches the convention used by :mod:`r2_find_artefacts` so the
    ``R2Artefact`` dataclass stays identity-stable across the chain.
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


_el = _load_sibling("expectations_loader")
_ra = _load_sibling("r2_run_artefact")


__all__ = (
    "SCHEMA_VERSION",
    "SUPPORTED_STAGES",
    "TOP_LEVEL_KEYS",
    "read_r2_artefact",
)


# Pinned against the writer so any drift trips the export-surface
# unit test rather than silently producing mismatched documents.
SCHEMA_VERSION: int = _ra.SCHEMA_VERSION
SUPPORTED_STAGES: Tuple[int, ...] = _ra.SUPPORTED_STAGES
TOP_LEVEL_KEYS: Tuple[str, ...] = (
    "schema_version",
    "stage",
    "arm",
    "controller",
    "payload",
    "passed",
    "reasons",
    "metadata",
    "theoretical",
    "measured",
)


def _raise(msg: str) -> None:
    raise ValueError(f"r2_read_artefact: {msg}")


def _type_error(msg: str) -> None:
    raise TypeError(f"r2_read_artefact: {msg}")


# --- YAML loader that rejects duplicate mapping keys ------------------------
#
# PyYAML's default SafeLoader silently keeps the last duplicate key
# (``{a: 1, a: 2}`` -> ``{"a": 2}``), which would let a corrupted
# artefact pass schema validation after dropping fields. Reject
# duplicates up front.


class _NoDuplicateKeySafeLoader(yaml.SafeLoader):
    """SafeLoader subclass that rejects duplicate mapping keys."""


def _construct_mapping_no_duplicates(loader, node, deep=False):  # type: ignore[no-untyped-def]
    if not isinstance(node, yaml.MappingNode):
        raise yaml.constructor.ConstructorError(
            None,
            None,
            f"expected a mapping node, but found {node.id}",
            node.start_mark,
        )
    mapping: dict = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in mapping:
            raise yaml.constructor.ConstructorError(
                "while constructing a mapping",
                node.start_mark,
                f"found duplicate key {key!r}",
                key_node.start_mark,
            )
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


_NoDuplicateKeySafeLoader.add_constructor(  # type: ignore[no-untyped-call]
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
    _construct_mapping_no_duplicates,
)


# --- Recursive JSON-safeness validation -------------------------------------


def _validate_json_safe(value: Any, path: str) -> None:
    """Validate ``value`` matches the writer's ``_normalise`` contract.

    Mirrors :func:`r2_run_artefact._normalise` but asserts rather than
    transforms: the writer already normalised the payload, so any
    violation on read means the file was corrupted or hand-edited.
    """
    # Booleans are a subclass of int in Python; keep them distinct so
    # ``True`` isn't accepted as an integer 1 anywhere nested.
    if isinstance(value, bool) or value is None:
        return
    if isinstance(value, int):
        return
    if isinstance(value, float):
        if not math.isfinite(value):
            _raise(f"{path}: non-finite float {value!r}")
        return
    if isinstance(value, str):
        return
    if isinstance(value, Mapping):
        for k, v in value.items():
            if not isinstance(k, str):
                _raise(f"{path}: non-str key {k!r} (type {type(k).__name__})")
            if not k:
                _raise(f"{path}: empty-string key not allowed")
            _validate_json_safe(v, f"{path}.{k}")
        return
    if isinstance(value, list):
        for i, v in enumerate(value):
            _validate_json_safe(v, f"{path}[{i}]")
        return
    _raise(f"{path}: unsupported type {type(value).__name__}")


def _require_mapping(value: Any, *, label: str) -> dict:
    if not isinstance(value, dict):
        _raise(f"{label} must be a mapping, got {type(value).__name__}")
    _validate_json_safe(value, label)
    return value


def _require_str(value: Any, *, label: str) -> str:
    if not isinstance(value, str):
        _raise(f"{label} must be str, got {type(value).__name__}")
    return value


# --- Public API -------------------------------------------------------------


def read_r2_artefact(path: Path):
    """Read and validate an R2 artefact YAML file.

    Returns an :class:`r2_run_artefact.R2Artefact` whose fields echo
    the on-disk document. ``reasons`` is returned as a ``tuple[str, ...]``
    so the result is identity-equal to the writer's dataclass default
    and can be round-tripped through
    :func:`r2_run_artefact.write_r2_artefact` unchanged.

    Raises
    ------
    TypeError
        If ``path`` is not a :class:`pathlib.Path`.
    FileNotFoundError
        If ``path`` does not exist.
    IsADirectoryError
        If ``path`` is a directory.
    ValueError
        For every content / schema / filename-round-trip violation,
        with a ``r2_read_artefact:`` prefix and a locator that points
        at the offending field.
    """
    if not isinstance(path, Path):
        _type_error(f"path must be a pathlib.Path, got {type(path).__name__}")
    if not path.exists():
        raise FileNotFoundError(f"r2_read_artefact: artefact file does not exist: {path}")
    if path.is_dir():
        raise IsADirectoryError(f"r2_read_artefact: path is a directory, not a file: {path}")
    if not path.is_file():
        _raise(f"path is not a regular file: {path}")

    # Decode + parse. Both failure modes surface as r2_read_artefact:
    # prefixed ValueError so consumers have one error class to catch
    # across the parse/schema boundary.
    try:
        text = path.read_text(encoding="utf-8")
    except UnicodeDecodeError as exc:
        _raise(f"{path}: file is not valid UTF-8 ({exc})")
        return None  # unreachable

    try:
        doc = yaml.load(text, Loader=_NoDuplicateKeySafeLoader)  # noqa: S506 - custom SafeLoader
    except yaml.YAMLError as exc:
        _raise(f"{path}: YAML parse failed ({exc.__class__.__name__}: {exc})")
        return None  # unreachable

    if doc is None:
        _raise(f"{path}: file is empty or contains only YAML null")
    if not isinstance(doc, dict):
        _raise(f"{path}: top-level YAML must be a mapping, " f"got {type(doc).__name__}")

    # Exact top-level key set (order intentionally not checked — that
    # is the writer's deterministic-bytes concern, not the reader's).
    actual_keys = set(doc.keys())
    expected_keys = set(TOP_LEVEL_KEYS)
    missing = expected_keys - actual_keys
    extra = actual_keys - expected_keys
    if missing or extra:
        parts = []
        if missing:
            parts.append(f"missing={sorted(missing)!r}")
        if extra:
            parts.append(f"unexpected={sorted(extra)!r}")
        _raise(f"{path}: top-level key set mismatch ({', '.join(parts)})")

    # schema_version: strict int, reject bool, must equal pinned.
    sv = doc["schema_version"]
    if isinstance(sv, bool) or not isinstance(sv, int):
        _raise(f"schema_version must be int, got {type(sv).__name__} {sv!r}")
    if sv != SCHEMA_VERSION:
        _raise(
            f"schema_version must be {SCHEMA_VERSION}, got {sv!r}"
            " (this reader is v1; a future v2 writer requires a v2 reader)"
        )

    # stage: strict int, reject bool.
    stage = doc["stage"]
    if isinstance(stage, bool) or not isinstance(stage, int):
        _raise(f"stage must be int, got {type(stage).__name__} {stage!r}")
    if stage not in SUPPORTED_STAGES:
        _raise(f"stage must be one of {SUPPORTED_STAGES}, got {stage!r}")

    arm = _require_str(doc["arm"], label="arm")
    if arm not in _el.SUPPORTED_ARMS:
        _raise(f"arm must be one of {_el.SUPPORTED_ARMS}, got {arm!r}")

    payload = _require_str(doc["payload"], label="payload")
    if payload not in _el.PAYLOAD_LEVELS:
        _raise(f"payload must be one of {_el.PAYLOAD_LEVELS}, got {payload!r}")

    controller = _require_str(doc["controller"], label="controller")
    if not controller:
        _raise("controller must be non-empty")
    if "/" in controller:
        _raise(f"controller={controller!r} must not contain '/' " "(reserved for path separator)")

    # passed: strictly bool.
    passed = doc["passed"]
    if not isinstance(passed, bool):
        _raise(f"passed must be bool, got {type(passed).__name__} {passed!r}")

    # reasons: list[str], non-empty when passed=False.
    reasons_raw = doc["reasons"]
    if not isinstance(reasons_raw, list):
        _raise(
            f"reasons must be a list, got {type(reasons_raw).__name__}"
            " (the writer emits a list; YAML round-trip of a tuple flattens to a list)"
        )
    reasons_list: list = []
    for i, r in enumerate(reasons_raw):
        if not isinstance(r, str):
            _raise(f"reasons[{i}] must be str, got {type(r).__name__} {r!r}")
        if not r:
            _raise(f"reasons[{i}] must be non-empty")
        reasons_list.append(r)
    if not passed and not reasons_list:
        _raise(
            "reasons must be non-empty when passed=False "
            "(a failing artefact without a diagnosis is almost always a test bug)"
        )
    reasons: Tuple[str, ...] = tuple(reasons_list)

    metadata = _require_mapping(doc["metadata"], label="metadata")
    theoretical = _require_mapping(doc["theoretical"], label="theoretical")
    measured = _require_mapping(doc["measured"], label="measured")

    # Filename / content round-trip: the file's basename must match
    # the derived name for the parsed primary keys. Catches renames /
    # copy-paste mishaps between the writer and downstream consumers.
    expected_name = _ra.artefact_filename(
        stage=stage,
        arm=arm,
        controller=controller,
        payload=payload,
    )
    if path.name != expected_name:
        _raise(
            f"filename {path.name!r} does not match content-derived "
            f"name {expected_name!r} (file renamed or content edited?)"
        )

    return _ra.R2Artefact(
        stage=stage,
        arm=arm,
        controller=controller,
        payload=payload,
        theoretical=theoretical,
        measured=measured,
        passed=passed,
        reasons=reasons,
        metadata=metadata,
    )
