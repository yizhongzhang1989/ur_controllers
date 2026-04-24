"""R2 run-artefact writer: (theoretical, measured, verdict) -> YAML on disk.

Pre-baked helper for the R2 integration test matrix (M6.12 / M6.13 /
M6.14, still gated on the M6.0 operator decision). The R2 hard
requirements (``docs/ROADMAP.md`` §"M6 hard requirements", bullet R2,
lines ~398-408) mandate verbatim:

    Each stage must publish, in the run artefact under
    ``evaluation/runs/<ts>/``, the **theoretical expectation** alongside
    the measured result, and the test fails if the gap exceeds the
    documented tolerance.

This module is the single place in the pre-bake chain that writes that
artefact. The three stage assertion harnesses (``r2_stage{1,2,3}_assertions``,
``r2_stage2_cartesian``) produce the ``theoretical`` and ``measured`` dicts
plus a pass/fail verdict; this module serialises the triple to a
deterministic YAML file under the caller-supplied run directory.

Contract
--------

* Output file name is derived, not user-chosen:
  ``r2_stage{stage}_{arm}_{controller}_{payload}.yaml``. Keeping the
  name derived pins a single shape across stages so the M5 comparison
  driver can glob for R2 artefacts without a schema-file lookup.
* Output path: ``<run_dir>/<derived-name>``. ``run_dir`` is created
  (parents too) if missing. Existing files are not overwritten unless
  ``overwrite=True`` is passed — accidental stomps of a prior run are
  almost always a bug at this layer.
* YAML is serialised with ``sort_keys=False`` and an explicit
  top-level key order (``schema_version`` first, then ``stage``, ``arm``,
  ``controller``, ``payload``, ``passed``, ``reasons``, ``metadata``,
  ``theoretical``, ``measured``). Nested dicts **are** sorted
  alphabetically by key inside the writer (after normalisation) so two
  runs that produce the same logical payload emit byte-identical files.
* ``theoretical`` and ``measured`` are recursively normalised before
  serialisation: ``Mapping`` -> ``dict``, ``tuple`` -> ``list``, all
  dict keys must be ``str``, all floats must be finite, and only
  JSON-safe leaf types (``str``, ``int``, ``float``, ``bool``, ``None``)
  are permitted. Validation errors carry a path-specific locator
  (``theoretical.response.zeta: non-finite float nan``) so a failing R2
  test points at the offending field instead of a flat stack trace.
* ``passed=False`` requires a non-empty ``reasons`` tuple; a failed
  artefact with no reasons gives downstream consumers no diagnosis and
  is almost always a test-author mistake. ``passed=True`` allows empty
  ``reasons``.

Non-goals
---------

* No timestamping of ``run_dir`` — the caller decides (e.g. the M5
  compare driver already picks a UTC timestamp; R2 tests will reuse
  that convention). This helper writes *into* whatever directory it is
  given.
* No schema evolution. ``schema_version: 1`` is pinned; a future
  incompatible change cuts a v2 writer rather than silently shifting
  fields.
* No reading / parsing of run artefacts. Consumers call
  ``yaml.safe_load(path.read_text())`` — the serialised shape is
  JSON-safe so a third-party reader works without this module.

Alignment with the rest of the pre-bake chain
---------------------------------------------

* Pure stdlib plus ``PyYAML`` (already a repo dependency — see
  ``expectations_loader.py`` and the M6 R3 helpers).
* No ROS / MuJoCo / numpy imports.
* Raises :class:`ValueError` / :class:`TypeError` with a leading
  ``r2_run_artefact:`` prefix matching the rest of the R2 pre-bake
  chain.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from types import MappingProxyType
from typing import Any, Mapping, Tuple

import yaml

try:  # Direct import when tests/integration/ is on sys.path.
    import expectations_loader as _el
except ImportError:  # Fallback: imported as part of the tests.integration package.
    from tests.integration import expectations_loader as _el  # type: ignore[no-redef]

__all__ = (
    "R2Artefact",
    "SCHEMA_VERSION",
    "SUPPORTED_STAGES",
    "artefact_filename",
    "write_r2_artefact",
)

SCHEMA_VERSION: int = 1
SUPPORTED_STAGES: Tuple[int, ...] = (1, 2, 3)

# Explicit top-level key ordering for the emitted YAML. ``schema_version``
# first so a human grep-ing the file sees the contract version at a glance;
# ``theoretical`` and ``measured`` last because they are the bulk of the
# document and a reviewer skimming top-to-bottom reads the context fields
# (stage / arm / controller / payload / verdict / reasons / metadata) first.
_TOP_LEVEL_KEY_ORDER: Tuple[str, ...] = (
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


# ---------------------------------------------------------------------------
# Result dataclass
# ---------------------------------------------------------------------------


_EMPTY_METADATA: Mapping[str, Any] = MappingProxyType({})


@dataclass(frozen=True)
class R2Artefact:
    """The (theoretical, measured, verdict) triple for one R2 test run.

    Instance validation is deferred to :func:`write_r2_artefact` so a
    caller can incrementally construct the payload (e.g. fill
    ``theoretical`` before ``measured`` lands) without tripping
    validation on intermediate states. The writer is the single choke
    point that enforces the on-disk contract.
    """

    stage: int
    arm: str
    controller: str
    payload: str
    theoretical: Mapping[str, Any]
    measured: Mapping[str, Any]
    passed: bool
    reasons: Tuple[str, ...] = field(default_factory=tuple)
    metadata: Mapping[str, Any] = field(default_factory=lambda: _EMPTY_METADATA)


# ---------------------------------------------------------------------------
# Validation helpers
# ---------------------------------------------------------------------------


def _raise(msg: str) -> None:
    raise ValueError(f"r2_run_artefact: {msg}")


def _normalise(value: Any, path: str) -> Any:
    """Recursively convert ``value`` into a JSON-safe, yaml-dumpable form.

    Raises ``ValueError`` with a path locator on any violation.
    """
    # Booleans are a subclass of int in Python; keep them distinct.
    if isinstance(value, bool) or value is None:
        return value
    if isinstance(value, int):
        return value
    if isinstance(value, float):
        if not math.isfinite(value):
            _raise(f"{path}: non-finite float {value!r}")
        return value
    if isinstance(value, str):
        return value
    if isinstance(value, Mapping):
        out: dict = {}
        for k, v in value.items():
            if not isinstance(k, str):
                _raise(f"{path}: non-str key {k!r} (type {type(k).__name__})")
            if not k:
                _raise(f"{path}: empty-string key not allowed")
            out[k] = _normalise(v, f"{path}.{k}")
        # Sort keys so nested dicts serialise deterministically.
        return {k: out[k] for k in sorted(out)}
    if isinstance(value, (list, tuple)):
        return [_normalise(v, f"{path}[{i}]") for i, v in enumerate(value)]
    _raise(f"{path}: unsupported type {type(value).__name__}")
    return None  # unreachable; _raise raises.


def _validate_mapping(
    value: Any,
    *,
    label: str,
) -> dict:
    if not isinstance(value, Mapping):
        _raise(f"{label} must be a mapping, got {type(value).__name__}")
    normalised = _normalise(value, label)
    # _normalise returns a dict for a Mapping input; narrow the type.
    assert isinstance(normalised, dict)
    return normalised


def _validate_reasons(reasons: Any, *, passed: bool) -> Tuple[str, ...]:
    if not isinstance(reasons, tuple):
        _raise(
            f"reasons must be a tuple[str, ...], got {type(reasons).__name__}"
            " (the dataclass default is a tuple; pass tuple(...) not list(...))"
        )
    for i, r in enumerate(reasons):
        if not isinstance(r, str):
            _raise(f"reasons[{i}] must be str, got {type(r).__name__}")
        if not r:
            _raise(f"reasons[{i}] must be non-empty")
    if not passed and not reasons:
        _raise(
            "reasons must be non-empty when passed=False "
            "(a failing artefact without a diagnosis is almost always a test bug)"
        )
    return tuple(reasons)


def _validate_controller(controller: Any) -> str:
    if not isinstance(controller, str):
        _raise(f"controller must be str, got {type(controller).__name__}")
    if not controller:
        _raise("controller must be non-empty")
    if "/" in controller:
        _raise(f"controller={controller!r} must not contain '/' (reserved for path separator)")
    # The filename derivation also uses '_' as a joiner between fields;
    # allowing it inside the controller name is fine because a filename
    # round-trip is not promised (we always write, never parse the name).
    return controller


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------


def artefact_filename(
    *,
    stage: int,
    arm: str,
    controller: str,
    payload: str,
) -> str:
    """Derive the artefact filename from the primary keys.

    Exposed so test authors can glob / lookup a specific artefact under
    a run directory without duplicating the format string.
    """
    if stage not in SUPPORTED_STAGES:
        _raise(f"stage must be one of {SUPPORTED_STAGES}, got {stage!r}")
    if arm not in _el.SUPPORTED_ARMS:
        _raise(f"arm must be one of {_el.SUPPORTED_ARMS}, got {arm!r}")
    if payload not in _el.PAYLOAD_LEVELS:
        _raise(f"payload must be one of {_el.PAYLOAD_LEVELS}, got {payload!r}")
    controller = _validate_controller(controller)
    return f"r2_stage{stage}_{arm}_{controller}_{payload}.yaml"


def write_r2_artefact(
    run_dir: Path,
    artefact: R2Artefact,
    *,
    overwrite: bool = False,
) -> Path:
    """Serialise ``artefact`` to ``run_dir/<derived-name>.yaml``.

    Creates ``run_dir`` (and parents) if missing. Returns the written
    path. Raises :class:`FileExistsError` if the target file already
    exists and ``overwrite`` is ``False``.

    All validation (stage / arm / payload / controller / reasons /
    JSON-safeness of ``theoretical`` and ``measured``) runs *before*
    any file I/O, so a rejected artefact never half-writes.
    """
    if not isinstance(artefact, R2Artefact):
        _raise(
            f"artefact must be an R2Artefact, got {type(artefact).__name__}"
            " (construct one with r2_run_artefact.R2Artefact(...))"
        )
    if not isinstance(run_dir, Path):
        _raise(f"run_dir must be a pathlib.Path, got {type(run_dir).__name__}")

    # Validates stage / arm / payload / controller.
    filename = artefact_filename(
        stage=artefact.stage,
        arm=artefact.arm,
        controller=artefact.controller,
        payload=artefact.payload,
    )

    if not isinstance(artefact.passed, bool):
        _raise(f"passed must be bool, got {type(artefact.passed).__name__}")

    reasons = _validate_reasons(artefact.reasons, passed=artefact.passed)
    theoretical = _validate_mapping(artefact.theoretical, label="theoretical")
    measured = _validate_mapping(artefact.measured, label="measured")
    metadata = _validate_mapping(artefact.metadata, label="metadata")

    target = run_dir / filename
    if target.exists() and not overwrite:
        raise FileExistsError(
            f"r2_run_artefact: refusing to overwrite {target} (pass overwrite=True to force)"
        )

    document = {
        "schema_version": SCHEMA_VERSION,
        "stage": artefact.stage,
        "arm": artefact.arm,
        "controller": artefact.controller,
        "payload": artefact.payload,
        "passed": artefact.passed,
        "reasons": list(reasons),
        "metadata": metadata,
        "theoretical": theoretical,
        "measured": measured,
    }
    # Assert the key set matches the pinned order exactly — a silent
    # drift would break the deterministic-bytes contract.
    assert (
        tuple(document.keys()) == _TOP_LEVEL_KEY_ORDER
    ), "internal: top-level key set drifted from _TOP_LEVEL_KEY_ORDER"

    run_dir.mkdir(parents=True, exist_ok=True)
    text = yaml.safe_dump(
        document,
        sort_keys=False,
        default_flow_style=False,
        allow_unicode=True,
    )
    target.write_text(text, encoding="utf-8")
    return target
