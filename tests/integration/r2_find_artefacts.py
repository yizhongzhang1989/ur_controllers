"""R2 artefact discovery helper: glob ``run_dir`` for R2 artefact YAMLs.

Pre-baked helper for the R2 integration test matrix (M6.12 / M6.13 /
M6.14, still gated on the M6.0 operator decision). The companion
writer :mod:`r2_run_artefact` pins the on-disk filename shape
(``r2_stage{stage}_{arm}_{controller}_{payload}.yaml``) and notes
verbatim (module docstring, line ~25):

    Keeping the name derived pins a single shape across stages so the
    M5 comparison driver can glob for R2 artefacts without a
    schema-file lookup.

This module is the "glob for R2 artefacts" piece: given a run
directory (typically the one returned by
:func:`r2_run_dir.make_r2_run_dir`), it enumerates every R2 artefact
file and returns a deterministic-ordered tuple of frozen
:class:`R2ArtefactLocator` records exposing the parsed
``(stage, arm, controller, payload)`` primary keys alongside the
concrete filesystem path.

Scope
-----

* Parse the filename shape fixed by
  :func:`r2_run_artefact.artefact_filename`: the ``stage`` field must
  be one of :data:`r2_run_artefact.SUPPORTED_STAGES`, the ``arm``
  field one of :data:`expectations_loader.SUPPORTED_ARMS`, and the
  ``payload`` field one of
  :data:`expectations_loader.PAYLOAD_LEVELS`. Controller is the
  middle span and may itself contain ``_`` (e.g.
  ``crisp_joint_impedance``) — parsing is anchored on the
  known-enum prefix (arm) and known-enum suffix (payload) so the
  centre span is unambiguous regardless of controller-internal
  underscores.
* Enumerate files under ``run_dir`` (non-recursive; R2 artefacts
  are always written as direct children of the run directory by
  :func:`r2_run_artefact.write_r2_artefact`).
* Optional narrow filters (``stage``, ``arm``, ``controller``,
  ``payload``) applied after parsing so a caller can pull a single
  combination in one call without post-filtering.
* Deterministic ordering: results are sorted by filesystem path so
  two callers iterating the same directory see the same sequence.

Non-goals
---------

* No reading / parsing of the YAML contents — callers interested in
  the serialised body call ``yaml.safe_load(locator.path.read_text())``
  as the writer's docstring advertises.
* No cross-directory search. The caller supplies the concrete
  ``run_dir``; enumerating candidate run directories (e.g.
  ``evaluation/runs/r2__*``) is a separate concern and intentionally
  out of scope (the R2 tests each own one run dir at a time, which
  they pick via :func:`r2_run_dir.make_r2_run_dir`).
* No recursion into subdirectories. The writer always writes to the
  top level of ``run_dir``; a recursive search would mask bugs where
  a caller accidentally wrote into a nested directory.
* No validation of the YAML schema. That is the artefact reader's
  concern (and the artefact is JSON-safe by construction per the
  writer's contract).

Alignment with the rest of the pre-bake chain
---------------------------------------------

* Pure stdlib; no PyYAML / numpy / ROS imports.
* Raises :class:`ValueError` / :class:`TypeError` / :class:`NotADirectoryError`
  with a leading ``r2_find_artefacts:`` prefix matching the rest of
  the R2 pre-bake chain.
* Sibling modules loaded via ``importlib``-based fallback so this
  module works both under the unit-test gate's direct file load and
  when imported as part of the ``tests.integration`` package.
"""

from __future__ import annotations

import importlib.util
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

_THIS_DIR = Path(__file__).resolve().parent


def _load_sibling(module_name: str):
    """Import a sibling ``tests/integration/`` module by file path.

    Mirrors the convention already used by
    :mod:`r2_stage1_theoretical` / :mod:`r2_stage2_theoretical` /
    :mod:`r2_stage3_theoretical`.
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
    "FILENAME_PREFIX",
    "FILENAME_SUFFIX",
    "R2ArtefactLocator",
    "find_r2_artefacts",
    "parse_artefact_filename",
)


# Pinned verbatim to the :func:`r2_run_artefact.artefact_filename`
# derivation so a drift on either side trips the "export surface" /
# "round-trip" unit tests rather than silently emitting mismatched
# artefacts.
FILENAME_PREFIX: str = "r2_stage"
FILENAME_SUFFIX: str = ".yaml"


@dataclass(frozen=True)
class R2ArtefactLocator:
    """Parsed artefact filename + filesystem path.

    Attributes
    ----------
    path
        Concrete filesystem path (absolute if the caller supplied an
        absolute ``run_dir``, relative otherwise).
    stage, arm, controller, payload
        Primary keys parsed from the filename. ``stage`` is ``int``;
        the rest are ``str``.
    """

    path: Path
    stage: int
    arm: str
    controller: str
    payload: str


def _raise(msg: str) -> None:
    raise ValueError(f"r2_find_artefacts: {msg}")


def _type_error(msg: str) -> None:
    raise TypeError(f"r2_find_artefacts: {msg}")


def parse_artefact_filename(filename: str) -> R2ArtefactLocator:
    """Parse a filename of the shape
    ``r2_stage{stage}_{arm}_{controller}_{payload}.yaml``.

    Returns an :class:`R2ArtefactLocator` whose ``path`` field is
    ``Path(filename)`` — i.e. the path is not resolved or joined
    against any run directory. Callers that need a full path layer
    it on top (see :func:`find_r2_artefacts`).

    Parsing is anchored on the known :data:`SUPPORTED_ARMS` (prefix
    after stage) and :data:`PAYLOAD_LEVELS` (suffix before
    ``.yaml``) enums so controller names that themselves contain
    ``_`` (e.g. ``crisp_joint_impedance``) parse unambiguously.

    Raises
    ------
    TypeError
        If ``filename`` is not a ``str``.
    ValueError
        If the filename does not match the pinned shape (unknown
        stage / arm / payload, missing controller span, bad prefix
        or suffix, non-integer stage, etc.).
    """
    if not isinstance(filename, str):
        _type_error(f"filename must be str, got {type(filename).__name__}")
    if not filename.endswith(FILENAME_SUFFIX):
        _raise(f"filename {filename!r} must end with {FILENAME_SUFFIX!r}")
    if not filename.startswith(FILENAME_PREFIX):
        _raise(f"filename {filename!r} must start with {FILENAME_PREFIX!r}")
    stem = filename[len(FILENAME_PREFIX) : -len(FILENAME_SUFFIX)]
    if not stem:
        _raise(f"filename {filename!r} has empty body between prefix and suffix")

    stage_str, sep, rest = stem.partition("_")
    if not sep or not stage_str:
        _raise(f"filename {filename!r} has no stage/arm separator")
    try:
        stage = int(stage_str)
    except ValueError:
        _raise(f"filename {filename!r} has non-integer stage {stage_str!r}")
    # int('+1') / int('-1') parse successfully; reject them so the
    # canonical emitted form is the sole round-trippable shape.
    if str(stage) != stage_str:
        _raise(f"filename {filename!r} has non-canonical stage {stage_str!r}")
    if stage not in _ra.SUPPORTED_STAGES:
        _raise(
            f"filename {filename!r} has unsupported stage {stage!r} "
            f"(expected one of {_ra.SUPPORTED_STAGES})"
        )

    arm_match: Optional[str] = None
    for candidate in _el.SUPPORTED_ARMS:
        if rest.startswith(candidate + "_"):
            arm_match = candidate
            break
    if arm_match is None:
        _raise(
            f"filename {filename!r} does not contain a known arm prefix "
            f"(expected one of {_el.SUPPORTED_ARMS})"
        )
    after_arm = rest[len(arm_match) + 1 :]
    if not after_arm:
        _raise(f"filename {filename!r} is missing controller/payload after arm")

    payload_match: Optional[str] = None
    for candidate in _el.PAYLOAD_LEVELS:
        if after_arm.endswith("_" + candidate):
            payload_match = candidate
            break
    if payload_match is None:
        _raise(
            f"filename {filename!r} does not end with a known payload suffix "
            f"(expected one of {_el.PAYLOAD_LEVELS})"
        )
    controller = after_arm[: -(len(payload_match) + 1)]
    if not controller:
        _raise(f"filename {filename!r} has empty controller span")

    return R2ArtefactLocator(
        path=Path(filename),
        stage=stage,
        arm=arm_match,
        controller=controller,
        payload=payload_match,
    )


def find_r2_artefacts(
    run_dir: Path,
    *,
    stage: Optional[int] = None,
    arm: Optional[str] = None,
    controller: Optional[str] = None,
    payload: Optional[str] = None,
) -> Tuple[R2ArtefactLocator, ...]:
    """Enumerate R2 artefact files under ``run_dir``.

    Non-recursive; direct children only — matching the writer's
    layout (see :func:`r2_run_artefact.write_r2_artefact`).

    Files under ``run_dir`` that do not match the R2 filename
    ``r2_stage*.yaml`` glob are silently ignored (a typical R2 run
    dir may also contain caller-supplied logs or scratch files).
    Files that *do* start with ``r2_stage`` and end with ``.yaml``
    but fail to parse raise :class:`ValueError` — a half-matching
    name is almost always a bug at the writer/reader seam.

    Parameters
    ----------
    run_dir
        Must be an existing :class:`pathlib.Path` pointing at a
        directory. ``str`` is rejected to match the "one canonical
        path type" convention the rest of the R2 pre-bake chain
        enforces.
    stage, arm, controller, payload
        Optional narrow filters. ``None`` means "any". ``stage``
        must be in :data:`r2_run_artefact.SUPPORTED_STAGES`,
        ``arm`` in :data:`expectations_loader.SUPPORTED_ARMS`,
        ``payload`` in :data:`expectations_loader.PAYLOAD_LEVELS`,
        and ``controller`` a non-empty ``str``.

    Returns
    -------
    tuple[R2ArtefactLocator, ...]
        Sorted by :class:`~pathlib.Path` so two callers iterating
        the same directory observe the same sequence.
    """
    if not isinstance(run_dir, Path):
        _type_error(f"run_dir must be a pathlib.Path, got {type(run_dir).__name__}")
    if not run_dir.exists():
        raise FileNotFoundError(f"r2_find_artefacts: run_dir {run_dir} does not exist")
    if not run_dir.is_dir():
        raise NotADirectoryError(f"r2_find_artefacts: run_dir {run_dir} is not a directory")

    if stage is not None:
        if isinstance(stage, bool) or not isinstance(stage, int):
            _type_error(f"stage filter must be int or None, got {type(stage).__name__}")
        if stage not in _ra.SUPPORTED_STAGES:
            _raise(f"stage filter {stage!r} must be one of " f"{_ra.SUPPORTED_STAGES} or None")
    if arm is not None:
        if not isinstance(arm, str):
            _type_error(f"arm filter must be str or None, got {type(arm).__name__}")
        if arm not in _el.SUPPORTED_ARMS:
            _raise(f"arm filter {arm!r} must be one of {_el.SUPPORTED_ARMS} or None")
    if payload is not None:
        if not isinstance(payload, str):
            _type_error(f"payload filter must be str or None, got {type(payload).__name__}")
        if payload not in _el.PAYLOAD_LEVELS:
            _raise(f"payload filter {payload!r} must be one of " f"{_el.PAYLOAD_LEVELS} or None")
    if controller is not None:
        if not isinstance(controller, str):
            _type_error(
                f"controller filter must be str or None, got " f"{type(controller).__name__}"
            )
        if not controller:
            _raise("controller filter must be a non-empty str or None")

    results: list[R2ArtefactLocator] = []
    for child in sorted(run_dir.iterdir()):
        name = child.name
        if not (name.startswith(FILENAME_PREFIX) and name.endswith(FILENAME_SUFFIX)):
            continue
        if not child.is_file():
            # A directory whose name happens to match the pattern is a
            # filesystem accident; surface it so the author notices.
            _raise(f"entry {child} matches R2 artefact name shape but is not a file")
        parsed = parse_artefact_filename(name)
        loc = R2ArtefactLocator(
            path=child,
            stage=parsed.stage,
            arm=parsed.arm,
            controller=parsed.controller,
            payload=parsed.payload,
        )
        if stage is not None and loc.stage != stage:
            continue
        if arm is not None and loc.arm != arm:
            continue
        if controller is not None and loc.controller != controller:
            continue
        if payload is not None and loc.payload != payload:
            continue
        results.append(loc)
    return tuple(results)
