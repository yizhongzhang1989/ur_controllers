"""R2 result → artefact bridge.

Pre-baked helper for the R2 integration test matrix (M6.12 / M6.13 /
M6.14, still gated on the M6.0 operator decision). Converts a stage
assertion harness's typed result into the :class:`R2Artefact` the
run-artefact writer consumes — closing the last seam between the R2
assertion harnesses and the on-disk artefact writer in the pre-bake
chain.

Input is one of the four stage result types:

* :class:`r2_stage1_assertions.Stage1Result` for ``stage=1``
* :class:`r2_stage2_assertions.Stage2Result` for ``stage=2`` (joint-space)
* :class:`r2_stage2_cartesian.Stage2CartesianResult` for ``stage=2``
  (cartesian)
* :class:`r2_stage3_assertions.TcpStage3Result` for ``stage=3``

Output is a :class:`r2_run_artefact.R2Artefact` ready to hand to
:func:`r2_run_artefact.write_r2_artefact`.

Design choices
--------------

* **Caller owns ``theoretical``.** Different evaluators read different
  fields from the expectation YAMLs; forcing one canonical extraction
  here would couple this module to the expectation schema. Instead,
  the caller passes the theoretical block (e.g. the tolerance values
  they asserted against) as a mapping, and this bridge just carries it
  through.
* **Stage-2 joint-space flattening.** ``Stage2Result.per_joint`` is a
  list of per-joint metric maps; the artefact's ``measured`` block is
  a flat ``Mapping[str, Any]``. This bridge flattens per-joint metrics
  into ``{joint}.{metric}`` keys so an R2 run artefact stays
  single-level and greppable, and auto-populates
  ``metadata['joints']`` with the joint ordering so the original
  per-joint structure is recoverable.
* **Stage-1 joint echo.** Stage-1 evaluations are per-joint; the
  ``joint`` attribute moves from the typed result into
  ``metadata['joint']`` so the artefact filename remains
  ``r2_stage1_{arm}_{controller}_{payload}.yaml`` (no joint suffix)
  and callers running the full six-joint matrix pick joint out of the
  metadata block.
* **Stage-3 notes.** ``TcpStage3Result.notes`` (e.g. "steady-drift
  window too short, skipped") is propagated to
  ``metadata['stage3_notes']`` so it's visible in the YAML without
  polluting ``measured``.
* **Controller echo check.** The ``controller`` field on the typed
  result must match the ``controller`` argument. Otherwise the
  artefact's filename and its embedded ``controller`` key would point
  at different things.
* **No overwrite of caller metadata.** If the caller already sets a
  metadata key this bridge would populate (``joint`` / ``joints`` /
  ``stage3_notes``), raises :class:`ValueError` instead of silently
  dropping either value.

Non-goals
---------

* No writing: the caller threads the returned :class:`R2Artefact`
  through :func:`r2_run_artefact.write_r2_artefact`.
* No reading of expectation YAMLs: the theoretical block comes in
  pre-built.
* No numpy / ROS / MuJoCo imports. Pure stdlib plus the three sibling
  modules (loaded via ``importlib`` because ``tests/integration/`` is
  not on ``sys.path`` as a package — matches the convention in
  ``r2_stage{1,2,3}_assertions.py``).
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from typing import Any, Mapping, Optional

__all__ = ("result_to_artefact",)


_HERE = Path(__file__).resolve().parent


def _load_sibling(name: str):
    # Reuse an already-loaded sibling if present under its plain name —
    # unit tests load these modules directly via ``_load(name, path)``
    # and the ``isinstance`` checks below would fail against a
    # freshly-reimported class object. Fall back to the namespaced key
    # for isolation when nothing has pre-loaded the module.
    mod = sys.modules.get(name)
    if mod is not None:
        return mod
    key = f"_r2bridge_{name}"
    mod = sys.modules.get(key)
    if mod is not None:
        return mod
    spec = importlib.util.spec_from_file_location(key, _HERE / f"{name}.py")
    assert spec and spec.loader, f"cannot locate sibling module {name!r}"
    mod = importlib.util.module_from_spec(spec)
    sys.modules[key] = mod
    spec.loader.exec_module(mod)
    return mod


_stage1 = _load_sibling("r2_stage1_assertions")
_stage2 = _load_sibling("r2_stage2_assertions")
_stage2c = _load_sibling("r2_stage2_cartesian")
_stage3 = _load_sibling("r2_stage3_assertions")
_artefact_mod = _load_sibling("r2_run_artefact")

Stage1Result = _stage1.Stage1Result
Stage2Result = _stage2.Stage2Result
Stage2CartesianResult = _stage2c.Stage2CartesianResult
TcpStage3Result = _stage3.TcpStage3Result
R2Artefact = _artefact_mod.R2Artefact
SUPPORTED_STAGES = _artefact_mod.SUPPORTED_STAGES


def _raise(msg: str) -> None:
    raise ValueError(f"r2_result_to_artefact: {msg}")


def _reserved_key(base: Mapping[str, Any], key: str, value: Any) -> dict:
    if key in base:
        _raise(
            f"metadata key {key!r} is reserved by this bridge but caller already set it "
            f"(caller={base[key]!r}, bridge={value!r})"
        )
    out = dict(base)
    out[key] = value
    return out


def result_to_artefact(
    *,
    stage: int,
    arm: str,
    controller: str,
    payload: str,
    theoretical: Mapping[str, Any],
    result: Any,
    metadata: Optional[Mapping[str, Any]] = None,
) -> R2Artefact:
    """Bridge a stage assertion-harness result into an :class:`R2Artefact`.

    Parameters
    ----------
    stage:
        One of :data:`r2_run_artefact.SUPPORTED_STAGES`.
    arm, controller, payload:
        Echoed into the artefact and into its derived filename. The
        ``controller`` argument must match the ``controller`` attribute
        of the typed ``result``.
    theoretical:
        Mapping carried through to :attr:`R2Artefact.theoretical`. The
        writer validates shape (finite floats, JSON-safe leaves);
        missing fields are the caller's responsibility.
    result:
        One of :class:`Stage1Result` (``stage=1``), :class:`Stage2Result`
        or :class:`Stage2CartesianResult` (``stage=2``), or
        :class:`TcpStage3Result` (``stage=3``).
    metadata:
        Optional extra metadata. The bridge augments this with:
        ``'joint'`` (stage-1), ``'joints'`` (stage-2 joint-space),
        ``'stage3_notes'`` (stage-3, only when non-empty). Clashes
        raise :class:`ValueError` rather than silently overwriting.

    Returns
    -------
    R2Artefact
        A frozen artefact ready for :func:`write_r2_artefact`.

    Raises
    ------
    ValueError
        On argument shape violations (wrong type for ``stage`` /
        ``arm`` / ``controller`` / ``payload``, non-mapping
        ``theoretical`` / ``metadata``, stage↔result-type mismatch,
        controller-echo mismatch, reserved-metadata-key clash).
    """
    if not isinstance(stage, int) or isinstance(stage, bool):
        _raise(f"stage must be int, got {type(stage).__name__}")
    if stage not in SUPPORTED_STAGES:
        _raise(f"stage must be one of {SUPPORTED_STAGES}, got {stage}")
    for name, value in (("arm", arm), ("controller", controller), ("payload", payload)):
        if not isinstance(value, str):
            _raise(f"{name} must be str, got {type(value).__name__}")
        if not value:
            _raise(f"{name} must be a non-empty string")
    if not isinstance(theoretical, Mapping):
        _raise(f"theoretical must be a Mapping, got {type(theoretical).__name__}")
    if metadata is None:
        metadata = {}
    elif not isinstance(metadata, Mapping):
        _raise(f"metadata must be a Mapping or None, got {type(metadata).__name__}")

    base_meta: Mapping[str, Any] = metadata

    if stage == 1:
        if not isinstance(result, Stage1Result):
            _raise(f"stage=1 requires a Stage1Result, got {type(result).__name__}")
        if result.controller != controller:
            _raise(
                f"controller echo mismatch: argument={controller!r}, "
                f"result.controller={result.controller!r}"
            )
        measured = dict(result.metrics)
        reasons = tuple(result.failures)
        passed = bool(result.ok)
        out_meta = _reserved_key(base_meta, "joint", result.joint)

    elif stage == 2:
        if isinstance(result, Stage2Result):
            if result.controller != controller:
                _raise(
                    f"controller echo mismatch: argument={controller!r}, "
                    f"result.controller={result.controller!r}"
                )
            measured: dict = {}
            joint_names = []
            for j in result.per_joint:
                joint_names.append(j.joint)
                for k, v in j.metrics.items():
                    flat_key = f"{j.joint}.{k}"
                    if flat_key in measured:
                        _raise(
                            f"stage-2 flattened key collision: {flat_key!r} "
                            f"(duplicate joint name in per_joint?)"
                        )
                        # pragma: no cover
                    measured[flat_key] = v
            reasons = tuple(result.failures)
            passed = bool(result.ok)
            out_meta = _reserved_key(base_meta, "joints", tuple(joint_names))
        elif isinstance(result, Stage2CartesianResult):
            if result.controller != controller:
                _raise(
                    f"controller echo mismatch: argument={controller!r}, "
                    f"result.controller={result.controller!r}"
                )
            measured = dict(result.metrics)
            reasons = tuple(result.failures)
            passed = bool(result.ok)
            out_meta = dict(base_meta)
        else:
            _raise(
                f"stage=2 requires a Stage2Result or Stage2CartesianResult, "
                f"got {type(result).__name__}"
            )

    elif stage == 3:
        if not isinstance(result, TcpStage3Result):
            _raise(f"stage=3 requires a TcpStage3Result, got {type(result).__name__}")
        if result.controller != controller:
            _raise(
                f"controller echo mismatch: argument={controller!r}, "
                f"result.controller={result.controller!r}"
            )
        measured = dict(result.metrics)
        reasons = tuple(result.failures)
        passed = bool(result.ok)
        if result.notes:
            out_meta = _reserved_key(base_meta, "stage3_notes", tuple(result.notes))
        else:
            out_meta = dict(base_meta)

    else:  # pragma: no cover - guarded above
        _raise(f"unreachable: stage={stage}")

    return R2Artefact(
        stage=stage,
        arm=arm,
        controller=controller,
        payload=payload,
        theoretical=dict(theoretical),
        measured=measured,
        passed=passed,
        reasons=reasons,
        metadata=out_meta,
    )
