"""R2 stage-2 theoretical-block builder.

Pre-baked helper for the R2 integration test matrix (M6.13, still
gated on the M6.0 operator decision). Companion to
:mod:`tests.integration.r2_stage1_theoretical` — that module carries
the stage-1 (single-joint) dispatch; this module carries the stage-2
(all-joints-together) dispatch.

Converts an :class:`expectations_loader.Stage2Tolerances` or
:class:`expectations_loader.Stage2TcpTolerances` into the
``theoretical`` mapping that
:func:`r2_result_to_artefact.result_to_artefact` accepts and that
:func:`r2_run_artefact.write_r2_artefact` serialises verbatim into
the R2 run artefact.

Scope
-----

Stage 2 only. Covers both the *joint-space* path (consumed by
:mod:`r2_stage2_assertions`) and the *cartesian-mode* path (consumed
by :mod:`r2_stage2_cartesian`). The theoretical prediction for
stage 2 is not a closed-form response like stage 1's second-order
block — the ROADMAP R2 text (§M6.R2 stage 2) phrases it as
"kinematic consistency: TCP FK from measured ``q`` matches the
expected trajectory within tolerance (joint-space) or 5 mm + 2°
(cartesian mode)", i.e. the commanded trajectory itself *is* the
theoretical prediction, and what the artefact needs to record is
the interpretation (joint-space or cartesian) plus the pass/fail
tolerances. That is exactly what this module emits.

Contract
--------

* Two narrow entry points, one per stage-2 path, matching the two
  R2 stage-2 assertion harnesses:

  - :func:`theoretical_for_stage2_joint_space` takes a
    :class:`expectations_loader.Stage2Tolerances` and emits a block
    with ``response_model='kinematic_consistency_joint_space'`` and
    the three tolerances
    (``completion_tol_rad`` / ``peak_tracking_err_rad`` /
    ``saturation_hold_ms``).
  - :func:`theoretical_for_stage2_cartesian` takes a
    :class:`expectations_loader.Stage2TcpTolerances` and emits a
    block with ``response_model='kinematic_consistency_tcp'`` and
    the two tolerances
    (``position_peak_err_mm`` / ``orientation_peak_err_deg``).

* Output is a plain :class:`dict` (not ``MappingProxyType``) ready
  to hand straight to
  :func:`r2_result_to_artefact.result_to_artefact` — the writer
  sorts deterministically before emitting YAML.
* All numeric values are plain ``float`` (loader already validates
  finite-ness at load time; this module re-wraps in ``float`` to
  harden against any future non-``float`` leaves in the dataclass).
* Strict typing: arguments must be the exact dataclass type from
  :mod:`expectations_loader`. Silently accepting a bare ``dict`` (or
  the wrong dataclass) would be an easy test-authoring trap.

Non-goals
---------

* No reading of the expectation YAML. The caller passes a typed
  dataclass (usually obtained via ``arm_exp.stage2`` /
  ``arm_exp.stage2_tcp``).
* No writing: the caller threads the returned ``dict`` through
  :func:`r2_result_to_artefact.result_to_artefact` and
  :func:`r2_run_artefact.write_r2_artefact`.
* No ROS / MuJoCo / numpy imports. Pure stdlib plus the sibling
  :mod:`expectations_loader` module (loaded via ``importlib`` so this
  module works both under the unit-test gate's direct load and when
  imported as part of the ``tests.integration`` package).
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from typing import Any, Dict

_THIS_DIR = Path(__file__).resolve().parent


def _load_sibling(module_name: str):
    """Import a sibling ``tests/integration/`` module by file path.

    Checks ``sys.modules`` for the plain name first so ``isinstance``
    checks line up across callers (unit tests, stage-harness modules,
    and this builder). Mirrors the convention in
    :mod:`r2_stage1_theoretical` and :mod:`r2_result_to_artefact`.
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


_EL = _load_sibling("expectations_loader")


__all__ = (
    "SUPPORTED_MODES",
    "theoretical_for_stage2_joint_space",
    "theoretical_for_stage2_cartesian",
)


# Pinned here rather than inferred so a new mode in a future stage-2
# harness trips the dispatch instead of silently slipping through.
SUPPORTED_MODES = ("joint_space", "cartesian")


def _raise(msg: str) -> None:
    raise ValueError(f"r2_stage2_theoretical: {msg}")


def theoretical_for_stage2_joint_space(stage2_tol) -> Dict[str, Any]:
    """Build the stage-2 joint-space theoretical block.

    Input
    -----
    ``stage2_tol``
        :class:`expectations_loader.Stage2Tolerances` — typically
        ``arm_exp.stage2`` from
        :func:`expectations_loader.load_arm`.

    Output
    ------
    ``dict`` with two keys:

    * ``response_model='kinematic_consistency_joint_space'`` — pins
      the ROADMAP R2 §stage 2 joint-space semantics (``q(t) ≈
      q_cmd(t)`` with per-joint bounds).
    * ``tolerances`` — mapping with the three stage-2 joint-space
      tolerances (``completion_tol_rad``, ``peak_tracking_err_rad``,
      ``saturation_hold_ms``). Values are plain ``float``.
    """
    if not isinstance(stage2_tol, _EL.Stage2Tolerances):
        _raise(
            "stage2_tol must be expectations_loader.Stage2Tolerances, "
            f"got {type(stage2_tol).__name__}"
        )

    return {
        "response_model": "kinematic_consistency_joint_space",
        "tolerances": {
            "completion_tol_rad": float(stage2_tol.completion_tol_rad),
            "peak_tracking_err_rad": float(stage2_tol.peak_tracking_err_rad),
            "saturation_hold_ms": float(stage2_tol.saturation_hold_ms),
        },
    }


def theoretical_for_stage2_cartesian(stage2_tcp_tol) -> Dict[str, Any]:
    """Build the stage-2 cartesian-mode theoretical block.

    Input
    -----
    ``stage2_tcp_tol``
        :class:`expectations_loader.Stage2TcpTolerances` — typically
        ``arm_exp.stage2_tcp`` from
        :func:`expectations_loader.load_arm`.

    Output
    ------
    ``dict`` with two keys:

    * ``response_model='kinematic_consistency_tcp'`` — pins the
      ROADMAP R2 §stage 2 cartesian-mode semantics (TCP FK of
      measured ``q`` matches the commanded TCP trajectory within
      5 mm + 2°).
    * ``tolerances`` — mapping with the two stage-2 cartesian
      tolerances (``position_peak_err_mm``,
      ``orientation_peak_err_deg``). Values are plain ``float``.
    """
    if not isinstance(stage2_tcp_tol, _EL.Stage2TcpTolerances):
        _raise(
            "stage2_tcp_tol must be expectations_loader.Stage2TcpTolerances, "
            f"got {type(stage2_tcp_tol).__name__}"
        )

    return {
        "response_model": "kinematic_consistency_tcp",
        "tolerances": {
            "position_peak_err_mm": float(stage2_tcp_tol.position_peak_err_mm),
            "orientation_peak_err_deg": float(stage2_tcp_tol.orientation_peak_err_deg),
        },
    }
