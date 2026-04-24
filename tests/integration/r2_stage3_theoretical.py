"""R2 stage-3 theoretical-block builder.

Pre-baked helper for the R2 integration test matrix (M6.14, still
gated on the M6.0 operator decision and on the downstream FK-source
decision for the stage-3 orchestrator — see ``docs/STATUS.md``).
Completes the R2 theoretical-block pre-bake chain: stage-1 dispatch
lives in :mod:`tests.integration.r2_stage1_theoretical`, the two
stage-2 paths in :mod:`tests.integration.r2_stage2_theoretical`, and
this module handles stage-3 (end-effector TCP trajectory).

Converts an :class:`expectations_loader.ArmExpectation` into the
``theoretical`` mapping that
:func:`r2_result_to_artefact.result_to_artefact` accepts and that
:func:`r2_run_artefact.write_r2_artefact` serialises verbatim into
the R2 run artefact.

Scope
-----

Stage 3 only. Pulls the four TCP tolerances pinned by the expectation
schema (``tests/unit/test_expectations_schema.py``:
``TCP_TOLERANCE_KEYS``):

* ``tcp_rmse_mm``
* ``tcp_peak_err_mm``
* ``tcp_orientation_peak_deg``
* ``tcp_steady_drift_mm_per_30s``

These map one-to-one onto the kwargs of
:func:`r2_stage3_assertions.evaluate_tcp_trajectory` and mirror the
ROADMAP R2 §stage-3 text ("TCP RMSE below 5 mm, peak below 10 mm;
yaw/pitch/roll peak error below 3°; steady-state TCP drift below
2 mm over 30 s").

Contract
--------

* Input: one :class:`expectations_loader.ArmExpectation`.
* Output: plain :class:`dict` (not ``MappingProxyType``) with two
  keys:

  - ``response_model='tcp_trajectory_tracking'`` — pins the
    ROADMAP R2 §stage-3 semantics (commanded TCP trajectory tracked
    within the four per-arm tolerances).
  - ``tolerances`` — mapping with all four TCP tolerance keys above.
    Values are plain ``float``.

* Strict typing: the argument must be an
  :class:`expectations_loader.ArmExpectation`. Silently accepting a
  bare ``dict`` (or the wrong dataclass) would mean the pre-bake no
  longer guarantees the numbers came from an expectation YAML.

Non-goals
---------

* No reading of the expectation YAML. The caller passes a typed
  dataclass (usually obtained via
  :func:`expectations_loader.load_arm`).
* No writing: the caller threads the returned ``dict`` through
  :func:`r2_result_to_artefact.result_to_artefact` and
  :func:`r2_run_artefact.write_r2_artefact`.
* No ROS / MuJoCo / numpy imports. Pure stdlib plus the sibling
  :mod:`expectations_loader` module (loaded via ``importlib`` so
  this module works both under the unit-test gate's direct load and
  when imported as part of the ``tests.integration`` package).
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
    :mod:`r2_stage1_theoretical` / :mod:`r2_stage2_theoretical`.
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
    "SUPPORTED_TOLERANCE_KEYS",
    "theoretical_for_stage3",
)


# Pinned here (matching ``test_expectations_schema.py:TCP_TOLERANCE_KEYS``)
# rather than inferred from the expectation YAMLs so a new key in a
# future YAML trips this module's dispatch instead of silently slipping
# through. Tuple (not set) so the on-disk ordering is deterministic and
# unit tests can pin it exactly.
SUPPORTED_TOLERANCE_KEYS = (
    "tcp_rmse_mm",
    "tcp_peak_err_mm",
    "tcp_orientation_peak_deg",
    "tcp_steady_drift_mm_per_30s",
)


def _raise(msg: str) -> None:
    raise ValueError(f"r2_stage3_theoretical: {msg}")


def theoretical_for_stage3(arm_exp) -> Dict[str, Any]:
    """Build the stage-3 TCP-trajectory theoretical block.

    Input
    -----
    ``arm_exp``
        :class:`expectations_loader.ArmExpectation` — typically the
        return value of :func:`expectations_loader.load_arm`.

    Output
    ------
    ``dict`` with two keys:

    * ``response_model='tcp_trajectory_tracking'`` — pins the ROADMAP
      R2 §stage-3 semantics (commanded TCP trajectory tracked within
      the four per-arm TCP tolerances).
    * ``tolerances`` — mapping with the four stage-3 TCP tolerances
      listed in :data:`SUPPORTED_TOLERANCE_KEYS`, as plain ``float``.
      Missing keys surface
      :class:`expectations_loader.ArmExpectation.tcp_tol`'s native
      :class:`KeyError` (with the ``available: [...]`` diagnostic
      intact) rather than being silently defaulted.
    """
    if not isinstance(arm_exp, _EL.ArmExpectation):
        _raise(
            "arm_exp must be expectations_loader.ArmExpectation, "
            f"got {type(arm_exp).__name__}"
        )

    return {
        "response_model": "tcp_trajectory_tracking",
        "tolerances": {k: float(arm_exp.tcp_tol(k)) for k in SUPPORTED_TOLERANCE_KEYS},
    }
