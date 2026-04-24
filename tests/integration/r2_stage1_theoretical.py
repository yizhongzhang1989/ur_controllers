"""R2 stage-1 theoretical-block builder.

Pre-baked helper for the R2 integration test matrix (M6.12, still
gated on the M6.0 operator decision). Converts an expectation row
for one controller×joint pair into the ``theoretical`` mapping that
:func:`r2_result_to_artefact.result_to_artefact` accepts and that
:func:`r2_run_artefact.write_r2_artefact` serialises verbatim into the
R2 run artefact.

Background
----------

``r2_result_to_artefact`` intentionally keeps the theoretical block
caller-supplied ("forcing one canonical extraction here would couple
this module to the expectation schema"). That leaves the
expectations-loader ↔ artefact-writer seam open: each R2 stage-1 test
body would otherwise open-code the `response_model` dispatch and
reassemble the same structured block. This module is that shared
dispatch — one place where the four known `response_model` values map
to their theoretical contents, so a failing stage-1 assertion in any
R2 test emits byte-identical theoretical blocks across arms and
controllers.

Scope
-----

Stage 1 only. Stage 2 / stage 3 theoretical blocks are structurally
different (joint-space all-joints vs cartesian-mode TCP tolerances);
they will get their own builders in follow-ups that reuse the shape
discipline established here.

Contract
--------

* Input: one :class:`expectations_loader.ControllerExpectation`, plus
  optional :class:`expectations_loader.JointExpectation` and
  configured ``stiffness_k`` / ``damping_d`` that are only consumed
  by the ``second_order`` response model.
* Output: plain :class:`dict` (not ``MappingProxyType``) suitable to
  hand straight to :func:`r2_result_to_artefact.result_to_artefact`
  (which will re-normalise and validate before writing).
* Keys inside the output are not sorted here — the run-artefact writer
  sorts deterministically before emitting YAML. Callers that want to
  compare dict shape in tests should sort explicitly.
* All numeric values are plain ``float`` (``JointExpectation`` and
  ``ControllerExpectation`` already enforce finite floats at load
  time; the ``second_order`` helper additionally validates
  ``K > 0`` / ``J > 0`` / ``D >= 0``).
* Strict argument matching: the three optional kwargs
  (``joint_exp``, ``stiffness_k``, ``damping_d``) must be *exactly*
  the set the ``response_model`` needs — providing any of them for a
  first-order or open-loop-torque controller raises
  :class:`ValueError` rather than silently ignoring. A silently
  ignored K/D would be a very easy test-authoring mistake to miss.

Non-goals
---------

* No reading of ``bringup/config/*.yaml`` — K / D are caller-supplied
  (matches :func:`expectations_loader.second_order_response`).
* No writing: the caller chains this output through
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
from typing import Any, Dict, Optional

_THIS_DIR = Path(__file__).resolve().parent


def _load_sibling(module_name: str):
    """Import a sibling module under ``tests/integration/`` by file path.

    Checks ``sys.modules`` for the plain name first so ``isinstance``
    checks across callers (unit tests loading via
    ``spec_from_file_location``, stage-harness modules loading via
    namespaced names, and this builder) see the same class objects.
    Mirrors the convention in ``r2_result_to_artefact.py``.
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
    "SUPPORTED_RESPONSE_MODELS",
    "theoretical_for_stage1",
)


# Pinned here rather than inferred from the expectation YAMLs so a new
# model string in a future YAML trips this module's dispatch (forcing a
# deliberate schema bump) instead of silently slipping through.
SUPPORTED_RESPONSE_MODELS = ("first_order_lag", "open_loop_torque", "second_order")


def _raise(msg: str) -> None:
    raise ValueError(f"r2_stage1_theoretical: {msg}")


def _require_tol(controller_exp, keys):
    """Return a ``dict`` of ``{k: float(tol)}`` for all ``keys``.

    Missing keys surface ``expectations_loader``'s native
    ``KeyError`` with the "available: [...]" diagnostic intact —
    this module does not reformat the message because the loader's
    one is already actionable.
    """
    return {k: float(controller_exp.tol(k)) for k in keys}


def theoretical_for_stage1(
    controller_exp,
    *,
    joint_exp=None,
    stiffness_k: Optional[float] = None,
    damping_d: Optional[float] = None,
) -> Dict[str, Any]:
    """Build the stage-1 theoretical block for one controller×joint pair.

    Dispatches on ``controller_exp.response_model``:

    * ``first_order_lag`` → ``steady_state_err_rad`` /
      ``drift_rad_per_10s`` / ``fft_peak_db_above_noise_floor``
      tolerances. ``joint_exp`` / ``stiffness_k`` / ``damping_d`` must
      all be ``None``.
    * ``open_loop_torque`` → ``max_runaway_rad`` /
      ``saturation_hold_ms`` tolerances. Same strict-``None`` rule.
    * ``second_order`` → ``bounded_err_rad`` /
      ``chatter_velocity_rms_rad_s`` / ``damping_ratio_pct``
      tolerances **plus** a ``response`` sub-block with
      ``omega_n_rad_s`` / ``zeta`` computed via
      :func:`expectations_loader.second_order_response`, echoing the
      input ``stiffness_k`` / ``damping_d`` /
      ``effective_inertia_kg_m2`` and the joint name for
      downstream auditing. ``joint_exp`` / ``stiffness_k`` /
      ``damping_d`` are all required.

    Returns a plain ``dict`` with string keys and finite-float /
    string leaves — ready to pass as the ``theoretical`` argument to
    :func:`r2_result_to_artefact.result_to_artefact`.
    """
    if not isinstance(controller_exp, _EL.ControllerExpectation):
        _raise(
            "controller_exp must be expectations_loader.ControllerExpectation, "
            f"got {type(controller_exp).__name__}"
        )

    model = controller_exp.response_model
    if model not in SUPPORTED_RESPONSE_MODELS:
        _raise(
            f"unknown response_model {model!r} on controller "
            f"{controller_exp.name!r}; expected one of {SUPPORTED_RESPONSE_MODELS}"
        )

    if model in ("first_order_lag", "open_loop_torque"):
        # Strict: K/D/joint_exp are silently ignored if we accept them,
        # which almost always hides a test-authoring mistake.
        for arg_name, arg_value in (
            ("joint_exp", joint_exp),
            ("stiffness_k", stiffness_k),
            ("damping_d", damping_d),
        ):
            if arg_value is not None:
                _raise(
                    f"{arg_name}={arg_value!r} provided for response_model={model!r} "
                    "which does not consume it (pass only for response_model='second_order')"
                )

    if model == "first_order_lag":
        return {
            "response_model": "first_order_lag",
            "interface": controller_exp.interface,
            "tolerances": _require_tol(
                controller_exp,
                ("steady_state_err_rad", "drift_rad_per_10s", "fft_peak_db_above_noise_floor"),
            ),
        }

    if model == "open_loop_torque":
        return {
            "response_model": "open_loop_torque",
            "interface": controller_exp.interface,
            "tolerances": _require_tol(controller_exp, ("max_runaway_rad", "saturation_hold_ms")),
        }

    # second_order
    if joint_exp is None:
        _raise("joint_exp is required for response_model='second_order'")
    if not isinstance(joint_exp, _EL.JointExpectation):
        _raise(
            "joint_exp must be expectations_loader.JointExpectation, "
            f"got {type(joint_exp).__name__}"
        )
    if stiffness_k is None:
        _raise("stiffness_k is required for response_model='second_order'")
    if damping_d is None:
        _raise("damping_d is required for response_model='second_order'")
    if isinstance(stiffness_k, bool) or not isinstance(stiffness_k, (int, float)):
        _raise(f"stiffness_k must be a number, got {type(stiffness_k).__name__}")
    if isinstance(damping_d, bool) or not isinstance(damping_d, (int, float)):
        _raise(f"damping_d must be a number, got {type(damping_d).__name__}")

    k = float(stiffness_k)
    d = float(damping_d)
    j = float(joint_exp.effective_inertia_kg_m2)

    # Delegates K>0 / D>=0 / J>0 validation to the loader so error
    # text matches the stage-1 assertion harness on malformed input.
    omega_n, zeta = _EL.second_order_response(k, d, j)

    return {
        "response_model": "second_order",
        "interface": controller_exp.interface,
        "response": {
            "joint": joint_exp.name,
            "stiffness_k": k,
            "damping_d": d,
            "effective_inertia_kg_m2": j,
            "omega_n_rad_s": float(omega_n),
            "zeta": float(zeta),
        },
        "tolerances": _require_tol(
            controller_exp,
            ("bounded_err_rad", "chatter_velocity_rms_rad_s", "damping_ratio_pct"),
        ),
    }
