"""R2 stage → controllers catalog.

Pre-baked for the R2 integration test matrix (M6.12 / M6.13 / M6.14,
still gated on the M6.0 operator decision). Canonicalises the
``stage → (controller, controller, ...)`` lookup that ROADMAP §M6.R2
enumerates, so the future live R2 orchestrator (M5-analogue) can
walk the full ``{stage × arm × controller × payload}`` matrix without
each test body open-coding its own stage membership test.

Pairs with :mod:`tests.integration.r3_payload_parametrize` (arm ×
payload axis) to give the orchestrator the four independent axes in
two narrow modules.

Scope
-----

ROADMAP §M6.R2 enumerates, per stage:

* **Stage 1 (single-joint motion, M6.12).**
  ``{JTC, forward_position, forward_effort + crisp_joint_impedance,
  simple_jimp}``. The ``+`` pairs ``forward_effort_controller`` with
  the crisp joint-impedance controller (crisp needs the effort
  interface claim) but each is a separately-asserted target — the
  expectations YAML declares both with their own ``response_model``
  (``open_loop_torque`` vs ``second_order``).
* **Stage 2 (all-joints-together, M6.13).** Joint-space controllers
  from stage 1, plus ``cartesian_motion_controller`` for the
  cartesian-mode branch (ROADMAP §M6.R2: "kinematic consistency: TCP
  FK from measured q matches expected trajectory within tolerance
  (joint-space) or 5 mm + 2° (cartesian mode)"; see
  :mod:`tests.integration.r2_stage2_cartesian` and the
  ``stage2_tcp`` block in ``expectations/*.yaml``).
* **Stage 3 (end-effector trajectory, M6.14).**
  ``{cartesian_motion_controller, JTC+ik_shim,
  crisp_cartesian_impedance}``. ``JTC+ik_shim`` is a *distinct
  variant* — it is still the ``joint_trajectory_controller`` plugin
  on the controller-manager side, but driven by an IK solver shim
  instead of a pre-baked joint trajectory, so the asserted path is
  not the same as stage 1/2. We keep it as its own variant name
  (``joint_trajectory_controller_ik_shim``) to avoid blurring the
  artefact ``controller`` key and the by-controller aggregation in
  :func:`r2_aggregate.aggregate_r2_run` — and expose
  :func:`expectation_key` as the (one-place) mapping back to the
  expectation YAML row (``joint_trajectory_controller``).

Design choices
--------------

* **Bare strings, not records.** A full per-variant descriptor
  (orchestration / expectation / launch / controller-manager names)
  is tempting but premature — the only variant that actually diverges
  today is the stage-3 JTC+IK shim. A single :func:`expectation_key`
  helper covers that case without introducing a dataclass the R2
  chain would then have to carry through `R2Artefact`.
* **Stage-1 scope matches ROADMAP, not expectations.** The expectations
  YAML declares ``forward_velocity_controller`` too, but ROADMAP
  §M6.12 does not list it as a stage-1 target. We follow ROADMAP so
  the catalog reflects *what is under test*, not *what is declared*.
  If M6 is later broadened, add it here and extend the schema test.
* **Frozen, deterministic order.** ``STAGE_CONTROLLERS`` is a
  :class:`types.MappingProxyType` wrapping a plain ``dict`` of
  ``int → tuple[str, ...]``. Insertion order for the outer mapping is
  ``1, 2, 3`` and the inner tuple order matches the ROADMAP
  enumeration so a reviewer can diff ROADMAP and this module
  line-for-line. Callers receive the same ``tuple`` object across
  calls.
* **Pure stdlib.** No ``yaml`` / numpy / ROS / pytest imports. Matches
  the sibling :mod:`r3_payload_parametrize` module.
* **Error prefix.** ``r2_stage_controllers:`` on every raised
  ``TypeError`` / ``ValueError``, matching the rest of the R2
  pre-bake chain so a caller grep-ing a stack trace can locate the
  layer that rejected them.

Non-goals
---------

* Does not load or validate ``expectations/*.yaml``. The
  :mod:`expectations_loader` module already does that; the schema
  test in :mod:`tests.unit.test_expectations_schema` pins the YAML
  shape.
* Does not spin controllers. This is a pure lookup table; the live
  orchestrator decides when / how to activate each controller.
* Does not map to ros2_control plugin class names. See
  ``bringup/launch/crisp_bringup.launch.py`` for the role →
  controller-manager-name translation (e.g. the
  ``crisp_cartesian_impedance`` *role* activates the
  ``cartesian_impedance_controller`` at the controller-manager).
  That mapping is a bringup concern, not a catalog concern.
"""

from __future__ import annotations

from types import MappingProxyType
from typing import Mapping, Tuple

__all__ = (
    "SUPPORTED_STAGES",
    "STAGE_CONTROLLERS",
    "JTC_IK_SHIM_VARIANT",
    "controllers_for_stage",
    "stages_for_controller",
    "all_controllers",
    "expectation_key",
)


SUPPORTED_STAGES: Tuple[int, ...] = (1, 2, 3)

# Distinct variant name for the stage-3 JTC-driven-by-an-IK-shim target.
# Kept separate from plain ``joint_trajectory_controller`` (stage 1/2) so
# the artefact ``controller`` key and the by-controller aggregation
# don't blur the two paths. Map back to the expectations row via
# ``expectation_key``.
JTC_IK_SHIM_VARIANT: str = "joint_trajectory_controller_ik_shim"


_STAGE_1: Tuple[str, ...] = (
    "joint_trajectory_controller",
    "forward_position_controller",
    "forward_effort_controller",
    "crisp_joint_impedance",
    "simple_joint_impedance",
)

_STAGE_2: Tuple[str, ...] = _STAGE_1 + ("cartesian_motion_controller",)

_STAGE_3: Tuple[str, ...] = (
    "cartesian_motion_controller",
    JTC_IK_SHIM_VARIANT,
    "crisp_cartesian_impedance",
)


STAGE_CONTROLLERS: Mapping[int, Tuple[str, ...]] = MappingProxyType(
    {
        1: _STAGE_1,
        2: _STAGE_2,
        3: _STAGE_3,
    }
)


# ---------------------------------------------------------------------------
# Error helpers
# ---------------------------------------------------------------------------


def _raise_type(msg: str) -> None:
    raise TypeError(f"r2_stage_controllers: {msg}")


def _raise_value(msg: str) -> None:
    raise ValueError(f"r2_stage_controllers: {msg}")


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------


def controllers_for_stage(stage: int) -> Tuple[str, ...]:
    """Return the ordered tuple of controllers exercised by ``stage``.

    ``stage`` must be one of :data:`SUPPORTED_STAGES`. ``bool`` is
    rejected explicitly — Python treats ``True``/``False`` as ``int``
    subclasses but a ``True`` ``stage`` is almost certainly a caller
    bug.
    """
    if isinstance(stage, bool) or not isinstance(stage, int):
        _raise_type(f"stage must be int, got {type(stage).__name__}")
    if stage not in STAGE_CONTROLLERS:
        _raise_value(
            f"stage {stage!r} not in SUPPORTED_STAGES {SUPPORTED_STAGES}"
        )
    return STAGE_CONTROLLERS[stage]


def stages_for_controller(name: str) -> Tuple[int, ...]:
    """Return the ordered tuple of stages that exercise ``name``.

    Returns ``()`` (empty tuple) for a controller that appears in
    none of the stages — a caller enumerating the full matrix can
    treat this as "skip" without an extra guard. The returned order
    matches :data:`SUPPORTED_STAGES`.
    """
    if not isinstance(name, str):
        _raise_type(f"name must be str, got {type(name).__name__}")
    if not name:
        _raise_value("name must be non-empty")
    return tuple(s for s in SUPPORTED_STAGES if name in STAGE_CONTROLLERS[s])


def all_controllers() -> Tuple[str, ...]:
    """Return the sorted union of every controller across all stages.

    Useful for a caller that wants to iterate once over the full
    vocabulary (e.g. a sanity-check that every declared variant is
    referenced somewhere). Sorted for deterministic output regardless
    of per-stage ordering.
    """
    union: set = set()
    for controllers in STAGE_CONTROLLERS.values():
        union.update(controllers)
    return tuple(sorted(union))


def expectation_key(variant: str) -> str:
    """Translate a catalog *variant* to its expectation-YAML key.

    Most variants map to themselves (the catalog names match the
    ``controllers:`` keys in ``tests/integration/expectations/*.yaml``
    by construction). The exception is the stage-3 JTC+IK shim
    variant, which maps back to ``joint_trajectory_controller`` so a
    caller chaining ``controllers_for_stage(3)`` into
    ``ArmExpectation.controller(expectation_key(v))`` gets a valid
    lookup without a stage-specific if-branch.

    Raises:
        TypeError: ``variant`` is not a ``str``.
        ValueError: ``variant`` is empty or not one of the catalog
            entries.
    """
    if not isinstance(variant, str):
        _raise_type(f"variant must be str, got {type(variant).__name__}")
    if not variant:
        _raise_value("variant must be non-empty")
    if variant not in all_controllers():
        _raise_value(
            f"variant {variant!r} not in catalog; "
            f"available: {all_controllers()}"
        )
    if variant == JTC_IK_SHIM_VARIANT:
        return "joint_trajectory_controller"
    return variant
