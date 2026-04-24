"""Unit tests for :mod:`tests.integration.r2_stage_controllers`.

Covers: export surface, stage membership pinned verbatim to ROADMAP
§M6.R2, `controllers_for_stage` / `stages_for_controller` inverse
consistency, `all_controllers` sorted/unique union, `expectation_key`
translation (identity for most, JTC+IK → plain JTC), MappingProxyType
immutability, and the full rejection matrix (unknown stage, unknown
controller, non-str, non-int, bool, empty, wrong type).
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from types import MappingProxyType, ModuleType

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
INT_DIR = REPO_ROOT / "tests" / "integration"


def _load(name: str, path: Path) -> ModuleType:
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)  # type: ignore[arg-type]
    sys.modules[name] = mod
    assert spec and spec.loader
    spec.loader.exec_module(mod)
    return mod


SC = _load("r2_stage_controllers", INT_DIR / "r2_stage_controllers.py")


# ---------------------------------------------------------------------------
# Export surface
# ---------------------------------------------------------------------------


def test_all_exports_present() -> None:
    expected = {
        "SUPPORTED_STAGES",
        "STAGE_CONTROLLERS",
        "JTC_IK_SHIM_VARIANT",
        "controllers_for_stage",
        "stages_for_controller",
        "all_controllers",
        "expectation_key",
    }
    assert set(SC.__all__) == expected
    for name in expected:
        assert hasattr(SC, name), f"missing export: {name}"


def test_supported_stages_is_ordered_1_2_3() -> None:
    assert SC.SUPPORTED_STAGES == (1, 2, 3)
    assert isinstance(SC.SUPPORTED_STAGES, tuple)


def test_jtc_ik_shim_variant_literal() -> None:
    assert SC.JTC_IK_SHIM_VARIANT == "joint_trajectory_controller_ik_shim"


# ---------------------------------------------------------------------------
# STAGE_CONTROLLERS pinning — ROADMAP §M6.R2
# ---------------------------------------------------------------------------


def test_stage_controllers_is_mapping_proxy() -> None:
    assert isinstance(SC.STAGE_CONTROLLERS, MappingProxyType)


def test_stage_controllers_immutable() -> None:
    with pytest.raises(TypeError):
        SC.STAGE_CONTROLLERS[4] = ("foo",)  # type: ignore[index]


def test_stage_controllers_keys_match_supported_stages() -> None:
    assert tuple(SC.STAGE_CONTROLLERS) == SC.SUPPORTED_STAGES


def test_stage_1_membership_pinned() -> None:
    assert SC.STAGE_CONTROLLERS[1] == (
        "joint_trajectory_controller",
        "forward_position_controller",
        "forward_effort_controller",
        "crisp_joint_impedance",
        "simple_joint_impedance",
    )


def test_stage_1_excludes_forward_velocity() -> None:
    # ROADMAP §M6.12 does not list forward_velocity; expectations YAML
    # declares it but that is broader than stage-1 scope today.
    assert "forward_velocity_controller" not in SC.STAGE_CONTROLLERS[1]


def test_stage_2_superset_of_stage_1_joint_space() -> None:
    stage1 = set(SC.STAGE_CONTROLLERS[1])
    stage2 = set(SC.STAGE_CONTROLLERS[2])
    assert stage1.issubset(stage2)


def test_stage_2_adds_cartesian_motion() -> None:
    stage1 = set(SC.STAGE_CONTROLLERS[1])
    stage2 = set(SC.STAGE_CONTROLLERS[2])
    added = stage2 - stage1
    assert added == {"cartesian_motion_controller"}


def test_stage_2_ordering_preserves_stage_1_prefix() -> None:
    s1 = SC.STAGE_CONTROLLERS[1]
    s2 = SC.STAGE_CONTROLLERS[2]
    assert s2[: len(s1)] == s1
    assert s2[len(s1):] == ("cartesian_motion_controller",)


def test_stage_3_membership_pinned() -> None:
    assert SC.STAGE_CONTROLLERS[3] == (
        "cartesian_motion_controller",
        SC.JTC_IK_SHIM_VARIANT,
        "crisp_cartesian_impedance",
    )


def test_stage_3_uses_ik_shim_variant_not_plain_jtc() -> None:
    # The catalog distinguishes stage-3 JTC+IK from stage-1/2 JTC so
    # the artefact `controller` key and by-controller aggregation stay
    # unambiguous.
    assert "joint_trajectory_controller" not in SC.STAGE_CONTROLLERS[3]
    assert SC.JTC_IK_SHIM_VARIANT in SC.STAGE_CONTROLLERS[3]


def test_each_stage_entries_are_unique() -> None:
    for stage, controllers in SC.STAGE_CONTROLLERS.items():
        assert len(controllers) == len(set(controllers)), (
            f"stage {stage} has duplicate entries: {controllers}"
        )


def test_each_stage_entries_are_non_empty_strs() -> None:
    for stage, controllers in SC.STAGE_CONTROLLERS.items():
        for c in controllers:
            assert isinstance(c, str) and c, (
                f"stage {stage} has invalid entry {c!r}"
            )
            # No path separators / whitespace; pins role-name style.
            assert "/" not in c
            assert c == c.strip()


# ---------------------------------------------------------------------------
# controllers_for_stage
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("stage", (1, 2, 3))
def test_controllers_for_stage_returns_pinned_tuple(stage: int) -> None:
    assert SC.controllers_for_stage(stage) == SC.STAGE_CONTROLLERS[stage]


def test_controllers_for_stage_returns_same_tuple_object() -> None:
    # Callers receive the same tuple object across calls — caching
    # is trivial for pure-data lookups.
    assert SC.controllers_for_stage(1) is SC.STAGE_CONTROLLERS[1]


@pytest.mark.parametrize("bad", (0, 4, -1, 99))
def test_controllers_for_stage_rejects_unknown_int(bad: int) -> None:
    with pytest.raises(ValueError, match="r2_stage_controllers:"):
        SC.controllers_for_stage(bad)


@pytest.mark.parametrize("bad", ("1", 1.0, None, [1], {"stage": 1}))
def test_controllers_for_stage_rejects_non_int(bad) -> None:
    with pytest.raises(TypeError, match="r2_stage_controllers:"):
        SC.controllers_for_stage(bad)


@pytest.mark.parametrize("bad", (True, False))
def test_controllers_for_stage_rejects_bool(bad) -> None:
    with pytest.raises(TypeError, match="r2_stage_controllers:"):
        SC.controllers_for_stage(bad)


# ---------------------------------------------------------------------------
# stages_for_controller
# ---------------------------------------------------------------------------


def test_stages_for_controller_joint_space_in_stage_1_and_2() -> None:
    # The joint-space controllers in stage 1 are also carried through
    # to stage 2 (joint-space branch of the all-joints test), so
    # their stages_for_controller() result is (1, 2).
    for controller in (
        "simple_joint_impedance",
        "crisp_joint_impedance",
        "forward_position_controller",
        "forward_effort_controller",
    ):
        assert SC.stages_for_controller(controller) == (1, 2)


def test_stages_for_controller_stage_2_only() -> None:
    # cartesian_motion_controller is stage 2 AND stage 3; not stage-2-only.
    # No controller is stage-2-exclusive in the current catalog.
    for controller in SC.STAGE_CONTROLLERS[2]:
        if controller == "cartesian_motion_controller":
            continue
        # All other stage-2 entries are stage-1 entries.
        assert 1 in SC.stages_for_controller(controller)


def test_stages_for_controller_jtc_in_stage_1_and_2() -> None:
    assert SC.stages_for_controller("joint_trajectory_controller") == (1, 2)


def test_stages_for_controller_cartesian_motion_in_stage_2_and_3() -> None:
    assert SC.stages_for_controller("cartesian_motion_controller") == (2, 3)


def test_stages_for_controller_stage3_only_ik_shim() -> None:
    assert SC.stages_for_controller(SC.JTC_IK_SHIM_VARIANT) == (3,)
    assert SC.stages_for_controller("crisp_cartesian_impedance") == (3,)


def test_stages_for_controller_unknown_returns_empty() -> None:
    assert SC.stages_for_controller("not_a_controller") == ()
    assert SC.stages_for_controller("gravity_compensation") == ()


def test_stages_for_controller_rejects_non_str() -> None:
    with pytest.raises(TypeError, match="r2_stage_controllers:"):
        SC.stages_for_controller(1)  # type: ignore[arg-type]


def test_stages_for_controller_rejects_empty_str() -> None:
    with pytest.raises(ValueError, match="r2_stage_controllers:"):
        SC.stages_for_controller("")


def test_stages_for_controller_return_order_is_stage_order() -> None:
    # Build a synthetic controller that hypothetically appears in all
    # three — use joint_trajectory_controller style indirectly: just
    # assert cartesian_motion_controller order is (2, 3) not (3, 2).
    assert SC.stages_for_controller("cartesian_motion_controller") == (2, 3)


def test_stages_for_controller_inverse_consistency() -> None:
    # For every (stage, controller) in STAGE_CONTROLLERS, the controller
    # must report `stage` via stages_for_controller. And every stage
    # returned by stages_for_controller must contain the controller.
    for stage, controllers in SC.STAGE_CONTROLLERS.items():
        for controller in controllers:
            stages = SC.stages_for_controller(controller)
            assert stage in stages, (
                f"stages_for_controller({controller!r}) = {stages} "
                f"missing stage {stage}"
            )
    for controller in SC.all_controllers():
        for stage in SC.stages_for_controller(controller):
            assert controller in SC.controllers_for_stage(stage)


# ---------------------------------------------------------------------------
# all_controllers
# ---------------------------------------------------------------------------


def test_all_controllers_is_sorted() -> None:
    result = SC.all_controllers()
    assert list(result) == sorted(result)


def test_all_controllers_has_no_duplicates() -> None:
    result = SC.all_controllers()
    assert len(result) == len(set(result))


def test_all_controllers_union_of_stages() -> None:
    expected = set()
    for c in SC.STAGE_CONTROLLERS.values():
        expected.update(c)
    assert set(SC.all_controllers()) == expected


def test_all_controllers_contains_known_entries() -> None:
    result = set(SC.all_controllers())
    for controller in (
        "joint_trajectory_controller",
        "forward_position_controller",
        "forward_effort_controller",
        "crisp_joint_impedance",
        "simple_joint_impedance",
        "cartesian_motion_controller",
        SC.JTC_IK_SHIM_VARIANT,
        "crisp_cartesian_impedance",
    ):
        assert controller in result


def test_all_controllers_deterministic_across_calls() -> None:
    assert SC.all_controllers() == SC.all_controllers()


# ---------------------------------------------------------------------------
# expectation_key
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "variant",
    (
        "joint_trajectory_controller",
        "forward_position_controller",
        "forward_effort_controller",
        "crisp_joint_impedance",
        "simple_joint_impedance",
        "cartesian_motion_controller",
        "crisp_cartesian_impedance",
    ),
)
def test_expectation_key_is_identity_for_most_variants(variant: str) -> None:
    assert SC.expectation_key(variant) == variant


def test_expectation_key_maps_ik_shim_to_plain_jtc() -> None:
    assert (
        SC.expectation_key(SC.JTC_IK_SHIM_VARIANT)
        == "joint_trajectory_controller"
    )


def test_expectation_key_rejects_non_str() -> None:
    with pytest.raises(TypeError, match="r2_stage_controllers:"):
        SC.expectation_key(1)  # type: ignore[arg-type]


def test_expectation_key_rejects_empty() -> None:
    with pytest.raises(ValueError, match="r2_stage_controllers:"):
        SC.expectation_key("")


def test_expectation_key_rejects_unknown_variant() -> None:
    with pytest.raises(ValueError, match="r2_stage_controllers:"):
        SC.expectation_key("not_in_catalog")


# ---------------------------------------------------------------------------
# Cross-module alignment: stage-1 expectation keys resolve in both arms
# ---------------------------------------------------------------------------


EL = _load("expectations_loader", INT_DIR / "expectations_loader.py")


@pytest.mark.parametrize("arm", ("ur5e", "ur15"))
def test_stage1_controllers_all_resolve_in_expectations_yaml(arm: str) -> None:
    # Every stage-1 catalog entry must have a matching controllers-block
    # row in the expectations YAML for both arms — otherwise a stage-1
    # test body chaining controllers_for_stage(1) into
    # ArmExpectation.controller(name) will hit a missing-key error at
    # run-time.
    expect = EL.load_arm(arm)
    for controller in SC.STAGE_CONTROLLERS[1]:
        # Must not raise.
        c = expect.controller(SC.expectation_key(controller))
        assert c.name == SC.expectation_key(controller)


@pytest.mark.parametrize("arm", ("ur5e", "ur15"))
def test_stage3_ik_shim_expectation_resolves_via_expectation_key(arm: str) -> None:
    expect = EL.load_arm(arm)
    # The catalog variant is distinct from the expectation key; the
    # translation via expectation_key must still land on a real row.
    key = SC.expectation_key(SC.JTC_IK_SHIM_VARIANT)
    assert expect.controller(key).name == "joint_trajectory_controller"
