"""Unit tests for :mod:`tests.integration.r3_payload_parametrize`.

Covers: export surface, default Cartesian-product enumeration against
the live ``SUPPORTED_ARMS`` and catalog, caller-supplied filtering
(both arms and payloads), order preservation, id shape
``"<arm>-<payload>"``, id / combination parallelism, and the full
rejection matrix (unknown / non-str / duplicate / empty / wrong
container / malformed id components).
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from types import ModuleType

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


EL = _load("expectations_loader", INT_DIR / "expectations_loader.py")
PP = _load("r3_payload_parametrize", INT_DIR / "r3_payload_parametrize.py")


# ---------------------------------------------------------------------------
# Export surface
# ---------------------------------------------------------------------------


def test_module_has_all_and_symbols():
    assert set(PP.__all__) == {"arm_payload_combinations", "arm_payload_ids"}
    for name in PP.__all__:
        assert callable(getattr(PP, name))


# ---------------------------------------------------------------------------
# arm_payload_combinations: defaults
# ---------------------------------------------------------------------------


def test_default_product_matches_supported_arms_cross_catalog():
    combos = PP.arm_payload_combinations()
    catalog = EL.load_payloads()
    expected = tuple((a, p) for a in EL.SUPPORTED_ARMS for p in catalog.names())
    assert combos == expected


def test_default_product_includes_all_three_payloads_per_arm():
    combos = PP.arm_payload_combinations()
    catalog = EL.load_payloads()
    for arm in EL.SUPPORTED_ARMS:
        arm_pairs = [p for (a, p) in combos if a == arm]
        assert tuple(arm_pairs) == catalog.names()


def test_default_is_ordered_arm_major():
    """Outer index = arm, inner index = payload."""
    combos = PP.arm_payload_combinations()
    catalog = EL.load_payloads()
    n = len(catalog.names())
    assert combos[0][0] == EL.SUPPORTED_ARMS[0]
    assert combos[n - 1][0] == EL.SUPPORTED_ARMS[0]
    assert combos[n][0] == EL.SUPPORTED_ARMS[1]


def test_default_product_returns_tuple_of_tuples():
    combos = PP.arm_payload_combinations()
    assert isinstance(combos, tuple)
    for c in combos:
        assert isinstance(c, tuple) and len(c) == 2
        assert all(isinstance(x, str) for x in c)


# ---------------------------------------------------------------------------
# arm_payload_combinations: filtering
# ---------------------------------------------------------------------------


def test_filter_arms_subset_preserves_caller_order():
    combos = PP.arm_payload_combinations(arms=["ur15", "ur5e"])
    catalog = EL.load_payloads()
    expected = tuple((a, p) for a in ("ur15", "ur5e") for p in catalog.names())
    assert combos == expected


def test_filter_payloads_subset_preserves_caller_order():
    combos = PP.arm_payload_combinations(
        payload_names=["large_payload", "no_payload"],
    )
    expected = tuple((a, p) for a in EL.SUPPORTED_ARMS for p in ("large_payload", "no_payload"))
    assert combos == expected


def test_filter_both_dimensions():
    combos = PP.arm_payload_combinations(
        arms=["ur5e"], payload_names=["no_payload", "small_payload"]
    )
    assert combos == (("ur5e", "no_payload"), ("ur5e", "small_payload"))


def test_single_arm_single_payload():
    assert PP.arm_payload_combinations(arms=["ur5e"], payload_names=["no_payload"]) == (
        ("ur5e", "no_payload"),
    )


def test_accepts_tuple_inputs():
    combos = PP.arm_payload_combinations(
        arms=("ur5e",), payload_names=("no_payload", "small_payload")
    )
    assert combos == (
        ("ur5e", "no_payload"),
        ("ur5e", "small_payload"),
    )


# ---------------------------------------------------------------------------
# arm_payload_combinations: rejection matrix
# ---------------------------------------------------------------------------


def test_rejects_unknown_arm():
    with pytest.raises(ValueError, match="unknown arm"):
        PP.arm_payload_combinations(arms=["ur5e", "ur3"])


def test_rejects_unknown_payload():
    with pytest.raises(ValueError, match="unknown payload"):
        PP.arm_payload_combinations(payload_names=["no_payload", "mystery_brick"])


def test_rejects_empty_arms():
    with pytest.raises(ValueError, match="must be non-empty"):
        PP.arm_payload_combinations(arms=[])


def test_rejects_empty_payloads():
    with pytest.raises(ValueError, match="must be non-empty"):
        PP.arm_payload_combinations(payload_names=[])


def test_rejects_duplicate_arm():
    with pytest.raises(ValueError, match="duplicate entry"):
        PP.arm_payload_combinations(arms=["ur5e", "ur5e"])


def test_rejects_duplicate_payload():
    with pytest.raises(ValueError, match="duplicate entry"):
        PP.arm_payload_combinations(payload_names=["no_payload", "no_payload"])


@pytest.mark.parametrize("bad", [123, 1.5, "ur5e", object()])
def test_rejects_non_sequence_arms(bad):
    with pytest.raises(ValueError, match="must be a list or tuple"):
        PP.arm_payload_combinations(arms=bad)


@pytest.mark.parametrize("bad", [123, "no_payload", object()])
def test_rejects_non_sequence_payloads(bad):
    with pytest.raises(ValueError, match="must be a list or tuple"):
        PP.arm_payload_combinations(payload_names=bad)


def test_rejects_non_str_arm_element():
    with pytest.raises(ValueError, match="must be str"):
        PP.arm_payload_combinations(arms=["ur5e", 42])


def test_rejects_non_str_payload_element():
    with pytest.raises(ValueError, match="must be str"):
        PP.arm_payload_combinations(payload_names=["no_payload", None])


# ---------------------------------------------------------------------------
# arm_payload_ids
# ---------------------------------------------------------------------------


def test_ids_default_shape_and_parallelism():
    combos = PP.arm_payload_combinations()
    ids = PP.arm_payload_ids(combos)
    assert isinstance(ids, tuple)
    assert len(ids) == len(combos)
    for (arm, payload), ident in zip(combos, ids):
        assert ident == f"{arm}-{payload}"


def test_ids_example_values():
    combos = (("ur5e", "no_payload"), ("ur15", "large_payload"))
    assert PP.arm_payload_ids(combos) == ("ur5e-no_payload", "ur15-large_payload")


def test_ids_preserve_input_order():
    combos = (("ur15", "small_payload"), ("ur5e", "no_payload"))
    assert PP.arm_payload_ids(combos) == ("ur15-small_payload", "ur5e-no_payload")


def test_ids_rejects_empty_sequence():
    with pytest.raises(ValueError, match="must be non-empty"):
        PP.arm_payload_ids([])


@pytest.mark.parametrize("bad", [None, 0, "ur5e-no_payload", object()])
def test_ids_rejects_non_sequence(bad):
    with pytest.raises(ValueError, match="must be a list or tuple"):
        PP.arm_payload_ids(bad)


def test_ids_rejects_non_tuple_element():
    with pytest.raises(ValueError, match="must be a 2-tuple"):
        PP.arm_payload_ids([["ur5e", "no_payload"]])


def test_ids_rejects_wrong_tuple_arity():
    with pytest.raises(ValueError, match="must be a 2-tuple"):
        PP.arm_payload_ids([("ur5e", "no_payload", "extra")])


def test_ids_rejects_non_str_component():
    with pytest.raises(ValueError, match="arm must be str"):
        PP.arm_payload_ids([(1, "no_payload")])
    with pytest.raises(ValueError, match="payload must be str"):
        PP.arm_payload_ids([("ur5e", None)])


def test_ids_rejects_empty_component():
    with pytest.raises(ValueError, match="arm must be non-empty"):
        PP.arm_payload_ids([("", "no_payload")])
    with pytest.raises(ValueError, match="payload must be non-empty"):
        PP.arm_payload_ids([("ur5e", "")])


def test_ids_rejects_dash_in_component():
    with pytest.raises(ValueError, match="must not contain '-'"):
        PP.arm_payload_ids([("ur-5e", "no_payload")])
    with pytest.raises(ValueError, match="must not contain '-'"):
        PP.arm_payload_ids([("ur5e", "no-payload")])


def test_ids_rejects_duplicate_result():
    with pytest.raises(ValueError, match="duplicate id"):
        PP.arm_payload_ids([("ur5e", "no_payload"), ("ur5e", "no_payload")])


# ---------------------------------------------------------------------------
# Interop: default combos feed cleanly into default ids
# ---------------------------------------------------------------------------


def test_default_interop_all_ids_unique_and_well_formed():
    combos = PP.arm_payload_combinations()
    ids = PP.arm_payload_ids(combos)
    assert len(set(ids)) == len(ids)
    for ident in ids:
        assert "-" in ident
        arm, payload = ident.split("-", 1)
        assert arm in EL.SUPPORTED_ARMS
        assert payload in EL.load_payloads().names()
