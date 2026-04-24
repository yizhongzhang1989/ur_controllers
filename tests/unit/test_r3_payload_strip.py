"""Unit tests for :mod:`tests.integration.r3_payload_strip`.

Covers: export surface, idempotence on inputs without the target
body (including malformed fragments), positive-mass strip after a
real splice (round-trip equivalence), strip of a body at deep
nesting, custom ``body_name`` override, the full rejection matrix
(non-str / empty / whitespace ``body_name``, non-str / malformed
``mjcf``), ambiguous-match error, sibling-preservation, and
interop with every in-tree catalog payload.
"""

from __future__ import annotations

import importlib.util
import sys
import xml.etree.ElementTree as ET
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
PV = _load("payload_validation", INT_DIR / "payload_validation.py")
PM = _load("r3_payload_mjcf", INT_DIR / "r3_payload_mjcf.py")
SP = _load("r3_payload_splice", INT_DIR / "r3_payload_splice.py")
ST = _load("r3_payload_strip", INT_DIR / "r3_payload_strip.py")


_GOOD_INERTIA = {"ixx": 0.01, "iyy": 0.02, "izz": 0.03, "ixy": 0.0, "ixz": 0.0, "iyz": 0.0}


def _payload(
    *,
    name: str = "p",
    mass_kg: float = 1.0,
    inertia: dict | None = None,
    pose_xyz: tuple = (0.0, 0.0, 0.0),
    pose_rpy: tuple = (0.0, 0.0, 0.0),
):
    return EL.Payload(
        name=name,
        mass_kg=mass_kg,
        inertia_kg_m2=dict(_GOOD_INERTIA if inertia is None else inertia),
        pose_xyz=pose_xyz,
        pose_rpy=pose_rpy,
    )


_SIMPLE_MJCF = (
    "<mujoco><worldbody>" '<body name="base"><body name="tool0"/></body>' "</worldbody></mujoco>"
)


# -- export surface ---------------------------------------------------------


def test_export_surface():
    assert callable(ST.strip_payload_from_mjcf)
    assert ST.DEFAULT_BODY_NAME == "ee_payload"
    assert set(ST.__all__) == {"strip_payload_from_mjcf", "DEFAULT_BODY_NAME"}


# -- idempotence ------------------------------------------------------------


def test_no_matching_body_returns_input_verbatim():
    out = ST.strip_payload_from_mjcf(_SIMPLE_MJCF)
    assert out == _SIMPLE_MJCF


def test_idempotent_on_malformed_mjcf_when_name_absent():
    # Deliberately malformed; fast path must skip the parse.
    bogus = "<mujoco><unclosed"
    assert ST.strip_payload_from_mjcf(bogus) == bogus


def test_idempotent_on_empty_string():
    assert ST.strip_payload_from_mjcf("") == ""


def test_strip_called_twice_is_idempotent():
    spliced = SP.splice_payload_into_mjcf(_SIMPLE_MJCF, _payload())
    once = ST.strip_payload_from_mjcf(spliced)
    twice = ST.strip_payload_from_mjcf(once)
    assert once == twice


# -- splice ↔ strip round-trip ---------------------------------------------


def _canonical(xml_str: str) -> str:
    # ET.tostring re-serialisation gives us a canonical-enough form
    # to compare trees that differ only by whitespace.
    return ET.tostring(ET.fromstring(xml_str), encoding="unicode")


def test_splice_then_strip_recovers_original_semantic_tree():
    spliced = SP.splice_payload_into_mjcf(_SIMPLE_MJCF, _payload())
    stripped = ST.strip_payload_from_mjcf(spliced)
    assert _canonical(stripped) == _canonical(_SIMPLE_MJCF)


def test_strip_actually_removes_the_body():
    spliced = SP.splice_payload_into_mjcf(_SIMPLE_MJCF, _payload())
    stripped = ST.strip_payload_from_mjcf(spliced)
    root = ET.fromstring(stripped)
    assert list(root.iterfind(".//body[@name='ee_payload']")) == []


def test_strip_preserves_anchor_body():
    spliced = SP.splice_payload_into_mjcf(_SIMPLE_MJCF, _payload())
    stripped = ST.strip_payload_from_mjcf(spliced)
    root = ET.fromstring(stripped)
    anchors = list(root.iterfind(".//body[@name='tool0']"))
    assert len(anchors) == 1


def test_strip_preserves_siblings():
    mjcf = (
        "<mujoco><worldbody>"
        '<body name="base">'
        '<body name="tool0">'
        '<inertial pos="0 0 0" mass="0.5" diaginertia="1 1 1"/>'
        '<geom name="tool_geom" size="0.05"/>'
        "</body>"
        "</body>"
        "</worldbody></mujoco>"
    )
    spliced = SP.splice_payload_into_mjcf(mjcf, _payload())
    stripped = ST.strip_payload_from_mjcf(spliced)
    root = ET.fromstring(stripped)
    tool0 = root.find(".//body[@name='tool0']")
    assert tool0 is not None
    tags = [c.tag for c in tool0]
    assert "inertial" in tags
    assert "geom" in tags
    assert "body" not in tags  # ee_payload gone


def test_strip_with_deeply_nested_anchor():
    mjcf = (
        "<mujoco><worldbody>"
        '<body name="a"><body name="b"><body name="c">'
        '<body name="tool0"/>'
        "</body></body></body>"
        "</worldbody></mujoco>"
    )
    spliced = SP.splice_payload_into_mjcf(mjcf, _payload())
    stripped = ST.strip_payload_from_mjcf(spliced)
    assert _canonical(stripped) == _canonical(mjcf)


# -- catalog interop --------------------------------------------------------


def test_interop_all_catalog_payloads():
    catalog = EL.load_payloads()
    for payload in catalog.payloads:
        spliced = SP.splice_payload_into_mjcf(_SIMPLE_MJCF, payload)
        stripped = ST.strip_payload_from_mjcf(spliced)
        # no_payload is a no-op at splice time, so spliced == input
        # and strip leaves the document semantically identical.
        assert _canonical(stripped) == _canonical(_SIMPLE_MJCF)


def test_interop_zero_mass_payload_noop():
    catalog = EL.load_payloads()
    zero = catalog.get("no_payload")
    spliced = SP.splice_payload_into_mjcf(_SIMPLE_MJCF, zero)
    # Splicer returns byte-identical input for zero-mass.
    assert spliced == _SIMPLE_MJCF
    # Strip also returns byte-identical input (fast path).
    assert ST.strip_payload_from_mjcf(spliced) == _SIMPLE_MJCF


# -- custom body_name -------------------------------------------------------


def test_custom_body_name_round_trip():
    spliced = SP.splice_payload_into_mjcf(_SIMPLE_MJCF, _payload(), body_name="custom_cube")
    # Default body_name must be ignored (no match), returns verbatim.
    assert ST.strip_payload_from_mjcf(spliced) == spliced
    # Explicit body_name strips it.
    out = ST.strip_payload_from_mjcf(spliced, body_name="custom_cube")
    assert _canonical(out) == _canonical(_SIMPLE_MJCF)


def test_custom_body_name_textual_false_positive_is_handled():
    # The string "ee_payload" appears only as a comment / attribute
    # elsewhere, not as a body name. Strip must still return
    # semantically equivalent output (fast path would pass it
    # through verbatim since no match is found; but this goes
    # through parse + zero-match path).
    mjcf = (
        "<mujoco><worldbody>"
        '<body name="base" class="ee_payload_marker"/>'
        "</worldbody></mujoco>"
    )
    out = ST.strip_payload_from_mjcf(mjcf)
    # No real match exists; the function must return verbatim.
    assert out == mjcf


# -- ambiguous match --------------------------------------------------------


def test_ambiguous_match_raises():
    mjcf = (
        "<mujoco><worldbody>"
        '<body name="ee_payload"/>'
        '<body name="tool0"><body name="ee_payload"/></body>'
        "</worldbody></mujoco>"
    )
    with pytest.raises(ValueError, match="ambiguous"):
        ST.strip_payload_from_mjcf(mjcf)


# -- input-validation matrix -----------------------------------------------


@pytest.mark.parametrize("bad", [None, 0, 1.5, b"<x/>", ["x"], {"x": 1}])
def test_non_str_mjcf_rejected(bad):
    with pytest.raises(ValueError, match="mjcf must be str"):
        ST.strip_payload_from_mjcf(bad)  # type: ignore[arg-type]


def test_malformed_mjcf_with_match_raises():
    # Body name appears so the fast path doesn't skip; parse then fails.
    bogus = '<mujoco><body name="ee_payload"></mujoco>'
    with pytest.raises(ValueError, match="not well-formed XML"):
        ST.strip_payload_from_mjcf(bogus)


@pytest.mark.parametrize("bad", [None, 0, 1.5, b"x", ["x"]])
def test_non_str_body_name_rejected(bad):
    with pytest.raises(ValueError, match="body_name must be str"):
        ST.strip_payload_from_mjcf(_SIMPLE_MJCF, body_name=bad)  # type: ignore[arg-type]


def test_empty_body_name_rejected():
    with pytest.raises(ValueError, match="body_name must be non-empty"):
        ST.strip_payload_from_mjcf(_SIMPLE_MJCF, body_name="")


@pytest.mark.parametrize("bad", [" ", "\t", "\n", "with space", "lead\ttab"])
def test_whitespace_body_name_rejected(bad):
    with pytest.raises(ValueError, match="must not contain whitespace"):
        ST.strip_payload_from_mjcf(_SIMPLE_MJCF, body_name=bad)


# -- swap-in-place composition ---------------------------------------------


def test_strip_then_splice_enables_in_place_payload_swap():
    p1 = _payload(name="p1", mass_kg=1.0, pose_xyz=(0.0, 0.0, 0.05))
    p2 = _payload(name="p2", mass_kg=3.0, pose_xyz=(0.0, 0.0, 0.1))

    attached_p1 = SP.splice_payload_into_mjcf(_SIMPLE_MJCF, p1)
    # Directly re-splicing onto attached_p1 would raise (double-splice
    # guard). Strip first to enable an in-place swap.
    cleared = ST.strip_payload_from_mjcf(attached_p1)
    attached_p2 = SP.splice_payload_into_mjcf(cleared, p2)

    root = ET.fromstring(attached_p2)
    bodies = list(root.iterfind(".//body[@name='ee_payload']"))
    assert len(bodies) == 1
    inertial = bodies[0].find("inertial")
    assert inertial is not None
    assert float(inertial.get("mass")) == pytest.approx(3.0)
