"""Unit tests for :mod:`tests.integration.r3_payload_splice`.

Covers: export surface, zero-mass byte-identity, positive-mass
splicing (well-formedness, correct anchor, correct child snippet,
attribute content), custom ``attach_body`` + ``body_name``, the full
rejection matrix (bad inputs, missing anchor, ambiguous anchor,
double-splice guard, malformed MJCF), nested-body anchor discovery,
validator-error propagation, interop with the real in-tree payload
catalog, and append-last ordering.
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


_ZERO_INERTIA = {"ixx": 0.0, "iyy": 0.0, "izz": 0.0, "ixy": 0.0, "ixz": 0.0, "iyz": 0.0}
_GOOD_INERTIA = {"ixx": 0.01, "iyy": 0.02, "izz": 0.03, "ixy": 0.0, "ixz": 0.0, "iyz": 0.0}


def _make(
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


def _zero_payload(name: str = "none"):
    return EL.Payload(
        name=name,
        mass_kg=0.0,
        inertia_kg_m2=dict(_ZERO_INERTIA),
        pose_xyz=(0.0, 0.0, 0.0),
        pose_rpy=(0.0, 0.0, 0.0),
    )


# Minimal MJCF with a nested body hierarchy: worldbody -> base -> tool0.
_MJCF_TOOL0 = (
    '<mujoco model="ur">'
    "<worldbody>"
    '<body name="base" pos="0 0 0">'
    '<body name="tool0" pos="0 0 0.5"/>'
    "</body>"
    "</worldbody>"
    "</mujoco>"
)

_MJCF_NO_TOOL0 = (
    '<mujoco model="ur">' "<worldbody>" '<body name="base" pos="0 0 0"/>' "</worldbody>" "</mujoco>"
)

_MJCF_TWO_TOOL0 = (
    '<mujoco model="ur">'
    "<worldbody>"
    '<body name="tool0" pos="0 0 0.1"/>'
    '<body name="tool0" pos="0 0 0.2"/>'
    "</worldbody>"
    "</mujoco>"
)


# ---------------------------------------------------------------------------
# Export surface
# ---------------------------------------------------------------------------


def test_exports():
    assert set(SP.__all__) == {
        "splice_payload_into_mjcf",
        "DEFAULT_ATTACH_BODY",
        "DEFAULT_BODY_NAME",
    }
    assert callable(SP.splice_payload_into_mjcf)
    assert SP.DEFAULT_ATTACH_BODY == "tool0"
    assert SP.DEFAULT_BODY_NAME == "ee_payload"


# ---------------------------------------------------------------------------
# Zero-mass byte-identity
# ---------------------------------------------------------------------------


def test_zero_mass_returns_input_unchanged():
    p = _zero_payload()
    out = SP.splice_payload_into_mjcf(_MJCF_TOOL0, p)
    assert out is _MJCF_TOOL0 or out == _MJCF_TOOL0


def test_zero_mass_no_tool0_still_passes():
    # Sentinel path bypasses the anchor lookup entirely — an MJCF with
    # no tool0 at all is a valid input when mass==0, because we don't
    # touch the tree.
    p = _zero_payload()
    out = SP.splice_payload_into_mjcf(_MJCF_NO_TOOL0, p)
    assert out == _MJCF_NO_TOOL0


def test_zero_mass_malformed_mjcf_still_passes():
    # Sentinel path must not parse the MJCF.
    p = _zero_payload()
    out = SP.splice_payload_into_mjcf("<not really xml", p)
    assert out == "<not really xml"


def test_zero_mass_ignores_body_name():
    # body_name is validated (must be a non-empty whitespace-free str)
    # but not consulted past that, since we don't emit a snippet.
    p = _zero_payload()
    out = SP.splice_payload_into_mjcf(_MJCF_TOOL0, p, body_name="totally-different-name")
    assert out == _MJCF_TOOL0


# ---------------------------------------------------------------------------
# Positive-mass happy path
# ---------------------------------------------------------------------------


def test_positive_mass_splices_under_tool0():
    p = _make(mass_kg=1.0, pose_xyz=(0.0, 0.0, 0.05))
    out = SP.splice_payload_into_mjcf(_MJCF_TOOL0, p)
    root = ET.fromstring(out)
    tool0 = root.findall(".//body[@name='tool0']")
    assert len(tool0) == 1
    children = list(tool0[0])
    assert len(children) == 1
    child = children[0]
    assert child.tag == "body"
    assert child.attrib["name"] == "ee_payload"


def test_positive_mass_preserves_pose_and_mass():
    p = _make(mass_kg=2.5, pose_xyz=(0.01, -0.02, 0.05))
    out = SP.splice_payload_into_mjcf(_MJCF_TOOL0, p)
    root = ET.fromstring(out)
    child = root.find(".//body[@name='ee_payload']")
    assert child is not None
    pos = [float(x) for x in child.attrib["pos"].split()]
    assert pos == [0.01, -0.02, 0.05]
    inertial = child.find("inertial")
    assert inertial is not None
    assert float(inertial.attrib["mass"]) == 2.5


def test_positive_mass_appends_as_last_child():
    # Anchor body has a pre-existing inertial child; the payload body
    # must come AFTER it.
    mjcf = (
        "<mujoco><worldbody>"
        '<body name="tool0"><inertial pos="0 0 0" mass="0.1" diaginertia="1 1 1"/>'
        "</body>"
        "</worldbody></mujoco>"
    )
    p = _make(mass_kg=1.0)
    out = SP.splice_payload_into_mjcf(mjcf, p)
    root = ET.fromstring(out)
    tool0 = root.find(".//body[@name='tool0']")
    assert tool0 is not None
    tags = [c.tag for c in tool0]
    assert tags == ["inertial", "body"]
    assert tool0[-1].attrib["name"] == "ee_payload"


def test_positive_mass_does_not_touch_siblings_of_anchor():
    mjcf = (
        "<mujoco><worldbody>"
        '<body name="other1"><inertial pos="0 0 0" mass="0.1" diaginertia="1 1 1"/>'
        "</body>"
        '<body name="tool0"/>'
        '<body name="other2"/>'
        "</worldbody></mujoco>"
    )
    p = _make(mass_kg=1.0)
    out = SP.splice_payload_into_mjcf(mjcf, p)
    root = ET.fromstring(out)
    assert root.find(".//body[@name='other1']/inertial") is not None
    assert list(root.find(".//body[@name='other2']")) == []
    tool0_children = list(root.find(".//body[@name='tool0']"))
    assert len(tool0_children) == 1
    assert tool0_children[0].attrib["name"] == "ee_payload"


def test_positive_mass_deeply_nested_anchor():
    mjcf = (
        "<mujoco><worldbody>"
        '<body name="L0"><body name="L1"><body name="L2">'
        '<body name="tool0"/>'
        "</body></body></body>"
        "</worldbody></mujoco>"
    )
    p = _make(mass_kg=1.0)
    out = SP.splice_payload_into_mjcf(mjcf, p)
    root = ET.fromstring(out)
    tool0 = root.find(".//body[@name='tool0']")
    assert tool0 is not None
    assert [c.attrib["name"] for c in tool0] == ["ee_payload"]


def test_positive_mass_custom_attach_body():
    mjcf = '<mujoco><worldbody><body name="custom_tcp"/></worldbody></mujoco>'
    p = _make(mass_kg=1.0)
    out = SP.splice_payload_into_mjcf(mjcf, p, attach_body="custom_tcp")
    root = ET.fromstring(out)
    anchor = root.find(".//body[@name='custom_tcp']")
    assert anchor is not None
    assert [c.attrib["name"] for c in anchor] == ["ee_payload"]


def test_positive_mass_custom_body_name():
    p = _make(mass_kg=1.0)
    out = SP.splice_payload_into_mjcf(_MJCF_TOOL0, p, body_name="gripper_mass")
    root = ET.fromstring(out)
    child = root.find(".//body[@name='gripper_mass']")
    assert child is not None
    # And the default name is absent.
    assert root.find(".//body[@name='ee_payload']") is None


def test_positive_mass_output_is_well_formed():
    p = _make(mass_kg=1.0, pose_rpy=(0.5, -0.25, 1.2))
    out = SP.splice_payload_into_mjcf(_MJCF_TOOL0, p)
    # Must round-trip through the XML parser.
    root = ET.fromstring(out)
    assert root.tag == "mujoco"
    # And the emitter's snippet carries a quat attribute.
    child = root.find(".//body[@name='ee_payload']")
    assert child is not None
    quat = child.attrib["quat"].split()
    assert len(quat) == 4
    # Quaternion is unit-norm.
    q = [float(x) for x in quat]
    n2 = sum(v * v for v in q)
    assert abs(n2 - 1.0) < 1e-12


# ---------------------------------------------------------------------------
# In-tree payload catalog interop
# ---------------------------------------------------------------------------


def _in_tree_catalog():
    # expectations_loader.load_payloads() consults the in-tree yaml.
    return EL.load_payloads()


def test_no_payload_from_catalog_is_no_op():
    catalog = _in_tree_catalog()
    none = catalog.get("no_payload")
    out = SP.splice_payload_into_mjcf(_MJCF_TOOL0, none)
    assert out == _MJCF_TOOL0


def test_small_payload_from_catalog_splices():
    catalog = _in_tree_catalog()
    small = catalog.get("small_payload")
    out = SP.splice_payload_into_mjcf(_MJCF_TOOL0, small)
    root = ET.fromstring(out)
    child = root.find(".//body[@name='ee_payload']")
    assert child is not None
    inertial = child.find("inertial")
    assert inertial is not None
    assert float(inertial.attrib["mass"]) == small.mass_kg


def test_large_payload_from_catalog_splices():
    catalog = _in_tree_catalog()
    large = catalog.get("large_payload")
    out = SP.splice_payload_into_mjcf(_MJCF_TOOL0, large)
    root = ET.fromstring(out)
    child = root.find(".//body[@name='ee_payload']")
    assert child is not None
    pos = [float(x) for x in child.attrib["pos"].split()]
    assert pos == list(large.pose_xyz)


# ---------------------------------------------------------------------------
# Rejection matrix: mjcf / attach_body / body_name / structural errors
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "bad",
    [None, 0, 0.0, b"<mujoco/>", ["<mujoco/>"]],
)
def test_rejects_non_str_mjcf(bad):
    with pytest.raises(ValueError, match="mjcf must be str"):
        SP.splice_payload_into_mjcf(bad, _make(mass_kg=1.0))  # type: ignore[arg-type]


def test_rejects_malformed_mjcf_on_positive_mass():
    with pytest.raises(ValueError, match="not well-formed XML"):
        SP.splice_payload_into_mjcf("<not closed", _make(mass_kg=1.0))


@pytest.mark.parametrize(
    "field,bad,msg",
    [
        ("attach_body", "", "attach_body must be non-empty"),
        ("attach_body", " ", "attach_body must not contain whitespace"),
        ("attach_body", "a\tb", "attach_body must not contain whitespace"),
        ("attach_body", "a\nb", "attach_body must not contain whitespace"),
        ("body_name", "", "body_name must be non-empty"),
        ("body_name", " ", "body_name must not contain whitespace"),
        ("body_name", "x y", "body_name must not contain whitespace"),
    ],
)
def test_rejects_bad_identifier_strings(field, bad, msg):
    kwargs = {field: bad}
    with pytest.raises(ValueError, match=msg):
        SP.splice_payload_into_mjcf(_MJCF_TOOL0, _make(mass_kg=1.0), **kwargs)


@pytest.mark.parametrize(
    "field,bad",
    [
        ("attach_body", None),
        ("attach_body", 0),
        ("attach_body", b"tool0"),
        ("body_name", None),
        ("body_name", 0),
        ("body_name", b"ee_payload"),
    ],
)
def test_rejects_non_str_identifier(field, bad):
    kwargs = {field: bad}
    with pytest.raises(ValueError, match=f"{field} must be str"):
        SP.splice_payload_into_mjcf(_MJCF_TOOL0, _make(mass_kg=1.0), **kwargs)


def test_rejects_missing_anchor():
    with pytest.raises(ValueError, match="not found in MJCF"):
        SP.splice_payload_into_mjcf(_MJCF_NO_TOOL0, _make(mass_kg=1.0))


def test_rejects_ambiguous_anchor():
    with pytest.raises(ValueError, match="ambiguous"):
        SP.splice_payload_into_mjcf(_MJCF_TWO_TOOL0, _make(mass_kg=1.0))


def test_rejects_existing_body_name_as_sibling():
    mjcf = (
        "<mujoco><worldbody>"
        '<body name="tool0"/>'
        '<body name="ee_payload"/>'
        "</worldbody></mujoco>"
    )
    with pytest.raises(ValueError, match="already present"):
        SP.splice_payload_into_mjcf(mjcf, _make(mass_kg=1.0))


def test_rejects_existing_body_name_under_anchor():
    # Double-splice detection: previous splice's body is already there.
    mjcf = (
        "<mujoco><worldbody>"
        '<body name="tool0"><body name="ee_payload"/></body>'
        "</worldbody></mujoco>"
    )
    with pytest.raises(ValueError, match="already present"):
        SP.splice_payload_into_mjcf(mjcf, _make(mass_kg=1.0))


def test_double_splice_second_call_rejected():
    p = _make(mass_kg=1.0)
    once = SP.splice_payload_into_mjcf(_MJCF_TOOL0, p)
    with pytest.raises(ValueError, match="already present"):
        SP.splice_payload_into_mjcf(once, p)


def test_validator_error_propagates():
    # Negative mass is rejected by payload_validation; error must
    # surface unchanged.
    bad = _make(mass_kg=-1.0)
    with pytest.raises(ValueError, match="mass"):
        SP.splice_payload_into_mjcf(_MJCF_TOOL0, bad)


def test_validator_error_propagates_non_diagonal():
    bad_inertia = dict(_GOOD_INERTIA)
    bad_inertia["ixy"] = 0.001
    bad = _make(mass_kg=1.0, inertia=bad_inertia)
    with pytest.raises(ValueError):
        SP.splice_payload_into_mjcf(_MJCF_TOOL0, bad)


# ---------------------------------------------------------------------------
# Custom attach_body still fires the "ambiguous / missing" guards
# ---------------------------------------------------------------------------


def test_custom_attach_body_missing():
    with pytest.raises(ValueError, match="'gripper_flange'"):
        SP.splice_payload_into_mjcf(_MJCF_TOOL0, _make(mass_kg=1.0), attach_body="gripper_flange")


def test_custom_attach_body_ambiguous():
    mjcf = (
        "<mujoco><worldbody>"
        '<body name="flange"/>'
        '<body name="flange"/>'
        "</worldbody></mujoco>"
    )
    with pytest.raises(ValueError, match="ambiguous"):
        SP.splice_payload_into_mjcf(mjcf, _make(mass_kg=1.0), attach_body="flange")
