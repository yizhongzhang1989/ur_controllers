"""Unit tests for :mod:`tests.integration.r3_payload_mjcf`.

Covers: export surface, zero-mass sentinel, positive-mass attribute
content, custom ``body_name``, quat conversion for canonical rotations
(X/Y/Z separately plus a composed one), XML well-formedness, and the
rejection matrix for ``body_name`` + propagation of validator errors.
"""

from __future__ import annotations

import importlib.util
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from types import ModuleType

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
INT_DIR = REPO_ROOT / "tests" / "integration"


def _load(name: str, path: Path) -> ModuleType:
    # tests/integration/ is not a package on sys.path; load siblings
    # by file path so the emitter's `from expectations_loader ...`
    # resolves against the same module object the tests use.
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)  # type: ignore[arg-type]
    sys.modules[name] = mod
    assert spec and spec.loader
    spec.loader.exec_module(mod)
    return mod


EL = _load("expectations_loader", INT_DIR / "expectations_loader.py")
PV = _load("payload_validation", INT_DIR / "payload_validation.py")
PM = _load("r3_payload_mjcf", INT_DIR / "r3_payload_mjcf.py")


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


# ---------------------------------------------------------------------------
# Export surface
# ---------------------------------------------------------------------------


def test_exports():
    assert set(PM.__all__) == {"payload_body_mjcf", "DEFAULT_BODY_NAME"}
    assert callable(PM.payload_body_mjcf)
    assert PM.DEFAULT_BODY_NAME == "ee_payload"


# ---------------------------------------------------------------------------
# Zero-mass sentinel
# ---------------------------------------------------------------------------


def test_zero_mass_emits_empty_string():
    p = _make(mass_kg=0.0, inertia=_ZERO_INERTIA)
    assert PM.payload_body_mjcf(p) == ""


def test_zero_mass_ignores_body_name():
    p = _make(mass_kg=0.0, inertia=_ZERO_INERTIA)
    assert PM.payload_body_mjcf(p, body_name="anything") == ""


def test_builtin_no_payload_emits_empty():
    cat = EL.load_payloads()
    assert PM.payload_body_mjcf(cat.get("no_payload")) == ""


# ---------------------------------------------------------------------------
# Positive-mass happy path
# ---------------------------------------------------------------------------


def test_positive_mass_emits_well_formed_xml():
    p = _make(mass_kg=2.0, inertia=_GOOD_INERTIA, pose_xyz=(0.1, -0.2, 0.3))
    snippet = PM.payload_body_mjcf(p)
    root = ET.fromstring(snippet)
    assert root.tag == "body"
    assert root.attrib["name"] == "ee_payload"
    assert root.attrib["pos"] == "0.1 -0.2 0.3"
    # Identity rpy -> quat (1, 0, 0, 0).
    assert root.attrib["quat"] == "1 0 0 0"

    children = list(root)
    assert len(children) == 1
    inertial = children[0]
    assert inertial.tag == "inertial"
    assert inertial.attrib["pos"] == "0 0 0"
    assert inertial.attrib["mass"] == "2"
    assert inertial.attrib["diaginertia"] == "0.01 0.02 0.03"


def test_custom_body_name():
    p = _make(mass_kg=1.0)
    out = PM.payload_body_mjcf(p, body_name="gripper_tool")
    root = ET.fromstring(out)
    assert root.attrib["name"] == "gripper_tool"


def test_builtin_small_payload_round_trips():
    cat = EL.load_payloads()
    snippet = PM.payload_body_mjcf(cat.get("small_payload"))
    root = ET.fromstring(snippet)
    inertial = root.find("inertial")
    assert inertial is not None
    assert float(inertial.attrib["mass"]) == pytest.approx(1.0)
    diag = [float(v) for v in inertial.attrib["diaginertia"].split()]
    assert diag == pytest.approx([0.00167, 0.00167, 0.00167])
    pos = [float(v) for v in root.attrib["pos"].split()]
    assert pos == pytest.approx([0.0, 0.0, 0.05])
    assert root.attrib["quat"] == "1 0 0 0"


def test_builtin_large_payload_round_trips():
    cat = EL.load_payloads()
    snippet = PM.payload_body_mjcf(cat.get("large_payload"))
    root = ET.fromstring(snippet)
    inertial = root.find("inertial")
    assert inertial is not None
    assert float(inertial.attrib["mass"]) == pytest.approx(5.0)
    diag = [float(v) for v in inertial.attrib["diaginertia"].split()]
    assert diag == pytest.approx([0.0244, 0.0244, 0.0244])
    pos = [float(v) for v in root.attrib["pos"].split()]
    assert pos == pytest.approx([0.0, 0.0, 0.1])


# ---------------------------------------------------------------------------
# Quaternion conversion — canonical rotations
# ---------------------------------------------------------------------------


def _quat_of(snippet: str) -> tuple:
    root = ET.fromstring(snippet)
    return tuple(float(v) for v in root.attrib["quat"].split())


def test_quat_identity():
    p = _make(pose_rpy=(0.0, 0.0, 0.0))
    assert _quat_of(PM.payload_body_mjcf(p)) == (1.0, 0.0, 0.0, 0.0)


def test_quat_rotation_about_x():
    # Rx(pi/2) -> w = cos(pi/4), x = sin(pi/4).
    c = math.cos(math.pi / 4)
    p = _make(pose_rpy=(math.pi / 2, 0.0, 0.0))
    q = _quat_of(PM.payload_body_mjcf(p))
    assert q == pytest.approx((c, c, 0.0, 0.0), abs=1e-12)


def test_quat_rotation_about_y():
    c = math.cos(math.pi / 4)
    p = _make(pose_rpy=(0.0, math.pi / 2, 0.0))
    q = _quat_of(PM.payload_body_mjcf(p))
    assert q == pytest.approx((c, 0.0, c, 0.0), abs=1e-12)


def test_quat_rotation_about_z():
    c = math.cos(math.pi / 4)
    p = _make(pose_rpy=(0.0, 0.0, math.pi / 2))
    q = _quat_of(PM.payload_body_mjcf(p))
    assert q == pytest.approx((c, 0.0, 0.0, c), abs=1e-12)


def test_quat_is_unit_norm():
    # Arbitrary non-canonical rotation; sanity-check that the emitter
    # does not produce a non-unit quaternion.
    p = _make(pose_rpy=(0.3, -0.7, 1.2))
    q = _quat_of(PM.payload_body_mjcf(p))
    assert sum(x * x for x in q) == pytest.approx(1.0, abs=1e-12)


def test_quat_composed_xyz_intrinsic():
    # r=pi/2, p=pi/2, y=0 under intrinsic XYZ:
    # q = qx(pi/2) ⊗ qy(pi/2). Closed-form:
    #   qx = (c, c, 0, 0); qy = (c, 0, c, 0); c = sqrt(2)/2.
    #   q_w = c*c - 0 = 0.5
    #   q_x = c*c + 0 = 0.5
    #   q_y = c*c - 0 = 0.5
    #   q_z = 0   + c*c = 0.5
    p = _make(pose_rpy=(math.pi / 2, math.pi / 2, 0.0))
    q = _quat_of(PM.payload_body_mjcf(p))
    assert q == pytest.approx((0.5, 0.5, 0.5, 0.5), abs=1e-12)


def test_quat_full_composition_xyz_intrinsic():
    # r=pi/2, p=pi/2, y=pi/2. Closed-form check:
    # q = qx(pi/2) ⊗ qy(pi/2) ⊗ qz(pi/2).
    # Let c = cos(pi/4), s = sin(pi/4) = c = sqrt(2)/2.
    #   w = cr*cp*cy - sr*sp*sy = c^3 - c^3 = 0
    #   x = sr*cp*cy + cr*sp*sy = c^3 + c^3 = 2c^3 = sqrt(2)/2
    #   y = cr*sp*cy - sr*cp*sy = c^3 - c^3 = 0
    #   z = cr*cp*sy + sr*sp*cy = c^3 + c^3 = sqrt(2)/2
    p = _make(pose_rpy=(math.pi / 2, math.pi / 2, math.pi / 2))
    q = _quat_of(PM.payload_body_mjcf(p))
    s = math.sqrt(2.0) / 2.0
    assert q == pytest.approx((0.0, s, 0.0, s), abs=1e-12)


# ---------------------------------------------------------------------------
# Non-zero pose_xyz
# ---------------------------------------------------------------------------


def test_pose_xyz_negative_components():
    p = _make(pose_xyz=(-0.1, -0.2, -0.3))
    snippet = PM.payload_body_mjcf(p)
    root = ET.fromstring(snippet)
    assert root.attrib["pos"] == "-0.1 -0.2 -0.3"


def test_pose_xyz_high_precision():
    p = _make(pose_xyz=(1.2345678901234567, 0.0, 0.0))
    snippet = PM.payload_body_mjcf(p)
    root = ET.fromstring(snippet)
    x = float(root.attrib["pos"].split()[0])
    # 15 significant digits round-trip to within 1e-14 relative.
    assert x == pytest.approx(1.2345678901234567, abs=1e-14)


# ---------------------------------------------------------------------------
# body_name validation
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("bad", ["", " ", "has space", "tab\tname", "newline\nname"])
def test_body_name_whitespace_or_empty_rejected(bad):
    p = _make()
    with pytest.raises(ValueError, match="body_name"):
        PM.payload_body_mjcf(p, body_name=bad)


@pytest.mark.parametrize("bad", [None, 42, 3.14, b"bytes", ["list"]])
def test_body_name_wrong_type_rejected(bad):
    p = _make()
    with pytest.raises(ValueError, match="body_name must be str"):
        PM.payload_body_mjcf(p, body_name=bad)


# ---------------------------------------------------------------------------
# Validator propagation
# ---------------------------------------------------------------------------


def test_negative_mass_rejected_by_validator():
    p = _make(mass_kg=-1.0)
    with pytest.raises(ValueError):
        PM.payload_body_mjcf(p)


def test_non_diagonal_inertia_rejected_by_validator():
    bad = dict(_GOOD_INERTIA, ixy=0.001)
    p = _make(inertia=bad)
    with pytest.raises(ValueError):
        PM.payload_body_mjcf(p)


def test_nan_pose_rejected_by_validator():
    p = _make(pose_xyz=(float("nan"), 0.0, 0.0))
    with pytest.raises(ValueError):
        PM.payload_body_mjcf(p)


def test_zero_mass_with_nonzero_inertia_rejected():
    # Validator pins mass==0 <=> inertia==0; this exercises that the
    # emitter's zero-mass short-circuit does not silently accept a
    # malformed payload.
    p = _make(mass_kg=0.0, inertia=_GOOD_INERTIA)
    with pytest.raises(ValueError):
        PM.payload_body_mjcf(p)


# ---------------------------------------------------------------------------
# Snippet shape (sanity: parseable, no whitespace surprises)
# ---------------------------------------------------------------------------


def test_snippet_is_single_line_no_leading_trailing_ws():
    p = _make(mass_kg=1.5, pose_xyz=(0.0, 0.0, 0.1))
    s = PM.payload_body_mjcf(p)
    assert s == s.strip()
    assert "\n" not in s
    # ET.fromstring would already fail if this were malformed XML, but
    # an explicit parse keeps the invariant visible here.
    ET.fromstring(s)


def test_inertial_child_exactly_once():
    p = _make(mass_kg=1.0)
    root = ET.fromstring(PM.payload_body_mjcf(p))
    assert [c.tag for c in root] == ["inertial"]


def test_emitted_names_match_default_constant():
    p = _make(mass_kg=1.0)
    root = ET.fromstring(PM.payload_body_mjcf(p))
    assert root.attrib["name"] == PM.DEFAULT_BODY_NAME
