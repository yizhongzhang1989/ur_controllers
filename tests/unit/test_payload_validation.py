"""Unit tests for :mod:`tests.integration.payload_validation`.

Happy-path: the three built-in payloads from ``payloads.yaml`` must
all validate. Everything else is an explicit rejection matrix:
finiteness, sign, sentinel consistency, diagonal-only, triangle
inequality, pose finiteness, and catalog-level uniqueness. NaN and
+inf / -inf are covered separately per rubber-duck guidance.
"""

from __future__ import annotations

import importlib.util
import math
import sys
from dataclasses import replace
from pathlib import Path
from types import ModuleType

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
INT_DIR = REPO_ROOT / "tests" / "integration"


def _load(name: str, path: Path) -> ModuleType:
    # tests/integration/ is not a package on sys.path; load siblings by
    # file path so payload_validation's `from expectations_loader
    # import ...` resolves against the same module object the tests
    # use.
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)  # type: ignore[arg-type]
    sys.modules[name] = mod
    assert spec and spec.loader
    spec.loader.exec_module(mod)
    return mod


EL = _load("expectations_loader", INT_DIR / "expectations_loader.py")
PV = _load("payload_validation", INT_DIR / "payload_validation.py")


_ZERO_INERTIA = {"ixx": 0.0, "iyy": 0.0, "izz": 0.0, "ixy": 0.0, "ixz": 0.0, "iyz": 0.0}
_GOOD_INERTIA = {"ixx": 0.01, "iyy": 0.01, "izz": 0.01, "ixy": 0.0, "ixz": 0.0, "iyz": 0.0}


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
    assert set(PV.__all__) == {"validate_payload", "validate_catalog"}
    assert callable(PV.validate_payload)
    assert callable(PV.validate_catalog)


# ---------------------------------------------------------------------------
# Happy path — in-tree catalog
# ---------------------------------------------------------------------------


def test_builtin_catalog_validates():
    cat = EL.load_payloads()
    PV.validate_catalog(cat)  # must not raise
    for p in cat.payloads:
        PV.validate_payload(p)


def test_builtin_catalog_has_three_levels():
    cat = EL.load_payloads()
    assert cat.names() == ("no_payload", "small_payload", "large_payload")


# ---------------------------------------------------------------------------
# Happy path — synthetic
# ---------------------------------------------------------------------------


def test_zero_mass_zero_inertia_ok():
    PV.validate_payload(_make(mass_kg=0.0, inertia=_ZERO_INERTIA))


def test_positive_mass_diagonal_inertia_ok():
    PV.validate_payload(
        _make(
            mass_kg=2.5,
            inertia={"ixx": 0.1, "iyy": 0.2, "izz": 0.15, "ixy": 0.0, "ixz": 0.0, "iyz": 0.0},
        )
    )


def test_triangle_inequality_boundary_ok():
    # Equality case: ixx + iyy == izz is allowed (thin-rod limit).
    PV.validate_payload(
        _make(
            mass_kg=1.0,
            inertia={"ixx": 0.1, "iyy": 0.1, "izz": 0.2, "ixy": 0.0, "ixz": 0.0, "iyz": 0.0},
        )
    )


def test_pose_nonzero_ok():
    PV.validate_payload(_make(pose_xyz=(0.1, -0.2, 0.3), pose_rpy=(0.1, 0.2, -0.3)))


# ---------------------------------------------------------------------------
# Rejection matrix: name
# ---------------------------------------------------------------------------


def test_empty_name_rejected():
    with pytest.raises(ValueError, match="payload.name"):
        PV.validate_payload(_make(name=""))


# ---------------------------------------------------------------------------
# Rejection matrix: mass
# ---------------------------------------------------------------------------


def test_negative_mass_rejected():
    with pytest.raises(ValueError, match="mass_kg.*>= 0"):
        PV.validate_payload(_make(mass_kg=-0.1))


def test_nan_mass_rejected():
    with pytest.raises(ValueError, match="mass_kg.*finite"):
        PV.validate_payload(_make(mass_kg=float("nan")))


def test_posinf_mass_rejected():
    with pytest.raises(ValueError, match="mass_kg.*finite"):
        PV.validate_payload(_make(mass_kg=float("inf")))


def test_neginf_mass_rejected():
    with pytest.raises(ValueError, match="mass_kg.*finite"):
        PV.validate_payload(_make(mass_kg=-math.inf))


# ---------------------------------------------------------------------------
# Rejection matrix: inertia — missing / non-finite
# ---------------------------------------------------------------------------


def test_missing_inertia_key_rejected():
    bad = dict(_GOOD_INERTIA)
    del bad["iyy"]
    with pytest.raises(ValueError, match="missing key 'iyy'"):
        PV.validate_payload(_make(inertia=bad))


def test_nan_diagonal_inertia_rejected():
    bad = dict(_GOOD_INERTIA, ixx=float("nan"))
    with pytest.raises(ValueError, match="ixx.*finite"):
        PV.validate_payload(_make(inertia=bad))


def test_inf_diagonal_inertia_rejected():
    bad = dict(_GOOD_INERTIA, izz=math.inf)
    with pytest.raises(ValueError, match="izz.*finite"):
        PV.validate_payload(_make(inertia=bad))


def test_nan_offdiagonal_inertia_rejected():
    bad = dict(_GOOD_INERTIA, ixy=float("nan"))
    with pytest.raises(ValueError, match="ixy.*finite"):
        PV.validate_payload(_make(inertia=bad))


def test_inf_offdiagonal_inertia_rejected():
    bad = dict(_GOOD_INERTIA, ixz=-math.inf)
    with pytest.raises(ValueError, match="ixz.*finite"):
        PV.validate_payload(_make(inertia=bad))


# ---------------------------------------------------------------------------
# Rejection matrix: diagonal-only v1 constraint
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("key", ["ixy", "ixz", "iyz"])
def test_nonzero_offdiagonal_rejected(key):
    bad = dict(_GOOD_INERTIA)
    bad[key] = 1e-6
    with pytest.raises(ValueError, match=rf"{key}.*diagonal"):
        PV.validate_payload(_make(inertia=bad))


def test_negative_offdiagonal_rejected():
    bad = dict(_GOOD_INERTIA, iyz=-1e-9)
    with pytest.raises(ValueError, match="iyz.*diagonal"):
        PV.validate_payload(_make(inertia=bad))


# ---------------------------------------------------------------------------
# Rejection matrix: sentinel consistency & v1 point-mass ban
# ---------------------------------------------------------------------------


def test_zero_mass_nonzero_inertia_rejected():
    bad = dict(_ZERO_INERTIA, ixx=1e-6)
    with pytest.raises(ValueError, match="zero mass.*zero inertia"):
        PV.validate_payload(_make(mass_kg=0.0, inertia=bad))


@pytest.mark.parametrize("key", ["ixx", "iyy", "izz"])
def test_positive_mass_zero_principal_moment_rejected(key):
    bad = dict(_GOOD_INERTIA)
    bad[key] = 0.0
    with pytest.raises(ValueError, match=rf"{key}.*> 0"):
        PV.validate_payload(_make(mass_kg=1.0, inertia=bad))


def test_negative_principal_moment_rejected():
    bad = dict(_GOOD_INERTIA, iyy=-0.01)
    with pytest.raises(ValueError, match=r"iyy.*> 0"):
        PV.validate_payload(_make(mass_kg=1.0, inertia=bad))


# ---------------------------------------------------------------------------
# Rejection matrix: triangle inequality
# ---------------------------------------------------------------------------


def test_triangle_inequality_izz_violation_rejected():
    bad = {"ixx": 1.0, "iyy": 1.0, "izz": 3.0, "ixy": 0.0, "ixz": 0.0, "iyz": 0.0}
    with pytest.raises(ValueError, match="triangle inequality.*ixx \\+ iyy"):
        PV.validate_payload(_make(mass_kg=1.0, inertia=bad))


def test_triangle_inequality_ixx_violation_rejected():
    bad = {"ixx": 3.0, "iyy": 1.0, "izz": 1.0, "ixy": 0.0, "ixz": 0.0, "iyz": 0.0}
    with pytest.raises(ValueError, match="triangle inequality.*iyy \\+ izz"):
        PV.validate_payload(_make(mass_kg=1.0, inertia=bad))


def test_triangle_inequality_iyy_violation_rejected():
    bad = {"ixx": 1.0, "iyy": 3.0, "izz": 1.0, "ixy": 0.0, "ixz": 0.0, "iyz": 0.0}
    with pytest.raises(ValueError, match="triangle inequality.*izz \\+ ixx"):
        PV.validate_payload(_make(mass_kg=1.0, inertia=bad))


# ---------------------------------------------------------------------------
# Rejection matrix: pose
# ---------------------------------------------------------------------------


def test_pose_xyz_wrong_length_rejected():
    with pytest.raises(ValueError, match="pose_xyz.*expected 3"):
        PV.validate_payload(_make(pose_xyz=(0.0, 0.0)))


def test_pose_xyz_nan_rejected():
    with pytest.raises(ValueError, match="pose_xyz.*finite"):
        PV.validate_payload(_make(pose_xyz=(0.0, float("nan"), 0.0)))


def test_pose_xyz_inf_rejected():
    with pytest.raises(ValueError, match="pose_xyz.*finite"):
        PV.validate_payload(_make(pose_xyz=(math.inf, 0.0, 0.0)))


def test_pose_rpy_wrong_length_rejected():
    with pytest.raises(ValueError, match="pose_rpy.*expected 3"):
        PV.validate_payload(_make(pose_rpy=(0.0, 0.0, 0.0, 0.0)))


def test_pose_rpy_nan_rejected():
    with pytest.raises(ValueError, match="pose_rpy.*finite"):
        PV.validate_payload(_make(pose_rpy=(float("nan"), 0.0, 0.0)))


def test_pose_rpy_neginf_rejected():
    with pytest.raises(ValueError, match="pose_rpy.*finite"):
        PV.validate_payload(_make(pose_rpy=(0.0, 0.0, -math.inf)))


# ---------------------------------------------------------------------------
# Error messages self-locate via the payload name
# ---------------------------------------------------------------------------


def test_error_message_echoes_payload_name():
    with pytest.raises(ValueError, match="payload 'my_widget'"):
        PV.validate_payload(_make(name="my_widget", mass_kg=-1.0))


# ---------------------------------------------------------------------------
# validate_catalog
# ---------------------------------------------------------------------------


def test_catalog_with_bad_entry_rejected():
    cat = EL.load_payloads()
    bad = replace(cat.payloads[1], mass_kg=-1.0)
    broken = EL.PayloadCatalog(draft=cat.draft, payloads=(cat.payloads[0], bad, cat.payloads[2]))
    with pytest.raises(ValueError, match="mass_kg.*>= 0"):
        PV.validate_catalog(broken)


def test_catalog_rejects_duplicate_names():
    a = _make(name="dup", mass_kg=0.0, inertia=_ZERO_INERTIA)
    b = _make(name="dup", mass_kg=1.0)
    cat = EL.PayloadCatalog(draft=False, payloads=(a, b))
    with pytest.raises(ValueError, match="duplicate payload name 'dup'"):
        PV.validate_catalog(cat)


def test_catalog_accepts_unique_names():
    a = _make(name="a", mass_kg=0.0, inertia=_ZERO_INERTIA)
    b = _make(name="b", mass_kg=1.0)
    cat = EL.PayloadCatalog(draft=False, payloads=(a, b))
    PV.validate_catalog(cat)


def test_catalog_empty_ok():
    cat = EL.PayloadCatalog(draft=False, payloads=())
    PV.validate_catalog(cat)
