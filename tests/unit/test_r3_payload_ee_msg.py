"""Unit tests for :mod:`tests.integration.r3_payload_ee_msg`.

Covers: export surface, frozen dataclass contract, happy paths on all
three built-in payloads, inertia symmetric row-major layout, pose echo,
zero-mass producing a valid message (with loader-supplied pose
preserved, not canonicalized to identity), canonical quaternion
rotations about X / Y / Z, unit-norm invariant, xyzw ordering pinned
against the sibling ``_euler_xyz_to_wxyz`` helper, ``to_dict`` shape,
and validator-error propagation.
"""

from __future__ import annotations

import dataclasses
import importlib.util
import math
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
PV = _load("payload_validation", INT_DIR / "payload_validation.py")
PM = _load("r3_payload_mjcf", INT_DIR / "r3_payload_mjcf.py")
EM = _load("r3_payload_ee_msg", INT_DIR / "r3_payload_ee_msg.py")


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


def test_exports_match_all():
    assert set(EM.__all__) == {"EePayloadMessage", "payload_to_ee_msg"}


def test_converter_callable():
    assert callable(EM.payload_to_ee_msg)


# ---------------------------------------------------------------------------
# Frozen dataclass contract
# ---------------------------------------------------------------------------


def test_ee_payload_message_is_frozen_dataclass():
    assert dataclasses.is_dataclass(EM.EePayloadMessage)
    msg = EM.payload_to_ee_msg(_make(mass_kg=0.0, inertia=_ZERO_INERTIA))
    with pytest.raises(dataclasses.FrozenInstanceError):
        msg.mass_kg = 42.0  # type: ignore[misc]


def test_ee_payload_message_fields_present():
    msg = EM.payload_to_ee_msg(_make())
    assert hasattr(msg, "mass_kg")
    assert hasattr(msg, "inertia_row_major")
    assert hasattr(msg, "pose_position_xyz")
    assert hasattr(msg, "pose_orientation_xyzw")


# ---------------------------------------------------------------------------
# Happy paths — in-tree catalog payloads
# ---------------------------------------------------------------------------


def test_no_payload_round_trip():
    catalog = EL.load_payloads()
    p = catalog.get("no_payload")
    msg = EM.payload_to_ee_msg(p)
    assert msg.mass_kg == 0.0
    assert msg.inertia_row_major == (0.0,) * 9
    # Pose preserved verbatim — loader's no_payload defines identity but
    # we assert that whatever the loader provides flows through.
    assert msg.pose_position_xyz == tuple(p.pose_xyz)
    # rpy=(0,0,0) => quaternion (0,0,0,1) in xyzw.
    assert msg.pose_orientation_xyzw == pytest.approx((0.0, 0.0, 0.0, 1.0), abs=1e-15)


def test_small_payload_round_trip():
    catalog = EL.load_payloads()
    p = catalog.get("small_payload")
    msg = EM.payload_to_ee_msg(p)
    assert msg.mass_kg == p.mass_kg
    assert msg.inertia_row_major[0] == p.inertia_kg_m2["ixx"]
    assert msg.inertia_row_major[4] == p.inertia_kg_m2["iyy"]
    assert msg.inertia_row_major[8] == p.inertia_kg_m2["izz"]
    # Off-diagonals all zero in v1 catalog.
    for idx in (1, 2, 3, 5, 6, 7):
        assert msg.inertia_row_major[idx] == 0.0
    assert msg.pose_position_xyz == tuple(p.pose_xyz)


def test_large_payload_round_trip():
    catalog = EL.load_payloads()
    p = catalog.get("large_payload")
    msg = EM.payload_to_ee_msg(p)
    assert msg.mass_kg == p.mass_kg
    # Large payload pose has a non-zero z offset in the catalog.
    assert msg.pose_position_xyz == tuple(p.pose_xyz)
    assert msg.pose_position_xyz[2] != 0.0


# ---------------------------------------------------------------------------
# Inertia symmetric layout
# ---------------------------------------------------------------------------


def test_inertia_row_major_is_symmetric_when_off_diagonals_present():
    # Bypass the validator's v1 diagonal pin by constructing a Payload
    # with a symmetric tensor *and* having the validator waived — we
    # can't actually run through payload_to_ee_msg with off-diagonals
    # because validate_payload rejects them. So test the layout by
    # inspecting what the converter does when the three diag values
    # differ, and confirm the off-diag positions are zero (the current
    # v1 behaviour).
    p = _make(inertia={"ixx": 0.5, "iyy": 0.75, "izz": 1.25, "ixy": 0.0, "ixz": 0.0, "iyz": 0.0})
    msg = EM.payload_to_ee_msg(p)
    # Row-major 3x3: [ixx ixy ixz; ixy iyy iyz; ixz iyz izz].
    assert msg.inertia_row_major[0] == 0.5
    assert msg.inertia_row_major[4] == 0.75
    assert msg.inertia_row_major[8] == 1.25
    # Symmetric: [1]==[3], [2]==[6], [5]==[7].
    assert msg.inertia_row_major[1] == msg.inertia_row_major[3]
    assert msg.inertia_row_major[2] == msg.inertia_row_major[6]
    assert msg.inertia_row_major[5] == msg.inertia_row_major[7]


def test_inertia_row_major_has_length_nine():
    msg = EM.payload_to_ee_msg(_make())
    assert len(msg.inertia_row_major) == 9


# ---------------------------------------------------------------------------
# Pose echo
# ---------------------------------------------------------------------------


def test_pose_position_echoed_verbatim():
    p = _make(pose_xyz=(0.1, -0.2, 0.35))
    msg = EM.payload_to_ee_msg(p)
    assert msg.pose_position_xyz == (0.1, -0.2, 0.35)


def test_pose_position_negative_components():
    p = _make(pose_xyz=(-0.5, -0.25, -0.125))
    msg = EM.payload_to_ee_msg(p)
    assert msg.pose_position_xyz == (-0.5, -0.25, -0.125)


# ---------------------------------------------------------------------------
# Zero-mass keeps loader pose — do NOT canonicalize to identity
# ---------------------------------------------------------------------------


def test_zero_mass_preserves_nonzero_pose_position():
    # ADR-0012 §R3.2 pins zero mass as the sim's launch state. The
    # loader's contract only ties mass to inertia, not to pose, so a
    # zero-mass payload with a non-zero pose is a legal loader output
    # and the converter must not silently discard the pose.
    p = _make(
        mass_kg=0.0,
        inertia=_ZERO_INERTIA,
        pose_xyz=(0.07, 0.0, 0.0),
    )
    msg = EM.payload_to_ee_msg(p)
    assert msg.mass_kg == 0.0
    assert msg.pose_position_xyz == (0.07, 0.0, 0.0)
    assert msg.inertia_row_major == (0.0,) * 9


def test_zero_mass_preserves_nonzero_pose_orientation():
    p = _make(
        mass_kg=0.0,
        inertia=_ZERO_INERTIA,
        pose_rpy=(0.0, 0.0, math.pi / 2),
    )
    msg = EM.payload_to_ee_msg(p)
    assert msg.mass_kg == 0.0
    # rpy=(0,0,pi/2) => quat xyzw = (0, 0, sin(pi/4), cos(pi/4)).
    qx, qy, qz, qw = msg.pose_orientation_xyzw
    assert qx == pytest.approx(0.0, abs=1e-15)
    assert qy == pytest.approx(0.0, abs=1e-15)
    assert qz == pytest.approx(math.sin(math.pi / 4), abs=1e-15)
    assert qw == pytest.approx(math.cos(math.pi / 4), abs=1e-15)


# ---------------------------------------------------------------------------
# Quaternion: canonical rotations
# ---------------------------------------------------------------------------


def test_quaternion_identity_for_zero_rpy():
    msg = EM.payload_to_ee_msg(_make(pose_rpy=(0.0, 0.0, 0.0)))
    assert msg.pose_orientation_xyzw == pytest.approx((0.0, 0.0, 0.0, 1.0), abs=1e-15)


def test_quaternion_roll_90_about_x():
    msg = EM.payload_to_ee_msg(_make(pose_rpy=(math.pi / 2, 0.0, 0.0)))
    s = math.sin(math.pi / 4)
    c = math.cos(math.pi / 4)
    assert msg.pose_orientation_xyzw == pytest.approx((s, 0.0, 0.0, c), abs=1e-15)


def test_quaternion_pitch_90_about_y():
    msg = EM.payload_to_ee_msg(_make(pose_rpy=(0.0, math.pi / 2, 0.0)))
    s = math.sin(math.pi / 4)
    c = math.cos(math.pi / 4)
    assert msg.pose_orientation_xyzw == pytest.approx((0.0, s, 0.0, c), abs=1e-15)


def test_quaternion_yaw_90_about_z():
    msg = EM.payload_to_ee_msg(_make(pose_rpy=(0.0, 0.0, math.pi / 2)))
    s = math.sin(math.pi / 4)
    c = math.cos(math.pi / 4)
    assert msg.pose_orientation_xyzw == pytest.approx((0.0, 0.0, s, c), abs=1e-15)


def test_quaternion_unit_norm_arbitrary_rpy():
    p = _make(pose_rpy=(0.3, -0.7, 1.1))
    msg = EM.payload_to_ee_msg(p)
    qx, qy, qz, qw = msg.pose_orientation_xyzw
    norm2 = qx * qx + qy * qy + qz * qz + qw * qw
    assert norm2 == pytest.approx(1.0, abs=1e-15)


# ---------------------------------------------------------------------------
# xyzw vs wxyz pinning — reuse the sibling helper so drift is caught
# ---------------------------------------------------------------------------


def test_xyzw_matches_sibling_wxyz_reordered():
    rpy = (0.15, -0.4, 0.85)
    w, x, y, z = PM._euler_xyz_to_wxyz(rpy)  # Hamilton w,x,y,z
    msg = EM.payload_to_ee_msg(_make(pose_rpy=rpy))
    # ROS geometry_msgs/Quaternion ordering: x, y, z, w.
    assert msg.pose_orientation_xyzw == pytest.approx((x, y, z, w), abs=1e-15)


def test_xyzw_last_component_is_real_part():
    # Sanity check the ordering: for identity rotation the real part
    # dominates and lives at index 3.
    msg = EM.payload_to_ee_msg(_make(pose_rpy=(0.0, 0.0, 0.0)))
    assert msg.pose_orientation_xyzw[3] == 1.0
    assert msg.pose_orientation_xyzw[:3] == (0.0, 0.0, 0.0)


# ---------------------------------------------------------------------------
# to_dict shape
# ---------------------------------------------------------------------------


def test_to_dict_shape_mirrors_ros_message():
    p = _make(pose_xyz=(0.1, 0.2, 0.3), pose_rpy=(0.0, 0.0, math.pi / 2))
    msg = EM.payload_to_ee_msg(p)
    d = msg.to_dict()
    assert set(d.keys()) == {"mass", "inertia", "pose"}
    assert d["mass"] == msg.mass_kg
    assert d["inertia"] == list(msg.inertia_row_major)
    assert set(d["pose"].keys()) == {"position", "orientation"}
    assert d["pose"]["position"] == {"x": 0.1, "y": 0.2, "z": 0.3}
    assert set(d["pose"]["orientation"].keys()) == {"x", "y", "z", "w"}


def test_to_dict_inertia_is_plain_list():
    msg = EM.payload_to_ee_msg(_make())
    d = msg.to_dict()
    assert isinstance(d["inertia"], list)
    assert len(d["inertia"]) == 9


# ---------------------------------------------------------------------------
# Validator propagation
# ---------------------------------------------------------------------------


def test_negative_mass_rejected():
    p = _make(mass_kg=-1.0)
    with pytest.raises(ValueError, match="mass"):
        EM.payload_to_ee_msg(p)


def test_non_diagonal_inertia_rejected():
    p = _make(inertia={"ixx": 0.01, "iyy": 0.02, "izz": 0.03, "ixy": 0.001, "ixz": 0.0, "iyz": 0.0})
    with pytest.raises(ValueError):
        EM.payload_to_ee_msg(p)


def test_zero_mass_with_nonzero_inertia_rejected():
    p = _make(mass_kg=0.0, inertia=_GOOD_INERTIA)
    with pytest.raises(ValueError):
        EM.payload_to_ee_msg(p)


def test_nan_in_pose_xyz_rejected():
    p = _make(pose_xyz=(float("nan"), 0.0, 0.0))
    with pytest.raises(ValueError):
        EM.payload_to_ee_msg(p)


def test_nan_in_pose_rpy_rejected():
    p = _make(pose_rpy=(float("nan"), 0.0, 0.0))
    with pytest.raises(ValueError):
        EM.payload_to_ee_msg(p)
