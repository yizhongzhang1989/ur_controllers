"""Unit tests for the R2 expectations loader (M6.15 follow-up).

Pins the consumer API of ``tests/integration/expectations_loader.py``
so the R2 integration tests (M6.12–M6.14, still gated on M6.0) can
be authored against a stable surface. Covers:

* ``load_arm`` / ``load_payloads`` happy path for both arms.
* Per-joint / per-controller / per-tolerance lookup helpers raise on
  unknown keys instead of returning ``None``.
* Second-order response formulas are numerically correct and reject
  non-physical inputs.
* ``damping_ratio_within_band`` implements the ±% band the
  expectation YAML keys advertise.
* Schema drift at load time (e.g. a joint reordering, a non-numeric
  tolerance, a wrong ``arm:`` field) raises rather than silently
  mis-parsing. These guard against regressions that the pure-schema
  test might miss because they exercise the *loader's* error path.
"""

from __future__ import annotations

import importlib.util
import math
import sys
from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
LOADER_PATH = REPO_ROOT / "tests" / "integration" / "expectations_loader.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("_expectations_loader", LOADER_PATH)
    assert spec and spec.loader
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


EL = _load_module()


# ---------------------------------------------------------------------------
# load_arm / load_payloads happy paths
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("arm", ["ur5e", "ur15"])
def test_load_arm_happy(arm):
    doc = EL.load_arm(arm)
    assert doc.arm == arm
    assert isinstance(doc.draft, bool)
    assert tuple(j.name for j in doc.joints) == EL.CANONICAL_JOINTS
    for j in doc.joints:
        assert j.effort_limit_nm > 0.0
        assert j.effective_inertia_kg_m2 > 0.0
    # Every required controller from M6.3 / M2 / M3 present.
    for cname in (
        "joint_trajectory_controller",
        "forward_position_controller",
        "forward_velocity_controller",
        "forward_effort_controller",
        "crisp_joint_impedance",
        "simple_joint_impedance",
    ):
        c = doc.controller(cname)
        assert c.interface in {"position", "velocity", "effort"}
        assert c.tolerances
    for key in (
        "tcp_rmse_mm",
        "tcp_peak_err_mm",
        "tcp_orientation_peak_deg",
        "tcp_steady_drift_mm_per_30s",
    ):
        assert doc.tcp_tol(key) > 0.0


def test_load_arm_rejects_unknown():
    with pytest.raises(ValueError, match="unsupported arm"):
        EL.load_arm("ur10e")


def test_arm_joint_lookup_raises():
    doc = EL.load_arm("ur5e")
    with pytest.raises(KeyError, match="no joint"):
        doc.joint("nonexistent_joint")


def test_arm_controller_lookup_raises():
    doc = EL.load_arm("ur5e")
    with pytest.raises(KeyError, match="no controller"):
        doc.controller("not_a_controller")


def test_controller_tol_lookup_raises():
    doc = EL.load_arm("ur5e")
    c = doc.controller("crisp_joint_impedance")
    with pytest.raises(KeyError, match="no tolerance"):
        c.tol("not_a_key")


def test_arm_tcp_tol_lookup_raises():
    doc = EL.load_arm("ur5e")
    with pytest.raises(KeyError, match="no tcp tolerance"):
        doc.tcp_tol("not_a_key")


def test_load_payloads_happy():
    cat = EL.load_payloads()
    assert cat.names() == EL.PAYLOAD_LEVELS
    # no_payload has zero mass and zero inertia.
    zero = cat.get("no_payload")
    assert zero.mass_kg == 0.0
    assert all(zero.inertia_kg_m2[k] == 0.0 for k in ("ixx", "iyy", "izz"))
    # Iteration yields the same ordering.
    assert [p.name for p in cat] == list(EL.PAYLOAD_LEVELS)
    # Monotonic masses.
    masses = [p.mass_kg for p in cat]
    assert masses == sorted(masses)
    assert masses[-1] > masses[0]


def test_payload_lookup_raises():
    cat = EL.load_payloads()
    with pytest.raises(KeyError, match="unknown payload"):
        cat.get("huge_payload")


# ---------------------------------------------------------------------------
# Theoretical response helpers
# ---------------------------------------------------------------------------


def test_second_order_response_critical_damping():
    # K=100, J=1, D=2*sqrt(K*J)=20 → omega_n=10, zeta=1 (critical).
    omega_n, zeta = EL.second_order_response(100.0, 20.0, 1.0)
    assert omega_n == pytest.approx(10.0)
    assert zeta == pytest.approx(1.0)


def test_second_order_response_underdamped():
    omega_n, zeta = EL.second_order_response(400.0, 8.0, 4.0)
    # omega_n = sqrt(400/4) = 10; zeta = 8 / (2*sqrt(400*4)) = 8/80 = 0.1.
    assert omega_n == pytest.approx(10.0)
    assert zeta == pytest.approx(0.1)


def test_second_order_response_zero_damping():
    omega_n, zeta = EL.second_order_response(25.0, 0.0, 1.0)
    assert omega_n == pytest.approx(5.0)
    assert zeta == 0.0


@pytest.mark.parametrize(
    "k,d,j,match",
    [
        (0.0, 1.0, 1.0, "stiffness K"),
        (-1.0, 1.0, 1.0, "stiffness K"),
        (1.0, -0.1, 1.0, "damping D"),
        (1.0, 0.0, 0.0, "effective inertia"),
        (1.0, 0.0, -2.0, "effective inertia"),
    ],
)
def test_second_order_response_rejects_bad(k, d, j, match):
    with pytest.raises(ValueError, match=match):
        EL.second_order_response(k, d, j)


def test_damping_ratio_within_band_basic():
    # 20% band around 0.7.
    assert EL.damping_ratio_within_band(0.7, 0.7, 20.0)
    assert EL.damping_ratio_within_band(0.56, 0.7, 20.0)  # at lower edge
    assert EL.damping_ratio_within_band(0.84, 0.7, 20.0)  # at upper edge
    assert not EL.damping_ratio_within_band(0.55, 0.7, 20.0)
    assert not EL.damping_ratio_within_band(0.85, 0.7, 20.0)


def test_damping_ratio_within_band_zero_theory():
    assert EL.damping_ratio_within_band(0.0, 0.0, 20.0)
    assert not EL.damping_ratio_within_band(0.01, 0.0, 20.0)


def test_damping_ratio_within_band_rejects_negative_tolerance():
    with pytest.raises(ValueError, match="tolerance_pct"):
        EL.damping_ratio_within_band(0.5, 0.5, -1.0)


# ---------------------------------------------------------------------------
# Loader error paths on doctored YAML (schema drift guard)
# ---------------------------------------------------------------------------


def _write_arm_doc(tmp: Path, arm: str, doc: dict) -> Path:
    (tmp / f"{arm}.yaml").write_text(yaml.safe_dump(doc))
    return tmp


def _good_arm_doc(arm: str) -> dict:
    # Minimal doc matching the real schema.
    def j(name, h=0.0, elim=10.0, inertia=1.0):
        return {
            "name": name,
            "home_rad": h,
            "effort_limit_nm": elim,
            "effective_inertia_kg_m2": inertia,
        }

    ctrl_tol = {"bounded_err_rad": 0.1, "damping_ratio_pct": 20.0}
    tcp_tol = {
        "tcp_rmse_mm": 1.0,
        "tcp_peak_err_mm": 2.0,
        "tcp_orientation_peak_deg": 1.0,
        "tcp_steady_drift_mm_per_30s": 0.5,
    }
    return {
        "arm": arm,
        "draft": True,
        "joints": [j(n) for n in EL.CANONICAL_JOINTS],
        "controllers": {
            "crisp_joint_impedance": {
                "interface": "effort",
                "response_model": "second_order",
                "tolerances": ctrl_tol,
            },
        },
        "tcp": {"tolerances": tcp_tol},
    }


def test_load_arm_rejects_arm_field_mismatch(tmp_path):
    doc = _good_arm_doc("ur5e")
    doc["arm"] = "ur10e"
    _write_arm_doc(tmp_path, "ur5e", doc)
    with pytest.raises(ValueError, match="does not match filename"):
        EL.load_arm("ur5e", expect_dir=tmp_path)


def test_load_arm_rejects_joint_reorder(tmp_path):
    doc = _good_arm_doc("ur5e")
    # Swap two joints.
    doc["joints"][0], doc["joints"][1] = doc["joints"][1], doc["joints"][0]
    _write_arm_doc(tmp_path, "ur5e", doc)
    with pytest.raises(ValueError, match="joint order"):
        EL.load_arm("ur5e", expect_dir=tmp_path)


def test_load_arm_rejects_non_numeric_tolerance(tmp_path):
    doc = _good_arm_doc("ur5e")
    doc["controllers"]["crisp_joint_impedance"]["tolerances"]["bounded_err_rad"] = "oops"
    _write_arm_doc(tmp_path, "ur5e", doc)
    with pytest.raises(ValueError, match="expected number"):
        EL.load_arm("ur5e", expect_dir=tmp_path)


def test_load_arm_rejects_bool_as_number(tmp_path):
    # yaml.safe_load turns `true`/`false` into Python bool, which is a
    # subclass of int. The loader must reject this, otherwise
    # effort_limit_nm=True would silently pass.
    doc = _good_arm_doc("ur5e")
    doc["joints"][0]["effort_limit_nm"] = True
    _write_arm_doc(tmp_path, "ur5e", doc)
    with pytest.raises(ValueError, match="expected number"):
        EL.load_arm("ur5e", expect_dir=tmp_path)


def test_load_arm_rejects_empty_tolerances(tmp_path):
    doc = _good_arm_doc("ur5e")
    doc["controllers"]["crisp_joint_impedance"]["tolerances"] = {}
    _write_arm_doc(tmp_path, "ur5e", doc)
    with pytest.raises(ValueError, match="non-empty mapping"):
        EL.load_arm("ur5e", expect_dir=tmp_path)


def test_load_payloads_rejects_reorder(tmp_path):
    # Build a minimal payloads doc with levels out of order.
    doc = {
        "draft": True,
        "payloads": [
            {
                "name": "small_payload",  # wrong: should be no_payload first
                "mass_kg": 1.0,
                "inertia_kg_m2": {k: 0.0 for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")},
                "pose_in_tool0": {"xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]},
            },
            {
                "name": "no_payload",
                "mass_kg": 0.0,
                "inertia_kg_m2": {k: 0.0 for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")},
                "pose_in_tool0": {"xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]},
            },
            {
                "name": "large_payload",
                "mass_kg": 5.0,
                "inertia_kg_m2": {k: 0.0 for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")},
                "pose_in_tool0": {"xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]},
            },
        ],
    }
    (tmp_path / "payloads.yaml").write_text(yaml.safe_dump(doc))
    with pytest.raises(ValueError, match="level order"):
        EL.load_payloads(expect_dir=tmp_path)


def test_canonicals_match_schema_test():
    # If CANONICAL_JOINTS or PAYLOAD_LEVELS drift, the independent
    # schema test (test_expectations_schema.py) would also fail — but
    # pin the link explicitly so a silent divergence is caught here
    # at loader-level too.
    assert EL.CANONICAL_JOINTS == (
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    )
    assert EL.PAYLOAD_LEVELS == ("no_payload", "small_payload", "large_payload")
    assert EL.SUPPORTED_ARMS == ("ur5e", "ur15")


def test_all_exports_importable():
    # Guard: every name in __all__ must exist on the module, so
    # callers using `from ... import *` don't ImportError at runtime.
    for name in EL.__all__:
        assert hasattr(EL, name), name


def test_second_order_home_pose_ur5e_sanity():
    # Sanity check: feed a plausible crisp K/D + the draft inertia
    # for shoulder_pan and make sure the math stays finite. Values
    # are draft; this test asserts only positivity, not numerics.
    doc = EL.load_arm("ur5e")
    j = doc.joint("shoulder_pan_joint")
    omega_n, zeta = EL.second_order_response(
        stiffness_k=200.0, damping_d=40.0, effective_inertia_j=j.effective_inertia_kg_m2
    )
    assert omega_n > 0.0 and math.isfinite(omega_n)
    assert zeta > 0.0 and math.isfinite(zeta)
