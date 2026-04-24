"""Unit tests for the R2 integration-test expectation files (M6.15).

Pins the schema of:
  * tests/integration/expectations/ur5e.yaml
  * tests/integration/expectations/ur15.yaml
  * tests/integration/expectations/payloads.yaml

These files carry the per-arm / per-controller theoretical-response
tolerances consumed by the (not-yet-landed) R2 test matrix
(M6.12–M6.14). Values inside the files are first-principles drafts
pending operator review; this schema test guards the **structure**,
not the numbers, so the drafts can evolve without breaking CI.

Rules encoded here:
  * Each arm file cross-references the `effort_limits` YAML shipped by
    `ur_sim_config` so a silent divergence between the sim's MJCF
    torque limit and the R2 expectation table fails loudly.
  * Joint order + names are the canonical UR six-joint order.
  * Every controller listed in ROADMAP M6.3 has a dedicated entry.
  * Tolerance blocks carry required keys with finite positive values.
  * `payloads.yaml` contains exactly the three levels
    {no_payload, small_payload, large_payload} that R2 parametrises
    over, in that order.

Pure-Python, runs under ``scripts/run_tests.sh`` stage 1.
"""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
EXPECT_DIR = REPO_ROOT / "tests" / "integration" / "expectations"
UR_TYPES_DIR = (
    REPO_ROOT / "third_party" / "ur_simulator" / "src" / "ur_sim_config" / "config" / "ur_types"
)

CANONICAL_JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)

# Every controller the R2 matrix exercises. Must match ROADMAP §M6.3 +
# the in-tree controllers from M2/M3. If a new controller is added,
# this list must grow in the same commit that adds the expectation
# entry.
REQUIRED_CONTROLLERS = {
    "joint_trajectory_controller": "position",
    "forward_position_controller": "position",
    "forward_velocity_controller": "velocity",
    "forward_effort_controller": "effort",
    "crisp_joint_impedance": "effort",
    "simple_joint_impedance": "effort",
}

# Expected keys per response_model. The R2 runner uses response_model
# to decide which theoretical prediction to compute; unknown models
# would silently skip the assertion, so we pin the vocabulary.
RESPONSE_MODELS = {
    "first_order_lag",
    "second_order",
    "open_loop_torque",
}

TCP_TOLERANCE_KEYS = {
    "tcp_rmse_mm",
    "tcp_peak_err_mm",
    "tcp_orientation_peak_deg",
    "tcp_steady_drift_mm_per_30s",
}

STAGE2_TOLERANCE_KEYS = {
    "completion_tol_rad",
    "peak_tracking_err_rad",
    "saturation_hold_ms",
}

INERTIA_KEYS = ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")


def _load_yaml(path: Path) -> dict:
    with path.open() as f:
        return yaml.safe_load(f)


@pytest.fixture(scope="module")
def ur5e() -> dict:
    return _load_yaml(EXPECT_DIR / "ur5e.yaml")


@pytest.fixture(scope="module")
def ur15() -> dict:
    return _load_yaml(EXPECT_DIR / "ur15.yaml")


@pytest.fixture(scope="module")
def payloads() -> dict:
    return _load_yaml(EXPECT_DIR / "payloads.yaml")


# ---------------------------------------------------------------------------
# Shared per-arm assertions
# ---------------------------------------------------------------------------


def _check_arm(doc: dict, arm_name: str) -> None:
    assert doc["arm"] == arm_name, doc["arm"]
    # Drafts are permitted while M6.15 is under review, but the key
    # must be present so reviewers see the status at a glance.
    assert isinstance(doc.get("draft"), bool)

    # Cross-reference effort limits with the sim's ur_types YAML —
    # the same file that feeds the MJCF ctrlrange.
    sim_effort = _load_yaml(UR_TYPES_DIR / f"{arm_name}.yaml")["effort_limits"]

    joints = doc["joints"]
    assert [j["name"] for j in joints] == list(CANONICAL_JOINTS), [j["name"] for j in joints]
    for j in joints:
        assert isinstance(j["home_rad"], (int, float))
        elim = j["effort_limit_nm"]
        assert isinstance(elim, (int, float)) and elim > 0.0
        assert elim == pytest.approx(sim_effort[j["name"]]), (
            f"{arm_name}:{j['name']} effort limit {elim} diverges from "
            f"ur_types/{arm_name}.yaml ({sim_effort[j['name']]})"
        )
        inertia = j["effective_inertia_kg_m2"]
        assert isinstance(inertia, (int, float)) and inertia > 0.0

    controllers = doc["controllers"]
    assert set(controllers.keys()) == set(REQUIRED_CONTROLLERS.keys()), set(controllers.keys())
    for cname, iface in REQUIRED_CONTROLLERS.items():
        entry = controllers[cname]
        assert entry["interface"] == iface, (cname, entry["interface"])
        assert entry["response_model"] in RESPONSE_MODELS, entry["response_model"]
        tol = entry["tolerances"]
        assert isinstance(tol, dict) and tol, cname
        for k, v in tol.items():
            assert isinstance(v, (int, float)), (cname, k, v)
            assert v > 0.0, (cname, k, v)
        # Second-order entries must carry the damping-ratio band
        # that R2 stage-1 asserts on.
        if entry["response_model"] == "second_order":
            assert "damping_ratio_pct" in tol, cname
            assert "bounded_err_rad" in tol, cname

    tcp = doc["tcp"]["tolerances"]
    assert set(tcp.keys()) == TCP_TOLERANCE_KEYS, set(tcp.keys())
    for k, v in tcp.items():
        assert isinstance(v, (int, float)) and v > 0.0, (k, v)

    stage2 = doc["stage2"]
    assert isinstance(stage2, dict), stage2
    assert set(stage2.keys()) == STAGE2_TOLERANCE_KEYS, set(stage2.keys())
    for k, v in stage2.items():
        assert isinstance(v, (int, float)) and not isinstance(v, bool), (k, v)
        assert v > 0.0, (k, v)


def test_ur5e_schema(ur5e: dict) -> None:
    _check_arm(ur5e, "ur5e")


def test_ur15_schema(ur15: dict) -> None:
    _check_arm(ur15, "ur15")


def test_both_arms_share_controller_set(ur5e: dict, ur15: dict) -> None:
    # The R2 runner must be arm-agnostic; controller key sets must match.
    assert set(ur5e["controllers"].keys()) == set(ur15["controllers"].keys())


def test_both_arms_share_tcp_tolerance_keys(ur5e: dict, ur15: dict) -> None:
    assert set(ur5e["tcp"]["tolerances"].keys()) == set(ur15["tcp"]["tolerances"].keys())


def test_both_arms_share_stage2_values(ur5e: dict, ur15: dict) -> None:
    # ROADMAP R2 requires identical pass/fail criteria across arms;
    # pin full block equality, not just the key set, so a per-arm
    # tweak has to be explicit (via an ADR).
    assert ur5e["stage2"] == ur15["stage2"]


# ---------------------------------------------------------------------------
# payloads.yaml
# ---------------------------------------------------------------------------


def test_payloads_required_levels(payloads: dict) -> None:
    assert isinstance(payloads.get("draft"), bool)
    levels = payloads["payloads"]
    names = [p["name"] for p in levels]
    assert names == ["no_payload", "small_payload", "large_payload"], names


def test_payloads_schema(payloads: dict) -> None:
    for p in payloads["payloads"]:
        mass = p["mass_kg"]
        assert isinstance(mass, (int, float)) and mass >= 0.0, p["name"]

        inertia = p["inertia_kg_m2"]
        assert set(inertia.keys()) == set(INERTIA_KEYS), p["name"]
        for k in INERTIA_KEYS:
            v = inertia[k]
            assert isinstance(v, (int, float)), (p["name"], k)
            # Principal moments non-negative; products unconstrained
            # in sign.
            if k in ("ixx", "iyy", "izz"):
                assert v >= 0.0, (p["name"], k, v)

        pose = p["pose_in_tool0"]
        xyz = pose["xyz"]
        rpy = pose["rpy"]
        assert len(xyz) == 3 and all(isinstance(x, (int, float)) for x in xyz), p["name"]
        assert len(rpy) == 3 and all(isinstance(x, (int, float)) for x in rpy), p["name"]

        if p["name"] == "no_payload":
            assert mass == 0.0
            assert all(inertia[k] == 0.0 for k in INERTIA_KEYS)


def test_payload_mass_monotonic(payloads: dict) -> None:
    # The levels must strictly increase in mass so tests that rely on
    # "heavier payload → larger gravity-comp error" ordering hold.
    masses = [p["mass_kg"] for p in payloads["payloads"]]
    assert masses == sorted(masses), masses
    assert masses[0] == 0.0
    assert masses[-1] > masses[0]
