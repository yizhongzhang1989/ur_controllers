"""Typed loader for the R2 expectation tables.

Pre-baked, tested consumer API for the YAML files under
``tests/integration/expectations/``. Landed ahead of the R2 test matrix
(M6.12–M6.14) so those tests can be authored against a single
well-typed surface instead of open-coding ``yaml.safe_load`` in three
places.

Scope (intentionally narrow):

* Parse ``ur5e.yaml`` / ``ur15.yaml`` / ``payloads.yaml`` into
  dataclasses.
* Provide per-arm / per-controller / per-joint lookup helpers that
  raise on missing keys rather than silently skipping assertions.
* Provide first-principles theoretical-response helpers that R2
  stage-1 needs (second-order natural frequency + damping ratio, from
  ``K``, ``D``, ``J_eff``).

Non-goals:

* Numeric value review — that is an ADR-0013 operator action.
* Reading controller configs (``bringup/config/*.yaml``) to feed
  ``K``/``D`` into the second-order helper. Callers pass those in,
  keeping this module decoupled from the controller YAMLs.
* First-order / open-loop-torque helpers — R2 stage-1 uses simple
  steady-state + drift bounds for those; no closed-form needed.

The schema itself is pinned by
``tests/unit/test_expectations_schema.py``; this module trusts that
contract and raises ``KeyError`` / ``ValueError`` on any divergence so
an upstream schema drift fails loudly at load time.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterator, Mapping, Sequence, Tuple

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
EXPECT_DIR = REPO_ROOT / "tests" / "integration" / "expectations"

CANONICAL_JOINTS: Tuple[str, ...] = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)

SUPPORTED_ARMS: Tuple[str, ...] = ("ur5e", "ur15")

PAYLOAD_LEVELS: Tuple[str, ...] = ("no_payload", "small_payload", "large_payload")


# ---------------------------------------------------------------------------
# Dataclasses
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class JointExpectation:
    name: str
    home_rad: float
    effort_limit_nm: float
    effective_inertia_kg_m2: float


@dataclass(frozen=True)
class ControllerExpectation:
    name: str
    interface: str
    response_model: str
    tolerances: Mapping[str, float]

    def tol(self, key: str) -> float:
        try:
            return float(self.tolerances[key])
        except KeyError as exc:
            raise KeyError(
                f"controller {self.name!r} has no tolerance {key!r}; "
                f"available: {sorted(self.tolerances)}"
            ) from exc


@dataclass(frozen=True)
class Stage2Tolerances:
    """R2 stage-2 (all-joints-together) joint-space tolerances.

    Matches the kwargs of
    ``r2_stage2_assertions.evaluate_all_joints_joint_space``. Keeps the
    pre-baked assertion module decoupled from the YAML while giving
    test authors one typed source-of-truth for the numbers.
    """

    completion_tol_rad: float
    peak_tracking_err_rad: float
    saturation_hold_ms: float


@dataclass(frozen=True)
class Stage2TcpTolerances:
    """R2 stage-2 *cartesian-mode* consistency tolerances.

    Matches the kwargs of
    ``r2_stage2_cartesian.evaluate_all_joints_cartesian``. Keeps the
    pre-baked evaluator decoupled from the YAML while giving test
    authors one typed source-of-truth for the numbers.
    """

    position_peak_err_mm: float
    orientation_peak_err_deg: float


@dataclass(frozen=True)
class ArmExpectation:
    arm: str
    draft: bool
    joints: Tuple[JointExpectation, ...]
    controllers: Mapping[str, ControllerExpectation]
    stage2: Stage2Tolerances
    stage2_tcp: Stage2TcpTolerances
    tcp_tolerances: Mapping[str, float]

    def joint(self, name: str) -> JointExpectation:
        for j in self.joints:
            if j.name == name:
                return j
        raise KeyError(f"arm {self.arm!r} has no joint {name!r}")

    def controller(self, name: str) -> ControllerExpectation:
        try:
            return self.controllers[name]
        except KeyError as exc:
            raise KeyError(
                f"arm {self.arm!r} has no controller {name!r}; "
                f"available: {sorted(self.controllers)}"
            ) from exc

    def tcp_tol(self, key: str) -> float:
        try:
            return float(self.tcp_tolerances[key])
        except KeyError as exc:
            raise KeyError(
                f"arm {self.arm!r} has no tcp tolerance {key!r}; "
                f"available: {sorted(self.tcp_tolerances)}"
            ) from exc


@dataclass(frozen=True)
class Payload:
    name: str
    mass_kg: float
    inertia_kg_m2: Mapping[str, float]
    pose_xyz: Tuple[float, float, float]
    pose_rpy: Tuple[float, float, float]


@dataclass(frozen=True)
class PayloadCatalog:
    draft: bool
    payloads: Tuple[Payload, ...]

    def __iter__(self) -> Iterator[Payload]:
        return iter(self.payloads)

    def names(self) -> Tuple[str, ...]:
        return tuple(p.name for p in self.payloads)

    def get(self, name: str) -> Payload:
        for p in self.payloads:
            if p.name == name:
                return p
        raise KeyError(f"unknown payload {name!r}; available: {self.names()}")


# ---------------------------------------------------------------------------
# Loaders
# ---------------------------------------------------------------------------


def _yaml(path: Path) -> dict:
    with path.open() as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError(f"{path}: expected top-level mapping, got {type(data).__name__}")
    return data


def _as_float(value: object, where: str) -> float:
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        raise ValueError(f"{where}: expected number, got {value!r}")
    return float(value)


def load_arm(arm: str, expect_dir: Path = EXPECT_DIR) -> ArmExpectation:
    if arm not in SUPPORTED_ARMS:
        raise ValueError(f"unsupported arm {arm!r}; expected one of {SUPPORTED_ARMS}")
    doc = _yaml(expect_dir / f"{arm}.yaml")

    if doc.get("arm") != arm:
        raise ValueError(f"{arm}.yaml: arm field {doc.get('arm')!r} does not match filename")

    joints_raw = doc["joints"]
    names = [j["name"] for j in joints_raw]
    if tuple(names) != CANONICAL_JOINTS:
        raise ValueError(
            f"{arm}.yaml: joint order {names} does not match canonical " f"{list(CANONICAL_JOINTS)}"
        )
    joints = tuple(
        JointExpectation(
            name=j["name"],
            home_rad=_as_float(j["home_rad"], f"{arm}:{j['name']}.home_rad"),
            effort_limit_nm=_as_float(j["effort_limit_nm"], f"{arm}:{j['name']}.effort_limit_nm"),
            effective_inertia_kg_m2=_as_float(
                j["effective_inertia_kg_m2"],
                f"{arm}:{j['name']}.effective_inertia_kg_m2",
            ),
        )
        for j in joints_raw
    )

    controllers: Dict[str, ControllerExpectation] = {}
    for cname, entry in doc["controllers"].items():
        tol_raw = entry["tolerances"]
        if not isinstance(tol_raw, dict) or not tol_raw:
            raise ValueError(f"{arm}:{cname}: tolerances must be non-empty mapping")
        tol = {k: _as_float(v, f"{arm}:{cname}.tolerances.{k}") for k, v in tol_raw.items()}
        controllers[cname] = ControllerExpectation(
            name=cname,
            interface=str(entry["interface"]),
            response_model=str(entry["response_model"]),
            tolerances=tol,
        )

    stage2_raw = doc.get("stage2")
    if not isinstance(stage2_raw, dict):
        raise ValueError(f"{arm}.yaml: missing required 'stage2' mapping")
    stage2_keys = ("completion_tol_rad", "peak_tracking_err_rad", "saturation_hold_ms")
    missing = [k for k in stage2_keys if k not in stage2_raw]
    if missing:
        raise ValueError(f"{arm}.yaml: stage2 missing required keys {missing}")
    stage2 = Stage2Tolerances(
        completion_tol_rad=_as_float(
            stage2_raw["completion_tol_rad"], f"{arm}:stage2.completion_tol_rad"
        ),
        peak_tracking_err_rad=_as_float(
            stage2_raw["peak_tracking_err_rad"], f"{arm}:stage2.peak_tracking_err_rad"
        ),
        saturation_hold_ms=_as_float(
            stage2_raw["saturation_hold_ms"], f"{arm}:stage2.saturation_hold_ms"
        ),
    )

    stage2_tcp_raw = doc.get("stage2_tcp")
    if not isinstance(stage2_tcp_raw, dict):
        raise ValueError(f"{arm}.yaml: missing required 'stage2_tcp' mapping")
    stage2_tcp_keys = ("position_peak_err_mm", "orientation_peak_err_deg")
    missing_tcp = [k for k in stage2_tcp_keys if k not in stage2_tcp_raw]
    if missing_tcp:
        raise ValueError(f"{arm}.yaml: stage2_tcp missing required keys {missing_tcp}")
    stage2_tcp = Stage2TcpTolerances(
        position_peak_err_mm=_as_float(
            stage2_tcp_raw["position_peak_err_mm"],
            f"{arm}:stage2_tcp.position_peak_err_mm",
        ),
        orientation_peak_err_deg=_as_float(
            stage2_tcp_raw["orientation_peak_err_deg"],
            f"{arm}:stage2_tcp.orientation_peak_err_deg",
        ),
    )

    tcp_raw = doc["tcp"]["tolerances"]
    tcp_tol = {k: _as_float(v, f"{arm}:tcp.tolerances.{k}") for k, v in tcp_raw.items()}

    return ArmExpectation(
        arm=arm,
        draft=bool(doc.get("draft", False)),
        joints=joints,
        controllers=controllers,
        stage2=stage2,
        stage2_tcp=stage2_tcp,
        tcp_tolerances=tcp_tol,
    )


def load_payloads(expect_dir: Path = EXPECT_DIR) -> PayloadCatalog:
    doc = _yaml(expect_dir / "payloads.yaml")
    levels = doc["payloads"]
    names = [p["name"] for p in levels]
    if tuple(names) != PAYLOAD_LEVELS:
        raise ValueError(
            f"payloads.yaml: level order {names} does not match canonical "
            f"{list(PAYLOAD_LEVELS)}"
        )
    payloads = []
    for p in levels:
        inertia_raw = p["inertia_kg_m2"]
        inertia = {
            k: _as_float(inertia_raw[k], f"{p['name']}.inertia.{k}")
            for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")
        }
        pose = p["pose_in_tool0"]
        xyz = pose["xyz"]
        rpy = pose["rpy"]
        if len(xyz) != 3 or len(rpy) != 3:
            raise ValueError(f"{p['name']}: pose_in_tool0 xyz/rpy must be length-3")
        payloads.append(
            Payload(
                name=p["name"],
                mass_kg=_as_float(p["mass_kg"], f"{p['name']}.mass_kg"),
                inertia_kg_m2=inertia,
                pose_xyz=(float(xyz[0]), float(xyz[1]), float(xyz[2])),
                pose_rpy=(float(rpy[0]), float(rpy[1]), float(rpy[2])),
            )
        )
    return PayloadCatalog(draft=bool(doc.get("draft", False)), payloads=tuple(payloads))


# ---------------------------------------------------------------------------
# Theoretical response helpers (R2 stage-1)
# ---------------------------------------------------------------------------


def second_order_response(
    stiffness_k: float,
    damping_d: float,
    effective_inertia_j: float,
) -> Tuple[float, float]:
    """Closed-loop second-order response for joint impedance.

    Returns ``(omega_n, zeta)`` where
    ``omega_n = sqrt(K / J)`` and ``zeta = D / (2 * sqrt(K * J))``.

    Raises ``ValueError`` on non-physical inputs so the R2 test fails
    at expectation-load time rather than silently producing ``nan``
    during assertion.
    """
    if stiffness_k <= 0.0:
        raise ValueError(f"stiffness K must be > 0 (got {stiffness_k})")
    if effective_inertia_j <= 0.0:
        raise ValueError(f"effective inertia J must be > 0 (got {effective_inertia_j})")
    if damping_d < 0.0:
        raise ValueError(f"damping D must be >= 0 (got {damping_d})")
    omega_n = math.sqrt(stiffness_k / effective_inertia_j)
    zeta = damping_d / (2.0 * math.sqrt(stiffness_k * effective_inertia_j))
    return omega_n, zeta


def damping_ratio_within_band(
    measured_zeta: float, theoretical_zeta: float, tolerance_pct: float
) -> bool:
    """True iff ``measured_zeta`` is within ``tolerance_pct`` of theory.

    ``tolerance_pct`` is expressed as a whole-percent band
    (``20.0`` means ±20%) to match the ``damping_ratio_pct`` key in
    the expectation YAMLs. Absolute band around zero is ignored: at
    ``theoretical_zeta == 0`` only an exact match passes, which is
    the correct failure mode (undamped theory ⇒ any measured damping
    is a modelling error).
    """
    if tolerance_pct < 0.0:
        raise ValueError(f"tolerance_pct must be >= 0 (got {tolerance_pct})")
    if theoretical_zeta == 0.0:
        return measured_zeta == 0.0
    frac = tolerance_pct / 100.0
    lo = theoretical_zeta * (1.0 - frac)
    hi = theoretical_zeta * (1.0 + frac)
    return lo <= measured_zeta <= hi


__all__: Sequence[str] = (
    "ArmExpectation",
    "CANONICAL_JOINTS",
    "ControllerExpectation",
    "EXPECT_DIR",
    "JointExpectation",
    "PAYLOAD_LEVELS",
    "Payload",
    "PayloadCatalog",
    "REPO_ROOT",
    "Stage2Tolerances",
    "Stage2TcpTolerances",
    "SUPPORTED_ARMS",
    "damping_ratio_within_band",
    "load_arm",
    "load_payloads",
    "second_order_response",
)
