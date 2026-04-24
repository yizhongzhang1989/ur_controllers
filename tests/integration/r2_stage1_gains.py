"""R2 stage-1 gain resolver: bringup controller YAML -> per-joint ``(K, D)``.

Pre-baked helper for the R2 integration test matrix (M6.12, still gated
on the M6.0 operator decision). Closes the last remaining seam on the
R2 stage-1 pre-bake chain: turning the on-disk controller configs
(``bringup/config/*.yaml``) into the ``stiffness_k`` / ``damping_d``
arguments that :func:`r2_stage1_theoretical.theoretical_for_stage1`
requires for the ``second_order`` response model.

Background
----------

``r2_stage1_theoretical`` intentionally keeps K / D caller-supplied
("matches :func:`expectations_loader.second_order_response`"). That
leaves each R2 stage-1 test body responsible for loading the
controller YAML and extracting per-joint gains — the same
``yaml.safe_load`` + key-walk that would otherwise be open-coded in
the `crisp_joint_impedance` and `simple_joint_impedance` tests. This
module is that shared extractor, one place that knows the two
supported impedance controllers' YAML shapes.

Scope (intentionally narrow)
----------------------------

* Two controllers only:

  * ``simple_joint_impedance`` — per-joint ``k[i]`` / ``d[i]`` arrays
    under ``simple_joint_impedance_controller.ros__parameters``.
  * ``crisp_joint_impedance`` — scalar ``nullspace.stiffness`` /
    ``nullspace.damping`` under
    ``joint_impedance_controller.ros__parameters`` (the crisp joint-
    impedance role uses the nullspace PD with all Cartesian task
    stiffnesses zeroed; see ``bringup/config/crisp_joint_impedance.*.yaml``
    and ``docs/crisp_controllers.md``). Scalar K / D broadcast to all
    joints.

* Two arms: ``ur5e`` and ``ur15``. Matches
  :data:`expectations_loader.SUPPORTED_ARMS`.

* Damping auto-fill: a negative ``d[i]`` entry (or negative scalar
  ``nullspace.damping``) resolves to critical damping ``2*sqrt(K)`` —
  matching both the simple controller's on-activation rule
  (`src/simple_joint_impedance_controller/src/simple_joint_impedance_controller.yaml`
  `d` parameter description) and crisp's nullspace convention
  (`nullspace.damping: -1.0` → "auto: 2*sqrt(stiffness)").

Non-goals
---------

* No reading of expectation YAMLs — callers still thread a
  :class:`expectations_loader.JointExpectation` into
  ``theoretical_for_stage1`` separately.
* No reading of cartesian-impedance / gravity-compensation YAMLs — R2
  stage-1 tests only drive joint-impedance controllers; the other
  crisp roles will get their own resolver when R2 stage-3 lands.
* No evaluation of the ros2_param substitutions (``$(var …)``). The
  bringup YAMLs do not use them for gain fields.
* No ROS / MuJoCo / numpy imports. ``yaml`` (PyYAML) is already a
  project dep (see ``bringup/launch/cartesian_bringup.launch.py`` and
  repo memory "python deps").

Contract
--------

:func:`resolve_joint_impedance_gains` returns a plain :class:`dict`
mapping joint name -> ``(K, D)`` tuple of plain ``float`` values. Key
order follows the ``joints`` list in the source YAML (which matches
:data:`CANONICAL_JOINTS` for both supported controllers, but we
preserve the YAML order rather than hard-coding it). Strict
validation on load:

* Unknown controller / arm → :class:`ValueError`.
* Missing YAML file → :class:`FileNotFoundError`.
* Missing required key → :class:`KeyError` with the dotted path.
* Length mismatch between ``joints`` / ``k`` / ``d`` (simple) →
  :class:`ValueError`.
* Duplicate joint names → :class:`ValueError`.
* Non-finite or negative ``K`` → :class:`ValueError` (K is strictly
  non-negative per the parameter validation in
  ``simple_joint_impedance_controller.yaml``).
* Non-finite ``D`` → :class:`ValueError` (a negative ``D`` is a valid
  request for auto-fill, not an error).
* Non-numeric ``k`` / ``d`` entries → :class:`TypeError`.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Dict, Tuple

import yaml

__all__ = (
    "SUPPORTED_CONTROLLERS",
    "SUPPORTED_ARMS",
    "CANONICAL_JOINTS",
    "DEFAULT_CONFIG_DIR",
    "resolve_joint_impedance_gains",
)


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG_DIR = REPO_ROOT / "bringup" / "config"

SUPPORTED_CONTROLLERS: Tuple[str, ...] = (
    "simple_joint_impedance",
    "crisp_joint_impedance",
)

SUPPORTED_ARMS: Tuple[str, ...] = ("ur5e", "ur15")

# Kept in lockstep with :data:`expectations_loader.CANONICAL_JOINTS` so
# a drift in either module fails the cross-check unit test below.
CANONICAL_JOINTS: Tuple[str, ...] = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)


# Per-controller YAML structure:
#   top-level key → "<top>.ros__parameters"
_TOP_KEY = {
    "simple_joint_impedance": "simple_joint_impedance_controller",
    "crisp_joint_impedance": "joint_impedance_controller",
}


def _raise_value(msg: str) -> None:
    raise ValueError(f"r2_stage1_gains: {msg}")


def _require_key(d, key: str, path: str):
    if not isinstance(d, dict):
        raise KeyError(f"r2_stage1_gains: expected mapping at '{path}', got " f"{type(d).__name__}")
    if key not in d:
        raise KeyError(f"r2_stage1_gains: missing key '{path}.{key}'")
    return d[key]


def _as_finite_float(value, path: str, *, allow_negative: bool) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(
            f"r2_stage1_gains: {path} must be a real number, got " f"{type(value).__name__}"
        )
    f = float(value)
    if not math.isfinite(f):
        _raise_value(f"{path} must be finite, got {f!r}")
    if not allow_negative and f < 0.0:
        _raise_value(f"{path} must be non-negative, got {f!r}")
    return f


def _auto_fill_damping(k: float, d: float) -> float:
    """Return ``d`` if non-negative, otherwise critical damping ``2*sqrt(K)``.

    Matches both the simple controller's activation rule and crisp's
    ``nullspace.damping: -1.0`` sentinel. ``K`` is already validated
    as finite and non-negative by the caller.
    """
    if d < 0.0:
        return 2.0 * math.sqrt(k)
    return d


def _load_yaml(path: Path) -> dict:
    if not path.is_file():
        raise FileNotFoundError(f"r2_stage1_gains: controller config not found at {path}")
    loaded = yaml.safe_load(path.read_text())
    if not isinstance(loaded, dict):
        _raise_value(f"{path}: expected top-level mapping, got {type(loaded).__name__}")
    return loaded


def _validate_joints(joints, path: str) -> Tuple[str, ...]:
    if not isinstance(joints, list) or not joints:
        _raise_value(f"{path} must be a non-empty list")
    seen = set()
    out = []
    for i, j in enumerate(joints):
        if not isinstance(j, str) or not j:
            _raise_value(f"{path}[{i}] must be a non-empty string, got {j!r}")
        if j in seen:
            _raise_value(f"{path} contains duplicate joint name '{j}'")
        seen.add(j)
        out.append(j)
    return tuple(out)


def _resolve_simple(params: dict) -> Dict[str, Tuple[float, float]]:
    joints = _validate_joints(
        _require_key(params, "joints", "simple_joint_impedance_controller.ros__parameters"),
        "simple_joint_impedance_controller.ros__parameters.joints",
    )
    k_arr = _require_key(params, "k", "simple_joint_impedance_controller.ros__parameters")
    d_arr = _require_key(params, "d", "simple_joint_impedance_controller.ros__parameters")
    if not isinstance(k_arr, list) or not isinstance(d_arr, list):
        _raise_value("simple_joint_impedance_controller.ros__parameters.{k,d} must be lists")
    if len(k_arr) != len(joints) or len(d_arr) != len(joints):
        _raise_value(
            "simple_joint_impedance_controller.ros__parameters.{k,d,joints} "
            f"must have matching length; got joints={len(joints)}, "
            f"k={len(k_arr)}, d={len(d_arr)}"
        )
    out: Dict[str, Tuple[float, float]] = {}
    for i, name in enumerate(joints):
        k = _as_finite_float(
            k_arr[i],
            f"simple_joint_impedance_controller.ros__parameters.k[{i}]",
            allow_negative=False,
        )
        d_raw = _as_finite_float(
            d_arr[i],
            f"simple_joint_impedance_controller.ros__parameters.d[{i}]",
            allow_negative=True,
        )
        out[name] = (k, _auto_fill_damping(k, d_raw))
    return out


def _resolve_crisp(params: dict) -> Dict[str, Tuple[float, float]]:
    joints = _validate_joints(
        _require_key(params, "joints", "joint_impedance_controller.ros__parameters"),
        "joint_impedance_controller.ros__parameters.joints",
    )
    nullspace = _require_key(params, "nullspace", "joint_impedance_controller.ros__parameters")
    if not isinstance(nullspace, dict):
        _raise_value("joint_impedance_controller.ros__parameters.nullspace must be a mapping")
    k_scalar = _require_key(
        nullspace,
        "stiffness",
        "joint_impedance_controller.ros__parameters.nullspace",
    )
    d_scalar = _require_key(
        nullspace,
        "damping",
        "joint_impedance_controller.ros__parameters.nullspace",
    )
    k = _as_finite_float(
        k_scalar,
        "joint_impedance_controller.ros__parameters.nullspace.stiffness",
        allow_negative=False,
    )
    d_raw = _as_finite_float(
        d_scalar,
        "joint_impedance_controller.ros__parameters.nullspace.damping",
        allow_negative=True,
    )
    d = _auto_fill_damping(k, d_raw)
    return {name: (k, d) for name in joints}


_RESOLVERS = {
    "simple_joint_impedance": _resolve_simple,
    "crisp_joint_impedance": _resolve_crisp,
}


def resolve_joint_impedance_gains(
    controller: str,
    arm: str,
    *,
    config_dir: Path | None = None,
) -> Dict[str, Tuple[float, float]]:
    """Resolve per-joint ``(K, D)`` gains from a bringup controller YAML.

    Parameters
    ----------
    controller:
        One of :data:`SUPPORTED_CONTROLLERS`.
    arm:
        One of :data:`SUPPORTED_ARMS`.
    config_dir:
        Directory containing ``{controller}.{arm}.yaml``. Defaults to
        :data:`DEFAULT_CONFIG_DIR` (``bringup/config`` under the repo
        root). Passed explicitly in tests so fixtures can point at a
        temp dir.

    Returns
    -------
    dict[str, tuple[float, float]]
        Mapping from joint name to ``(K, D)``. Key order follows the
        YAML's ``joints`` list (insertion-ordered :class:`dict`).

    Raises
    ------
    ValueError
        Unknown ``controller`` or ``arm``, or a malformed YAML field.
    FileNotFoundError
        The resolved YAML path does not exist.
    KeyError
        A required YAML key is missing.
    TypeError
        A ``k`` or ``d`` entry is not a real number.
    """
    if controller not in SUPPORTED_CONTROLLERS:
        _raise_value(
            f"unsupported controller {controller!r}; " f"supported: {SUPPORTED_CONTROLLERS}"
        )
    if arm not in SUPPORTED_ARMS:
        _raise_value(f"unsupported arm {arm!r}; supported: {SUPPORTED_ARMS}")
    base = Path(config_dir) if config_dir is not None else DEFAULT_CONFIG_DIR
    path = base / f"{controller}.{arm}.yaml"
    loaded = _load_yaml(path)

    top_key = _TOP_KEY[controller]
    top = _require_key(loaded, top_key, "<root>")
    if not isinstance(top, dict):
        _raise_value(f"{top_key} must be a mapping, got {type(top).__name__}")
    params = _require_key(top, "ros__parameters", top_key)
    if not isinstance(params, dict):
        _raise_value(
            f"{top_key}.ros__parameters must be a mapping, got " f"{type(params).__name__}"
        )

    return _RESOLVERS[controller](params)
