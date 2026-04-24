"""R2 stage-3 cartesian gain resolver: bringup YAML -> per-axis stiffness.

Pre-baked helper for the R2 integration test matrix (M6.14, still gated
on the M6.0 operator decision). Closes the bringup-controller-YAML ->
future stage-3 theoretical-response seam for the one shipped Cartesian
**impedance** controller so stage-3 test bodies for
``crisp_cartesian_impedance`` don't open-code ``yaml.safe_load`` + a
nested ``task.k_{pos,rot}_{x,y,z}`` key walk.

This module is the Cartesian counterpart of :mod:`r2_stage1_gains`,
which handles the two joint-space impedance controllers.

Background
----------

``crisp_controllers/CartesianController`` in its ``cartesian_impedance``
role (see ``docs/crisp_controllers.md`` and
``bringup/config/crisp_cartesian_impedance.{ur5e,ur15}.yaml``) exposes
six scalar Cartesian task stiffnesses under
``cartesian_impedance_controller.ros__parameters.task``:

* ``k_pos_x`` / ``k_pos_y`` / ``k_pos_z`` — translational stiffness
  (N/m) per TCP axis expressed in the base frame.
* ``k_rot_x`` / ``k_rot_y`` / ``k_rot_z`` — rotational stiffness
  (Nm/rad) per TCP axis.

The ``task`` block does not expose damping terms directly — the
controller derives cartesian task damping from the stiffness + an
internal critical-damping convention — so this resolver returns
stiffness only. Nullspace PD gains (``nullspace.stiffness`` /
``nullspace.damping``) and joint-limit-repulsion fields are
intentionally out of scope here; :mod:`r2_stage1_gains` already covers
the nullspace pair for the joint-impedance role, and the cartesian
role's nullspace block is a secondary behaviour (not the TCP task
response stage-3 will assert against).

Scope (intentionally narrow)
----------------------------

* One controller: ``crisp_cartesian_impedance``. ``cartesian_motion``
  is a **position-mode** controller whose ``pd_gains.{trans,rot}_{x,y,z}.p``
  fields are IK-solver proportional gains (not Cartesian stiffnesses)
  with different units and different closed-loop semantics; mixing them
  into the same resolver would invite silent misuse. If a future
  stage-3 test body needs those, it gets its own narrow resolver.
* Two arms: ``ur5e`` and ``ur15``. Matches
  :data:`expectations_loader.SUPPORTED_ARMS`.
* No reading of expectation YAMLs, no ROS / MuJoCo / numpy imports.
  ``yaml`` (PyYAML) is already a project dep (see
  ``bringup/launch/cartesian_bringup.launch.py`` and repo memory
  "python deps").

Contract
--------

:func:`resolve_cartesian_impedance_gains` returns a plain :class:`dict`
with a fixed two-key shape::

    {
        "translational": {"x": K_tx, "y": K_ty, "z": K_tz},
        "rotational":    {"x": K_rx, "y": K_ry, "z": K_rz},
    }

All six values are finite non-negative :class:`float`. Key order for
both sub-dicts is fixed: ``("x", "y", "z")``.

Strict validation on load:

* Unknown controller / arm -> :class:`ValueError`.
* Missing YAML file -> :class:`FileNotFoundError`.
* Missing required key -> :class:`KeyError` with the dotted path.
* Non-mapping top-level / ``task`` block -> :class:`ValueError`.
* Non-numeric (including :class:`bool`) stiffness -> :class:`TypeError`.
* Non-finite or negative stiffness -> :class:`ValueError`.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Dict

import yaml

__all__ = (
    "SUPPORTED_CONTROLLERS",
    "SUPPORTED_ARMS",
    "AXES",
    "DEFAULT_CONFIG_DIR",
    "resolve_cartesian_impedance_gains",
)


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG_DIR = REPO_ROOT / "bringup" / "config"

SUPPORTED_CONTROLLERS = ("crisp_cartesian_impedance",)

SUPPORTED_ARMS = ("ur5e", "ur15")

AXES = ("x", "y", "z")

_TOP_KEY = {
    "crisp_cartesian_impedance": "cartesian_impedance_controller",
}


def _raise_value(msg: str) -> None:
    raise ValueError(f"r2_stage3_cartesian_gains: {msg}")


def _require_key(d, key: str, path: str):
    if not isinstance(d, dict):
        raise KeyError(
            f"r2_stage3_cartesian_gains: expected mapping at '{path}', " f"got {type(d).__name__}"
        )
    if key not in d:
        raise KeyError(f"r2_stage3_cartesian_gains: missing key '{path}.{key}'")
    return d[key]


def _as_finite_nonneg_float(value, path: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(
            f"r2_stage3_cartesian_gains: {path} must be a real number, "
            f"got {type(value).__name__}"
        )
    f = float(value)
    if not math.isfinite(f):
        _raise_value(f"{path} must be finite, got {f!r}")
    if f < 0.0:
        _raise_value(f"{path} must be non-negative, got {f!r}")
    return f


def _load_yaml(path: Path) -> dict:
    if not path.is_file():
        raise FileNotFoundError(f"r2_stage3_cartesian_gains: controller config not found at {path}")
    loaded = yaml.safe_load(path.read_text())
    if not isinstance(loaded, dict):
        _raise_value(f"{path}: expected top-level mapping, got {type(loaded).__name__}")
    return loaded


def resolve_cartesian_impedance_gains(
    controller: str,
    arm: str,
    *,
    config_dir: Path | None = None,
) -> Dict[str, Dict[str, float]]:
    """Resolve per-axis Cartesian task stiffnesses from a bringup YAML.

    Parameters
    ----------
    controller:
        One of :data:`SUPPORTED_CONTROLLERS`. Currently only
        ``"crisp_cartesian_impedance"``.
    arm:
        One of :data:`SUPPORTED_ARMS`.
    config_dir:
        Directory containing ``{controller}.{arm}.yaml``. Defaults to
        :data:`DEFAULT_CONFIG_DIR` (``bringup/config`` under the repo
        root). Passed explicitly in tests so fixtures can point at a
        temp dir.

    Returns
    -------
    dict
        Nested mapping of the shape described in the module docstring.

    Raises
    ------
    ValueError
        Unknown ``controller`` or ``arm``, or a malformed YAML field.
    FileNotFoundError
        The resolved YAML path does not exist.
    KeyError
        A required YAML key is missing.
    TypeError
        A stiffness entry is not a real number.
    """
    if controller not in SUPPORTED_CONTROLLERS:
        _raise_value(f"unsupported controller {controller!r}; supported: {SUPPORTED_CONTROLLERS}")
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
        _raise_value(f"{top_key}.ros__parameters must be a mapping, got {type(params).__name__}")
    task = _require_key(params, "task", f"{top_key}.ros__parameters")
    if not isinstance(task, dict):
        _raise_value(f"{top_key}.ros__parameters.task must be a mapping, got {type(task).__name__}")

    translational: Dict[str, float] = {}
    rotational: Dict[str, float] = {}
    for axis in AXES:
        pos_key = f"k_pos_{axis}"
        rot_key = f"k_rot_{axis}"
        translational[axis] = _as_finite_nonneg_float(
            _require_key(task, pos_key, f"{top_key}.ros__parameters.task"),
            f"{top_key}.ros__parameters.task.{pos_key}",
        )
        rotational[axis] = _as_finite_nonneg_float(
            _require_key(task, rot_key, f"{top_key}.ros__parameters.task"),
            f"{top_key}.ros__parameters.task.{rot_key}",
        )

    return {"translational": translational, "rotational": rotational}
