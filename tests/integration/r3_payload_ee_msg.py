"""Pre-baked message-shape builder for ``ur_sim_msgs/EePayload`` (M6.16).

Pure-stdlib converter from the loader's
:class:`~tests.integration.expectations_loader.Payload` dataclass to the
payload-API message shape ADR-0012 addendum §R3.1 specifies for
``ur_sim_msgs/EePayload``:

* ``mass`` — double, kg.
* ``inertia`` — 9 doubles, kg·m², row-major, expressed at the payload's
  COM and in its local frame.
* ``pose`` — :class:`geometry_msgs/Pose`; position ``(x, y, z)`` metres
  plus orientation :class:`geometry_msgs/Quaternion` ordered
  ``(x, y, z, w)``.

This module closes the last pre-bake seam for M6.16 (payload runtime
API) without importing ROS, numpy, or MuJoCo. A later ROS-side adapter
materialises :class:`EePayloadMessage` as an ``ur_sim_msgs/EePayload``
message at test-run time; keeping the transform here means the unit
gate pins the data shape without the unit-gate pulling in
``rclpy`` / ``ur_sim_msgs``.

Design choices
--------------

* **Frozen dataclass contract.** :class:`EePayloadMessage` mirrors the
  eventual ROS message field-by-field. Callers that need a plain dict
  can call :meth:`EePayloadMessage.to_dict`; callers building the ROS
  message fill fields one-for-one.
* **Inertia as symmetric 9-double row-major.** The v1 validator pins
  ``ixy == ixz == iyz == 0`` but the ROS message carries the full
  3×3 tensor. We populate the symmetric layout
  ``[ixx, ixy, ixz, ixy, iyy, iyz, ixz, iyz, izz]`` so the builder
  keeps working if the validator is ever relaxed; today it degenerates
  to a diagonal tensor with zero off-diagonals.
* **Quaternion via the shared emitter helper.** Reuses
  :func:`r3_payload_mjcf._euler_xyz_to_wxyz` (the closed-form
  intrinsic-XYZ → Hamilton ``w,x,y,z`` product) and reorders to
  ``(x, y, z, w)`` for :class:`geometry_msgs/Quaternion`. Keeping one
  source of truth for the Euler convention avoids drift between the
  MJCF-side and ROS-side representations of the same rotation.
* **Zero-mass payload still produces a valid message.** ADR-0012 §R3.2
  pins the initial sim state as zero mass — the topic must be
  publishable in that state. We do **not** canonicalize the pose to
  identity; the loader allows non-zero pose with zero mass (the
  validator only ties mass to inertia), and silently overriding that
  would be a data-loss bug rather than a shape transform.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

try:  # Direct import when tests/integration/ is on sys.path.
    from expectations_loader import Payload
    from payload_validation import validate_payload
    from r3_payload_mjcf import _euler_xyz_to_wxyz
except ImportError:  # pragma: no cover - defensive fallback for file-path loaders.
    from tests.integration.expectations_loader import Payload  # type: ignore[no-redef]
    from tests.integration.payload_validation import (  # type: ignore[no-redef]
        validate_payload,
    )
    from tests.integration.r3_payload_mjcf import (  # type: ignore[no-redef]
        _euler_xyz_to_wxyz,
    )

__all__ = ("EePayloadMessage", "payload_to_ee_msg")


@dataclass(frozen=True)
class EePayloadMessage:
    """Frozen shape of ``ur_sim_msgs/EePayload``.

    Fields mirror the ROS message field-for-field so a ROS-side
    adapter can populate a real message without shape surprises.

    Attributes
    ----------
    mass_kg:
        Scalar payload mass, kilograms. Non-negative (validator
        enforced).
    inertia_row_major:
        3×3 inertia tensor flattened row-major, kg·m², expressed at
        the payload's COM in the payload's local frame. Symmetric:
        ``[0]==ixx, [4]==iyy, [8]==izz``; ``[1]==[3]==ixy``;
        ``[2]==[6]==ixz``; ``[5]==[7]==iyz``.
    pose_position_xyz:
        Translation of the payload frame w.r.t. ``tool0``, metres.
    pose_orientation_xyzw:
        Unit quaternion rotating ``tool0`` into the payload frame,
        ordered ``(x, y, z, w)`` to match
        :class:`geometry_msgs/Quaternion`.
    """

    mass_kg: float
    inertia_row_major: Tuple[float, float, float, float, float, float, float, float, float]
    pose_position_xyz: Tuple[float, float, float]
    pose_orientation_xyzw: Tuple[float, float, float, float]

    def to_dict(self) -> dict:
        """Return a plain dict view, keys shaped like the ROS message."""
        x, y, z = self.pose_position_xyz
        qx, qy, qz, qw = self.pose_orientation_xyzw
        return {
            "mass": self.mass_kg,
            "inertia": list(self.inertia_row_major),
            "pose": {
                "position": {"x": x, "y": y, "z": z},
                "orientation": {"x": qx, "y": qy, "z": qz, "w": qw},
            },
        }


def payload_to_ee_msg(payload: Payload) -> EePayloadMessage:
    """Convert a validated :class:`Payload` to an :class:`EePayloadMessage`.

    Re-runs :func:`payload_validation.validate_payload` on entry, so
    malformed inputs are rejected at this seam rather than producing a
    silently-wrong message. The emitted inertia is the symmetric
    row-major layout; the orientation is the intrinsic-XYZ Euler →
    Hamilton quaternion reordered to ``(x, y, z, w)``.

    Zero-mass payloads return a well-formed message with zero mass,
    zero inertia, and the loader-supplied pose. ADR-0012 §R3.2 pins
    zero mass as the sim's initial-launch state, so the topic must be
    publishable in that state.

    Raises
    ------
    ValueError
        Propagated from :func:`validate_payload` when the input fails
        a physical-validity check.
    """
    validate_payload(payload)

    i = payload.inertia_kg_m2
    ixx, iyy, izz = i["ixx"], i["iyy"], i["izz"]
    ixy, ixz, iyz = i["ixy"], i["ixz"], i["iyz"]
    inertia_row_major = (
        ixx,
        ixy,
        ixz,
        ixy,
        iyy,
        iyz,
        ixz,
        iyz,
        izz,
    )

    qw, qx, qy, qz = _euler_xyz_to_wxyz(payload.pose_rpy)
    pose_orientation_xyzw = (qx, qy, qz, qw)

    return EePayloadMessage(
        mass_kg=float(payload.mass_kg),
        inertia_row_major=inertia_row_major,
        pose_position_xyz=tuple(float(v) for v in payload.pose_xyz),  # type: ignore[arg-type]
        pose_orientation_xyzw=pose_orientation_xyzw,
    )
