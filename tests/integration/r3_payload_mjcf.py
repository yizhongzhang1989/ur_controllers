"""MJCF body snippet emitter for R3 end-effector payloads.

Pre-baked consumer API for **M6.17** (MJCF payload attachment). Given
a validated :class:`~tests.integration.expectations_loader.Payload`,
:func:`payload_body_mjcf` returns a string containing a MuJoCo
``<body>`` element ready to splice into an MJCF tree as a child of the
anchor body (typically ``tool0``). Zero-mass payloads return the empty
string, honouring the ROADMAP-R3 rule "zero-mass case = no body
injected" so the ``no_payload`` baseline leaves the MJCF byte-identical
to the pre-attachment tree.

Scope (v1, deliberately narrow)
-------------------------------

* Pure stdlib. No numpy, no ROS, no MuJoCo. The module imports only
  :mod:`math` and the sibling :mod:`payload_validation` / loader
  dataclass, so ROS-free tooling (CI linters, doc generators) can
  consume it without extra deps.
* Emits MuJoCo's standard attributes: ``pos`` (metres) and ``quat``
  (w x y z, Hamilton convention), plus a single ``<inertial>`` child
  with ``mass`` and ``diaginertia``. The v1 validator guarantees the
  inertia tensor is diagonal (``ixy == ixz == iyz == 0``), so
  ``diaginertia`` is always safe; a non-diagonal input is rejected
  upstream before it reaches this emitter.
* ``quat`` is used in preference to ``euler`` so the result is
  independent of the enclosing MJCF's ``<compiler eulerseq="...">``
  setting. The conversion uses the *intrinsic* XYZ convention that
  matches the loader's ``pose_rpy`` semantics and :mod:`scipy.spatial`
  / ``tf_transformations`` defaults. Callers that want to hand-check
  the output can reconstruct the quaternion as
  ``qx(r) ⊗ qy(p) ⊗ qz(y)`` (Hamilton product, ``q_w`` first).
* The returned snippet is self-contained and well-formed XML; callers
  may splice it via string concatenation *or* parse it with
  :mod:`xml.etree.ElementTree` and re-attach to their own tree. No
  trailing newline, no XML declaration, no namespace — MJCF uses the
  empty XML namespace.

Non-goals
---------

* Computing inertia tensors. The validator pins the input shape as
  "diagonal tensor, expressed in the payload's local frame at its
  COM"; reframing into ``tool0`` coordinates is physics, not string
  assembly, and lives outside this module.
* Building the whole MJCF document. M6.17 will either regenerate the
  MJCF on every payload change (recommended) or mutate the existing
  tree; in both cases this module only produces the leaf snippet.
* Validating the anchor body. The caller is responsible for knowing
  which body (``tool0``) it splices the snippet under. We accept a
  ``body_name`` attribute only, which names the *new* body, not the
  anchor.

Raises
------
:class:`ValueError` — propagated from :func:`payload_validation.validate_payload`
    when the payload fails a physical-validity check; re-raised with
    its message intact so the caller sees the root cause.
:class:`ValueError`
    when ``body_name`` is empty, not a string, or contains whitespace
    (MJCF identifiers are whitespace-free; allowing whitespace would
    silently break downstream MJCF parsers).
"""

from __future__ import annotations

import math
from typing import Tuple

try:  # Direct import when tests/integration/ is on sys.path.
    from expectations_loader import Payload
    from payload_validation import validate_payload
except ImportError:  # pragma: no cover - defensive fallback for file-path loaders.
    from tests.integration.expectations_loader import Payload  # type: ignore[no-redef]
    from tests.integration.payload_validation import (  # type: ignore[no-redef]
        validate_payload,
    )

__all__ = ("payload_body_mjcf", "DEFAULT_BODY_NAME")

DEFAULT_BODY_NAME = "ee_payload"


def _fmt(x: float) -> str:
    # %.15g gives round-trip precision for IEEE-754 doubles while
    # avoiding tails of noise zeros that %r / str() would produce for
    # exact values like 0.0 or 1.0.
    return format(float(x), ".15g")


def _euler_xyz_to_wxyz(rpy: Tuple[float, float, float]) -> Tuple[float, float, float, float]:
    """Intrinsic XYZ Euler (radians) -> Hamilton quaternion (w, x, y, z).

    Matches ``scipy.spatial.transform.Rotation.from_euler('xyz', rpy).as_quat()``
    after reordering to (w, x, y, z) (scipy returns x, y, z, w).
    """
    r, p, y = rpy
    hr, hp, hy = 0.5 * r, 0.5 * p, 0.5 * y
    cr, sr = math.cos(hr), math.sin(hr)
    cp, sp = math.cos(hp), math.sin(hp)
    cy, sy = math.cos(hy), math.sin(hy)

    # q = qx(r) ⊗ qy(p) ⊗ qz(y) under Hamilton (ijk = -1).
    #   qx = (cr, sr, 0, 0)
    #   qy = (cp, 0, sp, 0)
    #   qz = (cy, 0, 0, sy)
    # Closed-form product:
    w = cr * cp * cy - sr * sp * sy
    x = sr * cp * cy + cr * sp * sy
    y_ = cr * sp * cy - sr * cp * sy
    z = cr * cp * sy + sr * sp * cy
    return (w, x, y_, z)


def _validate_body_name(name: object) -> str:
    if not isinstance(name, str):
        raise ValueError(f"body_name must be str, got {type(name).__name__}")
    if not name:
        raise ValueError("body_name must be non-empty")
    if any(ch.isspace() for ch in name):
        raise ValueError(f"body_name must not contain whitespace: {name!r}")
    return name


def payload_body_mjcf(
    payload: Payload,
    *,
    body_name: str = DEFAULT_BODY_NAME,
) -> str:
    """Emit an MJCF ``<body>`` snippet for ``payload``.

    Parameters
    ----------
    payload:
        A validated :class:`Payload`. Re-validated on entry; if the
        input is malformed the :class:`ValueError` raised by the
        validator propagates.
    body_name:
        Name attribute for the emitted ``<body>``. Must be a non-empty
        whitespace-free string. Defaults to :data:`DEFAULT_BODY_NAME`
        (``"ee_payload"``).

    Returns
    -------
    str
        An MJCF snippet, or the empty string when
        ``payload.mass_kg == 0`` (which the validator pins to imply an
        all-zero inertia tensor). The caller splices the snippet as a
        child of the anchor body (typically ``tool0``).
    """
    validate_payload(payload)
    body_name = _validate_body_name(body_name)

    if payload.mass_kg == 0.0:
        # Sentinel: zero mass <=> zero inertia (validator guarantees).
        # Emit no body at all, so the ``no_payload`` baseline does not
        # touch the MJCF.
        return ""

    x, y, z = payload.pose_xyz
    qw, qx, qy, qz = _euler_xyz_to_wxyz(payload.pose_rpy)
    ixx = payload.inertia_kg_m2["ixx"]
    iyy = payload.inertia_kg_m2["iyy"]
    izz = payload.inertia_kg_m2["izz"]

    return (
        f'<body name="{body_name}" '
        f'pos="{_fmt(x)} {_fmt(y)} {_fmt(z)}" '
        f'quat="{_fmt(qw)} {_fmt(qx)} {_fmt(qy)} {_fmt(qz)}">'
        f'<inertial pos="0 0 0" mass="{_fmt(payload.mass_kg)}" '
        f'diaginertia="{_fmt(ixx)} {_fmt(iyy)} {_fmt(izz)}"/>'
        f"</body>"
    )
