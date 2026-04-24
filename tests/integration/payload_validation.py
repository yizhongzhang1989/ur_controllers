"""Physical-validity validator for R3 end-effector payloads.

Pre-baked consumer API for M6.16 (``~/set_ee_payload`` service) and
M6.17 (MJCF body attachment). Both will feed caller-supplied payload
descriptors through this validator before accepting them, so any
nonsense (negative mass, non-diagonal inertia, a NaN anywhere, a
triangle-inequality violation) is rejected at a single seam with a
clear message instead of silently producing a broken MuJoCo scene or
an unstable controller.

Scope (v1, intentionally narrow):

* Enforce finiteness and non-negativity of mass.
* Enforce finiteness of every inertia tensor entry.
* Enforce a *diagonal* inertia tensor (``ixy == ixz == iyz == 0``
  exactly). M6.17 targets MuJoCo's ``diaginertia`` form, which
  requires the tensor to be expressed in its principal-axis frame;
  non-diagonal inputs must be diagonalised upstream before they
  reach this validator. A v2 may add a general-SPD path (Sylvester +
  closed-form 3x3 symmetric eigenvalues) behind an explicit API
  change if a caller ever needs it; none do today.
* Enforce the sentinel consistency ``mass == 0 <=> inertia == 0``.
* Enforce the rigid-body modelling constraint ``mass > 0 =>`` every
  principal moment ``> 0``. This rules out point masses; we treat
  that as a v1 policy decision, not a claim of physical
  impossibility.
* Enforce the triangle inequality on the principal moments:
  ``I_xx + I_yy >= I_zz`` and the two cyclic permutations. A
  symmetric inertia tensor whose eigenvalues violate this is not
  realisable by any mass distribution; rejecting it here keeps the
  physics consistent before we ever touch MuJoCo.
* Enforce finiteness of the pose (``pose_xyz``, ``pose_rpy``).

Non-goals:

* Validating that the pose lies within the arm's workspace. That is
  a kinematic reachability question, not a payload-integrity one.
* Checking that ``inertia_kg_m2`` is expressed in the pose frame
  (``pose_xyz``/``pose_rpy``) rather than ``tool0``. The loader
  pins this as "inertia at the payload COM in the payload local
  frame"; we trust that contract.
* Numpy / scipy. Pure stdlib so the unit gate stays fast and this
  module can be imported by ROS-free tooling.

Raises
------
``ValueError`` with a self-contained message naming the offending
field and value. Callers log it; they do not introspect it.
"""

from __future__ import annotations

import math
from typing import Iterable

try:  # Direct import when tests/integration/ is on sys.path.
    from expectations_loader import Payload, PayloadCatalog
except ImportError:  # pragma: no cover - defensive fallback for file-path loaders.
    from tests.integration.expectations_loader import (  # type: ignore[no-redef]
        Payload,
        PayloadCatalog,
    )


__all__ = ("validate_payload", "validate_catalog")


_INERTIA_DIAG_KEYS: tuple[str, ...] = ("ixx", "iyy", "izz")
_INERTIA_OFFDIAG_KEYS: tuple[str, ...] = ("ixy", "ixz", "iyz")
_INERTIA_KEYS: tuple[str, ...] = _INERTIA_DIAG_KEYS + _INERTIA_OFFDIAG_KEYS


def _require_finite(name: str, value: float) -> float:
    try:
        v = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name}: not a real number ({value!r})") from exc
    if not math.isfinite(v):
        raise ValueError(f"{name}: must be finite, got {v!r}")
    return v


def _require_finite_triplet(name: str, xs: Iterable[float]) -> tuple[float, float, float]:
    seq = tuple(xs)
    if len(seq) != 3:
        raise ValueError(f"{name}: expected 3 elements, got {len(seq)}")
    return (
        _require_finite(f"{name}[0]", seq[0]),
        _require_finite(f"{name}[1]", seq[1]),
        _require_finite(f"{name}[2]", seq[2]),
    )


def validate_payload(payload: Payload) -> None:
    """Raise ``ValueError`` if *payload* is not a valid v1 rigid body.

    Parameters
    ----------
    payload:
        A :class:`~expectations_loader.Payload` (frozen dataclass).
        ``payload.name`` is echoed into every error message so a
        validator failing on one entry in a catalog is self-locating.

    Notes
    -----
    Diagonal-only inertia is a v1 constraint, not a physics
    constraint; see the module docstring. Off-diagonals must be
    *exactly* zero — the loader reads them from YAML as floats, so
    roundoff is the caller's problem before they reach us.
    """

    if not isinstance(payload.name, str) or not payload.name:
        raise ValueError(f"payload.name: must be a non-empty str, got {payload.name!r}")
    tag = f"payload {payload.name!r}"

    mass = _require_finite(f"{tag}.mass_kg", payload.mass_kg)
    if mass < 0.0:
        raise ValueError(f"{tag}.mass_kg: must be >= 0, got {mass}")

    inertia = payload.inertia_kg_m2
    for key in _INERTIA_KEYS:
        if key not in inertia:
            raise ValueError(f"{tag}.inertia_kg_m2: missing key {key!r}")
    entries = {
        key: _require_finite(f"{tag}.inertia_kg_m2.{key}", inertia[key]) for key in _INERTIA_KEYS
    }

    for key in _INERTIA_OFFDIAG_KEYS:
        if entries[key] != 0.0:
            raise ValueError(
                f"{tag}.inertia_kg_m2.{key}: v1 requires a diagonal tensor "
                f"(exactly 0.0); got {entries[key]}. Diagonalise in the "
                "principal-axis frame before calling validate_payload."
            )

    ixx, iyy, izz = entries["ixx"], entries["iyy"], entries["izz"]

    if mass == 0.0:
        for key in _INERTIA_KEYS:
            if entries[key] != 0.0:
                raise ValueError(
                    f"{tag}: mass_kg == 0 but inertia_kg_m2.{key} == "
                    f"{entries[key]} (sentinel consistency: zero mass "
                    "must carry zero inertia)"
                )
    else:
        for key in _INERTIA_DIAG_KEYS:
            v = entries[key]
            if v <= 0.0:
                raise ValueError(
                    f"{tag}.inertia_kg_m2.{key}: must be > 0 when "
                    f"mass_kg > 0 (v1 modelling constraint: no point "
                    f"masses); got {v}"
                )
        # Triangle inequality on principal moments. Since v1 rejects
        # non-diagonal tensors above, (ixx, iyy, izz) *are* the
        # eigenvalues.
        if ixx + iyy < izz:
            raise ValueError(
                f"{tag}.inertia_kg_m2: triangle inequality violated: "
                f"ixx + iyy = {ixx + iyy} < izz = {izz}"
            )
        if iyy + izz < ixx:
            raise ValueError(
                f"{tag}.inertia_kg_m2: triangle inequality violated: "
                f"iyy + izz = {iyy + izz} < ixx = {ixx}"
            )
        if izz + ixx < iyy:
            raise ValueError(
                f"{tag}.inertia_kg_m2: triangle inequality violated: "
                f"izz + ixx = {izz + ixx} < iyy = {iyy}"
            )

    _require_finite_triplet(f"{tag}.pose_xyz", payload.pose_xyz)
    _require_finite_triplet(f"{tag}.pose_rpy", payload.pose_rpy)


def validate_catalog(catalog: PayloadCatalog) -> None:
    """Raise ``ValueError`` if any entry in *catalog* fails
    :func:`validate_payload`, or if entry names collide.

    Uniqueness is enforced here (not in :func:`validate_payload`)
    because it is a catalog-level invariant — a single payload has
    no duplicate to compare against. Collisions would make
    :meth:`~expectations_loader.PayloadCatalog.get` ambiguous for
    programmatically-constructed catalogs.
    """

    seen: set[str] = set()
    for payload in catalog.payloads:
        validate_payload(payload)
        if payload.name in seen:
            raise ValueError(f"catalog: duplicate payload name {payload.name!r}")
        seen.add(payload.name)
