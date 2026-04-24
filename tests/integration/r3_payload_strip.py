"""MJCF stripper: remove a previously-spliced R3 payload ``<body>``.

Inverse of :mod:`r3_payload_splice`. Pre-baked consumer API for
**M6.17** (MJCF payload attachment) covering the "swap payload" path:
when the user updates the R3 payload via the dashboard (ADR-0012
§R3), the sim must produce an MJCF reflecting the new payload. The
splicer refuses to attach over an existing ``ee_payload`` body
(double-splice guard). This module closes that seam by letting the
caller strip the prior attachment before re-splicing, so an in-place
payload update becomes ``strip → splice``.

It is also the companion helper for the golden-reference
``mj_fullM`` before/after check (ROADMAP §M6.17): the "before"
reference is reproduced by stripping a spliced MJCF and comparing to
the original zero-payload tree.

Scope (v1, deliberately narrow)
-------------------------------

* Pure stdlib. Only :mod:`xml.etree.ElementTree`. ROS-free,
  MuJoCo-free, numpy-free.
* If no ``<body name="<body_name>">`` exists anywhere in the MJCF,
  the input is returned **verbatim**. This makes the stripper
  idempotent and safe to call unconditionally on a "reset to
  zero-mass" path — a zero-mass payload never produces a body in
  the first place (splicer returns the input byte-identical), so
  round-trip via ``strip(splice(..., no_payload))`` is the identity.
* If exactly one matching body exists, it is detached from its
  parent and the document re-serialised via
  :func:`ET.tostring`. Whitespace formatting is not byte-preserved
  (same caveat as the splicer; MuJoCo is whitespace-insensitive).
* If more than one matching body exists, the stripper raises
  :class:`ValueError` — the caller has either double-spliced or the
  MJCF has an unrelated collision, neither of which the stripper is
  allowed to guess at.

Non-goals
---------

* Parsing the stripped body back into a :class:`Payload`. A
  dedicated extractor is a separate seam if we ever need it (the
  spliced ``<inertial>`` only carries ``diaginertia``, not the
  off-diagonal terms the validator accepts in principle).
* Preserving non-semantic whitespace / comments. ElementTree does
  not round-trip comments; MJCF does not depend on them.

Raises
------
:class:`ValueError`
    On non-str ``mjcf``, malformed XML, empty / non-string /
    whitespace-bearing ``body_name``, or ambiguous (>1) matches.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET

try:  # Direct import when tests/integration/ is on sys.path.
    from r3_payload_mjcf import DEFAULT_BODY_NAME
except ImportError:  # pragma: no cover - defensive fallback for file-path loaders.
    from tests.integration.r3_payload_mjcf import DEFAULT_BODY_NAME  # type: ignore[no-redef]

__all__ = ("strip_payload_from_mjcf", "DEFAULT_BODY_NAME")


def _validate_identifier(value: object, *, field: str) -> str:
    if not isinstance(value, str):
        raise ValueError(f"{field} must be str, got {type(value).__name__}")
    if not value:
        raise ValueError(f"{field} must be non-empty")
    if any(ch.isspace() for ch in value):
        raise ValueError(f"{field} must not contain whitespace: {value!r}")
    return value


def _find_parent_of_matches(root: ET.Element, name: str) -> list[tuple[ET.Element, ET.Element]]:
    """Return ``(parent, child)`` pairs for every body matching ``name``.

    ElementTree has no built-in parent pointer, so we build a local
    parent map by iterating every element. The map is small (MJCF
    documents have O(thousands) of elements at most) and used only
    once per call.
    """
    pairs: list[tuple[ET.Element, ET.Element]] = []
    for parent in root.iter():
        for child in list(parent):
            if child.tag == "body" and child.get("name") == name:
                pairs.append((parent, child))
    return pairs


def strip_payload_from_mjcf(
    mjcf: str,
    *,
    body_name: str = DEFAULT_BODY_NAME,
) -> str:
    """Return ``mjcf`` with any ``<body name="<body_name>">`` removed.

    Parameters
    ----------
    mjcf:
        A well-formed MJCF document as a :class:`str`. Parsed only
        when the body is actually present (see below).
    body_name:
        ``name`` attribute of the payload body to remove. Must be a
        non-empty whitespace-free string. Defaults to
        :data:`DEFAULT_BODY_NAME` (``"ee_payload"``).

    Returns
    -------
    str
        The input MJCF unchanged when no body named ``body_name``
        is present anywhere in the document (idempotent / zero-mass
        path). Otherwise, a serialised MJCF with the single matching
        body removed.
    """
    if not isinstance(mjcf, str):
        raise ValueError(f"mjcf must be str, got {type(mjcf).__name__}")
    _validate_identifier(body_name, field="body_name")

    # Fast path: if the target name does not textually appear, skip the
    # parse entirely. This preserves byte-identity on the common "no
    # payload attached" input (including malformed fragments used as
    # the zero-mass baseline in the splicer tests).
    if body_name not in mjcf:
        return mjcf

    try:
        root = ET.fromstring(mjcf)
    except ET.ParseError as exc:
        raise ValueError(f"mjcf is not well-formed XML: {exc}") from exc

    matches = _find_parent_of_matches(root, body_name)
    if not matches:
        # Textual hit was a substring in some other attribute — no
        # actual body to strip. Return verbatim (same rationale as
        # the fast path above).
        return mjcf
    if len(matches) > 1:
        raise ValueError(
            f"body_name {body_name!r} is ambiguous: "
            f"found {len(matches)} <body> elements with that name; "
            "caller must disambiguate (regenerate the MJCF or remove "
            "the collision upstream)"
        )

    parent, child = matches[0]
    parent.remove(child)
    return ET.tostring(root, encoding="unicode")
