"""MJCF splicer: attach an R3 payload ``<body>`` under an anchor body.

Pre-baked consumer API for **M6.17** (MJCF payload attachment). Given a
full MJCF document string and a validated
:class:`~tests.integration.expectations_loader.Payload`, this module
returns an MJCF document string with the payload body spliced as a
child of the anchor body (typically ``tool0``) — closing the seam
between :func:`r3_payload_mjcf.payload_body_mjcf` (leaf snippet) and
whatever subprocess in M6.17 regenerates the MJCF for the
"regenerate + reload" path gated by the R3 design knob in ADR-0012.

Scope (v1, deliberately narrow)
-------------------------------

* Pure stdlib. Only :mod:`xml.etree.ElementTree` plus the sibling
  :mod:`r3_payload_mjcf` emitter and :mod:`payload_validation` /
  loader. ROS-free, MuJoCo-free, numpy-free.
* Zero-mass payload ⇒ input MJCF returned **byte-identical**. Matches
  the ROADMAP-R3 rule "zero-mass case = no body injected" so the
  ``no_payload`` baseline never perturbs the MJCF (and the R3
  physics test's "compare ``mj_fullM`` before/after" golden reference
  is trivially satisfied for that level).
* Positive-mass payload ⇒ the MJCF is parsed, the named anchor body
  (``tool0`` by default) is located unambiguously, the emitter's
  snippet is parsed into an :class:`xml.etree.ElementTree.Element`
  and appended as the anchor's last child. Result is serialised via
  :func:`ET.tostring` in ``unicode`` mode. Attribute order and element
  order are preserved by Python 3.8+ ElementTree; whitespace
  formatting is **not** preserved byte-for-byte (MuJoCo is whitespace-
  insensitive and we would rather not reinvent lxml), so callers who
  rely on byte-identity must branch on the zero-mass path anyway.

Non-goals
---------

* Validating the rest of the MJCF. If the document declares the
  anchor body with wrong inertia or a missing geom, that's a sim-
  config bug upstream, not this splicer's problem.
* Mutating an MJCF in place at runtime. M6.17's iteration-1 design
  (ADR-0012 §R3.3, recommendation "regenerate + reload") uses this
  module on the freshly-regenerated MJCF before handing it to MuJoCo.
  A hypothetical iteration-2 in-place mutator is a separate seam.
* Computing inertia in ``tool0`` coordinates. The caller supplies
  :class:`Payload` in the emitter's local-frame contract; reframing
  is physics, not string assembly.

Raises
------
:class:`ValueError`
    On malformed MJCF input (parse failure), when ``attach_body`` is
    empty / non-string / contains whitespace, when no body with
    ``name="<attach_body>"`` is found, when more than one such body
    exists (ambiguous splice target), or when a body with
    ``name="<body_name>"`` already exists anywhere in the document
    (would-be double-splice; caller must regenerate instead).
    Validator errors raised by
    :func:`payload_validation.validate_payload` via the emitter
    propagate unchanged.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET

try:  # Direct import when tests/integration/ is on sys.path.
    from expectations_loader import Payload
    from r3_payload_mjcf import DEFAULT_BODY_NAME, payload_body_mjcf
except ImportError:  # pragma: no cover - defensive fallback for file-path loaders.
    from tests.integration.expectations_loader import Payload  # type: ignore[no-redef]
    from tests.integration.r3_payload_mjcf import (  # type: ignore[no-redef]
        DEFAULT_BODY_NAME,
        payload_body_mjcf,
    )

__all__ = ("splice_payload_into_mjcf", "DEFAULT_ATTACH_BODY", "DEFAULT_BODY_NAME")

DEFAULT_ATTACH_BODY = "tool0"


def _validate_identifier(value: object, *, field: str) -> str:
    if not isinstance(value, str):
        raise ValueError(f"{field} must be str, got {type(value).__name__}")
    if not value:
        raise ValueError(f"{field} must be non-empty")
    if any(ch.isspace() for ch in value):
        raise ValueError(f"{field} must not contain whitespace: {value!r}")
    return value


def _find_bodies_by_name(root: ET.Element, name: str) -> list[ET.Element]:
    # MJCF puts bodies under <worldbody>, and bodies nest recursively.
    # ``.//body[@name='...']`` walks every descendant.
    return list(root.iterfind(f".//body[@name='{name}']"))


def splice_payload_into_mjcf(
    mjcf: str,
    payload: Payload,
    *,
    attach_body: str = DEFAULT_ATTACH_BODY,
    body_name: str = DEFAULT_BODY_NAME,
) -> str:
    """Return ``mjcf`` with ``payload`` attached under ``attach_body``.

    Parameters
    ----------
    mjcf:
        A well-formed MJCF document as a :class:`str`. The document
        must declare exactly one body with ``name="<attach_body>"``
        (any depth) when the payload is non-zero mass.
    payload:
        Validated :class:`Payload`. Re-validated inside the emitter;
        any :class:`ValueError` propagates from there.
    attach_body:
        ``name`` attribute of the MJCF body to splice under. Must be a
        non-empty whitespace-free string. Defaults to
        :data:`DEFAULT_ATTACH_BODY` (``"tool0"``).
    body_name:
        ``name`` attribute for the emitted payload body. Same
        validation as :func:`r3_payload_mjcf.payload_body_mjcf`.
        Defaults to :data:`DEFAULT_BODY_NAME` (``"ee_payload"``).

    Returns
    -------
    str
        The input MJCF unchanged when ``payload.mass_kg == 0``; else
        a serialised MJCF with the payload body appended as the last
        child of the anchor body.
    """
    if not isinstance(mjcf, str):
        raise ValueError(f"mjcf must be str, got {type(mjcf).__name__}")
    _validate_identifier(attach_body, field="attach_body")
    # body_name is re-validated inside the emitter, but we validate
    # here too so we can run the "already present" guard before
    # building the snippet.
    _validate_identifier(body_name, field="body_name")

    snippet = payload_body_mjcf(payload, body_name=body_name)
    if snippet == "":
        # Zero-mass sentinel: do not touch the MJCF at all.
        return mjcf

    try:
        root = ET.fromstring(mjcf)
    except ET.ParseError as exc:
        raise ValueError(f"mjcf is not well-formed XML: {exc}") from exc

    anchors = _find_bodies_by_name(root, attach_body)
    if not anchors:
        raise ValueError(
            f"attach_body {attach_body!r} not found in MJCF "
            f"(no <body name={attach_body!r}> element)"
        )
    if len(anchors) > 1:
        raise ValueError(
            f"attach_body {attach_body!r} is ambiguous: "
            f"found {len(anchors)} <body> elements with that name"
        )

    # Reject double-splice. Also guards against the caller feeding us
    # an MJCF that already has a body named `body_name` for some
    # unrelated reason (we wouldn't know which one to replace, and
    # merging is out of scope).
    existing = _find_bodies_by_name(root, body_name)
    if existing:
        raise ValueError(
            f"body_name {body_name!r} already present in MJCF "
            f"(found {len(existing)} existing <body> element(s)); "
            "regenerate the MJCF instead of re-splicing"
        )

    try:
        child = ET.fromstring(snippet)
    except ET.ParseError as exc:  # pragma: no cover - emitter output is well-formed.
        raise ValueError(f"emitter produced malformed snippet: {exc}") from exc

    anchors[0].append(child)
    return ET.tostring(root, encoding="unicode")
