"""R3 test parametrisation helper: ``(arm, payload)`` combinations.

Pre-baked companion for ROADMAP bullet **M6.19** ("Payload
parametrisation across R2 tests"). Every R2 integration test added
under M6.12 / M6.13 / M6.14 is required to run over the Cartesian
product of ``SUPPORTED_ARMS`` × ``PayloadCatalog`` from
``tests/integration/expectations/payloads.yaml``.

This module emits that product as an ordered tuple of
``(arm_name, payload_name)`` pairs plus a parallel tuple of
human-readable ids shaped as ``"<arm>-<payload>"`` (e.g.
``"ur5e-no_payload"``). The shape deliberately matches what
``pytest.mark.parametrize(..., ids=...)`` consumes so the same
tuple can be fed into both ``argvalues`` and ``ids`` keyword
arguments without an adaptor.

Design choices mirror the sibling R3 modules (`payload_validation`,
`r3_payload_mjcf`, `r3_payload_splice`):

* Pure stdlib; no numpy, no ROS, no pytest import — the helper
  produces data that pytest can consume, but does not itself
  depend on pytest so it can be reused from non-test tooling.
* Preserves **caller-supplied order** when explicit ``arms`` /
  ``payload_names`` arguments are passed; this keeps the id
  sequence stable under the caller's intent and makes test output
  order predictable.
* Default order follows ``SUPPORTED_ARMS`` (outer) × catalog
  order (inner). That is the order a human reviewing the report
  typically wants ("walk each arm through the payload ladder").
* ``ValueError`` on any genuinely inconsistent input (unknown arm,
  unknown payload, duplicate entry, non-``str`` type, empty
  selection) — tests written against this helper should fail
  loudly rather than silently skipping combinations.

There is no adjustment of expectation values here. That is a
separate seam — a payload-adjusted expectation resolver is a
follow-up pre-bake and depends on per-joint kinematics data we
do not yet carry in the YAMLs.
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional, Sequence, Tuple

try:  # Direct import when tests/integration/ is on sys.path.
    import expectations_loader as _el
except ImportError:  # Fallback: imported as part of the tests.integration package.
    from tests.integration import expectations_loader as _el  # type: ignore[no-redef]

__all__ = (
    "arm_payload_combinations",
    "arm_payload_ids",
)


def _validate_str_sequence(
    values: Sequence[str],
    *,
    label: str,
) -> Tuple[str, ...]:
    if not isinstance(values, (list, tuple)):
        raise ValueError(
            f"r3_payload_parametrize: {label} must be a list or tuple, got {type(values).__name__}"
        )
    out: list[str] = []
    seen: set[str] = set()
    for i, v in enumerate(values):
        if not isinstance(v, str):
            raise ValueError(
                f"r3_payload_parametrize: {label}[{i}] must be str, got {type(v).__name__}"
            )
        if v in seen:
            raise ValueError(f"r3_payload_parametrize: duplicate entry {v!r} in {label}")
        seen.add(v)
        out.append(v)
    if not out:
        raise ValueError(f"r3_payload_parametrize: {label} must be non-empty")
    return tuple(out)


def arm_payload_combinations(
    arms: Optional[Sequence[str]] = None,
    payload_names: Optional[Sequence[str]] = None,
    *,
    expect_dir: Optional[Path] = None,
) -> Tuple[Tuple[str, str], ...]:
    """Return the ordered Cartesian product of ``arms`` × ``payload_names``.

    When ``arms`` is ``None`` the default is
    :data:`expectations_loader.SUPPORTED_ARMS`. When ``payload_names``
    is ``None`` the default is the ordered list of payload names from
    :func:`expectations_loader.load_payloads`. Both defaults give the
    full M6.19 test matrix.

    The returned sequence is ordered outer-by-``arms`` and
    inner-by-``payload_names`` — the reverse (payload-major) would
    put the same arm's no / small / large results far apart in
    pytest output, which is the opposite of what a reviewer wants.

    Raises:
        ValueError: unknown arm, unknown payload, duplicate entry,
            non-``str`` element, or an empty selection.
    """
    if expect_dir is None:
        expect_dir = _el.EXPECT_DIR

    if arms is None:
        resolved_arms: Tuple[str, ...] = _el.SUPPORTED_ARMS
    else:
        resolved_arms = _validate_str_sequence(arms, label="arms")
        unknown = [a for a in resolved_arms if a not in _el.SUPPORTED_ARMS]
        if unknown:
            raise ValueError(
                f"r3_payload_parametrize: unknown arm(s) {unknown}; "
                f"expected subset of {_el.SUPPORTED_ARMS}"
            )

    catalog = _el.load_payloads(expect_dir)
    catalog_names = catalog.names()

    if payload_names is None:
        resolved_payloads: Tuple[str, ...] = catalog_names
    else:
        resolved_payloads = _validate_str_sequence(payload_names, label="payload_names")
        unknown_p = [p for p in resolved_payloads if p not in catalog_names]
        if unknown_p:
            raise ValueError(
                f"r3_payload_parametrize: unknown payload(s) {unknown_p}; "
                f"available: {catalog_names}"
            )

    return tuple((a, p) for a in resolved_arms for p in resolved_payloads)


def arm_payload_ids(
    combinations: Sequence[Tuple[str, str]],
) -> Tuple[str, ...]:
    """Return pytest-style ids ``"<arm>-<payload>"`` for ``combinations``.

    Intended to be used in parallel with
    :func:`arm_payload_combinations`::

        combos = arm_payload_combinations()
        ids = arm_payload_ids(combos)
        @pytest.mark.parametrize("arm, payload", combos, ids=ids)
        def test_foo(arm, payload): ...

    Order is preserved — ``ids[i]`` labels ``combinations[i]``.
    Validates tuple shape and string contents so a caller who
    passes a malformed input gets a clear, self-locating error
    rather than a confusing pytest collection failure.

    Raises:
        ValueError: ``combinations`` is not a list/tuple, is empty,
            contains a non-pair element, contains non-``str``
            components, any component is empty or contains ``"-"``
            (which would make the id round-trip ambiguous), or the
            resulting ids contain duplicates.
    """
    if not isinstance(combinations, (list, tuple)):
        raise ValueError(
            "r3_payload_parametrize: combinations must be a list or tuple, "
            f"got {type(combinations).__name__}"
        )
    if not combinations:
        raise ValueError("r3_payload_parametrize: combinations must be non-empty")

    ids: list[str] = []
    seen: set[str] = set()
    for i, combo in enumerate(combinations):
        if not isinstance(combo, tuple) or len(combo) != 2:
            raise ValueError(
                f"r3_payload_parametrize: combinations[{i}] must be a 2-tuple, " f"got {combo!r}"
            )
        arm, payload = combo
        for label, value in (("arm", arm), ("payload", payload)):
            if not isinstance(value, str):
                raise ValueError(
                    f"r3_payload_parametrize: combinations[{i}] {label} must be str, "
                    f"got {type(value).__name__}"
                )
            if not value:
                raise ValueError(
                    f"r3_payload_parametrize: combinations[{i}] {label} must be non-empty"
                )
            if "-" in value:
                raise ValueError(
                    f"r3_payload_parametrize: combinations[{i}] {label}={value!r} "
                    "must not contain '-' (reserved as id separator)"
                )
        ident = f"{arm}-{payload}"
        if ident in seen:
            raise ValueError(f"r3_payload_parametrize: duplicate id {ident!r} at combinations[{i}]")
        seen.add(ident)
        ids.append(ident)

    return tuple(ids)
