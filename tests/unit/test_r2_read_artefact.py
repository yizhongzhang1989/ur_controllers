"""Unit tests for ``tests/integration/r2_read_artefact.py``.

Pinned behaviour: parse a written artefact YAML back into the
``R2Artefact`` dataclass, revalidating every writer-side constraint
plus a filename/content round-trip check. Companion to
:mod:`r2_run_artefact` (writer), :mod:`r2_find_artefacts` (discovery),
and :mod:`r2_run_dir` (run-directory helper).
"""

from __future__ import annotations

import importlib.util
import math
import sys
from pathlib import Path

import pytest
import yaml

# --- module loading ---------------------------------------------------------
# Sibling modules loaded by file path with a plain module-name key so
# the ``R2Artefact`` dataclass identity is shared with the writer.

_INTEG_DIR = Path(__file__).resolve().parents[1] / "integration"


def _load(name: str):
    if name in sys.modules:
        return sys.modules[name]
    spec = importlib.util.spec_from_file_location(name, _INTEG_DIR / f"{name}.py")
    assert spec is not None and spec.loader is not None
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


EL = _load("expectations_loader")
RA = _load("r2_run_artefact")
RD = _load("r2_read_artefact")


# --- helpers ----------------------------------------------------------------


def _artefact(
    *,
    stage: int = 1,
    arm: str = "ur5e",
    controller: str = "simple_jimp",
    payload: str = "no_payload",
    theoretical: dict | None = None,
    measured: dict | None = None,
    passed: bool = True,
    reasons: tuple = (),
    metadata: dict | None = None,
):
    return RA.R2Artefact(
        stage=stage,
        arm=arm,
        controller=controller,
        payload=payload,
        theoretical=theoretical if theoretical is not None else {"omega_n": 10.0, "zeta": 0.7},
        measured=measured if measured is not None else {"omega_n": 10.2, "zeta": 0.68},
        passed=passed,
        reasons=reasons,
        metadata=metadata if metadata is not None else {},
    )


def _write(tmp_path: Path, **kwargs) -> Path:
    return RA.write_r2_artefact(tmp_path, _artefact(**kwargs))


def _write_custom(tmp_path: Path, doc: dict, *, name: str | None = None) -> Path:
    """Write a hand-crafted YAML document to simulate corruption.

    If ``name`` is omitted, derive it from ``doc`` so the filename /
    content round-trip check does not mask the schema error under test.
    """
    if name is None:
        name = RA.artefact_filename(
            stage=doc["stage"],
            arm=doc["arm"],
            controller=doc["controller"],
            payload=doc["payload"],
        )
    path = tmp_path / name
    path.write_text(yaml.safe_dump(doc, sort_keys=False), encoding="utf-8")
    return path


# ---------------------------------------------------------------------------
# Export surface
# ---------------------------------------------------------------------------


def test_export_surface():
    assert RD.__all__ == (
        "SCHEMA_VERSION",
        "SUPPORTED_STAGES",
        "TOP_LEVEL_KEYS",
        "read_r2_artefact",
    )
    assert callable(RD.read_r2_artefact)


def test_schema_version_pinned_against_writer():
    assert RD.SCHEMA_VERSION == RA.SCHEMA_VERSION == 1


def test_supported_stages_pinned_against_writer():
    assert RD.SUPPORTED_STAGES == RA.SUPPORTED_STAGES == (1, 2, 3)


def test_top_level_keys_pinned():
    assert RD.TOP_LEVEL_KEYS == (
        "schema_version",
        "stage",
        "arm",
        "controller",
        "payload",
        "passed",
        "reasons",
        "metadata",
        "theoretical",
        "measured",
    )


def test_r2artefact_class_identity_with_writer(tmp_path: Path):
    """Reader returns the same dataclass class the writer accepts."""
    path = RA.write_r2_artefact(tmp_path, _artefact())
    back = RD.read_r2_artefact(path)
    # Identity: the class object returned must be the writer's
    # dataclass, not a second copy loaded under a different module key.
    assert type(back) is RA.R2Artefact
    assert isinstance(back, RA.R2Artefact)


# ---------------------------------------------------------------------------
# Happy-path round-trip
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("stage", [1, 2, 3])
@pytest.mark.parametrize("arm", ["ur5e", "ur15"])
@pytest.mark.parametrize("payload", ["no_payload", "small_payload", "large_payload"])
def test_round_trip_matrix(tmp_path: Path, stage, arm, payload):
    art = _artefact(
        stage=stage,
        arm=arm,
        payload=payload,
        theoretical={"zeta": 0.7, "omega_n": 10.0},
        measured={"zeta": 0.69, "omega_n": 10.05},
        metadata={"tol": {"zeta": 0.2}, "note": "hello"},
    )
    path = RA.write_r2_artefact(tmp_path, art)
    back = RD.read_r2_artefact(path)
    assert isinstance(back, RA.R2Artefact)
    assert back.stage == stage
    assert back.arm == arm
    assert back.payload == payload
    assert back.controller == "simple_jimp"
    assert back.passed is True
    assert back.reasons == ()
    assert back.theoretical == {"zeta": 0.7, "omega_n": 10.0}
    assert back.measured == {"zeta": 0.69, "omega_n": 10.05}
    assert back.metadata == {"tol": {"zeta": 0.2}, "note": "hello"}


def test_round_trip_failed_artefact_with_reasons(tmp_path: Path):
    path = RA.write_r2_artefact(
        tmp_path,
        _artefact(passed=False, reasons=("zeta drift", "omega_n too slow")),
    )
    back = RD.read_r2_artefact(path)
    assert back.passed is False
    assert back.reasons == ("zeta drift", "omega_n too slow")
    assert isinstance(back.reasons, tuple)


def test_round_trip_reasons_on_passed_allowed(tmp_path: Path):
    """Writer permits explanatory reasons on passed=True; reader must too."""
    path = RA.write_r2_artefact(
        tmp_path,
        _artefact(passed=True, reasons=("warning: near tolerance",)),
    )
    back = RD.read_r2_artefact(path)
    assert back.passed is True
    assert back.reasons == ("warning: near tolerance",)


def test_round_trip_back_to_writer_unchanged(tmp_path: Path):
    """A reader output can be fed straight back into the writer."""
    src = RA.write_r2_artefact(
        tmp_path,
        _artefact(metadata={"k": 1, "a": 2}),
    )
    back = RD.read_r2_artefact(src)
    dst_dir = tmp_path / "round_trip"
    dst = RA.write_r2_artefact(dst_dir, back)
    assert dst.read_bytes() == src.read_bytes()


def test_round_trip_underscore_heavy_controller(tmp_path: Path):
    """Filename check must pass for controllers whose name contains ``_``."""
    art = _artefact(controller="crisp_joint_impedance")
    path = RA.write_r2_artefact(tmp_path, art)
    back = RD.read_r2_artefact(path)
    assert back.controller == "crisp_joint_impedance"


# ---------------------------------------------------------------------------
# Path validation
# ---------------------------------------------------------------------------


def test_non_path_type_error():
    with pytest.raises(TypeError, match="r2_read_artefact: path must be"):
        RD.read_r2_artefact("not-a-path")  # type: ignore[arg-type]


def test_missing_file_raises(tmp_path: Path):
    with pytest.raises(FileNotFoundError, match="does not exist"):
        RD.read_r2_artefact(tmp_path / "nope.yaml")


def test_directory_instead_of_file(tmp_path: Path):
    d = tmp_path / "adir"
    d.mkdir()
    with pytest.raises(IsADirectoryError, match="is a directory"):
        RD.read_r2_artefact(d)


# ---------------------------------------------------------------------------
# YAML parse + structural validation
# ---------------------------------------------------------------------------


def test_empty_file_rejected(tmp_path: Path):
    p = tmp_path / "r2_stage1_ur5e_simple_jimp_no_payload.yaml"
    p.write_text("", encoding="utf-8")
    with pytest.raises(ValueError, match="empty or contains only YAML null"):
        RD.read_r2_artefact(p)


def test_null_document_rejected(tmp_path: Path):
    p = tmp_path / "r2_stage1_ur5e_simple_jimp_no_payload.yaml"
    p.write_text("null\n", encoding="utf-8")
    with pytest.raises(ValueError, match="empty or contains only YAML null"):
        RD.read_r2_artefact(p)


def test_malformed_yaml_rejected(tmp_path: Path):
    p = tmp_path / "r2_stage1_ur5e_simple_jimp_no_payload.yaml"
    p.write_text("not: a: valid: mapping: [\n", encoding="utf-8")
    with pytest.raises(ValueError, match="YAML parse failed"):
        RD.read_r2_artefact(p)


def test_non_utf8_rejected(tmp_path: Path):
    p = tmp_path / "r2_stage1_ur5e_simple_jimp_no_payload.yaml"
    p.write_bytes(b"\xff\xfe\x00bad")
    with pytest.raises(ValueError, match="not valid UTF-8"):
        RD.read_r2_artefact(p)


def test_top_level_not_mapping_rejected(tmp_path: Path):
    p = tmp_path / "r2_stage1_ur5e_simple_jimp_no_payload.yaml"
    p.write_text("- 1\n- 2\n", encoding="utf-8")
    with pytest.raises(ValueError, match="top-level YAML must be a mapping"):
        RD.read_r2_artefact(p)


def test_duplicate_mapping_key_rejected(tmp_path: Path):
    p = tmp_path / "r2_stage1_ur5e_simple_jimp_no_payload.yaml"
    p.write_text(
        "schema_version: 1\n" "stage: 1\n" "stage: 2\n",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match=r"YAML parse failed[\s\S]*duplicate key"):
        RD.read_r2_artefact(p)


def test_missing_top_level_key(tmp_path: Path):
    art = _artefact()
    path = RA.write_r2_artefact(tmp_path, art)
    doc = yaml.safe_load(path.read_text())
    del doc["measured"]
    path.write_text(yaml.safe_dump(doc), encoding="utf-8")
    with pytest.raises(ValueError, match="missing=\\['measured'\\]"):
        RD.read_r2_artefact(path)


def test_extra_top_level_key(tmp_path: Path):
    art = _artefact()
    path = RA.write_r2_artefact(tmp_path, art)
    doc = yaml.safe_load(path.read_text())
    doc["surprise"] = 42
    path.write_text(yaml.safe_dump(doc), encoding="utf-8")
    with pytest.raises(ValueError, match="unexpected=\\['surprise'\\]"):
        RD.read_r2_artefact(path)


def test_top_level_order_not_required(tmp_path: Path):
    """Reader must accept any top-level key order (writer pins order, reader doesn't)."""
    art = _artefact()
    path = RA.write_r2_artefact(tmp_path, art)
    doc = yaml.safe_load(path.read_text())
    # Rewrite with reversed keys; should still parse cleanly.
    shuffled = {k: doc[k] for k in reversed(list(doc.keys()))}
    path.write_text(yaml.safe_dump(shuffled, sort_keys=False), encoding="utf-8")
    back = RD.read_r2_artefact(path)
    assert back.arm == "ur5e"


# ---------------------------------------------------------------------------
# Per-field validation
# ---------------------------------------------------------------------------


def _base_doc(**overrides):
    doc = {
        "schema_version": 1,
        "stage": 1,
        "arm": "ur5e",
        "controller": "simple_jimp",
        "payload": "no_payload",
        "passed": True,
        "reasons": [],
        "metadata": {},
        "theoretical": {"omega_n": 10.0},
        "measured": {"omega_n": 10.0},
    }
    doc.update(overrides)
    return doc


def test_wrong_schema_version(tmp_path: Path):
    p = _write_custom(tmp_path, _base_doc(schema_version=2))
    with pytest.raises(ValueError, match="schema_version must be 1"):
        RD.read_r2_artefact(p)


def test_schema_version_non_int(tmp_path: Path):
    p = _write_custom(tmp_path, _base_doc(schema_version="1"))
    with pytest.raises(ValueError, match="schema_version must be int"):
        RD.read_r2_artefact(p)


def test_schema_version_bool_rejected(tmp_path: Path):
    p = _write_custom(tmp_path, _base_doc(schema_version=True))
    with pytest.raises(ValueError, match="schema_version must be int"):
        RD.read_r2_artefact(p)


def test_stage_out_of_range(tmp_path: Path):
    doc = _base_doc(stage=4)
    # Use a canonical filename even though stage=4 — the stage check
    # must fire before the filename check. Hand-build the name with
    # stage=4 won't produce a stage check first because the writer's
    # artefact_filename refuses stage=4; write under a custom name.
    path = tmp_path / "r2_stage1_ur5e_simple_jimp_no_payload.yaml"
    path.write_text(yaml.safe_dump(doc, sort_keys=False), encoding="utf-8")
    with pytest.raises(ValueError, match="stage must be one of"):
        RD.read_r2_artefact(path)


def test_stage_non_int(tmp_path: Path):
    path = tmp_path / "r2_stage1_ur5e_simple_jimp_no_payload.yaml"
    path.write_text(yaml.safe_dump(_base_doc(stage="1"), sort_keys=False), encoding="utf-8")
    with pytest.raises(ValueError, match="stage must be int"):
        RD.read_r2_artefact(path)


def test_stage_bool_rejected(tmp_path: Path):
    path = tmp_path / "r2_stage1_ur5e_simple_jimp_no_payload.yaml"
    path.write_text(yaml.safe_dump(_base_doc(stage=True), sort_keys=False), encoding="utf-8")
    with pytest.raises(ValueError, match="stage must be int"):
        RD.read_r2_artefact(path)


def test_arm_unknown(tmp_path: Path):
    path = tmp_path / "r2_stage1_ur5e_simple_jimp_no_payload.yaml"
    path.write_text(yaml.safe_dump(_base_doc(arm="ur10"), sort_keys=False), encoding="utf-8")
    with pytest.raises(ValueError, match="arm must be one of"):
        RD.read_r2_artefact(path)


def test_arm_non_str(tmp_path: Path):
    path = tmp_path / "r2_stage1_ur5e_simple_jimp_no_payload.yaml"
    path.write_text(yaml.safe_dump(_base_doc(arm=42), sort_keys=False), encoding="utf-8")
    with pytest.raises(ValueError, match="arm must be str"):
        RD.read_r2_artefact(path)


def test_payload_unknown(tmp_path: Path):
    path = tmp_path / "r2_stage1_ur5e_simple_jimp_no_payload.yaml"
    path.write_text(yaml.safe_dump(_base_doc(payload="huge"), sort_keys=False), encoding="utf-8")
    with pytest.raises(ValueError, match="payload must be one of"):
        RD.read_r2_artefact(path)


def test_payload_non_str(tmp_path: Path):
    path = tmp_path / "r2_stage1_ur5e_simple_jimp_no_payload.yaml"
    path.write_text(yaml.safe_dump(_base_doc(payload=1), sort_keys=False), encoding="utf-8")
    with pytest.raises(ValueError, match="payload must be str"):
        RD.read_r2_artefact(path)


def test_controller_empty(tmp_path: Path):
    path = tmp_path / "r2_stage1_ur5e_simple_jimp_no_payload.yaml"
    path.write_text(yaml.safe_dump(_base_doc(controller=""), sort_keys=False), encoding="utf-8")
    with pytest.raises(ValueError, match="controller must be non-empty"):
        RD.read_r2_artefact(path)


def test_controller_with_slash(tmp_path: Path):
    path = tmp_path / "r2_stage1_ur5e_simple_jimp_no_payload.yaml"
    path.write_text(yaml.safe_dump(_base_doc(controller="a/b"), sort_keys=False), encoding="utf-8")
    with pytest.raises(ValueError, match="must not contain '/'"):
        RD.read_r2_artefact(path)


def test_controller_non_str(tmp_path: Path):
    path = tmp_path / "r2_stage1_ur5e_simple_jimp_no_payload.yaml"
    path.write_text(yaml.safe_dump(_base_doc(controller=5), sort_keys=False), encoding="utf-8")
    with pytest.raises(ValueError, match="controller must be str"):
        RD.read_r2_artefact(path)


def test_passed_non_bool_int(tmp_path: Path):
    path = tmp_path / "r2_stage1_ur5e_simple_jimp_no_payload.yaml"
    path.write_text(yaml.safe_dump(_base_doc(passed=1), sort_keys=False), encoding="utf-8")
    with pytest.raises(ValueError, match="passed must be bool"):
        RD.read_r2_artefact(path)


def test_passed_non_bool_str(tmp_path: Path):
    path = tmp_path / "r2_stage1_ur5e_simple_jimp_no_payload.yaml"
    path.write_text(yaml.safe_dump(_base_doc(passed="yes"), sort_keys=False), encoding="utf-8")
    with pytest.raises(ValueError, match="passed must be bool"):
        RD.read_r2_artefact(path)


def test_reasons_not_list(tmp_path: Path):
    path = tmp_path / "r2_stage1_ur5e_simple_jimp_no_payload.yaml"
    path.write_text(
        yaml.safe_dump(_base_doc(reasons="just one"), sort_keys=False),
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="reasons must be a list"):
        RD.read_r2_artefact(path)


def test_reasons_non_str_entry(tmp_path: Path):
    path = tmp_path / "r2_stage1_ur5e_simple_jimp_no_payload.yaml"
    path.write_text(
        yaml.safe_dump(_base_doc(reasons=["ok", 42]), sort_keys=False),
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match=r"reasons\[1\] must be str"):
        RD.read_r2_artefact(path)


def test_reasons_empty_str_entry(tmp_path: Path):
    path = tmp_path / "r2_stage1_ur5e_simple_jimp_no_payload.yaml"
    path.write_text(
        yaml.safe_dump(_base_doc(reasons=["ok", ""]), sort_keys=False),
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match=r"reasons\[1\] must be non-empty"):
        RD.read_r2_artefact(path)


def test_reasons_empty_when_failed(tmp_path: Path):
    path = tmp_path / "r2_stage1_ur5e_simple_jimp_no_payload.yaml"
    path.write_text(
        yaml.safe_dump(_base_doc(passed=False, reasons=[]), sort_keys=False),
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="reasons must be non-empty when passed=False"):
        RD.read_r2_artefact(path)


# ---------------------------------------------------------------------------
# Nested JSON-safeness
# ---------------------------------------------------------------------------


def test_theoretical_not_mapping(tmp_path: Path):
    path = tmp_path / "r2_stage1_ur5e_simple_jimp_no_payload.yaml"
    path.write_text(
        yaml.safe_dump(_base_doc(theoretical=[1, 2, 3]), sort_keys=False),
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="theoretical must be a mapping"):
        RD.read_r2_artefact(path)


def test_metadata_not_mapping(tmp_path: Path):
    path = tmp_path / "r2_stage1_ur5e_simple_jimp_no_payload.yaml"
    path.write_text(
        yaml.safe_dump(_base_doc(metadata="oops"), sort_keys=False),
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="metadata must be a mapping"):
        RD.read_r2_artefact(path)


def test_measured_not_mapping(tmp_path: Path):
    path = tmp_path / "r2_stage1_ur5e_simple_jimp_no_payload.yaml"
    path.write_text(
        yaml.safe_dump(_base_doc(measured=42), sort_keys=False),
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="measured must be a mapping"):
        RD.read_r2_artefact(path)


def test_theoretical_nested_non_finite(tmp_path: Path):
    """Hand-written ``.nan`` must be rejected — writer can't emit it."""
    path = tmp_path / "r2_stage1_ur5e_simple_jimp_no_payload.yaml"
    path.write_text(
        "schema_version: 1\n"
        "stage: 1\n"
        "arm: ur5e\n"
        "controller: simple_jimp\n"
        "payload: no_payload\n"
        "passed: true\n"
        "reasons: []\n"
        "metadata: {}\n"
        "theoretical:\n"
        "  omega_n: .nan\n"
        "measured:\n"
        "  omega_n: 10.0\n",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match=r"theoretical\.omega_n: non-finite"):
        RD.read_r2_artefact(path)


def test_measured_nested_inf(tmp_path: Path):
    path = tmp_path / "r2_stage1_ur5e_simple_jimp_no_payload.yaml"
    path.write_text(
        "schema_version: 1\n"
        "stage: 1\n"
        "arm: ur5e\n"
        "controller: simple_jimp\n"
        "payload: no_payload\n"
        "passed: true\n"
        "reasons: []\n"
        "metadata: {}\n"
        "theoretical:\n"
        "  omega_n: 10.0\n"
        "measured:\n"
        "  omega_n: .inf\n",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match=r"measured\.omega_n: non-finite"):
        RD.read_r2_artefact(path)


def test_metadata_non_str_key(tmp_path: Path):
    path = tmp_path / "r2_stage1_ur5e_simple_jimp_no_payload.yaml"
    # YAML int-keyed mapping ``42: hello``.
    path.write_text(
        "schema_version: 1\n"
        "stage: 1\n"
        "arm: ur5e\n"
        "controller: simple_jimp\n"
        "payload: no_payload\n"
        "passed: true\n"
        "reasons: []\n"
        "metadata:\n"
        "  42: hello\n"
        "theoretical: {a: 1}\n"
        "measured: {a: 1}\n",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="metadata: non-str key"):
        RD.read_r2_artefact(path)


def test_metadata_empty_key(tmp_path: Path):
    path = tmp_path / "r2_stage1_ur5e_simple_jimp_no_payload.yaml"
    path.write_text(
        "schema_version: 1\n"
        "stage: 1\n"
        "arm: ur5e\n"
        "controller: simple_jimp\n"
        "payload: no_payload\n"
        "passed: true\n"
        "reasons: []\n"
        'metadata: {"": hello}\n'
        "theoretical: {a: 1}\n"
        "measured: {a: 1}\n",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="empty-string key"):
        RD.read_r2_artefact(path)


def test_deep_nesting_finite_floats_pass(tmp_path: Path):
    art = _artefact(
        theoretical={"a": {"b": {"c": [1.0, 2.0, {"d": 3.14}]}}},
        measured={"a": [1, 2, 3, True, False, None, "text"]},
    )
    path = RA.write_r2_artefact(tmp_path, art)
    back = RD.read_r2_artefact(path)
    assert back.theoretical == {"a": {"b": {"c": [1.0, 2.0, {"d": 3.14}]}}}
    assert back.measured == {"a": [1, 2, 3, True, False, None, "text"]}


def test_list_with_non_finite_float_rejected(tmp_path: Path):
    path = tmp_path / "r2_stage1_ur5e_simple_jimp_no_payload.yaml"
    path.write_text(
        "schema_version: 1\n"
        "stage: 1\n"
        "arm: ur5e\n"
        "controller: simple_jimp\n"
        "payload: no_payload\n"
        "passed: true\n"
        "reasons: []\n"
        "metadata: {}\n"
        "theoretical:\n"
        "  values: [1.0, 2.0, .nan]\n"
        "measured: {a: 1}\n",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match=r"theoretical\.values\[2\]: non-finite"):
        RD.read_r2_artefact(path)


# ---------------------------------------------------------------------------
# Filename / content round-trip
# ---------------------------------------------------------------------------


def test_filename_mismatch_rejected(tmp_path: Path):
    art = _artefact(arm="ur5e")
    correct = RA.write_r2_artefact(tmp_path, art)
    renamed = tmp_path / "r2_stage1_ur15_simple_jimp_no_payload.yaml"
    renamed.write_bytes(correct.read_bytes())
    with pytest.raises(ValueError, match="does not match content-derived name"):
        RD.read_r2_artefact(renamed)


def test_filename_match_passes_with_underscore_controller(tmp_path: Path):
    art = _artefact(controller="crisp_cartesian_impedance", stage=3)
    path = RA.write_r2_artefact(tmp_path, art)
    assert path.name == "r2_stage3_ur5e_crisp_cartesian_impedance_no_payload.yaml"
    back = RD.read_r2_artefact(path)
    assert back.controller == "crisp_cartesian_impedance"


# ---------------------------------------------------------------------------
# Corruption / edge sanity
# ---------------------------------------------------------------------------


def test_non_finite_writer_cannot_produce(tmp_path: Path):
    """The writer normalises; the reader has to catch it if injected."""
    art = _artefact(theoretical={"x": float("nan")})
    with pytest.raises(ValueError, match="non-finite"):
        RA.write_r2_artefact(tmp_path, art)
    # Confirm what the reader would see if someone hand-wrote NaN is
    # also rejected (covered above by test_theoretical_nested_non_finite).
    assert math.isnan(art.theoretical["x"])  # sanity


def test_read_is_pure_function(tmp_path: Path):
    """Reading twice returns equal artefacts but independent dicts."""
    path = RA.write_r2_artefact(tmp_path, _artefact(metadata={"k": [1, 2]}))
    a = RD.read_r2_artefact(path)
    b = RD.read_r2_artefact(path)
    assert a == b
    assert a.metadata is not b.metadata
    assert a.theoretical is not b.theoretical


def test_large_matrix_round_trip_stable(tmp_path: Path):
    """Every supported stage/arm/payload triple round-trips byte-for-byte."""
    for stage in (1, 2, 3):
        for arm in ("ur5e", "ur15"):
            for payload in ("no_payload", "small_payload", "large_payload"):
                d = tmp_path / f"{stage}_{arm}_{payload}"
                d.mkdir()
                orig = RA.write_r2_artefact(
                    d,
                    _artefact(
                        stage=stage,
                        arm=arm,
                        payload=payload,
                        controller="simple_jimp",
                    ),
                )
                back = RD.read_r2_artefact(orig)
                dst_dir = d / "again"
                rewritten = RA.write_r2_artefact(dst_dir, back)
                assert rewritten.read_bytes() == orig.read_bytes()
