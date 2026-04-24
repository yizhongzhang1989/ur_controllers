"""Unit tests for :mod:`tests.integration.r2_run_artefact`.

Covers: export surface, filename derivation, round-trip
(write+yaml.safe_load matches the expected document), deterministic
byte output across repeated writes, overwrite flag, frozen dataclass,
nested-dict-key sorting, every validation error (bad stage / arm /
payload / controller / passed / reasons / theoretical / measured /
metadata types + path-locators in the nested normaliser), directory
auto-creation, JSON-safe-leaf-type enforcement (NaN / Inf / set /
bytes / non-str keys), tuple->list normalisation, and mapping
normalisation (``MappingProxyType`` round-trips through
``yaml.safe_load`` as a plain dict).
"""

from __future__ import annotations

import importlib.util
import math
import sys
from pathlib import Path
from types import MappingProxyType, ModuleType

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
INT_DIR = REPO_ROOT / "tests" / "integration"


def _load(name: str, path: Path) -> ModuleType:
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)  # type: ignore[arg-type]
    sys.modules[name] = mod
    assert spec and spec.loader
    spec.loader.exec_module(mod)
    return mod


EL = _load("expectations_loader", INT_DIR / "expectations_loader.py")
RA = _load("r2_run_artefact", INT_DIR / "r2_run_artefact.py")


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


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


# ---------------------------------------------------------------------------
# Export surface
# ---------------------------------------------------------------------------


def test_export_surface():
    assert RA.SCHEMA_VERSION == 1
    assert RA.SUPPORTED_STAGES == (1, 2, 3)
    assert set(RA.__all__) == {
        "R2Artefact",
        "SCHEMA_VERSION",
        "SUPPORTED_STAGES",
        "artefact_filename",
        "write_r2_artefact",
    }
    assert callable(RA.artefact_filename)
    assert callable(RA.write_r2_artefact)


def test_r2artefact_frozen():
    a = _artefact()
    with pytest.raises(Exception):  # FrozenInstanceError subclasses AttributeError
        a.stage = 2  # type: ignore[misc]


# ---------------------------------------------------------------------------
# Filename derivation
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("stage", [1, 2, 3])
@pytest.mark.parametrize("arm", ["ur5e", "ur15"])
@pytest.mark.parametrize("payload", ["no_payload", "small_payload", "large_payload"])
def test_artefact_filename_matrix(stage, arm, payload):
    name = RA.artefact_filename(stage=stage, arm=arm, controller="simple_jimp", payload=payload)
    assert name == f"r2_stage{stage}_{arm}_simple_jimp_{payload}.yaml"


def test_artefact_filename_rejects_bad_stage():
    with pytest.raises(ValueError, match="stage must be one of"):
        RA.artefact_filename(stage=0, arm="ur5e", controller="c", payload="no_payload")
    with pytest.raises(ValueError, match="stage must be one of"):
        RA.artefact_filename(stage=4, arm="ur5e", controller="c", payload="no_payload")


def test_artefact_filename_rejects_bad_arm():
    with pytest.raises(ValueError, match="arm must be one of"):
        RA.artefact_filename(stage=1, arm="ur10", controller="c", payload="no_payload")


def test_artefact_filename_rejects_bad_payload():
    with pytest.raises(ValueError, match="payload must be one of"):
        RA.artefact_filename(stage=1, arm="ur5e", controller="c", payload="huge")


def test_artefact_filename_rejects_empty_controller():
    with pytest.raises(ValueError, match="controller must be non-empty"):
        RA.artefact_filename(stage=1, arm="ur5e", controller="", payload="no_payload")


def test_artefact_filename_rejects_controller_with_slash():
    with pytest.raises(ValueError, match="must not contain '/'"):
        RA.artefact_filename(stage=1, arm="ur5e", controller="a/b", payload="no_payload")


def test_artefact_filename_rejects_non_str_controller():
    with pytest.raises(ValueError, match="controller must be str"):
        RA.artefact_filename(stage=1, arm="ur5e", controller=42, payload="no_payload")  # type: ignore[arg-type]


# ---------------------------------------------------------------------------
# Happy-path round-trip
# ---------------------------------------------------------------------------


def test_write_round_trip(tmp_path: Path):
    art = _artefact(
        theoretical={"zeta": 0.7, "omega_n": 10.0},
        measured={"zeta": 0.69, "omega_n": 10.05},
        metadata={"tolerances": {"zeta": 0.2}, "source": "ur5e.yaml"},
    )
    path = RA.write_r2_artefact(tmp_path, art)
    assert path == tmp_path / "r2_stage1_ur5e_simple_jimp_no_payload.yaml"
    assert path.exists()
    doc = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert doc == {
        "schema_version": 1,
        "stage": 1,
        "arm": "ur5e",
        "controller": "simple_jimp",
        "payload": "no_payload",
        "passed": True,
        "reasons": [],
        "metadata": {"source": "ur5e.yaml", "tolerances": {"zeta": 0.2}},
        "theoretical": {"omega_n": 10.0, "zeta": 0.7},
        "measured": {"omega_n": 10.05, "zeta": 0.69},
    }


def test_top_level_key_order_pinned(tmp_path: Path):
    """schema_version must be the first key so a human grep sees the contract."""
    path = RA.write_r2_artefact(tmp_path, _artefact())
    text = path.read_text(encoding="utf-8")
    lines = [ln for ln in text.splitlines() if ln and not ln.startswith(" ")]
    top_keys = [ln.split(":", 1)[0] for ln in lines]
    assert top_keys == [
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
    ]


def test_deterministic_bytes_across_writes(tmp_path: Path):
    art = _artefact(
        theoretical={"b": 1, "a": 2, "c": {"y": 1, "x": 2}},
        measured={"z": 0.1, "a": 0.2},
    )
    path1 = RA.write_r2_artefact(tmp_path, art)
    bytes1 = path1.read_bytes()
    # Overwrite in a fresh directory to simulate a second run.
    path2 = RA.write_r2_artefact(tmp_path / "other", art)
    bytes2 = path2.read_bytes()
    assert bytes1 == bytes2


def test_nested_dict_keys_sorted(tmp_path: Path):
    art = _artefact(theoretical={"b": 1, "a": {"zzz": 1, "aaa": 2}, "c": 3})
    path = RA.write_r2_artefact(tmp_path, art)
    doc = yaml.safe_load(path.read_text(encoding="utf-8"))
    # dict preserves insertion order on py3.7+; normaliser sorts.
    theoretical = doc["theoretical"]
    assert list(theoretical.keys()) == ["a", "b", "c"]
    assert list(theoretical["a"].keys()) == ["aaa", "zzz"]


def test_tuples_become_lists(tmp_path: Path):
    art = _artefact(theoretical={"samples": (1, 2, 3), "nested": ((1, 2), (3, 4))})
    path = RA.write_r2_artefact(tmp_path, art)
    doc = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert doc["theoretical"] == {"nested": [[1, 2], [3, 4]], "samples": [1, 2, 3]}


def test_mapping_proxy_round_trips_as_plain_dict(tmp_path: Path):
    inner = MappingProxyType({"k": 1})
    art = _artefact(theoretical=MappingProxyType({"a": inner}))
    path = RA.write_r2_artefact(tmp_path, art)
    doc = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert doc["theoretical"] == {"a": {"k": 1}}


def test_default_metadata_empty_dict(tmp_path: Path):
    art = _artefact(metadata=None)  # exercises the MappingProxyType default
    # Bypass _artefact's default: directly instantiate.
    art = RA.R2Artefact(
        stage=1,
        arm="ur5e",
        controller="c",
        payload="no_payload",
        theoretical={"a": 1},
        measured={"a": 1},
        passed=True,
    )
    path = RA.write_r2_artefact(tmp_path, art)
    doc = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert doc["metadata"] == {}
    assert doc["reasons"] == []


# ---------------------------------------------------------------------------
# Directory behaviour + overwrite
# ---------------------------------------------------------------------------


def test_creates_run_dir_if_missing(tmp_path: Path):
    run_dir = tmp_path / "runs" / "2026-04-24T00-00-00Z"
    assert not run_dir.exists()
    path = RA.write_r2_artefact(run_dir, _artefact())
    assert path.parent == run_dir
    assert run_dir.is_dir()


def test_refuses_to_overwrite_by_default(tmp_path: Path):
    RA.write_r2_artefact(tmp_path, _artefact())
    with pytest.raises(FileExistsError, match="refusing to overwrite"):
        RA.write_r2_artefact(tmp_path, _artefact())


def test_overwrite_flag(tmp_path: Path):
    RA.write_r2_artefact(tmp_path, _artefact(theoretical={"a": 1}, measured={"a": 1}))
    path = RA.write_r2_artefact(
        tmp_path,
        _artefact(theoretical={"a": 99}, measured={"a": 99}),
        overwrite=True,
    )
    doc = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert doc["theoretical"] == {"a": 99}


def test_rejects_non_path_run_dir(tmp_path: Path):
    with pytest.raises(ValueError, match="run_dir must be a pathlib.Path"):
        RA.write_r2_artefact(str(tmp_path), _artefact())  # type: ignore[arg-type]


def test_rejects_non_artefact():
    with pytest.raises(ValueError, match="must be an R2Artefact"):
        RA.write_r2_artefact(Path("/tmp/x"), {"stage": 1})  # type: ignore[arg-type]


# ---------------------------------------------------------------------------
# Validation: passed / reasons
# ---------------------------------------------------------------------------


def test_passed_must_be_bool(tmp_path: Path):
    art = RA.R2Artefact(
        stage=1,
        arm="ur5e",
        controller="c",
        payload="no_payload",
        theoretical={"a": 1},
        measured={"a": 1},
        passed=1,  # type: ignore[arg-type]
    )
    with pytest.raises(ValueError, match="passed must be bool"):
        RA.write_r2_artefact(tmp_path, art)


def test_failed_artefact_requires_reasons(tmp_path: Path):
    art = _artefact(passed=False, reasons=())
    with pytest.raises(ValueError, match="reasons must be non-empty when passed=False"):
        RA.write_r2_artefact(tmp_path, art)


def test_failed_artefact_with_reasons_ok(tmp_path: Path):
    art = _artefact(passed=False, reasons=("zeta exceeded 20% tolerance",))
    path = RA.write_r2_artefact(tmp_path, art)
    doc = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert doc["passed"] is False
    assert doc["reasons"] == ["zeta exceeded 20% tolerance"]


def test_passed_allows_empty_reasons(tmp_path: Path):
    art = _artefact(passed=True, reasons=())
    path = RA.write_r2_artefact(tmp_path, art)
    assert path.exists()


def test_reasons_must_be_tuple(tmp_path: Path):
    art = _artefact(reasons=["a"])  # type: ignore[arg-type]
    with pytest.raises(ValueError, match="reasons must be a tuple"):
        RA.write_r2_artefact(tmp_path, art)


def test_reasons_entries_must_be_str(tmp_path: Path):
    art = _artefact(passed=False, reasons=(42,))  # type: ignore[arg-type]
    with pytest.raises(ValueError, match=r"reasons\[0\] must be str"):
        RA.write_r2_artefact(tmp_path, art)


def test_reasons_entries_must_be_non_empty(tmp_path: Path):
    art = _artefact(passed=False, reasons=("",))
    with pytest.raises(ValueError, match=r"reasons\[0\] must be non-empty"):
        RA.write_r2_artefact(tmp_path, art)


# ---------------------------------------------------------------------------
# Validation: theoretical / measured / metadata
# ---------------------------------------------------------------------------


def test_theoretical_must_be_mapping(tmp_path: Path):
    art = RA.R2Artefact(
        stage=1,
        arm="ur5e",
        controller="c",
        payload="no_payload",
        theoretical=[1, 2, 3],  # type: ignore[arg-type]
        measured={"a": 1},
        passed=True,
    )
    with pytest.raises(ValueError, match="theoretical must be a mapping"):
        RA.write_r2_artefact(tmp_path, art)


def test_measured_must_be_mapping(tmp_path: Path):
    art = RA.R2Artefact(
        stage=1,
        arm="ur5e",
        controller="c",
        payload="no_payload",
        theoretical={"a": 1},
        measured="oops",  # type: ignore[arg-type]
        passed=True,
    )
    with pytest.raises(ValueError, match="measured must be a mapping"):
        RA.write_r2_artefact(tmp_path, art)


def test_metadata_must_be_mapping(tmp_path: Path):
    art = RA.R2Artefact(
        stage=1,
        arm="ur5e",
        controller="c",
        payload="no_payload",
        theoretical={"a": 1},
        measured={"a": 1},
        passed=True,
        metadata=123,  # type: ignore[arg-type]
    )
    with pytest.raises(ValueError, match="metadata must be a mapping"):
        RA.write_r2_artefact(tmp_path, art)


def test_rejects_non_finite_float(tmp_path: Path):
    for bad in (math.nan, math.inf, -math.inf):
        art = _artefact(theoretical={"zeta": bad})
        with pytest.raises(ValueError, match="non-finite float"):
            RA.write_r2_artefact(tmp_path, art)


def test_rejects_non_finite_float_in_nested_dict(tmp_path: Path):
    art = _artefact(theoretical={"response": {"zeta": math.nan}})
    with pytest.raises(ValueError, match=r"theoretical\.response\.zeta: non-finite float"):
        RA.write_r2_artefact(tmp_path, art)


def test_rejects_non_finite_float_in_list(tmp_path: Path):
    art = _artefact(theoretical={"samples": [1.0, 2.0, math.inf]})
    with pytest.raises(ValueError, match=r"theoretical\.samples\[2\]: non-finite float"):
        RA.write_r2_artefact(tmp_path, art)


def test_rejects_non_str_dict_key(tmp_path: Path):
    art = _artefact(theoretical={42: "x"})  # type: ignore[dict-item]
    with pytest.raises(ValueError, match="non-str key"):
        RA.write_r2_artefact(tmp_path, art)


def test_rejects_empty_string_dict_key(tmp_path: Path):
    art = _artefact(theoretical={"": 1})
    with pytest.raises(ValueError, match="empty-string key"):
        RA.write_r2_artefact(tmp_path, art)


def test_rejects_unsupported_leaf_types(tmp_path: Path):
    for bad in ({1, 2}, b"bytes", object()):
        art = _artefact(theoretical={"x": bad})
        with pytest.raises(ValueError, match="unsupported type"):
            RA.write_r2_artefact(tmp_path, art)


def test_accepts_json_safe_leaf_types(tmp_path: Path):
    art = _artefact(
        theoretical={
            "i": 1,
            "f": 1.5,
            "s": "hi",
            "b": True,
            "n": None,
            "lst": [1, 2.0, "x", False, None],
        },
    )
    path = RA.write_r2_artefact(tmp_path, art)
    doc = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert doc["theoretical"]["lst"] == [1, 2.0, "x", False, None]
