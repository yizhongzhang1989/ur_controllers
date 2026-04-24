"""Unit tests for ``tests/integration/r2_find_artefacts.py``.

Pinned behaviour: a narrow artefact-discovery helper that globs a
caller-supplied ``run_dir`` for R2 artefact YAML files and returns a
deterministic-ordered tuple of parsed :class:`R2ArtefactLocator`
records. Companion to :mod:`r2_run_artefact` (writer) and
:mod:`r2_run_dir` (run-directory helper).
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest

# --- module loading ---------------------------------------------------------
# Sibling modules are loaded the same way the runtime does — file-based
# importlib loads keyed on plain module names — so `isinstance` checks
# across the test file line up with the runtime.

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


_el = _load("expectations_loader")
_ra = _load("r2_run_artefact")
r2_find_artefacts = _load("r2_find_artefacts")


# --- export surface ---------------------------------------------------------


def test_export_surface():
    assert r2_find_artefacts.__all__ == (
        "FILENAME_PREFIX",
        "FILENAME_SUFFIX",
        "R2ArtefactLocator",
        "find_r2_artefacts",
        "parse_artefact_filename",
    )


def test_filename_prefix_matches_writer():
    assert r2_find_artefacts.FILENAME_PREFIX == "r2_stage"


def test_filename_suffix_matches_writer():
    assert r2_find_artefacts.FILENAME_SUFFIX == ".yaml"


def test_locator_is_frozen_dataclass():
    loc = r2_find_artefacts.R2ArtefactLocator(
        path=Path("r2_stage1_ur5e_simple_joint_impedance_no_payload.yaml"),
        stage=1,
        arm="ur5e",
        controller="simple_joint_impedance",
        payload="no_payload",
    )
    with pytest.raises((AttributeError, Exception)):
        loc.stage = 2  # type: ignore[misc]


# --- parse_artefact_filename happy path -------------------------------------


@pytest.mark.parametrize("stage", _ra.SUPPORTED_STAGES)
@pytest.mark.parametrize("arm", _el.SUPPORTED_ARMS)
@pytest.mark.parametrize("payload", _el.PAYLOAD_LEVELS)
@pytest.mark.parametrize(
    "controller",
    [
        "simple_joint_impedance",
        "crisp_joint_impedance",
        "crisp_cartesian_impedance",
        "cartesian_motion",
        "joint_trajectory_controller",
        "jtc",
        "a",  # single-letter controller still legal
    ],
)
def test_parse_happy_path_round_trips_writer(stage, arm, payload, controller):
    # Build the exact filename the writer would emit, then parse it.
    filename = _ra.artefact_filename(stage=stage, arm=arm, controller=controller, payload=payload)
    loc = r2_find_artefacts.parse_artefact_filename(filename)
    assert loc.stage == stage
    assert loc.arm == arm
    assert loc.controller == controller
    assert loc.payload == payload
    assert loc.path == Path(filename)


def test_parse_returns_plain_path_not_resolved(tmp_path):
    # parse_artefact_filename is filename-only; it does not touch fs.
    filename = "r2_stage2_ur15_simple_joint_impedance_large_payload.yaml"
    loc = r2_find_artefacts.parse_artefact_filename(filename)
    assert loc.path == Path(filename)
    # Not absolute, not joined with anything.
    assert not loc.path.is_absolute()


# --- parse_artefact_filename validation -------------------------------------


def test_parse_rejects_non_str():
    with pytest.raises(TypeError, match="filename must be str"):
        r2_find_artefacts.parse_artefact_filename(123)  # type: ignore[arg-type]


def test_parse_rejects_missing_yaml_suffix():
    with pytest.raises(ValueError, match="must end with"):
        r2_find_artefacts.parse_artefact_filename(
            "r2_stage1_ur5e_simple_joint_impedance_no_payload.json"
        )


def test_parse_rejects_missing_prefix():
    with pytest.raises(ValueError, match="must start with"):
        r2_find_artefacts.parse_artefact_filename(
            "stage1_ur5e_simple_joint_impedance_no_payload.yaml"
        )


def test_parse_rejects_empty_body():
    with pytest.raises(ValueError, match="empty body"):
        r2_find_artefacts.parse_artefact_filename("r2_stage.yaml")


def test_parse_rejects_missing_stage_separator():
    with pytest.raises(ValueError, match="stage/arm separator"):
        r2_find_artefacts.parse_artefact_filename("r2_stage1.yaml")


def test_parse_rejects_non_integer_stage():
    with pytest.raises(ValueError, match="non-integer stage"):
        r2_find_artefacts.parse_artefact_filename(
            "r2_stageX_ur5e_simple_joint_impedance_no_payload.yaml"
        )


def test_parse_rejects_unsupported_stage_number():
    with pytest.raises(ValueError, match="unsupported stage"):
        r2_find_artefacts.parse_artefact_filename(
            "r2_stage9_ur5e_simple_joint_impedance_no_payload.yaml"
        )


def test_parse_rejects_non_canonical_stage_sign():
    # '+1' and '-1' parse as int but are not the canonical shape.
    with pytest.raises(ValueError, match="non-canonical stage"):
        r2_find_artefacts.parse_artefact_filename(
            "r2_stage+1_ur5e_simple_joint_impedance_no_payload.yaml"
        )


def test_parse_rejects_unknown_arm():
    with pytest.raises(ValueError, match="known arm prefix"):
        r2_find_artefacts.parse_artefact_filename(
            "r2_stage1_panda_simple_joint_impedance_no_payload.yaml"
        )


def test_parse_rejects_missing_controller_payload_after_arm():
    with pytest.raises(ValueError, match="missing controller/payload"):
        r2_find_artefacts.parse_artefact_filename("r2_stage1_ur5e_.yaml")


def test_parse_rejects_unknown_payload():
    with pytest.raises(ValueError, match="known payload suffix"):
        r2_find_artefacts.parse_artefact_filename(
            "r2_stage1_ur5e_simple_joint_impedance_xlarge_payload.yaml"
        )


def test_parse_rejects_empty_controller_span():
    # arm immediately followed by payload (only a stray separator between)
    # = zero-length controller span.
    with pytest.raises(ValueError, match="empty controller"):
        r2_find_artefacts.parse_artefact_filename("r2_stage1_ur5e__no_payload.yaml")


def test_parse_rejects_no_controller_no_separator():
    # `r2_stage1_ur5e_no_payload.yaml` parses `after_arm = 'no_payload'`,
    # which is the payload name itself with no leading `_` — we surface
    # that as "no known payload *suffix*" so callers see a clear locator.
    with pytest.raises(ValueError, match="known payload suffix"):
        r2_find_artefacts.parse_artefact_filename("r2_stage1_ur5e_no_payload.yaml")


# --- find_r2_artefacts validation -------------------------------------------


def test_find_rejects_non_path_run_dir(tmp_path):
    with pytest.raises(TypeError, match="run_dir must be a pathlib.Path"):
        r2_find_artefacts.find_r2_artefacts(str(tmp_path))  # type: ignore[arg-type]


def test_find_rejects_missing_run_dir(tmp_path):
    with pytest.raises(FileNotFoundError, match="does not exist"):
        r2_find_artefacts.find_r2_artefacts(tmp_path / "missing")


def test_find_rejects_run_dir_that_is_a_file(tmp_path):
    f = tmp_path / "file.txt"
    f.write_text("x")
    with pytest.raises(NotADirectoryError, match="not a directory"):
        r2_find_artefacts.find_r2_artefacts(f)


def test_find_rejects_bad_stage_filter(tmp_path):
    with pytest.raises(ValueError, match="stage filter"):
        r2_find_artefacts.find_r2_artefacts(tmp_path, stage=9)


def test_find_rejects_bool_stage_filter(tmp_path):
    # bool is a subclass of int; reject explicitly.
    with pytest.raises(TypeError, match="stage filter must be int"):
        r2_find_artefacts.find_r2_artefacts(tmp_path, stage=True)


def test_find_rejects_non_int_stage_filter(tmp_path):
    with pytest.raises(TypeError, match="stage filter must be int"):
        r2_find_artefacts.find_r2_artefacts(tmp_path, stage="1")  # type: ignore[arg-type]


def test_find_rejects_bad_arm_filter(tmp_path):
    with pytest.raises(ValueError, match="arm filter"):
        r2_find_artefacts.find_r2_artefacts(tmp_path, arm="panda")


def test_find_rejects_non_str_arm_filter(tmp_path):
    with pytest.raises(TypeError, match="arm filter must be str"):
        r2_find_artefacts.find_r2_artefacts(tmp_path, arm=5)  # type: ignore[arg-type]


def test_find_rejects_bad_payload_filter(tmp_path):
    with pytest.raises(ValueError, match="payload filter"):
        r2_find_artefacts.find_r2_artefacts(tmp_path, payload="xlarge_payload")


def test_find_rejects_non_str_payload_filter(tmp_path):
    with pytest.raises(TypeError, match="payload filter must be str"):
        r2_find_artefacts.find_r2_artefacts(tmp_path, payload=5)  # type: ignore[arg-type]


def test_find_rejects_non_str_controller_filter(tmp_path):
    with pytest.raises(TypeError, match="controller filter must be str"):
        r2_find_artefacts.find_r2_artefacts(tmp_path, controller=5)  # type: ignore[arg-type]


def test_find_rejects_empty_controller_filter(tmp_path):
    with pytest.raises(ValueError, match="non-empty str"):
        r2_find_artefacts.find_r2_artefacts(tmp_path, controller="")


# --- find_r2_artefacts discovery --------------------------------------------


def _touch(dirpath: Path, name: str) -> Path:
    p = dirpath / name
    p.write_text("schema_version: 1\n", encoding="utf-8")
    return p


def test_find_on_empty_dir_returns_empty_tuple(tmp_path):
    out = r2_find_artefacts.find_r2_artefacts(tmp_path)
    assert out == ()
    assert isinstance(out, tuple)


def test_find_skips_non_matching_files(tmp_path):
    _touch(tmp_path, "scratch.log")
    _touch(tmp_path, "notes.md")
    _touch(tmp_path, "run.yaml")  # starts wrong; skip
    assert r2_find_artefacts.find_r2_artefacts(tmp_path) == ()


def test_find_returns_parsed_locators_sorted_by_path(tmp_path):
    # Write in non-sorted order; expect sorted output.
    names = [
        "r2_stage2_ur15_simple_joint_impedance_large_payload.yaml",
        "r2_stage1_ur5e_simple_joint_impedance_no_payload.yaml",
        "r2_stage3_ur15_crisp_cartesian_impedance_small_payload.yaml",
    ]
    paths = [_touch(tmp_path, n) for n in names]

    out = r2_find_artefacts.find_r2_artefacts(tmp_path)
    assert len(out) == 3
    # Sorted by path lexicographically.
    assert [loc.path for loc in out] == sorted(paths)
    # Parsed fields match the writer's derivation.
    for loc in out:
        derived = _ra.artefact_filename(
            stage=loc.stage,
            arm=loc.arm,
            controller=loc.controller,
            payload=loc.payload,
        )
        assert loc.path.name == derived


def test_find_is_non_recursive(tmp_path):
    # An artefact-named file in a subdirectory must not surface.
    sub = tmp_path / "sub"
    sub.mkdir()
    _touch(sub, "r2_stage1_ur5e_simple_joint_impedance_no_payload.yaml")
    assert r2_find_artefacts.find_r2_artefacts(tmp_path) == ()


def test_find_raises_on_half_matching_malformed_filename(tmp_path):
    # Starts with r2_stage, ends with .yaml, but middle is nonsense.
    _touch(tmp_path, "r2_stage1_panda_whatever_no_payload.yaml")
    with pytest.raises(ValueError, match="known arm prefix"):
        r2_find_artefacts.find_r2_artefacts(tmp_path)


def test_find_raises_when_artefact_named_entry_is_a_directory(tmp_path):
    (tmp_path / "r2_stage1_ur5e_simple_joint_impedance_no_payload.yaml").mkdir()
    with pytest.raises(ValueError, match="not a file"):
        r2_find_artefacts.find_r2_artefacts(tmp_path)


# --- find_r2_artefacts filter matrix ---------------------------------------


@pytest.fixture
def populated_run_dir(tmp_path):
    # Matrix of 12 artefacts across (stage × arm × payload × 2 controllers).
    controllers = ("simple_joint_impedance", "crisp_joint_impedance")
    for stage in (1, 2):
        for arm in _el.SUPPORTED_ARMS:
            for payload in _el.PAYLOAD_LEVELS:
                for controller in controllers:
                    name = _ra.artefact_filename(
                        stage=stage,
                        arm=arm,
                        controller=controller,
                        payload=payload,
                    )
                    _touch(tmp_path, name)
    # Add one stage-3 entry with a different controller to exercise
    # the controller filter picking it out.
    _touch(
        tmp_path,
        _ra.artefact_filename(
            stage=3,
            arm="ur5e",
            controller="crisp_cartesian_impedance",
            payload="no_payload",
        ),
    )
    # Plus a scratch file to confirm it's ignored.
    _touch(tmp_path, "scratch.log")
    return tmp_path


def test_filter_by_stage(populated_run_dir):
    out = r2_find_artefacts.find_r2_artefacts(populated_run_dir, stage=1)
    assert all(loc.stage == 1 for loc in out)
    # 2 arms × 3 payloads × 2 controllers = 12.
    assert len(out) == 12


def test_filter_by_arm(populated_run_dir):
    out = r2_find_artefacts.find_r2_artefacts(populated_run_dir, arm="ur5e")
    assert all(loc.arm == "ur5e" for loc in out)


def test_filter_by_payload(populated_run_dir):
    out = r2_find_artefacts.find_r2_artefacts(populated_run_dir, payload="small_payload")
    assert all(loc.payload == "small_payload" for loc in out)


def test_filter_by_controller(populated_run_dir):
    out = r2_find_artefacts.find_r2_artefacts(
        populated_run_dir, controller="crisp_cartesian_impedance"
    )
    assert len(out) == 1
    assert out[0].controller == "crisp_cartesian_impedance"
    assert out[0].stage == 3


def test_filter_by_all_four_keys_pulls_single_combo(populated_run_dir):
    out = r2_find_artefacts.find_r2_artefacts(
        populated_run_dir,
        stage=2,
        arm="ur15",
        controller="simple_joint_impedance",
        payload="large_payload",
    )
    assert len(out) == 1
    loc = out[0]
    assert (loc.stage, loc.arm, loc.controller, loc.payload) == (
        2,
        "ur15",
        "simple_joint_impedance",
        "large_payload",
    )


def test_filters_with_no_match_return_empty_tuple(populated_run_dir):
    # stage=3 + controller=simple_joint_impedance has no artefact.
    out = r2_find_artefacts.find_r2_artefacts(
        populated_run_dir, stage=3, controller="simple_joint_impedance"
    )
    assert out == ()


# --- round-trip with the writer --------------------------------------------


def test_round_trip_with_writer(tmp_path):
    # Use the real writer to emit an artefact, then find + parse it.
    artefact = _ra.R2Artefact(
        stage=1,
        arm="ur5e",
        controller="simple_joint_impedance",
        payload="no_payload",
        theoretical={"response_model": "pd"},
        measured={"q": [0.0, 0.0, 0.0]},
        passed=True,
    )
    run_dir = tmp_path / "r2__20260424T220625Z"
    _ra.write_r2_artefact(run_dir, artefact)

    out = r2_find_artefacts.find_r2_artefacts(run_dir)
    assert len(out) == 1
    loc = out[0]
    assert loc.stage == artefact.stage
    assert loc.arm == artefact.arm
    assert loc.controller == artefact.controller
    assert loc.payload == artefact.payload
    # The returned path is fully joined against run_dir.
    assert loc.path.parent == run_dir
    assert loc.path.is_file()
