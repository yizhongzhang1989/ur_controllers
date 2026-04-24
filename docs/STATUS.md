# Status

_Last updated: 2026-04-25 (**third consecutive blocker-only STATUS
refresh — no code change this iteration**). The previous two commits
(`af84dcb docs(status): blocker-only refresh — M6 gated on M6.0`
and `9a08914 docs(status): second blocker-only refresh on 2026-04-25
— still M6.0 gated`) landed earlier the same UTC day; this
iteration re-evaluated the state and reached the same conclusion.
No operator input has been received on M6.0 in the interval, and
the R2/R3 pre-bake chain remains complete (re-audited this
iteration against the last 30+ `test(m6): …` commits — every
declared seam has a landed helper,
pinned by its dedicated unit test; the listed "remaining open
seams" are all deferred-by-design rather than unwritten). The
honest read on §3 step 7 of AGENTS.md is that the blocker persists
and the correct action is again a STATUS-only commit rather than
synthesising a narrow pre-bake that closes no new seam. The loop's
no-progress guard is not tripped (each STATUS refresh is itself a
commit), but **three consecutive** blocker-only iterations on the
same operator gate is a strong signal the operator needs to act on
M6.0 before the auto-dev loop can continue making roadmap-moving
progress. The next iteration, absent operator input, will again
have no in-scope task that is not gated on M6.0 — the agent will
continue to honour AGENTS.md §3 step 7 rather than fabricate work
that doesn't close a real seam. Tests gate was not re-run — no code changed; the prior
iteration's `scripts/run_tests.sh` run (unit 1522 + colcon 10
packages + integration 12 launch tests × {ur5e, ur15}, ~5:18 wall
clock) remains the current green baseline. No submodule pointer
changes, no DECISIONS.md additions. Per AGENTS.md §3 step 7: the
single next highest-value task under the current milestone (M6)
is gated on the **M6.0 operator decision** (vendoring strategy
for `mujoco_ros2_control` and target `ur_robot_driver` version).
The pre-bake chain the agent has been extending across recent
iterations is **complete for the gated R2/R3 orchestrator**; the
STATUS entry below (from the prior iteration) enumerates the
remaining open seams
as:
(1) concrete FK/IK backends — deliberately kept out of tree so the
source / licensing decision is independent of the orchestrator
wiring;
(2) the ROS-side `JointTrajectoryGoal → FollowJointTrajectory.Goal`
and `EePayloadMessage → ur_sim_msgs/EePayload` materialisers — by
design kept outside the pre-bake chain so they can import
`trajectory_msgs` / `ur_sim_msgs` at test-run time (their pre-bake
counterparts, `r2_jtc_goal.py` and `r3_payload_ee_msg.py`, already
landed);
(3) M6.19 payload parametrisation of the R2 stage bodies — needs
M6.16–M6.18, which are themselves gated on M6.0;
(4) a stage-3 theoretical-response extension for
`cartesian_second_order` — deferred with the documented rationale
that the current `tcp_trajectory_tracking` block already suffices
for the `cartesian_motion` position-mode, the `JTC + ik_shim` path,
and the free-space-drift tolerance on `crisp_cartesian_impedance`.
All other in-scope M6 bullets (M6.1–M6.9, M6.16–M6.19) touch
`third_party/ur_simulator@auto_dev` or the vendored
`mujoco_ros2_control` plugin; M6.10 (real-driver parity audit)
needs the operator-confirmed `ur_robot_driver` version, and M6.11
(`robot_driver:={sim,real}` launch arg) needs the parity-audit
target to write against. Rather than force-fit another pre-bake
helper when every remaining open seam is either deferred-by-design
or gated on M6.0, this iteration commits only this STATUS refresh
(AGENTS.md §3 step 7: "if the task is blocked … update
`docs/STATUS.md` with a clear blocker entry, commit only the
STATUS update, and stop"). Tests gate was not re-run this
iteration — no code changed; the prior iteration's run of
`scripts/run_tests.sh` remains the current green baseline (unit
1522 + colcon 10 packages + integration 12 launch tests ×
{ur5e, ur15}). Blockers are enumerated below under "Blockers /
open questions for operator"; operator action on M6.0 unblocks
M6.1 onward. No submodule pointer changes, no DECISIONS.md
additions._

_Previous iteration: R2 **stage → controllers catalog** landed
as `tests/integration/r2_stage_controllers.py` + 60 unit tests —
closes the "live R2 orchestrator needs to enumerate which controllers
each stage exercises" seam that the M5-analogue R2 driver would
otherwise have to hardcode. Pairs with the existing
`r3_payload_parametrize` (arm × payload axis) to give the
orchestrator the four independent matrix axes in two narrow modules.
Exposes `SUPPORTED_STAGES = (1, 2, 3)`, `STAGE_CONTROLLERS: Mapping[
int, tuple[str, ...]]` (MappingProxyType, pinned per ROADMAP §M6.R2),
`JTC_IK_SHIM_VARIANT = "joint_trajectory_controller_ik_shim"`, plus
`controllers_for_stage(stage)`, `stages_for_controller(name)`,
`all_controllers()`, and `expectation_key(variant)`. Stage 1 (M6.12):
`joint_trajectory_controller`, `forward_position_controller`,
`forward_effort_controller`, `crisp_joint_impedance`,
`simple_joint_impedance` — ROADMAP-scoped, deliberately omits
`forward_velocity_controller` (declared in `expectations/*.yaml` but
not listed in M6.12). Stage 2 (M6.13): stage-1 set plus
`cartesian_motion_controller` (ROADMAP R2 stage 2 covers joint-space
and cartesian-mode branches; see the `stage2_tcp` block in
`expectations/*.yaml` and `r2_stage2_cartesian.py`). Stage 3
(M6.14): `cartesian_motion_controller`, `JTC_IK_SHIM_VARIANT`,
`crisp_cartesian_impedance`. The JTC+IK-shim variant uses a **distinct
catalog name** from the plain `joint_trajectory_controller` used in
stages 1/2 so the artefact `controller` key and the by-controller
aggregation in `aggregate_r2_run` stay unambiguous — a stage-3 JTC
run and a stage-1 JTC run land in different report rows.
`expectation_key(JTC_IK_SHIM_VARIANT)` returns
`"joint_trajectory_controller"` so a caller chaining
`controllers_for_stage(3)` into `ArmExpectation.controller(
expectation_key(v))` still resolves without a stage-specific
if-branch. `stages_for_controller` returns `()` for an unknown
controller so the orchestrator can treat "not in the R2 matrix" as
`skip` without an extra guard; return order always matches
`SUPPORTED_STAGES`. Pure stdlib + `MappingProxyType`; no PyYAML /
numpy / ROS imports in this module (the expectations loader brings
its own PyYAML dependency). Error prefix is `r2_stage_controllers:`
on every `TypeError` / `ValueError`, matching the rest of the R2
pre-bake chain so a consumer grep-ing a stack trace can locate the
layer that rejected them. Pinned by 60 unit tests in
`tests/unit/test_r2_stage_controllers.py`: export surface
(`__all__`, module-level constants pinned); `STAGE_CONTROLLERS`
shape (MappingProxyType, immutable, keys == `SUPPORTED_STAGES`,
stage-1 / stage-2 / stage-3 membership pinned verbatim, stage-2
superset-of-stage-1 with `cartesian_motion_controller` added,
stage-2 ordering preserves the stage-1 prefix, stage-1 explicitly
excludes `forward_velocity_controller`, stage 3 excludes plain JTC
and includes the IK-shim variant, each stage's entries unique and
non-empty strs with no `/` or whitespace); `controllers_for_stage`
(happy path × 3 stages, same tuple object returned across calls,
unknown-stage 4-case parametrise → `ValueError`, non-int 5-case
parametrise → `TypeError`, bool-as-int rejected); `stages_for_
controller` (joint-space controllers → `(1, 2)`, JTC → `(1, 2)`,
cartesian_motion → `(2, 3)`, JTC_IK_SHIM_VARIANT / crisp_cartesian →
`(3,)`, unknown name → `()`, non-str `TypeError`, empty str
`ValueError`, return order is `SUPPORTED_STAGES` order); full
inverse consistency between `stages_for_controller` ↔
`controllers_for_stage` (for every `(stage, controller)` pair in
`STAGE_CONTROLLERS`, both directions agree); `all_controllers`
(sorted, unique, equals the union of stage sets, deterministic across
calls, contains every known entry); `expectation_key` (identity for
the 7 non-shim variants via parametrise, JTC_IK_SHIM_VARIANT →
`"joint_trajectory_controller"`, non-str / empty / unknown
rejections); and two cross-module alignment tests that exercise
`expectations_loader.load_arm(arm).controller(expectation_key(
variant))` for every stage-1 entry and for the stage-3 IK-shim
variant on both `ur5e` and `ur15`, so a future stage-1 test body
chaining `controllers_for_stage(1)` → `ArmExpectation.controller`
cannot hit a missing-row error at run-time. Unit gate now reports
**1522 passed** (up from 1462). Full `scripts/run_tests.sh` green
end-to-end: unit (1522) + colcon test (10 packages, 22 `test_math`
+ 5 `test_filters` + 4 `test_pseudo_inverse` gtests) + integration
(12 launch tests × {ur5e, ur15}, 318.42s). With this catalog in
place plus `r3_payload_parametrize`, the future live R2 orchestrator
reduces to: `for stage in SUPPORTED_STAGES: for (arm, payload) in
arm_payload_combinations(): for variant in controllers_for_stage(
stage): ...` — no more stage-specific controller lists at the
consumer, no more stage-3-specific if-branch for the IK-shim path,
no more hidden coupling between the catalog and the expectation
YAML key. The pre-bake chain's only remaining open seams are the
concrete FK/IK backends (deliberately kept out of tree so the
source / licensing decision is independent of the orchestrator
wiring), the ROS-side `JointTrajectoryGoal →
FollowJointTrajectory.Goal` and `EePayloadMessage →
ur_sim_msgs/EePayload` materialisers (by design kept outside the
pre-bake chain so they can import `trajectory_msgs` / `ur_sim_msgs`
at test-run time), and M6.19 payload-parametrisation of the R2
stage bodies (needs M6.16–M6.18 to land first, which are gated on
M6.0). M6.0 operator gate remains active for every bullet that
touches the live sim._

_Previous iteration: R2 **bulk results writer** landed as
`tests/integration/r2_write_results.py` + 38 unit tests — closes the
"live orchestrator has a batch of stage results + per-combo context
and needs to deposit them as YAML artefacts in one R2 run directory"
seam that the previous iteration's aggregator / report-writer chain
left open at the **input** end (`r2_report_writer` closed the output
end). Exposes one frozen dataclass `R2ResultEntry(stage, arm,
controller, payload, theoretical, result, metadata=None)` mirroring
the keyword shape of `r2_result_to_artefact.result_to_artefact`,
plus two functions: `write_r2_result(run_dir, entry, *,
overwrite=False) -> Path` (single-entry shim) and
`write_r2_results(run_dir, entries, *, overwrite=False) ->
Tuple[Path, ...]` (bulk driver over `list | tuple` of entries,
returning written paths in input order so a caller can `zip` back
against the input). Bulk driver runs a **bridge pass first, then a
write pass**: every entry is converted to `R2Artefact` via
`result_to_artefact` **before** any file I/O, so a mid-batch
stage/result mismatch (or controller-echo mismatch, or reserved-
metadata clash) aborts the call with no on-disk side effect — the
"never half-write" contract the single-artefact writer pins extends
cleanly to the batch. Duplicate-combo guard runs ahead of the bridge
pass: two entries sharing the same `(stage, arm, controller,
payload)` 4-tuple raise `ValueError` locating both `entries[i]` /
`entries[j]` indices, avoiding the filesystem's later muddier
`FileExistsError`. Same combo across different stages is allowed
(stage differs ⇒ filename differs ⇒ no collision). Overwrite
semantics are delegated to `write_r2_artefact` per-entry. Sibling
modules loaded via file-path `importlib` with plain module-name
keys (matching the `r2_result_to_artefact` / `r2_aggregate` /
`r2_report_writer` convention — see the "module loading" repo
memory), so an `R2ResultEntry` whose `result` was built against the
direct-loaded stage modules shares class identity with the bridge's
internal `isinstance` checks (pinned by a dedicated
`WR._artefact_mod.R2Artefact is RA.R2Artefact` test). Error prefix
is `r2_write_results:` for the direct layer (non-`Path` `run_dir`,
non-`R2ResultEntry` entry, non-list/tuple `entries`, non-bool
`overwrite`, duplicate combo); bridge errors keep their
`r2_result_to_artefact:` prefix; writer errors keep their
`r2_run_artefact:` prefix — a caller grep-ing a stack trace can tell
which layer rejected them. Pure stdlib in this module; PyYAML is
pulled in transitively by the writer. Pinned by 38 unit tests in
`tests/unit/test_r2_write_results.py`: export surface (`__all__`,
`R2ResultEntry` frozen + default-metadata-is-None, class-identity
check against `r2_run_artefact.R2Artefact`); single-entry type
validation (non-Path `run_dir`, None `run_dir`, non-entry as 4-case
parametrise, non-bool `overwrite`); single-entry happy path for all
four stage result types (Stage1Result, Stage2Result joint-space +
per-joint flattening round-trip through reader, Stage2CartesianResult,
TcpStage3Result with notes → `metadata.stage3_notes`); failing-result
reasons carry-through; `run_dir` auto-creation at depth; error
propagation (bridge stage/result mismatch leaves `run_dir` empty,
controller echo mismatch, pre-existing file without
`overwrite=True` → `FileExistsError`, `overwrite=True` replaces);
bulk-driver type validation (non-Path `run_dir`, non-sequence
`entries` as 4-case parametrise including a generator, non-entry
element with index locator, non-bool `overwrite`); duplicate-combo
rejection (both indices in message, no file written); same-combo-
across-stages allowed (filename differs); empty batch returns `()`;
5-entry full-stage-coverage happy path with per-entry
`artefact_filename` match and input-ordering check; tuple input
accepted; bridge failure in batch aborts before any write;
pre-existing collision in batch aborts without landing new files
beyond the target set; `overwrite=True` allows reruns; end-to-end
chain through `r2_find_artefacts` / `r2_read_artefact` /
`aggregate_r2_run` with mixed pass/fail producing the expected
tallies (`by_stage`, `by_arm`); single-entry round-trip through
reader echoing all four primary keys. Unit gate now reports **1462
passed** (up from 1424). Full `scripts/run_tests.sh` green
end-to-end: unit (1462) + colcon test (10 packages, 22 `test_math`
+ 5 `test_filters` + 4 `test_pseudo_inverse` gtests) + integration
(12 launch tests × {ur5e, ur15}, 333.70s). With this helper in
place, the future live R2 orchestrator (M5-analogue) reduces to:
spin sim per combo, drive the R2 harness, collect results into a
list of `R2ResultEntry`, then call `write_r2_results(run_dir,
entries)` + `aggregate_r2_run(run_dir)` + `write_r2_reports(
report_dir, summary)` — **no more open-coded bridge/write loops at
the consumer, no more filename arithmetic, no more half-write
risk**. The pre-bake chain's only remaining open seams are the
concrete FK/IK backends (deliberately kept out of tree so the
source / licensing decision is independent of the orchestrator
wiring), the ROS-side `JointTrajectoryGoal →
FollowJointTrajectory.Goal` and `EePayloadMessage →
ur_sim_msgs/EePayload` materialisers (by design kept outside the
pre-bake chain so they can import `trajectory_msgs` /
`ur_sim_msgs` at test-run time), and M6.19 payload-parametrisation
of the R2 stage bodies (needs M6.16–M6.18 to land first, which are
gated on M6.0). M6.0 operator gate remains active for every bullet
that touches the live sim._

_Previous iteration: R2 **run-report writer** landed as
`tests/integration/r2_report_writer.py` + 54 unit tests — closes the
"future R2 CSV / Markdown report writer over `R2RunSummary`
(analogous to M5's `evaluation/compare.py::render_report`)" seam
the previous iteration's aggregator docstring pins (module
docstring, lines ~26-30). Exposes three narrow functions —
`write_r2_report_csv(path, summary)`,
`write_r2_report_markdown(path, summary, *, generated_at_utc=None)`,
and the convenience `write_r2_reports(report_dir, summary, *,
generated_at_utc=None) -> (csv_path, md_path)` — plus three pinned
module constants: `CSV_HEADER` (7 columns: `stage`, `arm`,
`controller`, `payload`, `passed`, `reasons`, `artefact_filename`),
`REPORT_CSV_NAME = "report.csv"`, and `REPORT_MD_NAME = "report.md"`.
The CSV preserves the summary's deterministic artefact order
(sorted by filesystem path, inherited from `find_r2_artefacts`) so a
downstream consumer can join on row index; each row's
`artefact_filename` is derived via `r2_run_artefact.artefact_filename`
so the CSV is decoupled from the on-disk path and stays valid across
directory moves. The Markdown renders **failures-first**: top-level
heading, optional `Generated:` line, `Run directory`, a bold
`Overall: PASS|FAIL` one-liner with `total/passed/failed`, a
`## Status summary` section with four per-axis tally tables
(`by_stage` / `by_arm` / `by_controller` / `by_payload` — each with
a `_No artefacts._` placeholder when its map is empty), a
`## Failures` section that lists every failing artefact with its
reasons (or `_No failures._` when overall_pass is True), and a
`## Artefacts` full matrix table at the bottom using ✅/❌ verdict
glyphs. Reasons containing `|` are escaped to `\|` so a rogue pipe
cannot break the Markdown table layout; newlines in reasons are
flattened. Both writers create parent directories with
`parents=True, exist_ok=True` and silently overwrite existing
files (a report is an output artefact, not a log). The CSV is
deterministic by construction — uses `csv.writer` with an explicit
LF (`\n`) line terminator, routed through an in-memory
`io.StringIO` buffer so a mid-iteration failure never half-writes a
partial file, and no per-row timestamp. The Markdown is also
deterministic: `generated_at_utc` is caller-supplied (never computed
inside the writer) so the same summary round-trips to byte-identical
output across two consecutive calls. Validation: non-Path `path` /
`report_dir` → `TypeError` with `r2_report_writer: path must be
pathlib.Path`; non-`R2RunSummary` `summary` (isinstance check
against the shared aggregator class identity) →
`TypeError` with `r2_report_writer: summary must be an
r2_aggregate.R2RunSummary`; non-str non-None `generated_at_utc` →
`TypeError` with `r2_report_writer: generated_at_utc must be str or
None`. Sibling modules loaded via file-path `importlib` with plain
module-name keys (matching the `r2_aggregate` / `r2_read_artefact`
convention — see the "module loading" repo memory), so the
`R2Artefact` / `R2RunSummary` class identities are shared across
the chain and a future live R2 driver (the R2 analogue of
`evaluation/compare.py`) can pass its aggregator output straight to
the report writer without a bridge. Pure stdlib: `csv`, `io`,
`pathlib`, `importlib`; no PyYAML / numpy / ROS imports in this
module (the reader chain brings its own PyYAML dependency).
Pinned by 54 unit tests in `tests/unit/test_r2_report_writer.py`:
export surface (`__all__`, `CSV_HEADER`, `REPORT_CSV_NAME`,
`REPORT_MD_NAME`); input validation (non-Path `path` / `report_dir`
as `None` / str / int / object via 4-case parametrise → `TypeError`;
non-`R2RunSummary` summary via 5-case parametrise → `TypeError`;
non-str `generated_at_utc` → `TypeError`; same checks on the
convenience driver); CSV happy path (empty run dir → header-only;
single passing row; single failing row with multi-item reasons
joined via `"; "`; scrambled-write order matches summary order;
deterministic across two writes; reasons with embedded `|` / `,`
round-trip through `csv.reader`; `passed` tokens are literal
`true`/`false`; parent-dir auto-creation; overwrite semantics;
LF-only line endings with exact line count); Markdown happy path
(empty run dir → `Overall: **FAIL**`, `_No failures._`, `_No
artefacts found..._`; all-passed → `Overall: **PASS**`; single
failure surfaced in its own section *before* the Artefacts table;
all four tally tables render with their column headers; empty-map
tally tables show `_No artefacts._` placeholder four times; tally
rows have correct counts for a hand-picked `stage1 pass/fail + stage2
pass` fixture; `Generated:` line present when supplied, absent when
omitted; embedded `|` in reasons escaped; deterministic across two
writes; trailing newline; parent-dir auto-creation; overwrite
semantics; ✅/❌ glyphs in the artefact table); `write_r2_reports`
convenience (emits both files at the expected names; creates deeply
nested `report_dir`; forwards `generated_at_utc` to the Markdown
writer only; CSV round-trip yields `header + N` rows); and two
end-to-end full-matrix tests (18-combo 3×2×3 sweep → CSV has
19 rows, Markdown surfaces every stage, arm, controller, and
payload in its tally tables). Unit gate now reports **1424 passed**
(up from 1370). Full `scripts/run_tests.sh` green end-to-end: unit
(1424) + colcon test (10 packages, 22 `test_math` + 5 `test_filters`
+ 4 `test_pseudo_inverse` gtests) + integration (12 launch tests ×
{ur5e, ur15}, 343.93s). With this writer in place, the future live
R2 orchestrator (M5-analogue) reduces to: spin sim per combo, drive
the R2 harness, write artefacts via `write_r2_artefact`, then
call `aggregate_r2_run` + `write_r2_reports` to produce the report
— **no more open-coded rendering, tally accumulation, or row
ordering at the consumer**. The pre-bake chain's only remaining
open seams are the concrete FK/IK backends (deliberately kept out
of tree so the source / licensing decision is independent of the
orchestrator wiring), the ROS-side `JointTrajectoryGoal →
FollowJointTrajectory.Goal` and `EePayloadMessage →
ur_sim_msgs/EePayload` materialisers (by design kept outside the
pre-bake chain so they can import `trajectory_msgs` / `ur_sim_msgs`
at test-run time), and M6.19 payload-parametrisation of the R2
stage bodies (needs M6.16–M6.18 to land first, which are gated on
M6.0). M6.0 operator gate remains active for every bullet that
touches the live sim._

_Previous iteration: R2 **run-level aggregator** landed as
`tests/integration/r2_aggregate.py` + 50 unit tests — closes the
"future M5-like R2 aggregation driver (analogous to
`evaluation/compare.py` over R2 artefacts)" seam the
`r2_read_artefact` module docstring pins (lines ~13-18). Exposes one
narrow function `aggregate_r2_run(run_dir: Path) -> R2RunSummary`
plus two frozen dataclasses: `R2Tally(total, passed, failed)` and
`R2RunSummary(run_dir, artefacts, total, passed, failed,
overall_pass, failed_artefacts, by_stage, by_arm, by_controller,
by_payload)`. Chains `find_r2_artefacts` ->
`read_r2_artefact(loc.path)` across every R2 artefact in
`run_dir` and returns a deterministic-ordered summary: artefacts
sorted by filesystem path (inherited from
`find_r2_artefacts`); per-axis tally maps are
`MappingProxyType`-wrapped so a caller cannot mutate the
summary post-construction; iteration order on each tally map is
stable by stringified key (so `by_stage` iterates `1,2,3` and
`by_arm` iterates `ur15,ur5e` regardless of write order).
`overall_pass` is **strict**: `True` iff `total > 0 and failed == 0`
— an empty run dir yields `total=0, overall_pass=False` so a
caller can treat "R2 sweep didn't produce any artefact" as a
failure without an extra guard. Validation is fully delegated: the
discovery layer raises `FileNotFoundError` / `NotADirectoryError` /
`ValueError` / `TypeError` with `r2_find_artefacts:` prefix; the
reader raises `ValueError` / `TypeError` / `FileNotFoundError` /
`IsADirectoryError` with `r2_read_artefact:` prefix; this module
itself only contributes `TypeError` on non-Path `run_dir` with
`r2_aggregate:` prefix. Duplicate primary-key tuples cannot occur
in a well-formed run dir (writer's filename is derived from
`(stage, arm, controller, payload)` and filesystems enforce
filename uniqueness; the reader's filename/content round-trip
check then forbids a hand-renamed file from presenting a different
combo than its name) — the aggregator therefore does not re-check
uniqueness, keeping the module free of dead-code safety belts.
`failed_artefacts` is pre-computed on the summary (same order as
`artefacts`, filtered to `passed=False`) so a future report
writer can show failures first without re-filtering at every
caller. Sibling modules loaded via file-path `importlib` with
plain module-name keys (matching the `r2_find_artefacts` /
`r2_read_artefact` convention, see the "module loading" repo
memory) so the `R2Artefact` class identity returned by the
aggregator is the very same class the reader returns (pinned by a
dedicated `type(s.artefacts[0]) is _ra.R2Artefact` test). Pure
stdlib + `MappingProxyType`; no PyYAML / numpy / ROS imports in
this module (the reader brings its own PyYAML dependency). Pinned
by 50 unit tests in `tests/unit/test_r2_aggregate.py`: export
surface (`__all__`, frozen dataclasses); input validation
(non-Path `run_dir` str/None/int → `TypeError` with
`r2_aggregate:` prefix; missing `run_dir` → `FileNotFoundError`
via the find layer; file-as-`run_dir` → `NotADirectoryError`;
half-matching filename → `ValueError` via the find layer;
corrupted YAML → `ValueError` via the reader); empty-run-dir
semantics (total=0, overall_pass=False, all four tally maps are
empty `MappingProxyType`s, `run_dir` preserved verbatim); single
passing / single failing happy path; overall_pass truth table
(empty, all-passed, mixed); artefact ordering (sorted by
filesystem path, repeatable across two calls on the same dir);
counts consistency as a 6-case parametrisation over
`(n_pass, n_fail)`; per-axis tally correctness (by_stage, by_arm,
by_controller, by_payload) with hand-picked combos; tally sum
invariants (each axis's `sum(total)` equals top-level `total`;
each tally's `total == passed + failed`); all four tally maps are
`MappingProxyType` (both populated and empty cases); deterministic
stringified-key iteration order on all four axes; non-recursive
discovery (nested artefacts ignored); unrelated files silently
skipped; a full-matrix 18-combo end-to-end (3 stages × 2 arms × 3
payloads, all passed); a one-failure-flips-overall variant of
that matrix; two independent run dirs aggregated side-by-side;
and a class-identity test that pins the aggregator's output
dataclass against the reader's. Unit gate now reports **1370
passed** (up from 1320). Full `scripts/run_tests.sh` green
end-to-end: unit (1370) + colcon test (10 packages, 22 `test_math`
+ 5 `test_filters` + 4 `test_pseudo_inverse` gtests) +
integration (12 launch tests × {ur5e, ur15}, 301.29s). With this
helper in place, the future R2 CSV / Markdown emitter (analogous
to M5's `report.csv` / `report.md`) reduces to a pure rendering
pass over `R2RunSummary` — no more open-coded file enumeration,
no more re-reading YAML at every consumer. The pre-bake chain's
only remaining open seams are the concrete FK/IK backends
(deliberately kept out of tree so the source / licensing decision
is independent of the orchestrator wiring), the ROS-side
`JointTrajectoryGoal → FollowJointTrajectory.Goal` and
`EePayloadMessage → ur_sim_msgs/EePayload` materialisers (by design
kept outside the pre-bake chain so they can import
`trajectory_msgs` / `ur_sim_msgs` at test-run time), and M6.19
payload-parametrisation of the R2 stage bodies (needs M6.16–M6.18
to land first, which are gated on M6.0). M6.0 operator gate
remains active for every bullet that touches the live sim._

_Previous iteration: R2 **artefact reader** landed as
`tests/integration/r2_read_artefact.py` + 72 unit tests — closes the
"schema validation is the artefact reader's concern" seam the
`r2_find_artefacts` docstring defers (module docstring, lines ~58-59:
"No validation of the YAML schema. That is the artefact reader's
concern (and the artefact is JSON-safe by construction per the
writer's contract)"). Exposes one narrow function
`read_r2_artefact(path: Path) -> R2Artefact` plus pinned
`SCHEMA_VERSION`, `SUPPORTED_STAGES`, and `TOP_LEVEL_KEYS` constants,
all tied verbatim to the writer's values. Parses the on-disk YAML
with a custom `SafeLoader` subclass that **rejects duplicate mapping
keys** (PyYAML's default keeps the last, silently dropping fields),
and re-asserts every writer-side constraint so a hand-edited or
corrupted file cannot silently feed the future M5-like R2 aggregator:
exact top-level key set (order intentionally not checked — that's
the writer's deterministic-bytes concern), `schema_version == 1`
with `bool`-vs-`int` guard, `stage` int in `SUPPORTED_STAGES` (bool
rejected), `arm` in `SUPPORTED_ARMS`, `payload` in `PAYLOAD_LEVELS`,
`controller` non-empty str with no `/`, `passed` strictly `bool`
(`int 0/1` / `"yes"` rejected), `reasons: list[str]` of non-empty
strings with the `passed=False ⇒ non-empty` rule (writer-mirrored),
`metadata` / `theoretical` / `measured` recursively validated for
JSON-safeness — `Mapping` with non-empty `str` keys, `list`
containers, leaves limited to `str|int|float|bool|None`, finite
floats only (`.nan` / `.inf` from hand-edits raise with a path
locator like `theoretical.omega_n: non-finite float nan`). Filename /
content round-trip: the file's basename must equal
`r2_run_artefact.artefact_filename(stage=..., arm=..., controller=...,
payload=...)` — catches renames where a discovered file says one
combination but contains another, which would otherwise silently
mis-aggregate under a future R2 report driver. Returns the very
same `r2_run_artefact.R2Artefact` dataclass (sibling module loaded
via file-path `importlib` with a plain `sys.modules` key so the
class identity is shared across the chain), with `reasons` coerced
back to `tuple[str, ...]` so a read artefact can be fed straight
into `write_r2_artefact` unchanged (explicit round-trip test pins
byte-equality). Exception contract: non-`Path` → `TypeError`,
missing path → `FileNotFoundError`, path-is-dir → `IsADirectoryError`,
every parse/decode/schema violation → `ValueError` prefixed with
`r2_read_artefact:` and a field locator — one error class for
consumers to catch across the parse/schema boundary. Pure stdlib +
PyYAML (already in-tree per the "python deps" repo memory). Pinned
by 72 unit tests in `tests/unit/test_r2_read_artefact.py`: export
surface (`__all__`, `SCHEMA_VERSION == 1`, `SUPPORTED_STAGES ==
(1,2,3)`, `TOP_LEVEL_KEYS` pinned verbatim, class-identity check
`type(read) is RA.R2Artefact`); happy-path round-trip matrix (3
stages × 2 arms × 3 payloads = 18 combos via pytest parametrisation,
each asserting all eight primary fields echo); failed-artefact
with reasons; passed-with-reasons (writer permits, reader must too);
read-then-write byte-equality; underscore-heavy controller name
(`crisp_cartesian_impedance` stage-3); path validation (non-Path →
`TypeError`, missing → `FileNotFoundError`, dir → `IsADirectoryError`);
YAML parse validation (empty file, null document, malformed YAML,
non-UTF-8, non-mapping top level, duplicate mapping key); structural
validation (missing top-level key, extra top-level key, top-level
order not required); per-field validation (schema_version wrong /
non-int / bool; stage out-of-range / non-int / bool; arm unknown /
non-str; payload unknown / non-str; controller empty / contains `/` /
non-str; passed non-bool int / str; reasons not-list / non-str entry /
empty-str entry / empty-when-failed); nested JSON-safeness
(theoretical / measured / metadata not-mapping; nested `.nan` in
theoretical / `.inf` in measured / non-finite in a list; non-str
metadata key; empty-str metadata key; deep-nested finite-floats-
and-JSON-safe-leaves happy path); filename / content round-trip
(renamed file rejected, underscore-heavy name passes); corruption
edge sanity (writer rejects NaN up front; read is a pure function
— `a == b` but `a.metadata is not b.metadata`); and a large
end-to-end 18-combo round-trip that writes, reads, rewrites, and
asserts byte-identical output across two runs of the same logical
artefact. Unit gate now reports **1320 passed** (up from 1248). Full
`scripts/run_tests.sh` green end-to-end: unit (1320) + colcon test
(10 packages, 22 `test_math` + 5 `test_filters` + 4 `test_pseudo_inverse`
gtests) + integration (12 launch tests × {ur5e, ur15}, 301.82s).
With this helper in place, the future R2 aggregation / reporting
driver (analogous to M5's `compare.py` but over R2 artefacts)
reduces to: `for loc in find_r2_artefacts(run_dir, **filters): art =
read_r2_artefact(loc.path); ...` — one call instead of
`yaml.safe_load` + open-coded schema checks at every consumer. The
pre-bake chain's only remaining open seams are the concrete FK/IK
backends (deliberately kept out of tree so the source / licensing
decision is independent of the orchestrator wiring), the ROS-side
`JointTrajectoryGoal → FollowJointTrajectory.Goal` and
`EePayloadMessage → ur_sim_msgs/EePayload` materialisers (by design
kept outside the pre-bake chain so they can import
`trajectory_msgs` / `ur_sim_msgs` at test-run time), and M6.19
payload-parametrisation of the R2 stage bodies (needs M6.16–M6.18
to land first, which are gated on M6.0). M6.0 operator gate
remains active for every bullet that touches the live sim._

_Previous iteration: R2 **artefact discovery helper** landed as
`tests/integration/r2_find_artefacts.py` + 39 unit tests — closes the
"M5 comparison driver can glob for R2 artefacts" seam the
`r2_run_artefact` writer docstring pins (module docstring, line ~25:
"Keeping the name derived pins a single shape across stages so the M5
comparison driver can glob for R2 artefacts without a schema-file
lookup"). Exposes one narrow function
`find_r2_artefacts(run_dir, *, stage=None, arm=None, controller=None,
payload=None) -> tuple[R2ArtefactLocator, ...]` plus a companion
`parse_artefact_filename(filename) -> R2ArtefactLocator` that inverts
`r2_run_artefact.artefact_filename`. `R2ArtefactLocator` is a frozen
dataclass with five fields: `path: Path`, `stage: int`, `arm: str`,
`controller: str`, `payload: str`. Non-recursive: direct children of
`run_dir` only (the writer always writes at the top level). Files that
don't match the `r2_stage*.yaml` glob are silently skipped (a run dir
may contain caller-supplied logs or scratch); files that half-match
(start with `r2_stage`, end with `.yaml`, but fail to parse) raise
`ValueError` with a path-specific locator rather than being dropped —
a half-matching name is almost always a writer/reader seam bug.
Filename parsing is anchored on the known `SUPPORTED_ARMS` prefix and
`PAYLOAD_LEVELS` suffix so controller names that themselves contain
`_` (e.g. `crisp_joint_impedance`, `crisp_cartesian_impedance`) parse
unambiguously. Non-canonical stage shapes (`+1` / `-1` / leading-zero)
are rejected so the writer's emitted form is the sole round-trippable
shape. Optional filter kwargs let a caller pull a single
`{stage, arm, controller, payload}` combination in one call — the
writer emits one file per combination so this matches the natural
test-body access pattern. Strict validation mirrors the rest of the
R2 pre-bake chain: non-Path `run_dir` → `TypeError`, missing
`run_dir` → `FileNotFoundError`, file-instead-of-dir `run_dir` →
`NotADirectoryError`, out-of-range `stage` filter → `ValueError`,
`bool` `stage` filter → `TypeError` (bool is an int subclass; reject
explicitly), non-`int` stage filter → `TypeError`, non-member
`arm`/`payload` filters → `ValueError`, non-str filters →
`TypeError`, empty-string `controller` filter → `ValueError`. Pure
stdlib; no PyYAML / numpy / ROS imports. Sibling `expectations_loader`
and `r2_run_artefact` are loaded via `importlib` file paths, matching
the convention used by the other `r2_stage*_theoretical` modules (see
the "module loading" repo memory) so this module works both under
the unit-test gate's direct load and when imported as part of the
`tests.integration` package. Pinned by 39 unit tests in
`tests/unit/test_r2_find_artefacts.py`: export surface (`__all__`,
`FILENAME_PREFIX == "r2_stage"`, `FILENAME_SUFFIX == ".yaml"`, frozen
dataclass); `parse_artefact_filename` happy-path matrix as a
full-Cartesian-product round-trip against
`r2_run_artefact.artefact_filename` (3 stages × 2 arms × 3 payloads
× 7 representative controller names = 126 combos, each asserting all
four primary keys + path echo); parse-side validation (non-str,
missing `.yaml`, missing `r2_stage` prefix, empty body, missing
stage/arm separator, non-integer / unsupported / non-canonical stage,
unknown arm, missing controller/payload-after-arm, unknown payload,
empty controller span via double-underscore, no-separator vs.
payload-suffix edge); `find_r2_artefacts` validation (non-Path
run_dir, missing run_dir → `FileNotFoundError`, file-as-run_dir →
`NotADirectoryError`, bad/bool/non-int stage filter, bad/non-str arm
filter, bad/non-str payload filter, non-str/empty controller filter);
discovery semantics (empty dir returns `()`, non-matching files
skipped, sorted-by-path ordering, non-recursive, half-matching
malformed name raises, directory-with-artefact-name raises); filter
matrix against a 25-file populated run dir (filter-by-stage,
filter-by-arm, filter-by-payload, filter-by-controller pulls the
single stage-3 crisp_cartesian_impedance entry, all-four-filters
pulls exactly one combo, no-match returns `()`); and an end-to-end
round-trip that writes via `r2_run_artefact.write_r2_artefact` then
reads back via `find_r2_artefacts`, asserting all four keys echo and
the returned `path` is joined against `run_dir`. Unit gate now
reports **1248 passed** (up from 1079). Full `scripts/run_tests.sh`
green end-to-end: unit (1248) + colcon test (10 packages, 22
`test_math` + 5 `test_filters` + 4 `test_pseudo_inverse` gtests) +
integration (12 launch tests × {ur5e, ur15}, 299.74s). With this
helper in place, future R2 aggregation / reporting (analogous to
M5's `compare.py` report driver but over R2 artefacts) reduces to:
`for loc in find_r2_artefacts(run_dir, **filters):
doc = yaml.safe_load(loc.path.read_text()); ...`. M6.0 operator
gate remains active for every bullet that requires live sim
changes._

_Previous iteration: R2 **run-directory helper** landed as
`tests/integration/r2_run_dir.py` + 47 unit tests — closes the
"caller decides run_dir" seam the `r2_run_artefact` writer docstring
explicitly defers to: "the M5 compare driver already picks a UTC
timestamp; R2 tests will reuse that convention". One narrow function
`make_r2_run_dir(runs_root=None, *, clock=None, prefix="r2",
suffix=None, exist_ok=False) -> Path` that picks a UTC timestamp,
builds the canonical directory name
`<runs_root>/<prefix>__<UTC-ts>[__<suffix>]`, creates the directory,
and returns the `Path`. Timestamp format is pinned verbatim to
`TS_FORMAT = "%Y%m%dT%H%M%SZ"` — the format
`evaluation/run_evaluation.py::make_run_dir` already writes — so the
M5 compare driver's `find_latest_run_dir` lexicographic sort keeps
working across R2 artefacts in the same `runs/` tree (pinned by a
dedicated chronological-sort invariant test). `runs_root` defaults to
`<repo>/evaluation/runs` (gitignored), created on demand. Optional
`suffix` (appended as `__<suffix>` *after* the timestamp) exists so
parallel R2 sweeps started in the same UTC second — e.g. one worker
per `{arm, payload}` in a pytest-xdist pool — can be disambiguated
without touching the timestamp. `exist_ok=False` by default so
sub-second collisions surface as `FileExistsError` instead of
silently co-mingling artefacts from two unrelated runs; callers that
genuinely want to reuse a pre-existing directory pass
`exist_ok=True`. `clock` is injectable (`Callable[[], datetime]`) so
unit tests pin the timestamp without monkey-patching `datetime`.
Strict validation mirrors the rest of the R2 pre-bake chain:
`runs_root` must be `None` or `pathlib.Path` (str rejected —
"one canonical path type" across the chain), `prefix`/`suffix` must
be non-empty `str` with no `/`, `\`, `__`, or surrounding whitespace
(the `__` ban protects the field separator so artefact globs stay
unambiguous), `clock()` must return a UTC-aware `datetime` (naive or
non-UTC rejected — the TS format drops tzinfo and would silently
mislabel a non-UTC stamp), `exist_ok` must be `bool`. Pure stdlib;
no PyYAML / numpy / ROS imports. Pinned by 47 unit tests in
`tests/unit/test_r2_run_dir.py`: export surface (`__all__`,
`TS_FORMAT == "%Y%m%dT%H%M%SZ"` verbatim match with M5,
`DEFAULT_PREFIX == "r2"`, `DEFAULT_RUNS_ROOT` resolves to
`<repo>/evaluation/runs`); happy-path matrix (default dir creation,
`Path` return type, missing `runs_root` auto-created with parents,
custom prefix, suffix appended *after* timestamp, prefix+suffix
together, `None` `runs_root` falls back to `DEFAULT_RUNS_ROOT`
monkey-patched to `tmp_path`, default clock produces a
TS_FORMAT-matching stamp within ±1s of wall-clock `now`); collision
handling (second call with same clock raises `FileExistsError`;
`exist_ok=True` returns the same directory); lexicographic-sort
invariant pinning the justification for M5's `find_latest_run_dir`
(four timestamps across two days sort chronologically as plain
`sorted()`); validation matrix (str-as-runs_root → `TypeError`,
int-as-runs_root → `TypeError`, empty / slash / backslash / `__` /
leading-ws / trailing-ws prefix or suffix → `ValueError`,
non-str prefix or suffix → `TypeError`, `suffix=None` permitted and
produces exactly one `__` separator, non-callable clock →
`TypeError`, clock returning non-datetime → `TypeError`, clock
returning naive datetime → `ValueError`, clock returning non-UTC
datetime → `ValueError`, non-bool `exist_ok` incl. `0`/`1`/`"yes"`/
`None`/`[True]` → `TypeError`); and an end-to-end compose test that
feeds the returned path straight into
`r2_run_artefact.write_r2_artefact(run_dir, R2Artefact(...))` +
`yaml.safe_load` — proving the helper's output needs no glue to
chain into the writer. Unit gate now reports **1079 passed** (up
from 1032). Full `scripts/run_tests.sh` green end-to-end: unit
(1079) + colcon test (10 packages, 22 `test_math` + 5 `test_filters`
+ 4 `test_pseudo_inverse` gtests) + integration (12 launch tests ×
{ur5e, ur15}, 340.21s). With this helper in place, any future R2
test body (M6.12 / M6.13 / M6.14) reduces to:
`run_dir = make_r2_run_dir()` → (per combination)
`result = evaluate_*(...)` → `theoretical = theoretical_for_stage*(...)` →
`artefact = result_to_artefact(stage=..., ..., theoretical=theoretical, result=result)` →
`write_r2_artefact(run_dir, artefact)`. The pre-bake chain's only
remaining open seams are the concrete FK/IK backends (deliberately
kept out of tree so the source / licensing decision is independent
of the orchestrator wiring), the ROS-side
`JointTrajectoryGoal → FollowJointTrajectory.Goal` and
`EePayloadMessage → ur_sim_msgs/EePayload` materialisers (by design
kept outside the pre-bake chain so they can import
`trajectory_msgs` / `ur_sim_msgs` at test-run time), and M6.19
payload-parametrisation of the R2 stage bodies (needs M6.16–M6.18
to land first, which are gated on M6.0). M6.0 operator gate remains
active for every bullet that touches the live sim._

_Previous iteration: R2 **stage-3 cartesian gain resolver**
landed as `tests/integration/r2_stage3_cartesian_gains.py` + 36 unit
tests — Cartesian counterpart of the stage-1 gain resolver: closes the
bringup-YAML → future stage-3 theoretical-response seam for the one
shipped Cartesian impedance controller, so stage-3 test bodies for
`crisp_cartesian_impedance` don't open-code `yaml.safe_load` + a
nested `task.k_{pos,rot}_{x,y,z}` key walk. One narrow function
`resolve_cartesian_impedance_gains(controller, arm, *, config_dir=None)
-> dict` returning a fixed two-key shape
`{"translational": {"x","y","z"}, "rotational": {"x","y","z"}}` of
finite non-negative floats, read from
`cartesian_impedance_controller.ros__parameters.task.k_{pos,rot}_{x,y,z}`.
Scope is intentionally narrow to **one** controller
(`crisp_cartesian_impedance`): `cartesian_motion` is a position-mode
controller whose `pd_gains.{trans,rot}_{x,y,z}.p` fields are IK-solver
proportional gains (different units, different closed-loop semantics),
and mixing them under the same resolver would invite silent misuse —
if a future stage-3 test body needs those, it gets its own narrow
resolver. Task-block damping is out of scope (the task block exposes
no damping keys; crisp derives cartesian task damping from stiffness
internally); nullspace / joint-limit-repulsion fields are also out of
scope (nullspace is already covered for the joint-impedance role by
`r2_stage1_gains`). Strict validation mirrors stage-1 gains:
unknown controller/arm → `ValueError`, missing file →
`FileNotFoundError`, missing key (including any of the six axis keys)
→ `KeyError` with dotted path, non-mapping top-level / `task` block →
`ValueError`, non-numeric / `bool` stiffness → `TypeError`, non-finite
or negative stiffness → `ValueError`, zero stiffness allowed (valid
"no task stiffness on this axis" request). Fixed key order
`("translational","rotational")` at the top and `("x","y","z")` in
each sub-dict so tests can rely on insertion order. Pure stdlib +
PyYAML (already in-tree per the "python deps" repo memory). Pinned by
36 unit tests in `tests/unit/test_r2_stage3_cartesian_gains.py`:
export surface (`__all__`, pinned `SUPPORTED_CONTROLLERS`,
cross-check `SUPPORTED_ARMS` vs. `expectations_loader`, pinned
`AXES`); happy-path matrix over both arms against the **committed**
`bringup/config/crisp_cartesian_impedance.{ur5e,ur15}.yaml` values
(per-axis equality via re-read `yaml.safe_load`); ur5e=ur15
cross-arm equality invariant pinning the file comment ("Gains follow
ur5e for first-light bring-up"); committed-stiffness positivity
sanity; return-type invariants (plain `dict`, plain-float leaves,
independent per-call instances, sub-dicts also independent); argument
rejection matrix (`cartesian_motion` / `crisp_joint_impedance` →
`ValueError`, unknown arm, missing file); temp-YAML matrix covering
happy-path, int-coerced-to-float, zero-stiffness-allowed, missing
top-level key, missing `ros__parameters`, missing `task`, each of the
six `k_{pos,rot}_{axis}` keys missing, non-mapping top-level,
non-mapping `task`, negative K, NaN K, inf K, non-numeric K, `bool`
K (guards against the `bool`-is-`int` gotcha). Unit gate now reports
**1032 passed** (up from 996). Full `scripts/run_tests.sh` green
end-to-end: unit (1032) + colcon test (10 packages, 22 `test_math` +
5 `test_filters` + 4 `test_pseudo_inverse` gtests) + integration (12
launch tests × {ur5e, ur15}), ~4:59 wall clock for the integration
slice. With this helper in place, any future R2 stage-3 test body
for `crisp_cartesian_impedance` that needs per-axis Cartesian
stiffnesses (e.g. to compute a cartesian second-order TCP response
analogous to the stage-1 joint-space second-order block) can call
`gains = resolve_cartesian_impedance_gains("crisp_cartesian_impedance",
arm)` in one line, rather than re-parsing the bringup YAML._

_Previous iteration: R2 **stage-1 gain resolver** landed as
`tests/integration/r2_stage1_gains.py` + 42 unit tests — closes the
last seam between the bringup controller YAMLs and the stage-1
theoretical-block builder's `stiffness_k`/`damping_d` arguments. One
narrow function `resolve_joint_impedance_gains(controller, arm, *,
config_dir=None) -> dict[str, tuple[float, float]]`, ordered by the
YAML's own `joints` list, returning per-joint `(K, D)` for the two
supported impedance controllers: `simple_joint_impedance`
(per-joint `k[i]`/`d[i]` arrays under
`simple_joint_impedance_controller.ros__parameters`) and
`crisp_joint_impedance` (scalar `nullspace.stiffness` /
`nullspace.damping` broadcast to all six joints). Damping auto-fill:
a negative `d[i]` or negative scalar `nullspace.damping` resolves to
critical damping `2*sqrt(K)` — matches both the simple controller's
on-activation rule (parameter description in
`src/simple_joint_impedance_controller/src/simple_joint_impedance_controller.yaml`)
and crisp's `nullspace.damping: -1.0` sentinel (comment in the
committed bringup YAMLs). Strict validation: unknown
controller/arm/`ValueError`, missing file/`FileNotFoundError`,
missing key/`KeyError` with dotted path, length mismatch / duplicate
joints / empty joints / non-finite or negative K /
`ValueError`, non-numeric `k`/`d` entries including
`bool`/`TypeError`, non-finite `D`/`ValueError`. `SUPPORTED_ARMS`
and `CANONICAL_JOINTS` pinned against
`expectations_loader`'s values by dedicated tests so a drift in
either fails loudly. Pure stdlib + PyYAML (already in-tree per the
"python deps" repo memory). Pinned by 42 unit tests in
`tests/unit/test_r2_stage1_gains.py`: export surface
(`__all__`, pinned `SUPPORTED_CONTROLLERS`, cross-check vs.
`expectations_loader`); happy-path matrix over both controllers x
both arms against the **committed** `bringup/config/*.yaml` values
(per-joint K/D equality with `yaml.safe_load` re-read of the same
file); crisp auto-damping sentinel verified across both arms;
cross-controller sanity (crisp broadcasts scalar K; simple does
not); UR15-heavier-than-UR5e stiffness invariant; return-type
invariants (plain `dict`, plain-float tuples, per-call independent
instances); temp-YAML matrix covering simple auto-damping,
crisp explicit / auto damping, missing-top-key, missing
`ros__parameters`, missing `joints`/`k`/`nullspace`/`stiffness`/
`damping`, length mismatch, duplicate joints, empty joints,
negative K, NaN K, non-numeric K, `bool` K (guards against the
`bool`-is-`int` gotcha), infinite D, non-mapping top-level, crisp
`nullspace` wrong type; and an end-to-end thread-through test that
feeds every resolved `(K, D)` into
`theoretical_for_stage1(controller_exp, joint_exp=..., stiffness_k=K,
damping_d=D)` on both arms and both controllers and verifies the
resulting `response` block echoes K/D verbatim and `omega_n_rad_s ==
sqrt(K / J_eff)` against the expectations loader's per-joint
`effective_inertia_kg_m2`. Unit gate now reports **996 passed** (up
from 954). Full `scripts/run_tests.sh` green end-to-end: unit (996)
+ colcon test (10 packages, 22 `test_math` + 5 `test_filters` + 4
`test_pseudo_inverse` gtests) + integration (12 launch tests x
{ur5e, ur15}), ~5:18 wall clock for the integration slice. With
this helper in place, the final R2 stage-1 test body (M6.12 for
the `crisp_joint_impedance` / `simple_joint_impedance` rows)
reduces to: `gains = resolve_joint_impedance_gains(c, arm)` ->
`for joint, (K, D) in gains.items(): theoretical =
theoretical_for_stage1(c_exp, joint_exp=arm_exp.joint(joint),
stiffness_k=K, damping_d=D)` -> `result = evaluate_*(...)` ->
`result_to_artefact(stage=1, ..., theoretical=theoretical,
result=result)` -> `write_r2_artefact(run_dir, artefact)`. The
only remaining open seams on the pre-bake chain are the concrete
FK/IK backends (deliberately kept out of tree) and the thin ROS-side
`JointTrajectoryGoal -> FollowJointTrajectory.Goal` materialiser
(must stay outside the pre-bake chain so it can import
`trajectory_msgs` at test-run time)._

_Previous iteration: R2 **stage-3 theoretical-block builder**
landed as `tests/integration/r2_stage3_theoretical.py` + 21 unit
tests — closes the expectations-loader → `r2_result_to_artefact`
seam for stage-3 and **completes the R2 theoretical-block pre-bake
chain** (stage-1 + stage-2 joint-space + stage-2 cartesian + stage-3
all emit `{response_model, tolerances}` blocks ready to hand straight
to `result_to_artefact`). Exports a single narrow function
`theoretical_for_stage3(arm_exp: ArmExpectation) -> dict` and a
pinned `SUPPORTED_TOLERANCE_KEYS = ('tcp_rmse_mm', 'tcp_peak_err_mm',
'tcp_orientation_peak_deg', 'tcp_steady_drift_mm_per_30s')`. Emits
`response_model='tcp_trajectory_tracking'` (matching the ROADMAP R2
§stage-3 semantics — "commanded TCP trajectory tracked within the
four per-arm TCP tolerances") and pulls all four values via
`arm_exp.tcp_tol(k)` so a missing key bubbles the loader's native
`KeyError` with the `available: [...]` diagnostic intact rather than
silently defaulting. Strict typing: non-`ArmExpectation` inputs
(including `Stage2Tolerances`, `Stage2TcpTolerances`, a bare dict
with the right fields, `None`, `int`, `str`) raise `ValueError` —
silently accepting a plain dict would drop the "numbers came from an
expectation YAML" guarantee. Pure stdlib; sibling `expectations_loader`
resolved via the `sys.modules`-first importlib loader convention
from `r2_stage1_theoretical.py` / `r2_stage2_theoretical.py`. Pinned
by 21 unit tests in `tests/unit/test_r2_stage3_theoretical.py`:
export surface (`__all__`, `SUPPORTED_TOLERANCE_KEYS` tuple exactly
matching the schema test's `TCP_TOLERANCE_KEYS` set); happy path
(value equality, plain-float leaf types, int→float coercion, custom
values propagate, plain-dict output); argument rejection matrix
(None / int / str / bare dict / `Stage2Tolerances` / `Stage2TcpTolerances`);
missing-tolerance-key test proving the loader's `KeyError` bubbles
unchanged; two round-trips through `result_to_artefact(stage=3, ...)`
+ `write_r2_artefact` + `yaml.safe_load` (with and without
`TcpStage3Result.notes`, verifying the notes → `metadata['stage3_notes']`
path); real-YAML integration matrix over `{ur5e, ur15}` × all four
tolerances; and a cross-arm equality check that pins the schema-test
invariant "both arms share identical TCP tolerance values" through
the builder. Unit gate now reports **954 passed** (up from 933).
Full `scripts/run_tests.sh` green end-to-end: unit (954) + colcon
test (10 packages, 22 `test_math` + 5 `test_filters` + 4
`test_pseudo_inverse` gtests) + integration (12 launch tests ×
{ur5e, ur15}), ~4:57 wall clock for the integration slice (one
flake on `test_crisp_gravity_compensation_hold_pose[ur15]`
recovered on immediate retry — pre-existing sim-bringup race,
unrelated to this pure-stdlib helper). With this helper in place,
any future R2 stage-3 test body (M6.14 `cartesian_motion` or
`JTC + ik_shim`) reduces to: `result = evaluate_tcp_trajectory_*(...)` →
`theoretical = theoretical_for_stage3(arm_exp)` →
`artefact = result_to_artefact(stage=3, ..., theoretical=theoretical, result=result)` →
`write_r2_artefact(run_dir, artefact)`. The R2 theoretical-block
pre-bake chain is now complete; the remaining sim-collection seams
are the concrete FK/IK backends (deliberately kept out of tree) and
the thin ROS-side `JointTrajectoryGoal → FollowJointTrajectory.Goal`
materialiser which must stay outside the pre-bake chain so it can
import `trajectory_msgs` at test-run time.)_

_Previous iteration: R2 **stage-2 theoretical-block builder**
landed as `tests/integration/r2_stage2_theoretical.py` + 28 unit
tests — closes the expectations-loader → `r2_result_to_artefact`
seam for stage-2 (both joint-space and cartesian paths). Exports
two narrow functions,
`theoretical_for_stage2_joint_space(stage2_tol: Stage2Tolerances) ->
dict` and
`theoretical_for_stage2_cartesian(stage2_tcp_tol: Stage2TcpTolerances)
-> dict`, each emitting `{response_model, tolerances}` with the
three / two tolerance keys the matching stage-2 harness uses.
`response_model` is `kinematic_consistency_joint_space` /
`kinematic_consistency_tcp` — stage-2's ROADMAP R2 text ("TCP FK of
measured q matches the expected trajectory within tolerance / 5 mm +
2°") has no closed-form response like stage-1's second-order block,
so the theoretical prediction IS the commanded trajectory and what
the artefact records is the interpretation + pass/fail tolerances.
Strict typing: each function rejects non-matching dataclasses
(including the other stage-2 tolerance dataclass or a bare
dict-with-the-right-fields) with `ValueError` — silently accepting
either would emit the wrong `response_model` / tolerance-key set.
Pure stdlib; sibling `expectations_loader` resolved via the
`sys.modules`-first importlib loader convention from
`r2_stage1_theoretical.py`. Pinned by 28 unit tests in
`tests/unit/test_r2_stage2_theoretical.py`: export surface
(SUPPORTED_MODES = ('joint_space', 'cartesian')); happy paths for
both modes (value equality, plain-float leaf types, custom values
propagate, plain-dict output); argument rejection matrix for both
modes (None, int, str, bare dict with right fields, wrong
dataclass); two round-trips through
`result_to_artefact(stage=2, ...)` + `write_r2_artefact` +
`yaml.safe_load` (joint-space via `Stage2Result`, cartesian via
`Stage2CartesianResult`); real-YAML integration matrix over
`{ur5e, ur15}` for both paths; and a cross-arm equality check that
pins the schema-test invariant "both arms share identical stage-2
values" through the builder. Unit gate now reports **933 passed**
(up from 905). Full `scripts/run_tests.sh` green end-to-end: unit
(933) + colcon test (10 packages, 22 `test_math` + 5 `test_filters`
+ 4 `test_pseudo_inverse` gtests) + integration (12 launch tests ×
{ur5e, ur15}), ~5:30 wall clock for the integration slice. With
this helper in place, any future R2 stage-2 test body (M6.13 joint-
space or M6.14 cartesian side) reduces to:
`result = evaluate_all_joints_*(...)` →
`theoretical = theoretical_for_stage2_{joint_space,cartesian}(arm_exp.stage2{_,tcp})` →
`artefact = result_to_artefact(stage=2, ..., theoretical=theoretical, result=result)` →
`write_r2_artefact(run_dir, artefact)`. Only stage-3 theoretical-
block builder remains on the R2-theoretical pre-bake chain.)_

_Previous iteration: R2 **stage-1 theoretical-block builder**
landed as `tests/integration/r2_stage1_theoretical.py` + 52 unit
tests — closes the expectations-loader → `r2_result_to_artefact`
seam for stage-1. Exports a single function
`theoretical_for_stage1(controller_exp, *, joint_exp=None,
stiffness_k=None, damping_d=None) -> dict` that dispatches on
`controller_exp.response_model`: `first_order_lag` (JTC,
`forward_position`, `forward_velocity`) and `open_loop_torque`
(`forward_effort_controller`) return `{response_model, interface,
tolerances}` with the three / two tolerance keys each YAML row
requires; `second_order` (`crisp_joint_impedance`,
`simple_joint_impedance`) additionally computes `omega_n_rad_s` and
`zeta` via `expectations_loader.second_order_response(K, D, J_eff)`
and returns them alongside the echoed K / D / J / joint name under a
`response` sub-block. `joint_exp` / `stiffness_k` / `damping_d` are
strictly rejected for the non-second-order branches (a silently
ignored K/D is almost always a test-authoring bug). K/D are
caller-supplied — this module stays decoupled from
`bringup/config/*.yaml`, matching the posture of
`expectations_loader.second_order_response`. Output is a plain
`dict` with string keys and finite-float / string leaves, ready to
hand straight to `r2_result_to_artefact.result_to_artefact(...,
theoretical=...)`. Pure stdlib; sibling `expectations_loader`
resolved via a `sys.modules`-first importlib loader matching the
convention in `r2_result_to_artefact.py`. Pinned by 52 unit tests
in `tests/unit/test_r2_stage1_theoretical.py`: export surface;
happy paths for all three response models (including tolerances-
are-plain-floats, interface-propagation, int-to-float coercion of
K/D, overdamped ζ≥1 branch, zero-damping branch); extraneous-kwarg
rejection matrix for first-order + open-loop; missing-required-kwarg
matrix for second-order; loader delegation for K≤0 / D<0 validation
errors; missing-tolerance-key bubbling of the loader's native
`KeyError`; unknown-response-model rejection; type rejection
matrix for `controller_exp` / `joint_exp` / non-numeric K / D;
three round-trip tests through `result_to_artefact` +
`write_r2_artefact` + `yaml.safe_load` for the three response
models; and a real-YAML integration matrix over {ur5e, ur15} × {JTC,
forward_position, forward_velocity, forward_effort} +
{crisp_joint_impedance, simple_joint_impedance}. Unit gate now
reports **905 passed** (up from 853). Full `scripts/run_tests.sh`
green end-to-end: unit (905) + colcon test (10 packages, 22
`test_math` + 5 `test_filters` + 4 `test_pseudo_inverse` gtests) +
integration (12 launch tests × {ur5e, ur15}), ~5:00 wall clock for
the integration slice. With this helper in place, any future R2
stage-1 test body reduces to:
`result = evaluate_*(c, ...)` →
`theoretical = theoretical_for_stage1(c, joint_exp=j, stiffness_k=K, damping_d=D)` →
`artefact = result_to_artefact(stage=1, ..., theoretical=theoretical, result=result)` →
`write_r2_artefact(run_dir, artefact)`.)_

_Previous iteration: R2 **result → artefact bridge** landed as
`tests/integration/r2_result_to_artefact.py` + 57 unit tests — closes
the last remaining seam between the four stage assertion harnesses
and the `R2Artefact` writer. Exports a single function
`result_to_artefact(*, stage, arm, controller, payload, theoretical,
result, metadata=None) -> R2Artefact`. Dispatches on `stage`:
`stage=1` → `Stage1Result` (adds `joint` to metadata); `stage=2` →
`Stage2Result` (flattens per-joint metrics into `{joint}.{metric}`
keys and records the joint ordering under `metadata['joints']`) or
`Stage2CartesianResult` (metrics pass through as-is); `stage=3` →
`TcpStage3Result` (attaches non-empty `notes` under
`metadata['stage3_notes']`). `passed` / `reasons` come from
`result.ok` / `result.failures` (stage-2 uses the `Stage2Result.failures`
property that already flattens per-joint failures as
`"{joint}: {msg}"`). Theoretical is caller-supplied — the bridge
carries it through unchanged, so the module stays uncoupled from the
expectation YAML schema. Echo check: `result.controller` must match
the `controller` argument, else `ValueError` (otherwise the
artefact's filename and its embedded `controller` key would point
at different things). Metadata clashes on reserved keys (`joint`,
`joints`, `stage3_notes`) raise instead of silently overwriting —
caller intent is ambiguous. Argument rejection: `stage` must be
`int` in `SUPPORTED_STAGES` (not `bool`); `arm` / `controller` /
`payload` must be non-empty `str`; `theoretical` must be a
`Mapping`; `metadata` must be a `Mapping` or `None`. Pure stdlib;
siblings resolved via a sys.modules-first `_load_sibling` (plain
name first, namespaced fallback) so `isinstance` works across both
the unit-test's direct `spec_from_file_location` load and the
bridge's own namespaced load — pinned by explicit round-trip tests
through `write_r2_artefact`. Pinned by 57 unit tests in
`tests/unit/test_r2_result_to_artefact.py`: export surface; happy
paths for all four result types (pass + fail); controller-echo
mismatches across all four types; stage↔result-type mismatches
(stage=1 with cartesian, stage=2 with TcpStage3, stage=3 with
Stage1); stage-2 per-joint flattening (including empty
`per_joint`); joints-ordering metadata; reserved-key clashes for
`joint` / `joints` / `stage3_notes`; metadata carry-through on the
cartesian stage-2 branch (no reserved key added); stage-3 notes
propagation with and without notes; full argument rejection matrix
(`stage`, `arm`, `controller`, `payload`, `theoretical`,
`metadata` types); `metadata=None` normalisation; and three
end-to-end round-trips through `write_r2_artefact` + `yaml.safe_load`
for stage-1, stage-2 joint-space, and stage-3-with-notes. Unit gate
now reports **853 passed** (up from 796). Full `scripts/run_tests.sh`
green end-to-end: unit (853) + colcon test (10 packages, 22
`test_math` + 5 `test_filters` + 4 `test_pseudo_inverse` gtests) +
integration (12 launch tests × {ur5e, ur15}), ~4:46 wall clock for
the integration slice. With the bridge in place, any future R2
test body is one call (`evaluate_*` → `result_to_artefact` →
`write_r2_artefact`) from sim-collected traces to an on-disk run
artefact.)_

_Previous iteration: R2 run-artefact writer landed as
`tests/integration/r2_run_artefact.py` + 55 unit tests — closes the
seam called out verbatim in ROADMAP §"M6 hard requirements" R2:
"Each stage must publish, in the run artefact under
`evaluation/runs/<ts>/`, the theoretical expectation alongside the
measured result". Exports `R2Artefact(stage, arm, controller,
payload, theoretical, measured, passed, reasons, metadata)` frozen
dataclass + `write_r2_artefact(run_dir, artefact, *, overwrite=False)
-> Path` + `artefact_filename(...)` helper + `SCHEMA_VERSION=1`
constant + `SUPPORTED_STAGES=(1,2,3)`. Filename is derived
(`r2_stage{stage}_{arm}_{controller}_{payload}.yaml`) so the M5
compare driver can glob R2 artefacts without a schema-file lookup.
Output is deterministic: `sort_keys=False` with a pinned top-level
key order (`schema_version` first, then stage / arm / controller /
payload / passed / reasons / metadata / theoretical / measured), and
nested dicts recursively sorted so two runs with the same logical
payload emit byte-identical files. `theoretical` / `measured` /
`metadata` are recursively normalised before serialisation: `Mapping
-> dict`, `tuple -> list`, all dict keys must be `str`, all floats
must be finite, and only JSON-safe leaf types (`str`, `int`, `float`,
`bool`, `None`) are permitted. Validation errors carry a path
locator (e.g. `theoretical.response.zeta: non-finite float nan`) so
the offending field is identified instead of a flat traceback.
`passed=False` requires non-empty `reasons` (a failed artefact with
no diagnosis is almost always a test bug); `passed=True` allows
empty reasons. `run_dir` is auto-created; existing files are not
overwritten unless `overwrite=True`. Pre-write validation runs before
any I/O so a rejected artefact never half-writes. Pure stdlib +
PyYAML (already a repo dep, see `expectations_loader.py`); no ROS,
no numpy. Pinned by 55 unit tests in `tests/unit/test_r2_run_artefact.py`:
export surface, frozen dataclass, filename derivation matrix
({stage} × {arm} × {payload} = 18 combos), all filename-derivation
validation errors (stage / arm / payload / empty / slash / non-str
controller), round-trip via `yaml.safe_load`, top-level key order
pinned, deterministic bytes across writes, nested-dict keys sorted,
tuples become lists, `MappingProxyType` round-trips as plain dict,
default (empty) metadata + reasons, `run_dir` auto-creation,
overwrite-refusal + `overwrite=True` flag, rejections for non-Path
`run_dir` / non-`R2Artefact` / non-bool `passed` / list-instead-of-tuple
reasons / non-str reason entry / empty-string reason / non-mapping
theoretical / non-mapping measured / non-mapping metadata, NaN / ±Inf
rejection (flat + nested dict + nested list with path locator),
non-str / empty-string dict keys, unsupported leaf types (`set`,
`bytes`, arbitrary `object()`), and a happy-path matrix across all
JSON-safe leaf types. Unit gate now reports **796 passed** (up from
741). Full `scripts/run_tests.sh` green end-to-end: unit (796) +
colcon test (10 packages, 22 `test_math` + 5 `test_filters` + 4
`test_pseudo_inverse` gtests) + integration (12 launch tests ×
{ur5e, ur15}), ~5:27 wall clock for the integration slice.
M6.0 operator gate still active for every bullet that requires
live sim changes.)._

_Previous iteration: R3 MJCF payload **stripper** landed as
`tests/integration/r3_payload_strip.py` + 34 unit tests — inverse of
the splicer, closes the M6.17 swap-in-place seam (see 2026-04-24
summary immediately above this entry; full detail preserved in
commit 05d839c)._

_Previous iteration: R3 `EePayload` message-shape builder
landed as `tests/integration/r3_payload_ee_msg.py` + 27 unit tests —
pre-bakes the M6.16 ROS runtime-API plumbing seam. Pure stdlib;
exports frozen dataclass `EePayloadMessage(mass_kg,
inertia_row_major: Tuple[9 floats], pose_position_xyz: Tuple[3],
pose_orientation_xyzw: Tuple[4])` plus
`payload_to_ee_msg(payload) -> EePayloadMessage`. Converts a
validated :class:`Payload` into the shape ADR-0012 addendum §R3.1
specifies for `ur_sim_msgs/EePayload`: `mass` (double, kg),
`inertia` as a symmetric 9-double row-major tensor
`[ixx, ixy, ixz, ixy, iyy, iyz, ixz, iyz, izz]` (v1 validator pins
off-diagonals to zero; symmetric layout future-proofs a
relaxation), and `pose` as `(x, y, z)` position +
`geometry_msgs/Quaternion`-shaped `(x, y, z, w)` orientation (xyzw
ordering, reusing the sibling `r3_payload_mjcf._euler_xyz_to_wxyz`
helper and reordering — one source of truth for the intrinsic-XYZ
convention avoids MJCF-vs-ROS drift). Zero-mass payloads produce a
well-formed message with zero mass / zero inertia and the
loader-supplied pose **preserved** (ADR-0012 §R3.2 pins zero mass
as the sim's launch state; loader doesn't tie pose to mass, so
canonicalizing to identity would be data loss). Also exposes
`EePayloadMessage.to_dict()` returning the ROS-message-shaped dict
so a ROS-side adapter can populate a real `ur_sim_msgs/EePayload`
field-by-field. Pinned by 27 unit tests: export surface, frozen
dataclass contract, all three catalog payloads (`no_payload`,
`small_payload`, `large_payload`) round-trip, inertia symmetric
9-tuple layout, pose echo (incl. negative components), zero-mass
preserving non-zero pose position and orientation, canonical
rotations about X / Y / Z at π/2, unit-norm invariant for
arbitrary rpy, `xyzw == wxyz reordered` cross-check against the
MJCF-side helper, `to_dict()` shape mirrors ROS message, and the
validator-propagation matrix (negative mass, non-diagonal `ixy`,
zero-mass-with-nonzero-inertia, NaN in `pose_xyz` / `pose_rpy`).
M6.0 operator gate still active for every bullet that requires
live sim changes.)._

## Current milestone

**M6 — Unified MuJoCo sim interfaces (runtime-switchable, real-UR
parity), with R1/R2/R3 hard requirements.** Design captured in
ADR-0012 (+ R1/R2/R3 addendum). M0–M5 remain fully ticked. M4
bullet 5 and M-REAL stay out of scope.

## Next task (agent should pick this up)

Still gated on **M6.0** (vendoring strategy for
`mujoco_ros2_control`). Once resolved, the unblocked ordering is:

1. M6.1 — three-actuator MJCF emission on
   `ur_simulator@auto_dev`.
2. M6.2 — unified URDF `<ros2_control>` interface surface.
3. M6.3 — merged `ur_controllers.yaml`.
4. M6.4 — claim-aware ctrl routing in the vendored
   `mujoco_ros2_control` plugin.
5. M6.5 — collapsed sim launch; `scripts/launch_sim.sh` drops
   `control_mode`.
6. M6.6 → M6.8 — runtime-switch test, shim retirement, bringup
   migration.
7. M6.10 / M6.11 / M6.16–M6.19 can interleave once M6.0 gives us
   a target `ur_robot_driver` version and payload plumbing.

With M6.15's schema live plus the loader, theoretical helpers,
measured-signal helpers, stage-1 assertion harness, stage-2
joint-space harness (settle-window support + the
`evaluate_all_joints_from_expectation` wrapper), per-arm stage-2
tolerance block wired through the loader, the stage-3 TCP assertion
harness, the stage-3 **commanded** TCP trajectory generators, the
FK-injected adapter (`r2_tcp_from_joints.py`), the stage-1
**commanded** joint-space generators (`r2_stage1_commands.py`), the
stage-2 **commanded** all-joints generator (`r2_stage2_commands.py`),
the IK-injected adapter (`r2_joints_from_tcp.py`), the
**JTC goal-builder** (`r2_jtc_goal.py`), all three R2
theoretical-block builders (`r2_stage{1,2,3}_theoretical.py`), the
**stage-1 gain resolver** (`r2_stage1_gains.py` — narrow
`resolve_joint_impedance_gains(controller, arm)` against the
committed `bringup/config/{simple,crisp}_joint_impedance.{ur5e,ur15}.yaml`,
auto-filling critical damping for negative `d` / `nullspace.damping:
-1.0`), and now the **stage-3 cartesian gain resolver**
(`r2_stage3_cartesian_gains.py` — narrow
`resolve_cartesian_impedance_gains(controller, arm)` against the
committed `bringup/config/crisp_cartesian_impedance.{ur5e,ur15}.yaml`,
returning per-axis translational/rotational stiffnesses), M6.12 (R2
stage-1), the joint-space path of M6.13 (R2 stage-2), and **both**
paths of M6.14 (R2 stage-3 `cartesian_motion` via the direct TCP
commanded generators **and** `JTC + ik_shim` via the IK adapter) can
all be authored end-to-end as sim-collection orchestrators — the
remaining seam is a thin ROS-side
`JointTrajectoryGoal -> FollowJointTrajectory.Goal`
materialiser which by design lives outside the pre-bake chain so it
can import `trajectory_msgs` at test-run time.

Still missing on the pre-bake chain:

1. ~~R2 run-directory helper~~ — landed as `r2_run_dir.py`.
2. ~~R2 stage-3 cartesian gain resolver~~ — landed as
   `r2_stage3_cartesian_gains.py`.
3. ~~R2 artefact discovery helper~~ — landed as
   `r2_find_artefacts.py`.
4. ~~R2 artefact reader~~ — landed as `r2_read_artefact.py`.
5. ~~R2 run-level aggregator~~ — landed as `r2_aggregate.py`.
6. The concrete FK **and** IK backends themselves — both adapters
   deliberately keep these out of tree so the source / licensing
   decision is independent of the orchestrator wiring.
7. M6.19 payload parametrisation across R2 stages (needs M6.16–
   M6.18 to land first). The validator + MJCF emitter + MJCF
   splicer + stripper + EePayload message-shape builder landed so
   far are the preflight seams those bullets will plug into; the
   message seam is deliberately decoupled from the MJCF seam so
   the `ur_sim_msgs/EePayload` vs `geometry_msgs/Inertia +
   PoseStamped` decision (M6 R3 blocker #3) can still be made
   independently.
8. A thin ROS-side `EePayloadMessage -> ur_sim_msgs/EePayload`
   materialiser — by design lives outside the pre-bake chain so
   it can import `ur_sim_msgs` (and decide whether to use
   `geometry_msgs/Inertia` instead) at test-run time.
9. A cartesian stage-3 theoretical-response extension for
   `cartesian_second_order` (would consume the stiffnesses the
   stage-3 cartesian gain resolver returns). Deferred — stage-3
   theoretical currently emits `tcp_trajectory_tracking` only,
   which suffices for the `cartesian_motion` position-mode and
   `JTC + ik_shim` paths and for the free-space-drift tolerance
   on `crisp_cartesian_impedance`.
10. ~~R2 CSV / Markdown report writer over `R2RunSummary`~~ —
    **landed this iteration** as `r2_report_writer.py`. A future
    live R2 orchestrator now has a one-call rendering seam.

## Last completed tasks

- **This iteration: R2 run-level aggregator (closes the "future
  M5-like R2 aggregation driver (analogous to `evaluation/compare.py`
  over R2 artefacts)" seam the `r2_read_artefact` docstring pins).**
  Added `tests/integration/r2_aggregate.py` exporting one narrow
  function `aggregate_r2_run(run_dir) -> R2RunSummary` plus two
  frozen dataclasses (`R2Tally`, `R2RunSummary`). Chains
  `find_r2_artefacts -> read_r2_artefact` across every R2 artefact
  in `run_dir` and returns a deterministic-ordered summary with
  per-axis `R2Tally` breakdowns by stage / arm / controller /
  payload. Tally maps are `MappingProxyType`-wrapped with stable
  stringified-key iteration order; `overall_pass` is strict
  (`True` iff `total > 0 and failed == 0`, so an empty run dir
  fails without an extra guard). `failed_artefacts` is
  pre-computed on the summary in the same order as `artefacts`.
  Validation fully delegated to the find / read layers; this
  module contributes only `TypeError` on non-Path `run_dir` with
  `r2_aggregate:` prefix. Duplicate primary keys cannot occur in
  a well-formed run dir (writer's derived filename + filesystem
  uniqueness + reader's filename/content round-trip), so the
  aggregator does not re-check uniqueness — no dead-code safety
  belts. Sibling modules loaded via file-path `importlib` so
  `type(s.artefacts[0]) is _ra.R2Artefact` holds across the
  chain. Pure stdlib + `MappingProxyType`. Pinned by 50 unit tests
  in `tests/unit/test_r2_aggregate.py`: export surface, frozen
  dataclasses; input validation (bad `run_dir` type / missing /
  file-as-dir / half-matching filename / corrupted YAML);
  empty-run-dir semantics; single-artefact happy path; overall_pass
  truth table; artefact ordering (sorted, repeatable); 6-case
  counts-consistency parametrisation; per-axis tally correctness
  for all four axes; tally sum invariants; all four tally maps are
  `MappingProxyType` (populated + empty); deterministic
  stringified-key iteration order; non-recursive discovery;
  unrelated files skipped; full-matrix 18-combo end-to-end
  (3 stages × 2 arms × 3 payloads); one-failure variant of same;
  independence of two run dirs; class-identity pin. Unit gate
  **1370 passed** (up from 1320). Full `scripts/run_tests.sh`
  green end-to-end: unit (1370) + colcon test (10 packages) +
  integration (12 launch tests × {ur5e, ur15}, 301.29s).

- **Prior iteration: R2 artefact discovery helper (closes the "M5
  comparison driver can glob for R2 artefacts without a schema-file
  lookup" seam the `r2_run_artefact` writer docstring pins).** Added
  `tests/integration/r2_find_artefacts.py` exporting one narrow
  function `find_r2_artefacts(run_dir, *, stage=None, arm=None,
  controller=None, payload=None) -> tuple[R2ArtefactLocator, ...]`
  plus a companion `parse_artefact_filename(filename)` that inverts
  `r2_run_artefact.artefact_filename`. `R2ArtefactLocator` is a
  frozen dataclass with five fields: `path: Path`, `stage: int`,
  `arm: str`, `controller: str`, `payload: str`. Non-recursive
  (direct children only); files that don't match the
  `r2_stage*.yaml` glob are silently skipped (a run dir may contain
  caller-supplied logs or scratch); files that half-match (start
  with `r2_stage`, end with `.yaml`, but fail to parse) raise
  `ValueError` rather than being dropped — a half-matching name is
  almost always a writer/reader seam bug. Filename parsing is
  anchored on the known `SUPPORTED_ARMS` prefix and `PAYLOAD_LEVELS`
  suffix so controller names that themselves contain `_` (e.g.
  `crisp_joint_impedance`, `crisp_cartesian_impedance`) parse
  unambiguously. Non-canonical stage shapes (`+1` / `-1` /
  leading-zero) are rejected so the writer's emitted form is the
  sole round-trippable shape. Pure stdlib; no PyYAML / numpy / ROS
  imports. Sibling modules loaded via `importlib` file paths,
  matching the `r2_stage*_theoretical` convention (see the "module
  loading" repo memory). Pinned by 39 unit tests in
  `tests/unit/test_r2_find_artefacts.py`: export surface, parse
  happy-path as a 126-combo Cartesian-product round-trip against
  `artefact_filename`, parse-side validation (12 cases), find-side
  validation (13 cases), discovery semantics (empty dir, skipped
  non-matches, sorted-by-path ordering, non-recursive, half-match
  raises, artefact-named directory raises), filter matrix against a
  25-file populated run dir (single-key filters, all-four
  pulls-one-combo, no-match returns `()`), and an end-to-end
  round-trip with the real `write_r2_artefact` writer. Unit gate
  **1248 passed** (up from 1079). Full `scripts/run_tests.sh` green
  end-to-end: unit (1248) + colcon test (10 packages) + integration
  (12 launch tests × {ur5e, ur15}, 299.74s).

- **Prior iteration: R2 run-directory helper (closes the "caller
  decides run_dir" seam the `r2_run_artefact` writer docstring
  explicitly defers to).** Added `tests/integration/r2_run_dir.py`
  exporting a single narrow function
  `make_r2_run_dir(runs_root=None, *, clock=None, prefix="r2",
  suffix=None, exist_ok=False) -> Path` plus pinned
  `TS_FORMAT = "%Y%m%dT%H%M%SZ"`, `DEFAULT_PREFIX = "r2"`, and
  `DEFAULT_RUNS_ROOT = <repo>/evaluation/runs`. The timestamp format
  matches `evaluation/run_evaluation.py::make_run_dir` verbatim so
  M5's `find_latest_run_dir` lexicographic sort keeps working
  across R2 artefacts in the same `runs/` tree. Directory layout is
  `<runs_root>/<prefix>__<UTC-ts>[__<suffix>]`; the optional
  `suffix` disambiguates parallel R2 sweeps that would otherwise
  collide in the same UTC second. `exist_ok=False` by default so
  sub-second collisions surface as `FileExistsError`; `clock` is
  injectable (`Callable[[], datetime]`) so unit tests pin the
  timestamp without monkey-patching `datetime`. Validation:
  `runs_root` must be `None` or `pathlib.Path` (str rejected — "one
  canonical path type" across the pre-bake chain),
  `prefix`/`suffix` must be non-empty `str` with no `/`, `\`, `__`,
  or surrounding whitespace, `clock()` must return a UTC-aware
  `datetime`. Pure stdlib; no PyYAML / numpy / ROS imports. Pinned
  by 47 unit tests in `tests/unit/test_r2_run_dir.py`: export
  surface (TS_FORMAT verbatim match, DEFAULT_PREFIX / DEFAULT_RUNS_ROOT);
  happy-path matrix (default dir, custom prefix, suffix-after-ts,
  prefix+suffix, None runs_root falls back to DEFAULT_RUNS_ROOT,
  default clock within ±1s of wall-clock now); collision handling
  (FileExistsError without `exist_ok`, same path with `exist_ok=True`);
  lexicographic-sort invariant pinning the justification for M5's
  `find_latest_run_dir`; full validation matrix (str/int runs_root,
  empty/slash/backslash/`__`/leading-ws/trailing-ws prefix or
  suffix, non-str prefix/suffix, non-callable clock, clock
  returning non-datetime / naive / non-UTC, non-bool `exist_ok`);
  and an end-to-end compose test feeding the returned path straight
  into `r2_run_artefact.write_r2_artefact(...)` + `yaml.safe_load`.
  Unit gate **1079 passed** (up from 1032). Full
  `scripts/run_tests.sh` green end-to-end.

- **Prior iteration: R2 stage-3 cartesian gain resolver (closes the
  bringup-controller-YAML → future stage-3 cartesian-response seam
  for `crisp_cartesian_impedance`).** Added
  `tests/integration/r2_stage3_cartesian_gains.py` exporting a single
  narrow function `resolve_cartesian_impedance_gains(controller,
  arm, *, config_dir=None) -> dict` plus pinned
  `SUPPORTED_CONTROLLERS = ("crisp_cartesian_impedance",)`,
  `SUPPORTED_ARMS`, `AXES = ("x","y","z")`, and
  `DEFAULT_CONFIG_DIR = bringup/config`. Returns a fixed two-key
  shape `{"translational": {"x","y","z"}, "rotational":
  {"x","y","z"}}` of finite non-negative floats, read from
  `cartesian_impedance_controller.ros__parameters.task.k_{pos,rot}_{x,y,z}`.
  Scope is intentionally narrow to **one** controller:
  `cartesian_motion` is a position-mode controller whose
  `pd_gains.{trans,rot}_{x,y,z}.p` fields are IK-solver proportional
  gains (different units, different closed-loop semantics), and
  mixing them under the same resolver would invite silent misuse.
  Task-block damping is out of scope (the task block exposes no
  damping keys); nullspace / joint-limit-repulsion fields are also
  out of scope (nullspace for the joint-impedance role is already
  covered by `r2_stage1_gains`). Strict validation mirrors stage-1
  gains: unknown controller/arm → `ValueError`, missing file →
  `FileNotFoundError`, missing key (including any of the six axis
  keys) → `KeyError` with dotted path, non-mapping top-level /
  `task` block → `ValueError`, non-numeric / `bool` stiffness →
  `TypeError`, non-finite or negative stiffness → `ValueError`,
  zero stiffness allowed. Pure stdlib + PyYAML. Pinned by 36 unit
  tests in `tests/unit/test_r2_stage3_cartesian_gains.py`: export
  surface; happy-path matrix over both arms against the
  **committed** `bringup/config/crisp_cartesian_impedance.{ur5e,ur15}.yaml`
  values; ur5e=ur15 cross-arm equality invariant pinning the file
  comment ("Gains follow ur5e for first-light bring-up"); positivity
  sanity; return-type invariants (plain `dict`, plain-float leaves,
  independent per-call instances); argument rejection matrix
  (`cartesian_motion` / `crisp_joint_impedance` → `ValueError`,
  unknown arm, missing file); full temp-YAML error matrix (int-
  coerced-to-float, zero-allowed, every axis key missing, non-
  mapping top-level / task, negative / NaN / inf / non-numeric /
  `bool` K). Unit gate **1032 passed** (up from 996). Full
  `scripts/run_tests.sh` green end-to-end.

- **Prior iteration: R2 stage-1 gain resolver (closes the bringup-
  controller-YAML → `r2_stage1_theoretical` seam so stage-1 test
  bodies don't open-code `yaml.safe_load` + key-walk).** Added
  `tests/integration/r2_stage1_gains.py` exporting a single narrow
  function `resolve_joint_impedance_gains(controller, arm, *,
  config_dir=None) -> dict[str, tuple[float, float]]` plus pinned
  `SUPPORTED_CONTROLLERS = ("simple_joint_impedance",
  "crisp_joint_impedance")`, `SUPPORTED_ARMS`, `CANONICAL_JOINTS`,
  and `DEFAULT_CONFIG_DIR = bringup/config`. Covers the two
  joint-impedance bringup YAMLs: `simple_joint_impedance` reads
  per-joint `k[i]` / `d[i]` arrays; `crisp_joint_impedance` reads
  scalar `nullspace.stiffness` / `nullspace.damping` and broadcasts
  across the six joints. Damping auto-fill: negative `d` value
  resolves to critical damping `2*sqrt(K)`, matching both the
  simple controller's on-activation rule (parameter description in
  `src/simple_joint_impedance_controller/src/simple_joint_impedance_controller.yaml`)
  and crisp's `nullspace.damping: -1.0` sentinel (comment in the
  committed bringup YAMLs). Strict validation: unknown
  controller/arm/missing file/malformed YAML all raise with
  actionable messages (`ValueError` with dotted path,
  `FileNotFoundError`, `KeyError`, `TypeError` for non-numeric or
  `bool` gains). Pure stdlib + PyYAML. Pinned by 42 unit tests in
  `tests/unit/test_r2_stage1_gains.py`: export surface; happy-path
  matrix over both controllers × both arms against the **committed**
  `bringup/config/*.yaml` values (re-reading the same file via
  `yaml.safe_load` for the expected values); crisp auto-damping
  sentinel across both arms; cross-controller sanity (crisp
  broadcasts scalar K, simple does not); UR15 > UR5e stiffness
  invariant; return-type invariants; full temp-YAML error matrix
  (missing keys, length mismatch, duplicates, empty joints,
  negative K, NaN K, inf D, non-numeric, `bool`-as-gain,
  non-mapping top-level); and an end-to-end thread-through feeding
  every resolved `(K, D)` into `theoretical_for_stage1` on both
  arms × both controllers to verify the resulting `response` block
  echoes K/D verbatim and `omega_n_rad_s == sqrt(K / J_eff)`
  against the loader's per-joint `effective_inertia_kg_m2`. Unit
  gate **996 passed** (up from 954). Full `scripts/run_tests.sh`
  green end-to-end.

- **Prior iteration: R2 stage-3 theoretical-block builder (closes the
  expectations-loader → `r2_result_to_artefact` seam for stage-3 and
  completes the R2 theoretical-block pre-bake chain across stages 1,
  2 (both paths) and 3).** Added
  `tests/integration/r2_stage3_theoretical.py` exporting a single
  narrow function `theoretical_for_stage3(arm_exp)` plus a pinned
  `SUPPORTED_TOLERANCE_KEYS` tuple (four TCP keys: `tcp_rmse_mm`,
  `tcp_peak_err_mm`, `tcp_orientation_peak_deg`,
  `tcp_steady_drift_mm_per_30s`). Emits
  `{response_model='tcp_trajectory_tracking', tolerances={...}}`
  sourcing all four values via `arm_exp.tcp_tol(k)` — missing keys
  bubble the loader's native `KeyError` with `available: [...]`
  intact. Strict typing on the input: non-`ArmExpectation` values
  (including `Stage2Tolerances`, `Stage2TcpTolerances`, and a bare
  dict with the right fields) raise `ValueError` — silently
  accepting any of these would drop the "numbers came from an
  expectation YAML" guarantee. Pure stdlib; sibling
  `expectations_loader` resolved via the `sys.modules`-first
  importlib loader from `r2_stage1_theoretical.py`. Pinned by 21
  unit tests in `tests/unit/test_r2_stage3_theoretical.py` (export
  surface + schema-set parity; happy path with plain-float /
  plain-dict leaves and int→float coercion; custom-values matrix;
  argument rejection matrix incl. cross-dataclass swaps;
  missing-tolerance-key bubbling; two round-trips through
  `result_to_artefact(stage=3, ...)` + `write_r2_artefact` +
  `yaml.safe_load` with and without `TcpStage3Result.notes`;
  real-YAML integration matrix over `{ur5e, ur15}`; cross-arm
  equality pinning the schema invariant). Unit gate **954 passed**
  (up from 933).

- **Prior iteration: R2 stage-2 theoretical-block builder (closes the
  expectations-loader → `r2_result_to_artefact` seam for stage-2 on
  both joint-space and cartesian paths).** Added
  `tests/integration/r2_stage2_theoretical.py` exporting two narrow
  functions — `theoretical_for_stage2_joint_space(stage2_tol)` and
  `theoretical_for_stage2_cartesian(stage2_tcp_tol)` — each
  returning `{response_model, tolerances}`. Response models pin the
  ROADMAP R2 stage-2 semantics: `kinematic_consistency_joint_space`
  carries `completion_tol_rad` / `peak_tracking_err_rad` /
  `saturation_hold_ms`; `kinematic_consistency_tcp` carries
  `position_peak_err_mm` / `orientation_peak_err_deg`. Strict
  typing: each function rejects non-matching dataclasses (including
  the other stage-2 tolerance dataclass or a bare dict) with
  `ValueError`. Pure stdlib; sibling `expectations_loader` resolved
  via the `sys.modules`-first loader from
  `r2_stage1_theoretical.py`. Pinned by 28 unit tests in
  `tests/unit/test_r2_stage2_theoretical.py` (export surface +
  `SUPPORTED_MODES`; happy-path matrix for both modes including
  plain-float/plain-dict leaves and custom-value propagation;
  full argument rejection matrix including cross-dataclass swaps;
  two round-trips through `result_to_artefact(stage=2, ...)` +
  `write_r2_artefact` + `yaml.safe_load` on both
  `Stage2Result` (joint-space) and `Stage2CartesianResult`
  branches; real-YAML integration matrix over `{ur5e, ur15}` for
  both paths; cross-arm equality check). Unit gate **933 passed**
  (up from 905).

- **Prior iteration: R2 stage-1 theoretical-block builder (closes
  the expectations-loader → `r2_result_to_artefact` seam for
  stage-1).**
  Added `tests/integration/r2_stage1_theoretical.py` exporting
  `theoretical_for_stage1(controller_exp, *, joint_exp=None,
  stiffness_k=None, damping_d=None) -> dict`. Dispatches on
  `controller_exp.response_model`: `first_order_lag` and
  `open_loop_torque` return `{response_model, interface, tolerances}`
  from the controller's tolerance row; `second_order` additionally
  computes `omega_n_rad_s`/`zeta` via
  `expectations_loader.second_order_response(K, D, J_eff)` and
  echoes K / D / J / joint name under a `response` sub-block. Strict
  kwarg matching: extraneous `joint_exp`/`stiffness_k`/`damping_d`
  for non-second-order controllers raises (silent K/D ignore is a
  test-authoring trap). K/D remain caller-supplied, keeping this
  module decoupled from `bringup/config/*.yaml`. Pure stdlib; sibling
  loader uses the `sys.modules`-first convention from
  `r2_result_to_artefact.py`. Pinned by 52 unit tests in
  `tests/unit/test_r2_stage1_theoretical.py` (export surface;
  happy-path matrix for all three response models; extraneous-kwarg
  rejection; missing-required-kwarg rejection for second-order;
  delegation to the loader for K≤0 / D<0; missing-tolerance-key
  bubbling of the loader's `KeyError`; unknown-response-model path;
  type rejection matrix; three round-trips through
  `result_to_artefact` + `write_r2_artefact` + `yaml.safe_load`; and
  a real-YAML integration matrix over {ur5e, ur15} × all six known
  stage-1 controllers). Unit gate **905 passed** (up from 853).

- **Prior iteration: R2 result → artefact bridge (closes the last
  stage-harness → writer seam).** Added
  `tests/integration/r2_result_to_artefact.py` exporting a single
  function `result_to_artefact(*, stage, arm, controller, payload,
  theoretical, result, metadata=None) -> R2Artefact`. Dispatches on
  `stage`: `stage=1` → `Stage1Result` (adds `joint` to metadata);
  `stage=2` → `Stage2Result` (flattens per-joint metrics into
  `{joint}.{metric}` keys, records ordering under
  `metadata['joints']`) or `Stage2CartesianResult` (pass-through);
  `stage=3` → `TcpStage3Result` (non-empty `notes` attached under
  `metadata['stage3_notes']`). `passed` / `reasons` pulled from
  `result.ok` / `result.failures`. Controller-echo check, reserved
  metadata keys raise on clash, full argument rejection matrix.
  Sibling loader checks `sys.modules` for the plain name first
  (namespaced fallback for isolation) so `isinstance` works across
  both the unit-test direct loader and the bridge's own loader —
  pinned by 3 round-trip tests through `write_r2_artefact`. Pure
  stdlib. Pinned by 57 unit tests in
  `tests/unit/test_r2_result_to_artefact.py` — unit gate now
  reports **853 passed** (up from 796).

- **Prior iteration: R2 run-artefact writer (closes the R2
  "publish theoretical alongside measured" seam).** Added
  `tests/integration/r2_run_artefact.py` exporting the frozen
  dataclass `R2Artefact` + `write_r2_artefact(run_dir, artefact,
  *, overwrite=False) -> Path` + `artefact_filename(...)` helper +
  `SCHEMA_VERSION=1` + `SUPPORTED_STAGES=(1,2,3)`. Filename is
  derived (`r2_stage{stage}_{arm}_{controller}_{payload}.yaml`)
  so the M5 compare driver can glob R2 artefacts without a
  schema-file lookup. Output is deterministic: pinned top-level
  key order (`schema_version` first) + nested dict keys sorted +
  `tuple -> list` + `Mapping -> dict` normalisation. JSON-safe
  leaf types only (`str` / `int` / `float` / `bool` / `None`);
  NaN / ±Inf and unsupported types (`set`, `bytes`, arbitrary
  objects) rejected with a path-locator error (e.g.
  `theoretical.response.zeta: non-finite float nan`).
  `passed=False` requires non-empty `reasons`; pre-write
  validation runs before any I/O; existing files are not
  overwritten unless `overwrite=True`; `run_dir` is auto-created.
  Pure stdlib + PyYAML. Pinned by 55 unit tests in
  `tests/unit/test_r2_run_artefact.py` — unit gate now reports
  **796 passed** (up from 741).

- **Prior iteration: R3 MJCF payload stripper (closes M6.17
  swap-in-place seam).** Added `tests/integration/r3_payload_strip.py`
  exporting `strip_payload_from_mjcf(mjcf, *, body_name="ee_payload")
  -> str` plus a re-exported `DEFAULT_BODY_NAME` sentinel. Inverse of
  `r3_payload_splice.py`: walks the MJCF for `<body
  name="<body_name>">`, removes the single match from its parent, and
  re-serialises via `ET.tostring(..., encoding="unicode")`. Parent
  lookup uses a one-pass local parent-map (`{child: parent for parent
  in root.iter() for child in parent}`) since ElementTree lacks a
  built-in back-pointer — MJCF documents are O(thousands) of elements
  so the map is cheap. Idempotent on inputs without the target body:
  if `body_name` does not appear textually, the parse is skipped and
  the input is returned byte-identical (crucial for the `no_payload`
  zero-mass baseline, where the splicer already returns
  byte-identical input, so `strip(splice(mjcf, no_payload)) == mjcf`
  holds losslessly and the malformed-MJCF zero-mass-baseline
  contract from the splicer is preserved). Ambiguous matches (more
  than one body with the same name) raise `ValueError` — a
  double-splice or collision is the caller's to resolve.
  `strip → splice` composition enables an in-place payload swap
  (updating the dashboard-published payload via the ADR-0012 §R3
  contract without re-regenerating the MJCF from source), without
  tripping the splicer's double-splice guard; pinned by a dedicated
  interop test that swaps payload `p1` (1 kg @ z=0.05) for payload
  `p2` (3 kg @ z=0.1) and reads the mass back off the resulting
  `<inertial>`. Pure stdlib; imports only `xml.etree.ElementTree`
  and the sibling emitter's `DEFAULT_BODY_NAME`. Pinned by 34 unit
  tests in `tests/unit/test_r3_payload_strip.py`: export surface
  (`__all__`, callable, default constant), idempotence matrix (no
  match, malformed-when-absent-fast-path, empty string,
  double-strip), splice↔strip semantic round-trip via
  `ET.tostring`-canonicalisation, body actually removed (iterfind
  empty post-strip), anchor (`tool0`) preserved, siblings
  (`<inertial>` + `<geom>`) left in place, strip under deeply-nested
  anchor (four levels), all three in-tree catalog payloads
  (`no_payload` / `small_payload` / `large_payload`) round-trip,
  zero-mass strict byte-identity, custom `body_name` override +
  textual false-positive (body name appearing inside an unrelated
  attribute must not cause a spurious strip), ambiguous-match
  rejection (two bodies with the same name anywhere in the tree),
  full input-validation matrix (`mjcf` non-str including `None` /
  `int` / `float` / `bytes` / `list` / `dict`; malformed MJCF when
  the body_name substring is present; non-str / empty /
  whitespace-bearing `body_name` including embedded space / tab /
  newline), and the in-place swap-payload composition. Unit gate
  now reports **741 passed** (up from 707). Full
  `scripts/run_tests.sh` green end-to-end: unit (741) + colcon
  test (10 packages, 22 `test_math` + 5 `test_filters` + 4
  `test_pseudo_inverse` gtests) + integration (12 launch tests ×
  {ur5e, ur15}), ~5:26 wall clock for the integration slice.
- **Prior iteration: R3 `EePayload` message-shape builder
  (pre-bakes M6.16 ROS runtime-API plumbing).** Added
  `tests/integration/r3_payload_ee_msg.py` exporting the frozen
  dataclass `EePayloadMessage(mass_kg,
  inertia_row_major: Tuple[9 floats], pose_position_xyz: Tuple[3],
  pose_orientation_xyzw: Tuple[4])` plus
  `payload_to_ee_msg(payload) -> EePayloadMessage`. Converts a
  validated :class:`Payload` into the shape ADR-0012 addendum
  §R3.1 specifies for `ur_sim_msgs/EePayload`: `mass` (double,
  kg), `inertia` as a symmetric 9-double row-major tensor
  `[ixx, ixy, ixz, ixy, iyy, iyz, ixz, iyz, izz]` (v1 validator
  pins off-diagonals to zero; symmetric layout future-proofs a
  relaxation), and `pose` as `(x, y, z)` position +
  `geometry_msgs/Quaternion`-shaped `(x, y, z, w)` orientation.
  Reuses the sibling `r3_payload_mjcf._euler_xyz_to_wxyz` helper
  and reorders `(w, x, y, z) -> (x, y, z, w)` so there's a single
  source of truth for the intrinsic-XYZ convention (prevents
  MJCF-vs-ROS drift in the rotation representation).
  **Zero-mass payloads produce a well-formed message with the
  loader-supplied pose preserved** (ADR-0012 §R3.2 pins zero mass
  as the sim's launch state; the loader doesn't tie pose to
  mass, so canonicalizing to identity would be data loss — this
  was an explicit rubber-duck finding and is pinned by dedicated
  tests). Exposes `EePayloadMessage.to_dict()` returning the
  ROS-message-shaped dict so a ROS-side adapter can populate a
  real `ur_sim_msgs/EePayload` field-by-field. Pure stdlib; no
  numpy, no ROS. Pinned by 27 unit tests in
  `tests/unit/test_r3_payload_ee_msg.py`: export surface, frozen
  dataclass contract + `FrozenInstanceError`, all three catalog
  payloads (`no_payload`, `small_payload`, `large_payload`)
  round-trip, inertia symmetric 9-tuple layout and length,
  pose-position echo (including negative components),
  zero-mass-preserves-nonzero-pose (position + orientation
  separately), canonical rotations about X / Y / Z at π/2,
  unit-norm invariant for arbitrary rpy, `xyzw == wxyz reordered`
  cross-check against the MJCF-side helper, last-component
  sanity (identity rpy → `w == 1` at index 3), `to_dict()`
  shape / inertia-as-list, and the validator-propagation matrix
  (negative mass, non-diagonal `ixy`, zero-mass-with-nonzero-
  inertia, NaN in `pose_xyz` / `pose_rpy`). Unit gate now reports
  **707 passed** (up from 680). Full `scripts/run_tests.sh` green
  end-to-end: unit (707) + colcon test (10 packages, 22
  `test_math` + 5 `test_filters` + 4 `test_pseudo_inverse`
  gtests) + integration (12 launch tests × {ur5e, ur15}), ~4:48
  wall clock for the integration slice.
- **Prior iteration: R3 test-parametrisation helper (pre-bakes the
  M6.19 `{arm} × {payload}` matrix).** Added
  `tests/integration/r3_payload_parametrize.py` exporting
  `arm_payload_combinations(arms=None, payload_names=None, *,
  expect_dir=None) -> Tuple[Tuple[str, str], ...]` and
  `arm_payload_ids(combinations) -> Tuple[str, ...]`. Defaults
  enumerate the full ROADMAP matrix: outer =
  `expectations_loader.SUPPORTED_ARMS`, inner = catalog-ordered
  `load_payloads()`. Pinned by 40 unit tests.
- **Prior iteration: R3 MJCF payload splicer (pre-bakes M6.17
  document-level seam).** Added
  `tests/integration/r3_payload_splice.py` exporting
  `splice_payload_into_mjcf(mjcf, payload, *,
  attach_body="tool0", body_name="ee_payload") -> str` plus the
  `DEFAULT_ATTACH_BODY = "tool0"` constant (re-exports
  `DEFAULT_BODY_NAME` from the emitter). Given a full MJCF document
  string and a validated :class:`Payload`, appends the emitter's
  `<body>` snippet as the last child of the anchor body. Zero-mass
  payloads short-circuit before parsing — the input string is
  returned as-is, so the `no_payload` baseline is byte-identical
  even when the MJCF is malformed or missing the anchor. Positive-
  mass path: parses with `xml.etree.ElementTree.fromstring`, finds
  the anchor via `.//body[@name='<attach_body>']` (any depth),
  enforces exactly-one match, enforces no pre-existing body with
  `body_name` anywhere in the tree (double-splice guard covering
  both "sibling of anchor" and "already-attached-under-anchor"
  cases), appends the parsed snippet as the anchor's last child,
  and serialises via `ET.tostring(root, encoding="unicode")`.
  Attribute / element order are preserved by ElementTree (Python
  3.8+); whitespace formatting is not byte-preserved, but MuJoCo
  is whitespace-insensitive — callers that want byte-identity are
  the zero-mass path, which already has it. Validator errors from
  `payload_validation.validate_payload` propagate unchanged via
  the emitter. Pure stdlib; imports only `xml.etree.ElementTree`
  plus the sibling emitter / loader. Pinned by 44 unit tests in
  `tests/unit/test_r3_payload_splice.py`: export surface
  (`__all__`, callable, default constants), zero-mass byte-identity
  (well-formed MJCF, MJCF with no tool0, malformed MJCF, ignored
  `body_name`), positive-mass happy path (single `ee_payload` child
  under `tool0`, correct `pos` / `mass`, appended AFTER a pre-
  existing `<inertial>` sibling, siblings of the anchor untouched,
  deeply-nested anchor at four levels, custom `attach_body`, custom
  `body_name`, output round-trips through the XML parser, emitted
  `quat` is unit-norm), in-tree catalog interop (all three of
  `no_payload` / `small_payload` / `large_payload` via
  `load_payloads()`), and the full rejection matrix: non-str
  `mjcf` (`None`, `int`, `float`, `bytes`, `list`), malformed XML
  (positive-mass only), non-str / empty / whitespace `attach_body`
  (space / tab / newline / embedded space), same matrix for
  `body_name`, missing anchor (default + custom `attach_body`),
  ambiguous anchor (default + custom), existing `body_name` as
  sibling, existing `body_name` under anchor, double-splice via
  two consecutive calls, validator-error propagation for negative
  mass, and validator-error propagation for non-diagonal inertia.
  Unit gate now reports **640 passed** (up from 596). Full
  `scripts/run_tests.sh` green end-to-end: unit (640) + colcon
  test (10 packages, 22 `test_math` + 5 `test_filters` + 4
  `test_pseudo_inverse` gtests) + integration (12 launch tests ×
  {ur5e, ur15}), ~4:47 wall clock for the integration slice.
- **Prior iteration: R3 MJCF payload body emitter (pre-bakes M6.17
  string-assembly seam).** Added
  `tests/integration/r3_payload_mjcf.py` exporting
  `payload_body_mjcf(payload, *, body_name="ee_payload") -> str`
  and a `DEFAULT_BODY_NAME = "ee_payload"` constant. Given a
  validated :class:`Payload`, emits a single-line, well-formed MJCF
  `<body>` snippet suitable to splice as a child of any MuJoCo
  body (in our case `tool0`). Attributes: `name` (from
  `body_name`), `pos="x y z"` (metres, from `pose_xyz`), `quat="w
  x y z"` (Hamilton order, derived from `pose_rpy` via the
  closed-form intrinsic XYZ Euler product `qx(r) ⊗ qy(p) ⊗
  qz(y)`). Exactly one `<inertial pos="0 0 0" mass="M"
  diaginertia="ixx iyy izz"/>` child. Zero-mass payloads return
  `""` (sentinel — validator pins `mass == 0 ⇔ inertia == 0`, so
  `no_payload` produces no body and leaves the MJCF byte-identical
  to the pre-attachment tree). Re-runs `validate_payload` on entry
  so invalid inputs are rejected at this seam rather than mutating
  the MJCF. `body_name` must be a non-empty whitespace-free `str`
  (MJCF identifiers are whitespace-free; accepting whitespace
  would silently break downstream MJCF parsers). `%.15g` numeric
  formatting gives round-trip precision without repr-noise zeros.
  Uses `quat` in preference to `euler` so the snippet is
  independent of the enclosing MJCF's `<compiler eulerseq="...">`
  setting. Pure stdlib; imports only `math` and the sibling
  `payload_validation.validate_payload` + loader `Payload`
  dataclass. Pinned by 34 unit tests in
  `tests/unit/test_r3_payload_mjcf.py`: export surface
  (`__all__`, callable, default constant), zero-mass empty-string
  emission (synthetic + in-tree `no_payload`, including
  `body_name` ignored for zero-mass), positive-mass happy path
  (well-formed XML via `xml.etree.ElementTree`, attribute values,
  exactly-one `<inertial>` child), `body_name` override, both
  in-tree positive payloads round-trip (`small_payload` at z =
  0.05 m, 1 kg, diag 1.67e-3; `large_payload` at z = 0.1 m, 5 kg,
  diag 2.44e-2), quat identity for `rpy=(0,0,0)`, canonical
  rotations about each axis (`Rx`, `Ry`, `Rz` at `π/2`) with
  closed-form checks to `abs=1e-12`, unit-norm invariant for an
  arbitrary rotation, composed rotations (`r=π/2, p=π/2, y=0` →
  `(0.5, 0.5, 0.5, 0.5)`; `r=p=y=π/2` → `(0, √2/2, 0, √2/2)`),
  negative `pose_xyz` components, 15-significant-digit precision,
  the full `body_name` rejection matrix (empty, single-space,
  embedded spaces/tabs/newlines, non-str types including `None`,
  `int`, `float`, `bytes`, `list`), validator propagation
  (negative mass, non-diagonal `ixy`, NaN in `pose_xyz`,
  zero-mass-with-nonzero-inertia), and snippet-shape sanity
  (single-line, no leading/trailing whitespace, exactly-one
  `inertial` child, emitted `name` matches `DEFAULT_BODY_NAME`).
  Unit gate now reports **596 passed** (up from 562). Full
  `scripts/run_tests.sh` green end-to-end: unit (596) + colcon
  test (10 packages, 22 `test_math` + 5 `test_filters` + 4
  `test_pseudo_inverse` gtests) + integration (12 launch tests ×
  {ur5e, ur15}), ~4:51 wall clock for the integration slice.
- **Prior iteration: R3 payload validator pre-bake (closes the
  preflight seam shared by M6.16 + M6.17).** Added
  `tests/integration/payload_validation.py` exporting
  `validate_payload(payload: Payload) -> None` and
  `validate_catalog(catalog: PayloadCatalog) -> None`. Both raise
  `ValueError` with a self-locating message (every error echoes
  `payload '<name>'`) that names the offending field and value.
  v1 rule set, each pinned by its own failure test:
  mass finiteness + non-negativity; every inertia key present and
  finite; diagonal-only tensor (`ixy == ixz == iyz == 0` exactly —
  v1 targets MuJoCo's `diaginertia`, documented as a v1 policy not
  a physics claim); sentinel consistency `mass == 0 ⇔ inertia ==
  0` (zero-mass-with-nonzero-inertia and mass-without-inertia both
  rejected); v1 modelling constraint banning point masses (`mass >
  0 ⇒ ixx, iyy, izz > 0`); triangle inequality on the three
  principal moments with all three cyclic permutations reported
  separately for self-locating failures; finite pose (`pose_xyz`,
  `pose_rpy` as 3-tuples of finite floats). `validate_catalog`
  additionally rejects duplicate payload names (catalog-level
  invariant — `PayloadCatalog.get()` would be ambiguous under
  collision). Pure stdlib; imports only `math` and the existing
  `expectations_loader.Payload` dataclass, so ROS-free tooling can
  consume it. Pinned by 40 unit tests in
  `tests/unit/test_payload_validation.py`: export surface,
  happy-path on all three built-in payloads via `load_payloads()`,
  synthetic happy-path (zero-mass/zero-inertia; positive-mass
  diagonal; triangle-inequality equality boundary; nonzero pose),
  and the full rejection matrix — empty name, negative mass, NaN
  / +inf / -inf mass and inertia (separately per rubber-duck
  guidance), missing inertia key, non-zero / negative
  off-diagonal (parametrised over `ixy, ixz, iyz`), zero-mass with
  nonzero inertia, zero / negative principal moment with positive
  mass (parametrised over `ixx, iyy, izz`), all three triangle-
  inequality violation directions, wrong-length / NaN / inf pose
  triplets for both `pose_xyz` and `pose_rpy`, error-message name
  echo, catalog with bad entry, catalog with duplicate names,
  catalog with unique names (happy), empty catalog (happy). Unit
  gate now reports **562 passed** (up from 522). Full
  `scripts/run_tests.sh` green end-to-end: unit (562) + colcon
  test (10 packages, 22 `test_math` + 5 `test_filters` + 4
  `test_pseudo_inverse` gtests) + integration (12 launch tests ×
  {ur5e, ur15}), ~5:01 wall clock.
- **Prior iteration: stage-2 cartesian-mode evaluator (closes the
  deferred "FK / TCP-level checks" note in
  `r2_stage2_assertions.py`).** Added
  `tests/integration/r2_stage2_cartesian.py` exporting
  `Stage2CartesianResult` (frozen dataclass with `ok` + `format()`)
  and `evaluate_all_joints_cartesian(controller, *, commanded_tcp,
  measured_tcp, position_peak_err_mm, orientation_peak_err_deg) ->
  Stage2CartesianResult`, plus an `_from_expectation` wrapper sourcing
  the 5 mm / 2° bounds from the loader's new `stage2_tcp` accessor.
  Takes two :class:`TcpTrajectory` (duck-typed via
  `times` / `positions` / `orientations` attributes so the FK-adapter
  and the commanded-side generator both qualify without an
  `isinstance` check — same cross-loader posture as
  `r2_joints_from_tcp.py`). Asserts the ROADMAP R2 wording exactly:
  peak position error ≤ 5 mm, peak per-axis (roll/pitch/yaw)
  orientation error ≤ 2°; uses the same intrinsic-XYZ Euler
  decomposition and antipodal-quaternion handling as
  `r2_stage3_assertions._quat_relative_rpy_deg` so stage-2 and
  stage-3 numbers are directly comparable. No RMSE / no drift check
  (stage-2 text doesn't ask for them — those are stage-3-only).
  Validation matrix: non-positive / non-finite tolerances,
  missing-attribute inputs (TypeError), sample-count mismatch
  (explicit "does not resample" error — caller must align
  timebases), < 2 samples, non-monotonic / non-finite times,
  wrong-length / non-finite positions, wrong-length / non-finite
  quaternions, quaternion norm outside the [0.5, 1.5] band that the
  sibling modules already use. Pure stdlib; no numpy, no ROS, no
  MuJoCo. Also extended `expectations_loader.py` with
  `Stage2TcpTolerances` dataclass and `ArmExpectation.stage2_tcp`
  field, added the `stage2_tcp: {position_peak_err_mm: 5.0,
  orientation_peak_err_deg: 2.0}` block to both
  `expectations/ur5e.yaml` and `expectations/ur15.yaml`, and pinned
  block equality across arms in `test_expectations_schema.py`
  (matches the ROADMAP R2 "pass/fail thresholds are the same"
  rule). Updated the `r2_stage2_assertions.py` module docstring
  so the previously-deferred cartesian variant now points to the
  new module. Pinned by 33 unit tests in
  `tests/unit/test_r2_stage2_cartesian.py`: export surface,
  frozen-dataclass property, `ok` / `format()`, identical
  trajectories → zero error / `ok=True`, antipodal quaternion pair
  yields zero orientation error (short-arc rule), position offsets
  reported in mm (not m), over-tolerance position fails with a
  single failure string, orientation rotation about Z measured in
  degrees to 1e-6, over-tolerance orientation fails, both
  tolerances violated → two failures reported, peak taken over the
  whole trace (linearly growing drift), controller-name echo, the
  full tolerance-validation matrix (zero / negative / NaN / inf for
  both tolerances), and the full trajectory-shape validation
  matrix. Expectation-driven wrapper is exercised with both
  in-tree `ur5e` and `ur15` YAMLs plus a duck-typed fake
  `ArmExpectation` so the `_from_expectation` contract is
  dataclass-structural, not module-typed. Unit gate now reports
  **522 passed** (up from 489 — +33 cartesian evaluator tests plus
  +1 schema equality test). Full `scripts/run_tests.sh` green
  end-to-end: unit (522) + colcon test (10 packages, 22 `test_math`
  + 5 `test_filters` + 4 `test_pseudo_inverse` gtests) +
  integration (12 launch tests × {ur5e, ur15}), ~5:10 wall clock.
- **Prior iteration: JTC goal-builder pre-bake (closes the
  `(joint_names, times, positions)` → `JointTrajectoryGoal` seam).**
  Added `tests/integration/r2_jtc_goal.py` exporting
  `JointTrajectoryPoint`, `JointTrajectoryGoal` (both frozen
  dataclasses), and `jtc_goal_from_trace(trace, *, time_offset_s=0.0,
  skip_initial_sample=False)`. Duck-types the input — any object
  exposing `joint_names`, `times`, `positions` qualifies, which covers
  all three existing trace producers (`JointCommandTrace` from
  stage-1 commands, `AllJointsCommandTrace` from stage-2 commands,
  `JointCommandTrajectory` from the IK-injected adapter). Emits a
  `JointTrajectoryGoal` whose `points` are ordered per `joint_names`
  (column-major → row-major transpose) and whose
  `time_from_start_s` is rebased from `times[start]` so traces that
  don't begin at `t=0` still yield a `time_from_start >= 0` goal.
  Options: `time_offset_s` (non-negative additive shift, for JTC
  implementations that reject a first point at `time_from_start=0`);
  `skip_initial_sample` (drop `times[0]`, required when the
  commanded first sample equals the robot's current state to avoid
  the instantaneous-jump pathology). Validates non-empty / unique
  str `joint_names`, strictly monotonic finite `times`, that
  `positions` is a mapping whose keys match `joint_names` exactly
  (missing = error, extra = error — the latter prevents data from
  silently disappearing), per-joint sample counts, finite numeric
  positions, and the `time_offset_s` / `skip_initial_sample`
  preconditions. Pure stdlib; no numpy, no ROS — the final
  `JointTrajectoryGoal -> trajectory_msgs/JointTrajectory`
  materialiser is deliberately left out of tree so it can import
  ROS at test-run time without polluting the unit gate. Pinned by
  38 unit tests in `tests/unit/test_r2_jtc_goal.py`: export surface,
  both dataclasses' frozen contract, `__len__` / `duration_s`,
  empty-goal duration, preserving joint order, time forwarding,
  non-zero-start rebasing, `time_offset_s` additive application,
  `skip_initial_sample` drops-and-rebases, option composition,
  dict-order independence (positions ordered by `joint_names` not
  insertion), list vs tuple inputs, single-sample trace, and
  `MappingProxyType` positions transparent handling. Validation
  matrix covers: missing attribute (raises `TypeError`), empty /
  duplicate / non-str joint names, empty / non-monotonic / repeated
  / non-finite / non-numeric times, missing / extra / non-mapping
  positions, wrong sample count, non-finite / non-numeric position
  values, negative and non-finite `time_offset_s`, and
  `skip_initial_sample` on a single-sample trace. Interop tests
  drive the builder from `stage1.step_command`,
  `stage1.sine_command`, `stage2.home_to_pose_to_home_command`, and
  `jft.joint_trajectory_from_tcp` (fed by
  `stage3.line_trajectory`), confirming shape-identity across all
  three trace families. Unit gate now reports **488 passed** (up
  from 450). Full `scripts/run_tests.sh` green end-to-end: unit
  (488) + colcon test (10 packages, 22 `test_math` + 5
  `test_filters` + 4 `test_pseudo_inverse` gtests) + integration
  (12 launch tests × {ur5e, ur15}), ~5 min wall clock.
- **Prior iteration: IK-injected adapter (pre-bake for M6.14 `JTC +
  ik_shim` combo).** Added `tests/integration/r2_joints_from_tcp.py`
  exporting `IkCallable`, `JointCommandTrajectory` (frozen dataclass:
  `joint_names`, `times`, read-only `MappingProxyType` `positions`,
  `__len__` and `duration_s` property — shape-compatible with stage-2's
  `AllJointsCommandTrace.positions` so a JTC goal-builder consumes
  either without branching), and `joint_trajectory_from_tcp(trajectory,
  ik, *, joint_names, q_seed)`. Injected IK contract:
  `ik(pos_xyz_m, quat_xyzw, q_seed) -> Sequence[float]` of exactly
  `len(joint_names)` finite floats — caller-supplied `q_seed` is used
  for the first sample only; every subsequent call is seeded with the
  previous solution so branch continuity only needs a good initial
  guess. `times` forwarded verbatim from the input `TcpTrajectory`
  (inherits its `times[0] == 0.0` and strict-monotonic invariants for
  free). Trajectory input accepted by **duck-type** (`times` +
  `positions` + `orientations` attributes) rather than `isinstance`,
  because `tests/integration/` is not a package on `sys.path` — the
  adapter and the test harness each load `TcpTrajectory` through their
  own file-path `sys.modules` key and `isinstance` would reject
  legitimately-shaped inputs across those two loader keys. IK
  exceptions re-raised as `ValueError` carrying `sample {i}
  (t={t}s)` context; `None` / non-sequence / wrong-length / non-finite
  / non-numeric returns rejected with the same style. Validation
  surface: non-`TcpTrajectory`-shaped input raises `TypeError`; empty
  or duplicate `joint_names`, seed length mismatch, non-finite /
  non-numeric seed all raise `ValueError`. Pure stdlib; no numpy, no
  ROS, no MuJoCo. Pinned by 30 unit tests in
  `tests/unit/test_r2_joints_from_tcp.py`: export surface, frozen
  dataclass property, `__len__` / `duration_s`, times forwarded
  verbatim, `MappingProxyType` read-only contract, positions keyed by
  joint names with correct length, degenerate single-sample
  `duration_s` = 0, seed-echo produces constant joint streams, IK
  receives TCP position and quaternion verbatim, seed threaded from
  previous output (counter-IK ramp `1, 2, 3, 4, 5`), first seed is the
  caller-supplied one (subsequent seeds are prior outputs), the full
  validation matrix (non-`TcpTrajectory` shape, empty / duplicate
  joint names, seed length mismatch, non-finite and non-numeric seed,
  IK-exception wrap with correct sample index, IK returning `None` /
  non-sequence / wrong length / non-finite / non-numeric), and
  interop with `arc_trajectory` and `sine_in_z_trajectory`. Unit
  gate now reports **450 passed** (up from 420). Full
  `scripts/run_tests.sh` green end-to-end: unit (450) + colcon test
  (10 packages, 22 `test_math` + 5 `test_filters` + 4
  `test_pseudo_inverse` gtests) + integration (12 launch tests ×
  {ur5e, ur15}), ~5 min wall clock.
- **Prior iteration: R2 stage-2 commanded-signal generator
  (pre-bake for M6.13).** Added
  `tests/integration/r2_stage2_commands.py` exporting
  `AllJointsCommandTrace` (frozen dataclass: `joint_names`,
  `times`, read-only `positions` mapping, `home_positions_rad`,
  `via_pose_rad`) and `home_to_pose_to_home_command(...)`. Every
  joint interpolates independently with a cosine pulse
  `q_i(t) = home_i + (via_i - home_i) * 0.5 * (1 - cos(2π t/T))`,
  giving `q_i(0) == q_i(T) == home_i`, `q_i(T/2) == via_i`, and
  zero velocity at `t ∈ {0, T/2, T}` — any chatter in the
  measured trace is then a controller artefact, not a commanded-
  signal artefact. `times[0] == 0.0` and `times[-1] ==
  duration_s` exactly, matching the sibling stage modules'
  invariant. Validation raises `ValueError` on empty / duplicate
  `joint_names`, `home_positions_rad` / `via_pose_rad` length
  mismatch, non-finite home / via, non-positive / non-finite
  `duration_s`, `dt_s` below `_MIN_DT_S = 1e-5`, `dt_s >
  duration_s`, and any `duration_s / dt_s` that rounds to fewer
  than 2 intervals (so the midpoint via sample always lands in
  the trace). Positions mapping is wrapped in
  `types.MappingProxyType` so callers can't mutate it. Pure
  stdlib; no numpy, no ROS. Pinned by 24 unit tests in
  `tests/unit/test_r2_stage2_commands.py`: export surface,
  frozen-dataclass property, `__len__` / `duration_s`,
  echo-of-home-and-via, exact time endpoints, q(0) = q(T) =
  home, midpoint-sample == via, waveform matches the closed-form
  expression to 1e-12, zero-motion joint stays constant while
  other joints still move, finite-difference velocity at
  endpoints below 5e-3, `MappingProxyType` read-only contract,
  and the full validation matrix. An interop test feeds the
  commanded trace in as both sides to
  `r2_stage2_assertions.evaluate_all_joints_joint_space` and
  asserts `result.ok` and `result.failures == ()`, so a future
  orchestrator pairing the two modules is guaranteed to meet the
  evaluator's input contract. Unit gate now reports **420
  passed** (up from 396). Full `scripts/run_tests.sh` green
  end-to-end: unit (420) + colcon test (10 packages, 22
  `test_math` + 5 `test_filters` + 4 `test_pseudo_inverse`
  gtests) + integration (12 launch tests × {ur5e, ur15}), ~5
  min 30 s wall clock.
- **Prior iteration: R2 stage-1 commanded-signal generators
  (pre-bake for M6.12).** Added
  `tests/integration/r2_stage1_commands.py` exporting
  `JointCommandTrace` (frozen dataclass: `joint_names`, `times`,
  read-only `positions` mapping, `active_joint`, scalar
  `target_rad`), `step_command(...)`, and `sine_command(...)`.
  `step_command` emits a single-joint step of `step_rad` radians
  firing at `t_step_s` (default 0), with all inactive joints
  held at their home positions for every sample; `target_rad` is
  `home + step_rad` so a stage-1 orchestrator can pass it straight
  through as the scalar `target` kwarg of
  `r2_stage1_assertions.evaluate_position_mode` /
  `evaluate_second_order`. `sine_command` emits
  `q_cmd(t) = home + amplitude_rad * sin(2π·frequency_hz·t +
  phase_rad)` on the active joint (0.5 Hz is the ROADMAP
  baseline), inactive joints held at home, and records
  `target_rad = home(active_joint)` so the stage-1 evaluators
  treat the DC offset as the steady-state reference. Both
  generators share a `_validate_timing` clone of the stage-3
  module's helper so `times[0] == 0.0` and `times[-1] ==
  duration_s` exactly, matching the `TcpTrajectory` invariant.
  Raises `ValueError` on empty / duplicate `joint_names`,
  `home_positions_rad` length mismatch, non-finite home /
  step / amplitude / frequency / phase / `t_step_s`,
  non-positive `duration_s`, `dt_s` below `_MIN_DT_S = 1e-5`,
  `dt_s > duration_s`, `t_step_s` outside `[0, duration_s]`,
  non-positive `frequency_hz`, and `active_joint` not in
  `joint_names`. Positions mapping is wrapped in
  `types.MappingProxyType` so callers can't mutate it. Pure
  stdlib; no numpy, no ROS. Pinned by 35 unit tests in
  `tests/unit/test_r2_stage1_commands.py`: export surface,
  frozen-dataclass property, `__len__` / `duration_s`,
  step-shape / endpoints, default-`t_step_s` zero-delay path,
  mid-trajectory step firing, negative step magnitude, inactive
  joints held at home, `MappingProxyType` read-only contract,
  sine waveform matches the closed-form expression to 1e-12,
  phase offset applied (`phase_rad=π/2` → cosine starts at
  `home+amp`), zero-amplitude degenerate case, and the full
  validation matrix for both generators. An interop test feeds
  a `step_command` trace (perfectly-tracked) into
  `r2_stage1_assertions.evaluate_position_mode` via a duck-typed
  `_FakeControllerExp` and asserts `result.failures == ()`, so a
  future orchestrator pairing the two modules is guaranteed to
  meet the evaluator's input contract. Unit gate now reports
  **396 passed** (up from 361). Full `scripts/run_tests.sh`
  green end-to-end: unit (396) + colcon test (10 packages,
  22 `test_math` + 5 `test_filters` + 4 `test_pseudo_inverse`
  gtests) + integration (12 launch tests × {ur5e, ur15}), ~5
  min wall clock.
- **Prior iteration: FK-injected TCP adapter (pre-bake glue for R2
  stage-2 cartesian consistency + R2 stage-3 measured side).**
  Added `tests/integration/r2_tcp_from_joints.py` exporting
  `tcp_trajectory_from_joints(times, joint_samples, fk) ->
  TcpTrajectory`.
- **Prior iteration: R2 stage-3 commanded TCP trajectory generators.**
  Added `tests/integration/r2_stage3_commands.py` exporting
  `TcpTrajectory` + `line_trajectory` / `arc_trajectory` /
  `sine_in_z_trajectory`, pinned by 30 unit tests.
- **Prior iteration: R2 stage-3 TCP assertion harness.** Added
  `tests/integration/r2_stage3_assertions.py` with
  `evaluate_tcp_trajectory(...)` and an expectation-driven
  wrapper.
- **Prior iteration: stage-2 evaluator ArmExpectation wrapper.**
- **Prior iteration: stage-2 evaluator terminal settle window.**
- **Prior iteration: R2 stage-2 tolerance block in the expectation
  YAMLs.**
- **Prior iteration: R2 stage-2 assertion harness (joint-space).**
- **Prior iteration: R2 stage-1 assertion harness** pairs
  expectations with measured signals for the three response models.
- **Prior iteration: R2 signal-analysis helpers.**
- **Prior iteration: R2 expectations loader** (+29 unit tests
  in `test_expectations_loader.py`).
- **Prior iteration: M6.15 — R2 expectation schema + first-draft
  values** (`tests/integration/expectations/{ur5e, ur15,
  payloads}.yaml` + schema tests; ADR-0013).
- **Prior iteration: blocker-only STATUS refresh** (M6.0 gate).
- **Prior iteration: propose M6 and ADR-0012** (docs-only, raised
  `mujoco_ros2_control` vendoring as an explicit operator gate).
- **M0 bullet `scripts/setup_env.sh`** (three-stage apt → rosdep
  → pip + pre-commit install wrapper; 26 unit tests).
- **M0 bullet `.github/workflows/ci.yml`** (two-job lint → build,
  `ros:humble-ros-base`, `--unit-only`; 12 unit tests).
- **M0 bullet `.pre-commit-config.yaml`** (+ `.clang-format`,
  `ruff.toml`; 12 unit tests).
- **M0 bullet `colcon_defaults.yaml`.**
- **M5 bullets 1–4** (evaluation harness: scenarios schema,
  `run_evaluation.py`, `compute_metrics.py`, `compare.py`).
- **M4 bullets 2–4:** `cartesian_motion_controller` on ur5e and
  ur15 with regulation integration test.

## Build status

- ROS 2 distro: **Humble** (Ubuntu 22.04, matches system install).
- `colcon build` (picking up `colcon_defaults.yaml`) produces
  **10** packages successfully. No build changes this iteration.
- No submodule pointer changes this iteration.

## Test status

- Unit tests: `scripts/run_tests.sh --unit-only` — **1370 passed**
  (up from 1320; +50 R2 run-level aggregator tests).
- Integration tests: re-run this iteration — all **12** launch
  tests green (sim smoke + 3 crisp roles + simple_joint_impedance
  + cartesian_motion, each × {ur5e, ur15}); ~5:01 wall clock.
- `colcon test`: **10** packages pass (22 `test_math` gtests +
  4 `test_pseudo_inverse` + 5 `test_filters` gtests from
  `crisp_controllers`).
- `pre-commit run --files <changed>`: clean (trim trailing
  whitespace, EOL fixer, ruff, ruff-format).


## Blockers / open questions for operator

Active for **M6** (please resolve in order):

- **[M6.0 gate — still active]** Decide how to vendor the
  `mujoco_ros2_control` patch required for claim-aware ctrl
  routing. ADR-0012 §Decisions-to-gate lists three options;
  recommendation is **(a) submodule under
  `third_party/mujoco_ros2_control` tracking an `auto_dev`
  branch**. Operator also to confirm the target
  `ur_robot_driver` version (Humble 2.4.x exposes effort;
  2.3.x does not). ADR-0012 will be amended before M6.1 starts.
- **[M6 R1 — pending]** Confirm the
  `robot_driver:={sim,real}` launch-arg design (M6.11).
- **[M6 R2 — partial]** Tolerance values in
  `tests/integration/expectations/*.yaml` now exist as
  first-principles **drafts** per ADR-0013, including the
  `stage2` block. The schema is frozen; numerical review is the
  open action. Operator is expected to edit values in place
  (`draft: true` flag clears when all values are confirmed).
  Per-joint `effective_inertia_kg_m2` values are the most
  fragile; they will be regenerated from the MJCF `armature` +
  H(q) at home pose during M6.1 and should be treated as
  placeholders until then.
- **[M6 R3 — pending]** Three design knobs to confirm before
  M6.17:
  1. MJCF-reload vs. in-place body mutation for payload
     updates (ADR-0012 §R3.3 — recommend "regenerate + reload"
     for iteration 1).
  2. Cube-size density (default 1000 kg/m³) and colour mapping
     (red = gravity-comp-unaware active; green = payload-aware).
  3. Whether `ur_sim_msgs/EePayload` is a new message
     (recommended) or we re-use
     `geometry_msgs/Inertia + PoseStamped`.
- **[M6 scope clarification — pending]** ADR-0007 becomes a
  runtime invariant under M6. Operator to confirm this is
  acceptable, otherwise ADR-0007 must be formally superseded
  inside ADR-0012 before M6.8.

Active for earlier milestones (unchanged):

- **[Active gate]** M4 bullet 5 (second cartesian mode) — still
  blocked on the `ur_simulator` F/T sensor patch.
- **[Active gate]** M-REAL (real UR15) — explicit operator
  authorisation required per AGENTS.md §5.

Informational:

- ROS distro pinned to Humble via ADR-0002.
- Dashboard at `http://localhost:8000`; rosbridge at
  `ws://localhost:9090`. `scripts/kill_sim.sh` cleans both.
- Gravity-comp hook for `simple_joint_impedance_controller` is
  unused (pure PD holds both arms within M3 tolerance).
- Cartesian metrics remain deferred (ADR-0010); `compare.py`
  surfaces cartesian combos as `not_yet_evaluated`.
- CI does not exercise integration tests — they require a live
  MuJoCo sim and there is no headless story yet.

## Recent commits

Run `git log --oneline -n 20` for the live list.
