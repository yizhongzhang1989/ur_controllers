# Status

_Last updated: 2026-04-23 (build(m0): `.pre-commit-config.yaml` (clang-format,
ruff, trailing whitespace) + `.clang-format` (LLVM base) + `ruff.toml`
(py310 target). Tree brought into compliance in the same commit; 21 source
files reformatted by ruff/clang-format. New unit test
`tests/unit/test_pre_commit_config.py` (12 cases) pins the structure. Full
test gate green in ~8:00.)_

## Current milestone

**M0 — bootstrap: 2 unchecked items remaining** (CI workflow,
`scripts/setup_env.sh`). M0 bullet "add `.pre-commit-config.yaml`" closed
this iteration. Per ROADMAP rule "agent works on the earliest milestone
that has unchecked items", M0 keeps priority over M2–M5 follow-ups.

M2–M4 done (with M4 bullet 5 still parked behind a human gate for the
sim F/T sensor patch). M5 bullets 1–4 are all ticked. M-REAL is
explicitly out-of-scope until operator approval.

This iteration tackled M0 bullet 2 (`.pre-commit-config.yaml`). Approach:

1. **`.pre-commit-config.yaml`** at repo root with three hook
   families per AGENTS.md §4:
   - `pre-commit/pre-commit-hooks` v5.0.0 — `trailing-whitespace`,
     `end-of-file-fixer`, `check-yaml` (with `--allow-multiple-documents
     --unsafe` so ROS launch/param YAML with custom tags doesn't false-
     positive), `check-merge-conflict`, `check-added-large-files`
     (`--maxkb=1024`).
   - `astral-sh/ruff-pre-commit` v0.6.9 — `ruff --fix` then
     `ruff-format`. Configured by repo-root `ruff.toml` (line-length
     100, target-version py310, rule families `E,F,W,I`, `E501`
     ignored because `ruff format` already enforces line length).
   - `pre-commit/mirrors-clang-format` v18.1.8 — runs against C++/C
     files. Configured by repo-root `.clang-format` (BasedOnStyle:
     LLVM, ColumnLimit 100, IndentWidth 4, PointerAlignment Left,
     SortIncludes off — touching `#include` order would churn every
     file we own without a corresponding test win).
2. **Excludes** the read-only submodules under `third_party/`
   (ADR-0003), all colcon build artefacts (`build/`, `install/`,
   `log/`, `.auto_dev_logs/`), and the gitignored evaluation outputs
   (`evaluation/runs/`, `evaluation/reports/`, baseline `*.bag/` and
   `*.mcap/` payloads). Encoded as a verbose-regex `exclude:` so the
   rule lives in one place and is structurally testable.
3. **Brought the tree into compliance in the same commit.** First
   `pre-commit run --all-files` reformatted 21 files (4 ruff lint
   auto-fixes + 18 ruff-format reformats + every C++ file
   re-flowed by clang-format). Re-ran until all hooks pass; verified
   `colcon build` green and `scripts/run_tests.sh` green (122 unit
   + 12 integration + 30 colcon gtests, ~8 min total). The
   reformat-and-introduce-config split would have left the repo in
   a "config but tree is dirty" state for one commit which is not
   useful — single commit is one logical change.
4. **Unit test `tests/unit/test_pre_commit_config.py` (12 cases)**:
   pins repos block present; required hooks (`trailing-whitespace`,
   `ruff`, `ruff-format`, `clang-format`) all configured; exclude
   pattern mentions `third_party/`, `build/`, `install/`, `log/`;
   `.clang-format` exists with `BasedOnStyle: LLVM`; `ruff.toml`
   exists with `target-version = "py310"`; every external repo has
   a non-empty `rev`. The test does not invoke `pre-commit` itself
   — that requires network for hook installs and is too slow for
   the unit gate. The CI workflow (next M0 bullet) is the right
   place to actually exercise `pre-commit run --all-files`.

Key non-obvious points:

- **Tools are not on the host PATH by default.** `pre-commit`, `ruff`
  and `clang-format` are not installed system-wide on the dev box;
  this iteration installed `pre-commit` + `ruff` to `~/.local/bin`
  (`pip install --user`). `clang-format` is supplied by the
  `pre-commit/mirrors-clang-format` hook via its own pinned binary,
  so the host does not need an apt-installed `clang-format`. The
  upcoming `scripts/setup_env.sh` (M0 bullet 3) should `pip install
  pre-commit ruff` to make the workflow reproducible.
- **`SortIncludes: false`** in `.clang-format` is deliberate. The
  default LLVM behaviour reorders `#include` blocks; flipping that on
  in a code base where include order sometimes carries semantic
  weight (e.g. ros2_control plugin macros) would force a churny
  manual review with no testable benefit. Re-enable in a follow-up
  iteration if it ever becomes a problem.
- **Ruff's `E501` is intentionally ignored.** `ruff format` already
  enforces line length (100); having the linter also flag E501 just
  produces noise on lines `format` chose not to break (e.g. long
  string literals). This matches the upstream `astral-sh` guidance.
- **`check-yaml` runs with `--unsafe --allow-multiple-documents`.**
  ROS launch and param YAMLs occasionally use custom tags or
  multi-document streams; the safe default would false-positive
  on legitimate files. The hook still catches malformed YAML.

## Last completed tasks

- **M0 bullet 2: `.pre-commit-config.yaml`.** Three hook families
  (clang-format, ruff, trailing-whitespace + friends) with pinned
  revs, repo-root `.clang-format` (LLVM base) and `ruff.toml` (py310,
  rules `E,F,W,I`). Tree brought into compliance in the same commit
  (21 files reformatted). New unit test
  `tests/unit/test_pre_commit_config.py` (12 cases) pins structure.
  ROADMAP M0 bullet ticked.
- **M0 bullet 1: top-level colcon workspace config.** See prior STATUS.
- **M5 bullet 4: `evaluation/compare.py`.** See prior STATUS.
- **M5 bullet 3: `evaluation/compute_metrics.py`.** See prior STATUS.
- **M5 bullet 2: `evaluation/run_evaluation.py`.** See prior STATUS.
- **M5 bullet 1: `evaluation/scenarios/*.yaml` schema v1.** See prior STATUS.
- **M4 bullets 2 + 3 + 4: `cartesian_motion_controller` brought up on
  ur5e and ur15 with regulation integration test.** See prior STATUS.
- **M4 kick-off: `docs/cartesian_controllers.md` reference + tick M4
  bullet 1 (build).** See prior STATUS.
- **M3 — sim bring-up + regulation integration test for
  `simple_joint_impedance_controller` on `{ur5e, ur15}`.** See prior
  STATUS.

## Next task (agent should pick this up)

**M0 has 2 unchecked bullets remaining**, in order:

1. **CI workflow `.github/workflows/ci.yml`**: build + unit tests
   headless. Should source ROS Humble, install `pre-commit` + `ruff`
   (`pip install pre-commit ruff`), run `pre-commit run --all-files`
   as the first gate, then `colcon build` (defaults file supplies
   the flags), then `scripts/run_tests.sh --unit-only` (integration
   tests need a running sim — defer until a sim-in-CI story lands).
   The workflow is the natural place to *enforce* the pre-commit
   contract added this iteration.
2. **`scripts/setup_env.sh`**: thin wrapper around `apt`/`rosdep` to
   install the prerequisites listed in README.md "Prerequisites".
   Idempotent; safe to re-run. Add a `--dry-run` mode mirroring
   `scripts/run_tests.sh`. Should also `pip install --user pre-commit
   ruff` so a fresh checkout can run the new pre-commit hooks without
   hunting for pip incantations.

After M0 is fully ticked, the only remaining roadmap items are
**human-gated**:

- **M4 bullet 5** — second cartesian mode. Still gated on the sim
  F/T sensor patch on `third_party/ur_simulator`'s `auto_dev`
  branch. Operator must direct this work before the agent may enter
  it (see AGENTS.md §7).
- **M-REAL** — real UR15 bringup. Explicitly out of scope per
  AGENTS.md §5 and ROADMAP (placeholder only).

Suggested follow-ups (not blocking the outer loop but useful):

- Live-dispatch mode for `compare.py` (subprocess
  `run_evaluation.py` per compatible combo) so the comparison is
  one command end-to-end once an operator is happy to pay the
  per-combo ~1 min sim cost. Out-of-scope for the test gate, so
  should land behind a `--live` flag with an explicit smoke test
  on exactly one combo.
- FK-backed cartesian metrics (ADR-0010) — would flip the
  `not_yet_evaluated` rows into real numbers. Needs either
  `pinocchio` (already pulled in by crisp) or `tf2_ros` transform
  lookups during the run.
- Wire `pre-commit install` into `scripts/setup_env.sh` once it
  exists, so contributors get the local git-hook automatically.

## Build status

- ROS 2 distro: **Humble** (Ubuntu 22.04, matches system install).
- `colcon build` (from the repo root, picking up
  `colcon_defaults.yaml`) produces **10** packages successfully,
  unchanged from prior iteration. `simple_joint_impedance_controller`
  rebuilt cleanly after clang-format reflowed every C++ file in the
  package.

## Test status

- `scripts/run_tests.sh` runs unit → colcon test → integration. Green
  in ~8:00 this iteration (35 schema + 34 runner-dry-run + 32
  metrics + 15 compare + 6 colcon-defaults + **12 new
  pre-commit-config** unit tests = 134 pytest unit tests; 27 colcon
  gtests; 12 integration tests).
- Test counts: **134** pytest unit tests + **22** `test_math` gtests
  (simple_joint_impedance_controller) + **5** `crisp_controllers`
  gtests + **12** integration tests (sim smoke + 3 crisp roles + our
  simple_joint_impedance_controller + `cartesian_motion_controller`,
  each ×{ur5e, ur15}).
- `pre-commit run --all-files` exits 0 against the current tree
  (verified this iteration). Hook installs are network-dependent on
  first run (~1 min on a warm cache); the unit-test gate
  deliberately does not invoke `pre-commit`.

## Blockers / open questions for operator

None currently blocking. Informational:

- ROS distro pinned to Humble (system install); formalise via ADR
  if/when a second distro becomes a candidate.
- Dashboard opens at `http://localhost:8000`; rosbridge at
  `ws://localhost:9090`. Both are pkill'd + port-cleared by
  `scripts/kill_sim.sh`.
- The cartesian compliance and force controllers both need an
  `ft_sensor_ref_link` on the URDF chain plus an `~/ft_sensor_wrench`
  publisher. The UR sim does not currently expose an F/T sensor
  frame; those controllers therefore require a sim-side `auto_dev`
  patch before they can be brought up end-to-end. Still gates M4
  bullet 5; unchanged from prior iteration.
- Gravity-comp hook for `simple_joint_impedance_controller` remains
  unused (pure PD holds both arms within the M3 tolerance).
- Cartesian metrics remain deferred: computing TCP-pose RMSE needs
  FK, which is not wired in v1 (see ADR-0010). `compare.py` now
  surfaces cartesian combos as `not_yet_evaluated` so the gap is
  visible without pretending it's passing.
- `compare.py --aggregate-only` is the only mode implemented; live
  matrix dispatch is deferred behind a future `--live` flag.
- `pre-commit`, `ruff`, and the hook-bundled `clang-format` are not
  yet installed by any setup script. Until M0 bullet `setup_env.sh`
  lands, contributors must `pip install --user pre-commit ruff`
  manually before `pre-commit run --all-files` will work.

## Recent commits

Run `git log --oneline -n 20` for the live list.
