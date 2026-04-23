# Status

_Last updated: 2026-04-23 (ci(m0): `.github/workflows/ci.yml` two-job
pipeline (`lint` → `build`) wiring the pre-commit gate + colcon build +
unit-test gate. Pinned by `tests/unit/test_ci_workflow.py` (12 cases).
Full local test gate green in ~7:50.)_

## Current milestone

**M0 — bootstrap: 1 unchecked item remaining** (`scripts/setup_env.sh`).
M0 bullet "CI workflow" closed this iteration. Per ROADMAP rule "agent
works on the earliest milestone that has unchecked items", M0 keeps
priority over M2–M5 follow-ups.

M2–M4 done (with M4 bullet 5 still parked behind a human gate for the
sim F/T sensor patch). M5 bullets 1–4 are all ticked. M-REAL is
explicitly out-of-scope until operator approval.

This iteration tackled M0 bullet "CI workflow `.github/workflows/ci.yml`".
Approach:

1. **Two-job split.** `lint` runs first on `ubuntu-22.04` with no ROS:
   `actions/setup-python@v5` (3.10), `pip install pre-commit ruff`,
   then `pre-commit run --all-files --show-diff-on-failure`. A
   `actions/cache@v4` step keyed on `.pre-commit-config.yaml` keeps
   hook installs warm between runs. `build` then runs inside the
   `ros:humble-ros-base` container (matches ADR-0002 — Humble on
   22.04), depends on `lint` via `needs:`, and is the only job that
   pays the rosdep + colcon-build cost.
2. **Build job pipeline.** `actions/checkout@v4` with
   `submodules: recursive`, then `apt-get install` the colcon /
   rosdep / pytest tooling, then `rosdep install --from-paths src
   third_party` with the ADR-0005 `--skip-keys` (matching the
   `colcon_defaults.yaml` `--packages-skip` list — otherwise rosdep
   would try to resolve the MuJoCo-and-catkin deps of
   `cartesian_controller_simulation` /
   `cartesian_controller_tests`). Then `colcon build` (defaults file
   auto-loaded from cwd) and `scripts/run_tests.sh --unit-only`.
3. **Why `--unit-only` and not the full test gate.** `scripts/run_tests.sh`
   stage 3 is `tests/integration/`, all of which spawn `launch_sim.sh`
   to drive a live MuJoCo sim. That requires GPU/audio/etc. inside the
   container and ~7 minutes of runtime per matrix entry; it is not
   suitable for the headless CI gate. Once a sim-in-CI story lands
   (e.g. xvfb + headless MuJoCo) the integration tests can be added in
   a follow-up workflow without touching this one.
4. **Triggers.** `push` and `pull_request` on `auto_dev` and `main`,
   plus `workflow_dispatch` so an operator can re-run on demand. The
   `.github/workflows/ci.yml` file itself is matched by the path filters
   so a workflow edit is exercised on its own PR.
5. **Unit test `tests/unit/test_ci_workflow.py` (12 cases)**: pins the
   workflow's name + triggers, the existence of both jobs, the
   container image targeting `humble`, the submodule-checkout flag, the
   `pre-commit run --all-files` invocation, the `colcon build` step,
   the `scripts/run_tests.sh --unit-only` step, the `rosdep install`
   step with the ADR-0005 skip-keys, the `needs: lint` dependency, and
   `runs-on: ubuntu-22.04` for both jobs. Same testing pattern as
   `tests/unit/test_pre_commit_config.py` from the previous iteration:
   load the YAML and assert structure, do not invoke `act` or any
   GitHub-hosted runner.

Key non-obvious points:

- **PyYAML parses the bare `on:` key as Python `True`.** The unit test
  guards `workflow.get("on") if "on" in workflow else workflow.get(True)`
  so the assertion stays robust against `safe_load`'s YAML 1.1
  treatment of `on` as a boolean. (PyYAML 6.x still does this; YAML 1.2
  loaders would not, but PyYAML is what the rest of the unit gate
  uses, see the `python deps` repo memory.)
- **`needs: lint` is intentional.** A red `pre-commit` should short-
  circuit the (much more expensive) build job. The override is the
  `workflow_dispatch` trigger, which still respects `needs:` ordering
  — operators can't bypass the lint gate from the GH UI without
  editing the workflow.
- **Container image is `ros:humble-ros-base`, not `ros:humble`.**
  `ros-base` excludes the desktop/GUI stack (rviz, gz, etc.), which
  saves ~1 GB of image pull and is irrelevant for headless build +
  unit tests. Anything controller-side that the unit tests need is
  already pulled in by `ros_core` + the rosdep step.
- **`rosdep init` is conditional.** The `ros:humble-ros-base` image
  ships with rosdep already initialised; calling `rosdep init` again
  errors out. The `if [ ! -f ... ]` guard keeps the step idempotent
  in case the base image ever changes.
- **The workflow does not push or release anything.** It is a pure
  read-only gate. `permissions: contents: read` documents that.

## Last completed tasks

- **M0 bullet "CI workflow `.github/workflows/ci.yml`".** Two-job
  pipeline (`lint` → `build`) wiring pre-commit, rosdep, colcon
  build, and `scripts/run_tests.sh --unit-only`. Pinned by 12 unit
  tests in `tests/unit/test_ci_workflow.py`. ROADMAP M0 bullet ticked.
- **M0 bullet `.pre-commit-config.yaml`.** See prior STATUS.
- **M0 bullet top-level colcon workspace config.** See prior STATUS.
- **M5 bullet 4: `evaluation/compare.py`.** See prior STATUS.
- **M5 bullet 3: `evaluation/compute_metrics.py`.** See prior STATUS.
- **M5 bullet 2: `evaluation/run_evaluation.py`.** See prior STATUS.
- **M5 bullet 1: `evaluation/scenarios/*.yaml` schema v1.** See prior STATUS.
- **M4 bullets 2 + 3 + 4: `cartesian_motion_controller` brought up on
  ur5e and ur15 with regulation integration test.** See prior STATUS.
- **M3 — sim bring-up + regulation integration test for
  `simple_joint_impedance_controller` on `{ur5e, ur15}`.** See prior
  STATUS.

## Next task (agent should pick this up)

**M0 has 1 unchecked bullet remaining**:

1. **`scripts/setup_env.sh`**: thin wrapper around `apt`/`rosdep` to
   install the prerequisites listed in README.md "Prerequisites".
   Idempotent; safe to re-run. Add a `--dry-run` mode mirroring
   `scripts/run_tests.sh`. Should also `pip install --user pre-commit
   ruff` so a fresh checkout can run the new pre-commit hooks without
   hunting for pip incantations. Optional: `pre-commit install` so the
   local git-hook is wired automatically.

After M0 is fully ticked, the only remaining roadmap items are
**human-gated**:

- **M4 bullet 5** — second cartesian mode. Still gated on the sim
  F/T sensor patch on `third_party/ur_simulator`'s `auto_dev`
  branch. Operator must direct this work before the agent may enter
  it (see AGENTS.md §7).
- **M-REAL** — real UR15 bringup. Explicitly out of scope per
  AGENTS.md §5 and ROADMAP (placeholder only).

Suggested follow-ups (not blocking the outer loop but useful):

- A second CI job (or a separate workflow) that runs the integration
  tests behind an `xvfb` / headless-MuJoCo wrapper, once that story
  exists. The current workflow deliberately stops at unit tests.
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

## Build status

- ROS 2 distro: **Humble** (Ubuntu 22.04, matches system install).
- `colcon build` (from the repo root, picking up
  `colcon_defaults.yaml`) produces **10** packages successfully,
  unchanged from prior iteration.

## Test status

- `scripts/run_tests.sh` runs unit → colcon test → integration. Green
  in ~7:50 this iteration (35 schema + 34 runner-dry-run + 32
  metrics + 15 compare + 6 colcon-defaults + 12 pre-commit-config +
  **12 new ci-workflow** unit tests = 146 pytest unit tests; 27
  colcon gtests; 12 integration tests).
- Test counts: **146** pytest unit tests + **22** `test_math` gtests
  (simple_joint_impedance_controller) + **5** `crisp_controllers`
  gtests + **12** integration tests (sim smoke + 3 crisp roles + our
  simple_joint_impedance_controller + `cartesian_motion_controller`,
  each ×{ur5e, ur15}).
- `pre-commit run --all-files` exits 0 against the current tree
  (verified this iteration on the new files via `--files`).

## Blockers / open questions for operator

None currently blocking. Informational:

- ROS distro pinned to Humble (system install); formalised via
  ADR-0002. The new CI workflow targets the same distro via the
  `ros:humble-ros-base` container image.
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
  yet installed by any setup script. The CI workflow now installs
  them in the `lint` job, but local contributors must still
  `pip install --user pre-commit ruff` until the M0 `setup_env.sh`
  bullet lands.
- CI does not yet exercise integration tests — they require a live
  MuJoCo sim that we have no headless story for. The workflow stops
  at `--unit-only` deliberately; a follow-up can add an integration
  job once headless sim is sorted.

## Recent commits

Run `git log --oneline -n 20` for the live list.
