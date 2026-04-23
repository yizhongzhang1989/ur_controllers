# Status

_Last updated: 2026-04-23 (build(m0): `scripts/setup_env.sh` —
idempotent three-stage wrapper (apt → rosdep → pip + pre-commit
install) with `--dry-run` and `--no-pre-commit` flags. Pinned by
`tests/unit/test_setup_env.py` (26 cases). **M0 is now fully
ticked.**)_

## Current milestone

**M0 — bootstrap: COMPLETE.** All 7 M0 bullets are ticked. With M2–M5
already done (cartesian live-dispatch and FK metrics remain as
voluntary follow-ups, not ROADMAP bullets), the only remaining
ROADMAP work is **human-gated**:

- **M4 bullet 5** — second cartesian mode. Still gated on the sim
  F/T sensor patch on `third_party/ur_simulator`'s `auto_dev`
  branch. Operator must direct this work before the agent may enter
  it (AGENTS.md §7).
- **M-REAL** — real UR15 bringup. Explicitly out of scope per
  AGENTS.md §5 and the ROADMAP (placeholder only).

This iteration tackled the final M0 bullet, `scripts/setup_env.sh`.
Approach:

1. **Three-stage, idempotent script.**
   1. **Stage 1 — `apt install`.** Installs the README "Prerequisites"
      packages (`ros-humble-ur-description`, `ros-humble-ros2-control`,
      `ros-humble-ros2-controllers`, `ros-humble-mujoco-ros2-control`,
      `ros-humble-pinocchio`, `ros-humble-rosbridge-suite`,
      `ros-humble-ros-gz`, `ros-humble-gz-ros2-control`,
      `ros-humble-xacro`) plus the build/test tooling the CI workflow
      installs explicitly (`python3-colcon-common-extensions`,
      `python3-colcon-defaults`, `python3-rosdep`, `python3-vcstool`,
      `python3-pytest`, `python3-yaml`, `git`, `build-essential`,
      `python3-pip`). `apt-get install` is idempotent so re-runs are
      cheap.
   2. **Stage 2 — `rosdep`.** Guards `rosdep init` behind an
      existence check of `/etc/ros/rosdep/sources.list.d/20-default.list`
      (the ROS apt package pre-creates it, and a second `init` errors
      out), then `rosdep update` + `rosdep install --from-paths src
      third_party --ignore-src -r -y --skip-keys "cartesian_controller_simulation
      cartesian_controller_tests"`. Skip keys match ADR-0005 and the
      CI workflow so all three paths agree on what is out of scope.
   3. **Stage 3 — `pip install --user --upgrade pre-commit ruff`**
      followed by `pre-commit install` to wire the local git hook.
      Stage 3 is skippable via `--no-pre-commit` for environments
      that manage those tools out-of-band (e.g. CI, which already
      installs them in the `lint` job).
2. **Flags mirror `scripts/run_tests.sh`.** `--dry-run` prefixes every
   actionable command with `DRY:` and exits 0, so the unit gate (and
   any operator on an unprovisioned box) can exercise the full code
   path without side effects. `--help` prints the header block via the
   same `sed -n '2,19p' "$0"` trick `run_tests.sh` uses.
3. **`sudo` opportunistic.** `$SUDO` is set to `sudo` only when
   `$EUID != 0` *and* `command -v sudo` succeeds. That covers:
   - Dev-box contributor (non-root, sudo present) → uses `sudo`.
   - ROS CI container (root, no sudo) → runs bare.
   Both are exercised in the wild; the CI workflow already runs as
   root inside `ros:humble-ros-base`, so a script that only worked
   with sudo would be a silent footgun.
4. **Unit test `tests/unit/test_setup_env.py` (26 cases)** pins: the
   shebang, the executable bit, the `--help` / `--dry-run` /
   `--no-pre-commit` / unknown-arg behaviour (subprocess-invoked so
   exit codes are verified), the presence of every README
   prerequisite apt package, the ADR-0005 skip-keys, the
   `rosdep install --from-paths src third_party` invocation with
   `--ignore-src -r -y`, the `rosdep init` guard, the pip install
   line, the `pre-commit install` wiring, and the opportunistic-sudo
   pattern. Same style as `tests/unit/test_pre_commit_config.py` and
   `tests/unit/test_ci_workflow.py` — no network, no apt, no pip, no
   ROS.

Key non-obvious points:

- **`--dry-run` actually exits 0.** The unit test invokes the script
  with `--dry-run` and asserts no `Reading package lists` leaked into
  stdout — a cheap guard against a future refactor that accidentally
  strips the dry-run wrapper from one of the stages.
- **`rosdep init` is guarded, not suppressed.** On a genuinely fresh
  box where nothing ever ran rosdep before, the guard's `if` falls
  through to the real `rosdep init` call. Only on boxes where the
  ROS apt package (or a prior agent run) already initialised rosdep
  does the guard skip it. Matches the same pattern the CI workflow
  uses.
- **`pre-commit install` is in-tree, not a global git config.** It
  writes `.git/hooks/pre-commit` in the current repo; running it
  from another clone of the same repo is harmless. Skippable via
  `--no-pre-commit` for headless environments that don't want the
  hook.
- **We install `ros-humble-xacro` explicitly** even though `xacro` is
  often pulled in transitively. The sim launch shells out to
  `xacro` directly and the README lists it under Prerequisites, so
  making the dependency explicit here removes a class of
  fresh-checkout failures.

## Last completed tasks

- **M0 bullet `scripts/setup_env.sh`.** Three-stage wrapper
  (apt → rosdep → pip + pre-commit install) with `--dry-run` and
  `--no-pre-commit` flags. Pinned by 26 unit tests in
  `tests/unit/test_setup_env.py`. **M0 is now fully ticked.**
- **M0 bullet "CI workflow `.github/workflows/ci.yml`".** See prior
  STATUS.
- **M0 bullet `.pre-commit-config.yaml`.** See prior STATUS.
- **M0 bullet top-level colcon workspace config.** See prior STATUS.
- **M5 bullet 4: `evaluation/compare.py`.** See prior STATUS.
- **M5 bullet 3: `evaluation/compute_metrics.py`.** See prior STATUS.
- **M5 bullet 2: `evaluation/run_evaluation.py`.** See prior STATUS.
- **M5 bullet 1: `evaluation/scenarios/*.yaml` schema v1.** See prior STATUS.
- **M4 bullets 2 + 3 + 4: `cartesian_motion_controller` brought up on
  ur5e and ur15 with regulation integration test.** See prior STATUS.

## Next task (agent should pick this up)

**There is no next mandatory ROADMAP task.** M0 is complete, and
M1–M5 were already complete coming into this iteration. The
remaining ROADMAP entries are human-gated:

- **M4 bullet 5** — second cartesian mode (needs the sim F/T sensor
  patch; operator-gated).
- **M-REAL** — real UR15 bringup (explicitly out of scope until the
  operator authorises).

Per the outer-loop stop conditions (AGENTS.md §8), the loop now
terminates on "all ROADMAP milestones are marked `[x]`" modulo the
human-gated bullets. The agent must stop and surface this to the
operator rather than speculatively entering M4-5 or M-REAL.

Suggested follow-ups (**not** ROADMAP bullets, do not pick these up
without operator sign-off):

- A second CI job (or a separate workflow) that runs the integration
  tests behind an `xvfb` / headless-MuJoCo wrapper, once that story
  exists. The current workflow deliberately stops at unit tests.
- Live-dispatch mode for `compare.py` (subprocess
  `run_evaluation.py` per compatible combo) so the comparison is
  one command end-to-end once an operator is happy to pay the
  per-combo ~1 min sim cost.
- FK-backed cartesian metrics (ADR-0010) — would flip the
  `not_yet_evaluated` rows into real numbers.
- During this iteration's full-gate run,
  `test_crisp_joint_impedance_regulation[ur15]` flaked on a
  `load_controller` RMW-response drop (same symptom as ADR-0006).
  A targeted re-run was green, and a subsequent full-gate run was
  green end-to-end. If this recurs, consider either bumping the
  spawner's `--service-call-timeout` further on the sim's
  `auto_dev` branch or adding a bounded retry in the integration
  fixture (both would be sim-side/test-side tweaks, not a new
  ROADMAP bullet).

## Build status

- ROS 2 distro: **Humble** (Ubuntu 22.04, matches system install).
- `colcon build` (from the repo root, picking up
  `colcon_defaults.yaml`) produces **10** packages successfully,
  unchanged from prior iteration.

## Test status

- `scripts/run_tests.sh` runs unit → colcon test → integration. Green
  end-to-end this iteration (146 + **26 new setup_env** =
  **172** pytest unit tests; 27 colcon gtests; 12 integration tests).
- Test counts: **172** pytest unit tests (35 schema + 34
  runner-dry-run + 32 metrics + 15 compare + 6 colcon-defaults + 12
  pre-commit-config + 12 ci-workflow + **26 new setup_env**) +
  **22** `test_math` gtests (simple_joint_impedance_controller) +
  **5** `crisp_controllers` gtests + **12** integration tests
  (sim smoke + 3 crisp roles + our simple_joint_impedance_controller
  + `cartesian_motion_controller`, each ×{ur5e, ur15}).
- `pre-commit run --all-files` exits 0 against the current tree
  (verified this iteration on the new files via `--files`).

## Blockers / open questions for operator

None currently blocking. Informational:

- ROS distro pinned to Humble (system install); formalised via
  ADR-0002. The CI workflow targets the same distro via the
  `ros:humble-ros-base` container image, and `scripts/setup_env.sh`
  hard-codes the `ros-humble-*` apt package names.
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
- Intermittent flake: `test_crisp_joint_impedance_regulation[ur15]`
  can trip the ADR-0006 `load_controller` RMW-response race even
  with spawners serialised. A retry cleared it this iteration, but
  a deterministic fix would live on the sim's `auto_dev` branch
  (spawner retry with idempotent load semantics, or a further
  `--service-call-timeout` bump). Flag only — not currently
  blocking.
- CI does not yet exercise integration tests — they require a live
  MuJoCo sim that we have no headless story for. The workflow stops
  at `--unit-only` deliberately.

## Recent commits

Run `git log --oneline -n 20` for the live list.
