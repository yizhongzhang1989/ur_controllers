# Status

_Last updated: 2026-04-24 (docs: M6 gains R1/R2/R3 hard requirements —
sim/real binary parity, three-stage structured test matrix with
theoretical expectations, configurable end-effector payload with
dashboard widget and cube visualisation. ADR-0012 addendum captures
the design; ROADMAP gains M6.10–M6.19. Operator review pending
before `scripts/auto_dev_loop.sh` is fired.)_

## Current milestone

**M6 — Unified MuJoCo sim interfaces (runtime-switchable, real-UR
parity), with R1/R2/R3 hard requirements.** New milestone, now
elaborated with three operator-stated acceptance criteria:

- **R1** — every controller must work on both sim and the real UR
  driver **without any change**. Controller YAMLs, launch files,
  and plugin configs do not branch on sim vs. real. The only diff
  is which hardware-interface node is launched.
- **R2** — tests run a three-stage matrix (single-joint → all
  joints → end-effector), each stage records the theoretical
  expectation alongside the measured result, and fails if the gap
  exceeds the declared tolerance. Parametrised over
  `{ur5e, ur15} × payload`.
- **R3** — end-effector payload is configurable on the dashboard
  (mass, inertia tensor, pose relative to `tool0`), physically
  simulated in MuJoCo, visualised as a wire-frame cube in the
  three.js canvas, and published via a latched `/ee_payload`
  topic + `~/set_ee_payload` service. Tests parametrised over
  `{no_payload, small_payload, large_payload}`.

Design captured in ADR-0012 (including the R1/R2/R3 addendum).

M0–M5 remain fully ticked. M4 bullet 5 (second cartesian mode) and
M-REAL are still deliberately not in scope — see "Blockers" below.

## Next task (agent should pick this up)

**M6.0 — Operator gate.** The agent must stop on this bullet and
surface the vendoring question for `mujoco_ros2_control` before
doing anything else. Per AGENTS.md §7, adding a submodule (or
choosing any of the three alternatives in ADR-0012 §Decisions-to-
gate) is a human gate.

Until the operator resolves M6.0, every subsequent M6 bullet is
blocked:

- M6.1 (MJCF three-actuator emission) can be prototyped on
  `third_party/ur_simulator@auto_dev` but cannot be tested
  end-to-end without the plugin patch from M6.4.
- M6.4 (claim-aware ctrl routing) lives in the
  `mujoco_ros2_control` plugin whose ownership is the topic of
  M6.0.

Because of that chain, this iteration only updates docs:

- `docs/ROADMAP.md`: adds the M6 block and its nine bullets.
- `docs/DECISIONS.md`: appends ADR-0012 (Proposed) with the
  design, the transition rules, and three vendoring options for
  operator selection.
- `docs/STATUS.md`: this rewrite.

No code, config, test, submodule, or launch-file changes this
iteration. The auto-dev loop should therefore see a single commit
touching `docs/**` only, and — if M6.0 is still unresolved on the
next iteration — hit the "two consecutive iterations produce no
commit" stop guard (AGENTS.md §8) once the doc update has landed.
That is the intended behaviour.

## Last completed tasks

- **This iteration: propose M6 and ADR-0012.** `docs/`-only
  change; no code. Raises the `mujoco_ros2_control` vendoring
  question as an explicit operator gate before any M6
  implementation work.
- **Prior iteration: blocker-only STATUS update** surfacing that
  M4 bullet 5 and M-REAL are the only remaining ROADMAP bullets
  and both are human-gated.
- **M0 bullet `scripts/setup_env.sh` (pre-M6).** Three-stage
  wrapper (apt → rosdep → pip + pre-commit install) with
  `--dry-run` and `--no-pre-commit` flags. Pinned by 26 unit tests
  in `tests/unit/test_setup_env.py`. **M0 is fully ticked.**
- **M0 bullet `.github/workflows/ci.yml`.** Two-job (`lint` →
  `build`) workflow, `ros:humble-ros-base`, `--unit-only`. Pinned
  by 12 unit tests in `tests/unit/test_ci_workflow.py`.
- **M0 bullet `.pre-commit-config.yaml`** (+ `.clang-format`,
  `ruff.toml`). Hooks pinned; `third_party/`, `build/`, `install/`,
  `log/` excluded. 12 unit tests in
  `tests/unit/test_pre_commit_config.py`.
- **M0 bullet `colcon_defaults.yaml`.** Pinned by
  `tests/unit/test_colcon_defaults.py`.
- **M5 bullets 1–4** (evaluation harness: scenarios schema,
  `run_evaluation.py`, `compute_metrics.py`, `compare.py`). 116
  unit tests across the four bullets.
- **M4 bullets 2 + 3 + 4:** `cartesian_motion_controller` on ur5e
  and ur15 with regulation integration test
  (`tests/integration/test_cartesian_motion.py`).

## Build status

- ROS 2 distro: **Humble** (Ubuntu 22.04, matches system install).
- `colcon build` (from the repo root, picking up
  `colcon_defaults.yaml`) produces **10** packages successfully,
  unchanged from prior iteration.
- No build changes this iteration (docs-only).

## Test status

- No test changes this iteration. Last green end-to-end run of
  `scripts/run_tests.sh` (pre-M6): **172** pytest unit tests +
  **22** `test_math` gtests (simple_joint_impedance_controller) +
  **5** `crisp_controllers` gtests + **12** integration tests
  (sim smoke + 3 crisp roles + our simple_joint_impedance_controller
  + `cartesian_motion_controller`, each ×{ur5e, ur15}).
- `pre-commit run --all-files` was clean at that point; this
  iteration only edits `docs/*.md` files, which pre-commit covers
  via trailing-whitespace / EOL hooks — must be re-run before
  commit.

## Blockers / open questions for operator

Active for **M6** (please resolve in order):

- **[M6.0 gate — new]** Decide how to vendor the
  `mujoco_ros2_control` patch required for claim-aware ctrl
  routing. ADR-0012 §Decisions-to-gate lists three options;
  recommendation is **(a) submodule under
  `third_party/mujoco_ros2_control` tracking an `auto_dev`
  branch**, matching ADR-0003's ownership discipline. Operator
  also to confirm the target `ur_robot_driver` version for
  interface parity (Humble 2.4.x exposes effort; 2.3.x does
  not) — ADR-0012 will be amended with both answers before
  M6.1 starts.
- **[M6 R1 — new]** Confirm the `robot_driver:={sim,real}` launch
  arg design (M6.11). The operator's stated requirement is "no
  change to any controller" between sim and real; the proposed
  realisation keeps all `bringup/config/*.yaml` identical and
  only swaps which hardware-interface node is included. Agent
  needs confirmation that this delegation pattern is acceptable
  (as opposed to, e.g., a single launch file that auto-detects).
- **[M6 R2 — new]** Confirm tolerance values in
  `tests/integration/expectations/*.yaml` (M6.15). The agent will
  author first drafts from first principles (±20% ζ, ≤ 1e-3 rad
  drift, 6 dB FFT peak threshold), but the operator may want
  stricter values per arm.
- **[M6 R3 — new]** Three design knobs to confirm before M6.17:
  1. MJCF-reload vs. in-place body mutation for payload updates
     (ADR-0012 addendum §R3.3 — recommend "regenerate + reload"
     for iteration 1).
  2. Cube-size density (default 1000 kg/m³) and colour mapping
     (red = gravity-comp-unaware active; green = payload-aware).
  3. Whether `ur_sim_msgs/EePayload` is a new message (recommended)
     or we re-use an existing `geometry_msgs` + `Inertia` pair.
- **[M6 scope clarification — new]** ADR-0007 ("crisp controllers
  own the effort interfaces exclusively") is currently a
  launch-time contract. Under M6 it becomes a runtime invariant
  (achieved by `switch_controllers --strict` on crisp role
  activation, which the existing bringup already does). Operator
  to confirm this is acceptable — otherwise ADR-0007 must be
  formally superseded inside ADR-0012 before M6.8.

Active for earlier milestones (unchanged):

- **[Active gate]** M4 bullet 5 (second cartesian mode) is still
  blocked on the `ur_simulator` F/T sensor patch. Orthogonal to
  M6; mentioned here only so the auto-dev loop does not silently
  retire it.
- **[Active gate]** M-REAL (real UR15) — explicit operator
  authorisation required per AGENTS.md §5. Still not authorised.

Informational:

- ROS distro pinned to Humble via ADR-0002.
- Dashboard at `http://localhost:8000`; rosbridge at
  `ws://localhost:9090`. `scripts/kill_sim.sh` cleans both.
- Gravity-comp hook for `simple_joint_impedance_controller` is
  unused (pure PD holds both arms within M3 tolerance).
- Cartesian metrics remain deferred (ADR-0010); `compare.py`
  surfaces cartesian combos as `not_yet_evaluated`.
- CI does not exercise integration tests — they require a live
  MuJoCo sim and there is no headless story yet. M6 does not
  change that.

## Recent commits

Run `git log --oneline -n 20` for the live list.
