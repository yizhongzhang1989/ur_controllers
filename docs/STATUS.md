# Status

_Last updated: 2026-04-24 (stage-2 evaluator gains an optional terminal
settle window — `completion_tol_rad` now samples a trailing mean when
`settle_window_s > 0`. M6.0 operator gate still active for every bullet
that requires live sim changes)._

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
joint-space harness (now with settle-window support), per-arm
stage-2 tolerance block wired through the loader, M6.12 (R2
stage-1) and the joint-space path of M6.13 (R2 stage-2) can both
be authored end-to-end as sim-collection orchestrators — they
just can't run until M6.5 ships. The orchestrator should pass
its probe's settle-window length (from the scenario definition)
directly as `settle_window_s` into
`evaluate_all_joints_joint_space`; the evaluator averages
`|q(t) - q_cmd(t)|` over the trailing window and uses that mean
as `final_err_rad`, so a tighter `completion_tol_rad` than
`peak_tracking_err_rad` is meaningful. Still missing on the
pre-bake chain:

1. M6.13 cartesian-mode / FK-based kinematic-consistency branch
   (needs a UR FK module; source and licensing to be agreed).
2. M6.14 (R2 stage-3, TCP trajectory): also blocked on FK.
3. M6.19 payload parametrisation across R2 stages (needs M6.16–
   M6.18 to land first).

## Last completed tasks

- **This iteration: stage-2 evaluator terminal settle window.**
  Extended `tests/integration/r2_stage2_assertions.py`
  `evaluate_all_joints_joint_space` with a `settle_window_s: float
  = 0.0` kwarg. When positive, the motion-completion check
  averages `|q(t) - q_cmd(t)|` over every sample whose timestamp
  is within `settle_window_s` of `times[-1]` (at minimum the
  final sample) and reports the mean as `final_err_rad`; a
  `settle_window_samples` diagnostic metric is exposed so
  orchestrator artefacts can log the effective window size on
  non-uniform sampling. Default `0.0` preserves existing
  single-sample behaviour — the seven pre-existing stage-2 tests
  pass unchanged. Validates `settle_window_s >= 0` and
  `settle_window_s <= times[-1] - times[0]` at entry; both raise
  `ValueError` with the same strictness as the rest of the
  harness. Pinned by +7 unit tests in
  `tests/unit/test_r2_stage2_assertions.py` covering: default =
  final-sample behaviour, trailing-mean damps a final-sample
  spike, persistent trailing error survives the mean, window
  smaller than `dt` degenerates to the single final sample,
  negative window rejected, over-span window rejected,
  window == span covers every sample. Unit gate now reports
  **278 passed** (up from 271).
- **Prior iteration: R2 stage-2 tolerance block in the expectation
  YAMLs.** Added `stage2:` block with `completion_tol_rad=0.05`,
  `peak_tracking_err_rad=0.15`, `saturation_hold_ms=100.0` to
  both `tests/integration/expectations/ur5e.yaml` and `ur15.yaml`
  and exposed it via the loader as `ArmExpectation.stage2`.
- **Prior iteration: R2 stage-2 assertion harness (joint-space).**
  `tests/integration/r2_stage2_assertions.py` +
  `tests/unit/test_r2_stage2_assertions.py` — per-joint motion-
  completion, peak-tracking, and torque-saturation-hold
  evaluators; reuses the stage-1 `_longest_contiguous_ms` helper
  via the sibling-module pattern.
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

- Unit tests: `scripts/run_tests.sh --unit-only` — **278 passed**
  (up from 271; +7 tests pinning the settle-window kwarg).
- Integration tests (pre-M6): still expected green —
  **22** `test_math` gtests + **5** `crisp_controllers` gtests +
  **12** integration tests (sim smoke + 3 crisp roles +
  simple_joint_impedance + cartesian_motion, each × {ur5e,
  ur15}). Not re-run this iteration: only the unit-gated stage-2
  module changed; nothing the integration suite depends on moved.
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
  first-principles **drafts** per ADR-0013, including the new
  `stage2` block added this iteration. The schema is frozen;
  numerical review is the open action. Operator is
  expected to edit values in place (`draft: true` flag clears
  when all values are confirmed). Per-joint
  `effective_inertia_kg_m2` values are the most fragile;
  they will be regenerated from the MJCF `armature` + H(q) at
  home pose during M6.1 and should be treated as placeholders
  until then.
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
