# Status

_Last updated: 2026-04-24 (IK-injected adapter pre-baked as
`tests/integration/r2_joints_from_tcp.py` + 30 unit tests — folds a
commanded `TcpTrajectory` through an injected IK callable into a
`JointCommandTrajectory` consumable by a JTC goal-builder, mirroring
the FK adapter and closing the last pre-bake gap on the R2 stage-3
`JTC + ik_shim` combo. M6.0 operator gate still active for every
bullet that requires live sim changes.)._

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
and now the IK-injected adapter (`r2_joints_from_tcp.py` —
`joint_trajectory_from_tcp` folds a commanded `TcpTrajectory` through
an injected `ik(pos, quat, q_seed) -> Sequence[float]` into a
`JointCommandTrajectory` whose `positions` mapping is shape-compatible
with `AllJointsCommandTrace.positions` so the same JTC goal-builder
can consume either; seeds threaded from previous output for branch
continuity; duck-type accepts anything TcpTrajectory-shaped so the
two-module-identity trap doesn't bite orchestrators), M6.12 (R2
stage-1), the joint-space path of M6.13 (R2 stage-2), and **both**
paths of M6.14 (R2 stage-3 `cartesian_motion` via the direct TCP
commanded generators **and** `JTC + ik_shim` via the new IK adapter)
can all be authored end-to-end as sim-collection orchestrators — they
just can't run until M6.5 ships **and** the operator picks concrete
FK and IK backends (KDL / Pinocchio / MJCF-derived) to plug into the
two adapters.

Still missing on the pre-bake chain:

1. The concrete FK **and** IK backends themselves — both adapters
   deliberately keep these out of tree so the source / licensing
   decision is independent of the orchestrator wiring.
2. M6.19 payload parametrisation across R2 stages (needs M6.16–
   M6.18 to land first).

## Last completed tasks

- **This iteration: IK-injected adapter (pre-bake for M6.14 `JTC +
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

- Unit tests: `scripts/run_tests.sh --unit-only` — **450 passed**
  (up from 420; +30 tests pinning the IK-injected adapter).
- Integration tests: re-run this iteration — all **12** launch
  tests green (sim smoke + 3 crisp roles + simple_joint_impedance
  + cartesian_motion, each × {ur5e, ur15}); ~5:20 wall clock.
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
