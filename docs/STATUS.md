# Status

_Last updated: 2026-04-24 (FK-injected TCP adapter pre-baked as
`tests/integration/r2_tcp_from_joints.py` + 26 unit tests — turns
measured joint-state samples into a `TcpTrajectory` via an injected
`fk` callable, closing the last purely-stdlib gap between stage-3's
commanded side and assertion side. M6.0 operator gate still active
for every bullet that requires live sim changes; the FK source /
licensing decision remains the only other non-sim gate on the
pre-bake chain.)._

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
harness, the stage-3 **commanded** TCP trajectory generators, and
now the **FK-injected adapter** (`r2_tcp_from_joints.py` —
`tcp_trajectory_from_joints(times, joint_samples, fk) ->
TcpTrajectory`, closes over `arm` inside the `fk` callable, rebases
input `times` to `t0=0` so the `TcpTrajectory` invariant holds, and
wraps `fk` exceptions with `sample index + absolute timestamp` for
long-trace debuggability), M6.12 (R2 stage-1), the joint-space path
of M6.13 (R2 stage-2), and the assertion side of M6.14 (R2 stage-3)
can all be authored end-to-end as sim-collection orchestrators — they
just can't run until M6.5 ships **and** the operator picks a concrete
FK backend (KDL / Pinocchio / MJCF-derived) to plug into the adapter.
Still missing on the pre-bake chain:

1. The concrete FK backend itself — the adapter deliberately
   keeps this out of tree so the source / licensing decision is
   independent of the orchestrator wiring.
2. M6.19 payload parametrisation across R2 stages (needs M6.16–
   M6.18 to land first).

## Last completed tasks

- **This iteration: FK-injected TCP adapter (pre-bake glue for R2
  stage-2 cartesian consistency + R2 stage-3 measured side).**
  Added `tests/integration/r2_tcp_from_joints.py` exporting
  `tcp_trajectory_from_joints(times, joint_samples, fk) ->
  TcpTrajectory`. The adapter takes an injected FK callable
  (`fk(q) -> (pos_xyz_m, quat_xyzw)`; any arm / kinematic-model
  selection the backend needs is closed over by the caller) and
  folds a measured joint-state stream into the same frozen
  `TcpTrajectory` dataclass that `r2_stage3_commands` emits and
  `evaluate_tcp_trajectory` consumes. Times are **rebased to
  `t0=0`** inside the adapter so the `TcpTrajectory` invariant
  `times[0] == 0.0` holds regardless of whether the caller's
  `/joint_states` trace started at wall-clock zero. Validates
  length match, strict monotonicity + finiteness of `times`,
  joint-sample dim consistency + finiteness, FK output shape
  (3-float pos / 4-float quat), FK output finiteness, and
  quaternion norm in the same `[0.5, 1.5]` band the stage-3
  modules already use — returned quats are unit-normalised so
  downstream consumers never have to re-normalise. FK exceptions
  are re-raised as `ValueError` carrying the sample index and the
  **absolute** (pre-rebase) timestamp, so a long-trace failure
  maps straight back to the raw joint-state log. Deliberately
  does **not** cover commanded/measured time alignment — that
  stays with the orchestrator so the adapter owns exactly one
  concern. Pure stdlib, loads the sibling `TcpTrajectory` via the
  file-based importlib pattern already used by
  `r2_stage2_assertions`. Pinned by 26 unit tests in
  `tests/unit/test_r2_tcp_from_joints.py`: happy path (returns
  `TcpTrajectory`, rebases times, forwards FK pos, unit-normalises
  FK quats, accepts numpy-ish `__float__`-coercible inputs),
  end-to-end interop with `r2_stage3_assertions.evaluate_tcp_trajectory`
  (feed adapter output as both commanded and measured -> zero
  RMSE / peak / orientation error, `result.ok == True`), input
  validation (length mismatch, empty / single-sample input,
  non-monotonic + duplicate + non-finite times, inconsistent
  joint-vector dim, empty joint vector, non-finite + non-numeric
  joint values), FK-output validation (exception wrapping with
  sample index + timestamp + original message, wrong pos length,
  wrong quat length, wrong overall shape, non-finite pos, non-finite
  quat, zero quat, oversize quat outside the `[0.5, 1.5]` band),
  and export surface (`__all__` lists `FkCallable`, `TcpTrajectory`,
  `tcp_trajectory_from_joints`; `TcpTrajectory` dataclass fields
  match the sibling one-for-one; `duration_s` is a property on
  both). Unit gate now reports **361 passed** (up from 335).
  Full `scripts/run_tests.sh` green end-to-end: unit (361) +
  colcon test (10 packages, 22 `test_math` + 5 `test_filters` +
  4 `test_pseudo_inverse` gtests) + integration (12 launch tests
  × {ur5e, ur15}), ~5 min wall clock.
- **Prior iteration: R2 stage-3 commanded TCP trajectory generators.**
  Added `tests/integration/r2_stage3_commands.py` exporting
  `TcpTrajectory` + `line_trajectory` / `arc_trajectory` /
  `sine_in_z_trajectory`, pinned by 30 unit tests.
- **Prior iteration: R2 stage-3 TCP assertion harness.** Added
  `tests/integration/r2_stage3_assertions.py` with
  `evaluate_tcp_trajectory(...)` (position RMSE + peak, per-axis
  RPY peak orientation error, optional two-half-means steady
  drift gated to `steady_window_s >= 30 s`) and the
  expectation-driven wrapper
  `evaluate_tcp_trajectory_from_expectation(arm, controller,
  ...)` that sources the four `tcp.tolerances` keys from
  `ArmExpectation.tcp_tolerances`.
- **Prior iteration: stage-2 evaluator ArmExpectation wrapper.**
  Added `evaluate_all_joints_from_expectation(arm, controller,
  ...)` to `tests/integration/r2_stage2_assertions.py`. The
  wrapper forwards to `evaluate_all_joints_joint_space` after
  sourcing `completion_tol_rad` / `peak_tracking_err_rad` /
  `saturation_hold_ms` from `arm.stage2` and
  `effort_limits_nm = {j.name: j.effort_limit_nm for j in
  arm.joints}`. Duck-typed on the input (any object exposing
  those attributes works, matching the stage-1 convention of
  accepting a `ControllerExpectation` object directly). Guards
  against silent drops by raising `ValueError` when `torques`
  contains joint keys not present in the arm. Added to
  `__all__`. Pinned by +7 unit tests in
  `tests/unit/test_r2_stage2_assertions.py`: happy path byte-
  matches the explicit-kwargs call, stage-2 `completion_tol_rad`
  propagates, stage-2 `peak_tracking_err_rad` propagates, per-
  joint `effort_limit_nm` feeds the saturation-hold check,
  `settle_window_s` is forwarded through, unknown torque joints
  reject, and a smoke test exercises the wrapper against the real
  `expectations_loader.ArmExpectation` dataclass (so a future
  schema drift in the loader surfaces here before integration).
  Unit gate now reports **285 passed** (up from 278). Full
  `scripts/run_tests.sh` green end-to-end: unit (285) +
  colcon test (10 packages, 22 `test_math` + 5 `test_filters`
  gtests) + integration (12 launch tests across sim_smoke, three
  crisp roles, simple_jimp, cartesian_motion, each × {ur5e,
  ur15}), total ~5 min.
- **Prior iteration: stage-2 evaluator terminal settle window.**
  `evaluate_all_joints_joint_space` accepts `settle_window_s`;
  when positive the motion-completion check averages
  `|q(t) - q_cmd(t)|` over the trailing window.
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

- Unit tests: `scripts/run_tests.sh --unit-only` — **361 passed**
  (up from 335; +26 tests pinning the FK-injected TCP adapter).
- Integration tests: re-run this iteration — all **12** launch
  tests green (sim smoke + 3 crisp roles + simple_joint_impedance
  + cartesian_motion, each × {ur5e, ur15}); ~4:54 wall clock.
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
