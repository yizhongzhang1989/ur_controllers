# Status

_Last updated: 2026-04-25 (**thirteenth consecutive blocker-only STATUS
iteration — no code change**). M6.0 remains the sole blocking gate and
no operator input has arrived in the interval. Per AGENTS.md §3 step 7
/ §7 the agent commits only this STATUS refresh and stops. Unit tests
re-run (`scripts/run_tests.sh --unit-only`) as a sanity gate on the
docs-only edit — green. The prior full-gate run (unit 1522 + colcon
10 packages + 12 launch tests × {ur5e, ur15}, ~5:18 wall clock)
remains the current green baseline; nothing runtime-facing changed so
integration tests were not re-run._

_Note: this is now the thirteenth consecutive no-progress iteration.
Per AGENTS.md §8, the outer loop's no-progress guard counts
consecutive iterations with **no commit**; these STATUS-only commits
technically satisfy the guard but the operator should be aware that
no code-carrying progress has landed since commit `082cf9b`
(2026-04-25). If operator input on M6.0 remains unavailable, please
drop a `.STOP` file at the repo root to halt the outer loop
explicitly — the agent has nothing productive to commit while M6.0
is open and is now re-requesting that the operator either resolve
M6.0 (recommendation: option (a) — submodule
`third_party/mujoco_ros2_control` tracking `auto_dev`) or halt the
loop._

## Current milestone

**M6 — Unified MuJoCo sim interfaces (runtime-switchable, real-UR
parity), with R1/R2/R3 hard requirements.** Design captured in
ADR-0012 (+ R1/R2/R3 addendum). M0–M5 remain fully ticked. M4
bullet 5 and M-REAL stay out of scope.

## Active blocker (the reason this iteration is STATUS-only)

**M6.0 operator gate — still active.** Decide how to vendor the
`mujoco_ros2_control` patch required by M6.4 and confirm the target
`ur_robot_driver` version. Until this is resolved, every in-scope M6
bullet either touches `third_party/ur_simulator@auto_dev` (M6.1–M6.9)
or depends on an operator-confirmed target version (M6.10, M6.11,
M6.16–M6.19). See "Blockers / open questions for operator" below.

The pre-bake chain the agent has been extending across prior
iterations is **complete for the gated R2/R3 orchestrator**; remaining
open seams are all either deferred-by-design or gated on M6.0 (see
"Pre-bake chain status" below).

This is the thirteenth consecutive iteration with this same
conclusion; no operator input has been received in the interval. Per
AGENTS.md §3 step 7, the agent commits only this STATUS update and
stops.

## Pre-bake chain status (R2/R3 orchestrator)

All of the following landed in-tree and are pinned by dedicated
`tests/unit/test_<module>.py` suites; see git log for the
commit-by-commit narrative.

Modules under `tests/integration/`:

- `expectations_loader.py`, `payload_validation.py`
- `signal_analysis.py`
- Stage-1: `r2_stage1_commands.py`, `r2_stage1_theoretical.py`,
  `r2_stage1_gains.py`, `r2_stage1_assertions.py`
- Stage-2: `r2_stage2_commands.py`, `r2_stage2_theoretical.py`,
  `r2_stage2_cartesian.py`, `r2_stage2_assertions.py`
- Stage-3: `r2_stage3_commands.py`, `r2_stage3_theoretical.py`,
  `r2_stage3_cartesian_gains.py`, `r2_stage3_assertions.py`
- Kinematic adapters (FK/IK **injected**):
  `r2_tcp_from_joints.py`, `r2_joints_from_tcp.py`
- JTC goal-builder pre-bake: `r2_jtc_goal.py`
- Artefact chain: `r2_run_dir.py`, `r2_run_artefact.py`,
  `r2_result_to_artefact.py`, `r2_write_results.py`,
  `r2_find_artefacts.py`, `r2_read_artefact.py`
- Aggregation & reporting: `r2_aggregate.py`, `r2_report_writer.py`
- Orchestrator axes: `r2_stage_controllers.py` (stage→controllers
  catalog), `r3_payload_parametrize.py` (arm×payload axis)
- R3 payload preflight: `r3_payload_ee_msg.py` (message shape),
  `r3_payload_mjcf.py`, `r3_payload_splice.py`, `r3_payload_strip.py`

### Remaining open seams (all deferred-by-design or gated on M6.0)

1. **Concrete FK/IK backends.** Deliberately kept out of tree so the
   source / licensing decision is independent of the orchestrator
   wiring. `r2_tcp_from_joints` / `r2_joints_from_tcp` are both
   injected-callable adapters; any backend (KDL, Pinocchio, analytic
   UR, MJCF-derived) satisfies the contract.
2. **ROS-side materialisers.** `JointTrajectoryGoal →
   FollowJointTrajectory.Goal` and `EePayloadMessage →
   ur_sim_msgs/EePayload`. By design kept outside the pre-bake
   chain so they can import `trajectory_msgs` / `ur_sim_msgs` at
   test-run time. Their pre-bake counterparts (`r2_jtc_goal.py`,
   `r3_payload_ee_msg.py`) already landed.
3. **M6.19 payload parametrisation of R2 stage bodies.** Needs
   M6.16–M6.18, which are themselves gated on M6.0.
4. **Stage-3 theoretical-response extension for
   `cartesian_second_order`.** Deferred: the current
   `tcp_trajectory_tracking` block suffices for
   `cartesian_motion` position-mode, the JTC+ik_shim path, and the
   free-space-drift tolerance on `crisp_cartesian_impedance`.

## Next task (once M6.0 is resolved)

Unblocked ordering:

1. **M6.1** — three-actuator MJCF emission on
   `ur_simulator@auto_dev`.
2. **M6.2** — unified URDF `<ros2_control>` interface surface.
3. **M6.3** — merged `ur_controllers.yaml`.
4. **M6.4** — claim-aware ctrl routing in the vendored
   `mujoco_ros2_control` plugin.
5. **M6.5** — collapsed sim launch; `scripts/launch_sim.sh` drops
   `control_mode`.
6. **M6.6 → M6.8** — runtime-switch test, shim retirement, bringup
   migration.
7. **M6.10 / M6.11 / M6.16–M6.19** — interleave once M6.0 fixes the
   target `ur_robot_driver` version and payload plumbing.

## Build status

- ROS 2 distro: **Humble** (Ubuntu 22.04, matches system install).
- `colcon build` (picking up `colcon_defaults.yaml`) produces **10**
  packages successfully. No build changes this iteration.
- No submodule pointer changes this iteration.

## Test status

- Unit tests: last full run reported **1522 passed**. Re-run this
  iteration (`scripts/run_tests.sh --unit-only`) as a sanity gate
  on the docs-only change — still green.
- Integration tests: last full run — all **12** launch tests green
  (sim smoke + 3 crisp roles + simple_joint_impedance +
  cartesian_motion, each × {ur5e, ur15}); ~5:18 wall clock. Not
  re-run this iteration (no runtime code changed).
- `colcon test`: **10** packages pass (22 `test_math` +
  4 `test_pseudo_inverse` + 5 `test_filters` gtests from
  `crisp_controllers`).
- `pre-commit run --files docs/STATUS.md`: clean.

## Blockers / open questions for operator

Active for **M6** (please resolve in order):

- **[M6.0 gate — still active, 13th iteration]** Decide how to vendor
  the `mujoco_ros2_control` patch required for claim-aware ctrl
  routing. ADR-0012 §Decisions-to-gate lists three options;
  recommendation is **(a) submodule under
  `third_party/mujoco_ros2_control` tracking an `auto_dev` branch**.
  Operator also to confirm the target `ur_robot_driver` version
  (Humble 2.4.x exposes effort; 2.3.x does not). ADR-0012 will be
  amended before M6.1 starts.
- **[M6 R1 — pending]** Confirm the `robot_driver:={sim,real}`
  launch-arg design (M6.11).
- **[M6 R2 — partial]** Tolerance values in
  `tests/integration/expectations/*.yaml` exist as first-principles
  **drafts** per ADR-0013, including the `stage2` block. Schema is
  frozen; numerical review is the open action. Per-joint
  `effective_inertia_kg_m2` values are the most fragile; they will
  be regenerated from the MJCF `armature` + H(q) at home pose during
  M6.1 and should be treated as placeholders until then.
- **[M6 R3 — pending]** Three design knobs to confirm before M6.17:
  1. MJCF-reload vs. in-place body mutation for payload updates
     (ADR-0012 §R3.3 — recommend "regenerate + reload" for iter 1).
  2. Cube-size density (default 1000 kg/m³) and colour mapping
     (red = gravity-comp-unaware active; green = payload-aware).
  3. Whether `ur_sim_msgs/EePayload` is a new message (recommended)
     or we re-use `geometry_msgs/Inertia + PoseStamped`.
- **[M6 scope clarification — pending]** ADR-0007 becomes a runtime
  invariant under M6. Operator to confirm, otherwise ADR-0007 must
  be formally superseded inside ADR-0012 before M6.8.

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

Run `git log --oneline -n 20` for the live list. Narrative summaries
of prior iterations' pre-bake work are preserved in individual commit
messages (grep `test(m6):` in the log) and in the `tests/unit/
test_r2_*.py` / `tests/unit/test_r3_*.py` files themselves.
