# Status

_Last updated: 2026-04-23 (feat(m5): draft scenario schema v1 +
validator + 4 example scenarios + 35 pytest unit tests. Full test gate
green in ~6:30.)_

## Current milestone

**M5 — evaluation and comparison harness: IN PROGRESS.** M0–M3 done.
M4 bullets 1–4 done; the optional M4 bullet 5 (second cartesian mode)
is parked on a human gate (sim F/T sensor patch on `auto_dev` —
unchanged from prior iteration). M5 bullet 1 done this iteration:
declarative scenario schema v1 lives in `evaluation/scenarios/`
(README spec + stdlib+PyYAML validator + 4 example YAMLs covering
step / sine / regulation / random_waypoints), exercised by 35
`tests/unit/test_scenarios_schema.py` cases.

Key observations from this iteration (relevant for future work):

- **Scenario schema v1 design choices** (see `evaluation/scenarios/README.md`
  for the full spec). Two shapes that future M5 bullets must respect:
  1. `target` describes *what* is being commanded (space, joints / frame +
     end_effector); `command` carries the *how* (per-`scenario_type`
     parameters). They were deliberately split so adding new
     scenario_types later (e.g. `trajectory`, `chirp`) does not require
     touching `target`.
  2. Cartesian targets are accepted in v1 only for `scenario_type:
     regulation` (position + unit quaternion in `target.frame_id`).
     `step`, `sine`, and `random_waypoints` are joint-only because their
     cartesian variants need explicit interpolation + frame semantics
     that we have not committed to. When `cartesian_motion_controller`
     starts being driven by the M5 harness, regulation is enough for
     a comparison test; richer cartesian scenarios will land in a
     schema v2 with a new ADR.
  3. Per-joint amplitudes are required as a length-6 vector (canonical
     UR joint order). Scalar `amplitude_rad` is rejected because it
     leaves the active joint ambiguous.
  4. Time consistency is validated up front: `step_time_s < duration_s`
     and `num_waypoints * dwell_s <= duration_s`. Pass-criteria keys
     must each map to a metric listed under `metrics` (so a threshold
     can never reference a metric that nothing computes).
- **`scripts/run_tests.sh` stage 1 now passes `-p no:anyio`.** This was
  required because the system pytest-anyio plugin is incompatible with
  our pinned pytest version (already worked around in stage 3 for
  integration tests). With `tests/unit/` now populated, stage 1 needed
  the same flag.

- **`robot_description` propagation gotcha.** On Humble 2.53, the
  controller manager propagates its own `robot_description` param
  down to newly-loaded controllers only when the param was given to
  the CM at node construction. Our sim's `ur_sim_mujoco.launch.py`
  instead relies on the `/robot_description` topic path (remapping
  `~/robot_description` → `/robot_description`), which populates the
  CM's internal copy but does **not** push it to each controller.
  `CartesianControllerBase::on_configure` reads from its own node
  param so it sees `robot_description = ""` and aborts. Crisp dodges
  this by querying `/controller_manager` directly via a parameters
  client (see `third_party/crisp_controllers/src/cartesian_controller.cpp:242`).
  We cannot patch the read-only `cartesian_controllers` submodule
  (ADR-0003), so the launch fetches `robot_description` from
  `/robot_state_publisher` via rclpy and passes it to the spawner as
  a second `--param-file` (see `bringup/launch/cartesian_bringup.launch.py`).
  If/when a future controller with the same "reads from own node
  param" pattern lands, reuse this approach or fix the sim to pass
  `robot_description` as a CM-node param on `auto_dev`.

## Last completed tasks

- **M5 bullet 1: `evaluation/scenarios/*.yaml` schema v1.** New files:
  `evaluation/scenarios/README.md` (full schema spec),
  `evaluation/scenarios/validate.py` (PyYAML + stdlib validator with a
  CLI entrypoint), four committed example scenarios
  (`step`, `sine`, `regulation`, `random_waypoints`),
  `tests/unit/test_scenarios_schema.py` (35 cases: positive + per-rule
  negative + YAML-load failure paths). The validator is deliberately
  controller-agnostic and arm-agnostic so M5 bullets 2–4 can drive
  every controller already wired in M2–M4 from the same scenario file.
  See ADR-0009 for the design rationale.
- **M4 bullets 2 + 3 + 4: `cartesian_motion_controller` brought up on
  ur5e and ur15 with regulation integration test.** New files:
  `bringup/config/cartesian_motion.{ur5e,ur15}.yaml`,
  `bringup/launch/cartesian_bringup.launch.py`,
  `tests/integration/test_cartesian_motion.py`. Sim runs in position
  mode (`scripts/launch_sim.sh <robot> position`); bring-up spawns the
  controller `--inactive` and atomically swaps
  `joint_trajectory_controller` → `cartesian_motion_controller` via
  `ros2 control switch_controllers --strict` (same pattern as ADR-0007
  but in position interfaces, not effort). Test asserts the swap
  landed and joints stay within 0.15 rad of the hold pose for 5 s —
  the same threshold used by the crisp and simple_jimp regulation
  tests so the M5 comparison harness can flip between controllers
  with one flag.
- **M4 kick-off: `docs/cartesian_controllers.md` reference + tick M4
  bullet 1 (build).** See prior STATUS.
- **M3 — sim bring-up + regulation integration test for
  `simple_joint_impedance_controller` on `{ur5e, ur15}`.** See prior
  STATUS.
- **M3 — control law baseline in `simple_joint_impedance_controller`.**
  See prior STATUS.
- **Fix: strictly serialise MuJoCo controller spawners
  (`third_party/ur_simulator`).** See prior STATUS.
- **M3 — `simple_joint_impedance_controller` package skeleton.** See
  prior STATUS.
- **M3 — ADR-0008: scope the simplified joint impedance controller.**
  See prior STATUS.
- **M2 baseline rosbags + manifests.** See prior STATUS.

## Next task (agent should pick this up)

**M5 bullet 2: `evaluation/run_evaluation.py`.** Take a scenario YAML
(validated by `evaluation/scenarios/validate.py`), a controller name
(one of the bring-ups already wired in M2–M4), and a robot
(`ur5e`|`ur15`); launch the sim + the chosen controller, drive
`/target_joint` (joint-space scenarios) or the relevant cartesian
target topic per the scenario's `command`, record `/joint_states` +
the controller's `~/tau_d` topic, and emit a CSV under
`evaluation/runs/<scenario>__<controller>__<robot>__<timestamp>/`
(plus a `manifest.yaml` mirroring the existing
`evaluation/baselines/crisp/*.manifest.yaml` shape). Metrics
computation (M5 bullet 3) and the comparison report (M5 bullet 4)
build on top of this. Reference for the launch+swap pattern: existing
`bringup/launch/{crisp,simple_jimp,cartesian}_bringup.launch.py`.

M4 bullet 5 (second cartesian mode) remains deferred behind a
**human gate** (sim-side F/T sensor patch on `auto_dev`).

## Build status

- ROS 2 distro: **Humble** (Ubuntu 22.04, matches system install).
- `colcon build --symlink-install --base-paths src third_party
  --packages-skip cartesian_controller_simulation cartesian_controller_tests`
  produces **10** packages successfully, unchanged from prior
  iteration (no new ROS packages added — this iteration only touched
  `evaluation/`, `tests/unit/`, `scripts/run_tests.sh`, and docs).

## Test status

- `scripts/run_tests.sh` runs unit → colcon test → integration. Green
  in ~6:30 this iteration (35 new unit tests + same 27 colcon gtests +
  same 12 integration tests).
- Test counts: **35** pytest unit tests
  (`tests/unit/test_scenarios_schema.py`) + **22** `test_math` gtests
  (simple_joint_impedance_controller) + **5** `crisp_controllers`
  gtests + **12** integration tests (sim smoke + 3 crisp roles + our
  simple_joint_impedance_controller + `cartesian_motion_controller`,
  each ×{ur5e, ur15}).

## Blockers / open questions for operator

None currently blocking. Informational:

- ROS distro pinned to Humble (system install); formalise via ADR if/
  when a second distro becomes a candidate.
- Dashboard opens at `http://localhost:8000`; rosbridge at
  `ws://localhost:9090`. Both are pkill'd + port-cleared by
  `scripts/kill_sim.sh`.
- The cartesian compliance and force controllers both need an
  `ft_sensor_ref_link` on the URDF chain plus an `~/ft_sensor_wrench`
  publisher. The UR sim does not currently expose an F/T sensor frame;
  those controllers therefore require a sim-side `auto_dev` patch
  before they can be brought up end-to-end. Now directly gating M4
  bullet 5 — surface to the operator when/if that bullet is picked up.
- Gravity-comp hook for `simple_joint_impedance_controller` remains
  unused (pure PD holds both arms within the M3 tolerance).

## Recent commits

Run `git log --oneline -n 20` for the live list.
