# Status

_Last updated: 2026-04-23 (M2: crisp cartesian_impedance_controller bring-up on ur5e and ur15 with hold-pose integration test)._

## Current milestone

**M2 — `crisp_controllers` in sim**, in progress. M1 is complete. Controllers
1 and 2 (the `joint_impedance_controller` and `cartesian_impedance_controller`
roles of `crisp_controllers/CartesianController`) are both brought up
end-to-end on `ur5e` and `ur15` via `bringup/launch/crisp_bringup.launch.py`
(`mode:={joint,cartesian}`) with per-arm YAML under `bringup/config/`.
Integration tests (`tests/integration/test_crisp_joint_impedance.py`,
`tests/integration/test_crisp_cartesian_impedance.py`) assert bounded
joint-space drift on both arms for each role. Next: Controller 3
(`gravity_compensation` role) same two-step rollout (ur5e, then ur15).
Priority order after M2: M3 (our own joint impedance), M4 (cartesian
controllers), M5 (evaluation harness). Every sim task must pass on both
`ur5e` and `ur15`. Real UR15 is explicitly out of scope.

## Last completed tasks

- **Controller 2 (crisp cartesian impedance) on ur5e and ur15.**
  - `bringup/config/crisp_cartesian_impedance.{ur5e,ur15}.yaml`: per-arm
    param files for the `cartesian_impedance_controller` role of
    `crisp_controllers/CartesianController` — `task.k_pos_* = 400`,
    `task.k_rot_* = 30`, `nullspace.stiffness = 10` with auto damping and
    `projector_type: kinematic` (redundant motion damped behind the
    Cartesian task), `use_gravity_compensation: true`,
    `use_coriolis_compensation: true`, `use_local_jacobian: true`. Joint
    list matches the sim's unprefixed UR set.
  - `bringup/launch/crisp_bringup.launch.py`: extended `mode` choices to
    `{joint, cartesian}` and added the `cartesian_impedance_controller`
    entry to `_ROLE_YAML_STEM`. The spawner `--inactive` +
    strict `switch_controllers` pattern from Controller 1 is reused
    unchanged.
  - `tests/integration/test_crisp_cartesian_impedance.py`: parametrised
    over `{ur5e, ur15}`. Brings up sim + crisp, asserts the swap landed
    (cartesian active, forward effort inactive), then samples
    `/joint_states` over 5 s and asserts max per-joint drift < 0.15 rad.
    No `/target_pose` publish is needed: CartesianController's
    `on_activate` captures the current tool0 pose as its target and
    `q_target` as the current joint vector, so "hold here" is the default
    behaviour under non-zero Cartesian stiffness.
  - `scripts/run_tests.sh` runs the full integration suite (sim smoke × 2
    arms + crisp joint × 2 arms + crisp cartesian × 2 arms = 6 tests) in
    ~2:32, all green.
- **Controller 1 (crisp joint impedance) on ur5e and ur15.**
  - `bringup/config/crisp_joint_impedance.{ur5e,ur15}.yaml`: per-arm param
    files for the `joint_impedance_controller` role of
    `crisp_controllers/CartesianController` — all `task.k_*` = 0,
    `nullspace.projector_type: none`, nullspace stiffness 50 with auto
    damping, `use_gravity_compensation: true`,
    `use_coriolis_compensation: true`. Joint list is the unprefixed UR set
    matching the sim's controller_manager.
  - `bringup/launch/crisp_bringup.launch.py`: takes `robot:={ur5e,ur15}` and
    `mode:=joint|cartesian`. Spawns the controller
    `--inactive` with `--controller-type crisp_controllers/CartesianController`
    and `--param-file <arm yaml>`, then atomically swaps
    `forward_effort_controller` → role controller via
    `ros2 control switch_controllers --strict` chained on spawner exit.
    The sim's `gravity_compensation.py` still runs but only targets the now
    inactive `forward_effort_controller`; crisp owns effort end-to-end via
    its pinocchio-based gravity + Coriolis terms.
  - `tests/integration/test_crisp_joint_impedance.py`: parametrised over
    `{ur5e, ur15}`. Brings up the sim via `scripts/launch_sim.sh`, waits
    for `joint_state_broadcaster` + `forward_effort_controller` active,
    launches `crisp_bringup.launch.py`, asserts the swap landed (crisp
    active, forward effort inactive), publishes `/target_joint` at
    the current joint positions, and asserts max per-joint drift stays
    under 0.15 rad over a 5 s window.
- `ur_sim_mujoco.launch.py` (submodule `third_party/ur_simulator`,
  `auto_dev` branch): serialise controller spawners. JSB spawner runs
  first alone; the other five chain off its `OnProcessExit`. Adds
  `--service-call-timeout 30` to every spawner. Fixes a Humble + FastRTPS
  race where concurrent spawners overflowed the RMW response queue,
  dropping `joint_state_broadcaster`'s `load_controller` reply → retry
  hit "already loaded" → FATAL → `/joint_states` never published. See
  ADR-0006. `scripts/run_tests.sh` now green on both `ur5e` and `ur15`
  (~25 s). Parent-repo pointer bump commit chained on the submodule
  commit.
- `docs/crisp_controllers.md` added: enumerates the plugin classes exported
  by `third_party/crisp_controllers`, clarifies that the "three impedance
  controllers" are three configuration roles of the same
  `crisp_controllers/CartesianController` plugin (cartesian_impedance,
  joint_impedance, gravity_compensation), lists required command/state
  interfaces and topics, and sketches the M2 bring-up pattern we'll follow
  (per-role YAML + one launch file with `robot` and `mode` args +
  parametrised integration test). Completes the first unchecked item of
  M2 in `docs/ROADMAP.md`.
- `scripts/auto_dev_loop.sh` now invokes the GitHub Copilot CLI with
  `--allow-all-tools -p` (previous default passed the prompt positionally
  and failed with "Invalid command format").
- `tests/integration/test_sim_smoke.py::_controllers_active` now strips
  ANSI colour escapes and sets `NO_COLOR=1` on the `ros2 control
  list_controllers` call. The humble CLI colourises its output, which
  broke the whitespace-sensitive `" active"` substring match and caused
  the smoke test to fail on both arms. Both `ur5e` and `ur15` now pass
  `scripts/run_tests.sh` (~25 s).
- Submodules pulled in and colcon-built (`cartesian_controllers`,
  `crisp_controllers`, `ur_sim_config`, `ur_simulation_gz`). See build
  status below.
- Sim launcher (`scripts/launch_sim.sh`) wraps the simulator's own
  `launch_all.sh` (MuJoCo backend, effort mode by default — needed for
  crisp and our own joint impedance).
- Port/process reaper (`scripts/kill_sim.sh`) clears stale rosbridge
  (9090), dashboard (8000), ROS 2 CLI daemon, and sim processes before
  each launch.
- Integration smoke test (`tests/integration/test_sim_smoke.py`)
  parametrised over `{ur5e, ur15}` asserting `/joint_states`, active
  controllers, rosbridge port, and dashboard HTTP 200.

## Next task (agent should pick this up)

Continue **M2** with **Controller 3 (crisp gravity compensation)** — same
two-step rollout pattern as Controllers 1 and 2:

1. Write `bringup/config/crisp_gravity_compensation.{ur5e,ur15}.yaml`.
   Use the `gravity_compensation` role from `docs/crisp_controllers.md`:
   all `task.k_pos_* = 0`, optionally `k_rot_* ≈ 30` (or 0),
   `nullspace.stiffness: 0` so the controller becomes a pure
   model-compensation passthrough. Keep `use_gravity_compensation: true`,
   `use_coriolis_compensation: true`. Same unprefixed UR joint list.
2. Extend `bringup/launch/crisp_bringup.launch.py`: add `"gravity"` to
   the `mode` choices and a
   `("gravity_compensation", "crisp_gravity_compensation")` entry to
   `_ROLE_YAML_STEM`. The spawner + strict `switch_controllers` pattern
   already works for this role — no structural changes needed.
3. Add `tests/integration/test_crisp_gravity_compensation.py`,
   parametrised over `{ur5e, ur15}`. Assert the swap landed, then sample
   `/joint_states` and assert bounded drift over a fixed window. Reuse
   the helper shape in `test_crisp_cartesian_impedance.py` (no target
   publish needed — pure gravity comp should drift very slowly if at
   all; use the same 0.15 rad tolerance to keep tests comparable).

After Controller 3 the remaining M2 items are the enumeration doc refresh
(already partially done), the integration-test umbrella check, and the
baseline rosbag manifest. See `docs/crisp_controllers.md` for plugin
names, required interfaces, and the shared topic API (`target_pose`,
`target_joint`, `target_wrench`).

## Build status

- ROS 2 distro: **Humble** (Ubuntu 22.04, matches system install).
- `colcon build --symlink-install --base-paths src third_party
  --packages-skip cartesian_controller_simulation cartesian_controller_tests`
  produces 9 packages successfully:
  - 6 from `cartesian_controllers`
  - `crisp_controllers`
  - `ur_sim_config`
  - `ur_simulation_gz`
- Skipped: `cartesian_controller_simulation` (needs MuJoCo C lib at
  `/home/robot/mujoco-3.0.0`, not installed; we use `ur_simulator`
  instead), `cartesian_controller_tests` (ROS 1 catkin package).

## Test status

- `scripts/run_tests.sh` runs unit → colcon test → integration.
- Integration stage sources `/opt/ros/humble/setup.bash` and `install/setup.bash`
  before pytest.
- `test_sim_smoke.py` validated via `scripts/run_tests.sh`: both `ur5e`
  and `ur15` pass (2 passed in ~25 s) after the spawner-serialisation
  fix in `ur_sim_mujoco.launch.py` (ADR-0006).
- `test_crisp_joint_impedance.py` + `test_crisp_cartesian_impedance.py`
  both green on both arms; total integration suite (6 tests) runs in
  ~2:32.

## Blockers / open questions for operator

None currently blocking. Informational:

- ROS distro is effectively pinned to Humble (system install); formalise
  via ADR if/when a second distro becomes a candidate.
- Dashboard opens at `http://localhost:8000`; rosbridge at
  `ws://localhost:9090`. Both are pkill'd + port-cleared by
  `scripts/kill_sim.sh`.

## Recent commits

Run `git log --oneline -n 20` for the live list.
