# Status

_Last updated: 2026-04-23 (M2: crisp gravity_compensation role brought up on ur5e and ur15)._

## Current milestone

**M2 — `crisp_controllers` in sim**, in progress. M1 is complete. All three
configuration roles of `crisp_controllers/CartesianController`
(`joint_impedance_controller`, `cartesian_impedance_controller`,
`gravity_compensation`) are now brought up end-to-end on `ur5e` and `ur15`
via `bringup/launch/crisp_bringup.launch.py`
(`mode:={joint,cartesian,gravity}`) with per-arm YAML under
`bringup/config/`. Integration tests (`tests/integration/test_crisp_*.py`)
assert bounded joint-space drift on both arms for each role. Next: refresh
the `docs/crisp_controllers.md` enumeration tick, then the broader
integration-test umbrella check and the baseline rosbag manifest to close
out M2. Priority order after M2: M3 (our own joint impedance), M4
(cartesian controllers), M5 (evaluation harness). Every sim task must pass
on both `ur5e` and `ur15`. Real UR15 is explicitly out of scope.

## Last completed tasks

- **Controller 3 (crisp gravity compensation) on ur5e and ur15.**
  - `bringup/config/crisp_gravity_compensation.{ur5e,ur15}.yaml`: per-arm
    param files for the `gravity_compensation` role of
    `crisp_controllers/CartesianController` — all `task.k_*` = 0,
    `nullspace.stiffness = 0` with `projector_type: none`, so the
    controller is a pure pinocchio gravity + Coriolis feed-forward
    passthrough. `use_gravity_compensation: true`,
    `use_coriolis_compensation: true`. Joint list matches the sim's
    unprefixed UR set.
  - `bringup/launch/crisp_bringup.launch.py`: extended `mode` choices to
    `{joint, cartesian, gravity}` and added the `gravity_compensation`
    entry to `_ROLE_YAML_STEM`. No structural changes — the existing
    spawner `--inactive` + strict `switch_controllers` pattern covers
    this role.
  - `tests/integration/test_crisp_gravity_compensation.py`: parametrised
    over `{ur5e, ur15}`. Brings up sim + crisp, asserts the swap landed
    (`gravity_compensation` active, `forward_effort_controller`
    inactive), then samples `/joint_states` over 5 s and asserts max
    per-joint drift < 0.15 rad — verifying the pinocchio gravity term
    is actually holding the arm up (zero-torque would cause the arm to
    fall, failing the tolerance).
  - `scripts/run_tests.sh` now runs 8 integration tests (sim smoke × 2
    arms + crisp joint × 2 arms + crisp cartesian × 2 arms + crisp
    gravity × 2 arms) in ~3:20, all green.
- **Controller 2 (crisp cartesian impedance) on ur5e and ur15.**
  See previous STATUS for details: `cartesian_impedance_controller`
  role with `k_pos_* = 400`, `k_rot_* = 30`, nullspace behind a
  kinematic Cartesian projector; `test_crisp_cartesian_impedance.py`
  asserts hold-pose drift < 0.15 rad on both arms.
- **Controller 1 (crisp joint impedance) on ur5e and ur15.**
  See previous STATUS for details: `joint_impedance_controller` role
  with nullspace PD (`projector_type: none`, stiffness 50);
  `test_crisp_joint_impedance.py` publishes `/target_joint` at current
  q and asserts drift < 0.15 rad on both arms.
- `ur_sim_mujoco.launch.py` (submodule `third_party/ur_simulator`,
  `auto_dev` branch): serialise controller spawners. See ADR-0006.
- `docs/crisp_controllers.md` added: enumerates the plugin classes
  exported by `third_party/crisp_controllers`, clarifies the three
  configuration roles, lists required interfaces and topics, and
  sketches the M2 bring-up pattern.
- `scripts/auto_dev_loop.sh` invokes the GitHub Copilot CLI with
  `--allow-all-tools -p`.
- `tests/integration/test_sim_smoke.py::_controllers_active` strips
  ANSI colour escapes and sets `NO_COLOR=1` on the `ros2 control
  list_controllers` call.
- Submodules pulled in and colcon-built (`cartesian_controllers`,
  `crisp_controllers`, `ur_sim_config`, `ur_simulation_gz`). See build
  status below.
- Sim launcher (`scripts/launch_sim.sh`) wraps the simulator's own
  `launch_all.sh` (MuJoCo backend, effort mode by default).
- Port/process reaper (`scripts/kill_sim.sh`) clears stale rosbridge
  (9090), dashboard (8000), ROS 2 CLI daemon, and sim processes before
  each launch.
- Integration smoke test (`tests/integration/test_sim_smoke.py`)
  parametrised over `{ur5e, ur15}` asserting `/joint_states`, active
  controllers, rosbridge port, and dashboard HTTP 200.

## Next task (agent should pick this up)

All three crisp roles are green. Remaining M2 items (in `docs/ROADMAP.md`):

1. **Enumeration doc refresh + tick.** `docs/crisp_controllers.md` already
   describes the three roles; verify it matches what actually ships and
   tick the first unchecked item of M2 in `docs/ROADMAP.md`.
2. **Integration-test umbrella check.** M2's "Integration tests under
   `tests/integration/test_crisp_*.py`" item is satisfied in practice
   (three test files, each parametrised over `{ur5e, ur15}`, all green).
   Tick the item in `docs/ROADMAP.md`.
3. **Baseline rosbag manifest.** Record a short regulation scenario per
   role × arm, commit `evaluation/baselines/crisp/<role>.<arm>.manifest.yaml`
   listing topics / duration / sim seed (bag payload gitignored per the
   existing rule in `docs/ROADMAP.md`).

Any one of these closes out another M2 task. After M2 is fully done, M3
(our own simplified joint impedance controller) is next — the bring-up
pattern and test shape from `test_crisp_joint_impedance.py` will carry
over directly.

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
- Integration stage sources `/opt/ros/humble/setup.bash` and
  `install/setup.bash` before pytest.
- Full integration suite: 8 tests (sim smoke + 3 crisp roles, each
  ×{ur5e, ur15}) — all green in ~3:20.

## Blockers / open questions for operator

None currently blocking. Informational:

- ROS distro is effectively pinned to Humble (system install); formalise
  via ADR if/when a second distro becomes a candidate.
- Dashboard opens at `http://localhost:8000`; rosbridge at
  `ws://localhost:9090`. Both are pkill'd + port-cleared by
  `scripts/kill_sim.sh`.

## Recent commits

Run `git log --oneline -n 20` for the live list.
