# Status

_Last updated: 2026-04-22 (M2 kickoff: crisp controllers reference doc)._

## Current milestone

**M2 — `crisp_controllers` in sim** (just kicked off). M1 is complete.
M2 step 1 (docs reference of crisp's controllers) is done; next is the
first controller role — `joint_impedance_controller` — wired up on ur5e,
then ur15, with an integration test. Priority order after M2: M3 (our own
joint impedance), M4 (cartesian controllers), M5 (evaluation harness).
Every sim task must pass on both `ur5e` and `ur15`. Real UR15 is
explicitly out of scope.

## Last completed tasks

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

Continue **M2**:

1. Write `bringup/config/crisp_joint_impedance.{ur5e,ur15}.yaml` using the
   role pattern documented in `docs/crisp_controllers.md` (k_pos_* = 0,
   nullspace.stiffness > 0, `projector_type: none`). Pick joint order
   matching the sim's controller_manager.
2. Add `bringup/launch/crisp_bringup.launch.py` that accepts
   `robot:={ur5e,ur15}` and `mode:={joint,cartesian,gravity}`; for this
   iteration wire `mode:=joint` end-to-end. Deactivate
   `forward_effort_controller` before activating the crisp controller
   (only one effort commander at a time).
3. Add `tests/integration/test_crisp_joint_impedance.py` parametrised over
   `{ur5e, ur15}` that publishes a small `target_joint` regulation command
   and asserts bounded tracking error on `/joint_states` within a fixed
   window.

See `docs/crisp_controllers.md` for plugin names, required interfaces, and
the shared topic API (`target_pose`, `target_joint`, `target_wrench`).

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
- `test_sim_smoke.py` last validated manually: sim, rosbridge, and dashboard
  came up on ur5e. Full two-arm pytest run still to be re-confirmed after
  the final patches (bumped timeouts, inline joint-name parsing).

## Blockers / open questions for operator

None currently blocking. Informational:

- ROS distro is effectively pinned to Humble (system install); formalise
  via ADR if/when a second distro becomes a candidate.
- Dashboard opens at `http://localhost:8000`; rosbridge at
  `ws://localhost:9090`. Both are pkill'd + port-cleared by
  `scripts/kill_sim.sh`.

## Recent commits

Run `git log --oneline -n 20` for the live list.
