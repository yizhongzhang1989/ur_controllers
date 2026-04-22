# Status

_Last updated: 2026-04-22._

## Current milestone

**M1 — Simulator brings up UR5e and UR15** (in progress; largely complete).
Next priority after M1 is M2 (crisp impedance controllers in sim), then M3
(our own joint impedance), then M4 (cartesian controllers). Every sim task
must pass on both `ur5e` and `ur15`. Real UR15 is explicitly out of scope.

## Last completed tasks

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

Start **M2**:

1. Read `third_party/crisp_controllers/README.md` (and any docs under it)
   to enumerate its three impedance controllers and their required
   command interfaces + params. Record in `docs/crisp_controllers.md`.
2. Wire the first impedance controller into a `bringup/launch/` entry
   that reuses `scripts/launch_sim.sh --control_mode effort` plus a crisp
   controllers YAML in `bringup/config/`.
3. Add an integration test under `tests/integration/test_crisp_*.py`
   parametrised over `{ur5e, ur15}` that sends a small regulation command
   and asserts bounded tracking error.

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
