# Status

_Last updated: 2026-04-23 (M2: roadmap ticks synced to reality — all
three crisp roles and the integration-tests umbrella are now `[x]`; only
the baseline rosbag manifest remains)._

## Current milestone

**M2 — `crisp_controllers` in sim**, nearly complete. M1 is complete. All
three configuration roles of `crisp_controllers/CartesianController`
(`joint_impedance_controller`, `cartesian_impedance_controller`,
`gravity_compensation`) are brought up end-to-end on `ur5e` and `ur15`
via `bringup/launch/crisp_bringup.launch.py`
(`mode:={joint,cartesian,gravity}`) with per-arm YAML under
`bringup/config/`. Integration tests (`tests/integration/test_crisp_*.py`)
assert bounded joint-space drift on both arms for each role. The
enumeration doc (`docs/crisp_controllers.md`) was verified against the
plugin manifest. The only remaining M2 item is the baseline rosbag
manifest under `evaluation/baselines/crisp/`. Priority order after M2:
M3 (our own joint impedance), M4 (cartesian controllers), M5 (evaluation
harness). Every sim task must pass on both `ur5e` and `ur15`. Real UR15
is explicitly out of scope.

## Last completed tasks

- **Sync `docs/ROADMAP.md` M2 ticks with reality.** Four items flipped
  `[ ]`→`[x]` against existing artifacts:
  - Enumeration of the three impedance controllers in
    `docs/crisp_controllers.md` — verified against
    `third_party/crisp_controllers/crisp_controllers.xml` (four plugin
    classes: `CartesianController`, `TorqueFeedbackController`,
    `PoseBroadcaster`, `TwistBroadcaster`) and the three
    `CartesianController` configuration roles.
  - Controller 2 rollout (`cartesian_impedance_controller` role) on
    ur5e + ur15 — configs
    `bringup/config/crisp_cartesian_impedance.{ur5e,ur15}.yaml`, test
    `tests/integration/test_crisp_cartesian_impedance.py`.
  - Controller 3 rollout (`gravity_compensation` role) on ur5e + ur15 —
    configs `bringup/config/crisp_gravity_compensation.{ur5e,ur15}.yaml`,
    test `tests/integration/test_crisp_gravity_compensation.py`.
  - Integration-tests umbrella — three `test_crisp_*.py` files each
    parametrised over `{ur5e, ur15}`, all green in `scripts/run_tests.sh`.
  No code change; documentation-only. Test gate: 8/8 green in ~3:17
  (one earlier sim-bringup flake on `ur15` — `forward_effort_controller`
  did not come active within the 90 s window on a cold run; re-ran clean,
  consistent with ADR-0006's known RMW-race envelope).
- **De-flake integration `/joint_states` sampling.** The regulation-window
  loops in `tests/integration/test_crisp_{joint,cartesian,gravity}*.py`
  used to spawn one `ros2 topic echo --once /joint_states` per sample.
  The ~0.5–2 s node-startup/discovery cost per echo meant the 5 s window
  occasionally collected only 2 samples, tripping the `samples >= 3`
  stall guard (observed once on `ur5e` cartesian). Replaced the per-sample
  subprocess with a single persistent rclpy subscription
  (`_collect_joint_state_samples(duration_s)`) that spins for the full
  window and returns all received messages. The drift tolerance
  (`< 0.15 rad`) and the `samples >= 3` stall guard are unchanged — the
  fix removes an instrumentation artefact, not a real assertion. Full
  suite is green (8/8, ~3:40).
- **Controller 3 (crisp gravity compensation) on ur5e and ur15.** See
  prior STATUS for details.
- **Controller 2 (crisp cartesian impedance) on ur5e and ur15.** See
  prior STATUS for details.
- **Controller 1 (crisp joint impedance) on ur5e and ur15.** See prior
  STATUS for details.
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

Only one M2 item remains: **baseline rosbag manifest.** Record a short
regulation scenario per role × arm, commit
`evaluation/baselines/crisp/<role>.<arm>.manifest.yaml` listing topics /
duration / sim seed (bag payload gitignored per the existing rule in
`docs/ROADMAP.md`). After that, M2 is closed and M3 (our own simplified
joint impedance controller) is next — the bring-up pattern and test
shape from `test_crisp_joint_impedance.py` will carry over directly.

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
