# Status

_Last updated: 2026-04-23 (M2 closed: crisp baseline rosbags recorded for
all 3 roles × {ur5e, ur15}; manifests committed under
`evaluation/baselines/crisp/`. Next: M3 — our own simplified joint
impedance controller)._

## Current milestone

**M2 — `crisp_controllers` in sim: DONE.** M1 is done. All three
configuration roles of `crisp_controllers/CartesianController`
(`joint_impedance_controller`, `cartesian_impedance_controller`,
`gravity_compensation`) are brought up end-to-end on `ur5e` and `ur15`
via `bringup/launch/crisp_bringup.launch.py`
(`mode:={joint,cartesian,gravity}`) with per-arm YAML under
`bringup/config/`. Integration tests (`tests/integration/test_crisp_*.py`)
assert bounded joint-space drift on both arms for each role. Baseline
rosbags for each of the 6 role×arm combinations are recorded by
`scripts/record_crisp_baseline.sh` (one-shot; not part of
`run_tests.sh`); payloads live under `evaluation/baselines/crisp/*.bag/`
(gitignored) and per-combination manifests
(`evaluation/baselines/crisp/<role>_<robot>.manifest.yaml`) are
committed. Next milestone: **M3 — our own simplified joint impedance
controller**. Priority order after M3: M4 (cartesian controllers), M5
(evaluation harness). Every sim task must pass on both `ur5e` and
`ur15`. Real UR15 is explicitly out of scope.

## Last completed tasks

- **M2 baseline rosbags + manifests.** Added
  `scripts/record_crisp_baseline.sh <role> <robot> [duration_s]` which,
  for one role × arm combination, clears stale sim state, launches the
  MuJoCo sim (`scripts/launch_sim.sh <robot> effort`), waits for
  `joint_state_broadcaster` + `forward_effort_controller` active,
  launches `crisp_bringup.launch.py` with the matching `mode:=`, waits
  for the role controller to become the sole active effort commander,
  optionally publishes `/target_joint` at the current joint config for
  the joint role (the cartesian and gravity roles capture their hold
  target on activation), records `/joint_states`, the role's `tau_d`,
  and (for joint) `/target_joint` to an SQLite rosbag for
  `duration_s` seconds (default 10), and emits a YAML manifest
  describing role, controller name, robot, duration, bag path,
  bring-up launch args, recorded topics, per-topic message counts, and
  a pointer back to the recorder script. The bag payload is gitignored
  via new `evaluation/baselines/**/*.bag/` +
  `evaluation/baselines/**/*.mcap/` patterns; only the manifest is
  committed. `scripts/record_all_crisp_baselines.sh` iterates the 3×2
  grid serially (one sim at a time) and continues on failure. Ran it
  clean end-to-end in ~4m36s on a warm machine; produced 6 manifests
  with 3500–4600 `/joint_states` messages each (sim rate ~350–460 Hz
  over the 10 s window). No `tau_d` messages because the CRISP
  controllers do not publish `tau_d` unless introspection is enabled;
  the manifests list the topic but `bag_contents` only shows topics
  that actually received messages, matching the bag's own
  `metadata.yaml`. Full test gate (`scripts/run_tests.sh`) green
  (8/8, ~4:15) after the change — the recorder is a one-shot tool and
  is intentionally kept out of `run_tests.sh`. With this the last
  unchecked M2 item in `docs/ROADMAP.md` is ticked and the milestone
  is closed.
- **Sync `docs/ROADMAP.md` M2 ticks with reality.** See prior STATUS.
- **De-flake integration `/joint_states` sampling.** See prior STATUS.
- **Controller 3 (crisp gravity compensation) on ur5e and ur15.** See
  prior STATUS.
- **Controller 2 (crisp cartesian impedance) on ur5e and ur15.** See
  prior STATUS.
- **Controller 1 (crisp joint impedance) on ur5e and ur15.** See prior
  STATUS.

## Next task (agent should pick this up)

**M3 — our own simplified joint impedance controller.** First M3 item
on the ROADMAP: "Study the crisp joint impedance implementation we got
running in M2. Append a `DECISIONS.md` entry enumerating what to keep
vs drop (gravity comp, nullspace, friction comp, command
interpolation, etc.)." That is a decision-only task (no code) and is
the natural entry point — the code pieces (new package skeleton,
control-law math, integration tests) follow the ADR. The bring-up and
test shape from `test_crisp_joint_impedance.py` and
`bringup/launch/crisp_bringup.launch.py` will carry over directly.

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
  ×{ur5e, ur15}) — all green in ~4:15.
- `scripts/record_crisp_baseline.sh` is a one-shot baseline recorder,
  NOT part of `run_tests.sh`. Re-run it manually when baselines need
  refreshing (e.g. after a gain retune or a controller change); it
  rewrites the matching manifest in place.

## Blockers / open questions for operator

None currently blocking. Informational:

- ROS distro is effectively pinned to Humble (system install); formalise
  via ADR if/when a second distro becomes a candidate.
- Dashboard opens at `http://localhost:8000`; rosbridge at
  `ws://localhost:9090`. Both are pkill'd + port-cleared by
  `scripts/kill_sim.sh`.

## Recent commits

Run `git log --oneline -n 20` for the live list.
