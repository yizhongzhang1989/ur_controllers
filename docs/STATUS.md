# Status

_Last updated: 2026-04-23 (feat(m3): flesh out simple_joint_impedance
control law — `update()` now reads state, applies PD + rate + abs
torque saturation, clamps targets into URDF limits, writes
`<joint>/effort`, and publishes `~/tau_d`. Gate green in ~3:50
with 22 gtests + 5 crisp_controllers gtests + 8 integration tests.)_

## Current milestone

**M3 — our own simplified joint impedance controller: IN PROGRESS.**
M0–M2 done. ADR-0008 fixes the feature scope; previous iteration
landed the package skeleton. This iteration implements the control
law baseline (ROADMAP M3 bullet 3):

- `on_configure` validates that `k`, `d`, `tau_max`, and `max_delta_tau`
  are the same length as `joints`; fetches `robot_description` from
  `/robot_state_publisher` (same pattern as crisp), parses it with
  `urdf::Model`, and caches per-joint `[q_lower, q_upper]` limits
  (treating CONTINUOUS joints as ±inf). Also creates the
  `sensor_msgs/JointState` subscriber on `params_.command_topic` with
  a `realtime_tools::RealtimeBuffer` and the `~/tau_d` publisher with
  `realtime_tools::RealtimePublisher`.
- `on_activate` refreshes gains, runs `auto_fill_critical_damping`
  (negative `d[i]` → `2*sqrt(k[i])`), zero-inits `tau_prev_` and
  `q_d_`, arms `seed_on_first_update_`, and drops any stale buffered
  target. `on_deactivate` zeroes the effort command before handing
  back interfaces (ADR-0007 clean hand-off).
- `update()` pipeline: read `<joint>/{position, velocity}` in the
  order we declared; on the very first cycle copy measured `q` into
  `q_d` (hold-position seed, same contract as crisp's joint role);
  non-blocking `readFromRT()` for a new target, validated via the new
  pure helper `validate_target_joint_state` (length + strict
  order-matching names + finiteness) then clamped by `clamp_to_limits`
  into the URDF limits — malformed messages are throttled-warned and
  the previous `q_d` is held; `compute_pd_torque` →
  `saturate_torque_rate` → `saturate_torque_abs` → write to
  `<joint>/effort` and publish `~/tau_d` via `RealtimePublisher`.

Tests added: 8 new gtests for `validate_target_joint_state` covering
position-only-without-names, position+velocity, name reorder reject,
position/velocity length-mismatch reject, non-finite reject on both
position and velocity, and empty-expected-joints reject. Total
`test_math` suite now 22 cases, all green.

CMake/package: added `urdf` to `find_package`/`ament_target_dependencies`/
`ament_export_dependencies` and to `<depend>` in `package.xml`. Switched
from deprecated `realtime_tools/*.h` to `realtime_tools/*.hpp` headers.

## Last completed tasks

- **M3 — control law baseline in `simple_joint_impedance_controller`.**
  Replaces the no-op skeleton `update()` with the full PD + saturation
  pipeline described above. ~280 lines of C++ (+header changes) and
  87 lines of new unit tests. Single commit, still ≈200 lines of
  controller code (ADR-0008 budget).
- **Fix: strictly serialise MuJoCo controller spawners
  (`third_party/ur_simulator`).** Previous STATUS entry retained.
- **M3 — `simple_joint_impedance_controller` package skeleton.** See
  prior STATUS.
- **M3 — ADR-0008: scope the simplified joint impedance controller.** See
  prior STATUS.
- **M2 baseline rosbags + manifests.** See prior STATUS.
- **Sync `docs/ROADMAP.md` M2 ticks with reality.** See prior STATUS.

## Next task (agent should pick this up)

**M3 — integration tests `tests/integration/test_simple_jimp_*.py`
parametrised over `{ur5e, ur15}`.** Mirror
`tests/integration/test_crisp_joint_impedance.py`: bring up sim,
switch from `forward_effort_controller` to
`simple_joint_impedance_controller` (ADR-0007 pattern), wait for
`/joint_states`, publish a small regulation target on
`/simple_joint_impedance_controller/target_joint`, assert bounded
tracking error within a fixed window. Also needed: a
`bringup/launch/simple_jimp_bringup.launch.py` mirroring
`bringup/launch/crisp_bringup.launch.py`, plus
`bringup/config/simple_joint_impedance.{ur5e,ur15}.yaml` with K/D/
tau_max/max_delta_tau tuned to hold against gravity on each arm with
pure PD. Per ADR-0008, if regulation cannot hold on either arm with
any reasonable PD-only gains, the gravity-comp hook becomes a
follow-up item with its own ADR before it is added.

## Build status

- ROS 2 distro: **Humble** (Ubuntu 22.04, matches system install).
- `colcon build --symlink-install --base-paths src third_party
  --packages-skip cartesian_controller_simulation cartesian_controller_tests`
  still produces **10** packages successfully:
  - 6 from `cartesian_controllers`
  - `crisp_controllers`
  - `ur_sim_config`
  - `ur_simulation_gz`
  - `simple_joint_impedance_controller` (now with real control law;
    adds `urdf` to deps)
- Skipped: `cartesian_controller_simulation`, `cartesian_controller_tests`
  (ADR-0005). `scripts/run_tests.sh` stage 2 skips the same pair.

## Test status

- `scripts/run_tests.sh` runs unit → colcon test → integration. Green
  in ~3:50 this iteration.
- Test counts: **22** `test_math` gtests (simple_joint_impedance_controller,
  up from 14) + 5 `crisp_controllers` gtests + 8 integration tests
  (sim smoke + 3 crisp roles, each ×{ur5e, ur15}).
- Integration stage sources `/opt/ros/humble/setup.bash` and
  `install/setup.bash` before pytest.
- `scripts/record_crisp_baseline.sh` is a one-shot baseline recorder,
  NOT part of `run_tests.sh`.

## Blockers / open questions for operator

None currently blocking. Informational:

- ROS distro is effectively pinned to Humble (system install); formalise
  via ADR if/when a second distro becomes a candidate.
- Dashboard opens at `http://localhost:8000`; rosbridge at
  `ws://localhost:9090`. Both are pkill'd + port-cleared by
  `scripts/kill_sim.sh`.
- Gravity-comp hook for our simple joint impedance controller is
  **not** pre-approved — ADR-0008 flags it as the first extension to
  revisit only if the M3 integration test can't hold against gravity
  with pure PD on either arm. If that happens, a follow-up ADR is
  required before adding the hook.

## Recent commits

Run `git log --oneline -n 20` for the live list.
