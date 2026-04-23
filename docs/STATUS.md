# Status

_Last updated: 2026-04-23 (feat(m3): wire `simple_joint_impedance_controller`
into sim via `simple_jimp_bringup.launch.py` + per-arm configs, add
`{ur5e, ur15}` regulation integration test. Gate green in ~4:30 with 22
`test_math` gtests + 5 `crisp_controllers` gtests + **10** integration
tests.)_

## Current milestone

**M3 — our own simplified joint impedance controller: IN PROGRESS.**
M0–M2 done. ADR-0008 fixes the feature scope; the control law landed
last iteration. This iteration closes the last two open M3 bullets
(launch + integration tests) and leaves only the `generate_parameter_
library` unit-test wiring bullet as explicitly optional polish — the
parameters are already generated and exercised end-to-end by the
integration tests below.

Previous iteration notes (control law baseline, still current):

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

- **M3 — sim bring-up + regulation integration test for
  `simple_joint_impedance_controller` on `{ur5e, ur15}`.** Added
  `bringup/config/simple_joint_impedance.{ur5e,ur15}.yaml` with per-arm
  K/D/tau_max/max_delta_tau tuned so pure PD (no gravity comp, per
  ADR-0008) holds pose within the same 0.15 rad tolerance used by the
  crisp joint-impedance test. UR15 stiffness is ~3× UR5e's to counter
  the larger gravity torques on shoulder_lift / elbow. Added
  `bringup/launch/simple_jimp_bringup.launch.py` mirroring
  `crisp_bringup.launch.py` (spawn `--inactive`, then
  `switch_controllers --strict` from `forward_effort_controller` to
  our controller, ADR-0007). Added
  `tests/integration/test_simple_jimp_regulation.py` parametrised over
  `{ur5e, ur15}`, structurally identical to
  `tests/integration/test_crisp_joint_impedance.py` so the M5
  comparison harness can flip between controllers with a single flag.
  All 10 integration tests pass (sim smoke ×2 + 3 crisp roles ×2 + our
  controller ×2); `scripts/run_tests.sh` green in ~4:30. Gravity-comp
  hook remains unused — pure PD holds both arms within tolerance.
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

**M3 done — move on to M4 (`cartesian_controllers` in sim).** All M3
ROADMAP bullets the agent can close alone are now ticked: package
skeleton, control-law baseline, `generate_parameter_library` schema,
unit tests on the math, integration tests on both arms, matching
launch file. The remaining `parameters via generate_parameter_library`
and "unit tests for the control-law math" bullets are already satisfied
by `src/simple_joint_impedance_controller/{src/*.yaml, tests/test_math.cpp}`
and the 22-case gtest suite exercised each run. Tick those in
`docs/ROADMAP.md` as the first step of the next iteration, then start
M4 bullet 1 (build `cartesian_controllers` from submodule — already
builds clean in the workspace per ADR-0005, so this mostly means
picking a primary cartesian mode and wiring the first per-arm YAML).

## Build status

- ROS 2 distro: **Humble** (Ubuntu 22.04, matches system install).
- `colcon build --symlink-install --base-paths src third_party
  --packages-skip cartesian_controller_simulation cartesian_controller_tests`
  still produces **10** packages successfully:
  - 6 from `cartesian_controllers`
  - `crisp_controllers`
  - `ur_sim_config`
  - `ur_simulation_gz`
  - `simple_joint_impedance_controller` (control law + sim bring-up;
    deps include `urdf`)
- Skipped: `cartesian_controller_simulation`, `cartesian_controller_tests`
  (ADR-0005). `scripts/run_tests.sh` stage 2 skips the same pair.
- Bring-up assets for our controller: `bringup/launch/simple_jimp_bringup
  .launch.py` + `bringup/config/simple_joint_impedance.{ur5e,ur15}.yaml`.
  Launch pattern matches `crisp_bringup.launch.py`.

## Test status

- `scripts/run_tests.sh` runs unit → colcon test → integration. Green
  in ~4:30 this iteration (extra ~40 s for the two new simple_jimp
  integration tests, which each cold-start MuJoCo).
- Test counts: **22** `test_math` gtests
  (simple_joint_impedance_controller) + 5 `crisp_controllers` gtests
  + **10** integration tests (sim smoke + 3 crisp roles + our
  simple_joint_impedance_controller, each ×{ur5e, ur15}).
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
  **not needed** for the M3 regulation test on either arm — pure PD
  with the per-arm gains in `bringup/config/simple_joint_impedance
  .{ur5e,ur15}.yaml` holds both arms within the same 0.15 rad
  tolerance used for the crisp joint-impedance test. ADR-0008's
  gravity-comp-only-if-needed escape hatch therefore remains unused;
  revisit only if a future scenario (dynamic targets, heavier payload)
  shows PD alone is insufficient.

## Recent commits

Run `git log --oneline -n 20` for the live list.
