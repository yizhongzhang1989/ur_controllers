# Status

_Last updated: 2026-04-23 (M3: created `src/simple_joint_impedance_controller/`
package skeleton — pluginlib-loadable `ControllerInterface` plugin with
`generate_parameter_library` schema (ADR-0008 keep-list), header-only
math helpers (`math.hpp`), and 14 gtest unit tests on the PD + saturation
+ clamp math. Also fixed `scripts/run_tests.sh` stage 2 to pass
`--packages-skip cartesian_controller_simulation cartesian_controller_tests`,
matching ADR-0005. Next: flesh out `update()` control-law wiring.)_

## Current milestone

**M3 — our own simplified joint impedance controller: IN PROGRESS.**
M0–M2 done; ADR-0008 fixes the scope. This iteration adds the package
skeleton: `src/simple_joint_impedance_controller/` with `package.xml`,
`CMakeLists.txt` (pluginlib export + `generate_parameter_library`
codegen), `include/simple_joint_impedance_controller/{math.hpp,
simple_joint_impedance_controller.hpp}`, `src/simple_joint_impedance_controller.cpp`
(no-op `update()` returning `OK`), `src/simple_joint_impedance_controller.yaml`
(ADR-0008 keep-list params: `joints`, `k`, `d`, `tau_max`,
`max_delta_tau`, `command_topic`, `diagnostics_topic`),
`simple_joint_impedance_controller.xml` pluginlib manifest, and
`tests/test_math.cpp` with 14 gtest cases covering
`auto_fill_critical_damping`, `clamp_to_limits`, `compute_pd_torque`,
`saturate_torque_rate`, and `saturate_torque_abs` — each tested for
happy-path correctness, length/bound validation, and rejection on
invalid inputs. Priority order unchanged: M3 → M4 → M5; M-REAL out of
scope.

## Last completed tasks

- **M3 — `simple_joint_impedance_controller` package skeleton.** Ninth
  package in the workspace; builds cleanly via the standard
  `colcon build --packages-skip cartesian_controller_simulation
  cartesian_controller_tests` invocation. `test_math` runs in <0.1 s
  with 14 passing assertions. Also switched
  `scripts/run_tests.sh` stage 2 to pass `--packages-skip` (previously
  stale `build/cartesian_controller_{simulation,tests}` dirs caused
  `colcon test` to fail once the new package introduced a testable
  target). Full `scripts/run_tests.sh` now exits 0 in ~4:10 (14 gtest
  + 5 crisp_controllers unit tests + 8 integration tests).
- **M3 — ADR-0008: scope the simplified joint impedance controller.** See
  prior STATUS.
- **M2 baseline rosbags + manifests.** See prior STATUS.
- **Sync `docs/ROADMAP.md` M2 ticks with reality.** See prior STATUS.
- **De-flake integration `/joint_states` sampling.** See prior STATUS.

## Next task (agent should pick this up)

**M3 — flesh out the control law in `update()`.** Next unchecked ROADMAP
item under M3 ("Control law baseline: `tau = K (q_d - q) - D * qdot`,
with torque saturation and safe defaults"). The skeleton in
`src/simple_joint_impedance_controller/src/simple_joint_impedance_controller.cpp`
currently returns `OK` without touching command interfaces. Wire up:
(a) in `on_activate`, seed `q_d` to the currently measured `q` (sole
effort commander while active — mirrors ADR-0007 for crisp and what the
integration test will assume); (b) in `update`, read the
`position`/`velocity` state interfaces in the order defined by
`state_interface_configuration()`, apply
`compute_pd_torque` + `saturate_torque_rate` + `saturate_torque_abs`
from `math.hpp`, and write the result to the `<joint>/effort` command
interfaces; (c) subscribe to `params_.command_topic` as
`sensor_msgs/JointState`, copy the `position` field (after length/name
validation + `clamp_to_limits` against URDF limits) into `q_d`; (d)
publish `tau_d` on `params_.diagnostics_topic` each cycle. Auto-fill
critical damping (`auto_fill_critical_damping`) once at `on_activate`.
After the control law compiles and exists, the next steps per ROADMAP
are: integration tests `tests/integration/test_simple_jimp_*.py`
parametrised over `{ur5e, ur15}` mirroring
`tests/integration/test_crisp_joint_impedance.py`, and a
`bringup/launch/simple_jimp_bringup.launch.py` mirroring
`bringup/launch/crisp_bringup.launch.py` so the M5 comparison is a
single flag.

## Build status

- ROS 2 distro: **Humble** (Ubuntu 22.04, matches system install).
- `colcon build --symlink-install --base-paths src third_party
  --packages-skip cartesian_controller_simulation cartesian_controller_tests`
  now produces **10** packages successfully:
  - 6 from `cartesian_controllers`
  - `crisp_controllers`
  - `ur_sim_config`
  - `ur_simulation_gz`
  - `simple_joint_impedance_controller` (new)
- Skipped: `cartesian_controller_simulation`, `cartesian_controller_tests`
  (ADR-0005). `scripts/run_tests.sh` stage 2 now skips the same pair.

## Test status

- `scripts/run_tests.sh` runs unit → colcon test → integration.
- Stage 2 colcon test now `--packages-skip` cartesian_controller_simulation
  + cartesian_controller_tests (stale build dirs that never build).
- Integration stage sources `/opt/ros/humble/setup.bash` and
  `install/setup.bash` before pytest.
- Test counts: 14 new `test_math` gtests (simple_joint_impedance_controller)
  + existing crisp_controllers gtests + 8 integration tests
  (sim smoke + 3 crisp roles, each ×{ur5e, ur15}). All green in ~4:10.
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
