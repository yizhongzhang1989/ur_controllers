# Status

_Last updated: 2026-04-23 (M3 kicked off: ADR-0008 scopes the simplified
joint impedance controller — keep vs drop split for what we take from
crisp's joint-impedance role. Next: create
`src/simple_joint_impedance_controller/` package skeleton)._

## Current milestone

**M3 — our own simplified joint impedance controller: IN PROGRESS.**
M0–M2 are done. The first M3 item ("Study crisp joint impedance and
record keep-vs-drop in DECISIONS.md") is now ticked in
`docs/ROADMAP.md`; the choice is captured as **ADR-0008** in
`docs/DECISIONS.md`. Summary of the ADR: keep a minimal joint-space
PD (`tau = K (q_d - q) - D qdot`) with per-joint gains, torque and
torque-rate saturation, `target_joint` subscription seeded to the
measured `q` on activation, URDF-limit clamping on `q_d`, a `~/tau_d`
diagnostics publisher, and parameters via
`generate_parameter_library`; drop the Cartesian/OSC branch, pinocchio
and all model-based terms (gravity, Coriolis, nullspace projectors,
Jacobian), Franka-tuned friction model, EMA filters, soft
joint-limit-repulsion, per-axis error clipping, noise injection,
logging / introspection knobs, and the `stop_commands` flag. Gravity
compensation is explicitly flagged as the first extension to consider
if the integration test can't hold against gravity on either arm; if
so, it lands as a follow-up ADR, not silently. Priority order remains:
M3 → M4 (cartesian controllers) → M5 (evaluation harness). M-REAL
stays out of scope.

## Last completed tasks

- **M3 — ADR-0008: scope the simplified joint impedance controller.**
  Read through `third_party/crisp_controllers/src/cartesian_controller.{cpp,yaml}`
  (~750 + ~320 lines) and
  `bringup/config/crisp_joint_impedance.{ur5e,ur15}.yaml`, then appended
  **ADR-0008** to `docs/DECISIONS.md`. The ADR enumerates a concrete
  "keep" list (9 items: pure joint-space PD with per-joint diagonal `K`
  and `D`, auto-damping when `D[i] < 0`, `position+velocity` state
  interfaces, `effort` command interface, `~/target_joint` subscriber
  seeded to the measured `q` on activation, absolute torque saturation,
  torque rate saturation, `target_joint` validation with URDF-limit
  clamping on `q_d`, `~/tau_d` diagnostics, and
  `generate_parameter_library` parameters) and a "drop" list (10 items:
  Cartesian/OSC task branch, pinocchio-based gravity / Coriolis /
  nullspace / Jacobian machinery, Franka-tuned 7-vector friction model,
  EMA filters, soft joint-limit repulsion, per-axis error clip,
  noise injection, log / introspection flags, `stop_commands`, and
  `TorqueFeedbackController` / broadcaster plugins). Tests plan is also
  stated: gtest unit tests on the PD + saturation math with no ROS,
  plus integration tests parametrised over `{ur5e, ur15}` re-using the
  same bounded-tracking-error thresholds as
  `tests/integration/test_crisp_joint_impedance.py` so the M5
  comparison is a direct swap. Ticked the corresponding ROADMAP bullet
  under M3. Docs-only change — no code added, no tests modified; full
  test gate (`scripts/run_tests.sh`) still green.
- **M2 baseline rosbags + manifests.** See prior STATUS.
- **Sync `docs/ROADMAP.md` M2 ticks with reality.** See prior STATUS.
- **De-flake integration `/joint_states` sampling.** See prior STATUS.
- **Controller 3 (crisp gravity compensation) on ur5e and ur15.** See
  prior STATUS.
- **Controller 2 (crisp cartesian impedance) on ur5e and ur15.** See
  prior STATUS.

## Next task (agent should pick this up)

**M3 — create `src/simple_joint_impedance_controller/` package
skeleton.** Next unchecked ROADMAP item under M3: an `ament_cmake`
package exporting a `controller_interface::ControllerInterface` plugin
class. Scaffold only for this iteration: `package.xml` with deps on
`controller_interface`, `hardware_interface`, `rclcpp_lifecycle`,
`realtime_tools`, `generate_parameter_library`, `sensor_msgs`, and
`pluginlib`; a `CMakeLists.txt` wiring pluginlib export + param lib
codegen; a minimal `simple_joint_impedance_controller.{hpp,cpp}` with
the `on_init/configure/activate/deactivate/update` skeleton returning
`OK`; a `src/simple_joint_impedance_controller.yaml` parameter schema
matching ADR-0008 ("keep" list); the pluginlib XML; and a first gtest
unit test covering the pure-math helpers that already exist in the
skeleton (even if just the per-joint `tau = K dq_err - D qdot` + clamp
helpers). After that iteration, the next steps are per ROADMAP: flesh
out the control law, integration tests parametrised over
`{ur5e, ur15}`, and a `simple_jimp_bringup.launch.py` mirroring
`bringup/launch/crisp_bringup.launch.py`.

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
- Gravity-comp hook for our simple joint impedance controller is
  **not** pre-approved — ADR-0008 flags it as the first extension to
  revisit only if the M3 integration test can't hold against gravity
  with pure PD on either arm. If that happens, a follow-up ADR is
  required before adding the hook.

## Recent commits

Run `git log --oneline -n 20` for the live list.
