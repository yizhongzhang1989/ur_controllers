# cartesian_controllers — quick reference

Short in-tree summary of the controllers shipped by
`third_party/cartesian_controllers` (FZI `cartesian_controllers`). Entry
point for M4 work. Not a substitute for the upstream docs at
<https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers> —
read those for theory and design rationale.

Submodule is **read-only**. Everything below is observed from the source at
`third_party/cartesian_controllers/` (plugin xml files, `src/*.cpp`, and the
reference `cartesian_controller_simulation/config/controller_manager.yaml`).

## Packages and build skip list

`third_party/cartesian_controllers/` is a multi-package workspace. We build
the controller plugins; per ADR-0005 we always pass
`--packages-skip cartesian_controller_simulation cartesian_controller_tests`
to `colcon build` because those two packages are not useful in our pipeline
(one needs MuJoCo at a hard-coded prefix we don't ship, the other is a ROS 1
catkin package).

Packages that build in our workspace:

| Package                            | Provides                                                              |
|------------------------------------|-----------------------------------------------------------------------|
| `cartesian_controller_base`        | Shared `CartesianControllerBase` (kinematics, IK solver, target PD).  |
| `cartesian_motion_controller`      | `cartesian_motion_controller/CartesianMotionController` plugin.       |
| `cartesian_compliance_controller`  | `cartesian_compliance_controller/CartesianComplianceController` plugin. |
| `cartesian_force_controller`       | `cartesian_force_controller/CartesianForceController` plugin.         |
| `cartesian_controller_handles`     | `cartesian_controller_handles/MotionControlHandle` (RViz interactive marker). |
| `cartesian_controller_utilities`   | Stand-alone utility nodes (not a `ControllerInterface` plugin).       |

Build status in this workspace is confirmed green via
`scripts/run_tests.sh` (stage 2, colcon build) with the skip list above —
see ADR-0005 and `docs/STATUS.md` "Build status".

## Plugin classes

All three controller plugins derive from `cartesian_controller_base::
CartesianControllerBase` and are `controller_interface::ControllerInterface`:

| Plugin name                                                     | Intended use                                                                 |
|-----------------------------------------------------------------|------------------------------------------------------------------------------|
| `cartesian_motion_controller/CartesianMotionController`         | Contact-free Cartesian motion. Track a target pose; no force feedback.       |
| `cartesian_compliance_controller/CartesianComplianceController` | Cartesian motion with in-contact compliance from an F/T sensor.              |
| `cartesian_force_controller/CartesianForceController`           | Pure force / wrench tracking (teleoperation with contact).                   |

### Common interface contract (inherited from `CartesianControllerBase`)

- **State interfaces:** `<joint>/position` only
  (`third_party/cartesian_controllers/cartesian_controller_base/src/cartesian_controller_base.cpp:77`).
- **Command interfaces:** whatever the per-instance `command_interfaces`
  param declares — typically `position` (and/or `velocity`). This is the
  key difference from crisp: these are **not** effort controllers. They
  drive joint positions via an internal forward-dynamics IK solver.
- **Required params (all three):** `joints` (6 names), `robot_base_link`,
  `end_effector_link`, `command_interfaces`, plus `solver.*` and
  `pd_gains.*` blocks. `cartesian_compliance_controller` and
  `cartesian_force_controller` additionally require `ft_sensor_ref_link`.
- **No `robot_description` auto-fetch.** Unlike crisp, the controller reads
  `robot_description` from its own param namespace — the launch file must
  pipe it in (e.g. via `<Parameter name="robot_description"/>` from
  `/robot_state_publisher`).

### Per-plugin topics

| Plugin                              | Subscribes                                                    | Msg type                        |
|-------------------------------------|---------------------------------------------------------------|---------------------------------|
| `CartesianMotionController`         | `~/target_frame`                                              | `geometry_msgs/PoseStamped`     |
| `CartesianComplianceController`     | `~/target_frame` + `~/target_wrench` + `~/ft_sensor_wrench`   | `PoseStamped` / `WrenchStamped` |
| `CartesianForceController`          | `~/target_wrench` + `~/ft_sensor_wrench`                      | `WrenchStamped`                 |

`CartesianMotionController::on_activate` seeds `m_target_frame =
m_current_frame` (motion-controller `on_activate`, `.cpp:91`), so the
controller holds pose on activation without needing a target publisher
— convenient for integration-test "don't diverge" assertions.

## Bring-up implications against `ur_simulator`

`cartesian_motion_controller` and `cartesian_compliance_controller` both
take `position` command interfaces by default (see the reference
`controller_manager.yaml`). In our sim:

- **Position mode** (`scripts/launch_sim.sh <robot> position`) spawns
  `joint_trajectory_controller` **active** and every other position/
  velocity/effort forward controller as inactive (see
  `third_party/ur_simulator/src/ur_sim_config/launch/ur_sim_mujoco.launch.py`
  controller-activation block). Bringing up a cartesian controller must
  therefore swap `joint_trajectory_controller` → `<cartesian_controller>`
  atomically via `ros2 control switch_controllers --strict`, mirroring the
  `forward_effort_controller` swap pattern crisp uses (ADR-0007). The
  sim's `gravity_compensation.py` node is **not** started in position
  mode, so no extra effort commander is in the mix.
- **Effort mode** is not applicable: the cartesian controllers do not
  claim `<joint>/effort`, and activating them while `forward_effort_
  controller` holds the effort interfaces would silently not collide but
  would also have nothing to drive the arm's position. Use position mode.

`cartesian_force_controller` and `cartesian_compliance_controller`
additionally need a valid `ft_sensor_ref_link` on the URDF chain between
`robot_base_link` and `end_effector_link`, plus a publisher on
`~/ft_sensor_wrench`. The UR sim does not currently expose an F/T sensor
link — wiring those two controllers will require either a URDF patch on
the sim's `auto_dev` branch (add a zero-offset sensor frame) or a stub
publisher that feeds a zero wrench. Out of scope for the first M4
bullets; `cartesian_motion_controller` is the primary mode for M4.

## Scope for M4

The M4 roadmap asks for **one working task-space controller on both arms,
for later comparison**. Per the contract above, the primary mode is
`cartesian_motion_controller/CartesianMotionController` — no F/T sensor
dependency, only a single `PoseStamped` target topic, and the
`on_activate` auto-hold means the integration test can verify "the arm
doesn't diverge when activated" without needing to synthesise a
meaningful Cartesian target. Compliance / force controllers are
candidates for the optional "second cartesian mode" roadmap bullet, after
the sim gains an F/T sensor link on `auto_dev`.
