# crisp_controllers — quick reference

Short in-tree summary of the controllers shipped by
`third_party/crisp_controllers`. Used as the entry point for M2 work. Not a
substitute for the upstream docs at
<https://utiasdsl.github.io/crisp_controllers/> — read those for theory and
design rationale.

Submodule is **read-only**. Everything below is observed from the source at
`third_party/crisp_controllers/` (plugin xml, yaml param schemas, `src/`).

## Plugin classes

The plugin library (`third_party/crisp_controllers/crisp_controllers.xml`)
exports four classes, all `controller_interface::ControllerInterface`:

| Plugin name                              | Role                                                |
|------------------------------------------|-----------------------------------------------------|
| `crisp_controllers/CartesianController`  | Torque-based impedance / OSC controller             |
| `crisp_controllers/TorqueFeedbackController` | External-torque feedback PD + friction comp     |
| `crisp_controllers/PoseBroadcaster`      | Publishes end-effector pose as `PoseStamped`        |
| `crisp_controllers/TwistBroadcaster`     | Publishes end-effector twist as `TwistStamped`      |

## The "three impedance controllers"

Despite the name, CRISP ships **one** controller class that is used in three
distinct roles by configuring it differently. This is the pattern followed in
the upstream per-robot bring-ups (see
`third_party/crisp_controllers/docs/new_robot_setup.md`): the same
`crisp_controllers/CartesianController` plugin is instantiated three times by
`controller_manager` under three different controller names, each with its
own YAML section.

All three roles share:

- **Command interface:** `<joint>/effort` for every joint in `joints`.
- **State interfaces:** `<joint>/position` and `<joint>/velocity`.
- **Requires:** the `robot_description` parameter on
  `/robot_state_publisher` (pinocchio builds the model from that URDF).
- **Subscribes:**
  - `target_pose`  (`geometry_msgs/PoseStamped`)   — Cartesian target.
  - `target_joint` (`sensor_msgs/JointState`)      — nullspace / joint target.
  - `target_wrench` (`geometry_msgs/WrenchStamped`) — feed-forward wrench.
- **Publishes:** `~/tau_d` (`sensor_msgs/JointState`) with the commanded
  torques, and introspection topics when `enable_introspection: true`.

| Role                          | Typical task gains        | Nullspace stiffness | Extras used                                   |
|-------------------------------|---------------------------|---------------------|-----------------------------------------------|
| `cartesian_impedance_controller` | `k_pos_* ≈ 400–500`, `k_rot_* ≈ 30` | > 0               | `use_coriolis_compensation`, `use_local_jacobian` |
| `joint_impedance_controller`     | all `k_pos_*`, `k_rot_*` = 0       | > 0, `projector_type: none` | `use_friction`, `use_coriolis_compensation` |
| `gravity_compensation`           | `k_pos_* = 0`, `k_rot_* ≈ 30` (or 0) | 0                 | `use_coriolis_compensation`, `use_friction`   |

In short: the same plugin behaves as a Cartesian impedance controller, a
joint-space impedance controller, or a pure model-compensation passthrough,
depending on which stiffness channels are non-zero and whether the
`projector_type` is `none` (direct joint control) or `kinematic`/`dynamic`
(nullspace projected behind a Cartesian task).

A fourth mode — **Operational Space Control** — is enabled by the same
plugin when `use_operational_space: true`; it is a different control law
under the same Cartesian-target API, useful only with proper inertia
compensation.

## Key `CartesianController` parameters

Full schema: `third_party/crisp_controllers/src/cartesian_controller.yaml`.
Highlights relevant to UR5e/UR15 bring-up:

- `joints` (`string[]`, required) — ordered list; must match the
  controller_manager's joint order for the arm.
- `end_effector_frame` (`string`, required) — TF frame of the tool flange
  (e.g. `tool0` for UR).
- `base_frame` (`string`, default `""`) — reference frame for Cartesian
  targets; leave blank for world or set to `base` for base-relative.
- `use_operational_space` (`bool`, default `false`) — switch between
  Cartesian impedance and OSC.
- `task.k_{pos,rot}_{x,y,z}` and matching `d_*` — Cartesian stiffness and
  damping per axis. Negative damping → auto `2·sqrt(k)`.
- `nullspace.stiffness`, `nullspace.damping`, `nullspace.projector_type`
  (`kinematic|dynamic|none`), `nullspace.weights.<joint>` — joint-level PD
  behind the Cartesian task. `projector_type: none` turns it into a direct
  joint PD, which is how `joint_impedance_controller` is built.
- Safety: `limit_error`, `limit_torques`, `max_delta_tau`,
  `joint_limit_repulsion.{enabled, safe_range, max_torque}`.
- Extras: `use_friction`, `use_coriolis_compensation`,
  `use_gravity_compensation`, `use_local_jacobian`.
- Logging: `log.*` flags (off by default).
- Introspection: `enable_introspection` (off by default).

## `TorqueFeedbackController`

Separate plugin (`crisp_controllers/TorqueFeedbackController`). Reads
`<joint>/{position,velocity,effort}` and commands `<joint>/effort`. Uses a
PD law on external torques plus friction compensation and a nullspace term
to hold joint positions. Intended for "back-drive / follow" style use, not
for direct trajectory tracking. Param schema in
`third_party/crisp_controllers/src/torque_feedback_controller.yaml`.

## Required ros2_control setup for UR sim

For all CartesianController roles and for TorqueFeedbackController the arm
must expose:

- `<joint>/effort` command interface → already active in
  `ur_simulator` MuJoCo effort mode (`forward_effort_controller` is active
  by default; see `scripts/launch_sim.sh`).
- `<joint>/position` and `<joint>/velocity` state interfaces → both
  present in the sim's joint interfaces.

TorqueFeedbackController additionally needs the `<joint>/effort` **state**
interface; `ur_simulator` exposes it via MuJoCo, so no sim changes are
required for M2.

## Bring-up pattern we will use

For each M2 rollout (one controller role × `{ur5e, ur15}`):

1. One YAML under `bringup/config/crisp_<role>.<arm>.yaml` carrying the
   `controller_manager` block plus the per-role parameter block, with
   `joints: [<arm>_shoulder_pan_joint, …]` matching the sim.
2. Launch file under `bringup/launch/crisp_bringup.launch.py` that
   accepts `robot:={ur5e,ur15}` and `mode:={cartesian,joint,gravity}`,
   wraps `scripts/launch_sim.sh <robot> effort`, loads the matching YAML
   via `controller_manager`, and spawns the named controller `--inactive`
   then activates it once `forward_effort_controller` is deactivated
   (only one effort commander at a time).
3. Integration test under `tests/integration/test_crisp_<role>.py`,
   parametrised over `{ur5e, ur15}`, that publishes a small regulation
   target and asserts bounded tracking error on `/joint_states` within a
   fixed window.

## References

- Plugin manifest: `third_party/crisp_controllers/crisp_controllers.xml`.
- Param schemas: `third_party/crisp_controllers/src/*.yaml`.
- Upstream setup guide: `third_party/crisp_controllers/docs/new_robot_setup.md`.
- Control-law math: `third_party/crisp_controllers/docs/getting_started_controller_details.md`.
