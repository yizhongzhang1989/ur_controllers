# Status

_Last updated: 2026-04-23 (feat(m4): bring up cartesian_motion_controller
on ur5e and ur15; tick M4 bullets 2, 3, 4. Full test gate green in
~5:25.)_

## Current milestone

**M4 — make `cartesian_controllers` work in sim: IN PROGRESS.** M0–M3
done. M4 bullets 1–4 done this iteration (primary mode
`cartesian_motion_controller` brought up on both arms with regulation
integration tests). Only the optional bullet 5 (second cartesian mode)
remains, and it is blocked on the sim gaining an `ft_sensor_ref_link`
plus an `~/ft_sensor_wrench` publisher on the `auto_dev` branch of the
sim submodule — same open question already flagged below.

Key observations from this iteration (relevant for future work):

- **`robot_description` propagation gotcha.** On Humble 2.53, the
  controller manager propagates its own `robot_description` param
  down to newly-loaded controllers only when the param was given to
  the CM at node construction. Our sim's `ur_sim_mujoco.launch.py`
  instead relies on the `/robot_description` topic path (remapping
  `~/robot_description` → `/robot_description`), which populates the
  CM's internal copy but does **not** push it to each controller.
  `CartesianControllerBase::on_configure` reads from its own node
  param so it sees `robot_description = ""` and aborts. Crisp dodges
  this by querying `/controller_manager` directly via a parameters
  client (see `third_party/crisp_controllers/src/cartesian_controller.cpp:242`).
  We cannot patch the read-only `cartesian_controllers` submodule
  (ADR-0003), so the launch fetches `robot_description` from
  `/robot_state_publisher` via rclpy and passes it to the spawner as
  a second `--param-file` (see `bringup/launch/cartesian_bringup.launch.py`).
  If/when a future controller with the same "reads from own node
  param" pattern lands, reuse this approach or fix the sim to pass
  `robot_description` as a CM-node param on `auto_dev`.

## Last completed tasks

- **M4 bullets 2 + 3 + 4: `cartesian_motion_controller` brought up on
  ur5e and ur15 with regulation integration test.** New files:
  `bringup/config/cartesian_motion.{ur5e,ur15}.yaml`,
  `bringup/launch/cartesian_bringup.launch.py`,
  `tests/integration/test_cartesian_motion.py`. Sim runs in position
  mode (`scripts/launch_sim.sh <robot> position`); bring-up spawns the
  controller `--inactive` and atomically swaps
  `joint_trajectory_controller` → `cartesian_motion_controller` via
  `ros2 control switch_controllers --strict` (same pattern as ADR-0007
  but in position interfaces, not effort). Test asserts the swap
  landed and joints stay within 0.15 rad of the hold pose for 5 s —
  the same threshold used by the crisp and simple_jimp regulation
  tests so the M5 comparison harness can flip between controllers
  with one flag.
- **M4 kick-off: `docs/cartesian_controllers.md` reference + tick M4
  bullet 1 (build).** See prior STATUS.
- **M3 — sim bring-up + regulation integration test for
  `simple_joint_impedance_controller` on `{ur5e, ur15}`.** See prior
  STATUS.
- **M3 — control law baseline in `simple_joint_impedance_controller`.**
  See prior STATUS.
- **Fix: strictly serialise MuJoCo controller spawners
  (`third_party/ur_simulator`).** See prior STATUS.
- **M3 — `simple_joint_impedance_controller` package skeleton.** See
  prior STATUS.
- **M3 — ADR-0008: scope the simplified joint impedance controller.**
  See prior STATUS.
- **M2 baseline rosbags + manifests.** See prior STATUS.

## Next task (agent should pick this up)

**M4 optional bullet 5: wire a second cartesian mode if time permits.**
Candidates are `cartesian_compliance_controller` and
`cartesian_force_controller`. Both require an `ft_sensor_ref_link` on
the URDF chain between `robot_base_link` and `end_effector_link` plus
an `~/ft_sensor_wrench` publisher. The UR sim does not currently
expose an F/T sensor link, so this bullet first needs a sim-side
`auto_dev` patch (xacro addition of a zero-offset sensor frame at
`tool0` or just before `flange`, plus either a stub F/T publisher or
wiring the MuJoCo contact-force sensor). **Human gate per AGENTS.md
§7** — the patch touches the simulator's URDF and sensor publishing
pipeline, so the operator should confirm they want the sim to grow an
F/T sensor before we enter this bullet.

If bullet 5 is deferred, the next highest-value item is **M5 bullet 1:
draft the `evaluation/scenarios/*.yaml` schema** (step, sine,
regulation, random waypoints). That is a pure-docs/spec task and does
not block on M4 bullet 5 landing — M5 will exercise the crisp + simple
joint-impedance + cartesian_motion controllers already wired today.

## Build status

- ROS 2 distro: **Humble** (Ubuntu 22.04, matches system install).
- `colcon build --symlink-install --base-paths src third_party
  --packages-skip cartesian_controller_simulation cartesian_controller_tests`
  produces **10** packages successfully, unchanged from prior
  iteration (no new packages added — this iteration only touched
  `bringup/` and `tests/integration/`).

## Test status

- `scripts/run_tests.sh` runs unit → colcon test → integration. Green
  in ~5:25 this iteration (12 integration tests: 10 prior + 2 new
  `test_cartesian_motion_regulation[{ur5e,ur15}]`).
- Test counts: **22** `test_math` gtests
  (simple_joint_impedance_controller) + 5 `crisp_controllers` gtests
  + **12** integration tests (sim smoke + 3 crisp roles + our
  simple_joint_impedance_controller + `cartesian_motion_controller`,
  each ×{ur5e, ur15}).

## Blockers / open questions for operator

None currently blocking. Informational:

- ROS distro pinned to Humble (system install); formalise via ADR if/
  when a second distro becomes a candidate.
- Dashboard opens at `http://localhost:8000`; rosbridge at
  `ws://localhost:9090`. Both are pkill'd + port-cleared by
  `scripts/kill_sim.sh`.
- The cartesian compliance and force controllers both need an
  `ft_sensor_ref_link` on the URDF chain plus an `~/ft_sensor_wrench`
  publisher. The UR sim does not currently expose an F/T sensor frame;
  those controllers therefore require a sim-side `auto_dev` patch
  before they can be brought up end-to-end. Now directly gating M4
  bullet 5 — surface to the operator when/if that bullet is picked up.
- Gravity-comp hook for `simple_joint_impedance_controller` remains
  unused (pure PD holds both arms within the M3 tolerance).

## Recent commits

Run `git log --oneline -n 20` for the live list.
