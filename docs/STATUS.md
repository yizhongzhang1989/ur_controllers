# Status

_Last updated: 2026-04-23 (docs(m4): enumerate cartesian_controllers
plugins in `docs/cartesian_controllers.md`; tick M4 bullet 1 (build).
Doc-only iteration — no code changes, test gate unchanged.)_

## Current milestone

**M4 — make `cartesian_controllers` work in sim: IN PROGRESS.** M0–M3
done. This iteration kicks M4 off the same way M2 started: with a short
in-tree reference doc (`docs/cartesian_controllers.md`) enumerating the
plugins shipped by the submodule so the next iteration can pick a
primary mode and wire its first per-arm YAML without re-doing the
investigation.

Key observations from that reference (relevant to the next iteration):

- Three plugin classes ship:
  `cartesian_motion_controller/CartesianMotionController`,
  `cartesian_compliance_controller/CartesianComplianceController`, and
  `cartesian_force_controller/CartesianForceController`. All derive from
  `cartesian_controller_base::CartesianControllerBase`.
- Command interfaces are **position** (and/or velocity), **not** effort.
  This is the key structural difference from crisp/our simple_jimp. In
  sim this means we run against `scripts/launch_sim.sh <robot> position`
  (not `effort`) and atomically swap
  `joint_trajectory_controller` → `<cartesian_controller>` via
  `ros2 control switch_controllers --strict`, mirroring the
  `forward_effort_controller` swap pattern from ADR-0007.
- `CartesianMotionController::on_activate` seeds
  `m_target_frame = m_current_frame` (motion controller `.cpp:91`), so
  the controller holds pose on activation — the first integration test
  can assert "arm doesn't diverge after activation" without needing a
  synthesised Cartesian target, analogous to the crisp joint-impedance
  regulation test.
- Compliance / force controllers need an `ft_sensor_ref_link` on the
  URDF chain + an `~/ft_sensor_wrench` publisher; the sim does not
  expose an F/T sensor frame today. Those are out of scope for M4
  bullet 1/2/3 and are candidates for the "optional second cartesian
  mode" roadmap bullet once the sim gains an F/T link on `auto_dev`.

## Last completed tasks

- **M4 kick-off: `docs/cartesian_controllers.md` reference + tick M4
  bullet 1 (build).** Same cadence as commit `7106265 docs(crisp):
  enumerate crisp_controllers plugins and impedance roles` at the
  start of M2. Records plugin names, command/state interfaces, target
  topics, and bring-up implications against the sim. M4 bullet 1 in
  `docs/ROADMAP.md` ticked off — `cartesian_controllers` already builds
  green in our workspace per ADR-0005 and stage 2 of
  `scripts/run_tests.sh`, nothing new to resolve. No code changes this
  iteration; test gate unchanged.
- **M3 — sim bring-up + regulation integration test for
  `simple_joint_impedance_controller` on `{ur5e, ur15}`.** See prior
  STATUS: per-arm YAMLs + `simple_jimp_bringup.launch.py` +
  `tests/integration/test_simple_jimp_regulation.py` (parametrised
  over both arms). 10 integration tests green in `scripts/run_tests.sh`.
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

**M4 bullet 2: pick the primary cartesian mode and write
`bringup/config/cartesian_motion.{ur5e,ur15}.yaml`.** Primary mode is
`cartesian_motion_controller/CartesianMotionController` (no F/T sensor
dependency, single `PoseStamped` target topic,
activation-auto-hold — see `docs/cartesian_controllers.md` §Scope for M4).
Configs should set `end_effector_link: tool0`, `robot_base_link:
base_link`, UR joint list, `command_interfaces: [position]`, conservative
`pd_gains` and `solver.iterations`. Both YAMLs can be nearly identical
(UR arms share joint names; tune gains if ur15 needs a larger
`error_scale`).

Then M4 bullet 3: `bringup/launch/cartesian_bringup.launch.py` mirroring
`crisp_bringup.launch.py` structure (spawn `--inactive`, then
`switch_controllers --strict` from `joint_trajectory_controller` to
`cartesian_motion_controller` since position-mode sim boots with JTC
active). IMPORTANT: unlike crisp, the cartesian controller does NOT
auto-fetch `robot_description` from `/robot_state_publisher` — the
launch file must feed it in via a `<param>` or the spawn's
`--param-file`, whichever pattern the upstream reference
`controller_manager.yaml` implies (see
`third_party/cartesian_controllers/cartesian_controller_simulation/config/controller_manager.yaml`).

Then M4 bullet 4: `tests/integration/test_cartesian_motion.py`
parametrised over `{ur5e, ur15}`, asserting (a) the
JTC→`cartesian_motion_controller` swap landed, and (b) joints stay
within the same 0.15 rad "regulation tolerance" window already used by
the crisp and simple_jimp tests — reusing that threshold lets the M5
comparison harness flip between controllers with one flag.

## Build status

- ROS 2 distro: **Humble** (Ubuntu 22.04, matches system install).
- `colcon build --symlink-install --base-paths src third_party
  --packages-skip cartesian_controller_simulation cartesian_controller_tests`
  produces **10** packages successfully, unchanged from prior iteration:
  - 6 from `cartesian_controllers` (base, motion, compliance, force,
    handles, utilities)
  - `crisp_controllers`
  - `ur_sim_config`
  - `ur_simulation_gz`
  - `simple_joint_impedance_controller`
- Skipped: `cartesian_controller_simulation`, `cartesian_controller_tests`
  (ADR-0005). `scripts/run_tests.sh` stage 2 skips the same pair.

## Test status

- `scripts/run_tests.sh` runs unit → colcon test → integration. Green
  in ~4:30 as of the prior iteration; no code changes this iteration so
  the gate is structurally unchanged (doc-only).
- Test counts (unchanged): **22** `test_math` gtests
  (simple_joint_impedance_controller) + 5 `crisp_controllers` gtests
  + **10** integration tests (sim smoke + 3 crisp roles + our
  simple_joint_impedance_controller, each ×{ur5e, ur15}).

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
  before they can be brought up end-to-end. Non-blocking for M4 bullets
  1–4 (primary mode is `cartesian_motion_controller`); surface for the
  operator only when/if the "optional second cartesian mode" bullet is
  picked up.
- Gravity-comp hook for `simple_joint_impedance_controller` remains
  unused (pure PD holds both arms within the M3 tolerance).

## Recent commits

Run `git log --oneline -n 20` for the live list.
