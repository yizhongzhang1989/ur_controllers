# Roadmap

Milestones are ordered. The agent works on the earliest milestone that has
unchecked items. Within a milestone, tasks are also ordered.

Legend: `[ ]` todo, `[~]` in progress, `[x]` done, `[!]` blocked (see STATUS.md).

Cross-cutting rules:

- **Every simulation task must pass on BOTH `ur5e` and `ur15`.** A task is
  not done until both arms run the same controller with identical metrics
  criteria (gains may differ, but scenarios and pass/fail thresholds are
  the same).
- **Real UR15 is out of scope right now.** Do not create or exercise any
  real-robot launch file or driver config in this phase. M-REAL below is a
  placeholder for later; the agent must not enter it.
- `third_party/ur_simulator` is ours — edit on its `auto_dev` branch
  whenever the simulator is missing something a controller task needs
  (e.g. UR15 description, better joint limits, effort interface fidelity).
  Bump the submodule pointer in a follow-up commit in this repo.
- `third_party/crisp_controllers` and `third_party/cartesian_controllers`
  are read-only. If they need patching, open an upstream issue/PR instead.

---

## M0 — Bootstrap

- [x] Create repo scaffolding (directories, docs, scripts).
- [x] Decide ROS 2 distro; record in `docs/DECISIONS.md` (Humble, ADR-0002).
- [x] Add git submodules under `third_party/`:
  - [x] `ur_simulator` — https://github.com/yizhongzhang1989/ur_simulator.git (tracking `auto_dev`)
  - [x] `crisp_controllers` — https://github.com/yizhongzhang1989/crisp_controllers.git (read-only)
  - [x] `cartesian_controllers` — https://github.com/yizhongzhang1989/cartesian_controllers.git (read-only)
- [x] Add top-level colcon workspace config (symlinks or `COLCON_IGNORE` markers).
      Done as `colcon_defaults.yaml` at the repo root — auto-loaded by
      `python3-colcon-defaults` from cwd, supplies `--symlink-install`,
      `--base-paths src third_party`, and the ADR-0005 `--packages-skip`
      list for both `build` and `test` verbs. Pinned by
      `tests/unit/test_colcon_defaults.py`.
- [x] Add `.pre-commit-config.yaml` (clang-format, ruff, trailing whitespace).
      Landed as `.pre-commit-config.yaml` + `.clang-format` (LLVM base) +
      `ruff.toml` (py310 target). Hook revs are pinned; `third_party/`,
      `build/`, `install/`, `log/` and the gitignored evaluation outputs
      are excluded. Tree was brought into compliance in the same commit
      (`pre-commit run --all-files` exits 0). Pinned by 12 unit tests in
      `tests/unit/test_pre_commit_config.py`.
- [x] Add CI workflow `.github/workflows/ci.yml`: build + unit tests headless.
      Two-job workflow (`lint` → `build`): the `lint` job runs
      `pre-commit run --all-files` on `ubuntu-22.04` (no ROS); the
      `build` job runs inside the `ros:humble-ros-base` container,
      `rosdep install`s with the ADR-0005 skip-keys, then
      `colcon build` (defaults file supplies the rest), then
      `scripts/run_tests.sh --unit-only`. Integration tests stay out
      of CI until a sim-in-CI story lands. Pinned by 12 unit tests in
      `tests/unit/test_ci_workflow.py`.
- [x] Write `scripts/setup_env.sh` to install ROS deps via `rosdep`.
      Idempotent three-stage wrapper: `apt-get install` of the README
      "Prerequisites" list + build/test tooling, `rosdep install` with
      the ADR-0005 `--skip-keys`, and `pip install --user pre-commit
      ruff` + `pre-commit install` for the local git hook. Flags:
      `--dry-run` (mirrors `scripts/run_tests.sh`), `--no-pre-commit`.
      Pinned by 26 unit tests in `tests/unit/test_setup_env.py`.

## M1 — Simulator brings up UR5e and UR15

Goal: a single launch file takes a `robot:=ur5e|ur15` arg and stands up the
sim with that arm. Both must work before M2 starts.

- [x] Confirm `ur_simulator` ships descriptions for both `ur5e` and `ur15`.
      System package `ros-humble-ur-description` covers both; the sim
      picks the arm via its `ur_type` config.
- [x] `bringup/launch/sim_bringup.launch.py` with `robot` arg; default
      `ur5e`. (Implemented as `scripts/launch_sim.sh <robot> <mode>`
      wrapping the sim's own `launch_all.sh`; a pure .launch.py wrapper
      can be added later if needed by ros2 launch composition.)
- [x] Verify `/joint_states` publishes at expected rate for both arms.
      Covered by `tests/integration/test_sim_smoke.py`.
- [x] Verify the command interfaces required by crisp's three impedance
      controllers (at minimum `effort`; `position` and `velocity` as
      available) are exposed for both arms. In MuJoCo effort mode the
      sim loads `forward_effort_controller` active and keeps position /
      velocity / trajectory controllers loaded-but-inactive for runtime
      switching.
- [x] Integration smoke test parametrised over `{ur5e, ur15}`: launch
      sim, wait for `/joint_states`, check controllers, check rosbridge
      port, check dashboard HTTP 200.

---

## M2 — Make `crisp_controllers` work in sim (PRIORITY 1)

Goal: the three impedance controllers shipped by `crisp_controllers` run
cleanly against the sim on both arms, with saved configs and a reproducible
launch flow. No modifications to the submodule — only our configs and
launches.

- [x] Build `crisp_controllers` from submodule; resolve deps via rosdep;
      document any missing-dep fixes in STATUS.md. (Built clean with
      `--packages-skip cartesian_controller_simulation
      cartesian_controller_tests`; see ADR-0005.)
- [x] Enumerate the three impedance controllers shipped by crisp; record
      their plugin names, command interfaces, and required params in
      `docs/crisp_controllers.md` (new, short reference file). Verified
      against `third_party/crisp_controllers/crisp_controllers.xml` (four
      plugin classes) and the three configuration roles of
      `CartesianController`.
- [x] Controller 1 (crisp joint impedance): write
      `bringup/config/crisp_joint_impedance.{ur5e,ur15}.yaml`, wire it into
      `bringup/launch/crisp_bringup.launch.py`, bring it up on ur5e.
- [x] Same controller, bring it up on ur15.
- [x] Controller 2 (second crisp impedance variant): same two-step rollout
      (ur5e, then ur15). Done as `cartesian_impedance_controller` role;
      configs `bringup/config/crisp_cartesian_impedance.{ur5e,ur15}.yaml`,
      test `tests/integration/test_crisp_cartesian_impedance.py`.
- [x] Controller 3 (third crisp impedance variant): same two-step rollout.
      Done as `gravity_compensation` role; configs
      `bringup/config/crisp_gravity_compensation.{ur5e,ur15}.yaml`, test
      `tests/integration/test_crisp_gravity_compensation.py`.
- [x] Integration tests under `tests/integration/test_crisp_*.py`: for each
      of the three controllers and each of `{ur5e, ur15}`, send a small
      regulation command and assert bounded tracking error within a fixed
      time window. Three test files, each parametrised over
      `{ur5e, ur15}`, all green in `scripts/run_tests.sh` (~3:20).
- [x] Record baseline rosbags under `evaluation/baselines/crisp/`
      (gitignored payload; commit a manifest `.yaml` of what was recorded).
      Done via `scripts/record_crisp_baseline.sh <role> <robot>` +
      `scripts/record_all_crisp_baselines.sh`; 6 manifests committed
      (`evaluation/baselines/crisp/<role>_<robot>.manifest.yaml`), bag
      payloads under `*.bag/` gitignored.

---

## M3 — Our own simplified joint impedance controller (PRIORITY 2)

Goal: a lightweight in-tree controller that we fully own, modelled on the
crisp joint impedance controller from M2 but stripped of features we don't
need. It must match crisp's behaviour on a baseline regulation scenario
within an agreed tolerance (set in M4).

- [x] Study the crisp joint impedance implementation we got running in M2.
      Append a `DECISIONS.md` entry enumerating what to keep vs drop
      (gravity comp, nullspace, friction comp, command interpolation, etc.).
      Done as **ADR-0008** in `docs/DECISIONS.md`.
- [x] Create package `src/simple_joint_impedance_controller/`
      (`controller_interface::ControllerInterface` plugin, `ament_cmake`).
- [x] Control law baseline: `tau = K (q_d - q) - D * qdot`, with torque
      saturation and safe defaults. Optional gravity-comp hook only if M2
      showed it is needed to match crisp. (Landed last iteration; this
      iteration confirms pure PD holds within tolerance on both arms —
      gravity-comp hook stays unused per ADR-0008.)
- [x] Parameters via `generate_parameter_library`: `joints`, `K`, `D`,
      `tau_max`, command topic, optional feature flags.
      (`src/simple_joint_impedance_controller/src/simple_joint_impedance_controller.yaml`,
      exercised by the integration tests below.)
- [x] Unit tests (gtest) for the control-law math, no ROS.
      (`src/simple_joint_impedance_controller/tests/test_math.cpp` — 22
      cases across PD, rate/abs saturation, critical-damping auto-fill,
      target validation, URDF-limit clamping.)
- [x] Integration tests under `tests/integration/test_simple_jimp_*.py`
      for `{ur5e, ur15}`: step + regulation, bounded error.
      (`tests/integration/test_simple_jimp_regulation.py`; 0.15 rad
      tolerance matching the crisp test.)
- [x] Update `bringup/launch/` with a `simple_jimp_bringup.launch.py`
      mirroring the crisp launch, so comparison later is a single flag.

---

## M4 — Make `cartesian_controllers` work in sim (PRIORITY 3)

Goal: at least one working task-space controller from `cartesian_controllers`
on both arms, for later comparison / use alongside our joint-space work.

- [x] Build `cartesian_controllers` from submodule; resolve deps.
      Green in our workspace with
      `--packages-skip cartesian_controller_simulation cartesian_controller_tests`
      (ADR-0005). Plugins enumerated in `docs/cartesian_controllers.md`;
      primary M4 mode is `cartesian_motion_controller`.
- [x] Pick a primary mode (e.g. cartesian motion + compliance) and write
      `bringup/config/cartesian_motion.{ur5e,ur15}.yaml`. Primary mode
      is `cartesian_motion_controller/CartesianMotionController`; both
      per-arm YAMLs share identical `pd_gains` + `solver` blocks (see
      the in-file notes for why the position-interface design makes a
      single gain set fit both arms).
- [x] Launch file `bringup/launch/cartesian_bringup.launch.py`; bring up on
      ur5e, then ur15. Swaps `joint_trajectory_controller` →
      `cartesian_motion_controller` via `switch_controllers --strict`.
      Fetches `robot_description` from `/robot_state_publisher` at
      launch time and feeds it to the spawner as a second
      `--param-file` — see STATUS "robot_description propagation
      gotcha" for why this is necessary on our Humble sim.
- [x] Integration tests under `tests/integration/test_cartesian_*.py` for
      each arm: commanded TCP pose is tracked within tolerance. Done as
      `tests/integration/test_cartesian_motion.py` parametrised over
      `{ur5e, ur15}`; relies on the plugin's `on_activate` auto-hold
      (seeds `m_target_frame = m_current_frame`) and asserts joints
      stay within 0.15 rad for 5 s — same threshold as the crisp and
      simple_jimp regulation tests so M5 can flip controllers with one
      flag.
- [ ] Optional: wire a second cartesian mode if time permits.

---

## M5 — Evaluation and comparison harness

- [x] `evaluation/scenarios/*.yaml` schema (step, sine, regulation, random
      waypoints). Landed as schema v1 (ADR-0009) with
      `evaluation/scenarios/validate.py` + 35 unit tests.
- [x] `evaluation/run_evaluation.py` runs a scenario against a named
      controller on a named arm, emits CSV + plot. Runner lives at
      `evaluation/run_evaluation.py` + `evaluation/_reference.py`; run
      artefacts land under `evaluation/runs/` (gitignored). Plots are
      deferred to bullet 4 (report generation).
- [x] Metrics: RMSE, settling time, overshoot, control effort. Landed
      as `evaluation/compute_metrics.py` (+ 32 unit tests).
      Joint-space only in v1 — cartesian metrics are recorded as
      `skipped` pending FK (ADR-0010).
- [x] Comparison test: same scenarios on crisp-joint-impedance vs our
      simple joint impedance, for both `ur5e` and `ur15`. Emit a report
      under `evaluation/reports/`. Landed as `evaluation/compare.py`
      (+ 15 unit tests in `tests/unit/test_compare.py`). Aggregate-only
      driver: enumerates `{scenario, controller, robot}` combos via
      `run_evaluation.check_compatibility`, picks the newest valid
      run dir per combo, invokes `compute_metrics_for_run`, and emits
      `report.csv` + `report.md` under `evaluation/reports/<UTC-ts>/`.
      Cartesian combos surface as `not_yet_evaluated` (ADR-0010). Live
      sim dispatch deferred to a follow-up (the test gate can't spin
      per-combo sims at M5 cost).

---

## M6 — Unified MuJoCo sim interfaces (runtime-switchable, real-UR parity)

Goal: the MuJoCo sim exposes `position`, `velocity`, and `effort`
command interfaces **simultaneously** on every joint, plus
`position`/`velocity`/`effort` state interfaces, so controllers can be
switched at runtime without relaunching — matching the interface
surface of `ur_robot_driver` on a real UR. A single controllers YAML,
mirroring upstream UR's `ur_controllers.yaml`, works in sim and (once
authorised) on the real UR15 without changes.

Design captured in **ADR-0012**. Any task below must not contradict it;
if it does, supersede ADR-0012 first.

Cross-cutting constraints (reiterating AGENTS.md):

- All changes to the MuJoCo stack land on
  `third_party/ur_simulator@auto_dev`; parent repo bumps the pointer in
  a separate commit (ADR-0003).
- Every sim integration test parametrised over `{ur5e, ur15}`; both
  must pass.
- `third_party/crisp_controllers` and `third_party/cartesian_controllers`
  stay read-only; their plugins must keep working through the
  transition without config edits inside those submodules.

- [ ] **M6.0 — Operator gate (human).** Decide how to vendor the
      `mujoco_ros2_control` patch required by M6.4. Options laid out
      in ADR-0012 §Decisions-to-gate. Agent must stop on this bullet
      until operator resolves (AGENTS.md §7 — adding a submodule is a
      human gate). Outputs: updated `.gitmodules` or an overlay
      package under `third_party/`; ADR-0012 amended with the chosen
      option; no code changes from the agent in this bullet.
- [ ] **M6.1 — MJCF actuator multiplexing.** Extend
      `third_party/ur_simulator/src/ur_sim_config/scripts/generate_mujoco_model.sh`
      to emit **three actuators per joint** unconditionally:
      `<position>` (PD, soft kp/kv per ADR-0012), `<velocity>`, and
      `<motor>`, each with `forcerange` from the per-arm ur_types
      YAML. Keep the `control_mode` CLI arg accepted but document
      that it now only selects the **default active** controller, not
      the MJCF shape. Pinned by extending the existing MJCF
      regression tests in the sim repo (same launch command must
      still start cleanly on both arms).
- [ ] **M6.2 — URDF `<ros2_control>` interface surface.** Extend
      `ur_sim_config`'s ros2_control xacro block so every joint
      declares `command_interface {position, velocity, effort}` and
      `state_interface {position, velocity, effort}`, matching
      upstream `ur_robot_driver`'s block. Generated URDF must parse
      through both `ros2_control_cli verify_urdf` (or equivalent) and
      the existing sim smoke test.
- [ ] **M6.3 — Unified controllers YAML.** Merge
      `ur_position_controllers.yaml` + `ur_effort_controllers.yaml`
      into a single `ur_controllers.yaml` aligned with upstream
      `Universal_Robots_ROS2_Driver/.../ur_controllers.yaml`:
      `joint_state_broadcaster`, `joint_trajectory_controller`
      (`[position]`), `scaled_joint_trajectory_controller`
      (`[position]` + `speed_scaling_interface_name: ""`),
      `forward_position_controller`, `forward_velocity_controller`,
      `forward_effort_controller`. All loaded; only
      `joint_state_broadcaster` + the launch-default command
      controller are `active` at startup, the rest `--inactive`.
- [ ] **M6.4 — `mujoco_ros2_control` claim-aware ctrl routing.** Patch
      the plugin (vendor strategy decided in M6.0) so that each
      update tick:
      1. for every joint, inspect which of its three command
         interfaces is currently claimed;
      2. forward the claimed interface's commanded value into the
         matching MuJoCo actuator's `ctrl`;
      3. for the two unclaimed actuators: zero the `<motor>` /
         `<velocity>` ctrl, and snap the `<position>` ctrl to the
         current joint position `q` so the PD term contributes zero
         torque on unclaim (ADR-0012 §Transition rules).
      Unit test inside the plugin patch covers the three transition
      cases (position→effort, effort→velocity, velocity→position).
- [ ] **M6.5 — Sim launch updates.** Collapse `ur_sim_mujoco.launch.py`
      and the `ur_sim_effort.launch.py` / `ur_sim_control.launch.py`
      variants into one launch file that always spawns the full
      controller set from M6.3, with `--default-controller` CLI arg
      choosing which command controller is `active` at startup
      (default: `scaled_joint_trajectory_controller`, for real-UR
      parity). `scripts/launch_sim.sh` stops passing
      `control_mode`. Existing smoke test still passes on both arms.
- [ ] **M6.6 — Runtime controller switch integration test.** New
      `tests/integration/test_runtime_switch.py`, parametrised over
      `{ur5e, ur15}`: bring the sim up with JTC active, regulate to
      home, `switch_controllers --deactivate
      scaled_joint_trajectory_controller --activate
      forward_effort_controller` **without relaunch**, send a gravity
      comp torque, assert bounded drift; then switch back to JTC and
      assert regulation recovers. Same 0.15 rad tolerance as the M3 /
      M4 regulation tests so M5's `compare.py` can drive both modes.
- [ ] **M6.7 — Retire `gravity_compensation.py` as an out-of-band
      actuator.** With JTC able to claim effort via M6 or with the
      real JTC→position→MuJoCo-`<position>` path live, the sim's
      Python gravity-comp shim no longer needs to drive the arm.
      Either (a) delete it from the sim's default launches, or (b)
      keep it as a reference torque node behind an explicit
      `--enable-gravity-shim` flag. Decision recorded in ADR-0012.
      Pinned by an integration test asserting `joint_trajectory`
      goals are followed by JTC (not the shim) on both arms.
- [ ] **M6.8 — Bringup migration.** Simplify
      `crisp_bringup.launch.py` and `simple_jimp_bringup.launch.py`
      to start from the M6.3 unified controller set: no more
      per-role `control_mode`, no more manual
      `forward_effort_controller` deactivation (ADR-0007 becomes a
      runtime fact rather than a launch-time contract; supersede it
      if needed in ADR-0012). All existing crisp + simple_jimp
      integration tests green on both arms without any YAML change
      inside `third_party/crisp_controllers`.
- [ ] **M6.9 — Parent repo submodule pointer bump.** After every
      bullet that commits inside `third_party/ur_simulator@auto_dev`
      passes M6 tests, bump the submodule pointer in a single
      follow-up commit per AGENTS.md §3. Do **not** batch pointer
      bumps across multiple M6 bullets.

### M6 hard requirements (operator-stated, 2026-04-24)

The following requirements override any softer statement elsewhere in
M6. The agent must treat them as acceptance criteria, not suggestions.

**R1 — Sim/real binary parity of controllers.** Every controller we
ship (crisp roles, simple_jimp, cartesian_motion, JTC, scaled-JTC,
forward_{position,velocity,effort}) **must launch against the real UR
driver without any YAML, launch-file, or plugin change**. The only
thing that may differ between sim and real is which *driver* is
launched (`ur_sim_mujoco` vs `ur_robot_driver`). This means:
- Controller YAMLs do not branch on sim vs real.
- `bringup/launch/*.launch.py` accepts a `robot_driver:={sim,real}` arg
  (or equivalent) and swaps only the hardware-interface node.
- Interface names (command/state) are byte-identical to
  `ur_robot_driver`'s, enforced by ADR-0012.
- Any sim-only shim (e.g. `sim_broadcasters.py`,
  `gravity_compensation.py`) must not publish/subscribe to topics
  whose names clash with real-driver topics, and must be disabled by
  default once M6 lands.

**R2 — Structured test matrix.** Integration tests must exercise the
arm in this explicit order, each stage parametrised over
`{ur5e, ur15}` × `{no_payload, small_payload, large_payload}` (R3):

  1. **Single-joint motion.** For each of the six joints individually,
     hold the other five at home and command a ±30° step + a 0.5 Hz
     sine. Assert, **per controller**:
     - Position mode (JTC / forward_position): steady-state error
       below tolerance, **no drift** over a 10 s hold, **no limit-cycle
       oscillation** (FFT of joint velocity has no peak > noise floor
       between 1 Hz and Nyquist).
     - Effort/impedance modes (crisp, simple_jimp): bounded tracking
       error, **no sustained chatter** (velocity RMS over last 2 s
       below threshold), damping ratio from step response within
       ±20% of theoretical prediction computed from the configured
       `K`/`D`.
  2. **All joints together.** Commanded home → hand-picked cluttered
     pose → home, over 5 s, all six joints simultaneously. Assert:
     - Kinematic consistency: TCP FK from measured `q` matches the
       expected trajectory within tolerance (joint-space) or 5 mm +
       2° (cartesian mode).
     - No stall: all joints complete the motion; none saturate torque
       for more than 100 ms contiguously.
  3. **End-effector motion.** Command a known TCP trajectory (line,
     then arc, then sine-in-z) via `cartesian_motion_controller` and
     — separately — via JTC driven by an IK solver shim. Assert:
     - TCP RMSE below 5 mm, peak below 10 mm.
     - Yaw/pitch/roll peak error below 3°.
     - With payload attached (R3), compensated controllers
       (crisp_cartesian_impedance with `use_gravity_compensation: true`,
       plus future payload-aware configs) hold position in free space
       with steady-state TCP drift below 2 mm over 30 s.

  Each stage must publish, in the run artefact under
  `evaluation/runs/<ts>/`, the **theoretical expectation** alongside
  the measured result, and the test fails if the gap exceeds the
  documented tolerance. "Theoretical expectation" means:
  - For rigid position mode: `q(t) = q_d(t)` (zero lag at DC,
    first-order lag at HF driven by the configured `kp`/`kv`).
  - For joint impedance: closed-loop second-order response with
    $\omega_n = \sqrt{K/J_{\text{eff}}}$ and $\zeta = D/(2\sqrt{K J_{\text{eff}}})$,
    using reflected inertia `armature` declared in the MJCF.
  - For cartesian impedance: analogous but on the TCP with the
    configured Cartesian stiffness/damping.

  Tolerances, theoretical formulas, and effective inertia values per
  arm live in `tests/integration/expectations/<arm>.yaml`; one file
  per arm to keep the assertion code arm-agnostic.

**R3 — Configurable end-effector payload.** The sim must support an
end-effector extra weight, configurable via the dashboard (mass,
3×3 inertia tensor, SE(3) pose relative to `tool0`), that is:

- **Physically simulated.** MJCF augmented at runtime (or
  regenerated) with a `<body>` rigidly attached to `tool0`, with the
  given mass/inertia/pose. MuJoCo then accounts for its gravity,
  Coriolis, and inertial terms automatically.
- **Visualised.** Rendered as a wire-frame cube (edge length
  ∝ cube_root(mass/density), density configurable; default 1000
  kg/m³ for user intuition) in the dashboard's 3D canvas, anchored
  at the configured pose.
- **Consumed by controllers.** The sim publishes the current payload
  on a latched topic (e.g. `/ee_payload` as
  `geometry_msgs/Inertia` + `geometry_msgs/PoseStamped` under a
  custom `ur_sim_msgs/EePayload` message), matching what UR's real
  driver exposes via the `set_payload` service. Controllers that
  support payload-aware gravity comp (crisp_cartesian_impedance,
  scaled JTC on real UR) must pick it up without relaunch.
- **Exercised in tests.** Every integration test in R2 is
  parametrised over `{no_payload, small_payload, large_payload}` with
  payload specs recorded in `tests/integration/expectations/
  payloads.yaml` (mass, inertia, pose). `no_payload` = zero mass
  baseline. `small_payload` ≈ 1 kg at `tool0` centre. `large_payload`
  ≈ 5 kg offset 100 mm along the tool-Z — large enough to exercise
  gravity-comp errors visibly but within each arm's rated payload.

Exit criteria for M6 (updated):

- R1: every bringup launch works against both `ur_sim_mujoco` and
  `ur_robot_driver` (once the real UR is authorised, a dry-parity
  check against the real driver's interface schema suffices while
  M-REAL is out of scope).
- R2: the three-stage test matrix is green on both arms, at all
  three payload levels, for every controller under test.
- R3: the dashboard's payload widget round-trips mass/inertia/pose
  to the sim and back, and the cube visualisation matches the
  published payload.
- One launch command brings up a sim whose ros2_control interface
  surface is interface-identical to a real UR on the same driver
  version.
- Every controller listed in M6.3 can be activated at runtime on both
  arms.
- All crisp, simple_jimp, cartesian_motion integration tests from
  M2/M3/M4 pass unchanged against the unified sim.
- `test_runtime_switch.py[ur5e, ur15]` is green.
- ADR-0007 is either upheld with a new justification or superseded in
  ADR-0012.
- No relaunch is required to change control mode.

### M6 extended bullets (cover R1–R3)

- [ ] **M6.10 — Real-driver parity audit (R1).** Produce
      `docs/real_driver_parity.md` enumerating every ros2_control
      command/state interface name we emit in sim vs. what
      `Universal_Robots_ROS2_Driver` emits on the operator-
      confirmed version. Diff must be empty or every row justified.
      No code change in this bullet; purely a verification artefact
      the agent regenerates each iteration touching interfaces.
- [ ] **M6.11 — `robot_driver` launch arg across bringup (R1).**
      All `bringup/launch/*.launch.py` accept `robot_driver:={sim,real}`.
      `sim` delegates to the unified MuJoCo launch from M6.5. `real`
      delegates to a new `bringup/launch/real_ur_driver.launch.py`
      stub that includes `ur_robot_driver`'s `ur_control.launch.py`
      with our common controllers YAML. The `real` path is **not**
      exercised autonomously (AGENTS.md §5); it is a launch-file-only
      change, pinned by a launch-syntax test (`launch_testing` dry
      include).
- [ ] **M6.12 — Single-joint test stage (R2 stage 1).** New
      `tests/integration/test_per_joint.py` parametrised over
      `{ur5e, ur15} × joint ∈ 6 × controller ∈ {JTC,
      forward_position, forward_effort + crisp_joint_impedance,
      simple_jimp}`. Assertions: steady-state error, drift,
      oscillation FFT, damping-ratio-from-step. Theoretical values
      from `tests/integration/expectations/<arm>.yaml`.
- [ ] **M6.13 — All-joints-together test stage (R2 stage 2).** New
      `tests/integration/test_all_joints.py`, parametrised over
      `{ur5e, ur15} × controller`. Asserts kinematic consistency via
      FK and no torque saturation.
- [ ] **M6.14 — End-effector test stage (R2 stage 3).** New
      `tests/integration/test_tcp_trajectory.py`, parametrised over
      `{ur5e, ur15} × {cartesian_motion, JTC+ik_shim,
      crisp_cartesian_impedance}`. Asserts TCP RMSE and peak error
      against the theoretical trajectory.
- [~] **M6.15 — Expectation files (R2 support).** Author
      `tests/integration/expectations/{ur5e,ur15}.yaml` with
      per-joint effective inertia (including `armature`), rated
      torque, and per-controller theoretical-response formulas.
      `tests/integration/expectations/payloads.yaml` for R3.
      Schema pinned by a unit test under
      `tests/unit/test_expectations_schema.py`.
      **Status:** schema + first-draft values landed (ADR-0013);
      7 unit tests in `tests/unit/test_expectations_schema.py`
      freeze the structure. Numeric values remain `draft: true`
      pending operator review + MJCF `armature` extraction
      during M6.1.
- [ ] **M6.16 — Payload runtime API (R3 core).** Add
      `ur_sim_msgs/EePayload.msg` (mass, inertia tensor, pose).
      Sim publishes a latched `/ee_payload` on the same topic name
      `ur_robot_driver` uses (or a wrapper node bridges the two).
      Sim service `~/set_ee_payload` for setting it at runtime.
      **Initial payload on launch must be zero** to match the real
      arm out-of-the-box.
- [ ] **M6.17 — MJCF payload attachment (R3 physics).** At MJCF
      generation time (or via `mj_resetData`-safe runtime body
      injection), attach a body with the configured mass / inertia
      / pose to `tool0`. Zero-mass case = no body injected.
      Verified by comparing `mj_fullM` before/after with a golden
      reference at three payload levels.
- [ ] **M6.18 — Dashboard payload widget (R3 UX).** Extend
      `third_party/ur_simulator/src/ur_web_dashboard` with a panel
      that publishes on `~/set_ee_payload`: mass (kg), 3×3 inertia
      (kg·m²), pose (x/y/z/rpy relative to `tool0`). Cube is
      rendered in the existing three.js canvas, anchored at `tool0`,
      size derived from mass and a configurable density. Cube
      colour encodes "gravity-comp unaware controller active"
      (red) vs "payload-aware" (green) so the user sees the
      mismatch.
- [ ] **M6.19 — Payload parametrisation across R2 tests.** Every
      test added in M6.12 / M6.13 / M6.14 is additionally
      parametrised over `{no_payload, small_payload, large_payload}`
      from `payloads.yaml`. Expected-result tables in
      `expectations/*.yaml` include the payload-adjusted values.

---

## M-REAL — Real UR15 bringup (OUT OF SCOPE FOR NOW)

Do not enter this milestone without an explicit instruction from the
operator. Left here as a placeholder only.
