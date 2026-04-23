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
- [ ] Write `scripts/setup_env.sh` to install ROS deps via `rosdep`.

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

## M-REAL — Real UR15 bringup (OUT OF SCOPE FOR NOW)

Do not enter this milestone without an explicit instruction from the
operator. Left here as a placeholder only.
