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
- [ ] Add top-level colcon workspace config (symlinks or `COLCON_IGNORE` markers).
- [ ] Add `.pre-commit-config.yaml` (clang-format, ruff, trailing whitespace).
- [ ] Add CI workflow `.github/workflows/ci.yml`: build + unit tests headless.
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
- [ ] Enumerate the three impedance controllers shipped by crisp; record
      their plugin names, command interfaces, and required params in
      `docs/crisp_controllers.md` (new, short reference file).
- [x] Controller 1 (crisp joint impedance): write
      `bringup/config/crisp_joint_impedance.{ur5e,ur15}.yaml`, wire it into
      `bringup/launch/crisp_bringup.launch.py`, bring it up on ur5e.
- [x] Same controller, bring it up on ur15.
- [ ] Controller 2 (second crisp impedance variant): same two-step rollout
      (ur5e, then ur15).
- [ ] Controller 3 (third crisp impedance variant): same two-step rollout.
- [ ] Integration tests under `tests/integration/test_crisp_*.py`: for each
      of the three controllers and each of `{ur5e, ur15}`, send a small
      regulation command and assert bounded tracking error within a fixed
      time window.
- [ ] Record baseline rosbags under `evaluation/baselines/crisp/`
      (gitignored payload; commit a manifest `.yaml` of what was recorded).

---

## M3 — Our own simplified joint impedance controller (PRIORITY 2)

Goal: a lightweight in-tree controller that we fully own, modelled on the
crisp joint impedance controller from M2 but stripped of features we don't
need. It must match crisp's behaviour on a baseline regulation scenario
within an agreed tolerance (set in M4).

- [ ] Study the crisp joint impedance implementation we got running in M2.
      Append a `DECISIONS.md` entry enumerating what to keep vs drop
      (gravity comp, nullspace, friction comp, command interpolation, etc.).
- [ ] Create package `src/simple_joint_impedance_controller/`
      (`controller_interface::ControllerInterface` plugin, `ament_cmake`).
- [ ] Control law baseline: `tau = K (q_d - q) - D * qdot`, with torque
      saturation and safe defaults. Optional gravity-comp hook only if M2
      showed it is needed to match crisp.
- [ ] Parameters via `generate_parameter_library`: `joints`, `K`, `D`,
      `tau_max`, command topic, optional feature flags.
- [ ] Unit tests (gtest) for the control-law math, no ROS.
- [ ] Integration tests under `tests/integration/test_simple_jimp_*.py`
      for `{ur5e, ur15}`: step + regulation, bounded error.
- [ ] Update `bringup/launch/` with a `simple_jimp_bringup.launch.py`
      mirroring the crisp launch, so comparison later is a single flag.

---

## M4 — Make `cartesian_controllers` work in sim (PRIORITY 3)

Goal: at least one working task-space controller from `cartesian_controllers`
on both arms, for later comparison / use alongside our joint-space work.

- [ ] Build `cartesian_controllers` from submodule; resolve deps.
- [ ] Pick a primary mode (e.g. cartesian motion + compliance) and write
      `bringup/config/cartesian_motion.{ur5e,ur15}.yaml`.
- [ ] Launch file `bringup/launch/cartesian_bringup.launch.py`; bring up on
      ur5e, then ur15.
- [ ] Integration tests under `tests/integration/test_cartesian_*.py` for
      each arm: commanded TCP pose is tracked within tolerance.
- [ ] Optional: wire a second cartesian mode if time permits.

---

## M5 — Evaluation and comparison harness

- [ ] `evaluation/scenarios/*.yaml` schema (step, sine, regulation, random
      waypoints).
- [ ] `evaluation/run_evaluation.py` runs a scenario against a named
      controller on a named arm, emits CSV + plot.
- [ ] Metrics: RMSE, settling time, overshoot, control effort.
- [ ] Comparison test: same scenarios on crisp-joint-impedance vs our
      simple joint impedance, for both `ur5e` and `ur15`. Emit a report
      under `evaluation/reports/`.

---

## M-REAL — Real UR15 bringup (OUT OF SCOPE FOR NOW)

Do not enter this milestone without an explicit instruction from the
operator. Left here as a placeholder only.
