# Roadmap

Milestones are ordered. The agent works on the earliest milestone that has
unchecked items. Within a milestone, tasks are also ordered.

Legend: `[ ]` todo, `[~]` in progress, `[x]` done, `[!]` blocked (see STATUS.md).

---

## M0 — Bootstrap

- [x] Create repo scaffolding (directories, docs, scripts).
- [ ] Decide ROS 2 distro; record in `docs/DECISIONS.md`.
- [ ] Add git submodules under `third_party/`:
  - [ ] `ur_simulator` — https://github.com/yizhongzhang1989/ur_simulator.git
  - [ ] `crisp_controllers` — https://github.com/yizhongzhang1989/crisp_controllers.git
  - [ ] `cartesian_controllers` — https://github.com/yizhongzhang1989/cartesian_controllers.git
- [ ] Add top-level colcon workspace config (symlinks or `COLCON_IGNORE` markers).
- [ ] Add `.pre-commit-config.yaml` (clang-format, ruff, trailing whitespace).
- [ ] Add CI workflow `.github/workflows/ci.yml`: build + unit tests headless.
- [ ] Write `scripts/setup_env.sh` to install ROS deps via `rosdep`.

## M1 — Simulator brings up a UR arm

- [ ] `bringup/launch/sim_bringup.launch.py` starts `ur_simulator` with a UR
      description.
- [ ] Verify `/joint_states` publishes at expected rate.
- [ ] Verify command interfaces accept position/velocity/effort as claimed
      by the sim.
- [ ] Add integration smoke test: launch sim, wait for `/joint_states`, pass.

## M2 — Third-party controllers run in sim

- [ ] Build `crisp_controllers` from submodule; resolve deps.
- [ ] Launch `crisp` joint impedance controller against sim; tune minimal
      working gains.
- [ ] Build `cartesian_controllers` from submodule; resolve deps.
- [ ] Launch at least one `cartesian_controllers` mode against sim.
- [ ] Record baseline rosbags under `evaluation/baselines/` (gitignored; store
      manifest only).

## M3 — Own simplified joint impedance controller

- [ ] Study crisp joint impedance; append `DECISIONS.md` entry listing kept
      vs dropped features.
- [ ] Create package `src/simple_joint_impedance_controller/` (ros2_control
      plugin, `controller_interface::ControllerInterface`).
- [ ] Control law: `tau = K (q_d - q) - D * qdot`, with torque saturation
      and safe defaults.
- [ ] Parameters via `generate_parameter_library` (`K`, `D`, `tau_max`,
      `joints`).
- [ ] Unit tests (gtest) for the control-law math, no ROS.
- [ ] Integration test: load plugin in sim, step input, assert settling
      within tolerance.

## M4 — Evaluation harness

- [ ] `evaluation/scenarios/*.yaml` schema (step, sine, regulation, random
      waypoints).
- [ ] `evaluation/run_evaluation.py` runs a scenario against a named
      controller, emits CSV + plot.
- [ ] Metrics: RMSE, settling time, overshoot, control effort.
- [ ] Comparison test: same scenario on crisp vs ours; emit report under
      `evaluation/reports/`.

## M5 — Real UR15 bringup (human-gated)

- [ ] `bringup/launch/real_bringup.launch.py` using the UR ROS 2 driver.
- [ ] Safety checklist in `docs/SAFETY.md` (e-stop, reduced mode, joint
      limits, stiffness caps).
- [ ] Dry-run: controllers loaded, commands zero, verify no motion.
- [ ] Re-run M4 scenarios at conservative gains, human in the loop.
