# Decisions

Append-only log of architectural decisions. Newer entries at the bottom.
Format: ADR-lite. Do not delete past entries; supersede with a new one.

---

## ADR-0001 — Repository layout and automation contract

- **Date:** 2026-04-22
- **Status:** Accepted
- **Context:** Project is driven by an automated coding agent iterating via a
  bash loop. Clear, stable structure is required so the agent can locate
  sources, tests, and state files deterministically.
- **Decision:**
  - `third_party/` holds git submodules, read-only to the agent.
  - `src/` holds our own controller packages.
  - `bringup/`, `evaluation/`, `tests/` are top-level and ours.
  - `docs/ROADMAP.md` is the goal source of truth (human-edited).
  - `docs/STATUS.md` is the live state (agent-overwritten).
  - `docs/DECISIONS.md` is append-only.
  - `AGENTS.md` defines the iteration contract.
- **Consequences:** Any agent or human reading these five files in order
  (AGENTS → ROADMAP → STATUS → DECISIONS → git log) has enough context to
  produce the next commit.

---

## ADR-0002 — ROS 2 distribution: Humble

- **Date:** 2026-04-22
- **Status:** Accepted
- **Context:** `ros2_control` APIs, `ur_simulator` documentation, and all
  system packages already installed on the dev machine target ROS 2
  Humble on Ubuntu 22.04. `crisp_controllers` and `cartesian_controllers`
  build against Humble out of the box. Moving to Jazzy would churn
  submodules and system packages without delivering a feature we need
  right now.
- **Decision:** Pin the workspace to **ROS 2 Humble**. All scripts,
  launch files, configs, and CI assume `/opt/ros/humble/setup.bash`.
  Re-evaluate when Humble reaches EOL (May 2027) or when a required
  feature lands in a newer distro.
- **Consequences:** Any agent change that assumes a newer distro must
  supersede this ADR first. `rosdep`, the CI image, and the dev-loop
  scripts all hard-code Humble.
---

## ADR-0003 — Submodule ownership split

- **Date:** 2026-04-22
- **Status:** Accepted
- **Context:** Three projects live under `third_party/`. Two are external
  (`crisp_controllers`, `cartesian_controllers`) and must stay pristine so
  they can be re-pulled from upstream without local drift. One
  (`ur_simulator`) is our own simulator, still under active development, and
  needs to be editable from within this workbench.
- **Decision:**
  - `crisp_controllers` and `cartesian_controllers` are **read-only**. The
    agent only ever updates their submodule pointer, and only on explicit
    human request.
  - `ur_simulator` development happens on a dedicated `auto_dev` branch of
    the submodule (created from `main`, pushed to origin). The branch is
    tracked via `branch = auto_dev` in `.gitmodules`. All edits inside the
    submodule must target this branch; commits happen inside
    `third_party/ur_simulator` first, then the parent repo bumps the
    pointer in a follow-up commit.
- **Consequences:** `git submodule update --remote third_party/ur_simulator`
  fast-forwards to the latest `auto_dev`. Upstream `main` of ur_simulator is
  never directly touched by the agent; merges from `auto_dev` → `main` are
  human-gated via pull request.

---

## ADR-0004 — Task priority and sim target matrix

- **Date:** 2026-04-22
- **Status:** Accepted
- **Context:** Operator set an explicit order of work: (1) get
  `crisp_controllers` running — in particular its three impedance
  controllers, (2) implement our own lightweight joint impedance
  controller, (3) get `cartesian_controllers` running. Real UR15 hardware
  is deferred. Both `ur5e` and `ur15` must be usable targets in sim so
  conclusions transfer.
- **Decision:**
  - ROADMAP milestones reordered: M2 crisp → M3 our joint impedance →
    M4 cartesian → M5 evaluation harness. M-REAL is a placeholder only.
  - Every sim integration test is parametrised over `{ur5e, ur15}` and
    must pass on both before the parent task is considered done. Gains
    may differ per arm; scenarios and pass/fail thresholds do not.
  - `ur_simulator` may be modified on its `auto_dev` branch whenever a
    controller task needs something the sim does not yet provide (e.g.
    UR15 description, correct effort interface). Such changes land as
    commits inside the submodule plus a pointer-bump commit here.
- **Consequences:** The agent will not enter M-REAL autonomously. The
  simulator itself is an active work item, not a frozen dependency.

---

## ADR-0005 — Build skip list and simulator choice

- **Date:** 2026-04-22
- **Status:** Accepted
- **Context:** `third_party/cartesian_controllers` ships two packages
  that are not useful for our pipeline:
  - `cartesian_controller_simulation` requires the MuJoCo C library at
    `/home/robot/mujoco-3.0.0`, which is not installed system-wide. We
    already have `ur_simulator` (same MuJoCo backend, better integrated)
    as our sim.
  - `cartesian_controller_tests` is a ROS 1 catkin package that colcon
    cannot build and that exercises a completely different simulator.
- **Decision:** Always pass
  `--packages-skip cartesian_controller_simulation cartesian_controller_tests`
  to `colcon build`. Our sim is `ur_simulator` (MuJoCo via
  `mujoco_ros2_control`). The skip list lives in README.md,
  `scripts/launch_sim.sh` assumes `install/setup.bash` produced with that
  skip list.
- **Consequences:** Controllers from `cartesian_controllers` are built
  and usable as ros2_control plugins, just not via that package's own
  simulation node. Our evaluation scenarios drive them through
  `ur_simulator` like every other controller.
