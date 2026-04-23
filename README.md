# ur_controllers

A workbench for evaluating and developing controllers for Universal Robots
arms (simulation first, real UR15 later). Driven by an automated coding
agent loop.

Current scope (ROS 2 Humble, Ubuntu 22.04):

- Run the three impedance controllers in
  [`crisp_controllers`](third_party/crisp_controllers) against the sim on
  both **UR5e** and **UR15**.
- Develop our own lightweight joint-impedance controller in
  [`src/`](src), modelled on crisp's joint impedance but stripped down and
  fully owned in-tree.
- Bring up [`cartesian_controllers`](third_party/cartesian_controllers)
  against the sim.
- Real UR15 is out of scope until explicitly authorised.

## Layout

- [`third_party/`](third_party) — git submodules.
  - [`ur_simulator`](third_party/ur_simulator) — **ours**, edited on the
    submodule's `auto_dev` branch.
  - [`crisp_controllers`](third_party/crisp_controllers) — read-only.
  - [`cartesian_controllers`](third_party/cartesian_controllers) — read-only.
- `src/` — our own packages (first target:
  `simple_joint_impedance_controller`).
- `bringup/` — launch files and configs (sim + controllers).
- `evaluation/` — benchmark scenarios and comparison harness.
- `tests/` — `unit/`, `integration/`, `compare/`.
- [`docs/`](docs) — [ROADMAP.md](docs/ROADMAP.md),
  [STATUS.md](docs/STATUS.md), [DECISIONS.md](docs/DECISIONS.md).
- `scripts/` — automation:
  [auto_dev_loop.sh](scripts/auto_dev_loop.sh),
  [status_check.sh](scripts/status_check.sh),
  [run_tests.sh](scripts/run_tests.sh),
  [launch_sim.sh](scripts/launch_sim.sh),
  [kill_sim.sh](scripts/kill_sim.sh).

## Prerequisites

- Ubuntu 22.04
- ROS 2 Humble (`/opt/ros/humble`)
- System packages used by `ur_simulator` (installed via apt + rosdep):
  `ros-humble-mujoco-ros2-control`, `ros-humble-ur-description`,
  `ros-humble-ros2-control`, `ros-humble-ros2-controllers`,
  `ros-humble-pinocchio`, `ros-humble-rosbridge-suite`,
  `ros-humble-ros-gz`, `ros-humble-gz-ros2-control`.
- `python3`, `colcon`, `git`, `xacro`.

## Quick start

```bash
# 1. Clone with submodules.
git clone --recurse-submodules <this repo>
cd ur_controllers
git submodule update --init --recursive

# 2. Install rosdeps and build. From the repo root, `colcon_defaults.yaml`
#    auto-supplies --symlink-install, --base-paths and --packages-skip.
source /opt/ros/humble/setup.bash
rosdep install --from-paths src third_party --ignore-src -r -y
colcon build

# 3. Launch the simulator with a chosen arm.
scripts/launch_sim.sh ur5e effort   # or: scripts/launch_sim.sh ur15 effort
# Dashboard: http://localhost:8000    rosbridge: ws://localhost:9090

# 4. Run tests (unit + integration, cleans sim between runs).
scripts/run_tests.sh
```

`effort` mode is the default because both crisp and our own joint-impedance
controller command torques. Use `scripts/launch_sim.sh ur5e position` for
trajectory-based control.

## Why some packages are skipped

The skip list lives in [`colcon_defaults.yaml`](colcon_defaults.yaml) at
the repo root and is loaded automatically by `colcon` when invoked from
this directory (via the `python3-colcon-defaults` plugin).

- `cartesian_controller_simulation` — needs the MuJoCo C library at
  `/home/robot/mujoco-3.0.0`. We use `ur_simulator` for simulation instead.
- `cartesian_controller_tests` — ROS 1 catkin package, not part of the
  ROS 2 build.

## Automation

The project is designed to be driven by an automated agent loop:

```bash
scripts/auto_dev_loop.sh           # defaults: MAX_ITERS=100, AUTO_PUSH=1
MAX_ITERS=5 scripts/auto_dev_loop.sh
AUTO_PUSH=0 scripts/auto_dev_loop.sh   # local-only
```

Each iteration:

1. `status_check.sh` refreshes a state snapshot.
2. The agent (Copilot CLI) reads [AGENTS.md](AGENTS.md) + `docs/*`, picks
   one task, implements it with tests, and commits.
3. `run_tests.sh` gates the commit — red tests block progress.
4. The loop pushes the branch to `origin` (non-force). Push failures stop
   the loop rather than trying to force.
5. The loop stops on `.STOP`, a fully-checked roadmap, two consecutive
   no-progress iterations, or the iteration cap.

See [AGENTS.md](AGENTS.md) for the full contract.

## Cleaning up a stuck sim

If a previous launch left stale processes or port conflicts (9090 /
8000), run:

```bash
scripts/kill_sim.sh
```

This kills rosbridge, the dashboard HTTP server, `ros2_control_node`,
MuJoCo/Gazebo processes, the ROS 2 CLI daemon, and any listeners on the
rosbridge/dashboard ports.

## Safety

Anything touching the real UR15 is human-gated. The agent is forbidden
from launching real-robot bringup autonomously. See [AGENTS.md](AGENTS.md)
§6–§7.
