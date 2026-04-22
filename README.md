# ur_controllers

A workbench for evaluating and developing controllers for Universal Robots
arms (simulation first, real UR15 second). Driven by an automated coding
agent loop.

## What's here

- `third_party/` — external controller stacks and the simulator, pulled in as
  git submodules. **Never edited directly.**
  - `ur_simulator` — simulation environment under development.
  - `crisp_controllers` — reference joint-impedance and related controllers.
  - `cartesian_controllers` — task-space controllers.
- `src/` — our own code. The first target is
  `simple_joint_impedance_controller`, a lightweight take on the `crisp`
  joint impedance controller.
- `bringup/` — launch files and configs for simulation and real robot.
- `evaluation/` — benchmark scenarios and the comparison harness.
- `tests/` — `unit/`, `integration/`, and `compare/` layers.
- `docs/` — `ROADMAP.md`, `STATUS.md`, `DECISIONS.md`.
- `scripts/` — the automation: `auto_dev_loop.sh`, `status_check.sh`,
  `run_tests.sh`.

## How the automation works

A bash loop drives the agent:

```
scripts/auto_dev_loop.sh
```

Each iteration:

1. `status_check.sh` refreshes the state snapshot.
2. The agent (Copilot CLI) is invoked with a prompt that forces it to read
   `AGENTS.md` + `docs/*`, pick one task, implement + test it, and commit.
3. `run_tests.sh` gates the commit. Red tests block progress.
4. The loop stops on `.STOP`, on a completed `ROADMAP.md`, on repeated
   no-progress, or on the iteration cap.

See [AGENTS.md](AGENTS.md) for the full contract and
[docs/ROADMAP.md](docs/ROADMAP.md) for the ordered milestones.

## Running a single iteration manually

```
scripts/status_check.sh > docs/STATUS.md
scripts/run_tests.sh
```

## Safety

Anything touching the real UR15 is human-gated. The agent is forbidden from
launching real-robot bringup autonomously. See `AGENTS.md` §6–§7.
