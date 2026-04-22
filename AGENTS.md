# AGENTS.md — Instructions for Automated Coding Agents

This file is the contract between the human operator and any automated agent
(e.g. Copilot CLI) iterating on this repository. Read it in full before every
iteration.

## 1. Project goal

Evaluate and develop UR robot controllers:

1. Integrate third-party controllers (`crisp_controllers`, `cartesian_controllers`)
   against a simulated UR arm (`ur_simulator`).
2. Run the same controllers on a real UR15.
3. Develop our own lightweight joint impedance controller, inspired by
   `crisp_controllers` but simpler and fully owned in-tree.

## 2. Directory map

| Path | Ownership | Rule |
|------|-----------|------|
| `third_party/*` | External (git submodules) | **Never modify** `crisp_controllers/` and `cartesian_controllers/` — read-only. `ur_simulator/` is ours: work on its `auto_dev` branch only, commit inside the submodule, then bump the pointer here. |
| `src/*` | Ours | All new C++/Python controller code goes here. |
| `bringup/*` | Ours | Launch files and configs for sim and real robot. |
| `evaluation/*` | Ours | Benchmark harness and scenarios. |
| `tests/*` | Ours | Unit, integration, and comparison tests. |
| `scripts/*` | Ours | Automation scripts for the dev loop. |
| `docs/ROADMAP.md` | Ours, human-edited | Ordered milestones. Source of truth for *what to do next*. |
| `docs/STATUS.md` | Ours, agent-edited | Live state snapshot. Overwrite each iteration. |
| `docs/DECISIONS.md` | Ours, append-only | Architectural decisions. Never delete entries. |

## 3. Iteration workflow (MANDATORY)

Every agent invocation must:

1. **Read state** — `docs/ROADMAP.md`, `docs/STATUS.md`, `docs/DECISIONS.md`,
   recent `git log --oneline -n 20`.
2. **Pick exactly ONE task** — the single next highest-value item consistent
   with the current milestone. Do not attempt multiple milestones in one
   iteration.
3. **Implement** — code + tests together. New behaviour without a test is not
   acceptable.
4. **Test** — run `scripts/run_tests.sh`. Must exit 0.
5. **Commit** — one logical change, Conventional Commits format:
   `feat(scope): …`, `fix(scope): …`, `test(scope): …`, `docs(scope): …`,
   `chore(scope): …`, `build(scope): …`.
6. **Update `docs/STATUS.md`** — reflect the new state (what was done, what is
   next, any blockers).
7. If a non-trivial architectural choice was made, append an entry to
   `docs/DECISIONS.md`.
8. **Do not `git push` from inside an iteration.** The outer
   `scripts/auto_dev_loop.sh` pushes the current branch to `origin` after
   the test gate passes. If you are running the agent without the loop,
   the operator will push manually.

If tests fail, the iteration must fix them before committing. Never commit a
red tree.

## 4. Coding standards

- ROS 2 distribution: see `docs/DECISIONS.md` (must be pinned before M1).
- C++17, `ament_cmake`, `ros2_control` plugin conventions.
- Python: ≥3.10, `ament_python` or plain scripts with type hints.
- Formatting: `clang-format` (LLVM base) for C++, `ruff format` for Python.
- Linting: `ament_lint_auto` for C++ packages, `ruff check` for Python.
- Parameters: prefer `generate_parameter_library` for ros2_control plugins.

## 5. Testing requirements

- **Unit tests** (`tests/unit/` or package `test/`): pure control-law math,
  no ROS. Use `gtest` for C++, `pytest` for Python.
- **Integration tests** (`tests/integration/`): `launch_testing` bringing up
  the controller against the sim; assert on `/joint_states` response.
  **Every sim integration test must be parametrised over `{ur5e, ur15}`**
  and pass on both arms before the task is considered done.
- **Comparison tests** (`tests/compare/`): run identical scenario against
  `crisp` baseline and our controller, compare metrics. Also parametrised
  over `{ur5e, ur15}`.
- **Real UR15**: do not write or run real-robot launches in this phase.
  M-REAL in the roadmap is a placeholder and must not be entered without
  an explicit operator instruction.

## 6. Forbidden actions

- `git push --force`, `git push --force-with-lease`, `git reset --hard` on
  shared branches, amending or rebasing commits that have already been
  pushed.
- Pushing from inside an iteration. The outer loop handles pushes.
- Modifying files under `third_party/crisp_controllers/` or
  `third_party/cartesian_controllers/`. These are read-only mirrors of
  upstream.
- Committing on the wrong branch inside `third_party/ur_simulator/`. All
  edits there must land on `auto_dev`; `main` is upstream and off-limits.
- Disabling, skipping, or deleting failing tests to get green.
- Running any launch file that connects to the real UR15 autonomously.
  Real-robot launches require explicit human confirmation for that iteration.
- Lowering safety limits (joint limits, torque saturation, stiffness caps)
  without a corresponding entry in `docs/DECISIONS.md`.
- Committing large binaries, rosbags, or build artefacts. Use `.gitignore`.

## 7. Human gates (stop and ask)

Pause and surface a question to the operator when:

- A submodule needs to be bumped to a new upstream commit.
- The ROS distro or major dependency version would change.
- A test must be modified in a way that weakens its assertions.
- Work would touch real-robot safety configuration.
- The next ROADMAP item is ambiguous or blocked.

## 8. Stop conditions for the outer loop

The `scripts/auto_dev_loop.sh` loop terminates when any of:

- `.STOP` file exists at the repo root.
- All ROADMAP milestones are marked `[x]`.
- The configured max-iterations count is reached (default 100).
- Two consecutive iterations produce no commit (no-progress guard).
- `git push` fails (typically because the remote has diverged). The loop
  stops instead of trying to force or rebase; a human resolves.
