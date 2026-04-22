# Copilot Workspace Instructions

This workspace is driven by automated agents. Before any change, read
[AGENTS.md](../AGENTS.md) at the repo root — it is the authoritative workflow
contract.

## Quick rules

- One logical change per commit. Conventional Commits format.
- Do not edit `third_party/crisp_controllers/` or
  `third_party/cartesian_controllers/` — they are read-only submodules.
  `third_party/ur_simulator/` is ours: edits must land on its `auto_dev`
  branch only.
- All new behaviour requires a test. Run `scripts/run_tests.sh` before commit.
- Update `docs/STATUS.md` at the end of every iteration.
- Append to `docs/DECISIONS.md` when making an architectural choice.
- **Do not `git push` from inside an iteration.** The outer loop
  (`scripts/auto_dev_loop.sh`) pushes after the test gate passes.
- Do not launch anything against the real UR15 without explicit operator
  approval in that iteration.

## Where to put things

- Your own controllers → `src/`
- Launch files and configs → `bringup/`
- Tests → package-local `test/` for unit, `tests/integration/` for launch tests,
  `tests/compare/` for baseline-vs-ours comparisons.
- Benchmark scenarios → `evaluation/scenarios/`.

## Where to read from

- `docs/ROADMAP.md` — milestones, in order.
- `docs/STATUS.md` — current state.
- `docs/DECISIONS.md` — prior architectural choices (do not contradict silently).
