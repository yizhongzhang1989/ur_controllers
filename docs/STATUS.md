# Status

_Last updated: 2026-04-22 (bootstrap commit; not yet auto-maintained)._

## Current milestone

**M0 — Bootstrap** (in progress). After M0, priority order is: M2 (crisp
controllers working in sim) → M3 (our own joint impedance) → M4 (cartesian
controllers). Every sim task must pass on both `ur5e` and `ur15`. Real UR15
is explicitly out of scope for now.

## Last completed task

- Submodules added under `third_party/`.
- Roadmap re-prioritised: crisp first, then our own joint impedance, then
  cartesian_controllers. UR5e and UR15 are both first-class sim targets.
  Real-robot work deferred.

## Next task (agent should pick this up)

- Decide ROS 2 distro and append an entry to `docs/DECISIONS.md`.
  (Blocked on human input — see "Blockers".)

## Build status

- Not yet a colcon workspace. No packages to build.

## Test status

- `scripts/run_tests.sh` is a placeholder; exits 0 with a warning when there
  is nothing to test.

## Blockers / open questions for operator

1. ROS 2 distro (Humble / Jazzy / Rolling)?
2. Which UR driver for real UR15 (official `Universal_Robots_ROS2_Driver`)?
3. Does `ur_simulator` expose a UR15 description, or do we proxy with UR5/UR10
   for M1–M4?
4. Run the auto-dev loop inside a container (recommended) or on host?
5. Exact copilot CLI invocation to use inside `scripts/auto_dev_loop.sh`.

## Recent commits

- (initial scaffolding pending)
