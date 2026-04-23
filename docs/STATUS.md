# Status

_Last updated: 2026-04-23 (fix(m4): cartesian_bringup fetches
robot_description from the transient-local /robot_description topic
instead of the RSP get_parameters service — removes a flaky first-run
timeout observed in tests/integration/test_cartesian_motion.py[ur5e].)_

## Current milestone

**No mandatory milestone in flight — awaiting operator input.**

Going into this iteration, M0 was fully ticked (previous iteration
landed `scripts/setup_env.sh`) and M1–M5 were already complete. Of
the full ROADMAP, the only unchecked bullet is:

- **M4 bullet 5** — `[ ] Optional: wire a second cartesian mode if
  time permits.`

That bullet is **both** explicitly optional **and** blocked on a
`third_party/ur_simulator` `auto_dev` patch (F/T sensor frame +
`~/ft_sensor_wrench` publisher — see "Blockers" below and the prior
STATUS `.prev` entry). Per AGENTS.md §7, a submodule bump is a human
gate; per §3, the agent picks the single next highest-value item
**consistent with the current milestone**, and there is no such item
to pick this iteration.

The other remaining ROADMAP entry is:

- **M-REAL** — real UR15 bringup. Explicitly out of scope per
  AGENTS.md §5 and the ROADMAP preamble (placeholder only; must not
  be entered without operator authorisation).

Per AGENTS.md §7 (stop and ask) and §3 step 7 of the iteration brief,
this iteration commits a STATUS update declaring the blocker and
stops — no code, config, or test changes. This keeps the record of
iterations honest instead of inventing low-value work.

The outer loop's stop conditions (AGENTS.md §8) should terminate on
the next no-commit iteration via the "two consecutive iterations
produce no commit" guard, or sooner if the operator drops a `.STOP`
file at the repo root.

## Last completed tasks

- **This iteration: blocker-only STATUS update.** No code changes;
  surfaces that only human-gated work remains (M4 bullet 5,
  M-REAL). Prior STATUS preserved ad-hoc in git history.
- **M0 bullet `scripts/setup_env.sh` (prior iteration).**
  Three-stage wrapper (apt → rosdep → pip + pre-commit install)
  with `--dry-run` and `--no-pre-commit` flags. Pinned by 26 unit
  tests in `tests/unit/test_setup_env.py`. **M0 is fully ticked.**
- **M0 bullet "CI workflow `.github/workflows/ci.yml`".** Two-job
  (`lint` → `build`) workflow; `build` runs inside
  `ros:humble-ros-base`, `rosdep install`s with ADR-0005 skip-keys,
  `colcon build`, then `scripts/run_tests.sh --unit-only`. Pinned
  by 12 unit tests in `tests/unit/test_ci_workflow.py`.
- **M0 bullet `.pre-commit-config.yaml`** (+ `.clang-format`,
  `ruff.toml`). Hooks pinned; `third_party/`, `build/`, `install/`,
  `log/` excluded. Pinned by 12 unit tests in
  `tests/unit/test_pre_commit_config.py`.
- **M0 bullet top-level colcon workspace config**
  (`colcon_defaults.yaml`). Pinned by
  `tests/unit/test_colcon_defaults.py`.
- **M5 bullet 4: `evaluation/compare.py`** (aggregate-only driver,
  ADR-0011). 15 unit tests in `tests/unit/test_compare.py`.
- **M5 bullet 3: `evaluation/compute_metrics.py`** (joint-space v1,
  ADR-0010). 32 unit tests.
- **M5 bullet 2: `evaluation/run_evaluation.py`** (scenario
  dispatch + dry-run). 34 unit tests.
- **M5 bullet 1: `evaluation/scenarios/*.yaml` schema v1**
  (ADR-0009). 35 unit tests.
- **M4 bullets 2 + 3 + 4:** `cartesian_motion_controller` on ur5e
  and ur15 with regulation integration test
  (`tests/integration/test_cartesian_motion.py`).

## Next task (agent should pick this up)

**There is no next mandatory ROADMAP task.** The agent should stop
and wait for operator input. Candidate next actions require
operator authorisation:

- **Enter M4 bullet 5** (second cartesian mode). Requires operator
  sign-off to patch `third_party/ur_simulator@auto_dev` with the
  F/T sensor frame + `~/ft_sensor_wrench` publisher, then bump the
  submodule pointer here (AGENTS.md §7 — submodule bump gate).
- **Enter M-REAL** (real UR15 bringup). Out of scope until the
  operator explicitly authorises real-robot work for an iteration
  (AGENTS.md §5, ROADMAP preamble).
- **Take on a non-ROADMAP follow-up** (see suggestions below) —
  but only with explicit operator direction; AGENTS.md §3 forbids
  picking up work that is not the next ordered ROADMAP item.

Suggested follow-ups (**not** ROADMAP bullets, do not pick these up
without operator sign-off):

- A second CI job (or a separate workflow) that runs the
  integration tests behind an `xvfb` / headless-MuJoCo wrapper,
  once that story exists. The current workflow deliberately stops
  at unit tests.
- Live-dispatch mode for `compare.py` (subprocess
  `run_evaluation.py` per compatible combo) so the comparison is
  one command end-to-end once an operator is happy to pay the
  per-combo ~1 min sim cost.
- FK-backed cartesian metrics (ADR-0010) — would flip the
  `not_yet_evaluated` rows into real numbers.
- Intermittent flake:
  `test_crisp_joint_impedance_regulation[ur15]` occasionally
  trips the ADR-0006 `load_controller` RMW-response race even
  with spawners serialised. A deterministic fix would live on
  the sim's `auto_dev` branch (spawner retry with idempotent load
  semantics, or a further `--service-call-timeout` bump).

## Build status

- ROS 2 distro: **Humble** (Ubuntu 22.04, matches system install).
- `colcon build` (from the repo root, picking up
  `colcon_defaults.yaml`) produces **10** packages successfully,
  unchanged from prior iteration.
- No build changes this iteration.

## Test status

- No test changes this iteration. Last green end-to-end run of
  `scripts/run_tests.sh` (prior iteration): **172** pytest unit
  tests + **22** `test_math` gtests (simple_joint_impedance_controller)
  + **5** `crisp_controllers` gtests + **12** integration tests
  (sim smoke + 3 crisp roles + our simple_joint_impedance_controller
  + `cartesian_motion_controller`, each ×{ur5e, ur15}).
- `pre-commit run --all-files` was clean at that point; the only
  file changed this iteration is `docs/STATUS.md`, which is not
  covered by pre-commit beyond trailing-whitespace / EOL hooks.

## Blockers / open questions for operator

Informational + active:

- **[Active gate]** M4 bullet 5 (second cartesian mode) is blocked
  on the `ur_simulator` F/T sensor patch. The cartesian compliance
  and force controllers both need an `ft_sensor_ref_link` on the
  URDF chain plus an `~/ft_sensor_wrench` publisher. The UR sim
  does not currently expose an F/T sensor frame. Fix must land on
  `third_party/ur_simulator`'s `auto_dev` branch, then the parent
  repo bumps the submodule pointer — both are operator-gated per
  AGENTS.md §7. **Operator: please confirm whether to enter this
  bullet, and with what sensor layout.**
- **[Active gate]** M-REAL (real UR15) — explicit operator
  authorisation required per AGENTS.md §5 before the agent may
  touch it. Still not authorised.
- ROS distro pinned to Humble (system install); formalised via
  ADR-0002. The CI workflow targets the same distro via
  `ros:humble-ros-base`; `scripts/setup_env.sh` hard-codes the
  `ros-humble-*` apt package names.
- Dashboard opens at `http://localhost:8000`; rosbridge at
  `ws://localhost:9090`. Both are pkill'd + port-cleared by
  `scripts/kill_sim.sh`.
- Gravity-comp hook for `simple_joint_impedance_controller` remains
  unused (pure PD holds both arms within the M3 tolerance).
- Cartesian metrics remain deferred (ADR-0010). `compare.py`
  surfaces cartesian combos as `not_yet_evaluated`.
- `compare.py --aggregate-only` is the only mode implemented; live
  matrix dispatch is deferred behind a future `--live` flag.
- CI does not yet exercise integration tests — they require a live
  MuJoCo sim that we have no headless story for. The workflow
  stops at `--unit-only` deliberately.

## Recent commits

Run `git log --oneline -n 20` for the live list.
