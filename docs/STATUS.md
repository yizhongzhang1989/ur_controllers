# Status

_Last updated: 2026-04-23 (feat(m5): comparison report driver
`evaluation/compare.py` wrapping bullets 2 + 3 with CSV + Markdown
output under `evaluation/reports/<UTC-ts>/`, 15 new unit tests. Full
test gate green in ~6:30.)_

## Current milestone

**M5 — evaluation and comparison harness: DONE for the M5 scope we can
reach without hardware.** M0–M4 done (with M4 bullet 5 still parked
behind a human gate for the sim F/T sensor patch). M5 bullets 1–4 are
all ticked. Next milestone is either M4 bullet 5 (cartesian second
mode, human-gated) or M-REAL (explicitly out-of-scope until operator
approval).

M5 bullet 4 landed this iteration as `evaluation/compare.py` plus 15
new unit tests in `tests/unit/test_compare.py`. The driver:

1. Enumerates every `{scenario, controller, robot}` combination from
   the requested matrix (defaults: all YAMLs under
   `evaluation/scenarios/`, every `known_controllers()`, both
   `{ur5e, ur15}`).
2. Filters using `run_evaluation.check_compatibility` and each
   scenario's `robots:` list. Incompatible combos are still surfaced
   on a separate Markdown table so the reader can see why they were
   excluded.
3. For each compatible combo, finds the newest *valid* run dir under
   `--runs-root` (newest name first, skipping partial runs that lack
   `manifest.yaml` — see ADR-0011) and invokes the library-level
   `compute_metrics.compute_metrics_for_run`, caching
   `metrics.yaml` + `metrics.csv` next to the run.
4. Emits `report.csv` + `report.md` under `--report-dir`
   (default `evaluation/reports/<UTC-ts>/`, gitignored) with one row
   per combination and one column per metric + overall status.

Cartesian combos are reported as `not_yet_evaluated` per ADR-0010,
even when a run dir exists — the report layer replaces the
metrics-layer `overall_status` to make "FK deferred in v1" visible.
Missing run dirs for *joint* combos are recorded as `no_run` and push
the exit code to `1`; cartesian combos without a run are still
`not_yet_evaluated` (the FK gap is the real reason, not the absent
bag).

Invocation:

```bash
# Default: aggregate every compatible combo in the canonical matrix
# against evaluation/runs/, write to evaluation/reports/<UTC-ts>/.
python3 evaluation/compare.py

# Narrower run of the crisp-joint vs simple-joint comparison.
python3 evaluation/compare.py \
    --scenarios evaluation/scenarios/step.example.yaml \
                evaluation/scenarios/regulation.example.yaml \
    --controllers crisp_joint_impedance simple_joint_impedance \
    --robots ur5e ur15 \
    --report-dir /tmp/m5_compare
```

Exit codes:

- `0` — every compatible combo is `pass` or `not_yet_evaluated`.
- `1` — at least one compatible combo is `fail` / `no_run` /
  `metrics_error`.
- `2` — driver misuse (bad CLI args, missing scenarios, unknown
  controller, etc.).

Live sim dispatch (spinning sim + controller per combo) is
deliberately **not** wired in this iteration — a 20-combo matrix of
~1 min/combo is too heavy for the test gate. The plan is: a human or
the outer loop drives individual `run_evaluation.py` invocations (or
a batch script) to populate `evaluation/runs/`, then re-runs
`compare.py` over the populated directory. When live dispatch is
added, it should subprocess `run_evaluation.py` (ROS/process isolation)
while continuing to import `compute_metrics_for_run` directly (no gain
from shelling out for pure Python work).

Key decisions this iteration (see ADR-0011):

- **Latest-valid run dir wins.** `run_evaluation.py` creates the run
  dir *before* the live run finishes, so a crashed attempt leaves a
  newer partial dir without `manifest.yaml`. `find_latest_run_dir`
  sorts prefix matches newest → oldest and picks the first one that
  has a `manifest.yaml`. This prevents a failed run from masking an
  older successful one.
- **Report-layer status differs from metrics-layer status for
  cartesian.** `compute_metrics_for_run` returns `overall_status=pass`
  for cartesian (because every pass key is `skipped`, which never
  fails the gate). The report needs to surface that as
  `not_yet_evaluated` to the human reader — done at the report layer
  so the metrics CLI semantics stay stable.
- **File-based module loading.** `compare.py` loads
  `run_evaluation.py` / `compute_metrics.py` / `validate.py` via
  `importlib.util.spec_from_file_location` — same pattern as
  `run_evaluation.py` already uses. `evaluation/` is not a package
  and shouldn't need to become one for bullet 4.

## Last completed tasks

- **M5 bullet 4: `evaluation/compare.py`.** New files:
  `evaluation/compare.py` (aggregate-only driver),
  `tests/unit/test_compare.py` (15 cases covering combo enumeration,
  latest-valid run-dir lookup, synthetic run-dir aggregation, CSV +
  Markdown emission, and CLI exit codes). No changes to
  `run_evaluation.py` or `compute_metrics.py` — the driver imports
  both via file-based `importlib` so the existing modules stay
  stable. `docs/ROADMAP.md` M5 bullet 4 ticked. `docs/DECISIONS.md`
  gets ADR-0011 (aggregate-only, latest-valid, report-layer cartesian
  status).
- **M5 bullet 3: `evaluation/compute_metrics.py`.** See prior STATUS.
- **M5 bullet 2: `evaluation/run_evaluation.py`.** See prior STATUS.
- **M5 bullet 1: `evaluation/scenarios/*.yaml` schema v1.** See prior STATUS.
- **M4 bullets 2 + 3 + 4: `cartesian_motion_controller` brought up on
  ur5e and ur15 with regulation integration test.** See prior STATUS.
- **M4 kick-off: `docs/cartesian_controllers.md` reference + tick M4
  bullet 1 (build).** See prior STATUS.
- **M3 — sim bring-up + regulation integration test for
  `simple_joint_impedance_controller` on `{ur5e, ur15}`.** See prior
  STATUS.

## Next task (agent should pick this up)

**M5 bullet 4 is done; M5 is closed for the scope we can reach
without new hardware / new simulator work.** The two remaining
roadmap items are both **human-gated** and must not be entered
autonomously:

- **M4 bullet 5** — second cartesian mode. Still gated on the sim
  F/T sensor patch on `third_party/ur_simulator`'s `auto_dev`
  branch. Operator must direct this work before the agent may enter
  it (see AGENTS.md §7).
- **M-REAL** — real UR15 bringup. Explicitly out of scope per
  AGENTS.md §5 and ROADMAP (placeholder only).

Suggested follow-ups (not blocking the outer loop but useful):

- Live-dispatch mode for `compare.py` (subprocess
  `run_evaluation.py` per compatible combo) so the comparison is
  one command end-to-end once an operator is happy to pay the
  per-combo ~1 min sim cost. Out-of-scope for the test gate, so
  should land behind a `--live` flag with an explicit smoke test
  on exactly one combo.
- FK-backed cartesian metrics (ADR-0010) — would flip the
  `not_yet_evaluated` rows into real numbers. Needs either
  `pinocchio` (already pulled in by crisp) or `tf2_ros` transform
  lookups during the run.

The outer loop should treat the current green test gate as a good
stopping point and wait for operator direction; two consecutive
agent iterations with no meaningful unchecked work would otherwise
trip the "no-progress" stop condition (AGENTS.md §8).

## Build status

- ROS 2 distro: **Humble** (Ubuntu 22.04, matches system install).
- `colcon build --symlink-install --base-paths src third_party
  --packages-skip cartesian_controller_simulation cartesian_controller_tests`
  produces **10** packages successfully, unchanged from prior
  iteration (no new ROS packages added — this iteration only touched
  `evaluation/` and `tests/unit/`).

## Test status

- `scripts/run_tests.sh` runs unit → colcon test → integration. Green
  in ~6:30 this iteration (35 schema + 34 runner-dry-run + 32
  metrics + **15 new compare** unit tests, 27 colcon gtests, 12
  integration tests).
- Test counts: **116** pytest unit tests
  (`tests/unit/test_scenarios_schema.py` +
  `test_run_evaluation_dry.py` + `test_compute_metrics.py` +
  `test_compare.py`) + **22** `test_math` gtests
  (simple_joint_impedance_controller) + **5** `crisp_controllers`
  gtests + **12** integration tests (sim smoke + 3 crisp roles +
  our simple_joint_impedance_controller + `cartesian_motion_controller`,
  each ×{ur5e, ur15}).

## Blockers / open questions for operator

None currently blocking. Informational:

- ROS distro pinned to Humble (system install); formalise via ADR
  if/when a second distro becomes a candidate.
- Dashboard opens at `http://localhost:8000`; rosbridge at
  `ws://localhost:9090`. Both are pkill'd + port-cleared by
  `scripts/kill_sim.sh`.
- The cartesian compliance and force controllers both need an
  `ft_sensor_ref_link` on the URDF chain plus an `~/ft_sensor_wrench`
  publisher. The UR sim does not currently expose an F/T sensor
  frame; those controllers therefore require a sim-side `auto_dev`
  patch before they can be brought up end-to-end. Still gates M4
  bullet 5; unchanged from prior iteration.
- Gravity-comp hook for `simple_joint_impedance_controller` remains
  unused (pure PD holds both arms within the M3 tolerance).
- Cartesian metrics remain deferred: computing TCP-pose RMSE needs
  FK, which is not wired in v1 (see ADR-0010). `compare.py` now
  surfaces cartesian combos as `not_yet_evaluated` so the gap is
  visible without pretending it's passing.
- `compare.py --aggregate-only` is the only mode implemented; live
  matrix dispatch is deferred behind a future `--live` flag.

## Recent commits

Run `git log --oneline -n 20` for the live list.
