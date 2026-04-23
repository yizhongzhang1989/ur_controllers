# Status

_Last updated: 2026-04-23 (feat(m5): metrics computation pipeline with
step/regulation/sine coverage, cartesian deferred, and 32 new unit
tests. Full test gate green in ~6:30.)_

## Current milestone

**M5 — evaluation and comparison harness: IN PROGRESS.** M0–M3 done.
M4 bullets 1–4 done; the optional M4 bullet 5 (second cartesian mode)
is parked on a human gate (sim F/T sensor patch on `auto_dev` —
unchanged from prior iteration). M5 bullets 1 + 2 + 3 done; bullet 4
(comparison report) is next.

M5 bullet 3 landed this iteration as `evaluation/compute_metrics.py`
(+ 32 new unit tests under `tests/unit/test_compute_metrics.py`).
The script consumes a run directory produced by M5 bullet 2
(`manifest.yaml`, `target.csv`, `joint_states.csv`, optionally
`tau_d.csv`), computes `rmse`, `settling_time`, `overshoot`, and
`control_effort`, and writes `metrics.yaml` + `metrics.csv` next to
the inputs. The CLI exits `0` when every `pass_criteria` key passes
or is skipped, `1` when any threshold is exceeded, `2` on malformed
inputs. Invocation:

```bash
python3 evaluation/compute_metrics.py --run-dir <run_dir>
```

Key observations from this iteration (relevant for future work):

- **Metric definitions in v1.**
  - `rmse` is joint-space RMSE per joint in rad, aggregated as the
    max across joints (the worst joint sets the bound).
  - `settling_time` is computed **only** for `scenario_type == step`
    with a 5%-of-amplitude tolerance band (floor 0.01 rad); reported
    as `inf` when a joint never settles before the end of the window.
    Non-step scenarios record it as `not_applicable` (skipped by the
    pass gate, never a failure).
  - `overshoot` is percent excess past the post-step target, signed
    by the direction of the step; step-only, same not-applicable rule
    as settling.
  - `control_effort` is **peak `|tau|`** across joints and time in
    N·m, read from `tau_d.csv`. When the controller doesn't publish
    `tau_d` (cartesian_motion), it is recorded as `skipped` and
    doesn't fail the pass gate.
- **Cartesian metrics deferred.** Cartesian regulation runs do not
  record a TCP pose trajectory — the observed signal is
  `/joint_states`, and converting that to a TCP pose needs FK on the
  URDF chain which would pull in `tf2`/`pinocchio`. For now the
  script emits a metrics file with every metric and every
  `pass_criteria` key recorded as `skipped` with a clear reason, and
  exits `0`. M5 bullet 4 (comparison report) will surface these as
  "not yet evaluated"; a future iteration will add FK-backed
  cartesian RMSE (likely via `tf2_ros` transform lookups against the
  sim's `/tf` tree, since the controller already publishes it).
- **Target / observed time alignment.** The live runner writes
  `target.csv` with `t_s = i / rate_hz` from scenario start and
  `joint_states.csv` with `t_s` from a monotonic clock that starts a
  bit later (after the 1 s settle sleep in `_run_live`). The metrics
  pipeline resamples observed onto the target time grid with linear
  interpolation + zero-order-hold clamping at the boundaries; the
  small wall-clock offset between the two CSVs is absorbed into that
  clamp and manifests as a slight RMSE bias for highly-dynamic
  scenarios. Good enough for regulation / step / slow sine; bullet 4
  will revisit if comparison numbers start disagreeing with the
  integration-test assertions.
- **Scenario doc is loaded from the referenced YAML, not the
  manifest.** The runner's manifest carries `scenario.name`,
  `.type`, `.metrics`, `.pass_criteria`, and a `path`, but not
  `command.per_joint_amplitude_rad` / `.step_time_s`. The metrics
  script re-opens the scenario YAML at `scenario.path` (resolved
  first against the repo root, then against the run dir) to recover
  the step amplitudes. If the scenario file moves or is renamed
  between run-time and metrics-time, `metrics.yaml` gets a clear
  `FileNotFoundError` and exits 2 — that's the intended signal.

## Last completed tasks

- **M5 bullet 3: `evaluation/compute_metrics.py`.** New files:
  `evaluation/compute_metrics.py` (CLI + metric primitives +
  pass-criteria evaluation), `tests/unit/test_compute_metrics.py`
  (32 cases across `interp_at`, `load_series`, RMSE / settling /
  overshoot / control-effort primitives, and the end-to-end
  `compute_metrics_for_run` / `main` CLI for step / regulation /
  sine / cartesian scenarios, including the fail-exit path). No
  changes to `run_evaluation.py`; the two scripts are split to let
  bullet 4 fan out runs and metrics independently.
- **M5 bullet 2: `evaluation/run_evaluation.py`.** See prior STATUS.
- **M5 bullet 1: `evaluation/scenarios/*.yaml` schema v1.** See prior STATUS.
- **M4 bullets 2 + 3 + 4: `cartesian_motion_controller` brought up on
  ur5e and ur15 with regulation integration test.** See prior STATUS.
- **M4 kick-off: `docs/cartesian_controllers.md` reference + tick M4
  bullet 1 (build).** See prior STATUS.
- **M3 — sim bring-up + regulation integration test for
  `simple_joint_impedance_controller` on `{ur5e, ur15}`.** See prior
  STATUS.
- **M3 — control law baseline in `simple_joint_impedance_controller`.**
  See prior STATUS.
- **Fix: strictly serialise MuJoCo controller spawners
  (`third_party/ur_simulator`).** See prior STATUS.

## Next task (agent should pick this up)

**M5 bullet 4: comparison report.** Wrap bullets 2 + 3 into a single
driver that runs each `{scenario, controller, robot}` combination
that passes `check_compatibility`, invokes
`evaluation/compute_metrics.py` on each resulting run dir, and emits
a summary table under `evaluation/reports/` (CSV + Markdown) with
one row per combination and one column per metric + pass status.
The existing test gate covers the metric math; the new driver
primarily needs integration-style coverage that its dispatch +
report emission works against synthetic run dirs (same tmp-path
pattern used by `test_compute_metrics.py`).

M4 bullet 5 (second cartesian mode) remains deferred behind a
**human gate** (sim-side F/T sensor patch on `auto_dev`).

## Build status

- ROS 2 distro: **Humble** (Ubuntu 22.04, matches system install).
- `colcon build --symlink-install --base-paths src third_party
  --packages-skip cartesian_controller_simulation cartesian_controller_tests`
  produces **10** packages successfully, unchanged from prior
  iteration (no new ROS packages added — this iteration only touched
  `evaluation/` and `tests/unit/`).

## Test status

- `scripts/run_tests.sh` runs unit → colcon test → integration. Green
  in ~6:30 this iteration (35 schema + 34 runner-dry-run + **32 new
  metrics** unit tests, 27 colcon gtests, 12 integration tests).
- Test counts: **101** pytest unit tests
  (`tests/unit/test_scenarios_schema.py` +
  `test_run_evaluation_dry.py` + `test_compute_metrics.py`) + **22**
  `test_math` gtests (simple_joint_impedance_controller) + **5**
  `crisp_controllers` gtests + **12** integration tests (sim smoke +
  3 crisp roles + our simple_joint_impedance_controller +
  `cartesian_motion_controller`, each ×{ur5e, ur15}).

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
  patch before they can be brought up end-to-end. Now directly
  gating M4 bullet 5 — surface to the operator when/if that bullet
  is picked up.
- Gravity-comp hook for `simple_joint_impedance_controller` remains
  unused (pure PD holds both arms within the M3 tolerance).
- The M5 runner's live path is not wired into `scripts/run_tests.sh`
  yet — it spins a full sim+controller per run and would add ~1 min
  per scenario × arm × controller, which is too heavy for the test
  gate. M5 bullet 4 will get a targeted smoke test once the
  comparison driver is in place.
- Cartesian metrics remain deferred: computing TCP-pose RMSE needs
  FK, which is not wired in v1 (see ADR-0010).

## Recent commits

Run `git log --oneline -n 20` for the live list.

