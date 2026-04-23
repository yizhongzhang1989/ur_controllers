# Status

_Last updated: 2026-04-23 (feat(m5): evaluation runner v1 with
scenario dispatch, dry-run path, and 34 new unit tests. Full test
gate green in ~6:30.)_

## Current milestone

**M5 — evaluation and comparison harness: IN PROGRESS.** M0–M3 done.
M4 bullets 1–4 done; the optional M4 bullet 5 (second cartesian mode)
is parked on a human gate (sim F/T sensor patch on `auto_dev` —
unchanged from prior iteration). M5 bullets 1 + 2 done; bullets 3
(metrics) and 4 (comparison report) are next.

M5 bullet 2 landed this iteration as `evaluation/run_evaluation.py`
(+ `evaluation/_reference.py` helper + 34 new unit tests under
`tests/unit/test_run_evaluation_dry.py`). The runner takes a
validated scenario YAML, a controller key (one of the five
bring-ups wired in M2–M4), a robot (`ur5e|ur15`) and an
`--out-dir`, launches the right sim mode + controller, records
`/joint_states` + the controller's `~/tau_d` into CSV, and writes a
`manifest.yaml` mirroring `evaluation/baselines/crisp/*.manifest.yaml`.

Key observations from this iteration (relevant for future work):

- **Controller registry.** The runner's `_CONTROLLERS` dict is the
  single place that maps a CLI key → (`sim_mode`, `bringup_launch`,
  `bringup_args`, `controller_node`, `displaced_controller`,
  `target_topic`, `has_tau_d`, `target_space`, `config_stem`).
  Five entries wired: `crisp_joint_impedance`,
  `crisp_cartesian_impedance`, `crisp_gravity_compensation`,
  `simple_joint_impedance`, `cartesian_motion`. Adding a new
  bring-up is a single dict entry plus a parametrisation in
  `tests/unit/test_run_evaluation_dry.py::
  test_controller_config_files_exist` /
  `..._bringup_files_exist`.
- **Target-publish path uses a child process.** The target publisher
  runs in a separate Python process (spawned with `setsid`) so the
  recorder's rclpy spin loop in the parent is not contended by
  another node in the same context. This matches the pattern in
  `scripts/record_crisp_baseline.sh` and lets us `SIGTERM` the whole
  PGID on teardown.
- **Cartesian scenarios in v1.** Cartesian regulation is the only
  schema-v1 cartesian `scenario_type` (ADR-0009). For cartesian
  controllers the runner writes a constant target CSV of the
  scheduled `hold` pose but does **not** drive a target topic:
  both `cartesian_motion_controller` (M4) and crisp's
  `cartesian_impedance_controller` latch their target at
  `on_activate` to the arm's current pose, which is exactly the
  regulation-hold behaviour we want. If/when a future scenario
  requires a driven cartesian target, `_run_live` grows a branch
  that publishes on `target_topic` using `PoseStamped` and pairs
  with a schema-v2 bump under a new ADR.
- **Dry-run path.** `--dry-run` loads + validates the scenario,
  generates the reference trajectory from `_reference.py`, and
  writes `target.csv` + `manifest.yaml` with no ROS imports. This
  keeps scenario-dispatch + manifest plumbing covered by the test
  gate (stage 1 pytest) without spinning a sim per scenario.
  `random_waypoints` with `bounds: urdf` rejects in dry-run because
  there is no URDF to query.
- **Output layout.** Runs land under
  `evaluation/runs/<scenario>__<controller>__<robot>__<UTC-ts>/`
  with `target.csv`, `joint_states.csv`, `tau_d.csv` (effort
  controllers only), and `manifest.yaml`. The whole
  `evaluation/runs/` tree is now gitignored (like
  `evaluation/reports/`).
- **`robot_description` propagation gotcha.** On Humble 2.53, the
  controller manager propagates its own `robot_description` param
  down to newly-loaded controllers only when the param was given to
  the CM at node construction. Our sim's `ur_sim_mujoco.launch.py`
  instead relies on the `/robot_description` topic path (remapping
  `~/robot_description` → `/robot_description`), which populates the
  CM's internal copy but does **not** push it to each controller.
  `CartesianControllerBase::on_configure` reads from its own node
  param so it sees `robot_description = ""` and aborts. Crisp dodges
  this by querying `/controller_manager` directly via a parameters
  client (see
  `third_party/crisp_controllers/src/cartesian_controller.cpp:242`).
  We cannot patch the read-only `cartesian_controllers` submodule
  (ADR-0003), so the launch fetches `robot_description` from
  `/robot_state_publisher` via rclpy and passes it to the spawner as
  a second `--param-file` (see
  `bringup/launch/cartesian_bringup.launch.py`). If/when a future
  controller with the same "reads from own node param" pattern
  lands, reuse this approach or fix the sim to pass
  `robot_description` as a CM-node param on `auto_dev`.

## Last completed tasks

- **M5 bullet 2: `evaluation/run_evaluation.py`.** New files:
  `evaluation/run_evaluation.py` (CLI + live + dry-run
  orchestration), `evaluation/_reference.py` (pure reference-
  trajectory generators for step/sine/regulation/random_waypoints),
  `tests/unit/test_run_evaluation_dry.py` (34 cases across reference
  generators, controller registry/compatibility, and the dry-run
  CLI end-to-end). Runs land under `evaluation/runs/` (gitignored).
  The live path uses `scripts/launch_sim.sh` + the per-controller
  bring-up launches already validated by M2–M4 integration tests;
  it's exercised manually and will be re-validated when M5 bullet
  3 (metrics) wires a real run into the harness.
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
- **M3 — `simple_joint_impedance_controller` package skeleton.** See
  prior STATUS.
- **M3 — ADR-0008: scope the simplified joint impedance controller.**
  See prior STATUS.

## Next task (agent should pick this up)

**M5 bullet 3: metrics computation.** Consume a run directory
(`evaluation/runs/<...>/{manifest.yaml, target.csv, joint_states.csv,
tau_d.csv}`) and compute the metrics listed under `scenario.metrics`
(`rmse`, `settling_time`, `overshoot`, `control_effort`). Emit a
`metrics.csv` + `metrics.yaml` next to the inputs, and return a
non-zero exit code when any threshold under `scenario.pass_criteria`
is exceeded. Target `rmse` + `control_effort` first (joint scenarios
use them in every committed example). Unit-test with synthetic CSVs
under `tests/unit/`. M5 bullet 4 (the comparison report) then wraps
bullets 2+3 to run each scenario × controller × arm and emit a
table.

M4 bullet 5 (second cartesian mode) remains deferred behind a
**human gate** (sim-side F/T sensor patch on `auto_dev`).

## Build status

- ROS 2 distro: **Humble** (Ubuntu 22.04, matches system install).
- `colcon build --symlink-install --base-paths src third_party
  --packages-skip cartesian_controller_simulation cartesian_controller_tests`
  produces **10** packages successfully, unchanged from prior
  iteration (no new ROS packages added — this iteration only touched
  `evaluation/`, `tests/unit/`, and `.gitignore`).

## Test status

- `scripts/run_tests.sh` runs unit → colcon test → integration. Green
  in ~6:30 this iteration (35 schema + **34 new runner** unit tests,
  27 colcon gtests, 12 integration tests).
- Test counts: **69** pytest unit tests
  (`tests/unit/test_scenarios_schema.py` + `test_run_evaluation_dry.py`)
  + **22** `test_math` gtests (simple_joint_impedance_controller) +
  **5** `crisp_controllers` gtests + **12** integration tests (sim
  smoke + 3 crisp roles + our simple_joint_impedance_controller +
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
  gate. It will get a targeted integration smoke test in a future
  iteration once M5 bullet 3 has a metric worth asserting on.

## Recent commits

Run `git log --oneline -n 20` for the live list.
