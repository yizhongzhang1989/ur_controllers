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

---

## ADR-0006 — Serialise controller spawners in MuJoCo sim launch

- **Date:** 2026-04-22
- **Status:** Accepted
- **Context:** `third_party/ur_simulator/src/ur_sim_config/launch/ur_sim_mujoco.launch.py`
  fired all six `controller_manager/spawner` processes concurrently at
  startup. On Humble + the default FastRTPS RMW this overflows the
  RMW response queue under peak load: the controller manager's reply
  to one spawner's `load_controller` call is silently dropped, the
  spawner retries the call 10 s later, and Humble's `load_controller`
  is **not** idempotent — the second attempt returns
  `A controller named X was already loaded`, causing the spawner to
  exit FATAL and leaving its controller un-activated. When the victim
  was `joint_state_broadcaster`, `/joint_states` never published and
  `tests/integration/test_sim_smoke.py::test_sim_bringup[ur5e]` failed.
  `ur15` happened to race luckier and usually passed; the bug was a
  startup-ordering race, not a per-arm issue.
- **Decision:** In `ur_sim_mujoco.launch.py`:
  1. Start `joint_state_broadcaster_spawner` alone first.
  2. Chain the other five controller spawners off its `OnProcessExit`,
     so at most one spawner is ever talking to the controller manager.
  3. Add `--service-call-timeout 30` to every spawner as defence in
     depth against slow responses.
- **Consequences:** Sim bringup is ~1–2 s slower (spawners now run
  sequentially instead of in parallel) but deterministic on both arms
  and every RMW. The fix lives on `third_party/ur_simulator`'s
  `auto_dev` branch; parent repo bumps the submodule pointer.

---

## ADR-0007 — crisp controllers own the effort interfaces exclusively

- **Date:** 2026-04-23
- **Status:** Accepted
- **Context:** The MuJoCo effort-mode sim ships with a
  `gravity_compensation.py` node that publishes gravity feed-forward +
  PID + external torque sums to `/forward_effort_controller/commands`.
  External controllers are meant to feed extra torques via
  `/external_effort_commands`, which the gravity node adds to its own
  output. That architecture assumes only `forward_effort_controller`
  ever claims the ros2_control `effort` command interfaces. CRISP's
  `CartesianController` (and any future torque controller) claims those
  same interfaces directly at `on_activate`, so it cannot coexist with
  `forward_effort_controller`.
- **Decision:** For every crisp role bring-up (joint_impedance,
  cartesian_impedance, gravity_compensation), `crisp_bringup.launch.py`
  performs a strict `switch_controllers --deactivate
  forward_effort_controller --activate <role>` once the role's
  controller has been loaded `--inactive`. While crisp is active it is
  the sole effort commander; the sim's `gravity_compensation.py` keeps
  running but its publishes land on an inactive controller and are
  ignored. Crisp provides its own gravity + Coriolis compensation via
  pinocchio (`use_gravity_compensation: true`,
  `use_coriolis_compensation: true` in every role yaml under
  `bringup/config/crisp_*.{ur5e,ur15}.yaml`).
- **Consequences:** We do not use `/external_effort_commands` in this
  phase — that path is reserved for the sim's own
  `forward_effort_controller`. Tests that bring up a crisp role must
  assert both `forward_effort_controller=inactive` and
  `<role>=active` so a partial swap cannot silently leave two
  gravity-comp sources fighting for the arm. When M3's own simple joint
  impedance lands, it will follow the same pattern (be the only effort
  commander while active, compute its own gravity comp).

---

## ADR-0008 — Scope of our simplified joint impedance controller (M3)

- **Date:** 2026-04-23
- **Status:** Accepted
- **Context:** M3 calls for a lightweight, in-tree controller modelled on
  the `crisp_controllers/CartesianController` plugin running in its
  `joint_impedance_controller` role (see `docs/crisp_controllers.md` and
  the configs under `bringup/config/crisp_joint_impedance.{ur5e,ur15}.yaml`).
  The upstream class is ~750 lines of C++ plus ~320 lines of parameter
  schema and pulls in pinocchio, the full Cartesian/OSC task with per-axis
  error clipping, three flavours of nullspace projector, EMA filters on
  target/state/output, a 7-DOF friction model, joint-limit repulsion,
  output torque rate saturation, and noise injection. Most of that
  machinery is inert in the joint-impedance role we actually run:
  `task.k_*=0` zeroes the Cartesian branch entirely, and
  `nullspace.projector_type: none` makes the nullspace PD the whole control
  law. For M3 we want a minimal controller we fully own and can reason
  about end-to-end — not a re-implementation of the whole plugin.
- **Decision:** Implement `src/simple_joint_impedance_controller/` as a
  `controller_interface::ControllerInterface` plugin with the following
  feature split relative to crisp's joint-impedance configuration.

  ### Keep (baseline must-haves)

  1. **Pure joint-space PD control law:**
     `tau_cmd = K (q_d - q) - D qdot`, computed per joint with diagonal
     `K` and `D`. Corresponds to crisp's nullspace term when
     `projector_type: none`, `task.k_*=0`. Sole commander on
     `<joint>/effort`.
  2. **State interfaces:** `<joint>/{position, velocity}` only
     (match crisp's joint-impedance role).
  3. **Per-joint gains:** `K` and `D` are `double_array` parameters the
     length of `joints`; negative `D[i]` auto-fills `2*sqrt(K[i])`
     (same convention as crisp's `nullspace.damping: -1.0`).
  4. **Target topic:** subscribe to `~/target_joint`
     (`sensor_msgs/JointState`); use its `position` field as `q_d` and,
     when populated and the right length, its `velocity` field as
     `qdot_d` (defaults to zero). On activation, seed `q_d` to the
     measured `q` so we hold position — this is what crisp does in its
     joint role (see `tests/integration/test_crisp_joint_impedance.py`).
  5. **Torque saturation (absolute):** per-joint `tau_max` clamp applied
     last. Maps to crisp's `limit_torques` + `nullspace.max_tau`.
     Safety-critical → mandatory.
  6. **Torque rate saturation:** per-joint `max_delta_tau` clamp between
     successive control cycles. Maps to crisp's `max_delta_tau` /
     `saturateTorqueRate`. Prevents effort-interface step jumps that the
     MuJoCo sim and real UR drivers both dislike. Safety-adjacent →
     mandatory.
  7. **Target validation:** reject `target_joint` messages whose joint
     names (when non-empty) don't match `joints` or whose position array
     length is wrong; clamp each `q_d[i]` to the URDF joint limits so a
     bad command can't drive the arm into a hard stop. Simpler, stricter
     analogue of crisp's `limit_error` + `joint_limit_repulsion`
     combination (see drop-list below).
  8. **Parameter handling:** `generate_parameter_library` for everything
     above (AGENTS.md §4).
  9. **Diagnostics:** publish commanded torque on `~/tau_d`
     (`sensor_msgs/JointState`, same topic shape as crisp) so existing
     test/eval tooling transfers one-to-one.

  ### Drop (explicitly not in M3 scope)

  1. **Cartesian / OSC task branch** (`task.k_*`, `task.d_*`,
     `task.error_clip`, `use_operational_space`,
     `operational_space_regularization`) — whole Cartesian side of the
     plugin. Our controller is joint-space only; anyone needing
     Cartesian uses crisp or the M4 cartesian_controllers.
  2. **Pinocchio dependency** and all model-based terms:
     - **Gravity compensation.** Drop for M3 baseline. UR's effort
       interface in `ur_simulator` MuJoCo mode is backed by the sim's
       own `gravity_compensation.py` node (ADR-0007); our controller
       will be the sole effort commander while active, so gravity must
       be re-added later **if the integration test fails without it**.
       Treat this as a follow-up per ROADMAP M3 line ("Optional gravity-
       comp hook only if M2 showed it is needed to match crisp") — we
       will first try pure PD with gains tuned high enough to hold
       against gravity within the same bounded-error thresholds used in
       `tests/integration/test_crisp_joint_impedance.py`; if that fails
       we add a minimal KDL- or urdf-based gravity hook in a follow-up
       commit and supersede this bullet with a new ADR.
     - **Coriolis compensation.** Drop. At UR5e/UR15 quasi-static
       regulation speeds the Coriolis torque is negligible.
     - **Nullspace projector machinery** (`kinematic`, `dynamic`,
       `weights`, `regularization`, Jacobian pseudo-inverse). Not
       meaningful without a Cartesian task.
     - **Local vs world Jacobian** (`use_local_jacobian`). Ditto.
  3. **Friction model** (`use_friction`, `friction.fp1/fp2/fp3`). The
     upstream defaults are Franka-calibrated 7-vectors; they are the
     wrong length for a 6-DOF UR and would need re-identification.
     Drop for M3.
  4. **EMA filters** on target pose, `q`, `dq`, `q_ref`, and output
     torque (`filter.*`). The sim + real UR publish at ≥125 Hz with
     clean signals; an extra first-order filter adds phase lag and
     another tuning knob with no measurable win at M3's regulation
     scenarios. Drop.
  5. **Joint-limit repulsion** (`joint_limit_repulsion.*`). Replaced
     with the simpler, deterministic approach of clamping `q_d` into
     the URDF limits (see keep-list #7). Avoids the tuning surface of
     a soft repulsive torque field at M3 gains.
  6. **Per-axis error clip** (`task.error_clip.*`). Not applicable
     without a Cartesian task.
  7. **Noise injection** (`noise.*`). Debug/stability test tool, not
     needed for production control.
  8. **Logging and introspection flags** (`log.*`, `enable_introspection`).
     Out of scope for M3. Rely on `~/tau_d` + standard `/rosout` logging.
  9. **`stop_commands`** safety flag. Redundant with `ros2_control`'s
     own `deactivate` / `switch_controllers` mechanism (ADR-0007).
  10. **`TorqueFeedbackController` / broadcaster plugins.** Separate
      classes in crisp; unrelated to M3.

  ### Consequences

  - Implementation budget is ≈200 lines of C++ + a small yaml param
    schema (vs ~750 + ~320 for the crisp plugin). No pinocchio /
    Eigen-heavy math dependencies beyond what `ros2_control` already
    pulls in.
  - The controller is launch-compatible with the crisp bring-up pattern:
    one `simple_jimp_bringup.launch.py` mirroring
    `bringup/launch/crisp_bringup.launch.py` so the M5 comparison
    harness can flip between controllers with a single flag (ROADMAP
    M3 last item).
  - Tests plan: (a) gtest unit tests on the PD + torque/rate saturation
    math with no ROS, (b) integration tests parametrised over
    `{ur5e, ur15}` with the same bounded-tracking-error thresholds as
    `tests/integration/test_crisp_joint_impedance.py` so a direct
    comparison in M5 is trivial.
  - If the integration test fails on either arm purely because of
    sagging under gravity, the gravity-comp hook listed under
    "Drop #2" becomes a follow-up item with its own ADR; this ADR
    does **not** pre-approve that change, only flags it as the first
    extension to consider.


---

## ADR-0009 — Evaluation scenario schema v1 (M5)

- **Date:** 2026-04-23
- **Status:** Accepted
- **Context:** ROADMAP M5 needs a declarative way to describe evaluation
  scenarios so the same file drives every controller from M2–M4
  (`crisp_*`, `simple_joint_impedance_controller`,
  `cartesian_motion_controller`) on both `ur5e` and `ur15` (per ADR-0004).
  M5 bullet 1 is the schema; bullets 2–4 are the runner, the metrics, and
  the comparison report. The schema must therefore be controller-agnostic
  and arm-agnostic, mechanically validatable, and stable enough that
  bullets 2–4 do not force a schema rev.
- **Decision:**
  - Schema lives in `evaluation/scenarios/README.md` and is enforced by
    `evaluation/scenarios/validate.py` (PyYAML + stdlib only — no
    `jsonschema` dep). Schema version is an explicit `schema_version: 1`
    integer field; future incompatible changes bump this and supersede
    this ADR.
  - Top-level shape splits **what** is being commanded (`target`: space,
    joints / frame + end_effector) from **how** it is being driven
    (`command`: per-`scenario_type` parameters). New scenario_types
    can be added without touching `target`.
  - `scenario_type` is one of `step | sine | regulation |
    random_waypoints`. `step`, `sine`, `random_waypoints` are
    joint-space only in v1. `regulation` accepts both joint and
    cartesian targets; cartesian regulation is specified as
    `position_xyz_m` + unit `orientation_xyzw` in `target.frame_id`.
    Cartesian step / sine / random are deferred — they need explicit
    interpolation + frame semantics that we have not committed to and
    that no current bring-up exercises.
  - Joint vectors are always length 6 in the canonical UR joint order
    (`shoulder_pan, shoulder_lift, elbow, wrist_1..3`). Scalar
    `amplitude_rad` is **rejected** for `step` / `sine` because it
    leaves the active joint ambiguous; `per_joint_amplitude_rad: list[6]`
    with zeros for inactive joints is the only accepted form.
  - Time consistency is validated up front: `step_time_s < duration_s`
    and `num_waypoints * dwell_s <= duration_s`.
  - `pass_criteria` keys (e.g. `max_settling_time_s`) must each map to
    a metric listed under `metrics`, so a threshold can never reference
    a metric that nothing computes. Joint vs cartesian thresholds are
    enforced separately (`max_rmse_rad` vs `max_rmse_m`).
- **Consequences:**
  - M5 bullets 2–4 read scenarios via the same loader; per-scenario_type
    dispatch is a `match` on `scenario_type` once validation has passed.
  - Adding a new scenario_type or a new metric is a single change to
    `validate.py` + a new bullet in `README.md`; existing files keep
    working because the validator does not reject *unknown* scenario_type
    only at top-level (it does — see "rejected"), forcing the rev to
    pass through this ADR-replacement gate.
  - The validator is exercised by `tests/unit/test_scenarios_schema.py`
    (35 cases). Adding cartesian step/sine/random later will require a
    new ADR superseding this one and bumping `schema_version` to 2.

---

## ADR-0010 — Evaluation metrics v1: joint-space only; cartesian deferred

- **Date:** 2026-04-23
- **Status:** Accepted
- **Context:** ROADMAP M5 bullet 3 asks for metrics computation over run
  directories produced by `evaluation/run_evaluation.py` (bullet 2). The
  schema-v1 pass-criteria keys (ADR-0009) include `max_rmse_rad`,
  `max_settling_time_s`, `max_overshoot_pct`, `max_control_effort_nm`
  for joint scenarios, and `max_rmse_m` + `max_rmse_rad` (orientation)
  for cartesian scenarios. Joint-space metrics are a direct function of
  `target.csv` + `joint_states.csv`. Cartesian RMSE requires converting
  `/joint_states` into a TCP pose via forward kinematics on the arm's
  URDF chain, which needs either `tf2_ros` transform lookups against
  the live `/tf` tree (only available mid-run) or `pinocchio` /
  `kdl_parser` in the offline pipeline — a non-trivial addition that
  would delay bullet 4 (comparison report).
- **Decision:**
  - `evaluation/compute_metrics.py` v1 implements
    `rmse` / `settling_time` / `overshoot` / `control_effort` for
    **joint-space** scenarios only.
    - `rmse` — per-joint RMSE in rad, aggregated as the max across
      joints (worst joint sets the bound).
    - `settling_time` + `overshoot` — computed **only** for
      `scenario_type == step`; a 5%-of-|amplitude| tolerance band with
      a 0.01 rad floor is used for settling. Non-step scenarios record
      both as `not_applicable`, and their corresponding pass-criteria
      keys are marked `skipped` (never a failure).
    - `control_effort` — **peak `|tau|`** across joints and time in
      N·m, read from `tau_d.csv`. Absent `tau_d.csv`
      (cartesian_motion_controller) → `skipped`.
  - For `target.space == cartesian`, the script emits a metrics file
    with every metric and every pass-criteria key recorded as
    `skipped` with a clear reason and exits 0. M5 bullet 4 will
    surface these as "not yet evaluated" in the comparison report.
  - Output artefacts are `<run_dir>/metrics.yaml` (structured) and
    `<run_dir>/metrics.csv` (long-format, one row per metric × joint
    + per pass key). Exit codes: `0` all-pass/skip, `1` any
    threshold exceeded, `2` malformed inputs.
- **Consequences:**
  - Joint-space scenarios (`step`, `regulation`, `sine`,
    `random_waypoints`) are fully graded by the v1 pipeline; the
    existing committed example scenarios all fall in this bucket.
  - Cartesian comparison is punted to a follow-up that will either
    add offline FK (likely via `pinocchio` since crisp already pulls
    it in as a dep) or extend the runner to record TCP pose from
    `/tf` during the run. When that lands, this ADR is superseded by
    an ADR-00XX and the cartesian branch of
    `compute_metrics_for_run` becomes a real computation.
  - The `control_effort` semantics (peak vs integral) is fixed to
    **peak** in v1 because the pass-criteria unit is `_nm` (N·m) not
    `_nms` (N·m·s); switching to integral later would change the
    threshold numbers in committed scenario files, which is a
    schema-version-visible change and must go through a new ADR.

---

## ADR-0011 — Comparison report driver: aggregate-only, latest-valid, cartesian deferred at the report layer (M5)

- **Date:** 2026-04-23
- **Status:** Accepted
- **Context:** ROADMAP M5 bullet 4 asks for a comparison report that
  wraps bullets 2 + 3. Three design questions had to be answered:
  (a) does the driver also drive live sim runs, or only aggregate
  existing run dirs? (b) how does it pick a run dir when multiple
  match a `{scenario, controller, robot}` combination, given that
  `run_evaluation.py` creates the run dir *before* the live run
  completes and so can leave partial directories behind? (c) how
  does the report surface cartesian combos, which
  `compute_metrics_for_run` returns with `overall_status=pass` (every
  key `skipped`) but which ADR-0010 explicitly says are "not yet
  evaluated"?
- **Decision:**
  - **Aggregate-only by default.** `evaluation/compare.py` has no
    `--live` flag in v1. The test gate synthesises run dirs in
    tmp_path and exercises the aggregation path end-to-end; a full
    matrix of live sim runs (~1 min × 20 combos) is too heavy for
    that gate. Operators (or a follow-up outer script) drive
    `run_evaluation.py` themselves to populate `evaluation/runs/`,
    then call `compare.py` to build the report. When live dispatch
    does land it will subprocess `run_evaluation.py` for ROS/process
    isolation while continuing to import `compute_metrics_for_run`
    directly.
  - **Latest-valid run dir wins.** Matching run dirs are sorted
    newest → oldest by name (UTC timestamp
    `YYYYMMDDTHHMMSSZ` is lexicographically sortable); the driver
    picks the first one that contains a `manifest.yaml`
    (the runner writes it last, so its presence is a soundness
    check). This prevents a partial/failed newer run from masking an
    older successful one.
  - **Report-layer `not_yet_evaluated` for cartesian.** The report
    overrides `overall_status` to `not_yet_evaluated` for combos
    with `target.space == cartesian`, regardless of what
    `compute_metrics_for_run` returned. Keeping the override at the
    report layer (not inside `compute_metrics.py`) lets the metrics
    CLI stay stable — its `overall_status` still means
    "every pass key passed or was skipped" — while the human-
    facing report correctly flags the FK gap. Rows also carry the
    reason "cartesian metrics deferred in v1 (ADR-0010)".
  - **Exit codes.** `0` when every compatible combo is
    `pass` / `not_yet_evaluated`; `1` when any combo is
    `fail` / `no_run` / `metrics_error`; `2` on driver misuse.
    `no_run` for joint combos is a failure because a missing joint
    run means the matrix is incomplete. Cartesian combos without a
    run stay `not_yet_evaluated` (the FK gap is the dominant
    reason, not the missing bag).
- **Consequences:**
  - `compare.py` imports `run_evaluation.py` and `compute_metrics.py`
    via `importlib.util.spec_from_file_location` (matching
    `run_evaluation.py`'s own pattern) so `evaluation/` stays a plain
    directory, not a Python package.
  - The outer loop can now run `python3 evaluation/compare.py` as a
    cheap read-only health check of the `evaluation/runs/` tree —
    even with no runs present it emits a valid report and exits 1 to
    signal the matrix is empty.
  - When cartesian FK metrics land (future ADR superseding ADR-0010),
    the cartesian override in `compare.py` must be dropped in the
    same change so the report starts showing real numbers.

---

## ADR-0012 — Unified MuJoCo sim: expose all command interfaces, switch at runtime

- **Date:** 2026-04-24
- **Status:** Proposed (operator review pending — see M6.0 gate in
  `docs/ROADMAP.md`).
- **Context:**
  - Today the sim is launch-locked to a single control mode
    (`--control_mode {position,effort}`). `generate_mujoco_model.sh`
    emits *either* `<position>` actuators *or* `<motor>` actuators,
    and the URDF `<ros2_control>` block declares the matching single
    command interface. Consequence: `joint_trajectory_controller`
    with `command_interfaces: [position]` cannot activate against an
    effort-mode launch, and `forward_effort_controller` cannot
    activate against a position-mode launch. Runtime controller
    switching is therefore restricted to controllers that share the
    launch-time interface (SHORTCOMINGS.md §2; ROADMAP M6 context).
  - On a real UR, `ur_robot_driver` exposes position, velocity, and
    effort command interfaces simultaneously. `ros2 control
    switch_controllers` is the only mechanism needed to change
    control mode. This is the interface surface we want to match.
  - MuJoCo itself can host multiple actuators per joint (torques sum
    into `qfrc_actuator`). The block is not physics, it is (a) the
    MJCF-is-generated-per-mode convention in
    `generate_mujoco_model.sh`, and (b) the `mujoco_ros2_control`
    plugin reading whichever interface the URDF declares and
    forwarding into the matching actuator. Both are our code to
    change (sim) / patch (external package).
  - Downstream consumers (crisp, simple_jimp, cartesian_motion, JTC,
    MoveIt) all target the same ros2_control APIs. If the sim
    matches the real UR at that API, none of them need per-platform
    YAML forks.
- **Decision:**
  1. **Unified interface surface.** MuJoCo sim exports, per joint,
     `command_interface {position, velocity, effort}` and
     `state_interface {position, velocity, effort}`, matching
     `ur_robot_driver`. No more `control_mode` launch gating of the
     interface set.
  2. **Three actuators per joint in MJCF.**
     - `<position name="J_pos" joint="J" kp="KP" kv="KV" forcerange="..."/>`
       for the position command interface. `KP`/`KV` stay at the
       current per-joint values (ITERATION_LOG.md round 3 tuning),
       but are now soft enough that on unclaim (§Transition rules)
       the PD term contributes ~0 torque.
     - `<velocity name="J_vel" joint="J" kv="..." forcerange="..."/>`
       for the velocity command interface.
     - `<motor name="J_eff" joint="J" ctrlrange="..."/>` for the
       effort command interface. `ctrlrange` = per-joint torque
       limit from `config/ur_types/<arm>.yaml`.
  3. **Claim-aware ctrl routing (`mujoco_ros2_control` patch).** On
     every update tick, for every joint:
     - If the `position` command interface is claimed: forward the
       commanded q_d into `J_pos.ctrl`; zero `J_eff.ctrl`; set
       `J_vel.ctrl = 0`.
     - If `effort` is claimed: forward commanded τ_d into
       `J_eff.ctrl`; **snap `J_pos.ctrl = q` (current position)** so
       the position PD contributes ~0 torque; set `J_vel.ctrl = 0`.
     - If `velocity` is claimed: forward commanded q̇_d into
       `J_vel.ctrl`; snap `J_pos.ctrl = q`; zero `J_eff.ctrl`.
     - If none is claimed: snap `J_pos.ctrl = q`, zero `J_vel` and
       `J_eff` — arm holds position via the position PD alone (this
       is the "safe idle" behaviour chosen to mirror a real UR
       with no controller active).
     This routing is implemented in the plugin, not in user-space
     nodes. Two ctrl surfaces are always zero or state-snapped; only
     one carries the active command.
  4. **Mutual exclusion relies on ros2_control.** ros2_control
     already forbids two active controllers from claiming the same
     command interface on the same joint. The plugin trusts that
     invariant; it never forwards a stale command from an unclaimed
     interface. Dual-claim (e.g. position + effort on the same
     joint simultaneously) is **not** supported in M6; cooperative
     position + torque requires chainable controllers, which is a
     separate future ADR.
  5. **Single controllers YAML.** Sim config collapses to one
     `ur_controllers.yaml` aligned with upstream UR. Controllers
     that are loaded-but-inactive at startup match the upstream
     default set; launch arg `--default-controller` picks which
     command controller is `active` at t=0 (default:
     `scaled_joint_trajectory_controller`, matching real-UR +
     MoveIt).
  6. **ADR-0007 transitions.** ADR-0007 ("crisp controllers own the
     effort interfaces exclusively") is *upheld as a runtime
     invariant* but no longer needs special launch-time handling:
     activating a crisp role deactivates `forward_effort_controller`
     via `switch_controllers --strict` (which is what the existing
     bringup already does). The sim's `gravity_compensation.py`
     shim becomes a development aid, not part of the default
     launch — see M6.7.
  7. **Real-UR parity is a goal, not a guarantee.** The sim's
     position interface is a pure MuJoCo PD; the real UR position
     interface is the outer of a cascaded loop with integral
     action, gearbox compliance, and current saturation. Matching
     *interfaces* gives portable YAMLs; matching *dynamics* is a
     separate modelling task (armature, joint damping/friction,
     torque-tracking lag — listed as a follow-up, not part of M6).
- **Decisions to gate (operator, M6.0):**
  - (a) **Vendor `mujoco_ros2_control` as a submodule under
    `third_party/mujoco_ros2_control`**, tracked on an `auto_dev`
    branch analogous to `ur_simulator` (ADR-0003). Pro: same
    ownership model as `ur_simulator`; clean upstreaming path. Con:
    new submodule = human gate (AGENTS.md §7).
  - (b) **Overlay patch package** under `third_party/` that replaces
    the plugin via package-name shadowing. Pro: no submodule bump.
    Con: fragile against upstream version bumps; harder to
    upstream.
  - (c) **Patch locally inside `ur_simulator@auto_dev` (treating
    `mujoco_ros2_control` as a vendored dep inside that repo)**.
    Pro: uses the submodule we already own. Con: conflates sim
    config and the ros2_control plugin in one repo; may bloat the
    sim.
  - Recommended: **(a)**. Matches ADR-0003's ownership discipline
    and keeps the sim repo focused on configs + MJCF generation.
  - Operator also to confirm: target UR driver version for
    interface parity (Humble `ur_robot_driver` 2.4.x exposes effort;
    2.3.x does not). Recorded as a fact under the chosen option.
- **Transition rules (summary for implementers):**
  - Switching *into* a position controller: no action needed
    beyond the routing in §3 — the position actuator's `ctrl` is
    overwritten the same tick.
  - Switching *out of* a position controller: the next tick the
    plugin must set `J_pos.ctrl = q`. Skipping this step will leave
    the arm being pulled toward a stale `q_d` while an
    effort/velocity controller thinks it has sole authority.
  - Switching between effort ⇄ velocity: position actuator stays
    snapped to `q`; the other two actuators swap which carries the
    command. No mid-tick fight because ros2_control guarantees the
    claim change is atomic w.r.t. the plugin's `read()/write()`.
- **Consequences:**
  - The user-facing CLI flag `--control_mode` is deprecated to
    `--default-controller`. `scripts/launch_sim.sh` and all bringup
    launch files drop the dichotomy.
  - Integration tests parametrised over `{ur5e, ur15}` must be
    extended with a runtime-switch test (M6.6). The existing
    effort-mode tests (crisp, simple_jimp) become runtime switches
    from the default JTC to the effort controller, rather than
    launches in a dedicated effort mode.
  - `docs/SHORTCOMINGS.md` §2 (S1) becomes obsolete and should be
    struck when M6 lands.
  - The sim's F/T sensor work (M4 bullet 5 blocker) is orthogonal
    to M6 and remains operator-gated; M6 does not unblock it.
  - If operator picks option (a), a new `.gitmodules` entry appears
    and CI's `rosdep` skip-keys gain `mujoco_ros2_control` (we
    build it from source).


---

## ADR-0012 addendum (2026-04-24) — R1, R2, R3 acceptance requirements

Operator added three hard requirements to M6 after the initial
ADR-0012 draft. They are recorded here, inline with the ADR they
extend, rather than as a separate ADR because they refine — not
supersede — the design.

- **R1 — Sim/real binary parity of controllers.** ADR-0012's
  "unified interface surface" decision is upgraded from a goal to a
  **hard constraint**: zero config diff between sim and real for any
  controller. Practical consequences:
  1. Controller YAMLs in `bringup/config/` are the single source of
     truth and carry no sim-vs-real branching. If a parameter must
     differ (e.g. a gain), the branching happens outside the YAML
     (per-arm YAML, or loaded from a profile), never per-driver.
  2. The sim's hardware-interface plugin (`mujoco_ros2_control`,
     post-M6.4 patch) must emit interface names byte-identical to
     `ur_robot_driver`. `docs/real_driver_parity.md` (new, M6.10) is
     the verification artefact.
  3. `sim_broadcasters.py` topics must be renamed if they collide
     with real-driver names; otherwise they publish stub values on
     the same topic names and ADR-0012's "safe idle" behaviour
     applies. Audit in M6.10.
  4. Gravity-comp shim (`gravity_compensation.py`) is
     disabled-by-default post-M6 (M6.7). It may still be enabled
     explicitly for debugging, but is never in the real-parity
     launch path.

- **R2 — Structured test matrix.** The integration test story
  expands from "regulation holds within 0.15 rad" to a three-stage
  matrix (single-joint → all-joints → end-effector), each
  parametrised over `{ur5e, ur15} × payload ∈ {none, small, large}`
  (R3). Every test records its **theoretical expectation** alongside
  the measured result, and fails if the gap exceeds the declared
  tolerance.
  - Theoretical baselines live in
    `tests/integration/expectations/{ur5e,ur15}.yaml` and
    `expectations/payloads.yaml`.
  - Expected responses are computed from first principles
    (second-order impedance law, rigid-body FK), not measured; the
    test harness runs the formula at assertion time so changing a
    stiffness updates the expected curve automatically.
  - "No drift" (position mode) is quantified as ≤ 1e-3 rad
    steady-state joint drift over a 10 s hold. "No chatter" is a
    velocity RMS threshold in the last 2 s of each test. "No
    limit-cycle oscillation" is an FFT check: no peak in the 1 Hz–
    Nyquist band above the measurement noise floor by more than
    6 dB. Concrete thresholds live in the expectations YAML so the
    agent can tune them per arm without touching assertion code.
  - Damping-ratio-from-step is the single most informative check:
    it directly verifies the closed-loop dynamics the YAML claims.
    Deviation > ±20% between measured and predicted ζ fails the
    test — same tolerance the simple_jimp gtest currently uses for
    its critical-damping auto-fill.

- **R3 — Configurable end-effector payload.** Every integration
  test runs at three payload levels. The sim must physically
  simulate the payload (via MJCF body attachment), visualise it
  (three.js cube anchored at `tool0`), and publish it to
  controllers (latched `/ee_payload`, service
  `~/set_ee_payload`). Key design points:
  1. **Message type.** New `ur_sim_msgs/EePayload` with `mass`
     (double, kg), `inertia` (9 doubles, kg·m², row-major, at the
     payload's COM and frame), and `pose`
     (`geometry_msgs/Pose`, relative to `tool0`). Rationale:
     matches `sensor_msgs/Imu`-style tensor packing and mirrors
     the arguments of UR's `set_payload` URScript function.
  2. **Initial state.** Zero mass on sim launch. This matches
     what `ur_robot_driver` publishes before an operator runs
     `set_payload`, preserving R1.
  3. **Runtime attachment in MuJoCo.** MuJoCo does not allow
     adding `<body>` elements post-`mj_loadXML`. Two options,
     first chosen for M6.17:
     - (A) Regenerate the MJCF whenever payload changes and
       reload the model. Works; costs ~50–200 ms and a brief
       physics pause. Acceptable for interactive dashboard use,
       not for closed-loop payload adaptation (out of scope).
     - (B) Pre-declare a "payload slot" body at MJCF generation
       time with zero mass and placeholder geom, then mutate
       `mj_model.body_mass`, `body_inertia`, and
       `body_pos`/`body_quat` at runtime via the plugin. No
       reload, no pause. Decision: **start with (A)** for M6.17
       (simpler, no plugin knowledge needed); migrate to (B) if
       the dashboard UX demands seamless updates.
  4. **Visualisation.** Cube edge length = $(m / \rho)^{1/3}$ with
     $\rho = 1000\,\text{kg/m}^3$ default, overridable in the
     dashboard. Wire-frame only, so it never occludes the arm.
     Cube colour: red if the currently-active controller's config
     has `use_gravity_compensation: false` or is unaware of
     payload, green otherwise. This gives the user an immediate
     visual cue when they add payload to a non-compensated
     controller and should expect drift.
  5. **Payload levels in tests.**
     - `no_payload`: mass = 0.
     - `small_payload`: 1.0 kg cube, inertia diag(8.3e-4, 8.3e-4,
       8.3e-4) for a 0.1 m edge, pose = identity at `tool0`.
     - `large_payload`: 5.0 kg cube, inertia diag(2.1e-2, 2.1e-2,
       2.1e-2) for a 0.2 m edge, pose = translation (0, 0, 0.1) m
       along tool-Z. 5 kg is within UR5e rated payload and well
       within UR15's; pick arm-specific values only if the large
       case exceeds the arm's limit (will be flagged by the
       tolerance-from-expectation check).
- **Consequences of R1/R2/R3 on the M6 bullet order.**
  - M6.10, M6.11 (R1) land early so every subsequent bullet is
    verified against the real driver's interface spec.
  - M6.15 (expectations YAML) must land before M6.12–M6.14 so the
    assertions have a source of truth.
  - M6.16 (payload message + API) must land before M6.17–M6.19 so
    the plumbing exists before the physics and UX bullets.
  - M6.19 (payload-parametrised tests) is the final bullet before
    M6.9 (submodule bump).
