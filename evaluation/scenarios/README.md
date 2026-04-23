# Evaluation scenarios — schema v1

This directory holds declarative descriptions of evaluation scenarios used by
the M5 harness (`evaluation/run_evaluation.py`, to be added) to drive any of
the controllers brought up in M2–M4 against the simulator and to score the
resulting trajectories.

A scenario file is a single YAML document that describes:

- **what** to command (`target` + `command`),
- **for how long** (`duration_s`),
- **on which arms** (`robots`),
- **how to grade the run** (`metrics`, `pass_criteria`).

A scenario is intentionally **controller-agnostic** and **arm-agnostic**: the
same file is run against `crisp_joint_impedance_controller`,
`simple_joint_impedance_controller`, `cartesian_motion_controller`, …, on both
`ur5e` and `ur15`, and the same `pass_criteria` apply (per ADR-0004; gains
may differ per arm, thresholds do not).

The canonical UR joint order, used everywhere a per-joint vector appears, is:

```
[shoulder_pan_joint, shoulder_lift_joint, elbow_joint,
 wrist_1_joint, wrist_2_joint, wrist_3_joint]
```

## Top-level fields

| Field            | Type        | Required | Notes                                                                           |
|------------------|-------------|----------|---------------------------------------------------------------------------------|
| `schema_version` | int         | yes      | Must be `1`.                                                                    |
| `name`           | str         | yes      | Kebab-case. Should match the file stem.                                         |
| `description`    | str         | yes      | One-line human description.                                                     |
| `scenario_type`  | str         | yes      | One of `step`, `sine`, `regulation`, `random_waypoints`.                        |
| `duration_s`     | float > 0   | yes      | Total scenario duration, seconds.                                               |
| `robots`         | list[str]   | yes      | Non-empty subset of `{ur5e, ur15}`, no duplicates.                              |
| `target`         | mapping     | yes      | What is being commanded; see below.                                             |
| `command`        | mapping     | yes      | Parameters of the `scenario_type`; see below.                                   |
| `metrics`        | list[str]   | yes      | Non-empty subset of `{rmse, settling_time, overshoot, control_effort}`, unique. |
| `pass_criteria`  | mapping     | yes      | Numeric thresholds; keys depend on `target.space`. See below.                   |

### `target`

```yaml
target:
  space: joint | cartesian      # required
  joints: [...]                  # required iff space==joint; must equal the canonical 6-joint list
  frame_id: <string>             # required iff space==cartesian; e.g. "base"
  end_effector: <string>         # required iff space==cartesian; e.g. "tool0"
```

### `command` — per `scenario_type`

#### `step`

```yaml
command:
  per_joint_amplitude_rad: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]   # length 6
  step_time_s: <float, 0 <= step_time_s < duration_s>
```

The reference equals the initial pose for `t < step_time_s` and equals
`initial + per_joint_amplitude_rad` afterwards. A scalar `amplitude_rad` is
**not** accepted in v1 because it is ambiguous given the fixed 6-joint order
(we would not know *which* joint to move); use a length-6 vector with the
inactive joints set to `0.0`.

Only valid when `target.space == joint`.

#### `sine`

```yaml
command:
  per_joint_amplitude_rad: [0.0, 0.05, 0.0, 0.0, 0.0, 0.0]
  frequency_hz: <float > 0>
  phase_rad: <float, default 0.0>
```

Reference is `initial + amplitude * sin(2*pi*frequency*t + phase)` per joint.
Only valid when `target.space == joint`.

#### `regulation`

```yaml
command:
  hold: initial                  # OR an explicit pose:
  # hold: [q1, q2, q3, q4, q5, q6]            # space == joint, length 6
  # hold:                                      # space == cartesian
  #   position_xyz_m:  [x, y, z]
  #   orientation_xyzw: [qx, qy, qz, qw]      # unit quaternion
```

For `space == cartesian`, only `regulation` is supported in schema v1: a
fixed Cartesian setpoint specified as position + unit quaternion in
`target.frame_id`. `step`, `sine`, and `random_waypoints` are joint-only
in v1; cartesian variants will land in a future schema rev with explicit
interpolation semantics.

#### `random_waypoints`

```yaml
command:
  num_waypoints: <int >= 1>
  dwell_s: <float > 0>           # also: num_waypoints * dwell_s <= duration_s
  seed: <int>
  bounds: urdf                    # OR
  # bounds:
  #   lower: [...]                # length 6, lower[i] < upper[i]
  #   upper: [...]
```

Only valid when `target.space == joint`.

### `pass_criteria`

Joint-space (`target.space == joint`):

| Key                       | Type      | Notes                          |
|---------------------------|-----------|--------------------------------|
| `max_rmse_rad`            | float > 0 | Per-joint RMSE bound.          |
| `max_settling_time_s`     | float > 0 | Optional.                      |
| `max_overshoot_pct`       | float >= 0| Optional.                      |
| `max_control_effort_nm`   | float > 0 | Optional. Sum or peak — TBD by metrics impl. |

Cartesian (`target.space == cartesian`):

| Key                       | Type      | Notes                          |
|---------------------------|-----------|--------------------------------|
| `max_rmse_m`              | float > 0 | Position RMSE bound, metres.   |
| `max_rmse_rad`            | float > 0 | Orientation RMSE bound, rad.   |
| `max_settling_time_s`     | float > 0 | Optional.                      |
| `max_control_effort_nm`   | float > 0 | Optional.                      |

Every key listed in `pass_criteria` must correspond to a metric listed in
`metrics` (otherwise the threshold is unverifiable). The validator enforces
this.

## Validation

`evaluation/scenarios/validate.py` validates a file against this schema:

```bash
python3 evaluation/scenarios/validate.py evaluation/scenarios/*.yaml
```

The validator returns a non-zero exit code and prints one error per problem.
It is exercised by `tests/unit/test_scenarios_schema.py`.
