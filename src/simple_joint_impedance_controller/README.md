# simple_joint_impedance_controller

Lightweight, in-tree `controller_interface::ControllerInterface` plugin
providing joint-space PD impedance control:

    tau_cmd = K (q_d - q) - D qdot

Scope (what stays in, what was deliberately left out) is frozen in
**ADR-0008** in `docs/DECISIONS.md`. This package currently ships the
skeleton: pluginlib-loadable lifecycle stubs, parameter schema via
`generate_parameter_library`, and header-only math helpers exercised by a
gtest suite. The control law wiring, `~/target_joint` subscription, and
`~/tau_d` diagnostics publisher land in follow-up iterations under
ROADMAP M3.

## Layout

- `include/simple_joint_impedance_controller/math.hpp` — header-only math
  helpers (PD torque, auto-damping, clamping, rate + absolute saturation).
- `include/simple_joint_impedance_controller/simple_joint_impedance_controller.hpp`
  + `src/simple_joint_impedance_controller.cpp` — controller class.
- `src/simple_joint_impedance_controller.yaml` —
  `generate_parameter_library` schema (keep-list from ADR-0008).
- `simple_joint_impedance_controller.xml` — pluginlib manifest.
- `tests/test_math.cpp` — gtest unit tests against the math helpers;
  build and run via `colcon test`.
