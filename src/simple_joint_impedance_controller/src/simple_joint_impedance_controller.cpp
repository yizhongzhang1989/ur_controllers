// Skeleton implementation for simple_joint_impedance_controller.
//
// This iteration only wires the pluginlib-loadable lifecycle hooks so the
// controller can be declared to ros2_control. The control law itself is
// implemented as header-only helpers in math.hpp and exercised directly by
// the gtest unit tests. Full state/command interface plumbing, the
// ~/target_joint subscriber, and the ~/tau_d diagnostics publisher land
// in a follow-up iteration per ROADMAP M3.

#include "simple_joint_impedance_controller/simple_joint_impedance_controller.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace simple_joint_impedance_controller
{

controller_interface::CallbackReturn SimpleJointImpedanceController::on_init()
{
  try {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Exception while initialising parameter listener: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
SimpleJointImpedanceController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  cfg.names.reserve(params_.joints.size());
  for (const auto & joint : params_.joints) {
    cfg.names.push_back(joint + "/effort");
  }
  return cfg;
}

controller_interface::InterfaceConfiguration
SimpleJointImpedanceController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  cfg.names.reserve(params_.joints.size() * 2);
  for (const auto & joint : params_.joints) {
    cfg.names.push_back(joint + "/position");
    cfg.names.push_back(joint + "/velocity");
  }
  return cfg;
}

controller_interface::CallbackReturn SimpleJointImpedanceController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (param_listener_) {
    params_ = param_listener_->get_params();
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SimpleJointImpedanceController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SimpleJointImpedanceController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type SimpleJointImpedanceController::update(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  // Control law lands in the follow-up iteration. A no-op update keeps the
  // controller loadable for manual-integration smoke testing without ever
  // actually commanding non-zero effort.
  return controller_interface::return_type::OK;
}

}  // namespace simple_joint_impedance_controller

PLUGINLIB_EXPORT_CLASS(
  simple_joint_impedance_controller::SimpleJointImpedanceController,
  controller_interface::ControllerInterface)
