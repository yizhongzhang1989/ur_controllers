// Skeleton header for the in-tree simple joint impedance controller.
// Full control-law wiring lands in a follow-up iteration (ROADMAP M3
// "Control law baseline"); this header declares the lifecycle hooks so
// the plugin can load.

#ifndef SIMPLE_JOINT_IMPEDANCE_CONTROLLER__SIMPLE_JOINT_IMPEDANCE_CONTROLLER_HPP_
#define SIMPLE_JOINT_IMPEDANCE_CONTROLLER__SIMPLE_JOINT_IMPEDANCE_CONTROLLER_HPP_

#include <memory>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "simple_joint_impedance_controller/simple_joint_impedance_controller_parameters.hpp"

namespace simple_joint_impedance_controller
{

class SimpleJointImpedanceController : public controller_interface::ControllerInterface
{
public:
  SimpleJointImpedanceController() = default;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

protected:
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;
};

}  // namespace simple_joint_impedance_controller

#endif  // SIMPLE_JOINT_IMPEDANCE_CONTROLLER__SIMPLE_JOINT_IMPEDANCE_CONTROLLER_HPP_
