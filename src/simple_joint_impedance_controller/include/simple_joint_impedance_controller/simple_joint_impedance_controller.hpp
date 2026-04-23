// Header for the in-tree simple joint impedance controller. Declares the
// ControllerInterface lifecycle hooks plus the runtime state needed to run
// `tau = K (q_d - q) - D qdot` against `<joint>/{position, velocity}` state
// interfaces and `<joint>/effort` command interfaces. See
// docs/DECISIONS.md ADR-0008 for the feature-split rationale.

#ifndef SIMPLE_JOINT_IMPEDANCE_CONTROLLER__SIMPLE_JOINT_IMPEDANCE_CONTROLLER_HPP_
#define SIMPLE_JOINT_IMPEDANCE_CONTROLLER__SIMPLE_JOINT_IMPEDANCE_CONTROLLER_HPP_

#include <memory>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

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

  // Gains cached at on_activate (after critical-damping auto-fill on d_).
  std::vector<double> k_;
  std::vector<double> d_;
  std::vector<double> tau_max_;
  std::vector<double> max_delta_tau_;

  // URDF-derived per-joint position limits (keep-list #7).
  std::vector<double> q_lower_;
  std::vector<double> q_upper_;

  // Reference and last-torque state.
  std::vector<double> q_d_;
  std::vector<double> qdot_d_;    // empty vector ⇒ zero feed-forward velocity
  std::vector<double> tau_prev_;  // previous cycle output for rate saturation

  // Seed `q_d_` from measured `q` on the first update after activation so the
  // controller holds position until a user target arrives. Bool lives in main
  // thread + update() only; no synchronisation needed.
  bool seed_on_first_update_ = true;

  // Target subscription (non-RT callback writes into the RT buffer).
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr target_sub_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<sensor_msgs::msg::JointState>>
    target_rt_buffer_;

  // Diagnostics publisher for the commanded torque (RT-safe).
  using JointStatePublisher = rclcpp::Publisher<sensor_msgs::msg::JointState>;
  using RealtimeJointStatePublisher =
    realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>;
  std::shared_ptr<JointStatePublisher> tau_d_pub_;
  std::shared_ptr<RealtimeJointStatePublisher> tau_d_rt_pub_;
};

}  // namespace simple_joint_impedance_controller

#endif  // SIMPLE_JOINT_IMPEDANCE_CONTROLLER__SIMPLE_JOINT_IMPEDANCE_CONTROLLER_HPP_
