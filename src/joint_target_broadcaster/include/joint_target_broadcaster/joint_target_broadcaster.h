// Copyright 2026 IRP, Hochschule Karlsruhe (HKA)
// SPDX-License-Identifier: BSD-3-Clause
// See the LICENSE file in the repository root for license information.

#ifndef JOINT_TARGET_BROADCASTER_HPP_
#define JOINT_TARGET_BROADCASTER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <joint_target_broadcaster/joint_target_broadcaster_parameters.hpp>

namespace joint_target_broadcaster
{

class JointTargetBroadcaster : public controller_interface::ControllerInterface
{
public:
  JointTargetBroadcaster();
  virtual ~JointTargetBroadcaster() = default;

  // Lifecycle management
  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  // Main control loop
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // Interface configuration
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

protected:

private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_target_state_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_target_acceleration_publisher_;

  std::shared_ptr<joint_target_broadcaster::ParamListener> param_listener_;
  joint_target_broadcaster::Params params_;

  std::vector<double> current_target_joint_positions_;
  std::vector<double> current_target_joint_velocities_;
  std::vector<double> current_target_joint_accelerations_;

  std::string target_joint_position_interface_name_;
  std::string target_joint_velocity_interface_name_;
  std::string target_joint_acceleration_interface_name_;
};

} // namespace joint_target_broadcaster

#endif // JOINT_TARGET_BROADCASTER_HPP_