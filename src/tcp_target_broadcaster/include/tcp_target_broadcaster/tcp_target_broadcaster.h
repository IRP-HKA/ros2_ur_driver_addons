// Copyright 2026 IRP, Hochschule Karlsruhe (HKA)
// SPDX-License-Identifier: BSD-3-Clause
// See the LICENSE file in the repository root for license information.

#ifndef TCP_TARGET_BROADCASTER_HPP_
#define TCP_TARGET_BROADCASTER_HPP_

#include <memory>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tcp_target_broadcaster/tcp_target_broadcaster_parameters.hpp>

namespace ctrl
{
using Vector6D = Eigen::Matrix<double, 6, 1>;
}  // namespace ctrl

namespace tcp_target_broadcaster
{

class TcpTargetBroadcaster : public controller_interface::ControllerInterface
{
public:
  TcpTargetBroadcaster();
  virtual ~TcpTargetBroadcaster() = default;

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
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr accel_publisher_;

  std::shared_ptr<tcp_target_broadcaster::ParamListener> param_listener_;
  tcp_target_broadcaster::Params params_;

  ctrl::Vector6D current_tcp_target_pose_state_;
  ctrl::Vector6D current_tcp_target_twist_state_;
  ctrl::Vector6D current_tcp_target_acceleration_state_;

  std::string tcp_target_pose_interface_name_;
  std::string tcp_target_twist_interface_name_;
  std::string tcp_target_acceleration_interface_name_;
};

} // namespace tcp_target_broadcaster

#endif // TCP_TARGET_BROADCASTER_HPP_