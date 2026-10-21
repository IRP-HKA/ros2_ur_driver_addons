// Copyright 2026 IRP, Hochschule Karlsruhe (HKA)
// SPDX-License-Identifier: BSD-3-Clause
// See the LICENSE file in the repository root for license information.

#include "tcp_dyn_state_broadcaster/tcp_dyn_state_broadcaster.h"

#include <cmath>
#include <exception>
#include <memory>

namespace tcp_dyn_state_broadcaster
{

TcpDynStateBroadcaster::TcpDynStateBroadcaster()
: controller_interface::ControllerInterface()
{
  current_tcp_twist_state_.setZero();
}

controller_interface::InterfaceConfiguration
TcpDynStateBroadcaster::command_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
TcpDynStateBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  config.names.push_back(tcp_twist_state_interface_name_ + "/linear.x");
  config.names.push_back(tcp_twist_state_interface_name_ + "/linear.y");
  config.names.push_back(tcp_twist_state_interface_name_ + "/linear.z");
  config.names.push_back(tcp_twist_state_interface_name_ + "/angular.x");
  config.names.push_back(tcp_twist_state_interface_name_ + "/angular.y");
  config.names.push_back(tcp_twist_state_interface_name_ + "/angular.z");

  config.names.push_back(tcp_accel_state_interface_name_ + "/linear.x");
  config.names.push_back(tcp_accel_state_interface_name_ + "/linear.y");
  config.names.push_back(tcp_accel_state_interface_name_ + "/linear.z");
  config.names.push_back(tcp_accel_state_interface_name_ + "/angular.x");
  config.names.push_back(tcp_accel_state_interface_name_ + "/angular.y");
  config.names.push_back(tcp_accel_state_interface_name_ + "/angular.z");

  return config;
}

controller_interface::CallbackReturn TcpDynStateBroadcaster::on_init()
{
  try {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<tcp_dyn_state_broadcaster::ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during init: %s", e.what());
    return CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TcpDynStateBroadcaster::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  // Get parameters
  tcp_twist_state_interface_name_ = params_.tcp_twist_interface_name;
  tcp_accel_state_interface_name_ = params_.tcp_accel_interface_name;

  // Create TCP twist and acceleration publishers
  twist_publisher_ = get_node()->create_publisher<geometry_msgs::msg::TwistStamped>(
  "~/twist",
  rclcpp::QoS(1).reliable());

  accel_publisher_ = get_node()->create_publisher<geometry_msgs::msg::AccelStamped>(
  "~/accel",
  rclcpp::QoS(1).reliable());

  RCLCPP_INFO(get_node()->get_logger(),
              "TCP Dyn State Broadcaster configured with interfaces: %s, %s",
              tcp_twist_state_interface_name_.c_str(),
              tcp_accel_state_interface_name_.c_str());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TcpDynStateBroadcaster::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{

  // Verify we have the expected interfaces
  if (state_interfaces_.size() != 12) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Expected 12 state interfaces, got %zu", state_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(), "TCP Dyn State Broadcaster activated");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TcpDynStateBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(get_node()->get_logger(), "TCP Dyn State Broadcaster deactivated");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type TcpDynStateBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Read the latest tcp twist state from the state interface for publishing
  for (size_t i = 0; i < 6; ++i) {
    current_tcp_twist_state_[i] = state_interfaces_[i].get_value();
    current_tcp_accel_state_[i] = state_interfaces_[i + 6].get_value();
  }

  // Round the values of the linear twists to X decimal places and the angular twists to Y decimal places
  // based on the given parameters
  for (size_t i = 0; i < 3; ++i) {
    current_tcp_twist_state_[i] = round(current_tcp_twist_state_[i] * pow(10, params_.linear_twist_decimal_places)) / pow(10, params_.linear_twist_decimal_places);
    current_tcp_accel_state_[i] = round(current_tcp_accel_state_[i] * pow(10, params_.linear_accel_decimal_places)) / pow(10, params_.linear_accel_decimal_places);
  }
  for (size_t i = 3; i < 6; ++i) {
    current_tcp_twist_state_[i] = round(current_tcp_twist_state_[i] * pow(10, params_.angular_twist_decimal_places)) / pow(10, params_.angular_twist_decimal_places);
    current_tcp_accel_state_[i] = round(current_tcp_accel_state_[i] * pow(10, params_.angular_accel_decimal_places)) / pow(10, params_.angular_accel_decimal_places);
  }

  // Publish the current TCP twist state for external use (e.g. in a separate controller or for monitoring)
  auto twist_msg = std::make_shared<geometry_msgs::msg::TwistStamped>();
  twist_msg->header.stamp = get_node()->now();
  twist_msg->twist.linear.x = current_tcp_twist_state_[0];
  twist_msg->twist.linear.y = current_tcp_twist_state_[1];
  twist_msg->twist.linear.z = current_tcp_twist_state_[2];
  twist_msg->twist.angular.x = current_tcp_twist_state_[3];
  twist_msg->twist.angular.y = current_tcp_twist_state_[4];
  twist_msg->twist.angular.z = current_tcp_twist_state_[5];
  twist_publisher_->publish(*twist_msg);

  // Publish the current TCP acceleration state for external use (e.g. in a separate controller or for monitoring)
  auto accel_msg = std::make_shared<geometry_msgs::msg::AccelStamped>();
  accel_msg->header.stamp = get_node()->now();
  accel_msg->accel.linear.x = current_tcp_accel_state_[0];
  accel_msg->accel.linear.y = current_tcp_accel_state_[1];
  accel_msg->accel.linear.z = current_tcp_accel_state_[2];
  accel_msg->accel.angular.x = current_tcp_accel_state_[3];
  accel_msg->accel.angular.y = current_tcp_accel_state_[4];
  accel_msg->accel.angular.z = current_tcp_accel_state_[5];
  accel_publisher_->publish(*accel_msg);

  return controller_interface::return_type::OK;
}

} // namespace tcp_dyn_state_broadcaster

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  tcp_dyn_state_broadcaster::TcpDynStateBroadcaster,
  controller_interface::ControllerInterface)