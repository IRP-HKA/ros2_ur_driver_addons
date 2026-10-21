// Copyright 2026 IRP, Hochschule Karlsruhe (HKA)
// SPDX-License-Identifier: BSD-3-Clause
// See the LICENSE file in the repository root for license information.

#include "cartesian_twist_passthrough_controller/cartesian_twist_passthrough_controller.h"

#include <algorithm>
#include <exception>
#include <functional>

namespace cartesian_twist_passthrough_controller
{

CartesianTwistPassthroughController::CartesianTwistPassthroughController()
: controller_interface::ControllerInterface()
{
  current_twist_command_.setZero();
}

controller_interface::InterfaceConfiguration
CartesianTwistPassthroughController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Build interface names - these must match your hardware interface exactly
  config.names.push_back(tcp_interface_name_ + "/linear.x");
  config.names.push_back(tcp_interface_name_ + "/linear.y");
  config.names.push_back(tcp_interface_name_ + "/linear.z");
  config.names.push_back(tcp_interface_name_ + "/angular.x");
  config.names.push_back(tcp_interface_name_ + "/angular.y");
  config.names.push_back(tcp_interface_name_ + "/angular.z");

  return config;
}

controller_interface::InterfaceConfiguration
CartesianTwistPassthroughController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  // This controller forwards Twist commands and does not consume state interfaces.
  // Requesting unavailable state interfaces prevents activation.
  config.type = controller_interface::interface_configuration_type::NONE;

  return config;
}

controller_interface::CallbackReturn CartesianTwistPassthroughController::on_init()
{
  try {
    param_listener_ =
      std::make_shared<cartesian_twist_passthrough_controller::ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  tcp_interface_name_ = params_.tcp_interface_name;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianTwistPassthroughController::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  // Get parameters
  params_ = param_listener_->get_params();
  tcp_interface_name_ = params_.tcp_interface_name;

  // Create twist command subscriber
  twist_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
  "~/cmd_vel",
  rclcpp::QoS(1).reliable(),
  std::bind(&CartesianTwistPassthroughController::commandCallback, this, std::placeholders::_1));

  RCLCPP_INFO(get_node()->get_logger(),
              "Cartesian Twist Controller configured with interface: %s",
              tcp_interface_name_.c_str());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianTwistPassthroughController::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{

  // Verify we have the expected interfaces
  if (command_interfaces_.size() != 6) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Expected 6 command interfaces, got %zu", command_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(), "Cartesian Twist Controller activated");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianTwistPassthroughController::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{

  // Drop any stored command so a stale twist cannot resume on reactivation
  resetCommands();

  // Zero out hardware interfaces
  for (auto & command_interface : command_interfaces_) {
    command_interface.set_value(0.0);
  }

  RCLCPP_INFO(get_node()->get_logger(), "Cartesian Twist Controller deactivated");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianTwistPassthroughController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Pick up runtime changes of the (non read-only) timeout parameter
  if (param_listener_->is_old(params_)) {
    params_ = param_listener_->get_params();
  }

  // Check for timeout
  if (checkTimeout(time)) {
    resetCommands();
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 5000,
      "TCP velocity command timeout - stopping robot");
  }

  // Get latest command and update current twist
  auto twist_command = rt_command_ptr_.readFromRT();
  if (!command_timeout_ && twist_command && (*twist_command)) {
    updateTwistCommand(*twist_command);
  }

  // Ensure we have the correct number of command interfaces (should be 6)
  if (command_interfaces_.size() != 6) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Expected 6 command interfaces, got %zu", command_interfaces_.size());
    return controller_interface::return_type::ERROR;
  }

  // Set the TCP velocity commands directly to hardware interfaces
  // Order: linear.x, linear.y, linear.z, angular.x, angular.y, angular.z
  command_interfaces_[0].set_value(current_twist_command_[0]); // linear.x
  command_interfaces_[1].set_value(current_twist_command_[1]); // linear.y
  command_interfaces_[2].set_value(current_twist_command_[2]); // linear.z
  command_interfaces_[3].set_value(current_twist_command_[3]); // angular.x
  command_interfaces_[4].set_value(current_twist_command_[4]); // angular.y
  command_interfaces_[5].set_value(current_twist_command_[5]); // angular.z

  return controller_interface::return_type::OK;
}

void CartesianTwistPassthroughController::resetCommands()
{
  rt_command_ptr_.reset();
  last_command_time_ns_ = get_node()->get_clock()->now().nanoseconds();
  command_timeout_ = true;
  current_twist_command_.setZero();
}

bool CartesianTwistPassthroughController::checkTimeout(const rclcpp::Time & time)
{
  if (params_.timeout <= 0.0 || command_timeout_) {
    // No active timeout needs to be handled.
    return false;
  }
  const double age_s = static_cast<double>(time.nanoseconds() - last_command_time_ns_.load()) * 1e-9;
  if (age_s > params_.timeout) {
    command_timeout_ = true;
    current_twist_command_.setZero();
    return true;
  }
  return false;
}

void CartesianTwistPassthroughController::updateTwistCommand(
  const geometry_msgs::msg::Twist::SharedPtr & msg)
{
  // Convert Twist message to our internal format
  current_twist_command_[0] = msg->linear.x;
  current_twist_command_[1] = msg->linear.y;
  current_twist_command_[2] = msg->linear.z;
  current_twist_command_[3] = msg->angular.x;
  current_twist_command_[4] = msg->angular.y;
  current_twist_command_[5] = msg->angular.z;
}

void CartesianTwistPassthroughController::commandCallback(
  const geometry_msgs::msg::Twist::SharedPtr msg)
{
  rt_command_ptr_.writeFromNonRT(msg);
  last_command_time_ns_ = get_node()->get_clock()->now().nanoseconds();
  command_timeout_ = false;
}

} // namespace cartesian_twist_passthrough_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  cartesian_twist_passthrough_controller::CartesianTwistPassthroughController,
  controller_interface::ControllerInterface)