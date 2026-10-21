// Copyright 2026 IRP, Hochschule Karlsruhe (HKA)
// SPDX-License-Identifier: BSD-3-Clause
// See the LICENSE file in the repository root for license information.

#include "joint_target_broadcaster/joint_target_broadcaster.h"

#include <exception>
#include <memory>

namespace joint_target_broadcaster
{

JointTargetBroadcaster::JointTargetBroadcaster()
: controller_interface::ControllerInterface()
{
}

controller_interface::InterfaceConfiguration
JointTargetBroadcaster::command_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
JointTargetBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint : params_.joints) {
    config.names.push_back(joint + "/" + params_.target_joint_position_interface_name);
    config.names.push_back(joint + "/" + params_.target_joint_velocity_interface_name);
    config.names.push_back(joint + "/" + params_.target_joint_acceleration_interface_name);
  }
  return config;
}

controller_interface::CallbackReturn JointTargetBroadcaster::on_init()
{
  try {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<joint_target_broadcaster::ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during init: %s", e.what());
    return CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointTargetBroadcaster::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  // Get parameters
  const size_t num_joints = params_.joints.size();
  current_target_joint_positions_.assign(num_joints, 0.0);
  current_target_joint_velocities_.assign(num_joints, 0.0);
  current_target_joint_accelerations_.assign(num_joints, 0.0);
  target_joint_position_interface_name_ = params_.target_joint_position_interface_name;
  target_joint_velocity_interface_name_ = params_.target_joint_velocity_interface_name;
  target_joint_acceleration_interface_name_ = params_.target_joint_acceleration_interface_name;

  // Create joint target state publisher
  joint_target_state_publisher_ = get_node()->create_publisher<sensor_msgs::msg::JointState>(
  "~/joint_targets", rclcpp::QoS(1).reliable());
  joint_target_acceleration_publisher_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
  "~/joint_targets_accelerations", rclcpp::QoS(1).reliable());
  RCLCPP_INFO(get_node()->get_logger(),
              "Joint Target Broadcaster configured with interfaces: %s, %s, %s",
              target_joint_position_interface_name_.c_str(),
              target_joint_velocity_interface_name_.c_str(),
              target_joint_acceleration_interface_name_.c_str());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointTargetBroadcaster::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{

  // Verify we have the expected interfaces
  if (state_interfaces_.size() != params_.joints.size() * 3) {
    RCLCPP_ERROR(get_node()->get_logger(), "Expected %zu state interfaces, but got %zu",
                 params_.joints.size() * 3, state_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(), "Joint Target Broadcaster activated");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointTargetBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(get_node()->get_logger(), "Joint Target Broadcaster deactivated");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type JointTargetBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Read the latest joint target state from the state interfaces
  for (size_t i = 0; i < params_.joints.size(); ++i) {
    const size_t base = 3 * i;
    current_target_joint_positions_[i] = state_interfaces_[base + 0].get_value();
    current_target_joint_velocities_[i] = state_interfaces_[base + 1].get_value();
    current_target_joint_accelerations_[i] = state_interfaces_[base + 2].get_value();
  }

  // Publish joint target positions and velocities
  auto joint_state_msg = std::make_shared<sensor_msgs::msg::JointState>();
  joint_state_msg->header.stamp = get_node()->now();
  joint_state_msg->name = params_.joints;
  joint_state_msg->position.resize(params_.joints.size());
  joint_state_msg->velocity.resize(params_.joints.size());
  for (size_t i = 0; i < params_.joints.size(); ++i) {
    joint_state_msg->position[i] = current_target_joint_positions_[i];
    joint_state_msg->velocity[i] = current_target_joint_velocities_[i];
  }
  joint_target_state_publisher_->publish(*joint_state_msg);

  // Publish joint target accelerations
  auto accel_msg = std::make_shared<std_msgs::msg::Float64MultiArray>();
  accel_msg->data.resize(params_.joints.size());
  for (size_t i = 0; i < params_.joints.size(); ++i) {
    accel_msg->data[i] = current_target_joint_accelerations_[i];
  }
  joint_target_acceleration_publisher_->publish(*accel_msg);

  return controller_interface::return_type::OK;
}

} // namespace joint_target_broadcaster

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  joint_target_broadcaster::JointTargetBroadcaster,
  controller_interface::ControllerInterface)