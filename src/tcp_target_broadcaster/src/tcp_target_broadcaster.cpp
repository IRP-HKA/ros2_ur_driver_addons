// Copyright 2026 IRP, Hochschule Karlsruhe (HKA)
// SPDX-License-Identifier: BSD-3-Clause
// See the LICENSE file in the repository root for license information.

#include "tcp_target_broadcaster/tcp_target_broadcaster.h"

#include <cmath>
#include <exception>
#include <memory>

namespace tcp_target_broadcaster
{

TcpTargetBroadcaster::TcpTargetBroadcaster()
: controller_interface::ControllerInterface()
{
  current_tcp_target_pose_state_.setZero();
  current_tcp_target_twist_state_.setZero();
  current_tcp_target_acceleration_state_.setZero();
}

controller_interface::InterfaceConfiguration
TcpTargetBroadcaster::command_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
TcpTargetBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  config.names.push_back(tcp_target_pose_interface_name_+ "/position.x");
  config.names.push_back(tcp_target_pose_interface_name_ + "/position.y");
  config.names.push_back(tcp_target_pose_interface_name_ + "/position.z");
  config.names.push_back(tcp_target_pose_interface_name_ + "/orientation.x");
  config.names.push_back(tcp_target_pose_interface_name_ + "/orientation.y");
  config.names.push_back(tcp_target_pose_interface_name_ + "/orientation.z");

  config.names.push_back(tcp_target_twist_interface_name_ + "/linear.x");
  config.names.push_back(tcp_target_twist_interface_name_ + "/linear.y");
  config.names.push_back(tcp_target_twist_interface_name_ + "/linear.z");
  config.names.push_back(tcp_target_twist_interface_name_ + "/angular.x");
  config.names.push_back(tcp_target_twist_interface_name_ + "/angular.y");
  config.names.push_back(tcp_target_twist_interface_name_ + "/angular.z");

  config.names.push_back(tcp_target_acceleration_interface_name_ + "/linear.x");
  config.names.push_back(tcp_target_acceleration_interface_name_ + "/linear.y");
  config.names.push_back(tcp_target_acceleration_interface_name_ + "/linear.z");
  config.names.push_back(tcp_target_acceleration_interface_name_ + "/angular.x");
  config.names.push_back(tcp_target_acceleration_interface_name_ + "/angular.y");
  config.names.push_back(tcp_target_acceleration_interface_name_ + "/angular.z");

  return config;
}

controller_interface::CallbackReturn TcpTargetBroadcaster::on_init()
{
  try {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<tcp_target_broadcaster::ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during init: %s", e.what());
    return CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TcpTargetBroadcaster::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  // Get parameters
  tcp_target_pose_interface_name_ = params_.tcp_target_pose_interface_name;
  tcp_target_twist_interface_name_ = params_.tcp_target_twist_interface_name;
  tcp_target_acceleration_interface_name_ = params_.tcp_target_acceleration_interface_name;

  // Create tcp target state publishers
  pose_publisher_ = get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
  "~/pose",
  rclcpp::QoS(1).reliable());

  twist_publisher_ = get_node()->create_publisher<geometry_msgs::msg::TwistStamped>(
  "~/twist",
  rclcpp::QoS(1).reliable());

  accel_publisher_ = get_node()->create_publisher<geometry_msgs::msg::AccelStamped>(
  "~/accel",
  rclcpp::QoS(1).reliable());

  RCLCPP_INFO(get_node()->get_logger(),
              "TCP Target Broadcaster configured with interfaces: %s, %s, %s",
              tcp_target_pose_interface_name_.c_str(),
              tcp_target_twist_interface_name_.c_str(),
              tcp_target_acceleration_interface_name_.c_str());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TcpTargetBroadcaster::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{

  // Verify we have the expected interfaces
  if (state_interfaces_.size() != 18) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Expected 18 state interfaces, got %zu", state_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(), "TCP Target Broadcaster activated");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TcpTargetBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(get_node()->get_logger(), "TCP Target Broadcaster deactivated");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type TcpTargetBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Read the latest tcp target state from the state interface for publishing
  for (size_t i = 0; i < 6; ++i) {
    current_tcp_target_pose_state_[i] = state_interfaces_[i].get_value();
    current_tcp_target_twist_state_[i] = state_interfaces_[i+6].get_value();
    current_tcp_target_acceleration_state_[i] = state_interfaces_[i+12].get_value();
  }

  // Round the values of the linear twists to X decimal places and the angular twists to Y decimal places
  // based on the given parameters
  for (size_t i = 0; i < 3; ++i) {
    current_tcp_target_twist_state_[i] = round(current_tcp_target_twist_state_[i] * pow(10, params_.tcp_target_twist_linear_decimal_places)) / pow(10, params_.tcp_target_twist_linear_decimal_places);
    current_tcp_target_acceleration_state_[i] = round(current_tcp_target_acceleration_state_[i] * pow(10, params_.tcp_target_acceleration_linear_decimal_places)) / pow(10, params_.tcp_target_acceleration_linear_decimal_places);
  }
  for (size_t i = 3; i < 6; ++i) {
    current_tcp_target_twist_state_[i] = round(current_tcp_target_twist_state_[i] * pow(10, params_.tcp_target_twist_angular_decimal_places)) / pow(10, params_.tcp_target_twist_angular_decimal_places);
    current_tcp_target_acceleration_state_[i] = round(current_tcp_target_acceleration_state_[i] * pow(10, params_.tcp_target_acceleration_angular_decimal_places)) / pow(10, params_.tcp_target_acceleration_angular_decimal_places);
  }

  // Convert the orientation vector to a quaternion for the pose message using the eigen library
  Eigen::Vector3d orientation_vector(current_tcp_target_pose_state_[3], current_tcp_target_pose_state_[4], current_tcp_target_pose_state_[5]);
  double orientation_angle = orientation_vector.norm();
  Eigen::Quaterniond orientation_quaternion;
  if (orientation_angle > 1e-16) {
    orientation_quaternion = Eigen::Quaterniond(Eigen::AngleAxisd(orientation_angle, orientation_vector.normalized()));
  } else {
    orientation_quaternion = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0); // default Quaternion is 1,0,0,0 which is valid
  }

  // Publish the current TCP pose target
  auto pose_msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
  pose_msg->header.stamp = get_node()->now();
  pose_msg->pose.position.x = current_tcp_target_pose_state_[0];
  pose_msg->pose.position.y = current_tcp_target_pose_state_[1];
  pose_msg->pose.position.z = current_tcp_target_pose_state_[2];
  pose_msg->pose.orientation.x = orientation_quaternion.x();
  pose_msg->pose.orientation.y = orientation_quaternion.y();
  pose_msg->pose.orientation.z = orientation_quaternion.z();
  pose_msg->pose.orientation.w = orientation_quaternion.w();
  pose_publisher_->publish(*pose_msg);

  // Publish the current TCP twist target
  auto twist_msg = std::make_shared<geometry_msgs::msg::TwistStamped>();
  twist_msg->header.stamp = get_node()->now();
  twist_msg->twist.linear.x = current_tcp_target_twist_state_[0];
  twist_msg->twist.linear.y = current_tcp_target_twist_state_[1];
  twist_msg->twist.linear.z = current_tcp_target_twist_state_[2];
  twist_msg->twist.angular.x = current_tcp_target_twist_state_[3];
  twist_msg->twist.angular.y = current_tcp_target_twist_state_[4];
  twist_msg->twist.angular.z = current_tcp_target_twist_state_[5];
  twist_publisher_->publish(*twist_msg);

  // Publish the current TCP acceleration target
  auto accel_msg = std::make_shared<geometry_msgs::msg::AccelStamped>();
  accel_msg->header.stamp = get_node()->now();
  accel_msg->accel.linear.x = current_tcp_target_acceleration_state_[0];
  accel_msg->accel.linear.y = current_tcp_target_acceleration_state_[1];
  accel_msg->accel.linear.z = current_tcp_target_acceleration_state_[2];
  accel_msg->accel.angular.x = current_tcp_target_acceleration_state_[3];
  accel_msg->accel.angular.y = current_tcp_target_acceleration_state_[4];
  accel_msg->accel.angular.z = current_tcp_target_acceleration_state_[5];
  accel_publisher_->publish(*accel_msg);

  return controller_interface::return_type::OK;
}

} // namespace tcp_target_broadcaster

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  tcp_target_broadcaster::TcpTargetBroadcaster,
  controller_interface::ControllerInterface)