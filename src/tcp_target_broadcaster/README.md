# TCP Target Broadcaster

This read-only `ros2_control` broadcaster publishes the target TCP pose, twist, and acceleration exposed by the hardware state interfaces. It reports values supplied by the hardware interface; it does not create targets or command robot motion.

It publishes `geometry_msgs/msg/PoseStamped`, `geometry_msgs/msg/TwistStamped`, and `geometry_msgs/msg/AccelStamped` on its private `~/pose`, `~/twist`, and `~/accel` topics. Each message gets a timestamp, but the current implementation does not set `header.frame_id`.

The target pose's three UR orientation state values are interpreted as a rotation vector (axis multiplied by angle in radians) and converted to a normalized quaternion for `PoseStamped`. A zero-length rotation vector is published as the identity quaternion. The message's `header.frame_id` is currently left empty, so consumers that need a reference frame must supply or configure it separately.

## Getting Started

Build and source this package in a ROS 2 workspace with compatible `ros2_control` and UR driver installations. Configure the controller manager to load `tcp_target_broadcaster/TcpTargetBroadcaster`. The hardware must export six state components for each of the following interfaces:

```text
target_tcp_pose/{position.x, position.y, position.z, orientation.x, orientation.y, orientation.z}
target_tcp_twist/{linear.x, linear.y, linear.z, angular.x, angular.y, angular.z}
target_tcp_acceleration/{linear.x, linear.y, linear.z, angular.x, angular.y, angular.z}
```

The updated hardware interface source files in `ur_driver_files/` export these interfaces for the UR driver.
Interface names must match the hardware exactly. Check them with `ros2 control list_hardware_interfaces`, then load and activate the broadcaster:

```bash
ros2 run controller_manager spawner tcp_target_broadcaster
```

For a controller named `tcp_target_broadcaster`, the output topics are typically `/tcp_target_broadcaster/pose`, `/tcp_target_broadcaster/twist`, and `/tcp_target_broadcaster/accel`. Include any controller namespace when subscribing.

## Example Configuration

Register the plugin with the controller manager and set the interface names to match your hardware. The names shown are those used in the supplied UR driver example.

```yaml
controller_manager:
  ros__parameters:
    tcp_target_broadcaster:
      type: tcp_target_broadcaster/TcpTargetBroadcaster

tcp_target_broadcaster:
  ros__parameters:
    tcp_target_pose_interface_name: target_tcp_pose
    tcp_target_twist_interface_name: target_tcp_twist
    tcp_target_acceleration_interface_name: target_tcp_acceleration
    tcp_target_twist_linear_decimal_places: 3
    tcp_target_twist_angular_decimal_places: 2
    tcp_target_acceleration_linear_decimal_places: 3
    tcp_target_acceleration_angular_decimal_places: 2
```

The decimal-place parameters round the published linear and angular twist and acceleration components. They do not affect the pose values.
