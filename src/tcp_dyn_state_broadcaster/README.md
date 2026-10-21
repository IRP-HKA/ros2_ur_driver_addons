# TCP Dynamic State Broadcaster

This read-only `ros2_control` broadcaster publishes the measured TCP twist and acceleration supplied by the hardware state interfaces. It is intended for monitoring or consumption by other ROS nodes; it does not command the robot.

It publishes `geometry_msgs/msg/TwistStamped` and `geometry_msgs/msg/AccelStamped` on its private `~/twist` and `~/accel` topics. The messages are timestamped, but the current implementation does not set `header.frame_id`. Linear and angular components can be rounded independently using the decimal-place parameters.

## Getting Started

Build and source this package in a ROS 2 workspace with compatible `ros2_control` and UR driver installations. Configure the controller manager to load `tcp_dyn_state_broadcaster/TcpDynStateBroadcaster`. The hardware must export these six-component state interfaces:

```text
tcp_twist/{linear.x, linear.y, linear.z, angular.x, angular.y, angular.z}
tcp_acceleration/{linear.x, linear.y, linear.z, angular.x, angular.y, angular.z}
```

The updated hardware interface source files in `ur_driver_files/` export these interfaces for the UR driver.
Interface names must match the hardware exactly. Check them with `ros2 control list_hardware_interfaces`, then load and activate the broadcaster:

```bash
ros2 run controller_manager spawner tcp_dyn_state_broadcaster
```

For a controller named `tcp_dyn_state_broadcaster`, the output topics are typically `/tcp_dyn_state_broadcaster/twist` and `/tcp_dyn_state_broadcaster/accel`. Include any controller namespace when subscribing.

## Example Configuration

Register the plugin with the controller manager. Set the interface names to the names exported by your hardware; the following acceleration name matches the supplied UR driver example.

```yaml
controller_manager:
  ros__parameters:
    tcp_dyn_state_broadcaster:
      type: tcp_dyn_state_broadcaster/TcpDynStateBroadcaster

tcp_dyn_state_broadcaster:
  ros__parameters:
    tcp_twist_interface_name: tcp_twist
    tcp_accel_interface_name: tcp_acceleration
    linear_twist_decimal_places: 3
    angular_twist_decimal_places: 2
    linear_accel_decimal_places: 3
    angular_accel_decimal_places: 2
```

The decimal-place parameters round the values before publication; they do not change the hardware state.
