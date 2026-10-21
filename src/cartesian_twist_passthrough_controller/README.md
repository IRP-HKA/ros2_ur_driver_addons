# Cartesian Twist Passthrough Controller

This `ros2_control` controller accepts Cartesian velocity commands and writes their six components directly to the configured hardware command interfaces. It does not perform inverse kinematics or trajectory generation; the hardware interface and UR driver must support the named Cartesian twist command interfaces.

The controller subscribes to `geometry_msgs/msg/Twist` on its private `~/cmd_vel` topic. Linear `x`, `y`, and `z` and angular `x`, `y`, and `z` are forwarded in that order. Because `Twist` has no frame identifier, interpret the components according to the hardware and robot configuration.

## Getting Started

Build and source this package in a ROS 2 workspace with compatible `ros2_control` and UR driver installations. Configure the controller manager to load `cartesian_twist_passthrough_controller/CartesianTwistPassthroughController`, and ensure the hardware exports six matching command interfaces:

```text
<tcp_interface_name>/linear.x
<tcp_interface_name>/linear.y
<tcp_interface_name>/linear.z
<tcp_interface_name>/angular.x
<tcp_interface_name>/angular.y
<tcp_interface_name>/angular.z
```

The updated hardware interface source files in `ur_driver_files/` export these interfaces for the UR driver.

Check the actual names with `ros2 control list_hardware_interfaces`. Once the controller is listed in the controller-manager configuration, load and activate it:

```bash
ros2 run controller_manager spawner cartesian_twist_passthrough_controller
```

Publish commands on the controller's topic:

```bash
ros2 topic pub /cartesian_twist_passthrough_controller/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

The `timeout` parameter is in seconds. A positive value zeros the commanded twist when no new command arrives before the timeout; `0` or a negative value disables timeout handling. Deactivating the controller also zeros its command interfaces. Configure a timeout suitable for the application, and retain the robot's normal limits and safety functions.

## Example Configuration

Add the plugin and parameters to the controller-manager YAML. The interface name must match the hardware export; `cartesian_twist_passthrough` is the name used in the supplied UR driver example.

```yaml
controller_manager:
  ros__parameters:
    cartesian_twist_passthrough_controller:
      type: cartesian_twist_passthrough_controller/CartesianTwistPassthroughController

cartesian_twist_passthrough_controller:
  ros__parameters:
    tcp_interface_name: cartesian_twist_passthrough
    timeout: 0.2
```