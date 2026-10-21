# Joint Target Broadcaster

This read-only `ros2_control` broadcaster publishes the joint target positions, velocities, and accelerations exposed by the hardware state interfaces. It does not calculate or modify targets; the UR hardware interface must export the target state for every configured joint.

The broadcaster publishes:

- `sensor_msgs/msg/JointState` on its private `~/joint_targets` topic. The message contains configured joint names, target positions, and target velocities.
- `std_msgs/msg/Float64MultiArray` on `~/joint_targets_accelerations`. Its `data` values are target accelerations in configured joint order; the message does not include joint names or a timestamp.

## Getting Started

Build and source this package in a ROS 2 workspace with compatible `ros2_control` and UR driver installations. Configure the controller manager to load `joint_target_broadcaster/JointTargetBroadcaster`. Set the `joints` parameter and ensure the hardware exports the following state interfaces for every listed joint:

```text
<joint>/joint_target_position
<joint>/joint_target_velocity
<joint>/joint_target_acceleration
```

The updated hardware interface source files in `ur_driver_files/` export these interfaces for the UR driver.
Interface names must match the hardware exactly. Check them with `ros2 control list_hardware_interfaces`, then load and activate the broadcaster:

```bash
ros2 run controller_manager spawner joint_target_broadcaster
```

For a controller named `joint_target_broadcaster`, the topics are typically `/joint_target_broadcaster/joint_targets` and `/joint_target_broadcaster/joint_targets_accelerations`. If the controller is namespaced, include that namespace in the topic names.

## Example Configuration

Register the plugin with the controller manager and configure the six joint names for the robot. The interface names below match the supplied UR driver example.

```yaml
controller_manager:
  ros__parameters:
    joint_target_broadcaster:
      type: joint_target_broadcaster/JointTargetBroadcaster

joint_target_broadcaster:
  ros__parameters:
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_1_joint
      - wrist_2_joint
      - wrist_3_joint
    target_joint_position_interface_name: joint_target_position
    target_joint_velocity_interface_name: joint_target_velocity
    target_joint_acceleration_interface_name: joint_target_acceleration
```

The topic names are relative to the controller node. For example, inspect the outputs with `ros2 topic echo /joint_target_broadcaster/joint_targets`.
