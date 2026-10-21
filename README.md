# UR Driver Add-ons

Custom ROS 2 `ros2_control` components and configuration examples for extending a Universal Robots (UR) driver setup. These add-ons are intended for developers integrating extra control and state interfaces with a UR robot; they do not replace the UR driver.

## Components

The `src/` directory contains:

- **`cartesian_twist_passthrough_controller`** — subscribes to `~/cmd_vel` (`geometry_msgs/msg/Twist`) and forwards its three linear and three angular velocity components to the hardware's Cartesian twist command interfaces. It can zero the command when the configured command timeout expires; a non-positive timeout disables that check.
- **`joint_target_broadcaster`** — reads each configured joint's target position, velocity, and acceleration state interfaces. It publishes positions and velocities together as `sensor_msgs/msg/JointState` on `~/joint_targets`, and accelerations as `std_msgs/msg/Float64MultiArray` on `~/joint_targets_accelerations`.
- **`tcp_target_broadcaster`** — publishes the robot's target TCP pose, twist, and acceleration from their state interfaces as `geometry_msgs/msg/PoseStamped`, `TwistStamped`, and `AccelStamped` on `~/pose`, `~/twist`, and `~/accel`, respectively.
- **`tcp_dyn_state_broadcaster`** — publishes the measured TCP twist and acceleration from their state interfaces as `geometry_msgs/msg/TwistStamped` and `AccelStamped` on `~/twist` and `~/accel`. Configurable decimal-place parameters control rounding of linear and angular values.

The `~` topics above are private to each controller instance (for example, the twist command topic is under that controller's name). The broadcasters are read-only: they report values already exposed by the hardware and do not command robot motion. The twist passthrough controller does send motion commands; configure the required hardware interfaces and command timeout appropriately.

The `ur_driver_files/` directory provides integration examples: a launch file, controller configuration, RTDE output recipe, and hardware-interface source files. Adapt these to your UR driver version and robot configuration.

## Requirements

- ROS 2 and a compatible `ros2_control` installation.
- The Universal Robots ROS 2 driver and its dependencies. The patched files in `ur_driver_files/` are compatible with, and tested only against, these versions:
  - `Universal_Robots_ROS2_Driver` (`ur_robot_driver`) **2.9.0**
  - `Universal_Robots_ROS2_Description` (`ur_description`) **2.7.0**
- A UR robot or compatible simulation, configured and verified with the driver.
- `colcon` and `rosdep` for building the workspace.

Check the individual package READMEs, `package.xml` files, and parameter YAMLs for component-specific requirements. Compatibility depends on the ROS 2 distribution and UR driver version; other driver or description versions are untested, so the patched files may not apply or work with them.

## Build

Clone this repository into a ROS 2 workspace, source your ROS 2 and UR driver installations, then install dependencies and build:

```bash
mkdir -p ~/ur_addons_ws/src
cd ~/ur_addons_ws/src
git clone https://github.com/IRP-HKA/ros2_ur_driver_addons ur_driver_addons
cd ..
source /opt/ros/humble/setup.bash
# Source the UR driver workspace here too, if it is separate.
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Configure and use

1. Start by bringing up and validating the UR driver without these add-ons.
2. Incorporate the required changes shown in the files under `ur_driver_files/` into your existing UR driver setup, including the `hardware_interface` changes and RTDE output recipe, or replace the corresponding files in your driver with the provided files. Check compatibility with your UR driver version and adapt the files to your hardware before using them.
3. Ensure the controller manager loads the desired plugin and that the hardware exports all interfaces the plugin requires. See each package README and plugin XML for its plugin type, topics, parameters, and interfaces.
4. Launch the driver with your adapted configuration. Use `ros2 control list_controllers` to inspect controller state; load and activate the required components with your controller manager or ROS 2 controller spawners.

The supplied files are integration examples, not a turnkey configuration for every robot. Verify command units, frames, limits, and interface names against your setup before sending commands.

## Safety

Test in simulation or under appropriate supervision before operating on hardware. Cartesian commands may move the robot immediately. Keep clear of the workspace and preserve the robot's safety functions and emergency stop.

## License

BSD-3-Clause, see [LICENSE](LICENSE).

The replacement files in `ur_driver_files/` are modified copies of files from the [Universal Robots ROS 2 driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver) repository. They keep their original copyright and license headers, and those licenses apply to them instead of the license above. This applies to `hardware_interface.cpp`, `hardware_interface.hpp` and `ur_control.launch.py`. Any other file in that directory that originates from the driver repository is likewise covered by the driver's license.
