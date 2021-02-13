# Universal Robots ROS2 Driver

Alpha version of the ROS2 Universal Robots driver. Should be transferred to the Universal Robots org when ready.

## General driver information
Driver currently only supports position joint interface which means only position-based controllers can be used with 
the ROS2 driver. [Universal Robots Client Library](https://github.com/UniversalRobots/Universal_Robots_Client_Library) includes also
velocity-based control whose support will be addressed in additional development of ROS2 driver.

## Requirements

Follow the [instructions](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver#setting-up-a-ur-robot-for-ur_robot_driver) in the paragraph 
[`Prepare the robot` ](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver#prepare-the-robot)

## Build Instructions

To build this package follow the [instructions](https://index.ros.org/doc/ros2/Tutorials/Workspace/Creating-A-Workspace/) for installation of ROS2.

After installation create a ROS Foxy workspace:
```
cd $HOME
mkdir -p ws_driver/src
cd ws_driver
source /opt/ros/$ROS_DISTRO/setup.bash

colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

```

Clone this repo into the `src/` directory, then:

```
# Clone source-based dependencies into src/ directory
vcs import --skip-existing --input src/Universal_Robots_ROS2_Driver/.repos.yaml src

# Install package-based dependencies
rosdep install -y --rosdistro $ROS_DISTRO --ignore-src --from-paths src

# Build sources
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```
Source the workspace:

```
cd ~/ws_driver
source install/setup.bash
```

Start the driver:

```
ros2 launch ur_ros2_control_demos ur5_e_system_position_only.launch.py
```

Start the `joint_state_controller`:

```
ros2 control load_start_controller joint_state_controller
```

Start the `forward_command_controller`:

```
ros2 control load_start_controller forward_command_controller_position
```

## Run a test node
Run a test node which will publish joint commands on /forward_command_controller_position/commands (std_msgs::msg::Float64MultiArray)
after checking the current joint states (to create minimal increment for safety). The node commands increment of 0.1 radians for each
joint. The commands are incremented in regards to /joint_states found when the node is run.

```
ros2 run ur_ros2_control_demos test_driver 
```
