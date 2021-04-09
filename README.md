# Universal Robots ROS2 Driver

Alpha version of the ROS2 Universal Robots driver. Should be transferred to the Universal Robots org when ready.


## Packages in the Repository:

  - `ur_bringup` - launch file and run-time configurations, e.g., controllers.
  - `ur_controllers` - implementations of controllers specific for UR robots.
  - `ur_dashboard_msgs` - package defining messages used by dashboard node.
  - `ur_description` - description files for the UR robots: meshes, URDF/XACRO files, etc.
  - `ur_moveit` - example MovieIt configuration for UR robots.
  - `ur_robot_driver` - driver / hardware interface for communication with UR robots.


## Current State of the Driver

Driver currently only supports position joint interface which means only position-based controllers can be used with
the ROS2 driver. [Universal Robots Client Library](https://github.com/UniversalRobots/Universal_Robots_Client_Library) includes also
velocity-based control whose support will be addressed in additional development of ROS2 driver.

### Expected Changes in the Near Future

- Using upstream `force_torque_sensor_broadcaster` (ros-controls/ros2_controllers#152)


## Requirements

Follow the [instructions](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver#setting-up-a-ur-robot-for-ur_robot_driver) in the paragraph
[`Prepare the robot` ](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver#prepare-the-robot)


## Getting-Started Instructions

1. [Install ROS2 Foxy](https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Install-Debians/).

2. Create a new ROS2 workspace:
   ```
   export COLCON_WS=~/workspace/ros_ws_foxy_ur_driver
   mkdir -p $COLCON_WS/src
   ```

3. Pull relevant packages, install dependencies, compile, and source the workspace by using:
   ```
   cd $COLCON_WS
   git clone git@github.com:PickNikRobotics/Universal_Robots_ROS2_Driver.git src/Universal_Robots_ROS2_Driver --branch develop
   vcs import src --skip-existing --input src/Universal_Robots_ROS2_Driver/Universal_Robots_ROS2_Driver.repos
   rosdep install --ignore-src --from-paths src -y -r
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
   source install/setup.bash
   ```

**NOTE**: If you are ROS2 beginner and got lost consult the [references - section](#references).



## Stuff to add

1. Explation of packages
2. How to start:
   - Simple simulation with FakeSystem
   - Hardware




## How to Use ROS2 Driver for UR Robots

For starting the driver there are three main launch files in the `ur_bringup` package.

  - `ur_control.launch.py` - starts ros2_control node including hardware interface, joint state broadcaster and a controller.
  - `ur_moveit.launch.py` - start everything from `ur_control.launch.py` plus an example scenario with [MoveIt2](https://moveit.ros.org/).
  - `ur_dashboard_client.launch.py` - start dashboard client for UR robots.

Also, there are predefined launch files for all supported types of UR robots.

The arguments for launch files can be listed using `ros2 launch ur_bringup <launch_file_name>.launch.py --show-args`.
The most relevant arguments are the following:

  - `ur_type` (*mandatory*) - a type of used UR robot (*ur3*, *ur3e*, *ur5*, *ur5e*, *ur10*, *ur10e*, or *ur16e*).
  - `robot_ip` (*mandatory*) - IP address b which the root can be reached.
  - `use_fake_hardware` (default: *false*) - use simple hardware emulator from ros2_control.
    Useful for testing launch files, descriptions, etc. See explanation below.
  - `fake_sensor_commands` (default: *false*) - enables setting sensor values for the hardware emulators.
    Use full for offline testing of controllers.
  - `robot_controller` (default: *joint_trajectory_controller*) - controller for robot joints to be started.
    Available controllers: *joint_trajectory_controller*, *scaled_joint_trajectory_controller*, *forward_position_controller*, and *forward_velocity_controller*.
    Note: *JointStateBroadcaster*, *SpeedScalingStateBroadcaster*, *ForceTorqueStateBroadcaster*, and *IO/StatusController* will always start.

    *HINT*: list all loaded controllers using `ros2 control list_controllers` command.

**NOTE**: The package implements support to use it without hardware with a simple emulator called `FakeSystem`. This emulator enables an environment for testing of "piping" of hardware and controllers, as well as testing robot's descriptions. For more details see [ros2_control documentation](https://ros-controls.github.io/control.ros.org/) for more details..

### Example Commands for Testing the Driver

1. To start the robot driver and controllers, use:
   ```
   ros2 launch ur_bringup ur_control.launch.py ur_type:=ur5e robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true
   ```
   If you want to do "offline" test with the emulated hardware you can just copy-paste this line.
   To run on the hardware, write correct IP-address of your robot and omit `use_fake_hardware` argument.

2. Send some goal to the Joint Trajectory Controller by using a demo node form [ros2_control_demos](https://github.com/ros-controls/ros2_control_demos) package by starting  the following command in another terminal:
   ```
   ros2 launch ur_bringup test_joint_trajectory_controller.launch.py
   ```
   After few seconds the robot should move.

3. To test an another controller, simply define it using `robot_controller` argument:
   ```
   ros2 launch ur_bringup ur_control.launch.py ur_type:=ur5e robot_ip:=yyy.yyy.yyy.yyy robot_controller:=forward_position_controller use_fake_hardware:=true
   ```
   And send command using demo node:
   ```
   ros2 launch ur_bringup test_forward_position_controller.launch.py
   ```
   After few seconds the robot should move (or jump when using emulation).


## Contributor Guidelines
Code is auto-formatted with clang-format 10 whenever a git commit is made. Please ensure these dependencies are installed so pre-commit formatting works:
  ```
  pip3 install pre-commit
  sudo apt install clang-format-10
  ```

Prepare the pre-commit formatting to run like this:
  ```
  pre-commit install`
  ```

## References

Here are some useful references for developing with ROS2:

 - [Official ROS2 Tutorials](https://index.ros.org/doc/ros2/Tutorials/)
   * [Luanchfile](https://index.ros.org/doc/ros2/Tutorials/Launch-Files/Creating-Launch-Files/)
   * [Package](https://index.ros.org/doc/ros2/Tutorials/Creating-Your-First-ROS2-Package/)
   * [Parameters](https://index.ros.org/doc/ros2/Tutorials/Parameters/Understanding-ROS2-Parameters/)
   * [Workspace](https://index.ros.org/doc/ros2/Tutorials/Workspace/Creating-A-Workspace/)
 - [Example ROS packages](https://github.com/ros2/examples)
 - [Colcon Documentation](https://colcon.readthedocs.io/en/released/#)
 - [ROS2 Design Documentation](https://design.ros2.org/)
 - [ROS2 Launch Architecture](https://github.com/ros2/launch/blob/master/launch/doc/source/architecture.rst)
