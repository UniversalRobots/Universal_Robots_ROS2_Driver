# Universal Robots ROS2 Driver

Beta version of the ROS2 Universal Robots driver. Should be transferred to the Universal Robots org when ready.


## Packages in the Repository:

  - `ur_bringup` - launch file and run-time configurations, e.g. controllers.
  - `ur_controllers` - implementations of controllers specific for UR robots.
  - `ur_dashboard_msgs` - package defining messages used by dashboard node.
  - `ur_description` - description files for the UR robots: meshes, URDF/XACRO files, etc.
  - `ur_moveit` - example MoveIt configuration for UR robots.
  - `ur_robot_driver` - driver / hardware interface for communication with UR robots.


## Getting Started

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

**NOTE**: If you are a ROS2 beginner and get lost consult the [references - section](#references).


## How to Use the ROS2 Driver for UR Robots

For starting the driver there are three main launch files in the `ur_bringup` package.

  - `ur_control.launch.py` - starts ros2_control node including hardware interface, joint state broadcaster and a controller. This launch file also starts `dashboard_client` if real robot is used.
  - `ur_moveit.launch.py` - start everything from `ur_control.launch.py` plus an example scenario with [MoveIt2](https://moveit.ros.org/).
  - `ur_dashboard_client.launch.py` - start the dashboard client for UR robots.

Also, there are predefined launch files for all supported types of UR robots.

The arguments for launch files can be listed using `ros2 launch ur_bringup <launch_file_name>.launch.py --show-args`.
The most relevant arguments are the following:

  - `ur_type` (*mandatory*) - a type of used UR robot (*ur3*, *ur3e*, *ur5*, *ur5e*, *ur10*, *ur10e*, or *ur16e*).
  - `robot_ip` (*mandatory*) - IP address by which the root can be reached.
  - `use_fake_hardware` (default: *false*) - use simple hardware emulator from ros2_control.
    Useful for testing launch files, descriptions, etc. See explanation below.
  - `fake_sensor_commands` (default: *false*) - enables setting sensor values for the hardware emulators.
    Useful for offline testing of controllers.
  - `robot_controller` (default: *joint_trajectory_controller*) - controller for robot joints to be started.
    Available controllers: *joint_trajectory_controller*, *scaled_joint_trajectory_controller*.
    Note: *joint_state_broadcaster*, *speed_scaling_state_broadcaster*, *force_torque_sensor_broadcaster*, and *io_and_status_controller* will always start.

    *HINT*: list all loaded controllers using `ros2 control list_controllers` command.

**NOTE**: The package can simulate hardware with the ros2_control `FakeSystem`. This emulator enables an environment for testing of "piping" of hardware and controllers, as well as testing robot's descriptions. For more details see [ros2_control documentation](https://ros-controls.github.io/control.ros.org/) for more details..

### Example Commands for Testing the Driver

- To start the robot driver and controllers, use:
   ```
   ros2 launch ur_bringup ur_control.launch.py ur_type:=ur5e robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true launch_rviz:=true
   ```
   For an offline test with the emulated hardware you can just copy-paste this line.
   To run on the hardware, write the IP address of your robot and omit the `use_fake_hardware` argument.

   **NOTE**: If controllers are not starting automatically, i.e., the robot state is not shown in rviz, you can start them manually:
   ```
   ros2 control load_controller --state joint_state_broadcaster
   ros2 control load_controller --state joint_trajectory_controller
   ```

   To check the controllers' state use `ros2 control list_controllers` command.

- Send some goal to the Joint Trajectory Controller by using a demo node from [ros2_control_demos](https://github.com/ros-controls/ros2_control_demos) package by starting  the following command in another terminal:
   ```
   ros2 launch ur_bringup test_joint_trajectory_controller.launch.py
   ```
   After a few seconds the robot should move.

- To test another controller, simply define it using `robot_controller` argument:
   ```
   ros2 launch ur_bringup ur_control.launch.py ur_type:=ur5e robot_ip:=yyy.yyy.yyy.yyy robot_controller:=scaled_joint_trajectory_controller use_fake_hardware:=true
   ```
   And send the command using demo node:
   ```
   ros2 launch ur_bringup test_scaled_joint_trajectory_controller.launch.py
   ```
   After a few seconds the robot should move (or jump when using emulation).

- To test the driver with example MoveIt-setup, first start the controllers then start MoveIt:
   ```
   ros2 launch ur_bringup ur_control.launch.py ur_type:=ur5e robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true launch_rviz:=false

   ros2 launch ur_bringup ur_moveit.launch.py ur_type:=ur5e robot_ip:="xxx.xxx" use_fake_hardware:=true launch_rviz:=true
   ```
   Now you should be able to use MoveIt Plugin in rviz2 to plan and execute trajectories with the robot.
   **NOTE**: This results in two instances of rviz2. You can safely close the one without *MotionPlanning* panel.

   If you have **issues** shows the correct configuration5 of the robot, try removing and re-adding *MotionPlanning* display.

5. If you just want to test description of the UR robots, e.g., after changes you can use the following command:
   ```
   ros2 launch ur_description view_ur.launch.py ur_type:=ur5e
   ```

## Expected Changes in the Near Future

- Using upstream `force_torque_sensor_broadcaster` (ros-controls/ros2_controllers#152)
- Trajectory control currently only supports position commands. In the future, velocity control will be added.


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
