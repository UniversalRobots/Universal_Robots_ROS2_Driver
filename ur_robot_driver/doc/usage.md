# Usage

## Launch files

For starting the driver there are two main launch files in the `ur_bringup` package.

  - `ur_control.launch.py` - starts ros2_control node including hardware interface, joint state broadcaster and a controller. This launch file also starts `dashboard_client` if real robot is used.
  - `ur_dashboard_client.launch.py` - start the dashboard client for UR robots.

Also, there are predefined launch files for all supported types of UR robots.

The arguments for launch files can be listed using `ros2 launch ur_bringup <launch_file_name>.launch.py --show-args`.
The most relevant arguments are the following:

  - `ur_type` (*mandatory* ) - a type of used UR robot (*ur3*, *ur3e*, *ur5*, *ur5e*, *ur10*, *ur10e*, or *ur16e*).
  - `robot_ip` (*mandatory* ) - IP address by which the root can be reached.
  - `use_fake_hardware` (default: *false* ) - use simple hardware emulator from ros2_control.
    Useful for testing launch files, descriptions, etc. See explanation below.
  - `initial_positions` (default: dictionary with all joint values set to 0) - Allows passing a dictionary to set the initial joint values for the fake hardware from [ros2_control](http://control.ros.org/).  It can also be set from a yaml file with the `load_yaml` commands as follows:
      ```
      <xacro:property name="initial_positions" value="${load_yaml(initial_positions_file)}" / >
      ```
      In this example, the **initial_positions_file** is a xacro argument that contains the absolute path to a yaml file. An example of the initial positions yaml file is as follows:
      ```
      elbow_joint: 1.158
      shoulder_lift_joint: -0.953
      shoulder_pan_joint: 1.906
      wrist_1_joint: -1.912
      wrist_2_joint: -1.765
      wrist_3_joint: 0.0
      ```

  - `fake_sensor_commands` (default: *false* ) - enables setting sensor values for the hardware emulators.
    Useful for offline testing of controllers.
  - `robot_controller` (default: *joint_trajectory_controller* ) - controller for robot joints to be started.
    Available controllers:
     * joint_trajectory_controller
     * scaled_joint_trajectory_controller

    Note: *joint_state_broadcaster*, *speed_scaling_state_broadcaster*, *force_torque_sensor_broadcaster*, and *io_and_status_controller* will always start.

    *HINT* : list all loaded controllers using `ros2 control list_controllers` command.

**NOTE**: The package can simulate hardware with the ros2_control `FakeSystem`. This emulator enables an environment for testing of "piping" of hardware and controllers, as well as testing robot's descriptions. For more details see [ros2_control documentation](https://ros-controls.github.io/control.ros.org/) for more details.

## Modes of operation
As mentioned in the last section the driver has two basic modes of operation: Using fake hardware or
using real hardware(Or the URSim simulator, which is equivalent from the driver's perspective).
Additionally, the robot can be simulated using
[Gazebo](https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation) or
[ignition](https://github.com/UniversalRobots/Universal_Robots_ROS2_Ignition_Simulation) but that's
outside of this driver's scope.

| mode                  | available controllers                                                                                                      |
| -------               | ----                                                                                                                       |
| fake_hardware         | <ul><li>joint_trajectory_controller</li><li>forward_velocity_controller</li><li>forward_position_controller</li></ul> |
| real hardware / URSim | <ul><li>joint_trajectory_controller</li><li>scaled_joint_trajectory_controller </li><li>forward_velocity_controller</li><li>forward_position_controller</li></ul>|

## Example Commands for Testing the Driver

Allowed UR - Type strings: `ur3`, `ur3e`, `ur5`, `ur5e`, `ur10`, `ur10e`, `ur16e`.

### 1. Start hardware, simulator or mockup

- To do test with hardware, use:
  ```
  ros2 launch ur_bringup ur_control.launch.py ur_type:=<UR_TYPE> robot_ip:=<IP_OF_THE_ROBOT> launch_rviz:=true
  ```
  For more details check the argument documentation with `ros2 launch ur_bringup ur_control.launch.py --show-arguments`

  After starting the launch file start the external_control URCap program from the pendant, as described above.

- To do an offline test with URSim check details about it in [this section](#usage-with-official-ur-simulator)

- To use mocked hardware(capability of ros2_control), use `use_fake_hardware` argument, like:
  ```
  ros2 launch ur_bringup ur_control.launch.py ur_type:=ur5e robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true launch_rviz:=true
  ```

  **NOTE**: Instead of using the global launch file for control stack, there are also prepeared launch files for each type of UR robots named. They accept the same arguments are the global one and are used by:
  ```
  ros2 launch ur_bringup <ur_type>.launch.py
  ```

### 2. Sending commands to controllers

Before running any commands, first check the controllers' state using `ros2 control list_controllers`.

- Send some goal to the Joint Trajectory Controller by using a demo node from [ros2_control_demos](https://github.com/ros-controls/ros2_control_demos) package by starting  the following command in another terminal:
  ```
  ros2 launch ur_bringup test_scaled_joint_trajectory_controller.launch.py
  ```
  After a few seconds the robot should move.

- To test another controller, simply define it using `initial_joint_controller` argument, for example when using fake hardware:
  ```
  ros2 launch ur_bringup ur_control.launch.py ur_type:=ur5e robot_ip:=yyy.yyy.yyy.yyy initial_joint_controller:=joint_trajectory_controller use_fake_hardware:=true launch_rviz:=true
  ```
  And send the command using demo node:
  ```
  ros2 launch ur_bringup test_joint_trajectory_controller.launch.py
  ```
  After a few seconds the robot should move(or jump when using emulation).

- To test the driver with the example MoveIt-setup, first start the controllers then start MoveIt. (This requires a `vcs import` of MoveIt packages):
  ```
  ros2 launch ur_bringup ur_control.launch.py ur_type:=ur5e robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true launch_rviz:=false

  (then start the external_control URCap program from the pendant, as described above)

  ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e use_fake_hardware:=true launch_rviz:=true
  ```
  Now you should be able to use the MoveIt Plugin in rviz2 to plan and execute trajectories with the robot.

### 3. Using only robot description

If you just want to test description of the UR robots, e.g., after changes you can use the following command:
```
ros2 launch ur_description view_ur.launch.py ur_type:=ur5e
```
