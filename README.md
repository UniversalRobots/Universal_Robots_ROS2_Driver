# Universal Robots ROS2 Driver

Universal Robots has become a dominant supplier of lightweight, robotic manipulators for industry, as well as for scientific research and education.
<center><img src="ur_robot_driver/doc/installation/initial_setup_images/e-Series.jpg" alt="Universal Robot e-Series family" style="width: 80%;"/></center>

This is one of the very first ROS2 manipulator drivers. Some of the new features are enabled by ROS2 and include decreased latency, improved security, and more flexibility regarding middleware configuration. The package contains launch files to quickly get started using the driver as a standalone version or in combination with MoveIt2

This driver is developed on top of [Universal_Robots_Client_Library](https://github.com/UniversalRobots/Universal_Robots_Client_Library) and support some key cobot functionalities like; pause at emergency stop, safeguard stop, automatic speed scaling to avoid violate the safety setting and manually speed scaling from the teach pendant. In addition the externalControl URCap makes it possible to include ROS2 behaviors in the robot program.

The driver is compatible across the entire line of UR robots -- from 3 kg payload to 16 kg payload and includes both the CB3 and the E-series.


Check also [presentations and videos](ur_robot_driver/doc/resources/README.md) about this driver.


## Build Status

<table width="100%">
  <tr>
    <th>ROS2 Distro</th>
    <th>Foxy</th>
    <th>Galactic</th>
    <th>Humble</th>
    <th>Rolling</th>
  </tr>
  <tr>
    <th>Branch</th>
    <td><a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/foxy">foxy</a></td>
    <td><a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/galactic">galactic</a></td>
    <td><a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/main">main</a></td>
    <td><a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/main">main</a></td>
  </tr>
  <tr>
    <th>Build Status</th>
    <td>
      <a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/actions/workflows/foxy-binary-build.yml?query=event%3Aschedule++">
         <img src="https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/actions/workflows/foxy-binary-build.yml/badge.svg?event=schedule"
              alt="Foxy Binary Build"/>
      </a> <br />
      <a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/actions/workflows/foxy-semi-binary-build.yml?query=event%3Aschedule++">
         <img src="https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/actions/workflows/foxy-semi-binary-build.yml/badge.svg?event=schedule"
              alt="Foxy Semi-Binary Build"/>
      </a> <br />
      <a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/actions/workflows/foxy-source-build.yml?query=branch%3Afoxy+">
         <img src="https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/actions/workflows/foxy-source-build.yml/badge.svg?branch=foxy"
              alt="Foxy Source Build"/>
      </a>
    </td>
    <td>
      <a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/actions/workflows/galactic-binary-build.yml?query=event%3Aschedule++">
         <img src="https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/actions/workflows/galactic-binary-build.yml/badge.svg?event=schedule"
              alt="Galactic Binary Build"/>
      </a> <br />
      <a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/actions/workflows/galactic-semi-binary-build.yml?query=event%3Aschedule++">
         <img src="https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/actions/workflows/galactic-semi-binary-build.yml/badge.svg?event=schedule"
              alt="Galactic Semi-Binary Build"/>
      </a> <br />
      <a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/actions/workflows/galactic-source-build.yml?query=branch%3Agalactic+">
         <img src="https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/actions/workflows/galactic-source-build.yml/badge.svg?branch=galactic"
              alt="Galactic Source Build"/>
      </a>
    </td>
    <td>
      <a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/actions/workflows/humble-binary-build.yml?query=branch%3Amain+">
         <img src="https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/actions/workflows/humble-binary-build.yml/badge.svg?branch=main"
              alt="Humble Binary Build"/>
      </a> <br />
      <a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/actions/workflows/humble-semi-binary-build.yml?query=branch%3Amain+">
         <img src="https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/actions/workflows/humble-semi-binary-build.yml/badge.svg?branch=main"
              alt="Humble Semi-Binary Build"/>
      </a> <br />
      <a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/actions/workflows/humble-source-build.yml?branch=main">
         <img src="https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/actions/workflows/humble-source-build.yml/badge.svg?branch=main"
              alt="Humble Source Build"/>
      </a>
    </td>
    <td>
      <a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/actions/workflows/rolling-binary-build.yml?query=branch%3Amain+">
         <img src="https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/actions/workflows/rolling-binary-build.yml/badge.svg?branch=main"
              alt="Rolling Binary Build"/>
      </a> <br />
      <a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/actions/workflows/rolling-semi-binary-build.yml?query=branch%3Amain+">
         <img src="https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/actions/workflows/rolling-semi-binary-build.yml/badge.svg?branch=main"
              alt="Rolling Semi-Binary Build"/>
      </a> <br />
      <a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/actions/workflows/rolling-source-build.yml?query=branch%3Amain+">
         <img src="https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/actions/workflows/rolling-source-build.yml/badge.svg?branch=main"
              alt="Rolling Source Build"/>
      </a>
    </td>
  </tr>
  <tr>
    <th>Execution Test</th>
    <td>&nbsp;</td>
    <td>
      <a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/actions/workflows/galactic-execution-test.yml?query=branch%3Amain+">
         <img src="https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/actions/workflows/galactic-execution-test.yml/badge.svg?branch=main"
              alt="Execution Testing"/>
      </a>
   </td>
   <td>
      <a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/actions/workflows/humble-execution-test.yml?query=branch%3Amain+">
         <img src="https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/actions/workflows/humble-execution-test.yml/badge.svg?branch=main"
              alt="Execution Testing"/>
      </a>
   </td>
   <td>
      <a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/actions/workflows/rolling-execution-test.yml?query=branch%3Amain+">
         <img src="https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/actions/workflows/rolling-execution-test.yml/badge.svg?branch=main"
              alt="Execution Testing"/>
      </a>
   </td>

  </tr>
</table>


**NOTE**: There are three build stages checking current and future compatibility of the driver.

1. Binary builds - against released packages (main and testing) in ROS distributions. Shows that direct local build is possible.

   Uses repos file: `src/Universal_Robots_ROS2_Driver/Universal_Robots_ROS2_Driver-not-released.<ros-distro>.repos`

1. Semi-binary builds - against released core ROS packages (main and testing), but the immediate dependencies are pulled from source.
   Shows that local build with dependencies is possible and if fails there we can expect that after the next package sync we will not be able to build.

   Uses repos file: `src/Universal_Robots_ROS2_Driver/Universal_Robots_ROS2_Driver.repos`

1. Source build - also core ROS packages are build from source. It shows potential issues in the mid future.


## Packages in the Repository:

  - `ur` - Meta-package that provides a single point of installation for the released packages.
  - `ur_bringup` - launch file and run-time configurations, e.g. controllers (DEPRECATED).
  - `ur_calibration` - tool for extracting calibration information from a real robot.
  - `ur_controllers` - implementations of controllers specific for UR robots.
  - `ur_dashboard_msgs` - package defining messages used by dashboard node.
  - `ur_moveit_config` - example MoveIt configuration for UR robots.
  - `ur_robot_driver` - driver / hardware interface for communication with UR robots.

Deprecation: The `ur_bringup` package is deprecated and will be removed from Iron Irwini on.

## Getting Started

1. [Install ROS2 Rolling](https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debians.html).
   For using this driver with ROS2 `foxy`. Checkout [foxy
   branch](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/foxy), for using it
   with ROS2 ``galactic``, use the [galactic branch](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/galactic).

2. Make sure that `colcon`, its extensions and `vcs` are installed:
   ```
   sudo apt install python3-colcon-common-extensions python3-vcstool
   ```

3. Create a new ROS2 workspace:
   ```
   export COLCON_WS=~/workspace/ros_ur_driver
   mkdir -p $COLCON_WS/src
   ```

4. Pull relevant packages, install dependencies, compile, and source the workspace by using:
   ```
   cd $COLCON_WS
   git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git src/Universal_Robots_ROS2_Driver
   vcs import src --skip-existing --input src/Universal_Robots_ROS2_Driver/Universal_Robots_ROS2_Driver-not-released.${ROS_DISTRO}.repos
   rosdep update
   rosdep install --ignore-src --from-paths src -y -r
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
   source install/setup.bash
   ```

## Using MoveIt

[MoveIt!](https://moveit.ros.org) support is built-in into this driver already.
Watch MoveIt in action with the Universal Robots ROS2 driver:

[![Video: MoveIt2 Demo](https://img.youtube.com/vi/d_cVXoZZ52w/0.jpg)](https://www.youtube.com/watch?v=d_cVXoZZ52w)

  *The video shows free-space trajectory planning around a modeled collision scene object using the MoveIt2 MotionPlanning widget for Rviz2.*


### Real robot / URSim
To test the driver with the example MoveIt-setup, first start the driver as described
[below](#connect-to-external-control-via-urcap).
```
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true
```
Now you should be able to use the MoveIt Plugin in rviz2 to plan and execute trajectories with the
robot as explained [here](https://moveit.picknik.ai/galactic/doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial.html).

## Network Setup

There are many possible ways to connect a UR robot. This section describes a good example using static IP addresses and a direct connection from the PC to the Robot to minimize latency introduced by network hardware. Though a good network switch usually works fine, as well.

1. Connect the UR control box directly to the remote PC with an ethernet cable.

2. Open the network settings from the UR teach pendant (Setup Robot -> Network) and enter these settings:

```
IP address: 192.168.1.102
Subnet mask: 255.255.255.0
Default gateway: 192.168.1.1
Preferred DNS server: 192.168.1.1
Alternative DNS server: 0.0.0.0
```

3. On the remote PC, turn off all network devices except the "wired connection", e.g. turn off wifi.

4. Open Network Settings and create a new Wired connection with these settings. You may want to name this new connection `UR` or something similar:

```
IPv4
Manual
Address: 192.168.1.101
Netmask: 255.255.255.0
Gateway: 192.168.1.1
```

5. Verify the connection from the PC with e.g. ping.

```
ping 192.168.1.102
```

## Connect to External Control via URCap

This section describes installation and launching of the URCap program from the pendant. It allows ROS to control the robot externally. Generally, you will launch the driver via ROS then start URCap from the pendant.

1. To enable external control of the UR robot from a remote PC you need to install the [**externalcontrol-1.0.5.urcap**](/ur_robot_driver/resources) which can be downloaded from [Universal_Robots_ExternalControl_URCap](https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases).

**Note:** For installing this URCap a minimal PolyScope version of 3.7 or 5.1 (in case of e-Series) is necessary.

2. For installing the necessary URCap and creating a program, please see the individual tutorial on how to [setup a CB3 robot](/ur_robot_driver/doc/installation/install_urcap_cb3.rst) or how to [setup an e-Series robot](/ur_robot_driver/doc/installation/install_urcap_e_series.rst)

3. On the remote PC, launch the suitable launch file which starts the robot driver and controllers (see details in [Usage](#usage) section).

4. From the Program Robot tab of the teach pendant, load `external_control.urp`. Click on the "Control by..." section of the program to check the Host IP of the external PC. If it needs to be modified, make the modification under the Installation tab (as prompted on screen). You do not need to modify the Custom Port.

5. When the Host IP is correct, click the play button to connect with the external PC.

## Usage

For starting the driver there are two main launch files in the `ur_robot_driver` package.

  - `ur_control.launch.py` - starts ros2_control node including hardware interface, joint state broadcaster and a controller. This launch file also starts `dashboard_client` if real robot is used.
  - `ur_dashboard_client.launch.py` - start the dashboard client for UR robots.

Also, there are predefined launch files for all supported types of UR robots.

The arguments for launch files can be listed using `ros2 launch ur_robot_driver <launch_file_name>.launch.py --show-args`.
The most relevant arguments are the following:

  - `ur_type` (*mandatory*) - a type of used UR robot (*ur3*, *ur3e*, *ur5*, *ur5e*, *ur10*, *ur10e*, or *ur16e*).
  - `robot_ip` (*mandatory*) - IP address by which the root can be reached.
  - `use_fake_hardware` (default: *false*) - use simple hardware emulator from ros2_control.
    Useful for testing launch files, descriptions, etc. See explanation below.
  - `initial_positions` (default: dictionary with all joint values set to 0) - Allows passing a dictionary to set the initial joint values for the fake hardware from [ros2_control](http://control.ros.org/).  It can also be set from a yaml file with the `load_yaml` commands as follows:
      ```
      <xacro:property name="initial_positions" value="${load_yaml(initial_positions_file)}"/>
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

  - `fake_sensor_commands` (default: *false*) - enables setting sensor values for the hardware emulators.
    Useful for offline testing of controllers.
  - `robot_controller` (default: *joint_trajectory_controller*) - controller for robot joints to be started.
    Available controllers: *joint_trajectory_controller*, *scaled_joint_trajectory_controller*.
    Note: *joint_state_broadcaster*, *speed_scaling_state_broadcaster*, *force_torque_sensor_broadcaster*, and *io_and_status_controller* will always start.

    *HINT*: list all loaded controllers using `ros2 control list_controllers` command.

**NOTE**: The package can simulate hardware with the ros2_control `FakeSystem`. This emulator enables an environment for testing of "piping" of hardware and controllers, as well as testing robot's descriptions. For more details see [ros2_control documentation](https://ros-controls.github.io/control.ros.org/) for more details.

### Example Commands for Testing the Driver

Allowed UR-Type strings: `ur3`, `ur3e`, `ur5`, `ur5e`, `ur10`, `ur10e`, `ur16e`.

##### 1. Start hardware, simulator or mockup

- To do test with hardware, use:
  ```
  ros2 launch ur_robot_driver ur_control.launch.py ur_type:=<UR_TYPE> robot_ip:=<IP_OF_THE_ROBOT> launch_rviz:=true
  ```
  For more details check the argument documentation with `ros2 launch ur_robot_driver ur_control.launch.py --show-arguments`

  After starting the launch file start the external_control URCap program from the pendant, as described above.

- To do an offline test with URSim check details about it in [this section](#usage-with-official-ur-simulator)

- To use mocked hardware (capability of ros2_control), use `use_fake_hardware` argument, like:
  ```
  ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true launch_rviz:=true
  ```

  **NOTE**: Instead of using the global launch file for control stack, there are also prepeared launch files for each type of UR robots named. They accept the same arguments are the global one and are used by:
  ```
  ros2 launch ur_robot_driver <ur_type>.launch.py
  ```

##### 2. Sending commands to controllers

Before running any commands, first check the controllers' state using `ros2 control list_controllers`.

- Send some goal to the Joint Trajectory Controller by using a demo node from [ros2_control_demos](https://github.com/ros-controls/ros2_control_demos) package by starting  the following command in another terminal:
   ```
   ros2 launch ur_robot_driver test_joint_trajectory_controller.launch.py
   ```
   After a few seconds the robot should move.

- To test another controller, simply define it using `initial_joint_controller` argument, for example when using fake hardware:
   ```
   ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=yyy.yyy.yyy.yyy initial_joint_controller:=joint_trajectory_controller use_fake_hardware:=true launch_rviz:=true
   ```
   And send the command using demo node:
   ```
   ros2 launch ur_robot_driver test_scaled_joint_trajectory_controller.launch.py
   ```
   After a few seconds the robot should move (or jump when using emulation).


##### 3. Using only robot description

If you just want to test description of the UR robots, e.g., after changes you can use the following command:
   ```
   ros2 launch ur_description view_ur.launch.py ur_type:=ur5e
   ```
## Usage with official UR simulator
The docker-compose setup is prepared for usage of driver with the official UR simulator. Follow instructions [here](ur_robot_driver/resources/ursim_driver/README.md).

## Expected Changes in the Near Future

- Trajectory control currently only supports position commands. In the future, velocity control will be added.


## Contributor Guidelines
Code is auto-formatted with clang-format 10 whenever a git commit is made. Please ensure these dependencies are installed:
  ```
  pip3 install pre-commit
  sudo apt install clang-format-10
  ```

Prepare the pre-commit formatting to run like this:
  ```
  pre-commit install`
  ```

## CI setup

There are three build stages checking current and future compatibility of the driver.

1. Binary builds - against released packages (main and testing) in ROS distributions. Shows that direct local build is possible.

1. Semi-binary builds - against released core ROS packages (main and testing), but the immediate dependencies are pulled from source.
   Shows that local build with dependencies is possible and if fails there we can expect that after the next package sync we will not be able to build.

1. Source build - also core ROS packages are build from source. It shows potential issues in the mid future.
