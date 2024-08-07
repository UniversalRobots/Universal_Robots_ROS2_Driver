# Universal Robots ROS2 Driver

Universal Robots has become a dominant supplier of lightweight, robotic manipulators for industry, as well as for scientific research and education.
<center><img src="ur_robot_driver/doc/installation/initial_setup_images/family_photo.png" alt="Universal Robot family" style="width: 80%;"/></center>

This is one of the very first ROS2 manipulator drivers. Some of the new features are enabled by ROS2 and include decreased latency, improved security, and more flexibility regarding middleware configuration. The package contains launch files to quickly get started using the driver as a standalone version or in combination with MoveIt2

This driver is developed on top of [Universal_Robots_Client_Library](https://github.com/UniversalRobots/Universal_Robots_Client_Library) and support some key cobot functionalities like; pause at emergency stop, safeguard stop, automatic speed scaling to avoid violate the safety setting and manually speed scaling from the teach pendant. In addition the externalControl URCap makes it possible to include ROS2 behaviors in the robot program.

The driver is compatible across the entire line of UR robots -- from 3 kg payload to 30 kg payload and includes all robots from the CB3 series and newer.


Check also [presentations and videos](ur_robot_driver/doc/resources/README.md) about this driver.


## Release Status

<table width="100%">
  <tr>
    <th>ROS2 Distro</th>
    <th>Foxy (EOL)</th>
    <th>Galactic (EOL)</th>
    <th>Humble</th>
    <th>Iron</th>
    <th>Rolling</th>
  </tr>
  <tr>
    <th>Branch</th>
    <td><a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/foxy">foxy</a></td>
    <td><a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/galactic">galactic</a></td>
    <td><a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble">humble</a></td>
    <td><a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/iron">iron</a></td>
    <td><a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/main">main</a></td>
  </tr>
  <tr>
    <th>Release status</th>
    <td>
      <a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/actions/workflows/foxy-binary-build.yml?query=event%3Aschedule++">
         <img src="https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/actions/workflows/foxy-binary-build.yml/badge.svg?event=schedule"
              alt="Foxy Binary Build"/>
      </a> <br />
    </td>
    <td>
      <a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/actions/workflows/galactic-binary-build.yml?query=event%3Aschedule++">
         <img src="https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/actions/workflows/galactic-binary-build.yml/badge.svg?event=schedule"
              alt="Galactic Binary Build"/>
      </a> <br />
    </td>
    <td>
      <a href='https://build.ros2.org/job/Hbin_uJ64__ur_calibration__ubuntu_jammy_amd64__binary/'><img src='https://build.ros2.org/job/Hbin_uJ64__ur_calibration__ubuntu_jammy_amd64__binary/badge/icon?subject=ur_calibration'></a><br/>
      <a href='https://build.ros2.org/job/Hbin_uJ64__ur_controllers__ubuntu_jammy_amd64__binary/'><img src='https://build.ros2.org/job/Hbin_uJ64__ur_controllers__ubuntu_jammy_amd64__binary/badge/icon?subject=ur_controllers'></a>
      <a href='https://build.ros2.org/job/Hbin_uJ64__ur_dashboard_msgs__ubuntu_jammy_amd64__binary/'><img src='https://build.ros2.org/job/Hbin_uJ64__ur_dashboard_msgs__ubuntu_jammy_amd64__binary/badge/icon?subject=ur_dashboard_msgs'></a>
      <a href='https://build.ros2.org/job/Hbin_uJ64__ur_moveit_config__ubuntu_jammy_amd64__binary/'><img src='https://build.ros2.org/job/Hbin_uJ64__ur_moveit_config__ubuntu_jammy_amd64__binary/badge/icon?subject=ur_moveit_config'></a>
      <a href='https://build.ros2.org/job/Hbin_uJ64__ur_robot_driver__ubuntu_jammy_amd64__binary/'><img src='https://build.ros2.org/job/Hbin_uJ64__ur_robot_driver__ubuntu_jammy_amd64__binary/badge/icon?subject=ur_robot_driver'></a>
    </td>
    <td>
      <a href='https://build.ros2.org/job/Ibin_uJ64__ur_calibration__ubuntu_jammy_amd64__binary/'><img src='https://build.ros2.org/job/Ibin_uJ64__ur_calibration__ubuntu_jammy_amd64__binary/badge/icon?subject=ur_calibration'></a><br/>
      <a href='https://build.ros2.org/job/Ibin_uJ64__ur_controllers__ubuntu_jammy_amd64__binary/'><img src='https://build.ros2.org/job/Ibin_uJ64__ur_controllers__ubuntu_jammy_amd64__binary/badge/icon?subject=ur_controllers'></a>
      <a href='https://build.ros2.org/job/Ibin_uJ64__ur_dashboard_msgs__ubuntu_jammy_amd64__binary/'><img src='https://build.ros2.org/job/Ibin_uJ64__ur_dashboard_msgs__ubuntu_jammy_amd64__binary/badge/icon?subject=ur_dashboard_msgs'></a>
      <a href='https://build.ros2.org/job/Ibin_uJ64__ur_moveit_config__ubuntu_jammy_amd64__binary/'><img src='https://build.ros2.org/job/Ibin_uJ64__ur_moveit_config__ubuntu_jammy_amd64__binary/badge/icon?subject=ur_moveit_config'></a>
      <a href='https://build.ros2.org/job/Ibin_uJ64__ur_robot_driver__ubuntu_jammy_amd64__binary/'><img src='https://build.ros2.org/job/Ibin_uJ64__ur_robot_driver__ubuntu_jammy_amd64__binary/badge/icon?subject=ur_robot_driver'></a>
    </td>
    <td>
      <a href='https://build.ros2.org/job/Rbin_uJ64__ur_calibration__ubuntu_jammy_amd64__binary/'><img src='https://build.ros2.org/job/Rbin_uJ64__ur_calibration__ubuntu_jammy_amd64__binary/badge/icon?subject=ur_calibration'></a><br/>
      <a href='https://build.ros2.org/job/Rbin_uJ64__ur_controllers__ubuntu_jammy_amd64__binary/'><img src='https://build.ros2.org/job/Rbin_uJ64__ur_controllers__ubuntu_jammy_amd64__binary/badge/icon?subject=ur_controllers'></a>
      <a href='https://build.ros2.org/job/Rbin_uJ64__ur_dashboard_msgs__ubuntu_jammy_amd64__binary/'><img src='https://build.ros2.org/job/Rbin_uJ64__ur_dashboard_msgs__ubuntu_jammy_amd64__binary/badge/icon?subject=ur_dashboard_msgs'></a>
      <a href='https://build.ros2.org/job/Rbin_uJ64__ur_moveit_config__ubuntu_jammy_amd64__binary/'><img src='https://build.ros2.org/job/Rbin_uJ64__ur_moveit_config__ubuntu_jammy_amd64__binary/badge/icon?subject=ur_moveit_config'></a>
      <a href='https://build.ros2.org/job/Rbin_uJ64__ur_robot_driver__ubuntu_jammy_amd64__binary/'><img src='https://build.ros2.org/job/Rbin_uJ64__ur_robot_driver__ubuntu_jammy_amd64__binary/badge/icon?subject=ur_robot_driver'></a>
    </td>
  </tr>
</table>

The table above shows the build status for each package of this repo from the [ROS buildfarm](https://build.ros2.org/). For end-of-life (EOL) distributions the nightly binary builds from our CI are shown. EOL distributions will receive no more updates and may be lacking features.

A more [detailed build status](ci_status.md) shows the state of all CI workflows inside this repo.
Please note that the detailed view is intended for developers, while the one here should give end
users an overview of the current released state.


## Packages in the Repository:

  - `ur` - Meta-package that provides a single point of installation for the released packages.
  - `ur_bringup` - launch file and run-time configurations, e.g. controllers (DEPRECATED).
  - `ur_calibration` - tool for extracting calibration information from a real robot.
  - `ur_controllers` - implementations of controllers specific for UR robots.
  - `ur_dashboard_msgs` - package defining messages used by dashboard node.
  - `ur_moveit_config` - example MoveIt configuration for UR robots.
  - `ur_robot_driver` - driver / hardware interface for communication with UR robots.

Deprecation: The `ur_bringup` package is deprecated and will be removed from Iron Irwini on.

## System Requirements

Please see the [requirements for the Universal_Robots_Client_Library](https://github.com/UniversalRobots/Universal_Robots_Client_Library#requirements), as this driver is build on top of Universal_Robots_Client_Library.

## Getting Started

For getting started, you'll basically need three steps:

1. **Install the driver**
   ```bash
   sudo apt-get install ros-humble-ur
   ```
   See the [installation instructions](https://docs.ros.org/en/ros2_packages/humble/api/ur_robot_driver/installation/installation.html) for more details and source-build instructions.

2. **Start & Setup the robot**. Once you've installed the driver, [setup the
   robot](https://docs.ros.org/en/ros2_packages/humble/api/ur_robot_driver/installation/robot_setup.html)
   and [create a program for external
   control](https://docs.ros.org/en/ros2_packages/humble/api/ur_robot_driver/installation/install_urcap_e_series.html).

   Please do this step carefully and extract the calibration as explained
   [here](https://docs.ros.org/en/ros2_packages/humble/api/ur_robot_driver/installation/robot_setup.html#extract-calibration-information).
   Otherwise the TCP's pose will not be correct inside the ROS ecosystem.

   If no real robot is required, you can [use a simulated
   robot](https://docs.ros.org/en/ros2_packages/humble/api/ur_robot_driver/usage.html#usage-with-official-ur-simulator)
   that will behave almost exactly like the real robot.

3. **Start the driver**. See the [usage
   documentation](https://docs.ros.org/en/ros2_packages/humble/api/ur_robot_driver/usage.html) for
   details.

   ```bash
   # Replace ur5e with one of ur3, ur3e, ur5, ur5e, ur10, ur10e, ur16e, ur20, ur30
   # Replace the IP address with the IP address of your actual robot / URSim
   ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.56.101
   ```

4. Unless started in [headless mode](https://docs.ros.org/en/ros2_packages/humble/api/ur_robot_driver/ROS_INTERFACE.html#headless-mode): Run the external_control program by **pressing `play` on the teach pendant**.


## MoveIt! support

[MoveIt!](https://moveit.ros.org) support is built-in into this driver already.
Watch MoveIt in action with the Universal Robots ROS2 driver:

[![Video: MoveIt2 Demo](https://img.youtube.com/vi/d_cVXoZZ52w/0.jpg)](https://www.youtube.com/watch?v=d_cVXoZZ52w)

  *The video shows free-space trajectory planning around a modeled collision scene object using the MoveIt2 MotionPlanning widget for Rviz2.*

See the [MoveIt!
section](https://docs.ros.org/en/ros2_packages/humble/api/ur_robot_driver/usage.html#using-moveit)
of the [Usage guide](https://docs.ros.org/en/ros2_packages/humble/api/ur_robot_driver/usage.html)
for details.



## Contributor Guidelines
pre-commit is used to run a couple of checks before committing. To install it, run:
  ```
  pip3 install pre-commit
  ```

Prepare the pre-commit formatting to run like this:
  ```
  pre-commit install
  ```
