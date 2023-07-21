# Universal Robots ROS2 Driver

Universal Robots has become a dominant supplier of lightweight, robotic manipulators for industry, as well as for scientific research and education.
<center><img src="ur_robot_driver/doc/installation/initial_setup_images/e-Series.jpg" alt="Universal Robot e-Series family" style="width: 80%;"/></center>

This is one of the very first ROS2 manipulator drivers. Some of the new features are enabled by ROS2 and include decreased latency, improved security, and more flexibility regarding middleware configuration. The package contains launch files to quickly get started using the driver as a standalone version or in combination with MoveIt2

This driver is developed on top of [Universal_Robots_Client_Library](https://github.com/UniversalRobots/Universal_Robots_Client_Library) and support some key cobot functionalities like; pause at emergency stop, safeguard stop, automatic speed scaling to avoid violate the safety setting and manually speed scaling from the teach pendant. In addition the externalControl URCap makes it possible to include ROS2 behaviors in the robot program.

The driver is compatible across the entire line of UR robots -- from 3 kg payload to 16 kg payload and includes both the CB3 and the E-series.


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
  - `ur_calibration` - tool for extracting calibration information from a real robot.
  - `ur_controllers` - implementations of controllers specific for UR robots.
  - `ur_dashboard_msgs` - package defining messages used by dashboard node.
  - `ur_moveit_config` - example MoveIt configuration for UR robots.
  - `ur_robot_driver` - driver / hardware interface for communication with UR robots.

## Getting Started

For getting started, you'll basically need three steps:

1. **Install the driver** (see below). You can either install this driver from binary packages or build it from source. We recommend a
binary package installation unless you want to join development and submit changes.

2. **Start & Setup the robot**. Once you've installed the driver, [setup the
robot](https://docs.ros.org/en/ros2_packages/rolling/api/ur_robot_driver/installation/robot_setup.html)

Please do this step carefully and extract the calibration as explained
[here](https://docs.ros.org/en/ros2_packages/rolling/api/ur_robot_driver/installation/robot_setup.html#extract-calibration-information).
Otherwise the TCP's pose will not be correct inside the ROS ecosystem.

If no real robot is required, you can [use a simulated
robot](https://docs.ros.org/en/ros2_packages/rolling/api/ur_robot_driver/usage.html#usage-with-official-ur-simulator)
that will behave almost exactly like the real robot.


3. **Start the driver**. See the [usage
   documentation](https://docs.ros.org/en/ros2_packages/rolling/api/ur_robot_driver/usage.html) for
   details.

### Install from binary packages
1. [Install ROS2](https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debians.html). This
      branch supports only ROS2 Rolling. For other ROS2 versions, please see the respective
      branches.
2. Install the driver using
   ```
   sudo apt-get install ros-${ROS_DISTRO}-ur
   ```

### Build from source
Before building from source please make sure that you actually need to do that. Building from source
might require some special treatment, especially when it comes to dependency management.
Dependencies might change from time to time. Upstream packages (such as the library) might change
their features / API which require changes in this repo. Therefore, this repo's source builds might
require upstream repositories to be present in a certain version as otherwise builds might fail.
Starting from scratch following exactly the steps below should always work, but simply pulling and
building might fail occasionally.

1. [Install ROS2](https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debians.html). This
      branch supports only ROS2 Rolling. For other ROS2 versions, please see the respective
      branches.

   Once installed, please make sure to actually [source ROS2](https://docs.ros.org/en/rolling/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html#source-the-setup-files) before proceeding.

3. Make sure that `colcon`, its extensions and `vcs` are installed:
   ```
   sudo apt install python3-colcon-common-extensions python3-vcstool
   ```

4. Create a new ROS2 workspace:
   ```
   export COLCON_WS=~/workspace/ros_ur_driver
   mkdir -p $COLCON_WS/src
   ```

5. Clone relevant packages, install dependencies, compile, and source the workspace by using:
   ```
   cd $COLCON_WS
   git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git src/Universal_Robots_ROS2_Driver
   vcs import src --skip-existing --input src/Universal_Robots_ROS2_Driver/Universal_Robots_ROS2_Driver-not-released.${ROS_DISTRO}.repos
   rosdep update
   rosdep install --ignore-src --from-paths src -y
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
   source install/setup.bash
   ```

6. When consecutive pulls lead to build errors it is possible that you'll have to build an upstream
   package from source, as well. See the [detailed build status](ci_status.md). When the binary builds are red, but
   the semi-binary builds are green, you need to build the upstream dependencies from source. The
   easiest way to achieve this, is using the repos file:

   ```
   cd $COLCON_WS
   vcs import src --skip-existing --input src/Universal_Robots_ROS2_Driver/Universal_Robots_ROS2_Driver.${ROS_DISTRO}.repos
   rosdep update
   rosdep install --ignore-src --from-paths src -y
   ```

## MoveIt! support

[MoveIt!](https://moveit.ros.org) support is built-in into this driver already.
Watch MoveIt in action with the Universal Robots ROS2 driver:

[![Video: MoveIt2 Demo](https://img.youtube.com/vi/d_cVXoZZ52w/0.jpg)](https://www.youtube.com/watch?v=d_cVXoZZ52w)

  *The video shows free-space trajectory planning around a modeled collision scene object using the MoveIt2 MotionPlanning widget for Rviz2.*

See the [MoveIt!
section](https://docs.ros.org/en/ros2_packages/rolling/api/ur_robot_driver/usage.html#using-moveit)
of the [Usage
guide](https://docs.ros.org/en/ros2_packages/rolling/api/ur_robot_driver/usage.html) for details.

## Expected Changes in the Near Future

- Trajectory control currently only supports position commands. In the future, velocity control will be added.


## Contributor Guidelines
Code is auto-formatted with clang-format 14 whenever a git commit is made. Please ensure these dependencies are installed:
  ```
  pip3 install pre-commit
  sudo apt install clang-format-14
  ```

Prepare the pre-commit formatting to run like this:
  ```
  pre-commit install
  ```
