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
    <td><a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble">humble</a></td>
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
      </a>
    </td>
    <td>
      <a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/actions/workflows/humble-binary-build.yml?query=event%3Aschedule++">
         <img src="https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/actions/workflows/humble-binary-build.yml/badge.svg?event=schedule"
              alt="Humble Binary Build"/>
      </a> <br />
      <a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/actions/workflows/humble-semi-binary-build.yml?query=event%3Aschedule++">
         <img src="https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/actions/workflows/humble-semi-binary-build.yml/badge.svg?event=schedule"
              alt="Humble Semi-Binary Build"/>
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
      </a>
    </td>
  </tr>
</table>


**NOTE**: There are two build stages checking current and future compatibility of the driver.

1. Binary builds - against released packages (main and testing) in ROS distributions. Shows that direct local build is possible.

   Uses repos file: `src/Universal_Robots_ROS2_Driver/Universal_Robots_ROS2_Driver-not-released.<ros-distro>.repos`

1. Semi-binary builds - against released core ROS packages (main and testing), but the immediate dependencies are pulled from source.
   Shows that local build with dependencies is possible and if fails there we can expect that after the next package sync we will not be able to build.

   Uses repos file: `src/Universal_Robots_ROS2_Driver/Universal_Robots_ROS2_Driver.repos`

Each of these stages also performs integration tests using ursim. In order to execute these tests locally, they have to be enabled:
  ```
  colcon build --packages-select ur_robot_driver --cmake-args -DUR_ROBOT_DRIVER_BUILD_INTEGRATION_TESTS=On
  ```


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

For getting started, you'll basically need three steps:

1. **Install the driver** (see below). You can either install this driver from binary packages or build it from source. We recommend a
binary package installation unless you want to join development and submit changes.

2. **Start & Setup the robot**. Once you've installed the driver, [setup the
   robot](https://docs.ros.org/en/ros2_packages/humble/api/ur_robot_driver/installation/robot_setup.html)

Please do this step carefully and extract the calibration as explained
[here](https://docs.ros.org/en/ros2_packages/humble/api/ur_robot_driver/installation/robot_setup.html#extract-calibration-information).
Otherwise the TCP's pose will not be correct inside the ROS ecosystem.

If no real robot is required, you can [use a simulated
robot](https://docs.ros.org/en/ros2_packages/humble/api/ur_robot_driver/usage.html#usage-with-official-ur-simulator)
that will behave almost exactly like the real robot.


3. **Start the driver**. See the [usage
   documentation](https://docs.ros.org/en/ros2_packages/humble/api/ur_robot_driver/usage.html) for
   details.

### Install from binary packages
1. [Install ROS2](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html). This
      branch supports only ROS2 Humble. For other ROS2 versions, please see the respective
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

1. [Install ROS2](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).
   branch supports only ROS2 Humble. For other ROS2 versions, please see the respective
   branches.

   Once installed, please make sure to actually [source ROS2](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html#source-the-setup-files) before proceeding.

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
   git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git src/Universal_Robots_ROS2_Driver
   vcs import src --skip-existing --input src/Universal_Robots_ROS2_Driver/Universal_Robots_ROS2_Driver-not-released.${ROS_DISTRO}.repos
   rosdep update
   rosdep install --ignore-src --from-paths src -y
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
   source install/setup.bash
   ```

6. When consecutive pulls leads to build errors, please make sure to update the upstream packages before
   filing an issue:
   ```
   cd $COLCON_WS
   vcs import src --skip-existing --input src/Universal_Robots_ROS2_Driver/Universal_Robots_ROS2_Driver-not-released.${ROS_DISTRO}.repos
   rosdep update
   rosdep install --ignore-src --from-paths src -y
   ```

## MoveIt! support

[MoveIt!](https://moveit.ros.org) support is built-in into this driver already.
Watch MoveIt in action with the Universal Robots ROS2 driver:

[![Video: MoveIt2 Demo](https://img.youtube.com/vi/d_cVXoZZ52w/0.jpg)](https://www.youtube.com/watch?v=d_cVXoZZ52w)

  *The video shows free-space trajectory planning around a modeled collision scene object using the MoveIt2 MotionPlanning widget for Rviz2.*

See the [MoveIt!
section](https://docs.ros.org/en/ros2_packages/humble/api/ur_robot_driver/usage.html#using-moveit)
of the [Usage guide](https://docs.ros.org/en/ros2_packages/humble/api/ur_robot_driver/usage.html)
for details.

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
