# Universal Robots ROS2 Driver

Alpha version of the ROS2 Universal Robots driver. Should be transferred to the Universal Robots org when ready.

## Build Instructions

Build MoveIt2 from source, per instructions here:

- https://moveit.ros.org/install-moveit2/source/

Clone the Universal_Robots_ROS2_Driver into /src:

`git clone https://github.com/PickNikRobotics/Universal_Robots_ROS2_Driver.git`

Clone additional dependencies of the UR driver:

`cd $COLCON_WS`

`vcs import --skip-existing --input src/Universal_Robots_ROS2_Driver/.repos.yaml src`

Build:

`colcon build`

## Run a Simulated MoveIt Demo

`ros2 launch ur_ros2_control_demos ur5_e_run_move_group.launch.py`

## Run MoveIt on hardware

TODO

## General driver information
The driver currently only supports position joint interface which means only position-based controllers can be used with 
the ROS2 driver. [Universal Robots Client Library](https://github.com/UniversalRobots/Universal_Robots_Client_Library) includes also a
velocity-based control whose support will be addressed in additional development of ROS2 driver.
