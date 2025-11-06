hardware_interface in motion primitives mode
==========================================

Hardware interface for executing motion primitives on a UR robot using the ROS 2 control framework. It allows the controller to execute linear (LINEAR_CARTESIAN/ LIN/ MOVEL), circular (CIRCULAR_CARTESIAN/ CIRC/ MOVEC), and joint-based (LINEAR_JOINT/ PTP/ MOVEJ) motion commands asynchronously and supports motion sequences for smooth trajectory execution.

[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

# Demo Video with motion_primitives_forward_controller
[![Play Video](doc/motion_primitive_ur_driver/moprim_forward_controller_ur_demo_thumbnail.png)](https://youtu.be/SKz6LFvJmhQ)

# Architecture
**with motion_primitives_forward_controller**
![Architecture Overview](doc/motion_primitive_ur_driver/ros2_control_motion_primitives_ur.drawio.png)

# Command and State Interfaces

In motion primitives mode, the following state and command interfaces are used to enable communication between the controller and the hardware interface.

## Command Interfaces

These interfaces are used to send motion primitive data to the hardware interface:

- `motion_type`: Type of motion primitive (LINEAR_JOINT, LINEAR_CARTESIAN, CIRCULAR_CARTESIAN)
- `q1` – `q6`: Target joint positions for joint-based motion
- `pos_x`, `pos_y`, `pos_z`: Target Cartesian position
- `pos_qx`, `pos_qy`, `pos_qz`, `pos_qw`: Orientation quaternion of the target pose
- `pos_via_x`, `pos_via_y`, `pos_via_z`: Intermediate via-point position for circular motion
- `pos_via_qx`, `pos_via_qy`, `pos_via_qz`, `pos_via_qw`: Orientation quaternion of via-point
- `blend_radius`: Blending radius for smooth transitions
- `velocity`: Desired motion velocity
- `acceleration`: Desired motion acceleration
- `move_time`: Optional duration for time-based execution (For LINEAR_JOINT and LINEAR_CARTESIAN. If move_time > 0, velocity and acceleration are ignored)

## State Interfaces

These interfaces are used to communicate the internal status of the hardware interface back to the controller.

- `execution_status`: Indicates the current execution state of the primitive. Possible values are:
  - `IDLE`: No motion in progress
  - `EXECUTING`: Currently executing a primitive
  - `SUCCESS`: Last command finished successfully
  - `ERROR`: An error occurred during execution
  - `STOPPING`: The hardware interface has received the `STOP_MOTION` command, but the robot has not yet come to a stop.
  - `STOPPED`: The robot was stopped using the `STOP_MOTION` command and must be reset with the `RESET_STOP` command before executing new commands.
- `ready_for_new_primitive`: Boolean flag indicating whether the interface is ready to receive a new motion primitive

In addition to these, the driver also provides all standard state interfaces from the original UR hardware interface (e.g., joint positions, velocities). These are used by components like the `joint_state_broadcaster` and allow tools like RViz to visualize the current robot state.


# Supported Motion Primitives

- Support for basic motion primitives:
  - `LINEAR_JOINT`
  - `LINEAR_CARTESIAN`
  - `CIRCULAR_CARTESIAN`
- Additional helper types:
  - `STOP_MOTION`: Immediately stops the current robot motion and clears all pending primitives in the controller's queue.
  - `RESET_STOP`: After `RESET_STOP`, new commands can get handled.
  - `MOTION_SEQUENCE_START` / `MOTION_SEQUENCE_END`: Define a motion sequence block. All primitives between these two markers will be executed as a single, continuous sequence. This allows seamless transitions (blending) between primitives.

![MotionPrimitiveExecutionWithHelperTypes](doc/motion_primitive_ur_driver/MotionPrimitiveExecutionWithHelperTypes_UR.drawio.png)

# Overview

In contrast to the standard UR hardware interface, this driver does not compute or execute trajectories on the ROS 2 side. Instead, it passes high-level motion primitives directly to the robot controller, which then computes and executes the trajectory internally.

This approach offers two key advantages:

- **Reduced real-time requirements** on the ROS 2 side, since trajectory planning and execution are offloaded to the robot.
- **Improved motion quality**, as the robot controller has better knowledge of the robot's kinematics and dynamics, leading to more optimized and accurate motion execution.


## write() Method

In motion primitives mode, the `write()` method checks whether a new motion primitive command has been received from the controller via the command interfaces. If a new command is present:

1. If the command is `STOP_MOTION`, a flag is set which leads to interrupting the current motion inside the `asyncStopMotionThread()`. If the command is `RESET_STOP`, the flag is reset, and new motion primitives can be received and executed.
2. For other commands, they are passed to the `asyncCommandThread()` and executed asynchronously. Individual primitives are executed directly via the Instruction Executor.
If a `MOTION_SEQUENCE_START` command is received, all subsequent primitives are added to a motion sequence. Once `MOTION_SEQUENCE_END` is received, the entire sequence is executed via the Instruction Executor.

Threading is required since calls to the Instruction Executor are blocking. Offloading these to separate threads ensures the control loop remains responsive during motion execution. The stopping functionality is also threaded to allow interrupting a primitive even during execution or in a motion sequence.

## read() Method

The `read()` method:

- Publishes the `execution_status` over a state interface with possible values: `IDLE`, `EXECUTING`, `SUCCESS`, `ERROR`, `STOPPED`.
- Publishes `ready_for_new_primitive` over a state interface to signal whether the interface is ready to receive a new primitive.
- Handles additional state interfaces from the UR driver, such as joint states, enabling RViz to visualize the current robot pose.


# Example usage notes with UR10e
## (optional) URSim
Start UR-Sim according to the [Universal Robots ROS 2 Driver Documentation](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_client_library/doc/setup/ursim_docker.html) or the [Documentation for universalrobots/ursim_e-series docker container](https://hub.docker.com/r/universalrobots/ursim_e-series)
```
ros2 run ur_client_library start_ursim.sh -m ur10e
```
Remote control needs to be enabled:
https://robodk.com/doc/en/Robots-Universal-Robots-How-enable-Remote-Control-URe.html

## With motion_primitives_forward_controller
**With URSim:**
```
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur10e robot_ip:=192.168.56.101 launch_rviz:=true headless_mode:=true initial_joint_controller:=motion_primitive_forward_controller
```
**With H-KA UR10e:**
```
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur10e robot_ip:=192.168.1.102 launch_rviz:=true headless_mode:=true initial_joint_controller:=motion_primitive_forward_controller
```
**(optional) switching control mode**
```
ros2 control switch_controllers --activate motion_primitive_forward_controller --deactivate scaled_joint_trajectory_controller
```
**Send motion primitives from python script**
> [!WARNING]
> Ensure that the robot in your configuration is able to execute these motion primitives without any risk of collision.
```
ros2 run ur_robot_driver send_dummy_motion_primitives_ur10e.py
```
During the execution of the motion primitives, the movement can be stopped by pressing the Enter key in the terminal.

# TODOs/ Improvements
- if trajectory is finished while `instruction_executer->cancelMotion()` is called --> returns with execution_status ERROR --> no new command can be sent to hw-interface --> need to call `instruction_executer->cancelMotion()` a second time
