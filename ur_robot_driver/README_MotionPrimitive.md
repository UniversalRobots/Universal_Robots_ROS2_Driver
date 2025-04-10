ur_robot_motion_primitives_driver
==========================================

Driver package to control ur robot using motion primitives like MOVEJ (PTP), MOVEL (LIN) and MOVEC (CIRC)

![Licence](https://img.shields.io/badge/License-BSD-3-Clause-blue.svg)

# Other packages needed
- [generate_parameter_library](https://github.com/PickNikRobotics/generate_parameter_library)
- [industrial_robot_motion_interfaces (with additional helper types for stop and motion sequence)](https://github.com/StoglRobotics-forks/industrial_robot_motion_interfaces/tree/helper-types)
- [motion_primitives_forward_controller from StoglRobotics-forks/ros2_controllers](https://github.com/StoglRobotics-forks/ros2_controllers/tree/motion_primitive_forward_controller/motion_primitives_forward_controller)
- [Universal_Robots_ROS2_Driver from StoglRobotics-forks/Universal_Robots_ROS2_Driver_MotionPrimitive](https://github.com/StoglRobotics-forks/Universal_Robots_ROS2_Driver_MotionPrimitive)
- [Universal_Robots_Client_Library with movec from urfeex](https://github.com/urfeex/Universal_Robots_Client_Library/tree/movec_movep)
 

# Notes
## Launch "normal" ur driver
```
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=172.20.0.2 launch_rviz:=true
```
## Launch motion_primitives_ur_driver
```
ros2 launch ur_robot_driver motion_primitive_controller_ur.launch.py ur_type:=ur5e robot_ip:=172.20.0.2 launch_rviz:=true
```
## Publish dummy commands
```
ros2 run ur_robot_driver send_dummy_motion_primitives.py
```
## Publish stop motion command 
```
ros2 topic pub /motion_primitive_controller/reference industrial_robot_motion_interfaces/msg/MotionPrimitive "{type: 66, blend_radius: 0.0, additional_arguments: [], poses: [], joint_positions: []}" --once

```

## Start UR-Sim
```
docker run --rm -it universalrobots/ursim_e-series
```

## Enable Remote Control on UR
Remote control needs to be enabled:
https://robodk.com/doc/en/Robots-Universal-Robots-How-enable-Remote-Control-URe.html



# TODO's
- if trajectory is finished while `instruction_executer->cancelMotion()` is called --> returns with execution_status ERROR --> no new command can be sent to hw-interface --> need to call `instruction_executer->cancelMotion()` a second time
- for the motion primitive driver `ur_joint_control.xacro` without command interfaces is needed: `motion_primitive_ur_joint_control.xacro` --> is there a better way than a copy of the file with commented command interfaces?

# Useful sources
- https://rtw.b-robotized.com/master/use-cases/ros_workspaces/aliases.html
- https://control.ros.org/master/doc/ros2_control/ros2controlcli/doc/userdoc.html
- ...