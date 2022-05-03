# Resources about the ROS 2 Universal Robot driver

The files in this folder are available for use under BSD-3-Clause license.


## Diagrams and images

- [ros2_control diagrams](ros2_control_ur_driver.drawio) - `.drawio` file, Author: [Dr.-Ing. Denis Stogl](mailto:denis@stoglrobotics.de) (PickNik Robotics)


## 2021-10 ROS World presentation

[Presentation: Making a robot ROS 2 powered - a case study using the UR manipulators](2021-10_ROS_World_2021_Making_a_robot_ROS2_powered.pdf)

**Summary:**

With the release of ros2_control and MoveIt 2, ROS 2 Foxy finally has all the “ingredients” needed to power a robot with similar features as in ROS 1. We present the driver for Universal Robot’s manipulators as a real-world example of how robots can be run using ROS 2. We show how to realize multi-interface support for position and velocity commands in the driver and how to support scaling controllers while respecting factors set on the teach pendant. Finally, we show how this real-world example influences development of ros2_control to support non-joint related inputs and outputs in its real-time control loop.

**Presenter:** *Dr.-Ing. Denis Štogl*

**Authors:**
  - Dr.-Ing. Denis Štogl (PickNik Inc.)
  - Dr. Nathan Brooks (PickNik Inc.)
  - Lovro Ivanov (PickNik Inc.)
  - Dr. Andy Zelenak (PickNik Inc.)
  - Rune Søe-Knudsen (Universal Robots A/S)

#### [Video] Presentation recording
[![Recording](https://i.vimeocdn.com/video/1309381824-05663ce76ceac80f043bc50addcc7c4874da323145c0df0df_640)](https://vimeo.com/649651707)


#### [Video] MoveIt2 Demo
[![Video: MoveIt2 Demo](https://img.youtube.com/vi/d_cVXoZZ52w/0.jpg)](https://www.youtube.com/watch?v=d_cVXoZZ52w)

This video shows free-space trajectory planning around a modeled collision scene object using the MoveIt2 MotionPlanning widget for Rviz2.

#### [Video] Scaled Joint Trajectory Controller Demo
[![Video: Scaled Joint Trajectory Controller Demo](https://img.youtube.com/vi/dHaxBpMGbw0/0.jpg)](https://www.youtube.com/watch?v=dHaxBpMGbw0)

This video demonstrates the following features:
- [[0:00](https://www.youtube.com/watch?v=dHaxBpMGbw0&t=0s)] - Previewing a plan with MoveIt2 MotionPlanning widget for Rviz2
- [[0:05](https://www.youtube.com/watch?v=dHaxBpMGbw0&t=5s)] - Executing a plan with MoveIt2 MotionPlanning widget for Rviz2
- [[0:09](https://www.youtube.com/watch?v=dHaxBpMGbw0&t=9s)] - Pendant speed scaling of a simulated MoveIt trajectory
- [[0:18](https://www.youtube.com/watch?v=dHaxBpMGbw0&t=18s)] - Pendant speed scaling of an executed MoveIt trajectory
- [[0:22](https://www.youtube.com/watch?v=dHaxBpMGbw0&t=22s)] - Online pendant speed scaling during trajectory execution
- [[0:26](https://www.youtube.com/watch?v=dHaxBpMGbw0&t=26s)] - Resuming trajectory execution from E-stop
