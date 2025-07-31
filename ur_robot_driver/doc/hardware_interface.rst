:github_url: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/main/ur_robot_driver/doc/hardware_interface.rst

UR Hardware interface
=====================

The UR hardware interface is the core piece of the ROS driver. It is responsible for communicating
with the robot controller, sending commands and receiving status updates.

The hardware interface is implemented using the ``ros2_control`` framework, which allows for modular
and flexible control of the robot.

.. note::
   The hardware interface itself doesn't define how the robot's motion can be controlled through
   ROS. For that, a **controller** is needed. There are many controllers to choose from, such as
   the ``JointTrajectoryController`` or the ``ForceModeController``. See `ros2_controllers
   <https://control.ros.org/rolling/doc/ros2_controllers/doc/controllers_index.html#controllers-for-manipulators-and-other-robots>`_
   for "standard" controllers and :ref:`ur_controllers` for more information on UR-specific
   controllers

Supported control modes
-----------------------

The UR hardware interface supports the following control modes:

- **Position control**: The robot's joints are controlled by specifying target positions.
- **Velocity control**: The robot's joints are controlled by specifying target velocities.
- **Effort control**: The robot's joints are controlled by specifying target efforts (torques).
  (Only available when running PolyScope >= 5.23.0 / 10.10.0)
- **Force control**: The robot's end-effector is controlled by specifying target forces
  in Cartesian space.
- **Freedrive mode**: The robot can be moved freely by the user without any active control.
- **Passthrough Trajectory control**: Complete trajectory points are forwarded to the robot for
  interpolation and execution.
- **Tool contact mode**: The robot stops when the tool comes into contact with an object, allowing for
  safe interaction with the environment.
- **Speed scaling**: Speed scaling on the robot can be read and written through the hardware
  interface.
- **GPIO**: Digital and analog I/O pins can be read and written through the hardware interface.
- **Payload**: Payload configuration can be changed during runtime through the hardware interface.
- **Force torque sensor**: Force torque sensor data can be read through the hardware interface.
  Zeroing the sensor is also supported.

Interacting with the hardware interface
---------------------------------------

As stated above, motion control is done through controllers. However, the ros2_control framework
provides a set of services to interact with the hardware interface directly. These services can be
comfortably used through the ``ros2 control`` `command line tool
<https://control.ros.org/rolling/doc/ros2_control/ros2controlcli/doc/userdoc.html>`_.

E.g. ``ros2 control list_hardware_components`` will list all hardware components, including the UR
hardware interface with its interfaces as listed above.
