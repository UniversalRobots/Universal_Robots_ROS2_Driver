Controllers
===========

This help page describes the different controllers available for the ``ur_robot_driver``. This
should help users finding the right controller for their specific use case.

Where are controllers defined?
------------------------------

Controllers are defined in the ``config/ur_controllers.yaml`` file.

How do controllers get loaded and started?
------------------------------------------

As this driver uses `ros2_control <https://control.ros.org>`_ all controllers are managed by the
controller_manager. During startup, a default set of running controllers is loaded and started,
another set is loaded in stopped mode. Stopped controllers won't be usable right away, but they
need to be started individually.

Controllers that are actually writing to some command interfaces (e.g. joint positions) will claim
those interfaces. Only one controller claiming a certain interface can be active at one point.

Controllers can be switched either through the controller_manager's service calls, through the
`rqt_controller_manager <https://control.ros.org/rolling/doc/ros2_control/controller_manager/doc/userdoc.html#rqt-controller-manager>`_ gui or through the ``ros2 control`` verb from the package ``ros-${ROS_DISTRO}-ros2controlcli`` package.

For example, to switch from the default ``scaled_joint_trajectory_controller`` to the
``forward_position_controller`` you can call

.. code-block:: console

   $ ros2 control switch_controllers --deactivate scaled_joint_trajectory_controller \
     --activate forward_position_controller
   [INFO 2024-09-23 20:32:04.373] [_ros2cli_1207798]: waiting for service /controller_manager/switch_controller to become available...
   Successfully switched controllers

Read-only broadcasters
----------------------

These broadcasters are read-only. They read states from the robot and publish them on a ROS topic.
As they are read-only, they don't claim any resources and can be combined freely. By default, they
are all started and running. Those controllers do not require the robot to have the
external_control script running.

joint_state_broadcaster
^^^^^^^^^^^^^^^^^^^^^^^

Type: `joint_state_broadcaster/JointStateBroadcaster <https://control.ros.org/rolling/doc/ros2_controllers/joint_state_broadcaster/doc/userdoc.html>`_

Publishes all joints' positions, velocities, and motor currents as ``sensor_msgs/JointState`` on the ``joint_states`` topic.

.. note::

   The effort field contains the currents reported by the joints and not the actual efforts in a
   physical sense.

speed_scaling_state_broadcaster
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Type: :ref:`ur_controllers/SpeedScalingStateBroadcaster <speed_scaling_state_broadcaster>`

This broadcaster publishes the current actual execution speed as reported by the robot. Values are
floating points between 0 and 1.

force_torque_sensor_broadcaster
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Type: `force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster <https://control.ros.org/rolling/doc/ros2_controllers/force_torque_sensor_broadcaster/doc/userdoc.html>`_

Publishes the robot's wrench as reported from the controller.

Commanding controllers
----------------------

The commanding controllers control the robot's motions. Those controllers can't be combined
arbitrarily, as they will claim hardware resources. Only one controller can claim one hardware
resource at a time.

scaled_joint_trajectory_controller (Default motion controller)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Type: :ref:`ur_controllers/ScaledJointTrajectoryController <scaled_jtc>`

Scaled version of the
`joint_trajectory_controller
<https://control.ros.org/master/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html>`_.
It uses the robot's speed scaling information and thereby the safety compliance features, like pause on safeguard stop. In addition, it also makes it possible to adjust execution speed using the speed slider on the teach pendant or set the program in pause and restart it again.
See it's linked documentation for details.

io_and_status_controller
^^^^^^^^^^^^^^^^^^^^^^^^

Type: :ref:`ur_controllers/GPIOController <io_and_status_controller>`

Allows setting I/O ports, controlling some UR-specific functionality and publishes status information about the robot.

forward_velocity_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Type: `velocity_controllers/JointGroupVelocityController <https://control.ros.org/rolling/doc/ros2_controllers/position_controllers/doc/userdoc.html#position-controllers-jointgrouppositioncontroller>`_

Allows setting target joint positions directly. The robot tries to reach the target position as
fast as possible. The user is therefore responsible for sending commands that are achievable. This
controller is particularly useful when doing servoing such as ``moveit_servo``.

forward_position_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Type: `position_controllers/JointGroupPositionController <https://control.ros.org/rolling/doc/ros2_controllers/velocity_controllers/doc/userdoc.html#velocity-controllers-jointgroupvelocitycontroller>`_

Allows setting target joint velocities directly. The user is responsible for sending commands that
are achievable. This controller is particularly useful when doing servoing such as
``moveit_servo``.
