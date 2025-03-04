.. _robot_state_helper:

Robot state helper
==================
After switching on the robot, it has to be manually started, the brakes have to be released and a
program has to be started in order to make the robot ready to use. This is usually done using the
robot's teach pendant.

Whenever the robot encounters an error, manual intervention is required to resolve the issue. For
example, if the robot goes into a protective stop, the error has to be acknowledged and the robot
program has to be unpaused.

When the robot is in :ref:`remote_control_mode <operation_modes>`, most interaction with the robot can be done
without using the teach pendant, many of that through the :ref:`dashboard client
<dashboard_client_ros2>`.

The ROS driver provides a helper node that can be used to automate some of these tasks. The
``robot_state_helper`` node can be used to start the robot, release the brakes, and (re-)start the
program through an action call. It is started by default and provides a
`dashboard_msgs/action/SetMode
<https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/main/ur_dashboard_msgs/action/SetMode.action>`_ action.

For example, to make the robot ready to be used by the ROS driver, call

.. code-block:: console

   $ ros2 action send_goal /ur_robot_state_helper/set_mode ur_dashboard_msgs/action/SetMode "{ target_robot_mode: 7, stop_program: true, play_program: true}"

The ``target_robot_mode`` can be one of the following:

.. table:: target_robot_mode
   :widths: auto

   =====  =====
   index  meaning
   =====  =====
   3      POWER_OFF -- Robot is powered off
   5      IDLE -- Robot is powered on, but brakes are engaged
   7      RUNNING -- Robot is powered on, brakes are released, ready to run a program
   =====  =====

.. note::

   When the ROBOT_STATE is in ``RUNNING``, that is equivalent to the robot showing the green dot in
   the lower left corner of the teach pendant (On PolyScope 5). The program state is independent of
   that and shows with the text next to that button.

The ``stop_program`` flag is used to stop the currently running program before changing the robot
state. In combination with the :ref:`controller_stopper`, this will deactivate any motion
controller and therefore stop any ROS action being active on those controllers.

.. warning::
   A robot's protective stop or emergency stop is only pausing the running program. If the program
   is resumed after the P-Stop or EM-Stop is released, the robot will continue executing what it
   has been doing. Therefore, it is advised to stop and re-start the program when recovering from a
   fault.

The ``play_program`` flag is used to start the program after the robot state has been set. This has
the same effects as explained in :ref:`continuation_after_interruptions`.
