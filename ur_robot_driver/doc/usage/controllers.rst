:github_url: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/main/ur_robot_driver/doc/usage/controllers.rst

.. _usage_controllers:

Controllers
===========

This help page describes the different controllers available for the ``ur_robot_driver``. This
should help users finding the right controller for their specific use case.

Available Controllers
---------------------
.. toctree::
   :maxdepth: 2

   position_velocity_control
   force_torque_control
   utility_controllers

How do controllers get loaded and started?
------------------------------------------

As this driver uses `ros2_control <https://control.ros.org>`_ all controllers are managed by the
controller_manager. During startup, a default set of running controllers is loaded and activated,
another set is loaded in inactive mode. Inactive controllers won't be usable right away, but they
need to be activated individually.

Controllers that are actually writing to some command interfaces (e.g. joint positions) will claim
those interfaces. Only one controller claiming a certain interface can be active at one point.

Controllers can be switched either through the controller_manager's service calls, through the
`rqt_controller_manager
<https://control.ros.org/rolling/doc/ros2_control/controller_manager/doc/userdoc.html#rqt-controller-manager>`_
gui or through the ``ros2 control`` verb from the ``ros-${ROS_DISTRO}-ros2controlcli`` package.

For example, to switch from the default ``scaled_joint_trajectory_controller`` to the
``forward_position_controller`` you can call

.. code-block:: console

   $ ros2 control switch_controllers --deactivate scaled_joint_trajectory_controller \
     --activate forward_position_controller
   [INFO 2024-09-23 20:32:04.373] [_ros2cli_1207798]: waiting for service /controller_manager/switch_controller to become available...
   Successfully switched controllers

Where are controllers defined?
------------------------------

Controllers are defined in the ``config/ur_controllers.yaml`` file.
