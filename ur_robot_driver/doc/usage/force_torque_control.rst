:github_url: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/main/ur_robot_driver/doc/usage/force_torque_control.rst

.. _force_torque_control:

Force and Torque Control
========================

This page provides an overview of the force and torque control capabilities available in this
driver. In ROS 2, joint torque control is also referred to as **effort control**. The driver
supports both direct joint-level torque control and the robot's built-in Cartesian force-mode control.

For position and velocity based control, see :ref:`position_velocity_control`.

For utility controllers such as freedrive, tool contact detection, and I/O control, see
:ref:`utility_controllers`.

Joint Torque Control
--------------------

Direct joint-level torque control lets you command torque values to each of the robot's joints.
This is useful for implementing custom compliance, force control strategies, or research
applications that require direct torque-level access.

Custom controllers should best be implemented as ros2_control controllers by using the driver's
exposed effort interfaces. The ``forward_effort_controller`` explained below is more of an
easy-to-use example of how to send direct torque commands to the robot, rather than a building
block for an application.

forward_effort_controller
^^^^^^^^^^^^^^^^^^^^^^^^^

Type: `forward_command_controller/ForwardCommandController <https://control.ros.org/rolling/doc/ros2_controllers/forward_command_controller/doc/userdoc.html>`_ with ``interface_name: effort``

This controller streams target joint efforts (torques) directly to the robot using the URScript function ``direct_torque(...)``. The robot automatically
compensates for gravity, so the provided target torques should not include gravity compensation. The user is
responsible for sending commands that are safe and achievable.

.. note::

   This controller requires PolyScope >= 5.25.1 (PolyScope 5) or >= 10.12.1 (PolyScope X).

To activate this controller, deactivate any active motion controller first:

.. code-block:: console

   $ ros2 control switch_controllers --deactivate joint_trajectory_controller \
     --activate forward_effort_controller

The controller accepts commands on its ``~/commands`` topic as ``std_msgs/msg/Float64MultiArray``
with one value per joint.

.. warning::

   This controller is mutually exclusive with position and velocity controllers. Only one
   motion controller can be active at a time.

.. note::

   The ``effort`` field in ``sensor_msgs/JointState`` (published by the ``joint_state_broadcaster``)
   contains motor currents, not physical joint torques.

Friction Compensation
^^^^^^^^^^^^^^^^^^^^^

When using direct torque control, the robot's internal friction compensation can be tuned using the
:ref:`friction_model_controller <friction_model_controller>`.

This controller sets per-joint viscous and Coulomb friction scale factors that determine how much
internal friction compensation the robot applies during torque control mode. Each scale factor is
in the range [0, 1], where 0 means no compensation and 1 means full compensation.

The controller exposes the ``~/set_friction_model_parameters`` service
(`ur_msgs/srv/SetFrictionModelParameters
<https://docs.ros.org/en/rolling/p/ur_msgs/srv/SetFrictionModelParameters.html>`_) to configure
these values. It is started as active by default in the driver's launch file.

.. note::

   The friction model controller is only relevant for effort control. While it is technically
   compatible with all motion controllers, adjusting friction parameters only has an effect during
   direct torque control.

Force Mode Controller
---------------------

The :ref:`force_mode_controller <force_mode_controller>` provides Cartesian-space force and torque
control using the robot's built-in force mode. This interfaces the URScript function
``force_mode(...)``.

Unlike direct joint torque control, force mode operates in Cartesian space: you specify forces and
torques relative to a task frame, along with a selection vector that determines which axes are
force-controlled (compliant) and which remain position-controlled.

- In **compliant axes**, the robot adjusts its position to achieve the specified force or torque.
- In **non-compliant axes**, motion commands from the active position controller are executed
  normally.

Force mode can be combined with all position control modes
(``joint_trajectory_controller``, ``passthrough_trajectory_controller``,
``forward_position_controller``, etc.) to execute motions under force constraints.

The controller provides two services:

* ``~/start_force_mode [ur_msgs/srv/SetForceMode]``
* ``~/stop_force_mode [std_srvs/srv/Trigger]``

.. tip::

   The robot's force/torque sensor is a relative sensor, meaning it measures changes in force and
   torque rather than absolute values. For best performance, zero the sensor using the
   ``~/zero_ftsensor`` service on the :ref:`io_and_status_controller <io_and_status_controller>`
   before activating force mode and before the tool makes contact with the environment.

See the :ref:`force_mode_controller <force_mode_controller>` documentation for full details on
parameters, the service interface, and the meaning of each field.

An example demonstrating force mode usage is available at ``ur_robot_driver/examples/force_mode.py``.

Freedrive Mode Controller
-------------------------

Type: :ref:`ur_controllers/FreedriveModeController <freedrive_mode_controller>`

Allows manually moving the robot's joints without any active control. This controller cannot be
combined with any other motion controller.

Freedrive mode is activated by publishing ``True`` messages on the
``~/enable_freedrive_mode`` topic (``std_msgs/msg/Bool``). Messages must be sent continuously to
keep freedrive active -- if no message is received within the ``inactive_timeout`` (default: 1
second), freedrive mode is automatically deactivated.

.. code-block:: console

   $ ros2 topic pub --rate 2 /freedrive_mode_controller/enable_freedrive_mode \
     std_msgs/msg/Bool "{data: true}"

To deactivate, either publish ``False``, deactivate the controller, or simply stop publishing.

See the :ref:`freedrive_mode_controller <freedrive_mode_controller>` documentation for full
details.

Force/Torque Sensing
--------------------

The robot's TCP force/torque sensor is an independent sensing capability. It is **not required**
for joint torque control to function. However, it can be useful alongside torque
control for monitoring contact forces or implementing sensor-based control strategies.

force_torque_sensor_broadcaster
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Type: `force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster <https://control.ros.org/rolling/doc/ros2_controllers/force_torque_sensor_broadcaster/doc/userdoc.html>`_

Publishes the TCP wrench (forces and torques) as reported by the robot's built-in sensor on the
``ft_data`` topic. This broadcaster is read-only and runs alongside any other controller.

The sensor can be zeroed using the ``~/zero_ftsensor`` service on the
:ref:`io_and_status_controller <io_and_status_controller>`.


Controller Compatibility
------------------------

The following table summarizes how torque-related controllers can be combined:

.. list-table::
   :header-rows: 1

   * - Controller
     - Can combine with
   * - ``forward_effort_controller``
     - Mutually exclusive with all other joint-level motion controllers (position, velocity,
       trajectory).
   * - ``force_mode_controller``
     - All position control modes (``joint_trajectory_controller``,
       ``passthrough_trajectory_controller``, ``forward_position_controller``, etc.)
   * - ``friction_model_controller``
     - All controllers (only affects torque control behavior).
   * - ``force_torque_sensor_broadcaster``
     - All controllers (read-only broadcaster).

Examples
--------

force_mode.py
^^^^^^^^^^^^^

The ``ur_robot_driver/examples/force_mode.py`` example demonstrates how to:

1. Switch to the ``passthrough_trajectory_controller`` and ``force_mode_controller``
2. Move the robot to a starting pose
3. Activate force mode with a specified wrench, task frame, and selection vector
4. Deactivate force mode after a set duration
