:github_url: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/main/ur_robot_driver/doc/position_velocity_control.rst

.. _position_velocity_control:

Position and Velocity Control
=============================

Position and velocity control are the most common ways to control a UR robot via ROS 2. The driver
provides several controllers for different use cases, from trajectory-based motion planning to
direct joint-level streaming for real-time servoing.

For force and torque based control, see :ref:`force_torque_control`.

For utility controllers such as freedrive, tool contact detection, and I/O control, see
:ref:`utility_controllers`.

Trajectory-based Control
------------------------

Trajectory-based control is the standard approach for executing planned motions. A trajectory
planner (such as MoveIt) generates a sequence of waypoints, and a trajectory controller
interpolates and executes them on the robot.

joint_trajectory_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Type: `joint_trajectory_controller/JointTrajectoryController <https://control.ros.org/rolling/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html>`_

The ``joint_trajectory_controller`` is the default motion controller in the driver. It receives
trajectories via the ``FollowJointTrajectory`` action interface and interpolates them on the ROS
control PC.

**Key features:**

* **Speed scaling** support: The controller respects the robot's speed scaling, including the speed
  slider on the teach pendant, emergency stops, safeguard stops, and program pauses. Trajectory
  execution is transparently scaled so that path accuracy is maintained even when execution speed
  changes.
* Used with **MoveIt** and other trajectory planners.
* Can be combined with the :ref:`force_mode_controller <force_mode_controller>` for motion under
  force constraints.
* Can be combined with the :ref:`tool_contact_controller <tool_contact_controller>` to stop motion
  on tool contact.

passthrough_trajectory_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Type: :ref:`ur_controllers/PassthroughTrajectoryController <passthrough_trajectory_controller>`

Instead of interpolating trajectories on the ROS PC, this controller forwards the complete
trajectory to the robot controller for interpolation and execution. This significantly reduces the
realtime requirements on the ROS PC, making it possible to run a stable driver connection without a
real-time patched kernel.

**Key features:**

* **Reduced realtime requirements** on the ROS PC.
* **Speed scaling** support: Handles speed scaling, emergency stops, safeguard stops, and program
  pauses.
* Can be used with **MoveIt** and other trajectory planners. Trajectories planned with MoveIt will be
  executed following the trajectory exactly.
* Uses the same ``FollowJointTrajectory`` action interface as the ``joint_trajectory_controller``.
* Can be combined with the :ref:`force_mode_controller <force_mode_controller>` for motion under
  force constraints.
* Can be combined with the :ref:`tool_contact_controller <tool_contact_controller>` to stop motion
  on tool contact.

See the :ref:`passthrough_trajectory_controller <passthrough_trajectory_controller>` documentation
for full details on parameters, tolerances, and implementation.

Motion Primitives
-----------------

The ``motion_primitive_forward_controller`` provides a high-level motion interface that forwards
motion primitives directly to the robot controller for execution. Instead of streaming individual
joint commands or interpolating trajectories on the ROS PC, this controller sends complete motion
descriptions (such as linear joint moves or linear Cartesian moves) to the robot, which executes
them using its built-in motion planner.

motion_primitive_forward_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Type: ``motion_primitives_controllers/MotionPrimitivesForwardController``

This controller accepts sequences of motion primitives via the
``~/motion_sequence [control_msgs/action/ExecuteMotionPrimitiveSequence]`` action interface. Each
motion primitive in the sequence specifies a motion type, target pose or joint positions, velocity,
acceleration, blend radius, and move time.

Currently supported motion types:

* ``LINEAR_JOINT``: Joint-space point-to-point move to target joint positions (moveJ).
* ``LINEAR_CARTESIAN``: Cartesian-space linear move to a target TCP pose (moveL).
* ``CIRCULAR_CARTESIAN``: Circular move in Cartesian space through a via-point to a target pose
  (moveC).

The underlying `Universal Robots Client Library
<https://github.com/UniversalRobots/Universal_Robots_Client_Library>`_ supports additional motion
types such as moveP (process move with constant TCP velocity), optiMoveJ, optiMoveL, and spline
motions. These are not yet exposed through the ``control_msgs`` motion primitive interface.

Key features:

* Motion primitives are executed natively by the robot controller, so the ROS PC has no realtime
  requirements during execution.
* Sequences of motion primitives can be blended together using blend radii for smooth, continuous
  motion.
* Can be combined with the :ref:`force_mode_controller <force_mode_controller>` for motion under
  force constraints.
* Can be combined with the :ref:`tool_contact_controller <tool_contact_controller>` to stop motion
  on tool contact.
* Mutually exclusive with all other motion controllers (``joint_trajectory_controller``,
  ``passthrough_trajectory_controller``, ``forward_position_controller``,
  ``forward_velocity_controller``, ``forward_effort_controller``, ``freedrive_mode_controller``).

To activate:

.. code-block:: console

   $ ros2 control switch_controllers --deactivate scaled_joint_trajectory_controller \
     --activate motion_primitive_forward_controller

An example demonstrating motion primitive usage is available at
``ur_robot_driver/examples/send_dummy_motion_primitives_ur10e.py``.

Streaming Control
-----------------

Streaming controllers allow direct, real-time control of joint positions or velocities. These are
useful for servoing applications (such as ``moveit_servo``), teleoperation, or any scenario
requiring continuous, low-latency joint commands.

.. warning::

   With streaming controllers, the user is responsible for sending commands that are safe and
   achievable. The robot will try to follow commands as fast as possible without trajectory
   planning.

.. note::

   The robot will scale down the execution speed if its safety limits require it. Unlike the
   trajectory-based controllers, streaming controllers do not automatically account for this
   scaling. Monitor the :ref:`speed_scaling_state_broadcaster <speed_scaling_state_broadcaster>` to
   detect when the robot is scaling down and adapt your commands accordingly.

forward_position_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Type: `forward_command_controller/ForwardCommandController <https://control.ros.org/rolling/doc/ros2_controllers/forward_command_controller/doc/userdoc.html>`_ with ``interface_name: position``

Streams target joint positions directly to the robot. The robot moves to each target position as
fast as possible. This interfaces the URScript function ``servoj(...)``.

To activate:

.. code-block:: console

   $ ros2 control switch_controllers --deactivate scaled_joint_trajectory_controller \
     --activate forward_position_controller

forward_velocity_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Type: `forward_command_controller/ForwardCommandController <https://control.ros.org/rolling/doc/ros2_controllers/forward_command_controller/doc/userdoc.html>`_ with ``interface_name: velocity``

Streams target joint velocities directly to the robot. This interfaces the URScript function ``speedj(...)``.

To activate:

.. code-block:: console

   $ ros2 control switch_controllers --deactivate scaled_joint_trajectory_controller \
     --activate forward_velocity_controller

Force Mode Controller
---------------------

The :ref:`force_mode_controller <force_mode_controller>` can be combined with any of the position
control modes above to add Cartesian force/torque constraints during motion. See the
:ref:`force_torque_control` page for full details.

Controller Compatibility
------------------------

The following table summarizes how position and velocity controllers can be combined:

.. list-table::
   :header-rows: 1

   * - Controller
     - Can combine with
   * - ``joint_trajectory_controller``
     - Mutually exclusive with other motion controllers. Can combine with
       ``force_mode_controller`` and ``tool_contact_controller``.
   * - ``passthrough_trajectory_controller``
     - Mutually exclusive with other motion controllers. Can combine with
       ``force_mode_controller`` and ``tool_contact_controller``.
   * - ``forward_position_controller``
     - Mutually exclusive with other motion controllers. Can combine with
       ``force_mode_controller``.
   * - ``forward_velocity_controller``
     - Mutually exclusive with other motion controllers. Can combine with
       ``force_mode_controller``.
   * - ``motion_primitive_forward_controller``
     - Mutually exclusive with other motion controllers. Can combine with
       ``force_mode_controller`` and ``tool_contact_controller``.
