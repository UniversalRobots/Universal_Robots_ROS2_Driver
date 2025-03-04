.. _controller_stopper:

Controller stopper
==================

As explained in the section :ref:`robot_startup_program`, the robot needs to run a program in order
to receive motion commands from the ROS driver. When the program is not running, commands sent to
the robot will have no effect.

To make that transparent, the ``controller_stopper`` node mirrors that state in the ROS
controller's state. It listens to the ``/io_and_status_controller/robot_program_running`` topic and
deactivates all motion controllers  (or any controller not explicitly marked as "consistent", see
below)when the program is not running.

Once the program is running again, any previously active motion controller will be activated again.

This way, when sending commands to an inactive controller the caller should be transparently
informed, that the controller cannot accept commands at the moment.

In the same way, any running action on the ROS controller will be aborted, as the controller gets
deactivated by the controller_stopper.

Parameters
----------

- ``~consistent_controllers`` (list of strings, default: ``[]``)

  A list of controller names that should not be stopped when the program is not running. Any
  controller that doesn't require the robot program to be running should be in that list.
