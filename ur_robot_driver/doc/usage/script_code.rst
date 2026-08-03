:github_url: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/main/ur_robot_driver/doc/usage/script_code.rst

Sending URScript code to the robot
==================================

.. _script_command_interface_ros2:

Custom URScript commands
------------------------

The driver's package contains a ``urscript_interface`` node that allows sending URScript snippets
directly to the robot when the robot is in remote control mode.

It gets started in the driver's launchfiles by default. To use it, simply
publish a message to its interface:

.. code-block:: bash

  # simple popup
  ros2 topic pub /urscript_interface/script_command std_msgs/msg/String '{data: popup("hello")}' --once

Be aware, that running a program on this interface (meaning publishing script code to that interface) stops any running program on the robot.
Thus, the motion-interpreting program that is started by the driver gets stopped and has to be
restarted again. Depending whether you use :ref:`headless_mode` or not, you'll have to call the
``resend_program`` service or press the ``play`` button on the teach panel to start the
external control URCap program again.

.. note::
  On E-series robots or newer the robot needs to be in *remote control mode* in order to execute custom URScript commands.
  Currently, there is no feedback on the code's correctness. If the code sent to the
  robot is incorrect, it will silently not get executed. Make sure that you send valid URScript code!

Multi-line programs
^^^^^^^^^^^^^^^^^^^

When you want to define multi-line programs, make sure to check that newlines are correctly
interpreted from your message. For this purpose the driver prints the program as it is being sent to
the robot. When sending a multi-line program from the command line, you can use an empty line
between each statement:

.. code-block:: bash

   ros2 topic pub --once /urscript_interface/script_command std_msgs/msg/String '{data:
   "def my_prog():

     set_digital_out(1, True)

     movej(p[0.2, 0.3, 0.8, 0, 0, 3.14], a=1.2, v=0.25, r=0)

     textmsg(\"motion finished\")

   end"}'

Non-interrupting programs
^^^^^^^^^^^^^^^^^^^^^^^^^

To prevent interrupting the main program, you can send certain commands as `secondary programs
<https://www.universal-robots.com/articles/ur/programming/secondary-program/>`_.

.. code-block:: bash

   ros2 topic pub --once /urscript_interface/script_command std_msgs/msg/String '{data:
     "sec my_prog():

       textmsg(\"This is a log message\")

     end"}'

Scripts with execution monitoring
---------------------------------
The action server at ``/urscript_interface/execute_script`` allows executing script programs and getting the information when execution is done and whether there was an error during execution.

The ``SendScript`` action definition can be seen in the `ur_msgs/action/SendScript <https://docs.ros.org/en/rolling/p/ur_msgs/action/SendScript.html>`_

This action server is a ROS wrapper around the `URCL primary client's <https://github.com/UniversalRobots/Universal_Robots_Client_Library/blob/master/doc/architecture/primary_client.rst/>`_
SendScriptBlocking method, and the meaning of parameters can be seen there.
The action will be reported as successful if no errors occur during script execution. See :ref:`script-cancel` for additional info about when the action might be reported as successful.
If an error is detected, a message explaining what went wrong will be made available in the action result.
While the action server is waiting for a script to finish execution, it will reject any further action goals, and the ``/urscript_interface/script_command`` topic will also be ignored during this time.

.. note::
  The previous note about restarting the external control URCap also applies when using the action server.

.. _script-cancel:

Script cancellation
^^^^^^^^^^^^^^^^^^^
If script execution is canceled through the ROS2 action interface, the running script will be stopped, and the action will be reported as canceled. If script execution is stopped via the teach pendant or **any** other non-ROS means, the corresponding ROS action will be reported as successful, as the action server can not distinguish between successful programs and programs that were stopped outside of ROS.

Command line example
^^^^^^^^^^^^^^^^^^^^
The action server can be called from the command line like this:

.. code-block:: bash

  ros2 action send_goal /urscript_interface/execute_script ur_msgs/action/SendScript '{
    program: "popup(\"cmd line example\")",
    script_name: "cmd_line_example",
    start_timeout: {sec: 1.0, nanosec: 0},
    fail_on_warnings: true}'

Parameters
^^^^^^^^^^
- ``robot_ip`` (string, **required**)

  The IP address at which the robot is reachable.

- ``retry_on_readonly_interface`` (boolean, default: ``true``)

  When the client is currently connected to a read-only interface (because the robot is not in
  remote_control mode / was put to local control mode since the client connected) script code will
  not be accepted by the primary interface.

  When set to ``true`` the client will attempt to reconnect to the primary interface and send the
  script again once. This mechanism is only active when using the action interface.
