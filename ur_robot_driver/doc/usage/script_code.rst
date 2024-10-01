Sending URScript code to the robot
==================================

.. _script_command_interface:

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
