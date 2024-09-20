.. _operation_modes:

Modes of operation
==================

There are two different sets of **modes** that are used throughout this documentation:

- The driver's mode of operation (External Control vs Headless)
- (e-series only) The robot's control mode (Local Control Mode vs Remote Control Mode)

Both will be explained below.

Robot's control modes
---------------------
On the e-series the robot itself can operate in different command modes: It can be either in local control mode where the teach pendant is the single point of command or in remote control mode, where motions from the TP, starting & loading programs from the TP activating the freedrive mode are blocked. Note that the remote control mode has to be explicitly enabled in the robot's settings under Settings -> System -> Remote Control. See the robot's manual for details.

The remote control mode is needed for many aspects of this driver such as

- :ref:`headless_mode`
- :ref:`Sending script code to the robot<script_command_interface>`
- Many :ref:`dashboard<dashboard_client>` functionalities such as

  - restarting the robot after protective / EM-Stop
  - powering on the robot and do brake release
  - loading and starting programs


Driver's operation modes
------------------------

There are two fundamentally different modes of operation when using this driver with a UR robot /
URSim: External Control Mode and Headless Mode. Depending on your requirements one can be more
suitable than the other.

.. _external_control_mode:

External Control Mode
^^^^^^^^^^^^^^^^^^^^^

In External Control mode you will need the :ref:`External Control URCap<install-urcap-e-series>`
installed on the robot. With that, create a program containing the *External Control* program node.
Once the program enters that node, it will request script code from an external source (in this
case the ROS driver) and execute that.

As soon as other script code is sent to the robot either by sending it directly through this driver
or by pressing any motion-related button on the teach pendant, the script will be overwritten by
this action and has to be restarted by using the "play" button on the teach pendant.

If this is necessary, you will see the output ``Connection to reverse interface dropped.`` from the driver.

.. note::
   It is also possible to use the driver's external control mode with the robot's local control
   mode together. In this case you will need to load and start the program containing the *External
   Control* program node through the :ref:`dashboard_client`'s ``load_program`` and ``play``
   services.

.. _headless_mode:

Headless mode
^^^^^^^^^^^^^

Inside this driver, there's the headless mode, which can be either enabled or not. When the headless mode is activated, required script code for external control will be sent to the robot directly when the driver starts.

As soon as other script code is sent to the robot either by sending it directly through this driver
or by pressing any motion-related button on the teach pendant, the script will be overwritten by
this action and has to be restarted by using the ``/io_and_status_controller/resend_robot_program``
service. If this is necessary, you will see the output ``Connection to reverse interface dropped.``
from the driver.

.. note::
   On e-Series robots the robot must be in remote_control_mode as explained above in order to use the
   driver's Headless Mode.
