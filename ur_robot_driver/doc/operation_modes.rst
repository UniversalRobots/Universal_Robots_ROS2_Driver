.. _operation_modes:

Operation modes
==================

There are two different sets of **modes** that are used throughout this documentation:

- The ROS2 driver's mode of operation: *Teach Pendant mode* and *Headless mode*.
- The Robot's control mode: *local control mode* and *remote control mode*. (Not applicable to the CB3 series)

Both will be explained below.

Robot's control modes
---------------------
E-series robots can operate in different command modes: It can be either in *local control mode* where the teach pendant is the single point of control, or in *remote control mode* where the teach pendant is locked and cannot be used to start any motion, including freedrive. Note that the ability to change to *remote control mode* has to be explicitly enabled in the robot's settings under Settings -> System -> Remote Control. See the [robot manual](https://myur.universal-robots.com/manuals) for details.

The remote control mode is needed for many aspects of this driver such as

- :ref:`headless_mode`
- :ref:`Sending script code to the robot<script_command_interface>`
- Many :ref:`dashboard<dashboard_client>` functionalities such as

  - restarting the robot after protective / EM-Stop
  - powering on the robot and do brake release
  - loading and starting programs


Driver's operation modes
------------------------

There are two fundamentally different control modes of the UR ROS driver to control a UR robot or the URSim robot simulator:
- *Teach Pendant mode* using the :ref:`External Control URCap<install-urcap-e-series>`
- *Headless mode* that works without interacting with Teach pendant, when *remote control mode* has been selected on the teach pendant.

.. _teach_pendant_mode:

Teach Pendant Mode
^^^^^^^^^^^^^^^^^^^^^

In Teach Pendant mode you will need the :ref:`External Control URCap<install-urcap-e-series>`
installed on the robot. Please follow the installation guidelines for the :ref:`CB3<install-urcap-cb3>` or :ref:`E-series<install-urcap-e-series>`. Remember to set the correct IP address for the ROS driver host computer, and ensure the configured port number matches the ``script_sender_port`` defined in the ROS driver's launch arguments, default is 50002.

With that, create a program containing the *External Control* program node and press "play" on the teach pendant to start the program.
Hereafter the URCap will request script code from the ROS driver and execute it, once the program enters that node. In addition, the ROS driver can return the control to the Teach pendent program. This gives the possibility to combine the teach pendant program and the use of the ROS driver.

Please note that a running program will stop as soon as another program is sent to the robot, either by sending it directly through this driver or by pressing any motion-related button on the teach pendant. Therefore, the ROS drivers will not be able to send commands to the robot again afterward until the ROS driver's program has been restarted. This can be done using the "play" button on the teach pendant.

If this is necessary, you will see the output ``Connection to reverse interface dropped.`` from the driver.

.. note::
   It is also possible to use the ROS driver's *Teach Pendant mode* with the robot's *remote control
   mode* together. In this case you will need to load and start the program containing the *External
   Control URCap* program node through the :ref:`dashboard_client`'s ``load_program`` and ``play``
   services.

.. _headless_mode:

Headless mode
^^^^^^^^^^^^^

When headless mode is activated while launching the ROS driver, the URScript code will be sent directly to the robot controller and started directly.

Please note, that a running program will stop as soon as another program is sent to the robot, either by sending it directly through this driver, or by pressing any motion-related button on the teach pendant. Therefore, the ROS driver will not be able to send commands to the robot again afterward until the ROS driver's program has been restarted.
The robot program can be restarted using the ``/io_and_status_controller/resend_robot_program`` service.
If this is necessary, you will see the output ``Connection to reverse interface dropped.`` from the driver.

.. note::
   On e-Series robots and newer, the robot must be in *remote control mode* as explained above in order to use the
   ROS driver's *Headless mode*.
