.. _ur_robot_driver_startup:

Startup the driver
==================

Prepare the robot
-----------------

If you want to use a real robot, or a URSim simulator, with this driver, you need to prepare it,
first. Make sure that you complete all steps from the :ref:`setup instructions<robot_setup>`,
installed the External Control URCap and created a program as explained
:ref:`here<install-urcap-e-series>`.

Launch files
------------

For starting the driver it is recommended to start the ``ur_control.launch.py`` launchfile from the
``ur_robot_driver`` package. It starts the driver, a set of controllers and a couple of helper
nodes for UR robots. The only required arguments are the ``ur_type`` and ``robot_ip`` parameters.

.. code-block:: console

   $ ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.56.101

Allowed ``ur_type`` strings: ``ur3``, ``ur3e``, ``ur5``, ``ur5e``, ``ur10``, ``ur10e``, ``ur16e``,
``ur20``, ``ur30``.

Other important arguments are:


* ``use_mock_hardware`` (default: *false* ) - Use simple hardware emulator from ros2_control. Useful for testing launch files, descriptions, etc.
* ``headless_mode`` (default: *false*) - Start driver in :ref:`headless_mode`.
* ``launch_rviz`` (default: *true*) - Start RViz together with the driver.
* ``initial_joint_controller`` (default: *scaled_joint_trajectory_controller*) - Use this if you
  want to start the robot with another controller.

  .. note::
     When the driver is started, you can list all loaded controllers using the ``ros2 control
     list_controllers`` command. For this, the package ``ros2controlcli`` must be installed (``sudo
     apt-get install ros-${ROS_DISTRO}-ros2controlcli``).


For all other arguments, please see


.. code-block:: console

   $ ros2 launch ur_robot_driver ur_control.launch.py --show-args

Also, there are predefined launch files for all supported types of UR robots.

.. _robot_startup_program:

Finish startup on the robot
---------------------------

Unless :ref:`headless_mode` is used, you will now have to start the *External Control URCap* program on
the robot that you have created earlier.

Depending on the :ref:`robot control mode<operation_modes>` do the following:

* In *local control mode*, load the program on the robot and press the "Play" button |play_button| on the teach pendant.
* In *remote control mode* load and start the program using the following dashboard calls:

  .. code-block:: console

     $ ros2 service call /dashboard_client/load_program ur_dashboard_msgs/srv/Load "filename: my_robot_program.urp"``
     $ ros2 service call /dashboard_client/play std_srvs/srv/Trigger {}

* When the driver is started with ``headless_mode:=true`` nothing is needed. The driver is running
  already.

.. _continuation_after_interruptions:

Continuation after interruptions
--------------------------------

Whenever the *External Control URCap* program gets interrupted, it has to be unpaused / restarted.

If that happens, you will see the output ``Connection to reverse interface dropped.``

This can happen, e,g, when

* The running program is actively stopped.
* The robot goes into a protective stop / EM stop. (The program will be paused, then)
* The communication is stopped, since the external source did not receive a command in time.
* There was another script sent for execution e.g.

  * Script code was sent to the robot via its primary interface
  * Robot motion is performed using the Teach pendant

Depending on the operation mode, perform one of the following steps:

* In *local control mode*, simply press the "Play" button |play_button| on the teach pendant.
* In *remote control mode* start the program using the following dashboard call:

  .. code-block:: console

     $ ros2 service call /dashboard_client/play std_srvs/srv/Trigger {}

* When the driver is started with ``headless_mode:=true`` perform the following service call:

  .. code-block:: console

     $ ros2 service call /io_and_status_controller/resend_robot_program std_srvs/srv/Trigger {}





.. |play_button| image:: ../resources/play_button.svg
                 :height: 20px
                 :width: 20px
