Startup the driver
==================

Prepare the robot
-----------------

If you want to use a real robot with this driver, you need to prepare it, first. Make sure that you
complete all steps from the :ref:`setup instructions<robot_setup>`, installed the External
Control URCap and created a program as explained :ref:`here<install-urcap-e-series>`.

Launch files
------------

For starting the driver there are two main launch files in the ``ur_robot_driver`` package.


* ``ur_control.launch.py`` - starts ros2_control node including hardware interface, joint state broadcaster and a controller. This launch file also starts ``dashboard_client`` if real robot is used.
* ``ur_dashboard_client.launch.py`` - start the dashboard client for UR robots.

Also, there are predefined launch files for all supported types of UR robots.

The arguments for launch files can be listed using ``ros2 launch ur_robot_driver <launch_file_name>.launch.py --show-args``.
The most relevant arguments are the following:


* ``ur_type`` (\ *mandatory* ) - a type of used UR robot (\ *ur3*\ , *ur3e*\ , *ur5*\ , *ur5e*\ , *ur10*\ , *ur10e*\ , or *ur16e*\ , *ur20*\ , *ur30*\ ).
* ``robot_ip`` (\ *mandatory* ) - IP address by which the root can be reached.
* ``use_mock_hardware`` (default: *false* ) - use simple hardware emulator from ros2_control.
  Useful for testing launch files, descriptions, etc. See explanation below.
* ``initial_positions`` (default: dictionary with all joint values set to 0) - Allows passing a dictionary to set the initial joint values for the mock hardware from `ros2_control <http://control.ros.org/>`_.  It can also be set from a yaml file with the ``load_yaml`` commands as follows:

  .. code-block::

     <xacro:property name="initial_positions" value="${load_yaml(initial_positions_file)}" / >

  In this example, the **initial_positions_file** is a xacro argument that contains the absolute path to a yaml file. An example of the initial positions yaml file is as follows:

  .. code-block::

     elbow_joint: 1.158
     shoulder_lift_joint: -0.953
     shoulder_pan_joint: 1.906
     wrist_1_joint: -1.912
     wrist_2_joint: -1.765
     wrist_3_joint: 0.0

* ``mock_sensor_commands`` (default: *false* ) - enables setting sensor values for the hardware emulators.
  Useful for offline testing of controllers.

* ``robot_controller`` (default: *joint_trajectory_controller* ) - controller for robot joints to be started.
  Available controllers:


  * joint_trajectory_controller
  * scaled_joint_trajectory_controller

  Note: *joint_state_broadcaster*\ , *speed_scaling_state_broadcaster*\ , *force_torque_sensor_broadcaster*\ , and *io_and_status_controller* will always start.

  *HINT* : list all loaded controllers using ``ros2 control list_controllers`` command. For this,
  the package ``ros2controlcli`` must be installed (``sudo apt-get install ros-${ROS_DISTRO}-ros2controlcli``).

**NOTE**\ : The package can simulate hardware with the ros2_control ``MockSystem``. This emulator enables an environment for testing of "piping" of hardware and controllers, as well as testing robot's descriptions. For more details see `ros2_control documentation <https://ros-controls.github.io/control.ros.org/>`_ for more details.
