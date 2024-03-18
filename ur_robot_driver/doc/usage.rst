.. role:: raw-html-m2r(raw)
   :format: html


Usage
=====

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
* ``use_fake_hardware`` (default: *false* ) - use simple hardware emulator from ros2_control.
  Useful for testing launch files, descriptions, etc. See explanation below.
* ``initial_positions`` (default: dictionary with all joint values set to 0) - Allows passing a dictionary to set the initial joint values for the fake hardware from `ros2_control <http://control.ros.org/>`_.  It can also be set from a yaml file with the ``load_yaml`` commands as follows:

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

* ``fake_sensor_commands`` (default: *false* ) - enables setting sensor values for the hardware emulators.
  Useful for offline testing of controllers.

* ``robot_controller`` (default: *joint_trajectory_controller* ) - controller for robot joints to be started.
  Available controllers:


  * joint_trajectory_controller
  * scaled_joint_trajectory_controller

  Note: *joint_state_broadcaster*\ , *speed_scaling_state_broadcaster*\ , *force_torque_sensor_broadcaster*\ , and *io_and_status_controller* will always start.

  *HINT* : list all loaded controllers using ``ros2 control list_controllers`` command.

**NOTE**\ : The package can simulate hardware with the ros2_control ``FakeSystem``. This emulator enables an environment for testing of "piping" of hardware and controllers, as well as testing robot's descriptions. For more details see `ros2_control documentation <https://ros-controls.github.io/control.ros.org/>`_ for more details.

Modes of operation
------------------

As mentioned in the last section the driver has two basic modes of operation: Using fake hardware or
using real hardware(Or the URSim simulator, which is equivalent from the driver's perspective).
Additionally, the robot can be simulated using
`Gazebo <https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation>`_ or
`ignition <https://github.com/UniversalRobots/Universal_Robots_ROS2_Ignition_Simulation>`_ but that's
outside of this driver's scope.

.. list-table::
   :header-rows: 1

   * - mode
     - available controllers
   * - fake_hardware
     - :raw-html-m2r:`<ul><li>joint_trajectory_controller</li><li>forward_velocity_controller</li><li>forward_position_controller</li></ul>`
   * - real hardware / URSim
     - :raw-html-m2r:`<ul><li>joint_trajectory_controller</li><li>scaled_joint_trajectory_controller </li><li>forward_velocity_controller</li><li>forward_position_controller</li></ul>`


Usage with official UR simulator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The easiest way to use URSim is the `Docker
image <https://hub.docker.com/r/universalrobots/ursim_e-series>`_ provided by Universal Robots (See
`this link <https://hub.docker.com/r/universalrobots/ursim_cb3>`_ for a CB3-series image).

To start it, we've prepared a script:

.. code-block:: bash

   ros2 run ur_client_library start_ursim.sh -m <ur_type>

With this, we can spin up a driver using

.. code-block:: bash

   ros2 launch ur_robot_driver ur_control.launch.py ur_type:=<ur_type> robot_ip:=192.168.56.101 launch_rviz:=true

You can view the polyscope GUI by opening `<http://192.168.56.101:6080/vnc.html>`_.

When we now move the robot in Polyscope, the robot's RViz visualization should move accordingly.

For details on the Docker image, please see the more detailed guide :ref:`here <ursim_docker>`.

Example Commands for Testing the Driver
---------------------------------------

Allowed UR - Type strings: ``ur3``\ , ``ur3e``\ , ``ur5``\ , ``ur5e``\ , ``ur10``\ , ``ur10e``\ , ``ur16e``\ , ``ur20``, ``ur30``.

1. Start hardware, simulator or mockup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


* To do test with hardware, use:

  .. code-block::

     ros2 launch ur_robot_driver ur_control.launch.py ur_type:=<UR_TYPE> robot_ip:=<IP_OF_THE_ROBOT> launch_rviz:=true

  For more details check the argument documentation with ``ros2 launch ur_robot_driver ur_control.launch.py --show-arguments``

  After starting the launch file start the external_control URCap program from the pendant, as described above.

* To do an offline test with URSim check details about it in `this section <#usage-with-official-ur-simulator>`_

* To use mocked hardware(capability of ros2_control), use ``use_fake_hardware`` argument, like:

  .. code-block::

     ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true launch_rviz:=true

  **NOTE**\ : Instead of using the global launch file for control stack, there are also prepeared launch files for each type of UR robots named. They accept the same arguments are the global one and are used by:

  .. code-block::

     ros2 launch ur_robot_driver <ur_type>.launch.py

2. Sending commands to controllers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Before running any commands, first check the controllers' state using ``ros2 control list_controllers``.


* Send some goal to the Joint Trajectory Controller by using a demo node from `ros2_control_demos <https://github.com/ros-controls/ros2_control_demos>`_ package by starting  the following command in another terminal:

  .. code-block::

     ros2 launch ur_robot_driver test_scaled_joint_trajectory_controller.launch.py

  After a few seconds the robot should move.

* To test another controller, simply define it using ``initial_joint_controller`` argument, for example when using fake hardware:

  .. code-block::

     ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=yyy.yyy.yyy.yyy initial_joint_controller:=joint_trajectory_controller use_fake_hardware:=true launch_rviz:=true

  And send the command using demo node:

  .. code-block::

     ros2 launch ur_robot_driver test_joint_trajectory_controller.launch.py

  After a few seconds the robot should move(or jump when using emulation).

3. Using only robot description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you just want to test description of the UR robots, e.g., after changes you can use the following command:

.. code-block::

   ros2 launch ur_description view_ur.launch.py ur_type:=ur5e

Using MoveIt
------------

`MoveIt! <https://moveit.ros.org>`_ support is built-in into this driver already.

Real robot / URSim
^^^^^^^^^^^^^^^^^^

To test the driver with the example MoveIt-setup, first start the driver as described
`above <#start-hardware-simulator-or-mockup>`_.

.. code-block::

   ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true

Now you should be able to use the MoveIt Plugin in rviz2 to plan and execute trajectories with the
robot as explained `here <https://moveit.picknik.ai/main/doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial.html>`_.

Fake hardware on ROS2 Galactic
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Currently, the ``scaled_joint_trajectory_controller`` does not work on ROS2 Galactic. There is an
`upstream Merge-Request <https://github.com/ros-controls/ros2_control/pull/635>`_ pending to fix that. Until this is merged and released, please change the
default controller in the `controllers.yaml <ur_moveit_config/config/controllers.yaml>`_ file. Make
sure that the ``default`` field is assigned ``true`` for the ``joint_trajectory_controller`` and ``false``
for the
``scaled_joint_trajectory_controller``.

.. code-block::

   controller_names:
     - scaled_joint_trajectory_controller
     - joint_trajectory_controller


   scaled_joint_trajectory_controller:
     action_ns: follow_joint_trajectory
     type: FollowJointTrajectory
     default: false
     joints:
       - shoulder_pan_joint
       - shoulder_lift_joint
       - elbow_joint
       - wrist_1_joint
       - wrist_2_joint
       - wrist_3_joint


   joint_trajectory_controller:
     action_ns: follow_joint_trajectory
     type: FollowJointTrajectory
     default: true
     joints:
       - shoulder_pan_joint
       - shoulder_lift_joint
       - elbow_joint
       - wrist_1_joint
       - wrist_2_joint
       - wrist_3_joint

Then start

.. code-block::

   ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true launch_rviz:=false initial_joint_controller:=joint_trajectory_controller
   # and in another shell
   ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true

Custom URScript commands
------------------------

The driver's package contains a ``urscript_interface`` node that allows sending URScript snippets
directly to the robot. It gets started in the driver's launchfiles by default. To use it, simply
publish a message to its interface:

.. code-block:: bash

  # simple popup
  ros2 topic pub /urscript_interface/script_command std_msgs/msg/String '{data: popup("hello")}' --once

Be aware, that running a program on this interface (meaning publishing script code to that interface) stops any running program on the robot.
Thus, the motion-interpreting program that is started by the driver gets stopped and has to be
restarted again. Depending whether you use headless mode or not, you'll have to call the
``resend_program`` service or press the ``play`` button on the teach panel to start the
external_control program again.

.. note::
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
