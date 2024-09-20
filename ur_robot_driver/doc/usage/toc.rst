#####
Usage
#####

This chapter explains how to use the ``ur_robot_driver``


.. toctree::
   :maxdepth: 4
   :caption: Contents:

   startup
   controllers
   simulation
   script_code

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

* To use mocked hardware(capability of ros2_control), use ``use_mock_hardware`` argument, like:

  .. code-block::

     ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=yyy.yyy.yyy.yyy use_mock_hardware:=true launch_rviz:=true

  **NOTE**\ : Instead of using the global launch file for control stack, there are also prepeared launch files for each type of UR robots named. They accept the same arguments are the global one and are used by:

  .. code-block::

     ros2 launch ur_robot_driver <ur_type>.launch.py

2. Sending commands to controllers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Before running any commands, first check the controllers' state using ``ros2 control list_controllers`` (Remember to install the ``ros2controlcli`` package as mentioned above).


* Send some goal to the Joint Trajectory Controller by using a demo node from `ros2_controllers_test_nodes <https://github.com/ros-controls/ros2_controllers/blob/master/ros2_controllers_test_nodes/ros2_controllers_test_nodes/publisher_joint_trajectory_controller.py>`_ package by starting  the following command in another terminal:

  .. code-block::

     ros2 launch ur_robot_driver test_scaled_joint_trajectory_controller.launch.py

  After a few seconds the robot should move.

* To test another controller, simply define it using ``initial_joint_controller`` argument, for example when using mock hardware:

  .. code-block::

     ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=yyy.yyy.yyy.yyy initial_joint_controller:=joint_trajectory_controller use_mock_hardware:=true launch_rviz:=true

  And send the command using demo node:

  .. code-block::

     ros2 launch ur_robot_driver test_joint_trajectory_controller.launch.py

  After a few seconds the robot should move(or jump when using emulation).

In case you want to write your own ROS node to move the robot, there is an example python node included that you can use as a start.


.. code-block:: console

   $ ros2 run ur_robot_driver example_move.py
   [INFO] [1720623611.547903428] [jtc_client]: Waiting for action server on scaled_joint_trajectory_controller/follow_joint_trajectory
   [INFO] [1720623611.548368095] [jtc_client]: Executing trajectory traj0
   [INFO] [1720623620.530203889] [jtc_client]: Done with result: SUCCESSFUL
   [INFO] [1720623622.530668700] [jtc_client]: Executing trajectory traj1
   [INFO] [1720623630.582108072] [jtc_client]: Done with result: SUCCESSFUL
   [INFO] [1720623632.582576444] [jtc_client]: Done with all trajectories
   [INFO] [1720623632.582957452] [jtc_client]: Done


.. warning::

   This is a very basic node that doesn't have the same safety checks as the test nodes above. Look
   at the code and make sure that the robot is able to perform the motions safely before running
   this on a real robot!


3. Using only robot description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you just want to test description of the UR robots, e.g., after changes you can use the following command:

.. code-block::

   ros2 launch ur_description view_ur.launch.py ur_type:=ur5e

Using MoveIt
------------

`MoveIt! <https://moveit.ros.org>`_ support is built-in into this driver already.

To test the driver with the example MoveIt-setup, first start the driver as described
`above <#start-hardware-simulator-or-mockup>`_ and then start the MoveIt! nodes using

.. code-block::

   ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true

Now you should be able to use the MoveIt Plugin in rviz2 to plan and execute trajectories with the
robot as explained `here <https://moveit.picknik.ai/main/doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial.html>`_.

For more details, please see :ref:`ur_moveit_config`.

Robot frames
------------

While most tf frames come from the URDF and are published from the ``robot_state_publisher``, there
are a couple of things to know:

- The ``base`` frame is the robot's base as the robot controller sees it.
- The ``tool0`` is the tool frame as calculated using forward kinematics. If there is no additional
  tool configured on the Teach pendant (TP) and the URDF uses the specific robot's
  :ref:`calibration <calibration_extraction>` the lookup from ``base`` -> ``tool0`` should be the
  same as the tcp pose shown on the TP when the feature "Base" is selected.
