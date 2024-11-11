.. _move_the_robot:

Move the robot
==============

First check the controllers' state using ``ros2 control list_controllers``, before running any commands. (Remember to install the ``ros2controlcli`` package).


* Send goals to the Joint Trajectory Controller by using a demo node from `ros2_controllers_test_nodes <https://github.com/ros-controls/ros2_controllers/blob/master/ros2_controllers_test_nodes/ros2_controllers_test_nodes/publisher_joint_trajectory_controller.py>`_ package by starting  the following command in another terminal:

  .. code-block:: console

     $ ros2 launch ur_robot_driver test_scaled_joint_trajectory_controller.launch.py

  The robot should move, after a few seconds.

* To test another controller, simply define it using ``initial_joint_controller`` argument, for example when using mock hardware:

  .. code-block:: console

     $ ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e \
       robot_ip:=yyy.yyy.yyy.yyy use_mock_hardware:=true \
       initial_joint_controller:=joint_trajectory_controller

  And send the command using demo node:

  .. code-block:: console

     $ ros2 launch ur_robot_driver test_joint_trajectory_controller.launch.py

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


Using MoveIt
------------

`MoveIt! <https://moveit.ros.org>`_ support is built-in into this driver already.

To test the driver with the example MoveIt-setup, first start the driver as described
`above <#start-hardware-simulator-or-mockup>`_ and then start the MoveIt! nodes using

.. code-block::

   ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true

Now you should be able to use the MoveIt Plugin in rviz2 to plan and execute trajectories with the
robot as explained `here <https://moveit.picknik.ai/main/doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial.html>`_.

.. note::
   The MoveIt configuration provided here has Trajectory Execution Monitoring (TEM) *disabled*, as the
   Scaled Joint Trajectory Controller may cause trajectories to be executed at a lower velocity
   than they were originally planned by MoveIt. MoveIt's TEM however is not aware of this
   deliberate slow-down due to scaling and will in most cases unnecessarily (and unexpectedly)
   abort goals.

   Until this incompatibility is resolved, the default value for ``execution_duration_monitoring``
   is set to ``false``. Users who wish to temporarily (re)enable TEM at runtime (for use with
   other, non-scaling controllers) can do so using the ROS 2 parameter services supported by
   MoveIt.

For more details, please see :ref:`ur_moveit_config`.
