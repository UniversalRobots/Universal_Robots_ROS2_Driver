#####
Usage
#####

This chapter explains how to use the ``ur_robot_driver``


.. toctree::
   :maxdepth: 4
   :caption: Contents:

   startup
   move
   controllers
   simulation
   script_code

Quick start
-----------

Driver startup
^^^^^^^^^^^^^^

* To start it with hardware, use:

  .. code-block:: console

     $ ros2 launch ur_robot_driver ur_control.launch.py ur_type:=<UR_TYPE> robot_ip:=<IP_OF_THE_ROBOT> launch_rviz:=true

  After starting the launch file start the external_control URCap program from the pendant, as described :ref:`here<robot_startup_program>`.

* To run it with URSim check details about it in :ref:`this section <usage_with_official_ur_simulator>`.

* To use mocked hardware (capability of ros2_control), use ``use_mock_hardware`` argument, like:

  .. code-block:: console

     $ ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=yyy.yyy.yyy.yyy use_mock_hardware:=true


Sending commands to controllers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Once the driver is started, you can use it like any other ROS robot arm driver by sending
trajectories to the ``/scaled_joint_trajectory_controller/follow_joint_trajectory`` action. If
you're new to that, please have a look at the section ":ref:`move_the_robot`".

Robot frames
------------

While most tf frames come from the URDF and are published from the ``robot_state_publisher``, there
are a couple of things to know:

- The ``base`` frame is the robot's base as the robot controller sees it.
- The ``tool0`` is the tool frame as calculated using forward kinematics. If there is no additional
  tool configured on the Teach pendant (TP) and the URDF uses the specific robot's
  :ref:`calibration <calibration_extraction>` the lookup from ``base`` -> ``tool0`` should be the
  same as the tcp pose shown on the TP when the feature "Base" is selected.
