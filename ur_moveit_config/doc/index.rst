.. _ur_moveit_config:
================
ur_moveit_config
================

This package contains an **example** MoveIt! configuration for Universal Robots arms. Since the
default description contains only the arm this MoveIt! configuration package also only contains the
arm without any objects around it.

Since a robotic arm is usually not floating in empty space, but is mounted somewhere, it is also
recommended to create a robot_description modelling the real-world scenario and to generate a
*scenario_moveit_config* package from that description as explained in the :ref:`Custom workcell
tutorial <custom_workcell_tutorial>`.

Usage
-----

With a running driver (Real hardware, URSim or mocked hardware), simply start the MoveIt!
interaction using

.. code-block::

   ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true

Now you should be able to use the MoveIt Plugin in rviz2 to plan and execute trajectories with the
robot as explained `here <https://moveit.picknik.ai/main/doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial.html>`_.

