.. _ur_moveit_config:

================
ur_moveit_config
================

This package contains an **example** MoveIt! configuration for Universal Robots arms. Since the
default description contains only the arm, this MoveIt! configuration package also only contains the
arm without any objects around it.
In a real-world scenario it is recommended to create a robot_description modelling the robot with its surroundings (e.g. table where it is mounted on, objects in its environment, etc.) and to generate a
*scenario_moveit_config* package from that description as explained in the :ref:`Custom workcell
tutorial <custom_workcell_tutorial>`.

Usage
-----

With a running driver (real hardware, URSim or mocked hardware), simply start the MoveIt!
interaction using

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

   .. literalinclude:: ../config/moveit_controllers.yaml
      :language: yaml
      :start-at: trajectory_execution:
      :end-at: execution_duration_monitoring
      :caption: moveit_controllers.yaml
