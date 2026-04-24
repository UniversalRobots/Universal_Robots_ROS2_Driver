:github_url: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/main/ur_robot_driver/doc/migration/lyrical.rst

ur_robot_driver
^^^^^^^^^^^^^^^

Since the ``scaled_joint_trajectory_controller`` has been removed in ROS Lyrical Luth, the default
controller is now the ``joint_trajectory_controller``. The reason for this is the scaling capabilities now has been merged upstream to ``joint_trajectory_controller``. Thus, making the ``scaled_joint_trajectory_controller`` redundant.

User applications will most likely have to change the target action server for execution motions

``scaled_joint_trajectory_controller/follow_joint_trajectory`` -> ``joint_trajectory_controller/follow_joint_trajectory``

In custom controller specification files it is enough to change the controller type and the speed
scaling state interface parameter. Go from

.. code:: yaml

   controller_manager:
     ros__parameters:
       my_traj_controller:
         type: ur_controllers/ScaledJointTrajectoryController
   my_traj_controller:
     ros__parameters:
       speed_scaling_interface_name: $(var tf_prefix)speed_scaling/speed_scaling_factor

to

.. code:: yaml

   controller_manager:
     ros__parameters:
       my_traj_controller:
         type: joint_trajectory_controller/JointTrajectoryController
   my_traj_controller:
     ros__parameters:
       speed_scaling:
         state_interface: $(var tf_prefix)speed_scaling/speed_scaling_factor
