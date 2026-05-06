:github_url: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/main/ur_robot_driver/doc/migration/lyrical.rst

ur_robot_driver
^^^^^^^^^^^^^^^

Removal of scaled_joint_trajectory_controller
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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


Automatic robot model verification at startup
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

In Lyrical, the robot model is automatically verified at startup. The robot model will be queried
from the connected robot and compared to the robot model specified in the URDF. If the models do
not match, configuring the hardware will fail and an error will be logged.

This feature is enabled by default, but can be disabled by setting the ``verify_robot_model``
parameter to ``false`` in the hardware configuration file. For example, the following command will
launch the driver without verifying the robot model:

.. code::

   ros2 launch ur_robot_driver ur_control.launch.py \
     robot_ip:=192.168.56.101 \
     ur_type:=ur18 \
     verify_robot_model:=false


.. note::

   This feature is also available in previous ROS distributions, but is disabled by default. It is
   recommended to enable this feature to ensure that the correct robot model is being used.
