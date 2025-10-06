.. _trajectory_until_node:

Trajectory until node
=====================

The trajectory until node allows a user to execute a trajectory with an "until" condition enabled (currently only tool contact is available) without having to call 2 actions at the same time. This means that the trajectory will execute until either the trajectory is finished or the "until" condition has been triggered. Both scenarios will result in the trajectory being reported as successful.

Action interface / usage
""""""""""""""""""""""""
The node provides an action to execute a trajectory with tool contact enabled. For the node to accept action goals, both the motion controller and the tool contact controller need to be in ``active`` state.

* ``/trajectory_until_node/execute [ur_msgs/action/TrajectoryUntil]``

The action contains all the same fields as the ordinary `FollowJointTrajectory <http://docs.ros.org/en/noetic/api/control_msgs/html/action/FollowJointTrajectory.html>`_ action, but has two additional fields.
One in the goal section called ``until_type``, which is used to choose between different conditions that can stop the trajectory. Currently only tool contact is available.
The result section contains the other new field called ``until_condition_result``, which reports whether the chosen condition was triggered or not.

Implementation details
""""""""""""""""""""""
Upon instantiation of the node, the internal trajectory action client will connect to an action named ``motion_controller/follow_joint_trajectory``.
This action does not exist, but upon launch of the driver, the node is remapped to connect to the ``initial_joint_controller``, default is ``scaled_joint_trajectory_controller``.
If you wish to use the node with another motion controller use the launch argument ``initial_joint_controller:=<your_motion_controller>`` when launching the driver.
The node is only compatible with motion controllers that use the FollowJointTrajectory action interface.
