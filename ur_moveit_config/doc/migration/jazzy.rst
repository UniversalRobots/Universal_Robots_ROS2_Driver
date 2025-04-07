:github_url: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/main/ur_moveit_config/doc/migration/jazzy.rst

ur_moveit_config
^^^^^^^^^^^^^^^^

Restructuring for moveit_configs_builder compatibility
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Many config files have been either renamed or rewritten in order to allow this package to use the moveit_configs_builder. In this way, it becomes a good basic example for packages of this kind: from this perspective, aiming for simplicity and not for completeness, it consequently avoids the implementation of all the possible features offered by the configs builder, e.g. the multiple IK solutions available, which are left to the users to be explored.

Robot description and semantic description updates
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Differently from before, the package allows to load the description from the robot_state_publisher's topic, also adding a "lazy" starting possibility of the move_group. In fact, the launcher waits 10 seconds for the description topic to be available, giving the user enough time to launch the robot_state_publisher.

In addition to that, now the launcher also allows the move_group to publish the semantic description. Enabled by the argument ``publish_robot_description_semantic``, it improves the reusability of the launch file: including the ``ur_moveit.launch.py`` in a custom launch file and then launching the MoveGroupInterface, allows users to do avoid manual definition for the srdf, since the MoveGroupInterface will pull such info from the move_group node.


Removed tf_prefix support
~~~~~~~~~~~~~~~~~~~~~~~~~

Switching to the moveit_configs_builder doesn't allow the ``tf_prefix`` option to work anymore, hence motivating its removal from this package.
This is due to the presence of config files containing information about joints, like ``joint_limits.yaml``: since they can't handle argument substitution during loading inside the configs builder, the ``tf_prefix`` can't be specified for them.
If necessary, this is left to be handled by the user through a definition for it in a custom moveit_config package.
