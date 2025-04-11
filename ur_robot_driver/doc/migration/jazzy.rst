:github_url: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/main/ur_robot_driver/doc/migration/jazzy.rst

ur_robot_driver
^^^^^^^^^^^^^^^

keep_alive_count -> robot_receive_timeout
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Doing real-time control with a robot requires the control pc to conform to certain timing
constraints. For this ``keep_alive_count`` was used to estimate the tolerance that was given to the
ROS controller in terms of multiples of 20 ms. Now the timeout is directly configured using the
``robot_receive_timeout`` parameter of the hardware interface.


ros2_control xacro tag moved to driver package
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The description package no longer adds the ``ros2_control`` tag to the robot's URDF. This is done
in the driver now. Therefore, there is a ``urdf`` folder containing the macro for generating a
``ros2_control`` tag and a ready-to-use xacro file for a UR robot with a ``ros2_control`` tag.

If you want to create your own controlled robot, you should mimic that structure, as done e.g. in
the `custom_workcell_tutorial`_.

.. _custom_workcell_tutorial: https://github.com/UniversalRobots/Universal_Robots_ROS2_Tutorials/blob/main/my_robot_cell/my_robot_cell_control/urdf/my_robot_cell_controlled.urdf.xacro

robot_description is now distributed by the robot_state_publisher
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The ``robot_description`` isn't propagated to all interested parties as a parameter. Instead, only
the ``robot_state_publisher`` is provided with the parameter, republishing it on the
``robot_description`` topic.

Therefore, there is a new launchfile ``ur_rsp.launch.py`` which takes care about loading the
description and starting the ``robot_state_publisher``. If you create a description that needs
other parameters than the packaged one, you can write your own ``rsp`` launch file and pass that as
``description_launchfile`` argument to ``ur_control.launch.py``.

Enforce absolute paths in launchfiles
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

All launchfiles now expect an absolute path for files that should be used to alter the launch
process (e.g. description file, controllers file). Before it was expecting e.g. a
``description_package`` and a ``description_file`` argument with a relative path to the package.

The default files have not been changed, so unless you specified your custom package / file
combinations, you will need to update that to an absolute path.

Absolute paths can still be generated dynamically using a package + relative path structure inside
other launchfiles or by using ``ros2 pkg prefix`` on the command line. For example, you can do

.. code-block:: console

   $ ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur20 robot_ip:=192.168.56.101 \
     kinematics_params_file:=$(ros2 pkg prefix my_robot_cell_control)/share/my_robot_cell_control/config/my_robot_calibration.yaml
