Usage
=====

To use the calibration correction this package provides a launchfile that extracts calibration
information directly from a robot, calculates the URDF correction and saves it into a .yaml file:

.. code-block:: bash

   $ ros2 launch ur_calibration calibration_correction.launch.py \
   robot_ip:=<robot_ip> target_filename:="${HOME}/my_robot_calibration.yaml"

For the parameter ``robot_ip`` insert the IP address on which the ROS pc can reach the robot. As
``target_filename`` provide an absolute path where the result will be saved to.

With that, you can launch your specific robot with the correct calibration using

.. code-block:: bash

   $ ros2 launch ur_robot_driver ur_control.launch.py \
     ur_type:=ur5e \
     robot_ip:=192.168.56.101 \
     kinematics_params_file:="${HOME}/my_robot_calibration.yaml"

Adapt the robot model matching to your robot.

Ideally, you would create a package for your custom workcell, as explained in `the custom workcell
tutorial
<https://github.com/UniversalRobots/Universal_Robots_ROS2_Tutorials/blob/main/my_robot_cell/doc/start_ur_driver.rst#extract-the-calibration>`_.
