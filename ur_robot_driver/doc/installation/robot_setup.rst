:github_url: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/main/ur_robot_driver/doc/installation/robot_setup.rst

Setting up a UR robot for ur_robot_driver
=========================================

Prepare robot and network connection
------------------------------------

Before you can use the ``ur_robot_driver`` you need to prepare the robot and the network
connection as described in the :ref:`robot_setup`  and :ref:`network_setup` section of the UR Client Library documentation.

Prepare the ROS PC
------------------

For using the driver make sure it is installed (either by the debian package or built from source
inside a colcon workspace).

.. _calibration_extraction:

Extract calibration information
-------------------------------

Each UR robot is calibrated inside the factory giving exact forward and inverse kinematics. To also
make use of this in ROS, you first have to extract the calibration information from the robot.

Though this step is not necessary to control the robot using this driver, it is highly recommended
to do so, as otherwise endeffector positions might be off in the magnitude of centimeters.

For this, there exists a helper script:

.. code:: bash

   $ ros2 launch ur_calibration calibration_correction.launch.py \
   robot_ip:=<robot_ip> target_filename:="${HOME}/my_robot_calibration.yaml"

.. note::
   The robot must be powered on (can be idle) before executing this script.


For the parameter ``robot_ip`` insert the IP address on which the ROS pc can reach the robot. As
``target_filename`` provide an absolute path where the result will be saved to.

See :ref:`ur_robot_driver_startup` for instructions on using the extracted calibration information.
