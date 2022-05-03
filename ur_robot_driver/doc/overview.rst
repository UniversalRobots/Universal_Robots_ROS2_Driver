Overview
========

This driver collaborates closely with other ROS packages:

``ur_bringup``
  Package to launch the driver, simulated robot and test scripts.
``ur_calibration``
  Package containing the calibration extraction program that will extract parameters for correctly
  parametrizing the URDF with calibration data from the specific robot.
``ur_controllers``
  Controllers specifically made for UR manipulators.
``ur_dashboard_msgs``
  Message packages used for the `dashboard <https://www.universal-robots.com/articles/ur/dashboard-server-e-series-port-29999/>`_ communication.
``ur_moveit_config``
  MoveIt! configuration for a plain robot. This is good as a starting point, but for a real
  application you would rather create your own MoveIt! configuration package containing your actual
  robot environment.
``ur_description`` (`separate repository <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description>`_)
  URDF description for UR manipulators

.. include:: features.rst
