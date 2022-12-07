^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ur_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.2 (2022-12-07)
------------------
* executable name change (`#514 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/514>`_)
* Don't attempt to start dashboard_client when use_fake_hardware is true (`#486 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/486>`_)
* Contributors: Felix Exner, mertgungor

2.0.1 (2022-08-01)
------------------

2.0.0 (2022-06-20)
------------------
* Updated package dependencies (`#399 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/399>`_)
* Foxy: Update dependencies and binary repos file (`#373 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/373>`_)
* Foxy controller stopper (`#324 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/324>`_)
* Fix ros2 control xacro (`#213 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/213>`_)
* Add parameters for checking start state (`#143 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/143>`_)
* Update for changes to ros2_control and ros2_controllers
  See: https://github.com/ros-controls/ros2_control/commit/156a3f6aaed319585a8a1fd445693e2e08c30ccd
  and: https://github.com/ros-controls/ros2_controllers/commit/612f610c24d026a41abd2dd026902c672cf778c9#diff-5d3e18800b3a217b37b91036031bdb170f5183970f54d1f951bb12f2e4847706
* Removed dashboard client from hardware interface
* README cleanup, make MoveIt installation optional (`#86 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/86>`_)
* Using modern python
* Some intermediate commit
* Restore ur_control.launch.py
* Added view_ur for checking description
* Make an optional launch arg for RViz, document it in README (`#82 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/82>`_)
* Review CI by correcting the configurations (`#71 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/71>`_)
* Add support for gpios, update MoveIt and ros2_control launching (`#66 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/66>`_)
* Delete controller_stopper and ur_bringup pkgs
* Add XML schema to all ``package.xml`` files
* Update packaging for ROS2
* Update package.xml files so ``ros2 pkg list`` shows all pkgs
* Delete all launch/config files with no UR5 relation
* Update CMakeLists and package.xml for:
* Change pkg versions to 0.0.0
* Add ur5_moveit_config, ur_bringup, ur_description pkgs
* Contributors: AndyZe, Denis Stogl, Denis Štogl, Felix Exner, Gaël Écorchard, John Morris, Kenneth Bogert, Marvin Große Besselmann, livanov93

0.0.3 (2020-10-29)
------------------
