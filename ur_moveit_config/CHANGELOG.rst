^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ur_moveit_config
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.4.13 (2024-10-28)
-------------------
* Pass use_sim_time to MoveIt's RViz instance (`#1144 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1144>`_)
* Contributors: Felix Exner (fexner)

2.4.12 (2024-10-14)
-------------------

2.4.11 (2024-10-10)
-------------------
* Add note about TEM (`#1136 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1136>`_)
* [moveit] Add config for trajectory execution and disable execution monitoring by default (`#1132 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1132>`_)
* Contributors: Felix Exner (fexner), G.A. vd. Hoorn

2.4.10 (2024-09-11)
-------------------

2.4.9 (2024-08-09)
------------------
* Added Jazzy migration notes for moveit_config (`#1058 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1058>`_)
* Add ur_moveit.launch.py arguments to control whether the move_group node publishes robot semantic description (`#1036 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1036>`_)
* Contributors: Felix Exner (fexner), Vincenzo Di Pentima, Haavard Pedersen Brandal

2.4.8 (2024-07-01)
------------------
* moveit_congig: Also install srdf folder (`#1033 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1033>`_)
* Contributors: Felix Exner (fexner)

2.4.7 (2024-06-19)
------------------

2.4.6 (2024-06-17)
------------------
* Make moveit_config compatible to moveit_configs_builder (`#998 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/998>`_)
* Contributors: Felix Exner (fexner), Vincenzo Di Pentima, Ruddick Lawrence

2.4.5 (2024-05-16)
------------------
* Fix multi-line strings in DeclareLaunchArgument (`#948 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/948>`_)
* Contributors: Matthijs van der Burgh

2.4.4 (2024-04-04)
------------------

2.4.3 (2024-02-02)
------------------
* fix move_group_node crash during initialization (`#906 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/906>`_)
* Add UR30 support (`#899 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/899>`_)
* Contributors: Chen Chen, Felix Exner (fexner)

2.4.2 (2023-11-23)
------------------
* moveit_servo package executable name has changed (`#854 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/854>`_)
* Contributors: Felix Durchdewald

2.4.1 (2023-09-21)
------------------
* Added support for UR20 (`#797 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/797>`_)
* Contributors: Felix Exner

2.4.0 (2023-08-28)
------------------
* Use mock_hardware and mock_sensor_commands instead of fake (`#739 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/739>`_)
  * Use mock_hardware and mock_sensor_commands instead of fake
  This has been deprecated a while back and was never adapted.
  * Update documentation to mock_hardware
* Contributors: Felix Exner (fexner)

2.3.2 (2023-06-02)
------------------
* Fixed formatting (`#685 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/685>`_)
  * Removed empty lines from python files
  * Fixed typo in changelogs
* Define default maximum accelerations for MoveIt (`#645 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/645>`_)
* Contributors: Felix Exner (fexner), RobertWilbrandt

2.3.1 (2023-03-16)
------------------

2.3.0 (2023-03-02)
------------------
* Fix capitalization of docstring
* Contributors: Felix Exner

2.2.4 (2022-10-07)
------------------
* Fix selecting the right controller given fake_hw
  This was falsely introduced earlier. This is a working version.
* add ur_moveit.launch.py parameter to use working controller when using fake hardware (`#464 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/464>`_)
  add script parameter to use correct controller when using fake hardware
* Contributors: Felix Exner, adverley

2.2.3 (2022-07-27)
------------------

2.2.2 (2022-07-19)
------------------
* Made sure all past maintainers are listed as authors (`#429 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/429>`_)
* Contributors: Felix Exner

2.2.1 (2022-06-27)
------------------
* Remove non-required dependency from CMakeLists (`#414 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/414>`_)
* Contributors: Felix Exner

2.2.0 (2022-06-20)
------------------
* Updated package maintainers
* Prepare for humble (`#394 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/394>`_)
* Update dependencies on all packages (`#391 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/391>`_)
* Replace warehouse_ros_mongo with warehouse_ros_sqlite (`#362 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/362>`_)
* Add missing dep to warehouse_ros_mongo (`#352 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/352>`_)
* Update license to BSD-3-Clause (`#277 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/277>`_)
* Correct loading kinematics parameters from yaml (`#308 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/308>`_)
* Update MoveIt file for working with simulation. (`#278 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/278>`_)
* Changing default controller in MoveIt config. (`#288 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/288>`_)
* Move Servo launching into the main MoveIt launch file. Make it optional. (`#239 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/239>`_)
* Joint limits parameters for Moveit planning (`#187 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/187>`_)
* Update Servo parameters, for smooth motion (`#188 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/188>`_)
* Enabling velocity mode (`#146 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/146>`_)
* Remove obsolete and unused files and packages. (`#80 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/80>`_)
* Review CI by correcting the configurations (`#71 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/71>`_)
* Add support for gpios, update MoveIt and ros2_control launching (`#66 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/66>`_)
* Contributors: AndyZe, Denis Štogl, Felix Exner, livanov93, Robert Wilbrandt
