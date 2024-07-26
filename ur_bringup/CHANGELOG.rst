^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ur_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.2.15 (2024-07-26)
-------------------

2.2.14 (2024-07-01)
-------------------

2.2.13 (2024-06-17)
-------------------
* Simplify launch file for ur_bringup pkg (`#1004 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1004>`_)
* Add calibration file to launch arguments (`#1001 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1001>`_)
* Contributors: Vincenzo Di Pentima

2.2.12 (2024-05-16)
-------------------

2.2.11 (2024-04-08)
-------------------

2.2.10 (2024-01-03)
-------------------

2.2.9 (2023-09-22)
------------------

2.2.8 (2023-06-26)
------------------

2.2.7 (2023-06-02)
------------------
* Default path to ur_client_library urscript (`#316 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/316>`_) (`#553 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/553>`_)
  * Change default path for urscript for headless mode.
  * Replace urscript path also in newer ur_robot_driver launchfile
* This commits adds additional configuration parameters needed for multiarm support.
* Contributors: Lennart Nachtigall, mergify[bot], livanov93

2.2.6 (2022-11-28)
------------------

2.2.5 (2022-11-19)
------------------

2.2.4 (2022-10-07)
------------------
* Remove duplicated update_rate parameter (`#479 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/479>`_)
* Contributors: Felix Exner

2.2.3 (2022-07-27)
------------------

2.2.2 (2022-07-19)
------------------
* Made sure all past maintainers are listed as authors (`#429 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/429>`_)
* Contributors: Felix Exner

2.2.1 (2022-06-27)
------------------

2.2.0 (2022-06-20)
------------------
* Updated package maintainers
* Rework bringup (`#403 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/403>`_)
* Fix force_torque_sensor_broadcaster config (`#405 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/405>`_)
* Prepare for humble (`#394 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/394>`_)
* Update dependencies on all packages (`#391 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/391>`_)
* Add sphinx documentation (`#340 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/340>`_)
* Use upstream fts_broadcaster (`#304 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/304>`_)
* Update license to BSD-3-Clause (`#277 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/277>`_)
* Only start controller_stopper when not using fake_hardware (`#332 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/332>`_)
* Added controller stopper node (`#309 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/309>`_)
* Fix package dependencies (`#306 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/306>`_)
* Make documentation on how to use driver clearer. (`#300 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/300>`_)
* Update MoveIt file for working with simulation. (`#278 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/278>`_)
* Start the tool communication script if the flag is set (`#267 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/267>`_)
* Used ``spawner`` instead of ``spanwer.py`` in launch files (`#293 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/293>`_)
* Do not start dashboard client if FakeHardware simuation is used. (`#286 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/286>`_)
* Use scaled trajectory controller per default. (`#287 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/287>`_)
* Separate control node (`#281 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/281>`_)
* Fix launch file arguments. (`#243 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/243>`_)
* Move Servo launching into the main MoveIt launch file. Make it optional. (`#239 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/239>`_)
* Tool communication (`#218 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/218>`_)
* fix missing executable arg of joint_state_broadcaster (`#248 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/248>`_)
* Remove obsolete comment 🐒 (`#242 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/242>`_)
* Revert "Ignition Gazebo simulation for UR robots (`#232 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/232>`_)" (`#241 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/241>`_)
* Use 'spawner' instead of 'spawner.py' (`#225 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/225>`_)
* Ignition Gazebo simulation for UR robots (`#232 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/232>`_)
* Empirically set update rates. (`#227 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/227>`_)
* Fix update rate configuration (`#222 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/222>`_)
* Test execution tests (`#216 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/216>`_)
* Integration tests improvement (`#206 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/206>`_)
* Add resend program service and enable headless mode (`#198 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/198>`_)
* Add default per joint constraints. (`#203 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/203>`_)
* Do not customize the planning scene topics (`#205 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/205>`_)
* Implement "choices" for robot_type param (`#204 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/204>`_)
* Joint limits parameters for Moveit planning (`#187 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/187>`_)
* Rename the joint controller that is launched by default (`#185 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/185>`_)
* Enabling velocity mode (`#146 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/146>`_)
* Add parameters for checking start state (`#143 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/143>`_)
* Update for changes to ros2_control and ros2_controllers
  See: https://github.com/ros-controls/ros2_control/commit/156a3f6aaed319585a8a1fd445693e2e08c30ccd
  and: https://github.com/ros-controls/ros2_controllers/commit/612f610c24d026a41abd2dd026902c672cf778c9#diff-5d3e18800b3a217b37b91036031bdb170f5183970f54d1f951bb12f2e4847706
* Removed dashboard client from hardware interface
* README cleanup, make MoveIt installation optional (`#86 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/86>`_)
* Using modern python
* Restore ur_control.launch.py
* Added view_ur for checking description
* Make an optional launch arg for RViz, document it in README (`#82 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/82>`_)
* Review CI by correcting the configurations (`#71 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/71>`_)
* Add support for gpios, update MoveIt and ros2_control launching (`#66 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/66>`_)
* Delete controller_stopper and ur_bringup pkgs
* Add XML schema to all ``package.xml`` files
  Better enable ``ament_xmllint`` to check validity.
* Update packaging for ROS2
* Update package.xml files so ``ros2 pkg list`` shows all pkgs
* Delete all launch/config files with no UR5 relation
* Update CMakeLists and package.xml for:
  - ur5_moveit_config
  - ur_bringup
  - ur_description
* Change pkg versions to 0.0.0
* Add ur5_moveit_config, ur_bringup, ur_description pkgs
* Contributors: AndyZe, Denis Stogl, Denis Štogl, Felix Exner, John Morris, Kenneth Bogert, Mads Holm Peters, Marvin Große Besselmann, Thomas Barbier, Vatan Aksoy Tezer, livanov93, relffok, Robert Wilbrandt
