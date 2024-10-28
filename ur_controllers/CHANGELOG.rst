^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ur_controllers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.4.13 (2024-10-28)
-------------------
* [SJTC] Make scaling interface optional (`#1145 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1145>`_)
* Contributors: Felix Exner (fexner)

2.4.12 (2024-10-14)
-------------------

2.4.11 (2024-10-10)
-------------------
* Allow setting the analog output domain when setting an analog IO (`#1123 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1123>`_)
* Service to get software version of robot (`#964 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/964>`_)
* Improve usage documentation (`#1110 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1110>`_)
* Contributors: Felix Exner (fexner), URJala, Rune Søe-Knudsen

2.4.10 (2024-09-11)
-------------------
* Updated get_state to get_lifecycle_state (`#1087 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1087>`_)
* Update maintainers team (`#1088 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1088>`_)
* Contributors: Vincenzo Di Pentima

2.4.9 (2024-08-09)
------------------

2.4.8 (2024-07-01)
------------------

2.4.7 (2024-06-19)
------------------

2.4.6 (2024-06-17)
------------------
* this simple fix should fix the goal time violated issue (`#882 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/882>`_)
* Restructure documentation for full stack documentation (`#984 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/984>`_)
* Contributors: Felix Exner (fexner), Lennart Nachtigall, Vincenzo Di Pentima

2.4.5 (2024-05-16)
------------------
* Use latched publishing for robot_mode and safety_mode
* Contributors: Felix Exner

2.4.4 (2024-04-04)
------------------

2.4.3 (2024-02-02)
------------------

2.4.2 (2023-11-23)
------------------
* Update read_state_from_hardware
* Renamed normalize_joint_error to joints_angle_wraparound
* Remove noisy controller log message
* Contributors: Felix Exner, Robert Wilbrandt

2.4.1 (2023-09-21)
------------------
* Update sjtc to newest upstream API (`#810 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/pull/810>`_)
* Contributors: Felix Exner

2.4.0 (2023-08-28)
------------------
* Handle api changes related to traj_external_point_ptr\_ (`#779 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/779>`_)
  * Handle api changes related to traj_external_point_ptr\_
  * Fix formatting
  ---------
  Co-authored-by: Robert Wilbrandt <wilbrandt@fzi.de>
* Contributors: Yadu

2.3.2 (2023-06-02)
------------------
* added missing command interfaces into gpio controller (`#693 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/693>`_)
* Fixed formatting (`#685 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/685>`_)
  * Removed empty lines from python files
  * Fixed typo in changelogs
* Adding maximum retry counter in gpio controller (Multiarm part 3) - v2 (`#672 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/672>`_)
* Ported controllers to generate_parameters library and added prefix for controllers (Multiarm part 2) (`#594 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/594>`_)
* Switched out a deprecated header to avoid buildfarm warnings.
* Introduce hand back control service (`#528 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/528>`_)
* Contributors: Felix Exner, Felix Exner (fexner), Lennart Nachtigall, livanov93

2.3.1 (2023-03-16)
------------------
* Adjust scaled jtc to new publish_state interface
  Until next sync we need to build against upstream ros2_controllers, as
  this is an API-breaking change
* Contributors: Robert Wilbrandt

2.3.0 (2023-03-02)
------------------
* Added services to set tool voltage and zero force torque sensor (`#466 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/466>`_)
  Added launch arguments for reverse ip and script command interface port.
* Contributors: Mads Holm Peters

2.2.4 (2022-10-07)
------------------
* Adapt jtc controller params to new param api
* Contributors: Felix Exner

2.2.3 (2022-07-27)
------------------
* Adapt ros control api (`#448 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/448>`_)
  * scaled jtc: Use get_interface_name instead of get_name
  * Migrate from stopped controllers to inactive controllers
  stopped controllers has been deprecated upstream
* Contributors: Felix Exner

2.2.2 (2022-07-19)
------------------
* Adapted to JTC interpolation method feature (`#439 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/439>`_)
* Made sure all past maintainers are listed as authors (`#429 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/429>`_)
* Contributors: Felix Exner

2.2.1 (2022-06-27)
------------------

2.2.0 (2022-06-20)
------------------
* Updated package maintainers
* Prepare for humble (`#394 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/394>`_)
* Update dependencies on all packages (`#391 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/391>`_)
* Update controllers' API (`#351 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/351>`_)
* Update binary dependencies (`#344 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/344>`_)
* Use upstream fts_broadcaster (`#304 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/304>`_)
* Update license to BSD-3-Clause (`#277 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/277>`_)
* Added controller stopper node (`#309 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/309>`_)
* Add missing dependency on angles and update formatting for linters. (`#283 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/283>`_)
* Payload service (`#238 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/238>`_)
* Integration tests improvement (`#206 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/206>`_)
* Add resend program service and enable headless mode (`#198 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/198>`_)
* Update controllers adding dt in to update as in ros2_control (`#171 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/171>`_)
* Update main branch with ros-controls changes (`#160 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/160>`_)
* Update CI configuration to support galactic and rolling (`#142 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/142>`_)
* Modify parameter declaration - approach equalization with ros-controls dependencies (`#152 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/152>`_)
* Moved registering publisher and service to on_active (`#151 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/151>`_)
* Correct formatting, include std::vector and update ros2_controller to master branch in repo file.
* Correct check for fixed has_trajectory_msg()
  See: https://github.com/ros-controls/ros2_controllers/commit/32f089b3f3b53a817412c6bbce9046028786431e
* Update for changes to ros2_control and ros2_controllers
  See: https://github.com/ros-controls/ros2_control/commit/156a3f6aaed319585a8a1fd445693e2e08c30ccd
  and: https://github.com/ros-controls/ros2_controllers/commit/612f610c24d026a41abd2dd026902c672cf778c9#diff-5d3e18800b3a217b37b91036031bdb170f5183970f54d1f951bb12f2e4847706
* Fix gpio controller (`#103 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/103>`_)
* Fixed speed slider service call (`#100 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/100>`_)
* Reintegrating missing ur_client_library dependency since the break the building process (`#97 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/97>`_)
* Setting speed slider with range of 0.0-1.0 and added warnings if range is exceeded (`#88 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/88>`_)
* Fix move to home bug (`#92 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/92>`_)
* Review CI by correcting the configurations (`#71 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/71>`_)
* Add support for gpios, update MoveIt and ros2_control launching (`#66 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/66>`_)
* Fix warning about deprecated controller_interface::return_type::SUCCESS (`#68 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/68>`_)
* Use GitHub Actions, use pre-commit formatting (`#56 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/56>`_)
* Scaled Joint Trajectory Controller (`#43 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/43>`_)
* Only load speed scaling interface
* Removed controller from config file to realign with current branch status
* Removed last remnants of joint_state_controller
* Added publisher rate
* Code formatting and cleanup
* Added publisher for speed scaling factor
* Initial version of the speed_scaling_state_controller
* Update licence.
* Fix clang tidy in multiple pkgs.
* Update force torque state controller.
* Prepare for testing.
* Update ft state controller with ros2_control changes.
* Remove lifecycle node (update with ros2_control changes).
* Claim individual resources.
* Add force torque controller.
* Claim individual resources.
* Add force torque controller.
* Add XML schema to all ``package.xml`` files
  Better enable ``ament_xmllint`` to check validity.
* Update package.xml files so ``ros2 pkg list`` shows all pkgs
* Clean out ur_controllers, it needs a complete rewrite
* Update CMakeLists and package.xml for:
  - ur5_moveit_config
  - ur_bringup
  - ur_description
* Change pkg versions to 0.0.0
* Contributors: AndyZe, Denis Stogl, Denis Štogl, Felix Exner, John Morris, Kenneth Bogert, Lovro, Mads Holm Peters, Marvin Große Besselmann, livanov93
