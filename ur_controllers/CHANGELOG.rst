^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ur_controllers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.2.15 (2024-07-26)
-------------------
* Updated scaled JTC to latest upstream updates (`#1067 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1067>`_)
* Contributors: Felix Exner (fexner)

2.2.14 (2024-07-01)
-------------------

2.2.13 (2024-06-17)
-------------------
* this simple fix should fix the goal time violated issue (backport of `#882 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/882>`_)
* Contributors: Lennart Nachtigall

2.2.12 (2024-05-16)
-------------------
* Use latched publishing for robot_mode and safety_mode (`#991 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/991>`_)
* Contributors: Felix Exner

2.2.11 (2024-04-08)
-------------------

2.2.10 (2024-01-03)
-------------------
* Update JTC API (`#896 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/896>`_)
* Remove noisy controller log message (`#858 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/858>`_)
* Contributors: Felix Exner (fexner), mergify[bot], Robert Wilbrandt

2.2.9 (2023-09-22)
------------------
* Update sjtc to newest upstream API
* Contributors: Felix Exner

2.2.8 (2023-06-26)
------------------

2.2.7 (2023-06-02)
------------------
* added missing command interfaces into gpio controller (`#693 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/693>`_) (`#702 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/702>`_)
* Adding maximum retry counter in gpio controller (Multiarm part 3) - v2 (`#672 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/672>`_) (`#696 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/696>`_)
* Ported controllers to generate_parameters library and added prefix for controllers (Multiarm part 2) (`#594 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/594>`_) (`#695 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/695>`_)
* Introduce hand back control service (`#528 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/528>`_) (`#670 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/670>`_)
* Added services to set tool voltage and zero force torque sensor (`#466 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/466>`_) (`#582 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/582>`_)
* Contributors: mergify[bot], Mads Holm Peters, Lennart Nachtigall, livanov93

2.2.6 (2022-11-28)
------------------
* Ros2 controllers 2.14 (`#547 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/547>`_)
* Contributors: Felix Exner

2.2.5 (2022-11-19)
------------------
* Revert "Adapt jtc controller params to new param api"
  This reverts commit 65ac3679004fb0a622b00d334fa57056607dd23f.
* Contributors: Felix Exner

2.2.4 (2022-10-07)
------------------
* Adapt jtc controller params to new param api
* Contributors: Felix Exner

2.2.3 (2022-07-27)
------------------
* Adapt ros control api (`#448 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/448>`_)
  * scaled jtc: Use get_interface_name instead of get_name
  * Migrate from stopped controllers to inactive controllers
  stopped controllers has been depreated upstream
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
