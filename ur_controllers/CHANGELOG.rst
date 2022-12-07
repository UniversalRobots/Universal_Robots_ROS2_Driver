^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ur_controllers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.2 (2022-12-07)
------------------

2.0.1 (2022-08-01)
------------------

2.0.0 (2022-06-20)
------------------
* Updated package dependencies (`#399 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/399>`_)
* Foxy controller stopper (`#324 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/324>`_)
* Fixing foxy CI (`#157 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/157>`_)
* Moved registering publisher and service to on_active (`#151 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/151>`_)
* Correct formatting, include std::vector and update ros2_controller to master branch in repo file.
* Correct check for fixed has_trajectory_msg()
* Update for changes to ros2_control and ros2_controllers
* Fix gpio controller (`#103 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/103>`_)
* Fixed speed slider service call (`#100 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/100>`_)
* Reintegrating missing ur_client_library dependency since the break the building process (`#97 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/97>`_)
* Setting speed slider with range of 0.0-1.0 and added warnings if range is exceeded (`#88 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/88>`_)
* Fix move to home bug (`#92 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/92>`_)
* Last fix-ups
* Some intermediate commit
* Last fix-ups
* Review CI by correcting the configurations (`#71 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/71>`_)
* Add support for gpios, update MoveIt and ros2_control launching (`#66 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/66>`_)
* Fix warning about deprecated controller_interface::return_type::SUCCESS (`#68 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/68>`_)
* Use GitHub Actions, use pre-commit formatting (`#56 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/56>`_)
* Scaled Joint Trajectory Controller (`#43 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/43>`_)
* Code cleanup
* Only load speed scaling interface
* Removed controller from config file to realign with current branch
* Removed last remnants of joint_state_controller
* Added publisher rate
* Code formatting and cleanup
* Added publisher for speed scaling factor
* Initial version of the speed_scaling_state_controller
  Controller is base on the current joint_state_controller of ros2 control
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
* Silence ``ament_lint_cmake`` errors
* Update package.xml files so ``ros2 pkg list`` shows all pkgs
* Clean out ur_controllers, it needs a complete rewrite
* Update CMakeLists and package.xml for:
  - ur5_moveit_config
  - ur_bringup
  - ur_description
* Change pkg versions to 0.0.0
* Contributors: AndyZe, Denis Stogl, Denis Štogl, Felix Exner, John Morris, Kenneth Bogert, Lovro, Mads Holm Peters, Marvin Große Besselmann, livanov93

0.0.3 (2020-10-29)
------------------
* Initial copy of the ROS1 driver
* Contributors: AndyZe
