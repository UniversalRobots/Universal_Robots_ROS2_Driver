2.3.2 (2023-06-02)
------------------
* Adds full nonblocking readout support (Multiarm part 4)  - v2 (`#673 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/673>`_)
* Removed workaround also in export_command_interfaces (`#692 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/692>`_)
* Calling on_deactivate in dtr (`#679 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/679>`_)
* Fixed formatting (`#685 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/685>`_)
* Remove tf_prefix workaround in hw interface
* Ported controllers to generate_parameters library and added prefix for controllers (Multiarm part 2) (`#594 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/594>`_)
* Remove ur_bringup package (`#666 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/666>`_)
* Introduce hand back control service (`#528 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/528>`_)
* Apply suggestions from code review
* Update definition of test goals to new version.
* Wait longer for controllers to load and activate
* Fix flaky tests (`#641 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/641>`_)
  * Move robot startup into test's setUp function
  * Robustify robot startup
* This commits adds additional configuration parameters needed for multiarm support.
* Add timeout to execution test
* Improve logging for robot execution tests
* Contributors: Denis Štogl, Dr. Denis, Felix Exner, Felix Exner (fexner), Lennart Nachtigall, Robert Wilbrandt, livanov93

2.3.1 (2023-03-16)
------------------
* Adjust controller switching to message change
* Controller spawner timeout (`#608 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/608>`_)
  * Simplify controller spawner definitions
  * Ignore flake8 W503 as it clashes with black and goes against PEP8 style
  * Add argument to set controller spawner timeout
  * Use longer controller manager timeout in CI
  The default timeout of 10s is the same as our RTDE retry timeout, which
  means if RTDE does not immediately connect (which happens regularly in
  CI runners) controller spawning would fail.
* Increase timeout for first test service call to driver (`#605 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/605>`_)
* Contributors: Robert Wilbrandt, RobertWilbrandt

2.3.0 (2023-03-02)
------------------
* Fix cmake dependency on controller_manager
* Correct calibration correction launch file in doc
* Added services to set tool voltage and zero force torque sensor (`#466 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/466>`_)
  Added launch arguments for reverse ip and script command interface port.
* Fix comment in test file
* Default path to ur_client_library urscript (`#316 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/316>`_)
  * Change default path for urscript for headless mode.
  * Replace urscript path also in newer ur_robot_driver launchfile
  * Remove ros_control.urscript
  Co-authored-by: Felix Exner <exner@fzi.de>
* Clean up & improve execution tests (`#512 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/512>`_)
  * Clean up execution test files
  * Start ursim as part of the execution tests
  * Dont use custom dockerursim for humble and rolling execution tests
  * Clean up test implementations
  * pep257 fixes
  * Perform rolling and humble execution tests as part of normal pipelines
  * Increase admissible timeouts as the CI needs to pull ursim first
  * Add more debug messages during tests
  * Wait until robot is in POWER_OFF mode before trying to power it on
  * Fix error introduced in last commit
  * Add additional cmake option to enable integration tests
  * Increase timeout for robot tests
  * Add CMake comment describing the execution test integration
  * Run source tests on pull request
  This is only here for testing the test setup! Remove before merging
  * call resend_robot_program twice
  This seems to be necessary, as otherwise the robot hangs after bootup.
  The first program execution (that gets automatically started at driver
  startup because of the headless_mode) gets paused, since it is sent while
  the robotis not yet switched on. To mitigate this, we send the robot program
  again after switching on the robot, but this seems to stop the robot program.
  Sending it again seems to set it correctly to a started state.
  * Increase timeouts for dashboard_client tests
  Otherwise they can fail, since in parallel we pull and start the docker
  container.
  Co-authored-by: Felix Exner <exner@fzi.de>
* Update and thin down README (`#494 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/494>`_)
  Avoid duplication between README and package doc.
  * Updated documentation about fake_hardware and MoveIt!
  * Remove trailing WS
  * [documentation] do not suggest -r for rosdep install
  * Added note about tool0_controller to docs.
  * Add additional part about calibration to toplevel README.
  * Added note about sourcing ROS in build instructions
* ur_robot_driver: Controller_stopper fix deprecation warning
  Use ``activate_controllers`` instead of ``start_controllers``.
* Fix tool voltage setup (`#526 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/526>`_)
  * Move BEGIN_REPLACE inside of header
  * Change default value of tool_voltage
  Keeping this at 0 requires users to explicitly set it to non-zero. This way
  we won't accitentally destroy hardware that cannot handle 24V.
* Added dependency to socat (`#527 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/527>`_)
  This is needed for the tool forwarding.
* Add a note in the tool_comm doc about a URCap conflict (`#524 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/524>`_)
  * Add a note in the tool_comm doc about a URCap conflict
  * Update ur_robot_driver/doc/setup_tool_communication.rst
  Co-authored-by: Mads Holm Peters <79145214+urmahp@users.noreply.github.com>
  * Fix formatting and one spelling mistake
  Co-authored-by: Mads Holm Peters <79145214+urmahp@users.noreply.github.com>
* Contributors: Felix Exner, Felix Exner (fexner), Mads Holm Peters, Robert Wilbrandt, RobertWilbrandt, livanov93

2.2.4 (2022-10-07)
------------------
* Remove the custom ursim docker files (`#478 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/478>`_)
  This has been migrated inside the docs and is not needed anymore.
* Remove duplicated update_rate parameter (`#479 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/479>`_)
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
* Made sure all past maintainers are listed as authors (`#429 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/429>`_)
* Silence a compilation warning (`#425 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/425>`_)
  Since setting the receive timeout takes the time_buffer as an argument
  this raises a "may be used uninitialized" warning. Setting this to 0
  explicitly should prevent that.
* Doc: Fix IP address in usage->ursim section (`#422 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/422>`_)
* Contributors: Felix Exner

2.2.1 (2022-06-27)
------------------
* Fixed controller name for force_torque_sensor_broadcaster (`#411 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/411>`_)
* Contributors: Felix Exner

2.2.0 (2022-06-20)
------------------
* Updated package maintainers
* Rework bringup (`#403 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/403>`_)
* Prepare for humble (`#394 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/394>`_)
* Update dependencies on all packages (`#391 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/391>`_)
* Update HW-interface API for humble. (`#377 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/377>`_)
* Use types in hardware interface from ros2_control in local namespace (`#339 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/339>`_)
* Update header extension to remove compile warning. (`#285 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/285>`_)
* Add resource files from ROS World. (`#226 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/226>`_)
* Add sphinx documentation (`#340 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/340>`_)
* Update license to BSD-3-Clause (`#277 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/277>`_)
* Update ROS_INTERFACE.md to current driver (`#335 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/335>`_)
* Fix hardware interface names in error output (`#329 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/329>`_)
* Added controller stopper node (`#309 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/309>`_)
* Correct link to calibration extraction (`#310 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/310>`_)
* Start the tool communication script if the flag is set (`#267 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/267>`_)
* Change driver constructor and change calibration check (`#282 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/282>`_)
* Use GPIO tag from URDF in driver. (`#224 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/224>`_)
* Separate control node (`#281 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/281>`_)
* Add missing dependency on angles and update formatting for linters. (`#283 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/283>`_)
* Do not print an error output if writing is not possible (`#266 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/266>`_)
* Update features.md (`#250 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/250>`_)
* Tool communication (`#218 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/218>`_)
* Payload service (`#238 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/238>`_)
* Import transformation of force-torque into tcp frame from ROS1 driver (https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/src/hardware_interface.cpp). (`#237 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/237>`_)
* Make reading and writing work when hardware is disconnected (`#233 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/233>`_)
* Add missing command and state interfaces to get everything working with the fake hardware and add some comment into xacro file to be clearer. (`#221 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/221>`_)
* Decrease the rate of async tasks. (`#223 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/223>`_)
* Change robot type. (`#220 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/220>`_)
* Driver to headless. (`#217 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/217>`_)
* Test execution tests (`#216 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/216>`_)
* Integration tests improvement (`#206 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/206>`_)
* Set start modes to empty. Avoid position ctrl loop on start. (`#211 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/211>`_)
* Add resend program service and enable headless mode (`#198 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/198>`_)
* Implement "choices" for robot_type param (`#204 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/204>`_)
* Calibration extraction package (`#186 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/186>`_)
* Add breaking api changes from ros2_control to hardware_interface (`#189 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/189>`_)
* Fix prepare and perform switch operation (`#191 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/191>`_)
* Update CI configuration to support galactic and rolling (`#142 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/142>`_)
* Dockerize ursim with driver in docker compose (`#144 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/144>`_)
* Enabling velocity mode (`#146 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/146>`_)
* Moved registering publisher and service to on_active (`#151 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/151>`_)
* Converted io_test and switch_on_test to ROS2 (`#124 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/124>`_)
* Added loghandler to handle log messages from the Client Library with … (`#126 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/126>`_)
* Removed dashboard client from hardware interface
* [WIP] Updated feature list (`#102 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/102>`_)
* Moved Async check out of script running check (`#112 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/112>`_)
* Fix gpio controller (`#103 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/103>`_)
* Fixed speed slider service call (`#100 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/100>`_)
* Adding missing backslash and only setting workdir once (`#108 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/108>`_)
* Added dockerfile for the driver (`#105 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/105>`_)
* Using official Universal Robot Client Library (`#101 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/101>`_)
* Reintegrating missing ur_client_library dependency since the break the building process (`#97 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/97>`_)
* Fix readme hardware setup (`#91 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/91>`_)
* Fix move to home bug (`#92 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/92>`_)
* Using modern python
* Some intermediate commit
* Remove obsolete and unused files and packages. (`#80 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/80>`_)
* Review CI by correcting the configurations (`#71 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/71>`_)
* Add support for gpios, update MoveIt and ros2_control launching (`#66 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/66>`_)
* Quickfix against move home bug
* Added missing initialization
* Use GitHub Actions, use pre-commit formatting (`#56 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/56>`_)
* Put dashboard services into corresponding namespace
* Start dashboard client from within the hardware interface
* Added try catch blocks for service calls
* Removed repeated declaration of timeout parameter which lead to connection crash
* Removed static service name in which all auto generated services where mapped
* Removed unused variable
* Fixed clang-format issue
* Removed all robot status stuff
* Exchanged hardcoded value for RobotState msgs enum
* Removed currently unused controller state variables
* Added placeholder for industrial_robot_status_interface
* Fixed clang issues
* Added checks for internal robot state machine
* Only load speed scaling interface
* Changed state interface to combined speed scaling factor
* Added missing formatting in hardware interface
* Initial version of the speed_scaling_state_controller
* Fix clang tidy in multiple pkgs.
* Clang tidy fix.
* Update force torque state controller.
* Prepare for testing.
* Fix decision breaker for position control. Make decision effect instantaneous.
* Use only position interface.
* Update hardware interface for ROS2 (`#8 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/8>`_)
* Update the dashboard client for ROS2 (`#5 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/5>`_)
* Hardware interface framework (`#3 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/3>`_)
* Add XML schema to all ``package.xml`` files
* Silence ``ament_lint_cmake`` errors
* Update packaging for ROS2
* Update package.xml files so ``ros2 pkg list`` shows all pkgs
* Clean out ur_robot_driver for initial ROS2 compilation
* Compile ur_dashboard_msgs for ROS2
* Delete all launch/config files with no UR5 relation
* Initial work toward compiling ur_robot_driver
* Update CMakeLists and package.xml for:
  - ur5_moveit_config
  - ur_bringup
  - ur_description
* Change pkg versions to 0.0.0
* Contributors: AndyZe, Denis Stogl, Denis Štogl, Felix Exner, John Morris, Lovro, Mads Holm Peters, Marvin Große Besselmann, Rune Søe-Knudsen, livanov93, Robert Wilbrandt

0.0.3 (2019-08-09)
------------------
* Added a service to end ROS control from ROS side
* Publish IO state on ROS topics
* Added write channel through RTDE with speed slider and IO services
* Added subscriber to send arbitrary URScript commands to the robot

0.0.2 (2019-07-03)
------------------
* Fixed dependencies and installation
* Updated README
* Fixed passing parameters through launch files
* Added support for correctly switching controllers during runtime and using the standard
  joint_trajectory_controller
* Updated externalcontrol URCap to version 1.0.2
  + Fixed Script timeout when running the URCap inside of a looping tree
  + Fixed a couple of typos
* Increased minimal required UR software version to 3.7/5.1

0.0.1 (2019-06-28)
------------------
Initial release
