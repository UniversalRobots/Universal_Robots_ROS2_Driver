2.10.0 (2025-10-23)
-------------------
* Add missing update_rate config files for UR7e and UR12e (backport of `#1544 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1544>`_) (`#1547 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1547>`_)
* Add support for UR18 (backport `#1524 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1524>`_) (`#1525 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1525>`_)
* Trajectory until node (backport of `#1461 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1461>`_) (`#1522 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1522>`_)
* Wait for used controllers in test setup (backport `#1519 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1519>`_) (`#1520 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1520>`_)
* Running integration tests with mock hardware (backport `#1226 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1226>`_) (`#1508 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1508>`_)
* Fix link to limits man page (`#1518 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1518>`_)
* Add test for hardware component lifecycle (backport `#1476 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1476>`_) (`#1506 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1506>`_)
* Contributors: Felix Exner, mergify[bot]

2.9.0 (2025-09-16)
------------------
* Add support for UR8 Long (`#1491 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1491>`_)
* Fix flaky controller switch test (backport of `#1447 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1447>`_) (`#1450 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1450>`_)
* fix_flaky_force_mode_test (backport of `#1429 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1429>`_) (`#1448 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1448>`_)
* Reduce flakiness of trajectory controller tests (backport of `#1443 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1443>`_) (`#1444 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1444>`_)
* Added 'is in remote control' call as a dashboard service (backport of `#1433 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1433>`_) (`#1436 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1436>`_)
* ur_robot_driver: Fix compilation on Windows (backport of `#1421 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1421>`_) (`#1431 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1431>`_)
  Co-authored-by: Silvio Traversaro <silvio@traversaro.it>
* Refactor prepare_switch method (backport `#1417 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1417>`_) (`#1427 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1427>`_)
* Contributors: Felix Exner, mergify[bot]

2.8.1 (2025-06-11)
------------------
* [force mode controller] Fix the task frame orientation (backport `#1379 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1379>`_) (`#1380 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1380>`_)
* Update feature list (backport of `#1372 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1372>`_) (`#1374 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1374>`_)
* Contributors: mergify[bot]

2.8.0 (2025-05-20)
------------------
* [CI] Check links using lychee instead of a custom script (backport `#1355 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1355>`_) (`#1361 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1361>`_)
* Add support for launching a UR15 robot (`#1359 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1359>`_)
* tool_contact_test: Check result status directly (backport of `#1345 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1345>`_) (`#1353 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1353>`_)
* Added controller to enable and disable tool contact (backport of `#940 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/940>`_) (`#1337 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1337>`_)
* Contributors: Felix Exner, mergify[bot]

2.7.0 (2025-04-10)
------------------
* Start executing passthrough trajectories earlier than all points are transferred. (backport of `#1313 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1313>`_) (`#1335 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1335>`_)
* Support PolyScopeX robots (backport of `#1318 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1318>`_) (`#1333 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1333>`_)
* Add support for UR7e and UR12e (`#1332 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1332>`_)
* Use UrDriverConfig struct to initialize UrDriver (backport of `#1328 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1328>`_) (`#1330 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1330>`_)
* Fix passthrough controller to not read non-existing state_interfaces (`#1314 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1314>`_) (`#1316 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1316>`_)
* Contributors: mergify[bot]

2.6.0 (2025-03-17)
------------------
* Update transformForceTorque to handle whether it is a cb3 or an e-Series robot (backport of `#1287 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1287>`_) (`#1299 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1299>`_)
* Port robot_state_helper to ROS2 (backport of  `#933 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/933>`_) (`#1286 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1286>`_)
* Fix crashes on shutting down (`#1270 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1270>`_) (`#1271 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1271>`_)
* Remove build warnings for Humble (`#1234 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1234>`_)
* Auto-update pre-commit hooks (backport `#1260 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1260>`_) (`#1261 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1261>`_)
* Fix doc links (`#1247 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1247>`_)
* Remove urdf folder (`#1257 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1257>`_)
* Contributors: Felix Exner, mergify[bot]

2.5.2 (2025-01-21)
------------------
* Check quaternions for equal dot_product instead of comparing their components individually (backport `#1238 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1238>`_) (`#1243 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1243>`_)
* fix sphinx doc link in ur_robot_driver (`#1240 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1240>`_) (`#1242 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1242>`_)
* Update pre-commit the same as on the main branch (`#1232 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1232>`_)
* Disable pose broadcaster on mock hardware (backport of `#1229 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1229>`_) (`#1230 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1230>`_)
* Remove unused include (backport of `#1220 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1220>`_)
  Co-authored-by: Bence Magyar <bence.magyar.robotics@gmail.com>
* Contributors: Felix Exner, mergify[bot]

2.5.1 (2024-12-21)
------------------

2.5.0 (2024-12-18)
------------------
* Freedrive Controller (`#1114 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1114>`_) (`#1211 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1211>`_)
* Add force mode controller (`#1049 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1049>`_) (`#1193 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1193>`_)
* Update package maintainers (backport of `#1203 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1203>`_)
* Forward trajectory controller (backport of `#944 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/944>`_)
* Use pose_broadcaster to publish the TCP pose (backport of `#1108 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1108>`_)
* Contributors: mergify[bot]

2.2.16 (2024-10-28)
-------------------
* Allow setting the analog output domain when setting an analog output (backport of `#1123 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1123>`_)
* Fix component lifecycle (backport of `#1098 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1098>`_)
* [moveit] Disable execution_duration_monitoring by default (`#1133 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1133>`_)
* Service to get software version of robot (backport of `#964 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/964>`_)
* Assure the description is loaded as string (backport of `#1107 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1107>`_)
* Contributors: Felix Exner (fexner), mergify[bot], Jacob Larsen

2.2.15 (2024-07-26)
-------------------
* Fix passing launch_dashobard_client launch argument (backport of `#1057 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1057>`_)
* Updated the UR family photo on the readme (backport of `#1064 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1064>`_)
* Contributors: Rune Søoe-Knudsen, Felix Exner

2.2.14 (2024-07-01)
-------------------
* Add sleep between controller stopper's controller queries (backport of `#1038 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1038>`_)
* Contributors: Felix Exner

2.2.13 (2024-06-17)
-------------------
* Use robot_receive_timeout instead of keepalive_count (`#1009 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1009>`_)
* Remove extra spaces from start_ursim statement in tests (backport of `#1010 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1010>`_)
* Add calibration file to launch arguments (`#1001 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1001>`_)
* Contributors: Vincenzo Di Pentima, Felix Exner

2.2.12 (2024-05-16)
-------------------
* Remove dependency to docker.io (backport `#985 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/985>`_)
* Simplify tests (backport of `#849 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/849>`_)
* Update installation instructions for source build (backport `#967 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/967>`_)
* Move installation instructions to subpage (backport `#870 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/870>`_)
* Reduce number of controller_spawners to 3 (`#928 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/928>`_)
* Fix multi-line strings in DeclareLaunchArgument (backport `#948 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/948>`_)
* "use_fake_hardware" for UR20 (`#950 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/950>`_)
* Contributors: Vincenzo Di Pentima, Felix Exner, Robert Wilbrandt, Matthijs van der Burgh

2.2.11 (2024-04-08)
-------------------
* Add UR30 support (`#930 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/930>`_)
* Move communication setup to on_configure instead of on_activate (`#936 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/936>`_)
* Contributors: Felix Exner, Vincenzo Di Pentima

2.2.10 (2024-01-03)
-------------------
* Add backward_ros to driver (`#872 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/872>`_) (`#878 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/878>`_)
* Port configuration  (`#835 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/835>`_) (`#847 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/847>`_)
* Update link to MoveIt! documentation (`#845 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/845>`_)
* Contributors: mergify[bot]

2.2.9 (2023-09-22)
------------------
* Added a test that sjtc correctly aborts on violation of constraints
* Added support for UR20 (`#805 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/805>`_)
* Introduced tf_prefix into log handler (`#713 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/713>`_)
* Start ursim from lib (`#733 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/733>`_)
* Run robot driver test also with tf_prefix (`#729 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/729>`_)
* Urscript interface (`#721 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/721>`_) (`#742 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/742>`_)
* Contributors: Felix Exner, Lennart Nachtigall, mergify[bot]

2.2.8 (2023-06-26)
------------------
* Use tf prefix properly (backport `#688 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/688>`_) (`#725 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/725>`_)
* Use SCHED_FIFO for controller_manager's main thread (`#719 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/719>`_) (`#722 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/722>`_)
* Contributors: mergify[bot]

2.2.7 (2023-06-02)
------------------
* Calling on_deactivate in dtr (`#679 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/679>`_) (`#704 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/704>`_)
* Adds full nonblocking readout support (Multiarm part 4)  - v2 (`#673 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/673>`_) (`#703 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/703>`_)
* Correct calibration correction launch file in doc (`#590 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/590>`_)
* Introduce hand back control service (`#528 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/528>`_) (`#670 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/670>`_)
* Update definition of test goals to new version. (backport `#637 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/637>`_) (`#668 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/668>`_)
* Default path to ur_client_library urscript (`#316 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/316>`_) (`#553 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/553>`_)
  * Change default path for urscript for headless mode.
  * Replace urscript path also in newer ur_robot_driver launchfile
* Wait longer for controllers to load and activate
* Fix flaky tests (`#641 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/641>`_)
* Added services to set tool voltage and zero force torque sensor (`#466 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/466>`_) (`#582 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/582>`_)
* Controller spawner timeout (backport `#608 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/608>`_) (`#609 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/609>`_)
* Fix cmake dependency on controller_manager (backport `#598 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/598>`_) (`#599 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/599>`_)
* Increase timeout for first test service call to driver (Backport of `#605 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/605>`_) (`#607 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/607>`_)
* Update linters & checkers (backport `#426 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/426>`_) (`#556 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/556>`_)
* Clean up & improve execution tests (Backport of `#512 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/512>`_) (`#552 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/552>`_)
* Contributors: Felix Exner (fexner), Lennart Nachtigall, Robert Wilbrandt, mergify[bot], Denis Stogl, livanov93, Mads Holm Peters

2.2.6 (2022-11-28)
------------------
* Cleanup humble branch (`#545 <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/545>`_)
* Contributors: Felix Exner (fexner)

2.2.5 (2022-11-19)
------------------
* ur_robot_driver: Controller_stopper fix deprecation warning
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
* Contributors: Felix Exner, Felix Exner (fexner)

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
