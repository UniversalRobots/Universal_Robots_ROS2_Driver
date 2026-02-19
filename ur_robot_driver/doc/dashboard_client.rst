:github_url: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/main/ur_robot_driver/doc/dashboard_client.rst

.. _dashboard_client_ros2:

Dashboard client
================

Advertised Services
-------------------

add_to_log (`ur_dashboard_msgs/AddToLog <http://docs.ros.org/en/rolling/p/ur_dashboard_msgs/srv/AddToLog.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Service to add a message to the robot's log

brake_release (`std_srvs/Trigger <http://docs.ros.org/en/rolling/p/std_srvs/srv/Trigger.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Service to release the brakes. If the robot is currently powered off, it will get powered on the fly.

clear_operational_mode (`std_srvs/Trigger <http://docs.ros.org/en/rolling/p/std_srvs/srv/Trigger.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If this service is called the operational mode can again be changed from PolyScope, and the user password is enabled.

close_popup (`std_srvs/Trigger <http://docs.ros.org/en/rolling/p/std_srvs/srv/Trigger.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Close a (non-safety) popup on the teach pendant.

close_safety_popup (`std_srvs/Trigger <http://docs.ros.org/en/rolling/p/std_srvs/srv/Trigger.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Close a safety popup on the teach pendant.

connect (`std_srvs/Trigger <http://docs.ros.org/en/rolling/p/std_srvs/srv/Trigger.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Service to reconnect to the dashboard server

download_program (`ur_dashboard_msgs/DownloadProgram <http://docs.ros.org/en/rolling/p/ur_dashboard_msgs/srv/DownloadProgram.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**PolyScope X only**: Download a program with the given name from the robot and save it as
``*.urpx`` file locally. This service is only available on a PolyScope X robot with version >=
10.12.0.

get_loaded_program (`ur_dashboard_msgs/GetLoadedProgram <http://docs.ros.org/en/rolling/p/ur_dashboard_msgs/srv/GetLoadedProgram.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Get the name of the currently loaded program.

get_programs (`ur_dashboard_msgs/GetPrograms <http://docs.ros.org/en/rolling/p/ur_dashboard_msgs/srv/GetPrograms.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**PolyScope X only**: Get a list of all programs on the robot. This service is only available on a
PolyScope X robot with version >= 10.12.0.

get_robot_mode (`ur_dashboard_msgs/GetRobotMode <http://docs.ros.org/en/rolling/p/ur_dashboard_msgs/srv/GetRobotMode.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Service to query the current robot mode

get_safety_mode (`ur_dashboard_msgs/GetSafetyMode <http://docs.ros.org/en/rolling/p/ur_dashboard_msgs/srv/GetSafetyMode.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Service to query the current safety mode

is_in_remote_control (`ur_dashboard_msgs/IsInRemoteControl <http://docs.ros.org/en/rolling/p/ur_dashboard_msgs/srv/IsInRemoteControl.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Service to query whether the robot is in remote control

load_installation (`ur_dashboard_msgs/Load <http://docs.ros.org/en/rolling/p/ur_dashboard_msgs/srv/Load.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Load a robot installation from a file

load_program (`ur_dashboard_msgs/Load <http://docs.ros.org/en/rolling/p/ur_dashboard_msgs/srv/Load.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Load a robot program from a file on the robot. On PolyScope 5, this can be either file in the
robot's programs folder or an absolute path to a file. The filename has to include the ``.urp``
ending. On PolyScope X, this has to be the program's name as shown in the Programs overview on the
robot, without a ``.urpx`` suffix.

pause (`std_srvs/Trigger <http://docs.ros.org/en/rolling/p/std_srvs/srv/Trigger.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Pause a running program.

play (`std_srvs/Trigger <http://docs.ros.org/en/rolling/p/std_srvs/srv/Trigger.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Start execution of a previously loaded program

popup (`ur_dashboard_msgs/Popup <http://docs.ros.org/en/rolling/p/ur_dashboard_msgs/srv/Popup.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Service to show a popup on the UR Teach pendant.

power_off (`std_srvs/Trigger <http://docs.ros.org/en/rolling/p/std_srvs/srv/Trigger.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Power off the robot motors

power_on (`std_srvs/Trigger <http://docs.ros.org/en/rolling/p/std_srvs/srv/Trigger.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Power on the robot motors. To fully start the robot, call 'brake_release' afterwards.

program_running (`ur_dashboard_msgs/IsProgramRunning <http://docs.ros.org/en/rolling/p/ur_dashboard_msgs/srv/IsProgramRunning.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Query whether there is currently a program running

program_saved (`ur_dashboard_msgs/IsProgramSaved <http://docs.ros.org/en/rolling/p/ur_dashboard_msgs/srv/IsProgramSaved.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Query whether the current program is saved

program_state (`ur_dashboard_msgs/GetProgramState <http://docs.ros.org/en/rolling/p/ur_dashboard_msgs/srv/GetProgramState.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Service to query the current program state

quit (`ur_dashboard_msgs/GetLoadedProgram <http://docs.ros.org/en/rolling/p/ur_dashboard_msgs/srv/GetLoadedProgram.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Disconnect from the dashboard service.

raw_request (`ur_dashboard_msgs/RawRequest <http://docs.ros.org/en/rolling/p/ur_dashboard_msgs/srv/RawRequest.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

General purpose service to send arbitrary messages to the dashboard server

restart_safety (`std_srvs/Trigger <http://docs.ros.org/en/rolling/p/std_srvs/srv/Trigger.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Used when robot gets a safety fault or violation to restart the safety. After safety has been rebooted the robot will be in Power Off. NOTE: You should always ensure it is okay to restart the system. It is highly recommended to check the error log before using this command (either via PolyScope or e.g. ssh connection).

resume (`std_srvs/Trigger <http://docs.ros.org/en/rolling/p/std_srvs/srv/Trigger.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**PolyScope X only**: Resume a paused program on a PolyScope X robot. This service is only available on a PolyScope X
robot with version >= 10.11.0.

shutdown (`std_srvs/Trigger <http://docs.ros.org/en/rolling/p/std_srvs/srv/Trigger.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Shutdown the robot controller

stop (`std_srvs/Trigger <http://docs.ros.org/en/rolling/p/std_srvs/srv/Trigger.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Stop program execution on the robot

unlock_protective_stop (`std_srvs/Trigger <http://docs.ros.org/en/rolling/p/std_srvs/srv/Trigger.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Dismiss a protective stop to continue robot movements. NOTE: It is the responsibility of the user to ensure the cause of the protective stop is resolved before calling this service.


update_program (`ur_dashboard_msgs/UpdateProgram <http://docs.ros.org/en/rolling/p/ur_dashboard_msgs/srv/UpdateProgram.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**PolyScope X only**: Update a program on the robot. If the program does not exist or the program
is currently loaded, this service call will fail. This service is only available on a PolyScope X
robot with version >= 10.12.0.

upload_program (`ur_dashboard_msgs/UploadProgram <http://docs.ros.org/en/rolling/p/ur_dashboard_msgs/srv/UploadProgram.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**PolyScope X only**: Upload a program to the robot. If a program with the same name already
exists, this service call will fail. This service is only available on a PolyScope X robot with
version >= 10.12.0.

get_polyscope_version (`ur_dashboard_msgs/GetPolyScopeVersion <http://docs.ros.org/en/rolling/p/ur_dashboard_msgs/srv/GetPolyScopeVersion.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**CB3 | PolyScope 5** Get polyScope version of robot

get_serial_number (`ur_dashboard_msgs/GetSerialNumber <http://docs.ros.org/en/rolling/p/ur_dashboard_msgs/srv/GetSerialNumber.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**CB3 | PolyScope 5** Get serial number of robot

get_user_role (`ur_dashboard_msgs/GetUserRole <http://docs.ros.org/en/rolling/p/ur_dashboard_msgs/srv/GetUserRole.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**CB3 only** Get current user role

set_user_role (`ur_dashboard_msgs/SetUserRole <http://docs.ros.org/en/rolling/p/ur_dashboard_msgs/srv/SetUserRole.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**CB3 only** Set user role on the robot

get_operational_mode (`ur_dashboard_msgs/GetOperationalMode <http://docs.ros.org/en/rolling/p/ur_dashboard_msgs/srv/GetOperationalMode.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**PolyScope 5 | PolyScope X 10.12.0 onwards** Get current operational mode of the robot

set_operational_mode (`ur_dashboard_msgs/SetOperationalMode <http://docs.ros.org/en/rolling/p/ur_dashboard_msgs/srv/SetOperationalMode.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**PolyScope 5 only** Set operational mode of the robot. When this has been called, the teach pendant can not be used to change the operational mode until clear_operational_mode has been called.

get_robot_model (`ur_dashboard_msgs/GetRobotModel <http://docs.ros.org/en/rolling/p/ur_dashboard_msgs/srv/GetRobotModel.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**CB3 | PolyScope 5** Get the robot model, in the format URx. It should be noted this call does not differentiate between e-series and CB3, so UR5 and UR5e will both report as UR5

get_safety_status (`ur_dashboard_msgs/GetSafetyStatus <http://docs.ros.org/en/rolling/p/ur_dashboard_msgs/srv/GetSafetyStatus.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**PolyScope 5 only** Get current safety status of the robot system, this is more detailed than get_safety_mode and should be preferred when possible

generate_flight_report (`ur_dashboard_msgs/GenerateFlightReport <http://docs.ros.org/en/rolling/p/ur_dashboard_msgs/srv/GenerateFlightReport.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**CB3 | PolyScope 5** Generate flight report of the chosen type, defaults to SYSTEM

generate_support_file (`ur_dashboard_msgs/GenerateSupportFile <http://docs.ros.org/en/rolling/p/ur_dashboard_msgs/srv/GenerateSupportFile.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**CB3 | PolyScope 5** Generate a support file a the specified location. Location is relative to the programs folder, if saving to a subfolder it must exist prior to the service call.
Defaults to saving to the programs folder


Parameters
----------

receive_timeout (default: 20.0)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Timeout (in seconds, double) after which a call to the dashboard server will be considered failure if no answer has been received. This defaults to 20 seconds, as some operations (like loading larger programs) can take a while.

robot_ip (Required)
^^^^^^^^^^^^^^^^^^^

The IP address under which the robot is reachable.
