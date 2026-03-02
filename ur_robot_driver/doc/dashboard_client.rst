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

Parameters
----------

receive_timeout (default: 20.0)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Timeout (in seconds, double) after which a call to the dashboard server will be considered failure if no answer has been received. This defaults to 20 seconds, as some operations (like loading larger programs) can take a while.

robot_ip (Required)
^^^^^^^^^^^^^^^^^^^

The IP address under which the robot is reachable.
