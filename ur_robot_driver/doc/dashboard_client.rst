.. _dashboard_client:

Dashboard client
================

Advertised Services
-------------------

add_to_log (`ur_dashboard_msgs/AddToLog <http://docs.ros.org/en/rolling/p/ur_dashboard_msgs/interfaces/srv/AddToLog.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Service to add a message to the robot's log

brake_release (`std_srvs/Trigger <http://docs.ros.org/en/rolling/p/std_srvs/interfaces/srv/Trigger.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Service to release the brakes. If the robot is currently powered off, it will get powered on the fly.

clear_operational_mode (`std_srvs/Trigger <http://docs.ros.org/en/rolling/p/std_srvs/interfaces/srv/Trigger.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If this service is called the operational mode can again be changed from PolyScope, and the user password is enabled.

close_popup (`std_srvs/Trigger <http://docs.ros.org/en/rolling/p/std_srvs/interfaces/srv/Trigger.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Close a (non-safety) popup on the teach pendant.

close_safety_popup (`std_srvs/Trigger <http://docs.ros.org/en/rolling/p/std_srvs/interfaces/srv/Trigger.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Close a safety popup on the teach pendant.

connect (`std_srvs/Trigger <http://docs.ros.org/en/rolling/p/std_srvs/interfaces/srv/Trigger.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Service to reconnect to the dashboard server

get_loaded_program (`ur_dashboard_msgs/GetLoadedProgram <http://docs.ros.org/en/rolling/p/ur_dashboard_msgs/interfaces/srv/GetLoadedProgram.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Load a robot installation from a file

get_robot_mode (`ur_dashboard_msgs/GetRobotMode <http://docs.ros.org/en/rolling/p/ur_dashboard_msgs/interfaces/srv/GetRobotMode.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Service to query the current robot mode

get_safety_mode (`ur_dashboard_msgs/GetSafetyMode <http://docs.ros.org/en/rolling/p/ur_dashboard_msgs/interfaces/srv/GetSafetyMode.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Service to query the current safety mode

load_installation (`ur_dashboard_msgs/Load <http://docs.ros.org/en/rolling/p/ur_dashboard_msgs/interfaces/srv/Load.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Load a robot installation from a file

load_program (`ur_dashboard_msgs/Load <http://docs.ros.org/en/rolling/p/ur_dashboard_msgs/interfaces/srv/Load.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Load a robot program from a file

pause (`std_srvs/Trigger <http://docs.ros.org/en/rolling/p/std_srvs/interfaces/srv/Trigger.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Pause a running program.

play (`std_srvs/Trigger <http://docs.ros.org/en/rolling/p/std_srvs/interfaces/srv/Trigger.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Start execution of a previously loaded program

popup (`ur_dashboard_msgs/Popup <http://docs.ros.org/en/rolling/p/ur_dashboard_msgs/interfaces/srv/Popup.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Service to show a popup on the UR Teach pendant.

power_off (`std_srvs/Trigger <http://docs.ros.org/en/rolling/p/std_srvs/interfaces/srv/Trigger.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Power off the robot motors

power_on (`std_srvs/Trigger <http://docs.ros.org/en/rolling/p/std_srvs/interfaces/srv/Trigger.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Power on the robot motors. To fully start the robot, call 'brake_release' afterwards.

program_running (`ur_dashboard_msgs/IsProgramRunning <http://docs.ros.org/en/rolling/p/ur_dashboard_msgs/interfaces/srv/IsProgramRunning.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Query whether there is currently a program running

program_saved (`ur_dashboard_msgs/IsProgramSaved <http://docs.ros.org/en/rolling/p/ur_dashboard_msgs/interfaces/srv/IsProgramSaved.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Query whether the current program is saved

program_state (`ur_dashboard_msgs/GetProgramState <http://docs.ros.org/en/rolling/p/ur_dashboard_msgs/interfaces/srv/GetProgramState.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Service to query the current program state

quit (`ur_dashboard_msgs/GetLoadedProgram <http://docs.ros.org/en/rolling/p/ur_dashboard_msgs/interfaces/srv/GetLoadedProgram.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Disconnect from the dashboard service.

raw_request (`ur_dashboard_msgs/RawRequest <http://docs.ros.org/en/rolling/p/ur_dashboard_msgs/interfaces/srv/RawRequest.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

General purpose service to send arbitrary messages to the dashboard server

restart_safety (`std_srvs/Trigger <http://docs.ros.org/en/rolling/p/std_srvs/interfaces/srv/Trigger.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Used when robot gets a safety fault or violation to restart the safety. After safety has been rebooted the robot will be in Power Off. NOTE: You should always ensure it is okay to restart the system. It is highly recommended to check the error log before using this command (either via PolyScope or e.g. ssh connection).

shutdown (`std_srvs/Trigger <http://docs.ros.org/en/rolling/p/std_srvs/interfaces/srv/Trigger.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Shutdown the robot controller

stop (`std_srvs/Trigger <http://docs.ros.org/en/rolling/p/std_srvs/interfaces/srv/Trigger.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Stop program execution on the robot

unlock_protective_stop (`std_srvs/Trigger <http://docs.ros.org/en/rolling/p/std_srvs/interfaces/srv/Trigger.html>`_)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Dismiss a protective stop to continue robot movements. NOTE: It is the responsibility of the user to ensure the cause of the protective stop is resolved before calling this service.

Parameters
----------

receive_timeout (Required)
^^^^^^^^^^^^^^^^^^^^^^^^^^

Timeout (in seconds, double) after which a call to the dashboard server will be considered failure if no answer has been received.

robot_ip (Required)
^^^^^^^^^^^^^^^^^^^

The IP address under which the robot is reachable.
