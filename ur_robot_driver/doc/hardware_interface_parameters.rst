UR Hardware interface parameters
================================

Note that parameters are passed through the ros2_control xacro definition.

headless_mode
-------------

Start robot in headless mode. This does not require the 'External Control' URCap to be running on the robot, but this will send the URScript to the robot directly. On e-Series robots this requires the robot to run in 'remote-control' mode.

input_recipe_filename (Required)
--------------------------------

Path to the file containing the recipe used for requesting RTDE inputs.

kinematics/hash (Required)
--------------------------

Hash of the calibration reported by the robot. This is used for validating the robot description is
using the correct calibration. If the robot's calibration doesn't match this hash, an error will be
printed. You can use the robot as usual, however Cartesian poses of the endeffector might be
inaccurate. See the "ur_calibration" package on help how to generate your own hash matching your
actual robot.

non_blocking_read (default: "true")
-----------------------------------

If set to false, the ROS control cycle will wait for the robot to send a status update. Tests have
shown that better real-time performance is achievable when setting this to ``true``. Required to be
set to ``true`` when combining with other hardware components.

output_recipe_filename (Required)
---------------------------------

Path to the file containing the recipe used for requesting RTDE outputs.

reverse_port (Required)
-----------------------

Port that will be used by to communicate between the robot controller and the driver. This port needs to be free and will be opened on the host running the driver.

robot_ip (Required)
-------------------

The robot's IP address.

script_filename (Required)
--------------------------

Path to the urscript code that will be sent to the robot.

script_sender_port (Required)
-----------------------------

Network port which the driver provides the URScript program that needs to run on the robot. This number needs to be configured for the [External Control URCap](https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap).
This port needs to be free and will be opened on the host running the driver.

servoj_gain (Required)
----------------------

Specify the gain for the underlying servoj command. This will be used whenever position control is
active. A higher value will lead to sharper motions, but might also introduce
higher jerks and vibrations.

Range: [100 - 3000]

servoj_lookahead_time (Required)
--------------------------------

Specify lookahead_time parameter of underlying servoj command. This will be used whenever position
control is active. A higher value will result in smoother trajectories, but will also introduce a
higher delay between the commands sent from ROS and the motion being executed on the robot.

Unit: seconds, range: [0.03 - 0.2]

tool_baud_rate (Required)
-------------------------

Baud rate used for tool communication. Will be set as soon as the UR-Program on the robot is started. See UR documentation for valid baud rates.  Note: This parameter is only evaluated, when the parameter "use_tool_communication" is set to ``true``.  Then, this parameter is required.

This can also be configured using the robot teach pendant. Remember to save the installation on the robot to keep the setting after reboot.

tool_parity (Required)
----------------------

Parity used for tool communication. Will be set as soon as the UR-Program on the robot is started. Can be 0 (None), 1 (odd) and 2 (even).  Note: This parameter is only evaluated, when the parameter "use_tool_communication" is set to ``true``.  Then, this parameter is required.

This can also be configured using the robot teach pendant. Remember to save the installation on the robot to keep the setting after reboot.

tool_rx_idle_chars (Required)
-----------------------------

Number of idle chars for the RX unit used for tool communication. Will be set as soon as the UR-Program on the robot is started. Valid values: min=1.0, max=40.0  Note: This parameter is only evaluated, when the parameter "use_tool_communication" is set to ``true``.  Then, this parameter is required.

This can also be configured using the robot teach pendant. Remember to save the installation on the robot to keep the setting after reboot.

tool_stop_bits (Required)
-------------------------

Number of stop bits used for tool communication. Will be set as soon as the UR-Program on the robot is started. Can be 1 or 2.  Note: This parameter is only evaluated, when the parameter "use_tool_communication" is set to ``true``.  Then, this parameter is required.

This can also be configured using the robot teach pendant. Remember to save the installation on the robot to keep the setting after reboot.

tool_tx_idle_chars (Required)
-----------------------------

Number of idle chars for the TX unit used for tool communication. Will be set as soon as the UR-Program on the robot is started. Valid values: min=0.0, max=40.0  Note: This parameter is only evaluated, when the parameter "use_tool_communication" is set to ``true``.  Then, this parameter is required.

This can also be configured using the robot teach pendant. Remember to save the installation on the robot to keep the setting after reboot.

tool_voltage (Required)
-----------------------

Tool voltage that will be set as soon as the UR-Program on the robot is started. Note: This parameter is only evaluated, when the parameter "use_tool_communication" is set to ``true``. Then, this parameter is required.

This can also be configured using the robot teach pendant. Remember to save the installation on the robot to keep the setting after reboot.

use_tool_communication (Required)
---------------------------------

Should the tool's RS485 interface be forwarded to the ROS machine? This is only available on e-Series models. Setting this parameter to ``true`` requires multiple other parameters to be set as well.
For more info please see :ref:`tool communication setup guide <setup-tool-communication>`.
