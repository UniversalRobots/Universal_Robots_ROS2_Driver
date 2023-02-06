
Setting up a UR robot for ur_robot_driver
=========================================

Network setup
-------------

There are many possible ways to connect a UR robot. This section describes a good example using static IP addresses and a direct connection from the PC to the Robot to minimize latency introduced by network hardware. Though a good network switch usually works fine, as well.


#.
   Connect the UR control box directly to the remote PC with an ethernet cable.

#.
   Open the network settings from the UR teach pendant (Setup Robot -> Network) and enter these settings:

.. code-block::

   IP address: 192.168.1.102
   Subnet mask: 255.255.255.0
   Default gateway: 192.168.1.1
   Preferred DNS server: 192.168.1.1
   Alternative DNS server: 0.0.0.0


#.
   On the remote PC, turn off all network devices except the "wired connection", e.g. turn off wifi.

#.
   Open Network Settings and create a new Wired connection with these settings. You may want to name this new connection ``UR`` or something similar:

.. code-block::

   IPv4
   Manual
   Address: 192.168.1.101
   Netmask: 255.255.255.0
   Gateway: 192.168.1.1


#. Verify the connection from the PC with e.g. ping.

.. code-block::

   ping 192.168.1.102

Prepare the robot
-----------------

This section describes installation and launching of the URCap program from the pendant. It allows ROS to control the robot externally. Generally, you will launch the driver via ROS then start URCap from the pendant.

For using the *ur_robot_driver* with a real robot you need to install the
**externalcontrol urcap**. The latest release can be downloaded from `its own repository <https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases>`_.

**Note**: For installing this URCap a minimal PolyScope version of 3.7 or 5.1 (in case of e-Series) is
necessary.

For installing the necessary URCap and creating a program, please see the individual tutorials on
how to :ref:`setup a cb3 robot <install-urcap-cb3>` or how to
:ref:`setup an e-Series robot <install-urcap-e-series>`.

To setup the tool communication on an e-Series robot, please consider the :ref:`tool communication setup
guide <setup-tool-communication>`.

Prepare the ROS PC
------------------

For using the driver make sure it is installed (either by the debian package or built from source
inside a colcon workspace).

Extract calibration information
-------------------------------

Each UR robot is calibrated inside the factory giving exact forward and inverse kinematics. To also
make use of this in ROS, you first have to extract the calibration information from the robot.

Though this step is not necessary to control the robot using this driver, it is highly recommended
to do so, as otherwise endeffector positions might be off in the magnitude of centimeters.

For this, there exists a helper script:

.. code:: bash

   $ ros2 launch ur_calibration calibration_correction.launch.py \
   robot_ip:=<robot_ip> target_filename:="${HOME}/my_robot_calibration.yaml"

For the parameter ``robot_ip`` insert the IP address on which the ROS pc can reach the robot. As
``target_filename`` provide an absolute path where the result will be saved to.
