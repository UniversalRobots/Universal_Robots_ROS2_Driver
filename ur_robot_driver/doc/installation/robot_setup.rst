:github_url: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/main/ur_robot_driver/doc/installation/robot_setup.rst

Setting up a UR robot for ur_robot_driver
=========================================

Prepare robot and network connection
------------------------------------

Before you can use the ``ur_robot_driver`` you need to prepare the robot and the network
connection as described in the :ref:`robot_setup`  and :ref:`network_setup` section of the UR Client Library documentation.

Running the driver on WSL2 (Windows)
------------------------------------

By default WSL2 uses NAT networking: the WSL2 instance gets its own IP address in a private
subnet that is not reachable from the robot. This is a problem because the robot controller
actively connects back to the driver on several ports:

* the *reverse interface* (default ``50001``), through which the driver sends cyclic instructions
  to the robot controller;
* the *script sender* interface (default ``50002``), where the driver offers the
  ``external_control`` URScript for the robot to query;
* the *trajectory* port (default ``50003``), where the robot streams trajectory data.

Because the robot initiates these connections, the robot must be able to reach the ROS PC on
those ports. There are two ways to make that work.

Option A – mirrored networking (recommended)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Recent WSL2 versions support *mirrored* networking, in which the WSL2 instance shares the Windows
host's network interfaces and IP address. No port forwarding is needed and the driver can keep
using its automatically detected IP address.

1. Create or edit ``%UserProfile%\.wslconfig`` and add::

      [wsl2]
      networkingMode=mirrored

2. Restart WSL (run ``wsl --shutdown`` in PowerShell/CMD, then reopen the distro).
3. Make sure Windows and the robot are on the same subnet and that the Windows firewall allows
   inbound traffic on ports ``50001``, ``50002`` and ``50003``.
4. Start the driver as usual (see :ref:`ur_robot_driver_startup`), pointing ``robot_ip`` at the
   robot.

Option B – NAT with port forwarding
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you cannot use mirrored networking, keep the default NAT mode and forward the inbound ports
from the Windows host to the WSL2 instance.

1. Find the WSL2 instance's IP address::

      wsl hostname -I

2. Forward the inbound ports from an elevated PowerShell::

      netsh interface portproxy add v4tov4 listenport=50001 listenaddress=0.0.0.0 connectport=50001 connectaddress=<WSL_IP>
      netsh interface portproxy add v4tov4 listenport=50002 listenaddress=0.0.0.0 connectport=50002 connectaddress=<WSL_IP>
      netsh interface portproxy add v4tov4 listenport=50003 listenaddress=0.0.0.0 connectport=50003 connectaddress=<WSL_IP>

3. Add matching inbound Windows firewall rules for those ports.
4. The WSL2 IP address changes after every restart, so the driver must not advertise that address
   to the robot. Instead, tell the robot to connect back to the Windows host's LAN IP using the
   ``reverse_ip`` argument::

      ros2 launch ur_robot_driver ur_control.launch.py \
        ur_type:=<UR_TYPE> robot_ip:=<IP_OF_THE_ROBOT> \
        reverse_ip:=<WINDOWS_HOST_LAN_IP>

5. In the URCap program on the robot, use the Windows host's LAN IP (not the WSL2 IP). Windows
   and the robot must be on the same subnet.

.. note::
   If the robot drops the connection after starting a program, increasing the robot's keepalive
   counter can help. See the UR Client Library documentation and the discussion in
   `this issue <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/507#issuecomment-1028128431>`_.

Prepare the ROS PC
------------------

For using the driver make sure it is installed (either by the debian package or built from source
inside a colcon workspace).

.. _calibration_extraction:

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

.. note::
   The robot must be powered on (can be idle) before executing this script.


For the parameter ``robot_ip`` insert the IP address on which the ROS pc can reach the robot. As
``target_filename`` provide an absolute path where the result will be saved to.

See :ref:`ur_robot_driver_startup` for instructions on using the extracted calibration information.
