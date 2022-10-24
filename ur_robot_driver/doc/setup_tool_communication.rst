.. _setup-tool-communication:

Setting up the tool communication on an e-Series robot
======================================================

.. note::
   Currently, there seems to be issues with having the tool communication setup from this guide and
   the "Robotiq Grippers" URCap running in parallel. If you are planning to use the "Robotiq
   Grippers" URCap then currently please refrain from installing the rs485 URCap.

   See
   `this issue <https://github.com/UniversalRobots/Universal_Robots_ToolComm_Forwarder_URCap/issues/9>`_
   for details and the current state.

The Universal Robots e-Series provides an rs485 based interface at the tool flange that can be used
to attach an rs485-based device to the robot's tcp without the need to wire a separate cable along
the robot.

This driver enables forwarding this tool communication interface to an external machine for example
to start a device's ROS driver on a remote PC.

This document will guide you through installing the URCap needed for this and setting up your ROS
launch files to utilize the robot's tool communication.

Robot setup
-----------

For setting up the robot, please install the **rs485-1.0.urcap** found in the **resources** folder.
Installing a URCap is explained in the :ref:`setup guide <install-urcap-e-series>` for the **external-control** URCap.

After installing the URCap the robot will expose its tool communication device to the network.

Setup the ROS side
------------------

In order to use the tool communication in ROS, simply pass the correct parameters to the bringup
launch files:

.. code-block:: bash

   $ ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=yyy.yyy.yyy.yyy use_tool_communication:=true use_fake_hardware:=false launch_rviz:=false
     # remember that your user needs to have the rights to write that file handle to /tmp/ttyUR

Following parameters can be set `ur.ros2_control.xacro <https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/blob/ros2/urdf/ur.ros2_control.xacro>`_\ :


* ``tool_voltage``
* ``tool_parity``
* ``tool_baud_rate``
* ``tool_stop_bits``
* ``tool_rx_idle_chars``
* ``tool_tx_idle_chars``
* ``tool_device_name``
* ``tool_tcp_port``

The ``tool_device_name`` is an arbitrary name for the device file at which the device will be
accessible in the local file system. Most ROS drivers for rs485 devices accept an argument to
specify the device file path. With the example above you could run the ``rs485_node`` from the package
``imaginary_drivers`` using the following command:

.. code-block:: bash

   $ ros2 run imaginary_drivers rs485_node --ros-args -p device:=/tmp/ttyUR

You can basically choose any device name, but your user has to have the correct rights to actually
create a new file handle inside this directory. Therefore, we didn't use the ``/dev`` folder in the
example, as users usually don't have the access rights to create new files there.

For all the other tool parameters seen above, please refer to the Universal Robots user manual.

More information can be found at `Universal_Robots_ToolComm_Forwarder_URCap <https://github.com/UniversalRobots/Universal_Robots_ToolComm_Forwarder_URCap>`_.
The convenience script for ``socat`` call is `here <https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/main/ur_robot_driver/scripts/tool_communication.py>`_ for ROS2 driver and can be run by:

.. code-block:: bash

   $ ros2 run ur_robot_driver tool_communication.py --ros-args -p robot_ip:=192.168.56.101 -p device_name:=/tmp/ttyUR
