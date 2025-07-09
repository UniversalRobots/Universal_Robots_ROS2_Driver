:github_url: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/main/ur_robot_driver/doc/usage/simulation.rst

Simulation
==========

Apart from being used with a real robot, the ROS driver can be used with ros2_control's mock hardware or the URSim simulator (which is equivalent from the driver's perspective).

Additionally, the robot can be simulated using
`Gazebo Classic <https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation>`_ or
:ref:`GZ Sim <ur_simulation_gz>` but that's
outside of this driver's scope.

.. _usage_with_official_ur_simulator:

Usage with official UR simulator
--------------------------------

The easiest way to use URSim is the Docker
image provided by Universal Robots. There is an image for each software generation:

- `PolyScope 5 <https://hub.docker.com/r/universalrobots/ursim_e-series>`_
- `PolyScope X <https://hub.docker.com/r/universalrobots/ursim_polyscopex>`_
- `CB3 <https://hub.docker.com/r/universalrobots/ursim_cb3>`_

We have prepared a script to unify the startup of the simulator independent of the software
platform version:

.. code-block:: bash

   ros2 run ur_client_library start_ursim.sh -m <ur_type> -v <ursim_version>

If you skip the ``-v`` option, the script will use the latest version of the PolyScope 5 URSim
image. If you skip the ``-m`` option, the robot model will default to a UR5(e) robot.

**Example:**

.. code-block:: console

   $ ros2 run ur_client_library start_ursim.sh -v 10.8.0 -m ur20
   ROBOT_MODEL: ur20
   ROBOT_SERIES: polyscopex
   URSIM_VERSION: 10.8.0
   5b140a83f8600c7ada0b7d75bc7a5808667adbeec77387e8c73b093f10fd2379
   Starting URSim. Waiting for UrService to be up...................................
   UrService is up
   Installing URCapX /home/feex/.ursim/polyscopex/urcaps/external-control-0.1.0.urcapx


   To access PolyScopeX, open the following URL in a web browser.


           http://192.168.56.101

   To exit, press CTRL+C


Accessing the URSim GUI is done using a web browser. The script will print the URL to access the
GUI. Depending on the PolyScope version, this is different.

.. tabs::

   .. group-tab:: PolyScope 5 / CB3

       Open http://192.168.56.101:6080/vnc.html in a web browser.

   .. group-tab:: PolyScope X

       Open http://192.168.56.101 in a web browser.

With this, we can spin up a driver using

.. code-block:: bash

   ros2 launch ur_robot_driver ur_control.launch.py ur_type:=<ur_type> robot_ip:=192.168.56.101 launch_rviz:=true

When we now move the robot in Polyscope, the robot's RViz visualization should move accordingly.

For details on the (PolyScopr 5 and CB3) Docker image, please see the more detailed guide :ref:`here <ursim_docker>`.

Mock hardware
-------------

The package can simulate hardware with the ros2_control ``MockSystem``. This emulator enables an
environment for testing of "piping" of hardware and controllers, as well as testing robot's
descriptions. For more details see `ros2_control documentation
<https://control.ros.org/rolling/doc/ros2_control/hardware_interface/doc/mock_components_userdoc.html>`_
for more details.

.. note::
   Some driver functionalities currently don't work with mock hardware:

   * The TCP pose broadcaster does not work.
   * The passthrough trajectory controller does not function when calling the follow joint trajectory action.
   * The force mode controller also does not respond when trying to start force mode.
   * The GPIO controller cannot verify that it has changed the state of an I/O pin, so it will report a failure when trying to set an I/O pin.
