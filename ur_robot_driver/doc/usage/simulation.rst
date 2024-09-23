Simulation
==========

As mentioned in the last section the driver has two basic modes of operation: Using mock hardware or
using real hardware (Or the URSim simulator, which is equivalent from the driver's perspective).
Additionally, the robot can be simulated using
`Gazebo <https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation>`_ or
`ignition <https://github.com/UniversalRobots/Universal_Robots_ROS2_Ignition_Simulation>`_ but that's
outside of this driver's scope.

.. _usage_with_official_ur_simulator:

Usage with official UR simulator
--------------------------------

The easiest way to use URSim is the `Docker
image <https://hub.docker.com/r/universalrobots/ursim_e-series>`_ provided by Universal Robots (See
`this link <https://hub.docker.com/r/universalrobots/ursim_cb3>`_ for a CB3-series image).

To start it, we've prepared a script:

.. code-block:: bash

   ros2 run ur_client_library start_ursim.sh -m <ur_type>

With this, we can spin up a driver using

.. code-block:: bash

   ros2 launch ur_robot_driver ur_control.launch.py ur_type:=<ur_type> robot_ip:=192.168.56.101 launch_rviz:=true

You can view the polyscope GUI by opening `<http://192.168.56.101:6080/vnc.html>`_.

When we now move the robot in Polyscope, the robot's RViz visualization should move accordingly.

For details on the Docker image, please see the more detailed guide :ref:`here <ursim_docker>`.
