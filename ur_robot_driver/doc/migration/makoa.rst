:github_url: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/main/ur_robot_driver/doc/migration/makoa.rst

ur_robot_driver
^^^^^^^^^^^^^^^

Blocking read enabled by default in launch files
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

From ROS Makoa onwards, the ``ur_robot_driver`` launch files have blocking read enabled by default.
If you want to disable this behavior, you can set the ``blocking_read`` parameter to ``false`` in your
launch file.

.. note::

   Blocking read might not be the desired behavior in multi-robot (or multi hardware-interface)
   setups, in which case you should create your own launch files anyway. This is why in the URDF
   the default value of the ``blocking_read`` parameter is still ``false``.

See :ref:`blocking_read` for more details on that feature.


Interface change of set_payload service
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The ``SetPayload`` service in ``ur_msgs`` has changed to use ``geometry_msgs.msg.Inertia`` as the
payload field. Thus, service calls to ``/io_and_status_controller/set_payload`` have to be updated

from

.. code::

   ros2 service call /io_and_status_controller/set_payload ur_msgs/srv/SetPayload \
     "{payload: {mass: 1.0, center_of_gravity: {x: 0.0, y: 0.0, z: 0.04}}}"


to

.. code::

   ros2 service call /io_and_status_controller/set_payload ur_msgs/srv/SetPayload \
     "{payload: {m: 1.0, com: {x: 0.0, y: 0.0, z: 0.04}}}"

Payload inertia is now also supported, as well as a transition time:

.. code::

   ros2 service call /io_and_status_controller/set_payload ur_msgs/srv/SetPayload "
     payload:
       m: 1.0
       com:
         x: 0.0
         y: 0.0
         z: 0.04
       ixx: 0.05
       ixy: 0.02
       ixz: 0.02
       iyy: 0.05
       iyz: 0.02
       izz: 0.1
     transition_time:
       sec: 1
       nanosec: 0
   "
