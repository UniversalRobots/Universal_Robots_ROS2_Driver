:github_url: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/main/ur_robot_driver/doc/migration/makoa.rst

ur_robot_driver
^^^^^^^^^^^^^^^

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
