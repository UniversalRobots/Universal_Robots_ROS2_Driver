ur_robot_driver
^^^^^^^^^^^^^^^

keep_alive_count -> robot_receive_timeout
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Doing real-time control with a robot requires the control pc to conform to certain timing
constraints. For this ``keep_alive_count`` was used to estimate the tolerance that was given to the
ROS controller in terms of multiples of 20 ms. Now the timeout is directly configured using the
``robot_receive_timeout`` parameter of the hardware interface.
