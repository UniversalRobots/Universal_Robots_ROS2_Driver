#####
Usage
#####

This chapter explains how to use the ``ur_robot_driver``


.. toctree::
   :maxdepth: 4
   :caption: Contents:

   startup
   move
   controllers
   simulation
   script_code

Quick start
-----------

Driver startup
^^^^^^^^^^^^^^

* To start it with hardware, use:

  .. code-block:: console

     $ ros2 launch ur_robot_driver ur_control.launch.py ur_type:=<UR_TYPE> robot_ip:=<IP_OF_THE_ROBOT> launch_rviz:=true

  After starting the launch file start the external_control URCap program from the pendant, as described :ref:`here<robot_startup_program>`.

* To run it with URSim check details about it in :ref:`this section <usage_with_official_ur_simulator>`.

* To use mocked hardware (capability of ros2_control), use ``use_mock_hardware`` argument, like:

  .. code-block:: console

     $ ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=yyy.yyy.yyy.yyy use_mock_hardware:=true


Sending commands to controllers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Once the driver is started, you can use it like any other ROS robot arm driver by sending
trajectories to the ``/scaled_joint_trajectory_controller/follow_joint_trajectory`` action. If
you're new to that, please have a look at the section ":ref:`move_the_robot`".

Robot frames
------------

While most tf frames come from the URDF and are published from the ``robot_state_publisher``, there
are a couple of things to know:

- The ``base`` frame is the robot's base as the robot controller sees it.
- The ``tool0`` is the tool frame as calculated using forward kinematics. If there is no additional
  tool configured on the Teach pendant (TP) and the URDF uses the specific robot's
  :ref:`calibration <calibration_extraction>` the lookup from ``base`` -> ``tool0`` should be the
  same as the tcp pose shown on the TP when the feature "Base" is selected.

Troubleshooting
---------------

This section will cover some previously raised issues.

I started everything, but I cannot control the robot
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The driver is started, but trajectory execution doesn't work. Instead, the driver logs

.. code-block::

   Can't accept new action goals. Controller is not running.


The ``External Control`` program node from the URCap is not running on the robot. Make sure to
create a program containing this node on the robot and start it using the "Play" button
|play_button|. Inside the ROS terminal you should see the output ``Robot connected to reverse
interface. Ready to receive control commands.``

.. note::

   When interacting with the teach pendant, or sending other primary programs to the robot, the
   program will be stopped. On the ROS terminal you will see an output ``Connection to reverse interface dropped.``. Please see the section :ref:`continuation_after_interruptions` for details on how to continue.

In general, make sure you're starting up the robot as explained :ref:`here<ur_robot_driver_startup>`.

When starting the program on the teach pendant, I get an error "The connection to the remote PC could not be established"
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Make sure, the IP address setup is correct, as described in the setup guides
(:ref:`CB3 robots<install-urcap-cb3>`, :ref:`e-Series robots<install-urcap-e-series>`)

.. note::

   This error can also show up, when the ROS driver is not running.

When starting the driver, it crashes with "Variable 'speed_slider_mask' is currently controlled by another RTDE client"
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Probably, there is a fieldbus enabled on the robot. Currently, this driver cannot be used together
with an enabled fieldbus. Disable any fieldbus to workaround this error. `ROS 1 driver issue
#204 <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/204>`_ contains a guide
how to do
this.

I sent raw script code to the robot but it is not executed
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

On the e-Series the robot has to be in :ref:`remote control mode <operation_modes>` to accept script code from an external source. This has to be switched from the Teach-Pendant.

Using the dashboard doesn't work
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

On the e-Series the robot has to be in :ref:`remote control mode <operation_modes>` to accept certain calls on the dashboard server. See :ref:`Available dashboard commands <dashboard_client>` for details.

.. |play_button| image:: ../resources/play_button.svg
                 :height: 20px
                 :width: 20px
