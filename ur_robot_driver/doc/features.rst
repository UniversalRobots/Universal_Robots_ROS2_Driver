.. role:: raw-html-m2r(raw)
   :format: html


Feature list and roadmap
------------------------

.. list-table::
   :header-rows: 1

   * - Feature
     - ROS2 Driver
   * - joint-position-based control
     - yes
   * - scaled joint-position-based control
     - yes (`scaled_jtc
       <https://docs.ros.org/en/humble/p/ur_controllers/doc/index.html#ur-controlers-scaledjointtrajectorycontroller>`_)
   * - joint-velocity-based control
     - yes\ :raw-html-m2r:`<sup>1</sup>`
   * - Cartesian position-based control
     - no
   * - Cartesian twist-based control
     - no
   * - Trajectory forwarding for execution on robot
     - yes (`passthrough_trajectory_controller <https://docs.ros.org/en/humble/p/ur_controllers/doc/index.html#ur-controllers-passthroughtrajectorycontroller>`_)
   * - reporting of tcp wrench
     - yes
   * - pausing of programs
     - yes
   * - continue trajectories after EM-Stop resume
     - yes
   * - continue trajectories after protective stop
     - yes
   * - panel interaction in between possible
     - yes
   * - get and set IO states
     - yes (`io_and_status_controller <https://docs.ros.org/en/humble/p/ur_controllers/doc/index.html#ur-controllers-gpiocontroller>`_)
   * - use `tool communication forwarder <https://github.com/UniversalRobots/Universal_Robots_ToolComm_Forwarder_URCap>`_ on e-series
     - yes (:ref:`setup-tool-communication`)
   * - use the driver without a teach pendant necessary
     - yes
   * - support of CB1 and CB2 robots
     - no
   * - trajectory extrapolation on robot on missing packages
     - yes
   * - use ROS as drop-in for TP-programs
     - yes
   * - headless mode
     - yes (`headless_mode
       <https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/doc/ur_robot_driver/ur_robot_driver/doc/operation_modes.html#headless-mode>`_)
   * - extract calibration from robot
     - yes (`ur_calibration <https://docs.ros.org/en/humble/p/ur_calibration/doc/index.html>`_)
   * - send custom script commands to robot
     - yes (`io_and_status_controller`_)
   * - Reconnect on a disconnected robot
     - yes
   * - Freedrive Mode
     - yes (`freedrive_mode_controller
       <https://docs.ros.org/en/humble/p/ur_controllers/doc/index.html#ur-controllers-forcemodecontroller>`_)
   * - Tool Contact mode
     - yes (`tool_contact_controller <https://docs.ros.org/en/humble/p/ur_controllers/doc/index.html#ur-controllers-toolcontactcontroller>`_)
   * - Force Mode
     - yes (`force_mode_controller <https://docs.ros.org/en/humble/p/ur_controllers/doc/index.html#ur-controllers-freedrivemodecontroller>`_)

:raw-html-m2r:`<sup>1</sup>` Velocity-based joint control is implemented in the driver, the velocity-based joint trajectory controller would need tweaking of the gain parameters for each model.
