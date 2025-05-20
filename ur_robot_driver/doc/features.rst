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
     - yes (:ref:`scaled_jtc`)
   * - joint-velocity-based control
     - yes\ :raw-html-m2r:`<sup>1</sup>`
   * - Cartesian position-based control
     - no
   * - Cartesian twist-based control
     - no
   * - Trajectory forwarding for execution on robot
     - yes (:ref:`passthrough_trajectory_controller`)
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
     - yes (:ref:`io_and_status_controller`)
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
     - yes (:ref:`headless_mode`)
   * - extract calibration from robot
     - yes (:ref:`ur_calibration`)
   * - send custom script commands to robot
     - yes (:ref:`io_and_status_controller`)
   * - Reconnect on a disconnected robot
     - yes
   * - Freedrive Mode
     - yes (:ref:`freedrive_mode_controller`)
   * - Tool Contact mode
     - yes (:ref:`tool_contact_controller`)
   * - Force Mode
     - yes (:ref:`force_mode_controller`)

:raw-html-m2r:`<sup>1</sup>` Velocity-based joint control is implemented in the driver, the velocity-based joint trajectory controller would need tweaking of the gain parameters for each model.
