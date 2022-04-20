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
     - yes
   * - joint-velocity-based control
     - yes\ :raw-html-m2r:`<sup>1</sup>`
   * - Cartesian position-based control
     - no
   * - Cartesian twist-based control
     - no
   * - Trajectory forwarding for execution on robot
     - no
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
     - yes
   * - use `tool communication forwarder <https://github.com/UniversalRobots/Universal_Robots_ToolComm_Forwarder_URCap>`_ on e-series
     - yes
   * - use the driver without a teach pendant necessary
     - yes
   * - support of CB1 and CB2 robots
     - no
   * - trajectory extrapolation on robot on missing packages
     - yes
   * - use ROS as drop-in for TP-programs
     - yes
   * - headless mode
     - yes
   * - extract calibration from robot
     - yes
   * - send custom script commands to robot
     - no
   * - Reconnect on a disconnected robot
     - yes


:raw-html-m2r:`<sup>1</sup>` Velocity-based joint control is implemented in the driver, but the current version of ros2_control do not yet support Velocity-based joint trajectory control
