# Feature list and roadmap

| Feature                                               | ROS2 Driver
| ---                                                   | ---                       |
| position-based control                                | yes                       |
| scaled position-based control                         | yes                       |
| velocity-based control                                | no<sup>1</sup>            |
| reporting of tcp wrench                               | not yet implemented       |
| reporting of tcp wrench in tcp link                   | not yet implemented       |
| pausing of programs                                   | yes                       |
| continue trajectories after EM-Stop resume            | yes                       |
| continue trajectories after protective stop           | yes                       |
| panel interaction in between possible                 | yes                       |
| get and set IO states                                 | yes                       |
| use [tool communication forwarder](https://github.com/UniversalRobots/Universal_Robots_ToolComm_Forwarder_URCap) on e-series | should work, not yet tested with ROS2            |
| use the driver without a teach pendant necessary      | yes                       |
| support of CB2 robots                                 | -                         |
| trajectory extrapolation on robot on missing packages | yes                       |
| use ROS as drop-in for TP-programs                    | yes                       |
| headless mode                                         | not yet implemented       |
| extract calibration from robot                        | not yet implemented       |
| send custom script commands to robot                  | yes                       |
| Reconnect on a disconnected robot                     | yes                       |

<sup>1</sup> Velocity-based control is currently not supported by ros2_control
