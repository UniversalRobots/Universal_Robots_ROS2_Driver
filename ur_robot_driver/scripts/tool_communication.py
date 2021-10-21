#!/usr/bin/env python3
# Copyright (c) 2021 PickNik, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Small helper script to start the tool communication interface."""

import subprocess

import rclpy.logging
from rclpy.node import Node


class UrToolCommunication(Node):
    """Starts socat."""

    def __init__(self):
        super().__init__("ur_tool_communication")
        # IP address of the robot
        self.declare_parameter("robot_ip", "192.168.56.101")
        robot_ip = self.get_parameter("robot_ip").get_parameter_value().string_value
        self.get_logger().info(robot_ip)
        # Port on which the remote pc (robot) publishes the interface
        self.declare_parameter("tcp_port", "54321")
        tcp_port = self.get_parameter_or("tcp_port", "54321").get_parameter_value().string_value
        # By default, socat will create a pty in /dev/pts/N with n being an increasing number.
        # Additionally, a symlink at the given location will be created. Use an absolute path here.
        self.declare_parameter("device_name", "/tmp/ttyUR")
        local_device = self.get_parameter("device_name").get_parameter_value().string_value

        self.get_logger().info("Remote device is available at '" + local_device + "'")

        cfg_params = ["pty"]
        cfg_params.append("link=" + local_device)
        cfg_params.append("raw")
        cfg_params.append("ignoreeof")
        cfg_params.append("waitslave")

        cmd = ["socat"]
        cmd.append(",".join(cfg_params))
        cmd.append(":".join(["tcp", robot_ip, tcp_port]))

        self.get_logger().info("Starting socat with following command:\n" + " ".join(cmd))
        subprocess.call(cmd)


def main():
    rclpy.init()
    node = UrToolCommunication()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
