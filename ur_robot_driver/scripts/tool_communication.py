#!/usr/bin/env python3
# Copyright (c) 2021 PickNik, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


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
        self.declare_parameter("tcp_port", 54321)
        tcp_port = self.get_parameter_or("tcp_port", 54321).get_parameter_value().integer_value
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
        cmd.append(":".join(["tcp", robot_ip, str(tcp_port)]))

        self.get_logger().info("Starting socat with following command:\n" + " ".join(cmd))
        subprocess.call(cmd)


def main():
    rclpy.init()
    node = UrToolCommunication()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
