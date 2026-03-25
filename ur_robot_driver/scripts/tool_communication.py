#!/usr/bin/env python3
# -- BEGIN LICENSE BLOCK ----------------------------------------------
# Copyright 2026 Universal Robots A/S
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
# -- END LICENSE BLOCK ------------------------------------------------

import subprocess
import rclpy.logging
from rclpy.node import Node
from pathlib import Path
from ament_index_python.packages import get_package_prefix

class UrToolCommunication(Node):
    """
    Wrapper node for tool communication.

    This node is kept for backwards compatibility. It reads the ROS parameters
    and forwards them to the canonical tool_communication.py script provided
    by ur_client_library.
    """

    def __init__(self):
        super().__init__("ur_tool_communication")

        self.get_logger().info("Initializing tool communication wrapper node")

        # IP address of the robot
        self.declare_parameter("robot_ip", "192.168.56.101")
        # TCP port used by the tool communication forwarder
        self.declare_parameter("tcp_port", 54321)
        # Path where a symlink to the PTY created by socat (/dev/pts/N) will be generated
        self.declare_parameter("device_name", "/tmp/ttyUR")

        robot_ip = self.get_parameter("robot_ip").get_parameter_value().string_value
        tcp_port = self.get_parameter_or("tcp_port", 54321).get_parameter_value().integer_value
        device_name = self.get_parameter("device_name").get_parameter_value().string_value

        # Path where the refactored (canonical) tool_communication script is located
        ur_client_lib_prefix = get_package_prefix("ur_client_library")
        tool_comm_script = Path(ur_client_lib_prefix) / "lib" / "ur_client_library" / "tool_communication.py"

        # Pass the arguments to the refactored script
        cmd = [
            str(tool_comm_script),
            robot_ip,                       
            "--tcp-port", str(tcp_port),
            "--device-name", device_name,
        ]

        self.get_logger().info("Launching tool communication via ur_client_library tool_communication.py")

        subprocess.call(cmd)

def main():
    rclpy.init()
    node = UrToolCommunication()
    rclpy.spin(node)


if __name__ == "__main__":
    main()