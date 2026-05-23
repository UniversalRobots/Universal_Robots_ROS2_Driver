#!/usr/bin/env python3
# Copyright 2026, Universal Robots A/S
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
#    * Neither the name of the copyright holder nor the names of its
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

"""
Publish URScript to the driver's script-command bridge (`urscript_interface`).

The C++ node ``urscript_interface`` subscribes to ``~/script_command``, i.e. by
default ``/urscript_interface/script_command``. This example is a small Python
client ("sender") that publishes a secondary program so the main external-control
program is left running; see ``doc/usage/script_code.rst``.

NOTE: For this example to work, the urscript_interface node must be started with a robot being in
remote_control mode. If the robot leaves remote_control mode after it has been connected, the
urscript_interface has to be restarted.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class UrscriptSenderExample(Node):
    def __init__(self):
        super().__init__("urscript_sender_example")
        self._pub = self.create_publisher(String, "urscript_interface/script_command", 1)

        self.timer = self.create_timer(1.0, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        self.set_output(2, (self.i % 2 == 0))
        self.i = self.i + 1

    def set_output(self, pin: int, value: bool):
        """
        Publish a URScript snippet that sets a digital output.

        We use a secondary program (``io_program``) to avoid interfering with the main program
        currently running on the robot. Note that not all URScript commands are allowed in a
        secondary program; see the documentation for details.
        """
        script_code = "sec io_program():\n" f"  set_digital_out({pin}, {value})\n" "end\n"
        self.send_once(script_code)

    def send_once(self, script_code: str = None):
        msg = String(data=script_code)
        self._pub.publish(msg)
        self.get_logger().info("Published URScript snippet")


def main(args=None):
    rclpy.init(args=args)
    node = UrscriptSenderExample()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
