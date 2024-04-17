#!/usr/bin/env python
# Copyright 2023, FZI Forschungszentrum Informatik
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
import os
import sys
import time
import unittest

import pytest
import rclpy
import rclpy.node
from std_msgs.msg import String as StringMsg
from ur_msgs.msg import IOStates

sys.path.append(os.path.dirname(__file__))
from test_common import (  # noqa: E402
    ControllerManagerInterface,
    DashboardInterface,
    IoStatusInterface,
    generate_driver_test_description,
)

ROBOT_IP = "192.168.56.101"


@pytest.mark.launch_test
def generate_test_description():
    return generate_driver_test_description()


class URScriptInterfaceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context
        rclpy.init()
        cls.node = rclpy.node.Node("urscript_interface_test")
        cls.init_robot(cls)

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        cls.node.destroy_node()
        rclpy.shutdown()

    def init_robot(self):
        self._dashboard_interface = DashboardInterface(self.node)
        self._controller_manager_interface = ControllerManagerInterface(self.node)
        self._io_status_controller_interface = IoStatusInterface(self.node)

        self.urscript_pub = self.node.create_publisher(
            StringMsg, "/urscript_interface/script_command", 1
        )

    def setUp(self):
        self._dashboard_interface.start_robot()
        time.sleep(1)
        self.assertTrue(self._io_status_controller_interface.resend_robot_program().success)

        self._controller_manager_interface.wait_for_controller("io_and_status_controller")

    def test_set_io(self):
        """Test setting an IO using a direct program call."""
        self.io_states_sub = self.node.create_subscription(
            IOStates,
            "/io_and_status_controller/io_states",
            self.io_msg_cb,
            rclpy.qos.qos_profile_system_default,
        )

        self.set_digout_checked(0, True)
        time.sleep(1)
        self.set_digout_checked(0, False)

        self.io_msg = None
        self.io_states_sub = self.node.create_subscription(
            IOStates,
            "/io_and_status_controller/io_states",
            self.io_msg_cb,
            rclpy.qos.qos_profile_system_default,
        )

        script_msg = StringMsg(
            data="sec my_program():\n  set_digital_out(0, False)\n  set_digital_out(1,True)\nend"
        )
        self.urscript_pub.publish(script_msg)
        self.check_pin_states([0, 1], [False, True])

        time.sleep(1)

        script_msg = StringMsg(
            data="sec my_program():\n  set_digital_out(0, True)\n  set_digital_out(1,False)\nend"
        )
        self.urscript_pub.publish(script_msg)
        self.check_pin_states([0, 1], [True, False])

    def io_msg_cb(self, msg):
        self.io_msg = msg

    def set_digout_checked(self, pin, state):
        self.io_msg = None

        script_msg = StringMsg(data=f"set_digital_out({pin}, {state})")
        self.urscript_pub.publish(script_msg)

        self.check_pin_states([pin], [state])

    def check_pin_states(self, pins, states):
        pin_states = [not x for x in states]
        end_time = time.time() + 50
        while pin_states != states and time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.io_msg is not None:
                for i, pin_id in enumerate(pins):
                    pin_states[i] = self.io_msg.digital_out_states[pin_id].state
        self.assertIsNotNone(self.io_msg, "Did not receive an IO state in requested time.")
        self.assertEqual(pin_states, states)
