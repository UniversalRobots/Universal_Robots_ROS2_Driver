#!/usr/bin/env python
# Copyright 2025, Universal Robots A/S
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

import logging
import os
import sys
import time
import unittest

import launch_testing
import pytest
import rclpy
from rclpy.node import Node
from ur_msgs.msg import IOStates

sys.path.append(os.path.dirname(__file__))
from test_common import (  # noqa: E402
    ControllerManagerInterface,
    DashboardInterface,
    IoStatusInterface,
    generate_driver_test_description,
)


@pytest.mark.launch_test
@launch_testing.parametrize("tf_prefix", [(""), ("my_ur_")])
def generate_test_description(tf_prefix):
    return generate_driver_test_description(tf_prefix=tf_prefix)


class IOControllerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context
        rclpy.init()
        cls.node = Node("io_controller_test")
        time.sleep(1)
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

    def setUp(self):
        self._dashboard_interface.start_robot()
        time.sleep(1)
        self.assertTrue(self._io_status_controller_interface.resend_robot_program().success)

    #
    # Test functions
    #

    def test_set_io(self):
        """Test to set an IO and check whether it has been set."""
        # Create io callback to verify result
        io_msg = None

        def io_msg_cb(msg):
            nonlocal io_msg
            io_msg = msg

        io_states_sub = self.node.create_subscription(
            IOStates,
            "/io_and_status_controller/io_states",
            io_msg_cb,
            rclpy.qos.qos_profile_system_default,
        )

        # Set pin 0 to 1.0
        test_pin = 0

        logging.info("Setting pin %d to 1.0", test_pin)
        self._io_status_controller_interface.set_io(fun=1, pin=test_pin, state=1.0)

        # Wait until the pin state has changed
        pin_state = False
        end_time = time.time() + 5
        while not pin_state and time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if io_msg is not None:
                pin_state = io_msg.digital_out_states[test_pin].state

        self.assertEqual(pin_state, 1.0)

        # Set pin 0 to 0.0
        logging.info("Setting pin %d to 0.0", test_pin)
        self._io_status_controller_interface.set_io(fun=1, pin=test_pin, state=0.0)

        # Wait until the pin state has changed back
        end_time = time.time() + 5
        while pin_state and time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if io_msg is not None:
                pin_state = io_msg.digital_out_states[test_pin].state

        self.assertEqual(pin_state, 0.0)

        # Clean up io subscription
        self.node.destroy_subscription(io_states_sub)
