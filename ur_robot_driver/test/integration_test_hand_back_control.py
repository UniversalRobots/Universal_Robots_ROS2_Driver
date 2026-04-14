#!/usr/bin/env python
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
import socket
import sys
import time
import unittest

import pytest
import rclpy
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import SwitchController
from rclpy.node import Node
from std_msgs.msg import Bool

from ur_dashboard_msgs.msg import ProgramState

sys.path.append(os.path.dirname(__file__))
from test_common import (  # noqa: E402
    generate_driver_test_description,
    ActionInterface,
    ControllerManagerInterface,
    DashboardInterface,
    IoStatusInterface,
)


@pytest.mark.launch_test
def generate_test_description():
    program_folder = os.path.join(
        os.path.dirname(__file__), "resources", "ursim", "e-series", "ur5e", "programs"
    )
    print(f"Using URSim program folder: {program_folder}")
    print(f"Available files in program folder: {os.listdir(program_folder)}")
    return generate_driver_test_description(
        headless_mode=False, ursim_program_folder=program_folder
    )


class HandBackControlTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("hand_back_control_test")
        time.sleep(1)
        cls.init_robot(cls)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def init_robot(self):
        self._dashboard_interface = DashboardInterface(self.node)
        self._controller_manager_interface = ControllerManagerInterface(self.node)
        self._io_status_controller_interface = IoStatusInterface(self.node)

        self._scaled_follow_joint_trajectory = ActionInterface(
            self.node,
            "/scaled_joint_trajectory_controller/follow_joint_trajectory",
            FollowJointTrajectory,
        )

        self._controller_manager_interface.wait_for_controller("scaled_joint_trajectory_controller")

    def setUp(self):
        self._dashboard_interface.start_robot()
        time.sleep(1)

        # Open a socket server on port 20957 and add a connection callback
        self.serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.serversocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.serversocket.settimeout(0.2)  # timeout for listening
        self.serversocket.bind(("0.0.0.0", 20957))
        self.serversocket.listen(1)

    def tearDown(self):
        self.serversocket.close()

    def wait_for_connection(self, timeout=5):
        end_time = time.time() + timeout
        conn = None
        while time.time() < end_time and conn is None:
            try:
                conn, address = self.serversocket.accept()
                logging.info(f"Received connection from {address}")
                return True
            except socket.timeout:
                continue
        return False

    def test_hand_back_control_stops_reverse_interface_and_program_keeps_running(self):
        """
        Use hand_back_control to return program flow to the robot.

        We loaded a program that connects to a test socket and waits 10 seconds after
        external_control has finished. Hence, if we hand back control, the program should still
        be running and we should receive a connection on the socket server.
        """
        program_name = "hand_back_control_test_prog.urp"

        self._dashboard_interface.load_program(filename=program_name)

        external_control_running = None

        def program_state_cb(msg):
            nonlocal external_control_running
            external_control_running = msg.data

        program_state_sub = self.node.create_subscription(
            Bool,
            "/io_and_status_controller/robot_program_running",
            program_state_cb,
            rclpy.qos.qos_profile_system_default,
        )

        self._dashboard_interface.play()
        self._controller_manager_interface.wait_for_controller(
            "scaled_joint_trajectory_controller", "active"
        )
        self.assertTrue(
            self._controller_manager_interface.switch_controller(
                strictness=SwitchController.Request.BEST_EFFORT,
                deactivate_controllers=["passthrough_trajectory_controller"],
                activate_controllers=["scaled_joint_trajectory_controller"],
            ).ok
        )

        self.assertTrue(
            external_control_running, "Robot program should be running before hand_back_control"
        )

        # Call hand_back_control
        logging.info("Calling hand_back_control")
        result = self._io_status_controller_interface.hand_back_control()
        self.assertTrue(result.success, "hand_back_control service call should succeed")

        # Verify external control stops running
        end_time = time.time() + 10
        while external_control_running is not False and time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertFalse(
            external_control_running, "External Control should stop after hand_back_control"
        )

        self.assertTrue(
            self.wait_for_connection(),
            "Robot program should connect back to the socket server after hand_back_control",
        )
        program_state = self._dashboard_interface.program_state()
        self.assertEqual(
            program_state.state.state,
            ProgramState.PLAYING,
            "Robot program should still be running after hand_back_control",
        )
        self.assertEqual(
            program_state.program_name,
            program_name,
            "Robot should still be running the same program after hand_back_control",
        )

        self.node.destroy_subscription(program_state_sub)
