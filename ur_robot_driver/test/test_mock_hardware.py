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

import os
import sys
import time
import unittest

import launch_testing
import pytest
import rclpy
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import SwitchController

sys.path.append(os.path.dirname(__file__))
from test_common import (  # noqa: E402
    ActionInterface,
    ControllerManagerInterface,
    IoStatusInterface,
    ConfigurationInterface,
    generate_mock_hardware_test_description,
    sjtc_trajectory_test,
    sjtc_illegal_trajectory_test,
)


@pytest.mark.launch_test
@launch_testing.parametrize("tf_prefix", [(""), ("my_ur_")])
def generate_test_description(tf_prefix):
    return generate_mock_hardware_test_description(tf_prefix=tf_prefix)


class MockHWTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context
        rclpy.init()
        cls.node = Node("mock_hardware_test")
        time.sleep(1)
        cls.init_robot(cls)

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        cls.node.destroy_node()
        rclpy.shutdown()

    # Connect to all interfaces and actions, even ones we know won't work with mock hardware (Except dashboard)
    def init_robot(self):
        self._controller_manager_interface = ControllerManagerInterface(self.node)
        self._io_status_controller_interface = IoStatusInterface(self.node)
        self._configuration_controller_interface = ConfigurationInterface(self.node)

        self._scaled_follow_joint_trajectory = ActionInterface(
            self.node,
            "/scaled_joint_trajectory_controller/follow_joint_trajectory",
            FollowJointTrajectory,
        )

        self._passthrough_forward_joint_trajectory = ActionInterface(
            self.node,
            "/passthrough_trajectory_controller/follow_joint_trajectory",
            FollowJointTrajectory,
        )

    #
    # Test functions
    #

    def test_get_robot_software_version(self):
        self.assertEqual(
            self._configuration_controller_interface.get_robot_software_version().major, 1
        )

    def test_start_scaled_jtc_controller(self):
        # Deactivate controller, if it is not already
        self.assertTrue(
            self._controller_manager_interface.switch_controller(
                strictness=SwitchController.Request.BEST_EFFORT,
                deactivate_controllers=["scaled_joint_trajectory_controller"],
            ).ok
        )
        # Activate controller
        self.assertTrue(
            self._controller_manager_interface.switch_controller(
                strictness=SwitchController.Request.STRICT,
                activate_controllers=["scaled_joint_trajectory_controller"],
            ).ok
        )

    def test_trajectory(self, tf_prefix):
        sjtc_trajectory_test(self, tf_prefix)

    def test_illegal_trajectory(self, tf_prefix):
        sjtc_illegal_trajectory_test(self, tf_prefix)
