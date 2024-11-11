#!/usr/bin/env python
# Copyright 2019, Universal Robots A/S
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

import launch_testing
import rclpy
from rclpy.node import Node

from controller_manager_msgs.srv import SwitchController

sys.path.append(os.path.dirname(__file__))
from test_common import (  # noqa: E402
    ControllerManagerInterface,
    DashboardInterface,
    IoStatusInterface,
    generate_driver_test_description,
)


@pytest.mark.launch_test
@launch_testing.parametrize(
    "tf_prefix",
    [(""), ("my_ur_")],
)
def generate_test_description(tf_prefix):
    return generate_driver_test_description(tf_prefix=tf_prefix)


class RobotDriverTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context
        rclpy.init()
        cls.node = Node("robot_driver_test")
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

    def test_activating_multiple_controllers_same_interface_fails(self):
        # Deactivate all writing controllers
        self.assertTrue(
            self._controller_manager_interface.switch_controller(
                strictness=SwitchController.Request.BEST_EFFORT,
                deactivate_controllers=[
                    "scaled_joint_trajectory_controller",
                    "joint_trajectory_controller",
                    "forward_position_controller",
                    "forward_velocity_controller",
                    "passthrough_trajectory_controller",
                ],
            ).ok
        )

        # Activating different motion controllers should not be possible
        self.assertFalse(
            self._controller_manager_interface.switch_controller(
                strictness=SwitchController.Request.STRICT,
                activate_controllers=[
                    "scaled_joint_trajectory_controller",
                    "joint_trajectory_controller",
                ],
            ).ok
        )
        self.assertFalse(
            self._controller_manager_interface.switch_controller(
                strictness=SwitchController.Request.STRICT,
                activate_controllers=[
                    "scaled_joint_trajectory_controller",
                    "forward_position_controller",
                ],
            ).ok
        )
        self.assertFalse(
            self._controller_manager_interface.switch_controller(
                strictness=SwitchController.Request.STRICT,
                activate_controllers=[
                    "scaled_joint_trajectory_controller",
                    "forward_position_controller",
                ],
            ).ok
        )

    def test_activating_multiple_controllers_different_interface_fails(self):
        # Deactivate all writing controllers
        self.assertTrue(
            self._controller_manager_interface.switch_controller(
                strictness=SwitchController.Request.BEST_EFFORT,
                deactivate_controllers=[
                    "scaled_joint_trajectory_controller",
                    "joint_trajectory_controller",
                    "forward_position_controller",
                    "forward_velocity_controller",
                    "passthrough_trajectory_controller",
                ],
            ).ok
        )
        self.assertFalse(
            self._controller_manager_interface.switch_controller(
                strictness=SwitchController.Request.STRICT,
                activate_controllers=[
                    "scaled_joint_trajectory_controller",
                    "forward_velocity_controller",
                ],
            ).ok
        )
        self.assertFalse(
            self._controller_manager_interface.switch_controller(
                strictness=SwitchController.Request.STRICT,
                activate_controllers=[
                    "scaled_joint_trajectory_controller",
                    "passthrough_trajectory_controller",
                ],
            ).ok
        )
        self.assertFalse(
            self._controller_manager_interface.switch_controller(
                strictness=SwitchController.Request.STRICT,
                activate_controllers=[
                    "forward_velocity_controller",
                    "passthrough_trajectory_controller",
                ],
            ).ok
        )
        self.assertFalse(
            self._controller_manager_interface.switch_controller(
                strictness=SwitchController.Request.STRICT,
                activate_controllers=[
                    "forward_position_controller",
                    "forward_velocity_controller",
                ],
            ).ok
        )

    def test_activating_controller_with_running_position_controller_fails(self):
        # Having a position-based controller active, no other controller should be able to
        # activate.
        self.assertTrue(
            self._controller_manager_interface.switch_controller(
                strictness=SwitchController.Request.BEST_EFFORT,
                activate_controllers=[
                    "scaled_joint_trajectory_controller",
                ],
                deactivate_controllers=[
                    "joint_trajectory_controller",
                    "forward_position_controller",
                    "forward_velocity_controller",
                    "passthrough_trajectory_controller",
                ],
            ).ok
        )
        self.assertFalse(
            self._controller_manager_interface.switch_controller(
                strictness=SwitchController.Request.STRICT,
                activate_controllers=[
                    "forward_position_controller",
                ],
            ).ok
        )
        self.assertFalse(
            self._controller_manager_interface.switch_controller(
                strictness=SwitchController.Request.STRICT,
                activate_controllers=[
                    "forward_velocity_controller",
                ],
            ).ok
        )
        self.assertFalse(
            self._controller_manager_interface.switch_controller(
                strictness=SwitchController.Request.STRICT,
                activate_controllers=[
                    "passthrough_trajectory_controller",
                ],
            ).ok
        )
        # Stop controller again
        self.assertTrue(
            self._controller_manager_interface.switch_controller(
                strictness=SwitchController.Request.STRICT,
                deactivate_controllers=[
                    "scaled_joint_trajectory_controller",
                ],
            ).ok
        )

    def test_activating_controller_with_running_passthrough_trajectory_controller_fails(self):
        # Having a position-based controller active, no other controller should be able to
        # activate.
        self.assertTrue(
            self._controller_manager_interface.switch_controller(
                strictness=SwitchController.Request.BEST_EFFORT,
                activate_controllers=["passthrough_trajectory_controller"],
                deactivate_controllers=[
                    "scaled_joint_trajectory_controller",
                    "joint_trajectory_controller",
                    "forward_position_controller",
                    "forward_velocity_controller",
                ],
            ).ok
        )
        self.assertFalse(
            self._controller_manager_interface.switch_controller(
                strictness=SwitchController.Request.STRICT,
                activate_controllers=[
                    "scaled_joint_trajectory_controller",
                ],
            ).ok
        )
        self.assertFalse(
            self._controller_manager_interface.switch_controller(
                strictness=SwitchController.Request.STRICT,
                activate_controllers=[
                    "forward_position_controller",
                ],
            ).ok
        )
        self.assertFalse(
            self._controller_manager_interface.switch_controller(
                strictness=SwitchController.Request.STRICT,
                activate_controllers=[
                    "forward_velocity_controller",
                ],
            ).ok
        )
        self.assertFalse(
            self._controller_manager_interface.switch_controller(
                strictness=SwitchController.Request.STRICT,
                activate_controllers=[
                    "joint_trajectory_controller",
                ],
            ).ok
        )
        # Stop the controller again
        self.assertTrue(
            self._controller_manager_interface.switch_controller(
                strictness=SwitchController.Request.STRICT,
                deactivate_controllers=[
                    "passthrough_trajectory_controller",
                ],
            ).ok
        )
