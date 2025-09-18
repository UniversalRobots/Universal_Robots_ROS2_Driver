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
import logging

import pytest

import launch_testing
import rclpy
from rclpy.node import Node

from controller_manager_msgs.srv import SwitchController
from ur_msgs.action import ToolContact
from action_msgs.msg import GoalStatus

sys.path.append(os.path.dirname(__file__))
from test_common import (  # noqa: E402
    ControllerManagerInterface,
    DashboardInterface,
    IoStatusInterface,
    ActionInterface,
    generate_driver_test_description,
)


@pytest.mark.launch_test
@launch_testing.parametrize(
    "tf_prefix",
    [(""), ("my_ur_")],
)
def generate_test_description(tf_prefix):
    return generate_driver_test_description(tf_prefix=tf_prefix)


class ToolContactTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context
        rclpy.init()
        cls.node = Node("tool_contact_test")
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
        self._tool_contact_interface = ActionInterface(
            self.node, "/tool_contact_controller/detect_tool_contact", ToolContact
        )

    def setUp(self):
        self._dashboard_interface.start_robot()
        time.sleep(1)
        self.assertTrue(self._io_status_controller_interface.resend_robot_program().success)

    #
    # Tests
    #

    def test_start_tool_contact_controller(self):
        self.assertTrue(
            self._controller_manager_interface.switch_controller(
                strictness=SwitchController.Request.BEST_EFFORT,
                activate_controllers=["tool_contact_controller"],
            ).ok
        )

    def test_goal_can_be_canceled(self):
        self.assertTrue(
            self._controller_manager_interface.switch_controller(
                strictness=SwitchController.Request.BEST_EFFORT,
                activate_controllers=["tool_contact_controller"],
            ).ok
        )
        goal_handle = self._tool_contact_interface.send_goal()
        self.assertTrue(goal_handle.accepted)

        cancel_res = self._tool_contact_interface.cancel_goal(goal_handle)
        self.assertEqual(cancel_res.return_code, 0)

    def test_deactivate_controller_aborts_action(self):
        self.assertTrue(
            self._controller_manager_interface.switch_controller(
                strictness=SwitchController.Request.BEST_EFFORT,
                activate_controllers=["tool_contact_controller"],
            ).ok
        )

        goal_handle = self._tool_contact_interface.send_goal()
        self.assertTrue(goal_handle.accepted)

        self.assertTrue(
            self._controller_manager_interface.switch_controller(
                strictness=SwitchController.Request.BEST_EFFORT,
                deactivate_controllers=["tool_contact_controller"],
            ).ok
        )
        future_res = goal_handle.get_result_async()

        timeout = 5.0
        logging.info("Waiting for action result from controller with timeout %fs", timeout)
        rclpy.spin_until_future_complete(self.node, future_res, timeout_sec=timeout)

        if future_res.result() is not None:
            logging.info("  Received result: %s", future_res.result().result)
            # Check status of goal handle, as result does not contain information about the status of the action. Only the empty result definition.
            self.assertEqual(future_res.result().status, GoalStatus.STATUS_ABORTED)
        else:
            raise Exception(
                f"Exception while calling action '{self.__action_name}': {future_res.exception()}"
            )

    def test_inactive_controller_rejects_actions(self):
        self.assertTrue(
            self._controller_manager_interface.switch_controller(
                strictness=SwitchController.Request.BEST_EFFORT,
                deactivate_controllers=["tool_contact_controller"],
            ).ok
        )

        goal_handle = self._tool_contact_interface.send_goal()
        self.assertFalse(goal_handle.accepted)

    def test_busy_controller_rejects_actions(self):
        self.assertTrue(
            self._controller_manager_interface.switch_controller(
                strictness=SwitchController.Request.BEST_EFFORT,
                activate_controllers=["tool_contact_controller"],
            ).ok
        )

        goal_handle = self._tool_contact_interface.send_goal()
        self.assertTrue(goal_handle.accepted)

        goal_handle = self._tool_contact_interface.send_goal()
        self.assertFalse(goal_handle.accepted)
