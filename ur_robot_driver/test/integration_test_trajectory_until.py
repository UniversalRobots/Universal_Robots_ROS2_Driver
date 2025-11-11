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

import pytest

import launch_testing
import rclpy
from rclpy.node import Node

from controller_manager_msgs.srv import SwitchController
from ur_msgs.action import FollowJointTrajectoryUntil
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

sys.path.append(os.path.dirname(__file__))
from test_common import (  # noqa: E402
    ControllerManagerInterface,
    DashboardInterface,
    IoStatusInterface,
    ActionInterface,
    ROBOT_JOINTS,
    TIMEOUT_EXECUTE_TRAJECTORY,
    generate_driver_test_description,
)


@pytest.mark.launch_test
@launch_testing.parametrize(
    "tf_prefix, initial_joint_controller",
    [("", "scaled_joint_trajectory_controller"), ("my_ur_", "passthrough_trajectory_controller")],
)
def generate_test_description(tf_prefix, initial_joint_controller):
    return generate_driver_test_description(
        tf_prefix=tf_prefix, initial_joint_controller=initial_joint_controller
    )


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
        self._trajectory_until_interface = ActionInterface(
            self.node, "/trajectory_until_node/execute", FollowJointTrajectoryUntil
        )
        self.test_traj = {
            "waypts": [[1.5, -1.5, 0.0, -1.5, -1.5, -1.5], [2.1, -1.2, 0.0, -2.4, -1.5, -1.5]],
            "time_vec": [Duration(sec=3, nanosec=0), Duration(sec=6, nanosec=0)],
        }

        self._controller_manager_interface.wait_for_controller("tool_contact_controller")

    def setUp(self, initial_joint_controller):
        self._dashboard_interface.start_robot()
        time.sleep(1)
        self.assertTrue(self._io_status_controller_interface.resend_robot_program().success)

        self._controller_manager_interface.wait_for_controller(initial_joint_controller, "active")

    #
    # Tests
    #

    def test_trajectory_with_tool_contact_no_trigger_succeeds(
        self, tf_prefix, initial_joint_controller
    ):
        self._controller_manager_interface.wait_for_controller(initial_joint_controller)
        self.assertTrue(
            self._controller_manager_interface.switch_controller(
                strictness=SwitchController.Request.BEST_EFFORT,
                activate_controllers=["tool_contact_controller", initial_joint_controller],
            ).ok
        )
        trajectory = JointTrajectory()
        trajectory.joint_names = [tf_prefix + joint for joint in ROBOT_JOINTS]

        trajectory.points = [
            JointTrajectoryPoint(
                positions=self.test_traj["waypts"][i], time_from_start=self.test_traj["time_vec"][i]
            )
            for i in range(len(self.test_traj["waypts"]))
        ]
        goal_handle = self._trajectory_until_interface.send_goal(
            trajectory=trajectory, until_type=FollowJointTrajectoryUntil.Goal.TOOL_CONTACT
        )
        self.assertTrue(goal_handle.accepted)
        if goal_handle.accepted:
            result = self._trajectory_until_interface.get_result(
                goal_handle, TIMEOUT_EXECUTE_TRAJECTORY
            )
        self.assertEqual(result.error_code, FollowJointTrajectoryUntil.Result.SUCCESSFUL)
        self.assertEqual(
            result.until_condition_result, FollowJointTrajectoryUntil.Result.NOT_TRIGGERED
        )

    def test_trajectory_until_can_cancel(self, tf_prefix, initial_joint_controller):
        self._controller_manager_interface.wait_for_controller(initial_joint_controller)
        self.assertTrue(
            self._controller_manager_interface.switch_controller(
                strictness=SwitchController.Request.BEST_EFFORT,
                activate_controllers=["tool_contact_controller", initial_joint_controller],
            ).ok
        )
        trajectory = JointTrajectory()
        trajectory.joint_names = [tf_prefix + joint for joint in ROBOT_JOINTS]

        trajectory.points = [
            JointTrajectoryPoint(
                positions=self.test_traj["waypts"][i], time_from_start=self.test_traj["time_vec"][i]
            )
            for i in range(len(self.test_traj["waypts"]))
        ]
        goal_handle = self._trajectory_until_interface.send_goal(
            trajectory=trajectory, until_type=FollowJointTrajectoryUntil.Goal.TOOL_CONTACT
        )
        self.assertTrue(goal_handle.accepted)
        result = self._trajectory_until_interface.cancel_goal(goal_handle)
        self.assertTrue(len(result.goals_canceling) > 0)
