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

from math import pi
import launch_testing
import pytest
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from controller_manager_msgs.srv import SwitchController
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

sys.path.append(os.path.dirname(__file__))
from test_common import (  # noqa: E402
    ActionInterface,
    ControllerManagerInterface,
    DashboardInterface,
    IoStatusInterface,
    generate_driver_test_description,
    ROBOT_JOINTS,
    TIMEOUT_EXECUTE_TRAJECTORY,
)


# Mock hardware does not work with passthrough controller, so dont test with it
@pytest.mark.launch_test
@launch_testing.parametrize("tf_prefix", [(""), ("my_ur_")])
def generate_test_description(tf_prefix):
    return generate_driver_test_description(tf_prefix=tf_prefix)


HOME = {
    "elbow_joint": 0.0,
    "shoulder_lift_joint": -1.5708,
    "shoulder_pan_joint": 0.0,
    "wrist_1_joint": -1.5708,
    "wrist_2_joint": 0.0,
    "wrist_3_joint": 0.0,
}
waypts = [[HOME[joint] + i * pi / 4 for joint in ROBOT_JOINTS] for i in [0, -1, 1]]
time_vec = [
    Duration(sec=4, nanosec=0),
    Duration(sec=8, nanosec=0),
    Duration(sec=12, nanosec=0),
]
TEST_TRAJECTORY = [(time_vec[i], waypts[i]) for i in range(len(waypts))]


class PassthroughControllerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context
        rclpy.init()
        cls.node = Node("passthrough_controller_test")
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

        self._passthrough_forward_joint_trajectory = ActionInterface(
            self.node,
            "/passthrough_trajectory_controller/follow_joint_trajectory",
            FollowJointTrajectory,
        )

        # Wait for all controllers needed below, as controller manager services might fail
        # e.g. when attempting to deactivate an unknown controller
        self._controller_manager_interface.wait_for_controller("scaled_joint_trajectory_controller")

    def setUp(self):
        self._dashboard_interface.start_robot()
        time.sleep(1)
        self.assertTrue(self._io_status_controller_interface.resend_robot_program().success)

    #
    # Test functions
    #

    def test_start_passthrough_controller(self):
        self.assertTrue(
            self._controller_manager_interface.switch_controller(
                strictness=SwitchController.Request.BEST_EFFORT,
                activate_controllers=["passthrough_trajectory_controller"],
                deactivate_controllers=["scaled_joint_trajectory_controller"],
            ).ok
        )

    def test_passthrough_trajectory(self, tf_prefix):
        self.assertTrue(
            self._controller_manager_interface.switch_controller(
                strictness=SwitchController.Request.BEST_EFFORT,
                activate_controllers=["passthrough_trajectory_controller"],
                deactivate_controllers=["scaled_joint_trajectory_controller"],
            ).ok
        )

        goal_tolerance = [
            JointTolerance(position=0.01, name=tf_prefix + joint) for joint in ROBOT_JOINTS
        ]
        goal_time_tolerance = Duration(sec=1, nanosec=0)
        trajectory = JointTrajectory(
            points=[
                JointTrajectoryPoint(positions=pos, time_from_start=times)
                for (times, pos) in TEST_TRAJECTORY
            ],
            joint_names=[tf_prefix + joint for joint in ROBOT_JOINTS],
        )
        goal_handle = self._passthrough_forward_joint_trajectory.send_goal(
            trajectory=trajectory,
            goal_time_tolerance=goal_time_tolerance,
            goal_tolerance=goal_tolerance,
        )
        self.assertTrue(goal_handle.accepted)
        if goal_handle.accepted:
            result = self._passthrough_forward_joint_trajectory.get_result(
                goal_handle, TIMEOUT_EXECUTE_TRAJECTORY
            )
            self.assertEqual(result.error_code, FollowJointTrajectory.Result.SUCCESSFUL)

    def test_quintic_trajectory(self, tf_prefix):
        # Full quintic trajectory
        self.assertTrue(
            self._controller_manager_interface.switch_controller(
                strictness=SwitchController.Request.BEST_EFFORT,
                activate_controllers=["passthrough_trajectory_controller"],
                deactivate_controllers=["scaled_joint_trajectory_controller"],
            ).ok
        )
        trajectory = JointTrajectory(
            points=[
                JointTrajectoryPoint(
                    positions=pos,
                    time_from_start=times,
                    velocities=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    accelerations=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                )
                for (times, pos) in TEST_TRAJECTORY
            ],
            joint_names=[tf_prefix + joint for joint in ROBOT_JOINTS],
        )
        goal_time_tolerance = Duration(sec=1, nanosec=0)
        goal_tolerance = [
            JointTolerance(position=0.01, name=tf_prefix + joint) for joint in ROBOT_JOINTS
        ]
        goal_handle = self._passthrough_forward_joint_trajectory.send_goal(
            trajectory=trajectory,
            goal_time_tolerance=goal_time_tolerance,
            goal_tolerance=goal_tolerance,
        )

        self.assertTrue(goal_handle.accepted)
        if goal_handle.accepted:
            result = self._passthrough_forward_joint_trajectory.get_result(
                goal_handle, TIMEOUT_EXECUTE_TRAJECTORY
            )
            self.assertEqual(result.error_code, FollowJointTrajectory.Result.SUCCESSFUL)

    def test_impossible_goal_tolerance_fails(self, tf_prefix):
        # Test impossible goal tolerance, should fail.
        self.assertTrue(
            self._controller_manager_interface.switch_controller(
                strictness=SwitchController.Request.BEST_EFFORT,
                activate_controllers=["passthrough_trajectory_controller"],
                deactivate_controllers=["scaled_joint_trajectory_controller"],
            ).ok
        )
        trajectory = JointTrajectory(
            points=[
                JointTrajectoryPoint(positions=pos, time_from_start=times)
                for (times, pos) in TEST_TRAJECTORY
            ],
            joint_names=[tf_prefix + joint for joint in ROBOT_JOINTS],
        )
        goal_tolerance = [
            JointTolerance(position=0.000000001, name=tf_prefix + joint) for joint in ROBOT_JOINTS
        ]
        goal_time_tolerance = Duration(sec=1, nanosec=0)
        goal_handle = self._passthrough_forward_joint_trajectory.send_goal(
            trajectory=trajectory,
            goal_time_tolerance=goal_time_tolerance,
            goal_tolerance=goal_tolerance,
        )
        self.assertTrue(goal_handle.accepted)
        if goal_handle.accepted:
            result = self._passthrough_forward_joint_trajectory.get_result(
                goal_handle, TIMEOUT_EXECUTE_TRAJECTORY
            )
            self.assertEqual(
                result.error_code, FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
            )

    def test_impossible_goal_time_tolerance_fails(self, tf_prefix):
        # Test impossible goal time
        self.assertTrue(
            self._controller_manager_interface.switch_controller(
                strictness=SwitchController.Request.BEST_EFFORT,
                activate_controllers=["passthrough_trajectory_controller"],
                deactivate_controllers=["scaled_joint_trajectory_controller"],
            ).ok
        )

        goal_tolerance = [
            JointTolerance(position=0.01, name=tf_prefix + joint) for joint in ROBOT_JOINTS
        ]
        goal_time_tolerance = Duration(sec=0, nanosec=10)
        trajectory = JointTrajectory(
            points=[
                JointTrajectoryPoint(positions=pos, time_from_start=times)
                for (times, pos) in TEST_TRAJECTORY
            ],
            joint_names=[tf_prefix + joint for joint in ROBOT_JOINTS],
        )
        goal_handle = self._passthrough_forward_joint_trajectory.send_goal(
            trajectory=trajectory,
            goal_time_tolerance=goal_time_tolerance,
            goal_tolerance=goal_tolerance,
        )
        self.assertTrue(goal_handle.accepted)
        if goal_handle.accepted:
            result = self._passthrough_forward_joint_trajectory.get_result(
                goal_handle, TIMEOUT_EXECUTE_TRAJECTORY
            )
            self.assertEqual(
                result.error_code, FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
            )
        self.assertTrue(
            self._controller_manager_interface.switch_controller(
                strictness=SwitchController.Request.BEST_EFFORT,
                deactivate_controllers=["passthrough_trajectory_controller"],
                activate_controllers=["scaled_joint_trajectory_controller"],
            ).ok
        )
