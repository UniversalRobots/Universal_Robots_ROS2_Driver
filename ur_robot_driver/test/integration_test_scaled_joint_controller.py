#!/usr/bin/env python3
# Copyright 2019, FZI Forschungszentrum Informatik
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
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import SwitchController
from rclpy.node import Node
from sensor_msgs.msg import JointState
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
    sjtc_trajectory_test,
    sjtc_illegal_trajectory_test,
)


@pytest.mark.launch_test
@launch_testing.parametrize("tf_prefix", [(""), ("my_ur_")])
def generate_test_description(tf_prefix):
    return generate_driver_test_description(tf_prefix=tf_prefix)


class SJTCTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context
        rclpy.init()
        cls.node = Node("sjtc_test")
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

        self._scaled_follow_joint_trajectory = ActionInterface(
            self.node,
            "/scaled_joint_trajectory_controller/follow_joint_trajectory",
            FollowJointTrajectory,
        )

        # Wait for all controllers needed below, as controller manager services might fail
        # e.g. when attempting to deactivate an unknown controller
        self._controller_manager_interface.wait_for_controller("passthrough_trajectory_controller")

    def setUp(self):
        self._dashboard_interface.start_robot()
        time.sleep(1)
        self.assertTrue(self._io_status_controller_interface.resend_robot_program().success)

        self._controller_manager_interface.wait_for_controller(
            "scaled_joint_trajectory_controller", "active"
        )

    #
    # Test functions
    #

    def test_start_scaled_jtc_controller(self):
        self.assertTrue(
            self._controller_manager_interface.switch_controller(
                strictness=SwitchController.Request.BEST_EFFORT,
                activate_controllers=["scaled_joint_trajectory_controller"],
            ).ok
        )

    def test_trajectory(self, tf_prefix):
        sjtc_trajectory_test(self, tf_prefix)

    def test_illegal_trajectory(self, tf_prefix):
        sjtc_illegal_trajectory_test(self, tf_prefix)

    def test_trajectory_scaled(self, tf_prefix):
        """Test robot movement."""
        self.assertTrue(
            self._controller_manager_interface.switch_controller(
                strictness=SwitchController.Request.BEST_EFFORT,
                deactivate_controllers=["passthrough_trajectory_controller"],
                activate_controllers=["scaled_joint_trajectory_controller"],
            ).ok
        )
        # Construct test trajectory
        test_trajectory = [
            (Duration(sec=6, nanosec=0), [0.0 for j in ROBOT_JOINTS]),
            (Duration(sec=6, nanosec=500000000), [-1.0 for j in ROBOT_JOINTS]),
        ]

        trajectory = JointTrajectory(
            joint_names=[tf_prefix + joint for joint in ROBOT_JOINTS],
            points=[
                JointTrajectoryPoint(positions=test_pos, time_from_start=test_time)
                for (test_time, test_pos) in test_trajectory
            ],
        )

        # Execute trajectory
        logging.info("Sending goal for robot to follow")
        goal_handle = self._scaled_follow_joint_trajectory.send_goal(trajectory=trajectory)
        self.assertTrue(goal_handle.accepted)

        # Verify execution
        result = self._scaled_follow_joint_trajectory.get_result(
            goal_handle,
            TIMEOUT_EXECUTE_TRAJECTORY,
        )
        self.assertEqual(result.error_code, FollowJointTrajectory.Result.SUCCESSFUL)

    def test_trajectory_scaled_aborts_on_violation(self, tf_prefix):
        """Test that the robot correctly aborts the trajectory when the constraints are violated."""
        # Construct test trajectory
        test_trajectory = [
            (Duration(sec=6, nanosec=0), [0.0 for j in ROBOT_JOINTS]),
            (
                Duration(sec=6, nanosec=50000000),
                [-1.0 for j in ROBOT_JOINTS],
            ),  # physically unfeasible
            (
                Duration(sec=8, nanosec=0),
                [-1.5 for j in ROBOT_JOINTS],
            ),  # physically unfeasible
        ]

        trajectory = JointTrajectory(
            joint_names=[tf_prefix + joint for joint in ROBOT_JOINTS],
            points=[
                JointTrajectoryPoint(positions=test_pos, time_from_start=test_time)
                for (test_time, test_pos) in test_trajectory
            ],
        )

        last_joint_state = None

        def js_cb(msg):
            nonlocal last_joint_state
            last_joint_state = msg

        joint_state_sub = self.node.create_subscription(JointState, "/joint_states", js_cb, 1)
        joint_state_sub  # prevent warning about unused variable

        # Send goal
        logging.info("Sending goal for robot to follow")
        goal_handle = self._scaled_follow_joint_trajectory.send_goal(trajectory=trajectory)
        self.assertTrue(goal_handle.accepted)

        # Get result
        result = self._scaled_follow_joint_trajectory.get_result(
            goal_handle,
            TIMEOUT_EXECUTE_TRAJECTORY,
        )
        self.assertEqual(result.error_code, FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED)

        state_when_aborted = last_joint_state

        # This section is to make sure the robot stopped moving once the trajectory was aborted
        time.sleep(2.0)
        # Ugly workaround since we want to wait for a joint state in the same thread
        while last_joint_state == state_when_aborted:
            rclpy.spin_once(self.node)
        state_after_sleep = last_joint_state

        logging.info("Joint states before sleep:\t %s", state_when_aborted.position.tolist())
        logging.info("Joint states after sleep:\t %s", state_after_sleep.position.tolist())

        self.assertTrue(
            all(
                [
                    abs(a - b) < 0.01
                    for a, b in zip(state_after_sleep.position, state_when_aborted.position)
                ]
            )
        )

    # TODO: uncomment when JTC starts taking into account goal_time_tolerance from goal message
    # see https://github.com/ros-controls/ros2_controllers/issues/249
    # Now do the same again, but with a goal time constraint
    # self.node.get_logger().info("Sending scaled goal with time restrictions")
    #
    # goal.goal_time_tolerance = Duration(nanosec=10000000)
    # goal_response = self.call_action("/scaled_joint_trajectory_controller/follow_joint_trajectory", goal)
    #
    # self.assertEqual(goal_response.accepted, True)
    #
    # if goal_response.accepted:
    #     result = self.get_result("/scaled_joint_trajectory_controller/follow_joint_trajectory", goal_response, TIMEOUT_EXECUTE_TRAJECTORY)
    #     self.assertEqual(result.error_code, FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED)
    #     self.node.get_logger().info("Received result GOAL_TOLERANCE_VIOLATED")
