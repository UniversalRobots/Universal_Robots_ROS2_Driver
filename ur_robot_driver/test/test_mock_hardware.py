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

import launch_testing
import pytest
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import SwitchController
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


sys.path.append(os.path.dirname(__file__))
from test_common import (  # noqa: E402
    ActionInterface,
    ControllerManagerInterface,
    IoStatusInterface,
    ConfigurationInterface,
    generate_mock_hardware_test_description,
    ROBOT_JOINTS,
    TIMEOUT_EXECUTE_TRAJECTORY,
)


@pytest.mark.launch_test
@launch_testing.parametrize("tf_prefix, use_mock_hardware", [("", "true"), ("my_ur_", "true")])
def generate_test_description(tf_prefix, use_mock_hardware):
    return generate_mock_hardware_test_description(
        tf_prefix=tf_prefix, use_mock_hardware=use_mock_hardware
    )


class RobotDriverTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls, use_mock_hardware):
        # Initialize the ROS context
        rclpy.init()
        cls.node = Node("mock_hardware_test")
        time.sleep(1)
        cls.mock_hardware = use_mock_hardware == "true"
        cls.init_robot(cls)

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        cls.node.destroy_node()
        rclpy.shutdown()

    def init_robot(self):
        self._dashboard_interface = None
        self._controller_manager_interface = ControllerManagerInterface(self.node)
        self._io_status_controller_interface = IoStatusInterface(self.node)
        self._configuration_controller_interface = ConfigurationInterface(self.node)

        self._scaled_follow_joint_trajectory = ActionInterface(
            self.node,
            "/scaled_joint_trajectory_controller/follow_joint_trajectory",
            FollowJointTrajectory,
        )

    def setUp(self):
        time.sleep(1)
        self.assertTrue(self._io_status_controller_interface.resend_robot_program().success)

    #
    # Test functions
    #

    def test_get_robot_software_version(self):
        self.assertEqual(
            self._configuration_controller_interface.get_robot_software_version().major, 1
        )

    def test_start_scaled_jtc_controller(self):
        self.assertTrue(
            self._controller_manager_interface.switch_controller(
                strictness=SwitchController.Request.BEST_EFFORT,
                activate_controllers=["scaled_joint_trajectory_controller"],
            ).ok
        )

    def test_trajectory(self, tf_prefix):
        """Test robot movement."""
        # Construct test trajectory
        test_trajectory = [
            (Duration(sec=6, nanosec=0), [0.0 for j in ROBOT_JOINTS]),
            (Duration(sec=9, nanosec=0), [-0.5 for j in ROBOT_JOINTS]),
            (Duration(sec=12, nanosec=0), [-1.0 for j in ROBOT_JOINTS]),
        ]

        trajectory = JointTrajectory(
            joint_names=[tf_prefix + joint for joint in ROBOT_JOINTS],
            points=[
                JointTrajectoryPoint(positions=test_pos, time_from_start=test_time)
                for (test_time, test_pos) in test_trajectory
            ],
        )

        # Sending trajectory goal
        logging.info("Sending simple goal")
        goal_handle = self._scaled_follow_joint_trajectory.send_goal(trajectory=trajectory)
        self.assertTrue(goal_handle.accepted)

        # Verify execution
        result = self._scaled_follow_joint_trajectory.get_result(
            goal_handle, TIMEOUT_EXECUTE_TRAJECTORY
        )
        self.assertEqual(result.error_code, FollowJointTrajectory.Result.SUCCESSFUL)

    def test_illegal_trajectory(self, tf_prefix):
        """
        Test trajectory server.

        This is more of a validation test that the testing suite does the right thing
        """
        # Construct test trajectory, the second point wrongly starts before the first
        test_trajectory = [
            (Duration(sec=6, nanosec=0), [0.0 for j in ROBOT_JOINTS]),
            (Duration(sec=3, nanosec=0), [-0.5 for j in ROBOT_JOINTS]),
        ]

        trajectory = JointTrajectory(
            joint_names=[tf_prefix + joint for joint in ROBOT_JOINTS],
            points=[
                JointTrajectoryPoint(positions=test_pos, time_from_start=test_time)
                for (test_time, test_pos) in test_trajectory
            ],
        )

        # Send illegal goal
        logging.info("Sending illegal goal")
        goal_handle = self._scaled_follow_joint_trajectory.send_goal(
            trajectory=trajectory,
        )

        # Verify the failure is correctly detected
        self.assertFalse(goal_handle.accepted)

    def test_trajectory_scaled(self, tf_prefix):
        """Test robot movement."""
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
