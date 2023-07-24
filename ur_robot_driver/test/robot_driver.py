#!/usr/bin/env python
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
import time

import pytest
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ur_msgs.msg import IOStates
from ur_msgs.srv import SetIO

from .conftest import (
    call_action,
    call_service,
    get_action_result,
    wait_for_action,
    wait_for_service,
)
from .robot_launch_descriptions import *  # noqa: E402, F403

TIMEOUT_WAIT_SERVICE = 10
TIMEOUT_WAIT_SERVICE_INITIAL = 60
TIMEOUT_WAIT_ACTION = 10
TIMEOUT_EXECUTE_TRAJECTORY = 30

ROBOT_JOINTS = [
    "elbow_joint",
    "shoulder_lift_joint",
    "shoulder_pan_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


@pytest.mark.launch(fixture=launch_robot_driver)  # noqa: F405
@pytest.mark.parametrize("node", [("robot_driver_test")], indirect=True)
class TestRobotDriver:
    def test_start_scaled_jtc_controller(
        self, dashboard_interface, controller_manager_interface, robot_program_running
    ):
        controller_manager_interface.switch_controller(
            activate_controllers=["scaled_joint_trajectory_controller"]
        )

    def test_set_io(self, node, robot_program_running):
        """Test to set an IO and check whether it has been set."""
        # Create io callback to verify result
        io_msg = None

        def io_msg_cb(msg):
            nonlocal io_msg
            io_msg = msg

        io_states_sub = node.create_subscription(
            IOStates,
            "/io_and_status_controller/io_states",
            io_msg_cb,
            rclpy.qos.qos_profile_system_default,
        )

        set_io_client = wait_for_service(node, "/io_and_status_controller/set_io", SetIO)

        # Set pin 0 to 1.0
        test_pin = 0

        set_io_req = SetIO.Request()
        set_io_req.fun = 1
        set_io_req.pin = test_pin
        set_io_req.state = 1.0

        logging.info("Setting pin %d to %f", set_io_req.pin, set_io_req.state)
        call_service(node, set_io_client, set_io_req)

        # Wait until the pin state has changed
        pin_state = False
        end_time = time.time() + 5
        while not pin_state and time.time() < end_time:
            rclpy.spin_once(node, timeout_sec=0.1)
            if io_msg is not None:
                pin_state = io_msg.digital_out_states[test_pin].state

        assert pin_state == set_io_req.state

        # Set pin 0 to 0.0
        set_io_req.state = 0.0
        logging.info("Setting pin %d to %f", set_io_req.pin, set_io_req.state)
        call_service(node, set_io_client, set_io_req)

        # Wait until the pin state has changed back
        end_time = time.time() + 5
        while pin_state and time.time() < end_time:
            rclpy.spin_once(node, timeout_sec=0.1)
            if io_msg is not None:
                pin_state = io_msg.digital_out_states[test_pin].state

        assert pin_state == set_io_req.state

        # Clean up io subscription
        node.destroy_subscription(io_states_sub)

    def test_trajectory(self, node, robot_program_running):
        """Test robot movement."""
        # Construct test trajectory
        test_trajectory = [
            (Duration(sec=6, nanosec=0), [0.0 for j in ROBOT_JOINTS]),
            (Duration(sec=9, nanosec=0), [-0.5 for j in ROBOT_JOINTS]),
            (Duration(sec=12, nanosec=0), [-1.0 for j in ROBOT_JOINTS]),
        ]

        trajectory = JointTrajectory(
            joint_names=ROBOT_JOINTS,
            points=[
                JointTrajectoryPoint(positions=test_pos, time_from_start=test_time)
                for (test_time, test_pos) in test_trajectory
            ],
        )

        follow_joint_trajectory_client = wait_for_action(
            node,
            "/scaled_joint_trajectory_controller/follow_joint_trajectory",
            FollowJointTrajectory,
        )

        # Sending trajectory goal
        logging.info("Sending simple test trajectory")
        goal_response = call_action(
            node, follow_joint_trajectory_client, FollowJointTrajectory.Goal(trajectory=trajectory)
        )

        assert goal_response.accepted
        logging.info("Goal accepted")

        result = get_action_result(
            node, follow_joint_trajectory_client, goal_response, TIMEOUT_EXECUTE_TRAJECTORY
        )
        assert result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        logging.info("Successfully executed simple test trajectory")

    def test_illegal_trajectory(self, node):
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
            joint_names=ROBOT_JOINTS,
            points=[
                JointTrajectoryPoint(positions=test_pos, time_from_start=test_time)
                for (test_time, test_pos) in test_trajectory
            ],
        )

        follow_joint_trajectory_client = wait_for_action(
            node,
            "/scaled_joint_trajectory_controller/follow_joint_trajectory",
            FollowJointTrajectory,
        )

        # Send illegal goal
        logging.info("Sending illegal test trajectory")
        goal_response = call_action(
            node, follow_joint_trajectory_client, FollowJointTrajectory.Goal(trajectory=trajectory)
        )

        # Verify the failure is correctly detected
        assert not goal_response.accepted
        logging.info("Correctly rejected illegal trajectory")

    def test_trajectory_scaled(self, node):
        """Test robot movement."""
        # Construct test trajectory
        test_trajectory = [
            (Duration(sec=6, nanosec=0), [0.0 for j in ROBOT_JOINTS]),
            (Duration(sec=6, nanosec=500000000), [-1.0 for j in ROBOT_JOINTS]),
        ]

        trajectory = JointTrajectory(
            joint_names=ROBOT_JOINTS,
            points=[
                JointTrajectoryPoint(positions=test_pos, time_from_start=test_time)
                for (test_time, test_pos) in test_trajectory
            ],
        )

        follow_joint_trajectory_client = wait_for_action(
            node,
            "/scaled_joint_trajectory_controller/follow_joint_trajectory",
            FollowJointTrajectory,
        )

        # TODO: uncomment when JTC starts taking into account goal_time_tolerance from goal message
        # see https://github.com/ros-controls/ros2_controllers/issues/249
        # self.node.get_logger().info("Sending scaled goal without time restrictions")
        logging.info("Sending trajectory goal")
        goal_response = call_action(
            node, follow_joint_trajectory_client, FollowJointTrajectory.Goal(trajectory=trajectory)
        )

        assert goal_response.accepted
        logging.info("Goal accepted")

        result = get_action_result(
            node, follow_joint_trajectory_client, goal_response, TIMEOUT_EXECUTE_TRAJECTORY
        )
        assert result.error_code in (
            FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED,
            FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED,
            FollowJointTrajectory.Result.SUCCESSFUL,
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
