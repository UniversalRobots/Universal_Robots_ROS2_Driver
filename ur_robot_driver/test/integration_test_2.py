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


import unittest
import os
import time
import pytest

import launch_testing
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from controller_manager_msgs.srv import SwitchController
from ur_msgs.srv import SetIO
from ur_msgs.msg import IOStates

from std_srvs.srv import Trigger


@pytest.mark.launch_test
def generate_test_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.56.101",
            description="IP address by which the robot can be reached.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5e",
            description="Type/series of used UR robot.",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e"],
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="scaled_joint_trajectory_controller",
            description="Type/series of used UR robot.",
        )
    )

    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    dir_path = os.path.dirname(os.path.realpath(__file__))

    launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([dir_path, "/../launch/ur_control.launch.py"]),
        launch_arguments={
            "robot_ip": robot_ip,
            "ur_type": ur_type,
            "launch_rviz": "false",
            "initial_joint_controller": initial_joint_controller,
            "headless_mode": "true",
            "launch_dashboard_client": "false",
            "start_joint_controller": "false",
            "ci_testing": "true",
        }.items(),
    )

    ld = []
    ld += declared_arguments
    ld += [launch_testing.actions.ReadyToTest(), launch_file]

    return LaunchDescription(ld)


class URTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context
        rclpy.init()
        cls.node = Node("ur_robot_driver_integration_test_2")
        time.sleep(1)
        cls.init_robot(cls)

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        cls.node.destroy_node()
        rclpy.shutdown()

    def init_robot(self):

        # Initialize clients and test service appearance
        self.switch_controller_client = self.node.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )
        if self.switch_controller_client.wait_for_service(10) is False:
            raise Exception(
                "Could not reach switch controller service, make sure that the controller_manager is actually running"
            )

        self.set_io_client = self.node.create_client(SetIO, "/io_and_status_controller/set_io")
        if self.set_io_client.wait_for_service(10) is False:
            raise Exception(
                "Could not reach set IO service, make sure that the driver is actually running"
            )

        self.resend_robot_program_client = self.node.create_client(
            Trigger, "/io_and_status_controller/resend_robot_program"
        )
        if self.resend_robot_program_client.wait_for_service(10) is False:
            raise Exception(
                "Could not reach stop service, make sure that the driver is actually running"
            )
        # test action appearance
        self.jtc_action_client = ActionClient(
            self.node,
            FollowJointTrajectory,
            "/scaled_joint_trajectory_controller/follow_joint_trajectory",
        )
        if self.jtc_action_client.wait_for_server(10) is False:
            raise Exception(
                "Could not reach /scaled_joint_trajectory_controller/follow_joint_trajectory action server,"
                "make sure that controller is active (load + start)"
            )

    def test_2_manager(self):
        req = SwitchController.Request()
        req.strictness = SwitchController.Request.BEST_EFFORT
        req.start_controllers = ["scaled_joint_trajectory_controller"]
        result = self.call_service(self.switch_controller_client, req)

        self.assertEqual(result.ok, True)

    def test_4_set_io(self):
        """Test to set an IO and check whether it has been set."""
        self.io_msg = None
        io_states_sub = self.node.create_subscription(
            IOStates,
            "/io_and_status_controller/io_states",
            self.io_msg_cb,
            rclpy.qos.qos_profile_system_default,
        )

        pin = 0
        set_io_req = SetIO.Request()
        set_io_req.fun = 1
        set_io_req.pin = pin
        set_io_req.state = 1.0

        self.call_service(self.set_io_client, set_io_req)
        pin_state = False

        end_time = time.time() + 5
        while not pin_state and time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.io_msg is not None:
                pin_state = self.io_msg.digital_out_states[pin].state

        self.assertEqual(pin_state, 1)

        set_io_req.state = 0.0
        self.call_service(self.set_io_client, set_io_req)

        end_time = time.time() + 5
        while pin_state and time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.io_msg is not None:
                pin_state = self.io_msg.digital_out_states[pin].state

        self.assertEqual(pin_state, 0)

        self.node.destroy_subscription(io_states_sub)

    def io_msg_cb(self, msg):
        self.io_msg = msg

    def call_service(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is not None:
            return future.result()
        else:
            raise Exception(f"Exception while calling service: {future.exception()}")

    def call_action(self, ac_client, g):
        future = ac_client.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, future)

        if future.result() is not None:
            return future.result()
        else:
            raise Exception(f"Exception while calling action: {future.exception()}")

    def get_result(self, ac_client, goal_response):

        future_res = ac_client._get_result_async(goal_response)
        rclpy.spin_until_future_complete(self.node, future_res)
        if future_res.result() is not None:
            return future_res.result().result
        else:
            raise Exception(f"Exception while calling action: {future_res.exception()}")

    def test_5_trajectory(self):
        """Test robot movement."""
        goal = FollowJointTrajectory.Goal()

        goal.trajectory.joint_names = [
            "elbow_joint",
            "shoulder_lift_joint",
            "shoulder_pan_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        position_list = [[0.0 for i in range(6)]]
        position_list.append([-0.5 for i in range(6)])
        position_list.append([-1.0 for i in range(6)])
        duration_list = [
            Duration(sec=6, nanosec=0),
            Duration(sec=9, nanosec=0),
            Duration(sec=12, nanosec=0),
        ]

        for i, position in enumerate(position_list):
            point = JointTrajectoryPoint()
            point.positions = position
            point.time_from_start = duration_list[i]
            goal.trajectory.points.append(point)

        self.node.get_logger().info("Sending simple goal")

        goal_response = self.call_action(self.jtc_action_client, goal)

        self.assertEqual(goal_response.accepted, True)

        if goal_response.accepted:
            result = self.get_result(self.jtc_action_client, goal_response)
            self.assertEqual(result.error_code, FollowJointTrajectory.Result.SUCCESSFUL)
            self.node.get_logger().info("Received result SUCCESSFUL")

    def test_6_trajectory_illegal(self):
        """Test trajectory server."""
        """This is more of a validation test that the testing suite does the right thing."""
        goal = FollowJointTrajectory.Goal()

        goal.trajectory.joint_names = [
            "elbow_joint",
            "shoulder_lift_joint",
            "shoulder_pan_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        position_list = [[0.0 for i in range(6)]]
        position_list.append([-0.5 for i in range(6)])
        # Create illegal goal by making the second point come earlier than the first
        duration_list = [Duration(sec=6, nanosec=0), Duration(sec=3, nanosec=0)]

        for i, position in enumerate(position_list):
            point = JointTrajectoryPoint()
            point.positions = position
            point.time_from_start = duration_list[i]
            goal.trajectory.points.append(point)

        self.node.get_logger().info("Sending illegal goal")

        goal_response = self.call_action(self.jtc_action_client, goal)

        self.assertEqual(goal_response.accepted, False)

        self.node.get_logger().info("Goal response REJECTED")

    def test_7_trajectory_scaled(self):
        """Test robot movement."""
        goal = FollowJointTrajectory.Goal()

        goal.trajectory.joint_names = [
            "elbow_joint",
            "shoulder_lift_joint",
            "shoulder_pan_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        position_list = [[0.0 for i in range(6)]]
        position_list.append([-1.0 for i in range(6)])
        duration_list = [Duration(sec=6, nanosec=0), Duration(sec=6, nanosec=500000000)]

        for i, position in enumerate(position_list):
            point = JointTrajectoryPoint()
            point.positions = position
            point.time_from_start = duration_list[i]
            goal.trajectory.points.append(point)

        # TODO: uncomment when JTC starts taking into account goal_time_tolerance from goal message
        # see https://github.com/ros-controls/ros2_controllers/issues/249
        # self.node.get_logger().info("Sending scaled goal without time restrictions")
        self.node.get_logger().info("Sending goal for robot to follow")

        goal_response = self.call_action(self.jtc_action_client, goal)

        self.assertEqual(goal_response.accepted, True)

        if goal_response.accepted:
            result = self.get_result(self.jtc_action_client, goal_response)
            self.assertIn(
                result.error_code,
                (
                    FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED,
                    FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED,
                    FollowJointTrajectory.Result.SUCCESSFUL,
                ),
            )

            self.node.get_logger().info("Received result")

        # TODO: uncomment when JTC starts taking into account goal_time_tolerance from goal message
        # see https://github.com/ros-controls/ros2_controllers/issues/249
        # Now do the same again, but with a goal time constraint
        # self.node.get_logger().info("Sending scaled goal with time restrictions")
        #
        # goal.goal_time_tolerance = Duration(nanosec=10000000)
        # goal_response = self.call_action(self.jtc_action_client, goal)
        #
        # self.assertEqual(goal_response.accepted, True)
        #
        # if goal_response.accepted:
        #     result = self.get_result(self.jtc_action_client, goal_response)
        #     self.assertEqual(result.error_code, FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED)
        #     self.node.get_logger().info("Received result GOAL_TOLERANCE_VIOLATED")
