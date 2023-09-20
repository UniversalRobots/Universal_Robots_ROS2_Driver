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


import time
import unittest

import launch_testing
import pytest
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import SwitchController
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare
from launch_testing.actions import ReadyToTest
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ur_dashboard_msgs.msg import RobotMode
from ur_dashboard_msgs.srv import GetRobotMode
from ur_msgs.msg import IOStates
from ur_msgs.srv import SetIO

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


@pytest.mark.launch_test
@launch_testing.parametrize(
    "tf_prefix",
    [(""), ("my_ur_")],
)
def generate_test_description(tf_prefix):
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5e",
            description="Type/series of used UR robot.",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20"],
        )
    )

    ur_type = LaunchConfiguration("ur_type")

    robot_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"]
            )
        ),
        launch_arguments={
            "robot_ip": "192.168.56.101",
            "ur_type": ur_type,
            "launch_rviz": "false",
            "controller_spawner_timeout": str(TIMEOUT_WAIT_SERVICE_INITIAL),
            "initial_joint_controller": "scaled_joint_trajectory_controller",
            "headless_mode": "true",
            "launch_dashboard_client": "false",
            "start_joint_controller": "false",
            "tf_prefix": tf_prefix,
        }.items(),
    )
    ursim = ExecuteProcess(
        cmd=[
            PathJoinSubstitution(
                [
                    FindPackagePrefix("ur_client_library"),
                    "lib",
                    "ur_client_library",
                    "start_ursim.sh",
                ]
            ),
            " ",
            "-m ",
            ur_type,
        ],
        name="start_ursim",
        output="screen",
    )
    wait_dashboard_server = ExecuteProcess(
        cmd=[
            PathJoinSubstitution(
                [FindPackagePrefix("ur_robot_driver"), "bin", "wait_dashboard_server.sh"]
            )
        ],
        name="wait_dashboard_server",
        output="screen",
    )
    driver_starter = RegisterEventHandler(
        OnProcessExit(target_action=wait_dashboard_server, on_exit=robot_driver)
    )

    return LaunchDescription(
        declared_arguments + [ReadyToTest(), wait_dashboard_server, ursim, driver_starter]
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

        # Wait longer for the first service clients:
        #  - The robot has to start up
        #  - The controller_manager has to start
        #  - The controllers need to load and activate
        service_interfaces_initial = {
            "/dashboard_client/power_on": Trigger,
            "/controller_manager/switch_controller": SwitchController,
            "/io_and_status_controller/set_io": SetIO,
        }
        self.service_clients = {
            srv_name: waitForService(
                self.node, srv_name, srv_type, timeout=TIMEOUT_WAIT_SERVICE_INITIAL
            )
            for (srv_name, srv_type) in service_interfaces_initial.items()
        }

        # Connect to the rest of the required interfaces
        service_interfaces = {
            "/dashboard_client/brake_release": Trigger,
            "/dashboard_client/stop": Trigger,
            "/dashboard_client/get_robot_mode": GetRobotMode,
            "/controller_manager/switch_controller": SwitchController,
            "/io_and_status_controller/resend_robot_program": Trigger,
        }
        self.service_clients.update(
            {
                srv_name: waitForService(self.node, srv_name, srv_type)
                for (srv_name, srv_type) in service_interfaces.items()
            }
        )

        action_interfaces = {
            "/scaled_joint_trajectory_controller/follow_joint_trajectory": FollowJointTrajectory
        }
        self.action_clients = {
            action_name: waitForAction(self.node, action_name, action_type)
            for (action_name, action_type) in action_interfaces.items()
        }

    def setUp(self):
        # Start robot
        empty_req = Trigger.Request()
        self.call_service("/dashboard_client/power_on", empty_req)
        self.call_service("/dashboard_client/brake_release", empty_req)
        time.sleep(1)
        robot_mode_resp = self.call_service(
            "/dashboard_client/get_robot_mode", GetRobotMode.Request()
        )
        self.assertEqual(robot_mode_resp.robot_mode.mode, RobotMode.RUNNING)
        self.call_service("/dashboard_client/stop", empty_req)
        time.sleep(1)
        self.call_service("/io_and_status_controller/resend_robot_program", empty_req)

    #
    # Test functions
    #

    def test_start_scaled_jtc_controller(self):
        req = SwitchController.Request()
        req.strictness = SwitchController.Request.BEST_EFFORT
        req.start_controllers = ["scaled_joint_trajectory_controller"]
        result = self.call_service("/controller_manager/switch_controller", req)

        self.assertEqual(result.ok, True)

    def test_set_io(self):
        """Test to set an IO and check whether it has been set."""
        # Create io callback to verify result
        io_msg = None

        def io_msg_cb(msg):
            nonlocal io_msg
            io_msg = msg

        io_states_sub = self.node.create_subscription(
            IOStates,
            "/io_and_status_controller/io_states",
            io_msg_cb,
            rclpy.qos.qos_profile_system_default,
        )

        # Set pin 0 to 1.0
        test_pin = 0

        set_io_req = SetIO.Request()
        set_io_req.fun = 1
        set_io_req.pin = test_pin
        set_io_req.state = 1.0

        self.node.get_logger().info(f"Setting pin {test_pin} to {set_io_req.state}")
        self.call_service("/io_and_status_controller/set_io", set_io_req)

        # Wait until the pin state has changed
        pin_state = False
        end_time = time.time() + 5
        while not pin_state and time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if io_msg is not None:
                pin_state = io_msg.digital_out_states[test_pin].state

        self.assertEqual(pin_state, set_io_req.state)

        # Set pin 0 to 0.0
        set_io_req.state = 0.0
        self.node.get_logger().info(f"Setting pin {test_pin} to {set_io_req.state}")
        self.call_service("/io_and_status_controller/set_io", set_io_req)

        # Wait until the pin state has changed back
        end_time = time.time() + 5
        while pin_state and time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if io_msg is not None:
                pin_state = io_msg.digital_out_states[test_pin].state

        self.assertEqual(pin_state, set_io_req.state)

        # Clean up io subscription
        self.node.destroy_subscription(io_states_sub)

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
        self.node.get_logger().info("Sending simple goal")
        goal_response = self.call_action(
            "/scaled_joint_trajectory_controller/follow_joint_trajectory",
            FollowJointTrajectory.Goal(trajectory=trajectory),
        )
        self.assertEqual(goal_response.accepted, True)

        # Verify execution
        result = self.get_result(
            "/scaled_joint_trajectory_controller/follow_joint_trajectory",
            goal_response,
            TIMEOUT_EXECUTE_TRAJECTORY,
        )
        self.assertEqual(result.error_code, FollowJointTrajectory.Result.SUCCESSFUL)
        self.node.get_logger().info("Received result SUCCESSFUL")

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
        self.node.get_logger().info("Sending illegal goal")
        goal_response = self.call_action(
            "/scaled_joint_trajectory_controller/follow_joint_trajectory",
            FollowJointTrajectory.Goal(trajectory=trajectory),
        )

        # Verify the failure is correctly detected
        self.assertEqual(goal_response.accepted, False)
        self.node.get_logger().info("Goal response REJECTED")

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

        goal = FollowJointTrajectory.Goal(trajectory=trajectory)

        # TODO: uncomment when JTC starts taking into account goal_time_tolerance from goal message
        # see https://github.com/ros-controls/ros2_controllers/issues/249
        # self.node.get_logger().info("Sending scaled goal without time restrictions")
        self.node.get_logger().info("Sending goal for robot to follow")
        goal_response = self.call_action(
            "/scaled_joint_trajectory_controller/follow_joint_trajectory", goal
        )

        self.assertEqual(goal_response.accepted, True)

        if goal_response.accepted:
            result = self.get_result(
                "/scaled_joint_trajectory_controller/follow_joint_trajectory",
                goal_response,
                TIMEOUT_EXECUTE_TRAJECTORY,
            )
            self.assertIn(
                result.error_code,
                (FollowJointTrajectory.Result.SUCCESSFUL,),
            )

            self.node.get_logger().info("Received result")

    def test_trajectory_scaled_aborts_on_violation(self, tf_prefix):
        """Test that the robot correctly aborts the trajectory when the constraints are violated."""
        # Construct test trajectory
        test_trajectory = [
            (Duration(sec=6, nanosec=0), [0.0 for j in ROBOT_JOINTS]),
            (
                Duration(sec=6, nanosec=50000000),
                [-1.0 for j in ROBOT_JOINTS],
            ),  # physically unfeasible
            (Duration(sec=8, nanosec=0), [-1.5 for j in ROBOT_JOINTS]),  # physically unfeasible
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

        goal = FollowJointTrajectory.Goal(trajectory=trajectory)

        self.node.get_logger().info("Sending goal for robot to follow")
        goal_response = self.call_action(
            "/scaled_joint_trajectory_controller/follow_joint_trajectory", goal
        )

        self.assertEqual(goal_response.accepted, True)

        if goal_response.accepted:
            result = self.get_result(
                "/scaled_joint_trajectory_controller/follow_joint_trajectory",
                goal_response,
                TIMEOUT_EXECUTE_TRAJECTORY,
            )
            self.assertIn(
                result.error_code,
                (FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED,),
            )
            self.node.get_logger().info("Received result")

            # self.node.get_logger().info(f"Joint state before sleep {last_joint_state.position}")
            state_when_aborted = last_joint_state

            # This section is to make sure the robot stopped moving once the trajectory was aborted
            time.sleep(2.0)
            # Ugly workaround since we want to wait for a joint state in the same thread
            while last_joint_state == state_when_aborted:
                rclpy.spin_once(self.node)
            state_after_sleep = last_joint_state
            self.node.get_logger().info(f"before: {state_when_aborted.position.tolist()}")
            self.node.get_logger().info(f"after: {state_after_sleep.position.tolist()}")
            self.assertTrue(
                all(
                    [
                        abs(a - b) < 0.2
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

    #
    # Utility functions
    #

    def call_service(self, srv_name, request):
        self.node.get_logger().info(f"Calling service '{srv_name}' with request {request}")
        future = self.service_clients[srv_name].call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is not None:
            self.node.get_logger().info(f"Received result {future.result()}")
            return future.result()
        else:
            raise Exception(f"Exception while calling service: {future.exception()}")

    def call_action(self, action_name, goal):
        self.node.get_logger().info(f"Sending goal to action server '{action_name}'")
        future = self.action_clients[action_name].send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, future)

        if future.result() is not None:
            return future.result()
        else:
            raise Exception(f"Exception while calling action: {future.exception()}")

    def get_result(self, action_name, goal_response, timeout):
        self.node.get_logger().info(
            f"Waiting for result for action server '{action_name}' (timeout: {timeout} seconds)"
        )
        future_res = self.action_clients[action_name]._get_result_async(goal_response)
        rclpy.spin_until_future_complete(self.node, future_res, timeout_sec=timeout)

        if future_res.result() is not None:
            self.node.get_logger().info(f"Received result {future_res.result().result}")
            return future_res.result().result
        else:
            raise Exception(f"Exception while calling action: {future_res.exception()}")


def waitForService(node, srv_name, srv_type, timeout=TIMEOUT_WAIT_SERVICE):
    client = node.create_client(srv_type, srv_name)
    if client.wait_for_service(timeout) is False:
        raise Exception(f"Could not reach service '{srv_name}' within timeout of {timeout}")

    node.get_logger().info(f"Successfully connected to service '{srv_name}'")
    return client


def waitForAction(node, action_name, action_type, timeout=TIMEOUT_WAIT_ACTION):
    client = ActionClient(node, action_type, action_name)
    if client.wait_for_server(timeout) is False:
        raise Exception(
            f"Could not reach action server '{action_name}' within timeout of {timeout}"
        )

    node.get_logger().info(f"Successfully connected to action '{action_name}'")
    return client
