#!/usr/bin/env python
# Copyright 2019, FZI Forschungszentrum Informatik
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
from rclpy.node import Node

from ur_msgs.srv import SetIO
from ur_msgs.msg import IOStates
from ur_dashboard_msgs.srv import GetRobotMode
from ur_dashboard_msgs.msg import RobotMode
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
            "ur_type", default_value="ur5e", description="Type/series of used UR robot."
        )
    )

    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    dir_path = os.path.dirname(os.path.realpath(__file__))

    launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([dir_path, "/../../ur_bringup/launch/ur_control.launch.py"]),
        launch_arguments={"robot_ip": robot_ip, "ur_type": ur_type, "launch_rviz": "false"}.items(),
    )

    return LaunchDescription(
        declared_arguments + [launch_testing.actions.ReadyToTest(), launch_file]
    )


class IOTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context
        rclpy.init()
        cls.node = Node("ur_robot_driver_integrations_test")
        cls.init_robot(cls)

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        cls.node.destroy_node()
        rclpy.shutdown()

    def init_robot(self):
        # Initialize clients
        self.set_io_client = self.node.create_client(SetIO, "/io_and_status_controller/set_io")
        if self.set_io_client.wait_for_service(10) is False:
            raise Exception(
                "Could not reach set IO service, make sure that the driver is actually running"
            )

        self.power_on_client = self.node.create_client(Trigger, "/dashboard_client/power_on")
        if self.power_on_client.wait_for_service(10) is False:
            raise Exception(
                "Could not reach power on service, make sure that the driver is actually running"
            )

        self.brake_release_client = self.node.create_client(
            Trigger, "/dashboard_client/brake_release"
        )
        if self.brake_release_client.wait_for_service(10) is False:
            raise Exception(
                "Could not reach brake release service, make sure that the driver is actually running"
            )

        self.get_robot_mode_client = self.node.create_client(
            GetRobotMode, "/dashboard_client/get_robot_mode"
        )
        if self.get_robot_mode_client.wait_for_service(10) is False:
            raise Exception(
                "Could not reach get robot mode service, make sure that the driver is actually running"
            )

    def test_switch_on(self):
        """Test power on a robot."""
        empty_req = Trigger.Request()
        get_robot_mode_req = GetRobotMode.Request()

        self.call_service(self.power_on_client, empty_req)
        end_time = time.time() + 10
        mode = RobotMode.DISCONNECTED
        while mode not in (RobotMode.IDLE, RobotMode.RUNNING) and time.time() < end_time:
            result = self.call_service(self.get_robot_mode_client, get_robot_mode_req)
            mode = result.robot_mode.mode

        self.assertIn(mode, (RobotMode.IDLE, RobotMode.RUNNING))

        self.call_service(self.brake_release_client, empty_req)
        end_time = time.time() + 10
        while mode != RobotMode.RUNNING and time.time() < end_time:
            result = self.call_service(self.get_robot_mode_client, get_robot_mode_req)
            mode = result.robot_mode.mode

        self.assertEqual(mode, RobotMode.RUNNING)

    def test_set_io(self):
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
