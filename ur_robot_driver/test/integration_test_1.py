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

from ur_dashboard_msgs.srv import GetLoadedProgram, GetProgramState, GetRobotMode
from ur_dashboard_msgs.srv import IsProgramRunning
from ur_dashboard_msgs.srv import Load
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

    robot_ip = LaunchConfiguration("robot_ip")
    dir_path = os.path.dirname(os.path.realpath(__file__))

    launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [dir_path, "/../../ur_bringup/launch/ur_dashboard_client.launch.py"]
        ),
        launch_arguments={
            "robot_ip": robot_ip,
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
        cls.node = Node("ur_robot_driver_integration_test_1")
        cls.init_robot(cls)

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        cls.node.destroy_node()
        rclpy.shutdown()

    def init_robot(self):

        # Initialize clients
        self.power_on_client = self.node.create_client(Trigger, "/dashboard_client/power_on")
        if self.power_on_client.wait_for_service(10) is False:
            raise Exception(
                "Could not reach power on service, make sure that the driver is actually running"
            )

        self.power_off_client = self.node.create_client(Trigger, "/dashboard_client/power_off")
        if self.power_off_client.wait_for_service(10) is False:
            raise Exception(
                "Could not reach power off service, make sure that the driver is actually running"
            )

        self.brake_release_client = self.node.create_client(
            Trigger, "/dashboard_client/brake_release"
        )
        if self.brake_release_client.wait_for_service(10) is False:
            raise Exception(
                "Could not reach brake release service, make sure that the driver is actually running"
            )

        self.unlock_protective_stop_client = self.node.create_client(
            Trigger, "/dashboard_client/unlock_protective_stop"
        )
        if self.unlock_protective_stop_client.wait_for_service(10) is False:
            raise Exception(
                "Could not reach unlock protective stop service, make sure that the driver is actually running"
            )

        self.restart_safety_client = self.node.create_client(
            Trigger, "/dashboard_client/restart_safety"
        )
        if self.restart_safety_client.wait_for_service(10) is False:
            raise Exception(
                "Could not reach restart safety service, make sure that the driver is actually running"
            )

        self.get_robot_mode_client = self.node.create_client(
            GetRobotMode, "/dashboard_client/get_robot_mode"
        )
        if self.get_robot_mode_client.wait_for_service(10) is False:
            raise Exception(
                "Could not reach get robot mode service, make sure that the driver is actually running"
            )

        self.load_installation_client = self.node.create_client(
            Load, "/dashboard_client/load_installation"
        )
        if self.load_installation_client.wait_for_service(10) is False:
            raise Exception(
                "Could not reach load installation service, make sure that the driver is actually running"
            )

        self.load_program_client = self.node.create_client(Load, "/dashboard_client/load_program")
        if self.load_program_client.wait_for_service(10) is False:
            raise Exception(
                "Could not reach load program service, make sure that the driver is actually running"
            )

        self.close_popup_client = self.node.create_client(Trigger, "/dashboard_client/close_popup")
        if self.load_program_client.wait_for_service(10) is False:
            raise Exception(
                "Could not reach close popup service, make sure that the driver is actually running"
            )

        self.get_loaded_program_client = self.node.create_client(
            GetLoadedProgram, "/dashboard_client/get_loaded_program"
        )
        if self.get_loaded_program_client.wait_for_service(10) is False:
            raise Exception(
                "Could not reach get loaded program service, make sure that the driver is actually running"
            )

        self.get_program_state_client = self.node.create_client(
            GetProgramState, "/dashboard_client/program_state"
        )
        if self.get_program_state_client.wait_for_service(10) is False:
            raise Exception(
                "Could not reach program state service, make sure that the driver is actually running"
            )

        self.is_program_running_client = self.node.create_client(
            IsProgramRunning, "/dashboard_client/program_running"
        )
        if self.is_program_running_client.wait_for_service(10) is False:
            raise Exception(
                "Could not reach program running service, make sure that the driver is actually running"
            )

        self.play_program_client = self.node.create_client(Trigger, "/dashboard_client/play")
        if self.play_program_client.wait_for_service(10) is False:
            raise Exception(
                "Could not reach play service, make sure that the driver is actually running"
            )

        self.stop_program_client = self.node.create_client(Trigger, "/dashboard_client/stop")
        if self.stop_program_client.wait_for_service(10) is False:
            raise Exception(
                "Could not reach stop service, make sure that the driver is actually running"
            )

    def test_1_switch_on(self):
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

    def call_service(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is not None:
            return future.result()
        else:
            raise Exception(f"Exception while calling service: {future.exception()}")
