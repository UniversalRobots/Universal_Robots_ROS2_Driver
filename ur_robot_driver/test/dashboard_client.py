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

import pytest
import rclpy
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare
from launch_testing.actions import ReadyToTest
from rclpy.node import Node
from std_srvs.srv import Trigger
from ur_dashboard_msgs.msg import RobotMode
from ur_dashboard_msgs.srv import (
    GetLoadedProgram,
    GetProgramState,
    GetRobotMode,
    IsProgramRunning,
    Load,
)

TIMEOUT_WAIT_SERVICE = 10
# If we download the docker image simultaneously to the tests, it can take quite some time until the
# dashboard server is reachable and usable.
TIMEOUT_WAIT_SERVICE_INITIAL = 120


@pytest.mark.launch_test
def generate_test_description():
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

    dashboard_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ur_robot_driver"),
                    "launch",
                    "ur_dashboard_client.launch.py",
                ]
            )
        ),
        launch_arguments={
            "robot_ip": "192.168.56.101",
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

    return LaunchDescription(declared_arguments + [ReadyToTest(), dashboard_client, ursim])


class DashboardClientTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context
        rclpy.init()
        cls.node = Node("dashboard_client_test")
        cls.init_robot(cls)

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        cls.node.destroy_node()
        rclpy.shutdown()

    def init_robot(self):

        # We wait longer for the first client, as the robot is still starting up
        power_on_client = waitForService(
            self.node, "/dashboard_client/power_on", Trigger, timeout=TIMEOUT_WAIT_SERVICE_INITIAL
        )

        # Connect to all other expected services
        dashboard_interfaces = {
            "power_off": Trigger,
            "brake_release": Trigger,
            "unlock_protective_stop": Trigger,
            "restart_safety": Trigger,
            "get_robot_mode": GetRobotMode,
            "load_installation": Load,
            "load_program": Load,
            "close_popup": Trigger,
            "get_loaded_program": GetLoadedProgram,
            "program_state": GetProgramState,
            "program_running": IsProgramRunning,
            "play": Trigger,
            "stop": Trigger,
        }
        self.dashboard_clients = {
            srv_name: waitForService(self.node, f"/dashboard_client/{srv_name}", srv_type)
            for (srv_name, srv_type) in dashboard_interfaces.items()
        }

        # Add first client to dict
        self.dashboard_clients["power_on"] = power_on_client

    #
    # Test functions
    #

    def test_switch_on(self):
        """Test power on a robot."""
        # Wait until the robot is booted completely
        end_time = time.time() + 10
        mode = RobotMode.DISCONNECTED
        while mode != RobotMode.POWER_OFF and time.time() < end_time:
            time.sleep(0.1)
            result = self.call_dashboard_service("get_robot_mode", GetRobotMode.Request())
            self.assertTrue(result.success)
            mode = result.robot_mode.mode

        # Power on robot
        self.assertTrue(self.call_dashboard_service("power_on", Trigger.Request()).success)

        # Wait until robot mode changes
        end_time = time.time() + 10
        mode = RobotMode.DISCONNECTED
        while mode not in (RobotMode.IDLE, RobotMode.RUNNING) and time.time() < end_time:
            time.sleep(0.1)
            result = self.call_dashboard_service("get_robot_mode", GetRobotMode.Request())
            self.assertTrue(result.success)
            mode = result.robot_mode.mode

        self.assertIn(mode, (RobotMode.IDLE, RobotMode.RUNNING))

        # Release robot brakes
        self.assertTrue(self.call_dashboard_service("brake_release", Trigger.Request()).success)

        # Wait until robot mode is RUNNING
        end_time = time.time() + 10
        mode = RobotMode.DISCONNECTED
        while mode != RobotMode.RUNNING and time.time() < end_time:
            time.sleep(0.1)
            result = self.call_dashboard_service("get_robot_mode", GetRobotMode.Request())
            self.assertTrue(result.success)
            mode = result.robot_mode.mode

        self.assertEqual(mode, RobotMode.RUNNING)

    #
    # Utility functions
    #

    def call_dashboard_service(self, srv_name, request):
        self.node.get_logger().info(
            f"Calling dashboard service '{srv_name}' with request {request}"
        )
        future = self.dashboard_clients[srv_name].call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is not None:
            self.node.get_logger().info(f"Received result {future.result()}")
            return future.result()
        else:
            raise Exception(f"Exception while calling service: {future.exception()}")


def waitForService(node, srv_name, srv_type, timeout=TIMEOUT_WAIT_SERVICE):
    client = node.create_client(srv_type, srv_name)
    if client.wait_for_service(timeout) is False:
        raise Exception(f"Could not reach service '{srv_name}' within timeout of {timeout}")

    node.get_logger().info(f"Successfully connected to service '{srv_name}'")
    return client
