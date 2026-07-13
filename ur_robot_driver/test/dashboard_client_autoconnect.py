#!/usr/bin/env python
# Copyright 2026, FZI Forschungszentrum Informatik
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

import launch_testing
import pytest
import rclpy
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_testing.actions import ReadyToTest
from rclpy.node import Node
from std_srvs.srv import Trigger

sys.path.append(os.path.dirname(__file__))
from test_common import (  # noqa: E402
    TIMEOUT_WAIT_SERVICE_INITIAL,
    _call_service,
    _declare_launch_arguments,
    _ursim_action,
    _wait_for_service,
)


@pytest.mark.launch_test
def generate_test_description():
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
            "autoconnect": "false",
        }.items(),
    )

    return LaunchDescription(
        _declare_launch_arguments()
        + [ReadyToTest(), dashboard_client, _ursim_action("latest", "ur30")]
    )


class DashboardClientAutoconnectTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("dashboard_client_autoconnect_test")
        cls.connect_client = _wait_for_service(
            cls.node, "/dashboard_client/connect", Trigger, TIMEOUT_WAIT_SERVICE_INITIAL
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_manual_connect(self):
        """With autoconnect disabled, ~/connect succeeds once URSim is reachable."""
        end_time = time.time() + TIMEOUT_WAIT_SERVICE_INITIAL
        last_result = None
        while time.time() < end_time:
            last_result = _call_service(self.node, self.connect_client, Trigger.Request())
            if last_result.success:
                break
            time.sleep(1.0)
        self.assertIsNotNone(last_result)
        self.assertTrue(last_result.success)
