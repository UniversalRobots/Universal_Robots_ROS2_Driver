#!/usr/bin/env python
# Copyright 2024, FZI Forschungszentrum Informatik
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
import pytest
import sys
import time
import unittest

import rclpy
from rclpy.node import Node


from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare

import launch_testing
from launch_testing.actions import ReadyToTest

sys.path.append(os.path.dirname(__file__))
from test_common import (  # noqa: E402
    ControllerManagerInterface,
    _declare_launch_arguments,
    _ursim_action,
)


@pytest.mark.launch_test
@launch_testing.parametrize(
    "launch_dashboard_client",
    [("true"), ("false")],
)
def generate_test_description(launch_dashboard_client):
    ur_type = LaunchConfiguration("ur_type")
    launch_arguments = {
        "robot_ip": "192.168.56.101",
        "ur_type": ur_type,
        "launch_rviz": "false",
        "controller_spawner_timeout": str(120),
        "initial_joint_controller": "scaled_joint_trajectory_controller",
        "headless_mode": "true",
        "launch_dashboard_client": launch_dashboard_client,
        "start_joint_controller": "false",
    }

    robot_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"]
            )
        ),
        launch_arguments=launch_arguments.items(),
    )
    wait_dashboard_server = ExecuteProcess(
        cmd=[
            PathJoinSubstitution(
                [
                    FindPackagePrefix("ur_robot_driver"),
                    "bin",
                    "wait_dashboard_server.sh",
                ]
            )
        ],
        name="wait_dashboard_server",
        output="screen",
    )
    driver_starter = RegisterEventHandler(
        OnProcessExit(target_action=wait_dashboard_server, on_exit=robot_driver)
    )

    return LaunchDescription(
        _declare_launch_arguments()
        + [ReadyToTest(), wait_dashboard_server, _ursim_action(), driver_starter]
    )


class DashboardClientTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context
        rclpy.init()
        cls.node = Node("robot_driver_launch_test")
        time.sleep(1)
        cls.init_robot(cls)

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        cls.node.destroy_node()
        rclpy.shutdown()

    def init_robot(self):
        # This waits for the controller_manager to be available
        self._controller_manager_interface = ControllerManagerInterface(self.node)

    def setUp(self):
        pass

    def test_dashboard_client_exists(self, launch_dashboard_client):
        # Use the get_node_names_and_namespaces() method to get the list of nodes and their namespaces
        nodes_with_ns = self.node.get_node_names_and_namespaces()

        nodes = [namespace + name for (name, namespace) in nodes_with_ns]

        for node in nodes:
            print(node)

        # Print out the nodes and their namespaces
        logging.info("Discovered ROS nodes:")
        if launch_dashboard_client == "true":
            self.assertIn("/dashboard_client", nodes)
        else:
            self.assertNotIn("/dashboard_client", nodes)
