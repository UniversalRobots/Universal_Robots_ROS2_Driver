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

import pytest
import unittest

from launch import LaunchDescription
import launch_testing
from launch_testing.actions import ReadyToTest
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare
from launch.event_handlers import OnProcessExit

TIMEOUT_WAIT_SERVICE_INITIAL = 120  # If we download the docker image simultaneously to the tests, it can take quite some time until the dashboard server is reachable and usable.


@pytest.mark.launch_test
def generate_test_description():
    controller_spawner_timeout = TIMEOUT_WAIT_SERVICE_INITIAL
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5e",
            description="Type/series of used UR robot.",
            choices=[
                "ur3",
                "ur3e",
                "ur5",
                "ur5e",
                "ur10",
                "ur10e",
                "ur16e",
                "ur20",
                "ur30",
            ],
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
            "controller_spawner_timeout": str(controller_spawner_timeout),
            "initial_joint_controller": "scaled_joint_trajectory_controller",
            "headless_mode": "true",
            "launch_dashboard_client": "false",
            "start_joint_controller": "false",
        }.items(),
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
    moveit_setup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_moveit_config"), "launch", "ur_moveit.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": ur_type,
            "launch_rviz": "false",
            "launch_servo": "false",  # the servo node currently doesn't startup correctly,
            # it waits for the robot state until the robot has actually
            # executed a plan. Therefore, it doesn't exit cleanly which
            # breaks this test.
        }.items(),
    )

    return LaunchDescription(
        declared_arguments
        + [ReadyToTest(), wait_dashboard_server, _ursim_action(), driver_starter, moveit_setup]
    )


def _ursim_action():
    ur_type = LaunchConfiguration("ur_type")

    return ExecuteProcess(
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


class TestReadyForPlanning(unittest.TestCase):
    def test_read_stdout(self, proc_output):
        """Check if 'You can start planning now!' was found in the stdout."""
        proc_output.assertWaitFor("You can start planning now!", timeout=120, stream="stdout")
        proc_output.assertWaitFor(
            "Dashboard server connections are possible.", timeout=120, stream="stdout"
        )


@launch_testing.post_shutdown_test()
class TestProcessExitCode(unittest.TestCase):
    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
