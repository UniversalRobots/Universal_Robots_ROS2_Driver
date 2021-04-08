# Copyright (c) 2021, PickNik, Inc.
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

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()
    description_package_path = get_package_share_directory("ur_description")
    robot_description_file = os.path.join(description_package_path, "urdf", "ur5.urdf.xacro")

    controller_package_path = get_package_share_directory("ur_robot_driver")
    robot_controller_file = os.path.join(controller_package_path, "config", "ur5_controllers.yaml")

    with open(robot_description_file, "r") as infile:
        descr = infile.read()
    robot_description = {"robot_description": descr}

    return LaunchDescription(
        [
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[robot_description, robot_controller_file],
                output={
                    "stdout": "screen",
                    "stderr": "screen",
                },
            )
        ]
    )
