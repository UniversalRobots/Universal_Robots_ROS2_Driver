# Copyright (c) 2021 PickNik, Inc.
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
#
# Author: Lovro Ivanov

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip", description="The IP address at which the robot is reachable."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "target_filename",
            default_value="robot_calibration.yaml",
            description="The extracted calibration information "
            "will be written to this target file.",
        )
    )

    # Initialize Arguments
    robot_ip = LaunchConfiguration("robot_ip")
    output_filename = LaunchConfiguration("target_filename")

    calibration_correction = Node(
        package="ur_calibration",
        executable="calibration_correction",
        parameters=[{"robot_ip": robot_ip}, {"output_filename": output_filename}],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    nodes_to_start = [
        calibration_correction,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
