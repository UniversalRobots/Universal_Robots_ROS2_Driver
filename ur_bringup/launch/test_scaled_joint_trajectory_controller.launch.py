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
# Author: Denis Stogl
#
# Description: After a robot has been loaded, this will execute a series of trajectories.
import sys

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError


def generate_launch_description():

    position_goals = PathJoinSubstitution(
        [FindPackageShare("ur_bringup"), "config", "test_goal_publishers_config.yaml"]
    )

    # Kind of hack until this dependency is released
    # TODO (@anyone): Remove this once the upstream package is released
    try:
        get_package_share_directory("ros2_control_test_nodes")
    except PackageNotFoundError:
        print(
            "ERROR:"
            "Could not find package 'ros2_control_test_nodes'. Please install it (build it from "
            "source) in order to run this launchfile. See here for details: explanation:\n"
            "https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/foxy"
            "#example-commands-for-testing-the-driver"
        )
        sys.exit(1)

    return LaunchDescription(
        [
            Node(
                package="ros2_control_test_nodes",
                executable="publisher_joint_trajectory_position_controller",
                name="publisher_scaled_joint_trajectory_controller",
                parameters=[position_goals],
                output={"stdout": "screen", "stderr": "screen"},
            )
        ]
    )
