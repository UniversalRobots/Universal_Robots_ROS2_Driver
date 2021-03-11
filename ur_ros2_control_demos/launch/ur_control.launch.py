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

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

import xacro


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument(
        'ur_type',
        description='Type/series of used UR robot.'))
    declared_arguments.append(DeclareLaunchArgument(
        'robot_ip',
        description='IP address by which the robot can be reached.'))
    declared_arguments.append(DeclareLaunchArgument(
        'safety_limits', default_value='true',
        description='Enables the safety limits controller if true.'))
    declared_arguments.append(DeclareLaunchArgument(
        'safety_pos_margin', default_value='0.15',
        description='The margin to lower and upper limits in the safety controller.'))
    declared_arguments.append(DeclareLaunchArgument(
        'safety_k_position', default_value='20',
        description='k-position factor in the safety controller.'))
    declared_arguments.append(DeclareLaunchArgument(
        'use_fake_hardware', default_value='false',
        description='Start robot with fake hardware mirroring command to its states.'))
    declared_arguments.append(DeclareLaunchArgument(
        'fake_sensor_commands', default_value='false',
        description='Enable fake command interfaces for sensors used for simple simulations. \
            Used only if \'use_fake_hardware\' parameter is true.'))

    ur_type = LaunchConfiguration('ur_type')
    robot_ip = LaunchConfiguration('robot_ip')
    safety_limits = LaunchConfiguration('safety_limits')
    safety_pos_margin = LaunchConfiguration('safety_pos_margin')
    safety_k_position = LaunchConfiguration('safety_k_position')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    use_ros2_control = 'true'
    fake_sensor_commands = LaunchConfiguration('fake_sensor_commands')

    joint_limit_params = PathJoinSubstitution([
        get_package_share_directory('ur_description'), 'config', ur_type, 'joint_limits.yaml'])
    kinematics_params = PathJoinSubstitution([
        get_package_share_directory('ur_description'), 'config', ur_type,
        'default_kinematics.yaml'])
    physical_params = PathJoinSubstitution([
        get_package_share_directory('ur_description'), 'config', ur_type,
        'physical_parameters.yaml'])
    visual_params = PathJoinSubstitution([
        get_package_share_directory('ur_description'), 'config', ur_type, 'visual_parameters.yaml'])
    script_filename = PathJoinSubstitution([
        get_package_share_directory('ur_robot_driver'), 'resources', 'ros_control.urscript'])
    input_recipe_filename = PathJoinSubstitution([
        get_package_share_directory('ur_robot_driver'), 'resources', 'rtde_input_recipe.txt'])
    output_recipe_filename = PathJoinSubstitution([
        get_package_share_directory('ur_robot_driver'), 'resources', 'rtde_output_recipe.txt'])

    robot_description_content = Command([
        PathJoinSubstitution([get_package_prefix('xacro'), 'bin', 'xacro']),
        ' ',
        PathJoinSubstitution([get_package_share_directory('ur_description'), 'urdf', 'ur.xacro']),
        ' ',
        'robot_ip:=', robot_ip, ' ',
        'joint_limit_params:=', joint_limit_params, ' ',
        'kinematics_params:=', kinematics_params, ' ',
        'physical_params:=', physical_params, ' ',
        'visual_params:=', visual_params, ' ',
        'safety_limits:=', safety_limits, ' ',
        'safety_pos_margin:=', safety_pos_margin, ' ',
        'safety_k_position:=', safety_k_position, ' ',
        'name:=', ur_type, ' ',
        'use_ros2_control:=', use_ros2_control, ' ',
        'script_filename:=', script_filename, ' ',
        'input_recipe_filename:=', input_recipe_filename, ' ',
        'output_recipe_filename:=', output_recipe_filename, ' ',
        ])
    robot_description = {'robot_description': robot_description_content}

    ur_controllers = PathJoinSubstitution([
      get_package_share_directory('ur_ros2_control_demos'), 'config', 'ur_controllers.yaml'])

    # Publishes tf's for the robot
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # RViz
    rviz_config_file = PathJoinSubstitution([
        get_package_share_directory('ur_ros2_control_demos'), 'config', 'rviz', 'config.rviz'])
    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     output='log',
                     arguments=['-d', rviz_config_file],
                     parameters=[robot_description]
                     # parameters=[robot_description, robot_description_semantic]
                     )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, ur_controllers],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
    )

    dashboard_client_node = Node(
        package="ur_robot_driver",
        executable="dashboard_client",
        name="dashboard_client",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"robot_ip": robot_ip}
        ]
    )

    return LaunchDescription(
        declared_arguments,
        [
            robot_state_pub_node,
            rviz_node,
            ros2_control_node,
            dashboard_client_node
        ])
