# Copyright 2020 ROS2-Control Development Team (2020)
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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
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

    ld = LaunchDescription()

    # Get URDF via xacro
    robot_description_path = os.path.join(
        get_package_share_directory('ur_description'),
        'urdf',
        'ur5_robot.urdf.xacro')

    script_filename = os.path.join(
        get_package_share_directory('ur_robot_driver'),
        'resources',
        'ros_control.urscript')

    input_recipe_filename = os.path.join(
        get_package_share_directory('ur_robot_driver'),
        'resources',
        'rtde_output_recipe.txt')

    output_recipe_filename = os.path.join(
        get_package_share_directory('ur_robot_driver'),
        'resources',
        'rtde_input_recipe.txt')

    use_ros2_control = True

    robot_description_config = xacro.process_file(robot_description_path,
                                                  mappings={'use_ros2_control': 'true' if use_ros2_control else 'false',
                                                            'script_filename': script_filename,
                                                            'input_recipe_filename': input_recipe_filename,
                                                            'output_recipe_filename': output_recipe_filename,
                                                            'robot_ip': '10.0.1.186'}
                                                  )

    robot_description = {'robot_description': robot_description_config.toxml()}

    robot_description_semantic_config = load_file('ur5_moveit_config', 'config/ur5.srdf')
    robot_description_semantic = {'robot_description_semantic' : robot_description_semantic_config}

    ur5_controller = os.path.join(
        get_package_share_directory('ur_ros2_control_demos'),
        'config',
        'ur5_system_position_only.yaml'
        )
    # Publishes tf's for the robot
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # RViz
    rviz_config_file = get_package_share_directory(
        'ur_ros2_control_demos') + "/config/rviz/config.rviz"
    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     output='log',
                     arguments=['-d', rviz_config_file],
                     parameters=[robot_description, robot_description_semantic]
                     )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, ur5_controller],
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
            {"robot_ip": "10.0.1.186"}
        ]
    )

    return LaunchDescription([ros2_control_node, dashboard_client_node, rviz_node, robot_state_pub_node])

