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

    # set ur robot
    robot_name = 'ur5e'

    # <robot_name> parameters files
    joint_limits_params = os.path.join(get_package_share_directory('ur_description'), 'config/' +
                                       robot_name, 'joint_limits.yaml')
    kinematics_params = os.path.join(get_package_share_directory('ur_description'), 'config/' +
                                     robot_name, 'default_kinematics.yaml')
    physical_params = os.path.join(get_package_share_directory('ur_description'), 'config/' +
                                   robot_name, 'physical_parameters.yaml')
    visual_params = os.path.join(get_package_share_directory('ur_description'), 'config/' +
                                 robot_name, 'visual_parameters.yaml')

    # common parameters
    # If True, enable the safety limits controller
    safety_limits = False
    # The lower/upper limits in the safety controller
    safety_pos_margin = 0.15
    # Used to set k position in the safety controller
    safety_k_position = 20

    use_ros2_control = True

    script_filename = os.path.join(
        get_package_share_directory('ur_robot_driver'),
        'resources',
        'ros_control.urscript')

    input_recipe_filename = os.path.join(
        get_package_share_directory('ur_robot_driver'),
        'resources',
        'rtde_input_recipe.txt')

    output_recipe_filename = os.path.join(
        get_package_share_directory('ur_robot_driver'),
        'resources',
        'rtde_output_recipe.txt')

    # Get URDF via xacro
    robot_description_path = os.path.join(get_package_share_directory('ur_description'), 'urdf', 'ur.xacro')

    robot_description_config = xacro.process_file(robot_description_path,
                                                  mappings={'joint_limit_params': joint_limits_params,
                                                            'kinematics_params': kinematics_params,
                                                            'physical_params': physical_params,
                                                            'visual_params': visual_params,
                                                            'safety_limits': str(safety_limits).lower(),
                                                            'safety_pos_margin': str(safety_pos_margin),
                                                            'safety_k_position': str(safety_k_position),
                                                            'use_ros2_control': str(use_ros2_control).lower(),
                                                            'script_filename': script_filename,
                                                            'input_recipe_filename': input_recipe_filename,
                                                            'output_recipe_filename': output_recipe_filename,
                                                            'robot_ip': '10.0.1.186'}
                                                  )

    robot_description = {'robot_description': robot_description_config.toxml()}

    robot_description_semantic_config = load_file(robot_name + '_moveit_config', 'config/' + robot_name + '.srdf')
    robot_description_semantic = {'robot_description_semantic' : robot_description_semantic_config}


    ur_controller = os.path.join(
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
        parameters=[robot_description, ur_controller],
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

