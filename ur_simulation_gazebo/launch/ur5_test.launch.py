# Copyright (c) 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)
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
#
# Author: Denis Stogl

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    ExecuteProcess,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):

    # Initialize Arguments
    # ur_type = LaunchConfiguration("ur_type")
    # safety_limits = LaunchConfiguration("safety_limits")
    # safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    # safety_k_position = LaunchConfiguration("safety_k_position")
    # # General arguments
    # runtime_config_package = LaunchConfiguration("runtime_config_package")
    # controllers_file = LaunchConfiguration("controllers_file")
    # description_package = LaunchConfiguration("description_package")
    # description_file = LaunchConfiguration("description_file")
    # prefix = LaunchConfiguration("prefix")
    # start_joint_controller = LaunchConfiguration("start_joint_controller")
    # initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    # launch_rviz = LaunchConfiguration("launch_rviz")
    # world_path = LaunchConfiguration('world_path')

    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare("ur_simulation_gazebo"), "config", "ur_controllers.yaml"]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "rviz", "view_robot.rviz"]
    )

    initial_position_file = PathJoinSubstitution(
        [FindPackageShare("ur_simulation_gazebo"), "config", "initial_positions.yaml"]
    )

    joint_limits_file = PathJoinSubstitution(
        [FindPackageShare("ur_simulation_gazebo"), "config", "ur_joint_limits.yaml"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ur_description"), "urdf", "ur5.urdf.xacro"]
            ),
            # " ",
            # "safety_limits:=",
            # safety_limits,
            # " ",
            # "safety_pos_margin:=",
            # safety_pos_margin,
            # " ",
            # "safety_k_position:=",
            # safety_k_position,
            " ",
            "name:=ur",
            " ",
            "ur_type:=ur5",
            " ",
            "prefix:=''",
            " ",
            "is_sim:=true",
            " ",
            "gazebo_controllers:=",
            initial_joint_controllers,
            " ",
            "initial_positions_file:=",
            initial_position_file,
            " ",
            "joint_limit_params:=",
            joint_limits_file,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        #condition=IfCondition(launch_rviz),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # There may be other controllers of the joints, but this is the initially-started one
    initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=['joint_trajectory_controller', "-c", "/controller_manager"],
        #condition=IfCondition(start_joint_controller),
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=initial_joint_controller_spawner_started,
            on_exit=[rviz_node],
        )
    )





    # Make sure initial_joint_controller_spawner_started starts after spawn_joint_state_broadcaster
    delay_controller_spawn_callback = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[initial_joint_controller_spawner_started],
        )
    )

    # Gazebo nodes
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]
        ),
    )

    # # Gazebo server
    # gzserver = ExecuteProcess(
    #     cmd=['gzserver',
    #          '-s', 'libgazebo_ros_init.so',
    #          '-s', 'libgazebo_ros_factory.so',
    #          '--verbose'],
    #     output='screen',
    # )
    #
    # # Gazebo client
    # gzclient = ExecuteProcess(
    #     cmd=['gzclient','--verbose'],
    #     output='screen',
    #     # condition=IfCondition(LaunchConfiguration('gui')),
    # )

    # Spawn robot
    gazebo_spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_ur",
        arguments=["-entity", "ur", "-topic", "robot_description"],
        output="screen",
    )

    nodes_to_start = [
        robot_state_publisher_node,
        #joint_state_publisher_node,
        joint_state_broadcaster_spawner,
        delay_controller_spawn_callback,
        delay_rviz_after_joint_state_broadcaster_spawner,
        gazebo,
        #gzserver,
        #gzclient,
        gazebo_spawn_robot,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    # # UR specific arguments
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "ur_type",
    #         description="Type/series of used UR robot.",
    #         choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e"],
    #         default_value="ur5e",
    #     )
    # )
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "safety_limits",
    #         default_value="true",
    #         description="Enables the safety limits controller if true.",
    #     )
    # )
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "safety_pos_margin",
    #         default_value="0.15",
    #         description="The margin to lower and upper limits in the safety controller.",
    #     )
    # )
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "safety_k_position",
    #         default_value="20",
    #         description="k-position factor in the safety controller.",
    #     )
    # )
    # # General arguments
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "runtime_config_package",
    #         default_value="ur_simulation_gazebo",
    #         description='Package with the controller\'s configuration in "config" folder. \
    #     Usually the argument is not set, it enables use of a custom setup.',
    #     )
    # )
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "controllers_file",
    #         default_value="ur_controllers.yaml",
    #         description="YAML file with the controllers configuration.",
    #     )
    # )
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "description_package",
    #         default_value="ur_description",
    #         description="Description package with robot URDF/XACRO files. Usually the argument \
    #     is not set, it enables use of a custom description.",
    #     )
    # )
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "description_file",
    #         default_value="ur5.urdf.xacro",
    #         description="URDF/XACRO description file with the robot.",
    #     )
    # )
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "prefix",
    #         default_value="",
    #         description="Prefix of the joint names, useful for \
    #     multi-robot setup. If changed than also joint names in the controllers' configuration \
    #     have to be updated.",
    #     )
    # )
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "start_joint_controller",
    #         default_value="true",
    #         description="Enable headless mode for robot control",
    #     )
    # )
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "initial_joint_controller",
    #         default_value="joint_trajectory_controller",
    #         description="Robot controller to start.",
    #     )
    # )
    # declared_arguments.append(
    #     DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    # )
    #
    # declared_arguments.append(
    #     DeclareLaunchArgument('world_path', default_value='',
    #                       description='The world path, by default is empty.world')
    # )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
