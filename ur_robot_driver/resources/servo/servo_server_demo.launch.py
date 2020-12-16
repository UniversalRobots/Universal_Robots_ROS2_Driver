import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import xacro

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():

    ur5_controller = os.path.join(
        get_package_share_directory('ur_robot_driver'),
        'resources',
        'ur5_system_position_only.yaml'
    )

    # Get URDF via xacro
    robot_description_path = os.path.join(
        get_package_share_directory('ur_robot_driver'),
        'resources',
        'ur5.urdf.xacro')
    robot_description_config = xacro.process_file(robot_description_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Get SRDF
    robot_description_semantic_config = load_file('ur5_moveit_config', 'config/ur5.srdf')
    robot_description_semantic = {'robot_description_semantic' : robot_description_semantic_config}

    # Get parameters for the Servo node
    servo_yaml = load_yaml('ur_robot_driver', 'resources/servo/ur5_simulated_config.yaml')
    servo_params = { 'moveit_servo' : servo_yaml }

    # RViz
    rviz_config_file = get_package_share_directory('ur_robot_driver') + "/resources/servo/demo_rviz_config.rviz"
    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     output='log',
                     arguments=['-d', rviz_config_file],
                     parameters=[robot_description, robot_description_semantic])

    # Publishes tf's for the robot
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Launch as much as possible in components
    container = ComposableNodeContainer(
            name='moveit_servo_demo_container',
            namespace='/',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='moveit_servo',
                    plugin='moveit_servo::ServoServer',
                    name='servo_server',
                    parameters=[servo_params, robot_description, robot_description_semantic],
                    extra_arguments=[{'use_intra_process_comms' : True}])
            ],
            output='screen',
    )
    
    # return LaunchDescription([ , container ])

    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[robot_description, ur5_controller],
            output={
                'stdout': 'screen',
                'stderr': 'screen',
            },
        ), rviz_node, robot_state_pub_node, container
    ])
