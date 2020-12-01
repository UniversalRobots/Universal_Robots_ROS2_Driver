import os
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():

    # Get URDF via xacro
    robot_description_path = os.path.join(get_package_share_directory('ur_description'), 'urdf', 'ur5_robot.urdf.xacro')
    robot_description_config = xacro.process_file(robot_description_path).toxml()
    robot_description = {'robot_description': robot_description_config}

    robot_description_semantic_config = load_file('ur5_moveit_config', 'config/ur5.srdf')
    robot_description_semantic = {'robot_description_semantic' : robot_description_semantic_config}

    # We do not have a robot connected, so publish fake joint states
    joint_state_pub_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
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
        'ur_description') + "/config/view_robot.rviz"
    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     output='log',
                     arguments=['-d', rviz_config_file],
                     parameters=[robot_description, robot_description_semantic]
                     )

    return LaunchDescription([rviz_node, robot_state_pub_node, joint_state_pub_node])
