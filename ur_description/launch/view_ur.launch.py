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

    # set ur robot
    robot_name = 'ur5_e'

    # <robot_name> parameters files
    joint_limits_params = os.path.join(get_package_share_directory('ur_description'), 'config/' +
                                       robot_name.replace('_', ''), 'joint_limits.yaml')
    kinematics_params = os.path.join(get_package_share_directory('ur_description'), 'config/' +
                                     robot_name.replace('_', ''), 'default_kinematics.yaml')
    physical_params = os.path.join(get_package_share_directory('ur_description'), 'config/' +
                                   robot_name.replace('_', ''), 'physical_parameters.yaml')
    visual_params = os.path.join(get_package_share_directory('ur_description'), 'config/' +
                                 robot_name.replace('_', ''), 'visual_parameters.yaml')

    # common parameters
    # If True, enable the safety limits controller
    safety_limits = False
    # The lower/upper limits in the safety controller
    safety_pos_margin = 0.15
    # Used to set k position in the safety controller
    safety_k_position = 20

    # Get URDF via xacro
    robot_description_path = os.path.join(get_package_share_directory('ur_description'), 'urdf', 'ur.xacro')

    robot_description_config = xacro.process_file(robot_description_path,
                                                  mappings={'joint_limit_params': joint_limits_params,
                                                            'kinematics_params': kinematics_params,
                                                            'physical_params': physical_params,
                                                            'visual_params': visual_params,
                                                            'safety_limits': str(safety_limits).lower(),
                                                            'safety_pos_margin': str(safety_pos_margin),
                                                            'safety_k_position': str(safety_k_position)}
                                                  )

    robot_description = {'robot_description': robot_description_config.toxml()}

    robot_description_semantic_config = load_file(robot_name + '_moveit_config', 'config/' +
                                                  robot_name.replace('_', '') + '.srdf')
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
        'ur_description') + "/config/rviz/view_robot.rviz"
    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     output='log',
                     arguments=['-d', rviz_config_file],
                     parameters=[robot_description, robot_description_semantic]
                     )

    return LaunchDescription([rviz_node, robot_state_pub_node, joint_state_pub_node])
