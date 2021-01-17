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
                                                            'name': robot_name.replace('_', ''),
                                                            'use_ros2_control': str(use_ros2_control).lower(),
                                                            'script_filename': script_filename,
                                                            'input_recipe_filename': input_recipe_filename,
                                                            'output_recipe_filename': output_recipe_filename,
                                                            'robot_ip': '10.0.1.186'}
                                                  )

    robot_description = {'robot_description': robot_description_config.toxml()}

    robot_description_semantic_config = load_file(robot_name + '_moveit_config', 'config/' +
                                                  robot_name.replace('_', '') + '.srdf')
    robot_description_semantic = {'robot_description_semantic' : robot_description_semantic_config}

    # Get parameters for the Servo node
    servo_yaml = load_yaml('ur_ros2_control_demos', 'config/ur5_e_servo_config.yaml')
    servo_params = { 'moveit_servo' : servo_yaml }

    ur_controller = os.path.join(
        get_package_share_directory('ur_ros2_control_demos'),
        'config',
        'ur_ros2_control.yaml'
    )

    # RViz
    rviz_config_file = get_package_share_directory('ur_ros2_control_demos') + "/config/rviz/demo_rviz_config.rviz"
    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     output='log',
                     arguments=['-d', rviz_config_file],
                     parameters=[robot_description, robot_description_semantic])

    # Static TF
    static_tf = Node(package='tf2_ros',
                     executable='static_transform_publisher',
                     name='static_transform_publisher',
                     output='log',
                     arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'base_link'])

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
    
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, ur_controller],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
    )

    return LaunchDescription([ros2_control_node, rviz_node, robot_state_pub_node, container, static_tf])
