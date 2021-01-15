import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

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

    # planning_context
    # set ur robot
    robot_name = 'ur5'

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

    robot_description_semantic_config = load_file('ur5_moveit_config', 'config/ur5.srdf')
    robot_description_semantic = {'robot_description_semantic' : robot_description_semantic_config}

    kinematics_yaml = load_yaml('ur5_moveit_config', 'config/kinematics.yaml')
    robot_description_kinematics = { 'robot_description_kinematics' : kinematics_yaml }

    # Planning Functionality
    ompl_planning_pipeline_config = { 'move_group' : {
        'planning_plugin' : 'ompl_interface/OMPLPlanner',
        'request_adapters' : """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""" ,
        'start_state_max_bounds_error' : 0.1 } }
    ompl_planning_yaml = load_yaml('ur5_moveit_config', 'config/ompl_planning.yaml')
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    # Trajectory Execution Functionality
    controllers_yaml = load_yaml('run_move_group', 'config/controllers.yaml')
    moveit_controllers = { 'moveit_simple_controller_manager' : controllers_yaml,
                           'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'}

    trajectory_execution = {'moveit_manage_controllers': True,
                            'trajectory_execution.allowed_execution_duration_scaling': 1.2,
                            'trajectory_execution.allowed_goal_duration_margin': 0.5,
                            'trajectory_execution.allowed_start_tolerance': 0.01}

    planning_scene_monitor_parameters = {"publish_planning_scene": True,
                 "publish_geometry_updates": True,
                 "publish_state_updates": True,
                 "publish_transforms_updates": True}

    # Start the actual move_group node/action server
    run_move_group_node = Node(package='moveit_ros_move_group',
                               executable='move_group',
                               output='screen',
                               parameters=[robot_description,
                                           robot_description_semantic,
                                           kinematics_yaml,
                                           ompl_planning_pipeline_config,
                                           trajectory_execution,
                                           moveit_controllers,
                                           planning_scene_monitor_parameters])

    # RViz
    rviz_config_file = get_package_share_directory('ur_description') + "/cfg/view_robot.rviz"
    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     output='log',
                     arguments=['-d', rviz_config_file],
                     parameters=[robot_description,
                                 robot_description_semantic,
                                 ompl_planning_pipeline_config,
                                 kinematics_yaml])

    # Static TF
    static_tf = Node(package='tf2_ros',
                     executable='static_transform_publisher',
                     name='static_transform_publisher',
                     output='log',
                     arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'base_link'])

    # Publish TF
    robot_state_publisher = Node(package='robot_state_publisher',
                                 executable='robot_state_publisher',
                                 name='robot_state_publisher',
                                 output='both',
                                 parameters=[robot_description])

# TODO(andyz): fake_joint_driver conflicts with the current ros2_controller commits
#    # Fake joint driver
#    fake_joint_driver_node = Node(package='fake_joint_driver',
#                                  executable='fake_joint_driver_node',
#                                  parameters=[{'controller_name': 'panda_arm_controller'},
#                                              os.path.join(get_package_share_directory("run_move_group"), "config", "panda_controllers.yaml"),
#                                              os.path.join(get_package_share_directory("run_move_group"), "config", "start_positions.yaml"),
#                                              robot_description]
#                                  )

    return LaunchDescription([ rviz_node, static_tf, robot_state_publisher, run_move_group_node ])
